//! dot phase-3 coordinate assignment — a faithful port of
//! `lib/dotgen/position.c` (Graphviz main @ `2e92c7f`, 1159 lines).
//!
//! [`dot_position`] sets `ND_coord` for every node of `g` using the rank
//! arrays populated by mincross. Y coordinates are computed rank-by-rank in
//! [`set_ycoords`]; X coordinates come from an auxiliary constraint graph
//! ranked by network simplex (`rank(g, 2, nsiter2(g))`, position.c:141) via
//! [`ns`][super::ns], then copied into `ND_coord.x` by [`set_xcoords`].
//!
//! Pipeline (position.c:121-153):
//! `mark_lowclusters` → `set_ycoords` → [`dot_concentrate`] →
//! `expand_leaves` → [`flat_edges`][super::flat::flat_edges] [→
//! `set_ycoords`] → `create_aux_edges` (`allocate_aux_edges` →
//! `make_LR_constraints` → `make_edge_pairs` → `pos_clusters` →
//! `compress_graph`) → `rank(g,2,…)` [→ `connectGraph` → `rank` again] →
//! `set_xcoords` → `set_aspect` → `remove_aux_edges`.
//!
//! Port notes on C state with no arena counterpart:
//!
//! * **`rank_t.pht1/pht2`** (position.c:781-784) track the *primitive* node
//!   half-heights; `ht1/ht2` start equal but are raised by cluster nesting and
//!   cluster labels in `clust_ht`. `set_ycoords` needs both (`d0` uses `pht`,
//!   `d1` uses `ht`), so the arena keeps them as separate [`Rank`] fields —
//!   assuming `pht ≡ ht` (as an earlier revision of this port did) inflates
//!   every rank gap by the cluster label band.
//! * **`GD_ln`/`GD_rn`** (cluster bbox vnodes) are used only inside
//!   position.c, so they live in [`PosState::ln_rn`].
//! * **`GD_drawing`** (ratio/size/page/margin) is modeled by [`Drawing`] in
//!   [`PosState`]; the default is `R_NONE`, i.e. `set_aspect` only computes
//!   bounding boxes.
//! * **`ND_alg`** and **`ED_dist`** — see `flat.rs` (`nd_alg` / `edge_dist`).
//! * The graph `margin` attribute (`G_margin`, `late_int` at utils.c:40) has
//!   no storage on `DGraph`; it defaults to `CL_OFFSET` and can be supplied
//!   per graph through [`PosState::set_margin`].
//!
//! Rank-array convention: `DGraph.rank` is stored dense from `DGraph.minrank`
//! (root `minrank` is always 0 in the dot1 pipeline); `abomination` inserts
//! an empty rank at index 0 and decrements `minrank`, mirroring the C
//! pointer rebase of flat.c:193. All row lookups go through [`rank_row`] /
//! [`rank_row_mut`].

use std::collections::HashMap;

use super::classes::{fast_edge, find_fast_edge, ports_eq, virtual_node, zapinlist};
use super::flat;
use super::geom::{self, BoxF, PointF};
use super::model::{CL_OFFSET, DEdge, EId, Fg, GId, NId, NodeType, Rank, SLACKNODE};
use super::ns;

/// `EDGE_LABEL` bit of `GD_has_labels` (const.h:167).
const EDGE_LABEL: u8 = 1;
/// `SELF_EDGE_SIZE` (const.h:98).
const SELF_EDGE_SIZE: f64 = 18.0;
/// Port side flags (const.h:111-120; `BOTTOM_IX`…`LEFT_IX`).
const SIDE_BOTTOM: u8 = 1 << 0;
const SIDE_TOP: u8 = 1 << 2;
const SIDE_LEFT: u8 = 1 << 3;
/// `USHRT_MAX` cap on the `ratio=compress` width pin (position.c:539).
const USHRT_MAX: f64 = 65535.0;

/// `ratio_t` (types.h:215-216) — the `ratio` attribute kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RatioKind {
    /// `R_NONE` (default).
    #[default]
    None,
    /// `R_VALUE` — numeric ratio.
    Value,
    /// `R_FILL`.
    Fill,
    /// `R_COMPRESS` — handled by `compress_graph` (position.c:519-541).
    Compress,
    /// `R_AUTO`.
    Auto,
    /// `R_EXPAND`.
    Expand,
}

/// `layout_t` subset read by position.c (`GD_drawing`, types.h:218-228).
#[derive(Debug, Clone, Default)]
pub struct Drawing {
    pub ratio_kind: RatioKind,
    /// `GD_drawing->size` (points).
    pub size: PointF,
    /// `GD_drawing->page` (points).
    pub page: PointF,
    /// `GD_drawing->margin` (points).
    pub margin: PointF,
    /// `GD_drawing->ratio`.
    pub ratio: f64,
    /// `GD_drawing->quantum` — applied to node dimensions upstream
    /// (shapes.c:2013-2019), never to coordinates in position (§12).
    #[allow(dead_code)]
    pub quantum: f64,
}

/// Per-run state of [`dot_position`] for the C globals/fields that have no
/// arena home (see the module notes).
#[derive(Debug, Default)]
pub struct PosState {
    /// `GD_ln`/`GD_rn` per cluster (position.c:1078-1096).
    ln_rn: HashMap<GId, (NId, NId)>,
    /// Graph `margin` attribute per graph id; absent → `CL_OFFSET`.
    margin: HashMap<GId, i32>,
    /// `GD_drawing` of the root graph.
    pub drawing: Drawing,
}

/// `late_int(g, G_margin, CL_OFFSET, 0)` (utils.c:40-52).
fn gmargin(state: &PosState, g: GId) -> i32 {
    state.margin.get(&g).copied().unwrap_or(CL_OFFSET as i32)
}

/// Maps an absolute rank index to the `Vec` slot of `DGraph.rank`.
///
/// `allocate_ranks` fills rows by *absolute* rank (`mincross::ri`), which is
/// what a cluster's rows use too; the one exception is `flat.c`'s
/// `abomination`, which inserts an empty rank at index 0 and decrements
/// `minrank` to -1, shifting every later row up by one slot. Both cases are
/// the same expression: the slot is `r - min(minrank, 0)`.
pub(crate) fn rank_row_index(fg: &Fg, g: GId, r: i32) -> usize {
    let base = fg.graphs[g].minrank.min(0);
    (r - base) as usize
}

/// [`rank_row_index`] applied to `DGraph.rank`.
pub(crate) fn rank_row(fg: &Fg, g: GId, r: i32) -> &Rank {
    &fg.graphs[g].rank[rank_row_index(fg, g, r)]
}

/// Mutable variant of [`rank_row`].
pub(crate) fn rank_row_mut(fg: &mut Fg, g: GId, r: i32) -> &mut Rank {
    let i = rank_row_index(fg, g, r);
    &mut fg.graphs[g].rank[i]
}

/// The live nodes of a rank row — `GD_rank(g)[r].v[0..n]`. C's loops stop at
/// the trailing NULL sentinel; rows can carry one here too (a cluster
/// expansion opens and fills slots, and `make_slots` keeps the terminator), so
/// every iteration must use the counted prefix rather than the whole `Vec`.
pub(crate) fn rank_nodes(fg: &Fg, g: GId, r: i32) -> Vec<NId> {
    let row = rank_row(fg, g, r);
    row.v[..row.n.min(row.v.len())].to_vec()
}

/// The fast node list `GD_nlist(g)` as a Vec in list order.
fn nlist_nodes(fg: &Fg, g: GId) -> Vec<NId> {
    let mut out = Vec::new();
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        out.push(id);
        n = fg.nodes[id].next;
    }
    out
}

/// `dot_position` (position.c:121-153) — phase 3 of dot layout.
///
/// Returns non-zero only from `dot_concentrate` (not ported — Concentrate is
/// false in this milestone) or `create_aux_edges` (`-1` when `make_aux_edge`
/// hits the `INT_MAX` length cap, position.c:192-199).
pub fn dot_position(fg: &mut Fg, g: GId) -> Result<(), i32> {
    if fg.graphs[g].nlist.is_none() {
        return Ok(()); // ignore empty graph (position.c:122-123)
    }
    let mut state = PosState::default();
    let timing = std::env::var_os("GD_TIMING").is_some();
    let mut t = std::time::Instant::now();
    let mut lap = |fg: &Fg, name: &str| {
        if timing {
            eprintln!(
                "[timing]   position::{name}: {:.3}s",
                t.elapsed().as_secs_f64()
            );
            t = std::time::Instant::now();
        }
        let _ = fg;
    };
    mark_lowclusters(fg, g); // position.c:124 (cluster.c:399-444)
    set_ycoords(fg, g, &state); // position.c:125
    lap(fg, "set_ycoords");
    if fg.concentrate {
        // position.c:126-131 — merge parallel edge chains onto one
        // concentrator before the x-coordinates are assigned.
        if super::conc::dot_concentrate(fg, g).is_err() {
            return Err(1);
        }
    }
    lap(fg, "mark_lowclusters");
    expand_leaves(fg, g); // position.c:132
    lap(fg, "expand_leaves");
    if flat::flat_edges(fg, g) {
        // redo y if label vnodes were added (position.c:133-134)
        set_ycoords(fg, g, &state);
    }
    lap(fg, "flat_edges");
    create_aux_edges(fg, g, &mut state)?; // position.c:135-140
    lap(fg, "create_aux_edges");
    let maxiter = nsiter2(fg, g); // position.c:141 (nsiter2, L155-163)
    let search_size = fg.graphs[g].search_size; // ns.c rank(): "searchsize" attr
    // rank(g, 2, nsiter2(g)) — LR balance == 2 (position.c:141-146). Returns
    // non-zero iff the aux graph is not connected.
    if ns::rank2(fg, nlist_nodes(fg, g), 2, maxiter, search_size, None).is_err() {
        connect_graph(fg, g); // position.c:142
        lap(fg, "rank2#1 + connect_graph");
        // assert(rank_result == 0) in C — release builds proceed with
        // whatever ranks resulted, so the second result is ignored here too.
        let _ = ns::rank2(fg, nlist_nodes(fg, g), 2, maxiter, search_size, None);
    }
    lap(fg, "rank2");
    set_xcoords(fg, g); // position.c:147
    set_aspect(fg, g, &mut state); // position.c:148
    remove_aux_edges(fg, g); // position.c:149-151 — must come after set_aspect
    lap(fg, "set_xcoords+aspect");
    Ok(())
}

/// `nsiter2` (position.c:155-163) — `INT_MAX`, or `scale_clamp(agnnodes, nslimit)`.
fn nsiter2(fg: &Fg, g: GId) -> i32 {
    match fg.graphs[g].nslimit {
        Some(s) => super::scale_clamp(fg.graphs[g].nodes_order.len(), s),
        None => i32::MAX,
    }
}

/// `connectGraph` (position.c:72-119) — link rank-first nodes with 0-weight
/// slacknode edges after a failed `rank()`. NB: `ND_rank` holds simplex x
/// values here, and the comparisons against `r` are made in those terms
/// exactly as written.
fn connect_graph(fg: &mut Fg, g: GId) {
    for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
        let row_v = rank_nodes(fg, g, r);
        let mut found = false;
        let mut tp: Option<NId> = None;
        for i in 0..row_v.len() {
            let v = row_v[i];
            tp = Some(v);
            for &e in &fg.nodes[v].save_out {
                // position.c:90-94
                if fg.nodes[fg.edges[e].head].rank > r || fg.nodes[fg.edges[e].tail].rank > r {
                    found = true;
                    break;
                }
            }
            if found {
                break;
            }
            for &e in &fg.nodes[v].save_in {
                // position.c:99-103
                if fg.nodes[fg.edges[e].tail].rank > r || fg.nodes[fg.edges[e].head].rank > r {
                    found = true;
                    break;
                }
            }
            if found {
                break;
            }
        }
        if found || tp.is_none() {
            continue; // position.c:108
        }
        let tp = row_v[0]; // position.c:109
        let hp = if r < fg.graphs[g].maxrank {
            rank_row(fg, g, r + 1).v[0] // position.c:110
        } else {
            rank_row(fg, g, r - 1).v[0] // position.c:111
        };
        let sn = virtual_node(fg, g); // position.c:113
        fg.nodes[sn].node_type = SLACKNODE; // position.c:114
        let _ = make_aux_edge(fg, sn, tp, 0.0, 0); // position.c:115
        let _ = make_aux_edge(fg, sn, hp, 0.0, 0); // position.c:116
        let (rt, rh) = (fg.nodes[tp].rank, fg.nodes[hp].rank);
        fg.nodes[sn].rank = rt.min(rh); // position.c:117
    }
}

/// `go` (position.c:165-176) — plain DFS reachability over the *current*
/// `ND_out` (aux edges only during `make_LR_constraints`). No visited set,
/// like the C; termination relies on the aux graph being acyclic at the
/// call sites (the `canreach` guards exist to keep it that way).
fn go(fg: &Fg, u: NId, v: NId) -> bool {
    if u == v {
        return true;
    }
    for &e in &fg.nodes[u].out {
        if go(fg, fg.edges[e].head, v) {
            return true;
        }
    }
    false
}

/// `canreach` (position.c:178-180).
fn canreach(fg: &Fg, u: NId, v: NId) -> bool {
    go(fg, u, v)
}

/// `make_aux_edge` (position.c:182-204) — the only aux-edge factory:
/// `ED_minlen = ROUND(len)`, `ED_weight = wt`, appended to both fast lists.
/// Returns `None` when `len > INT_MAX` (position.c:192-199).
fn make_aux_edge(fg: &mut Fg, u: NId, v: NId, len: f64, wt: i32) -> Option<EId> {
    if len > i32::MAX as f64 {
        // agerrorf (position.c:193-195)
        eprintln!(
            "Error: Edge length {len} larger than maximum {} allowed.\nCheck for overwide node(s).",
            i32::MAX
        );
        return None;
    }
    let seq = fg.seq;
    fg.seq += 1;
    fg.edges.push(DEdge {
        tail: u,
        head: v,
        minlen: geom::round_i(len), // position.c:200
        weight: wt,                 // position.c:201
        seq,
        ..Default::default()
    });
    let e = fg.edges.len() - 1;
    Some(fast_edge(fg, e)) // position.c:202
}

/// `selfRightSpace` (lib/common/splines.c:1137-1157) — extra right-side
/// space of a self edge: `SELF_EDGE_SIZE` plus the label width (flipped for
/// LR) unless ports place it elsewhere.
fn self_right_space(fg: &Fg, e: EId) -> f64 {
    let tp = &fg.edges[e].tail_port;
    let hp = &fg.edges[e].head_port;
    let on_left = tp.side & SIDE_LEFT != 0 || hp.side & SIDE_LEFT != 0;
    let cond = (!tp.defined && !hp.defined)
        || (!on_left && (tp.side != hp.side || (tp.side & (SIDE_TOP | SIDE_BOTTOM)) == 0));
    if cond {
        let mut sw = SELF_EDGE_SIZE;
        if let Some(l) = fg.edges[e].label {
            let d = fg.labels[l].dimen;
            sw += if fg.graphs[fg.root_g()].rankdir.flip() {
                d.y
            } else {
                d.x
            };
        }
        sw
    } else {
        0.0
    }
}

/// `allocate_aux_edges` (position.c:206-221) — snapshot the fast in/out
/// lists into `ND_save_in`/`ND_save_out` and wipe the originals, so
/// afterwards `ND_in`/`ND_out` contain only auxiliary edges. (The C
/// `alloc_elist(n, …)` pre-allocates slots; the `Vec` grows on demand, which
/// is semantically identical.)
fn allocate_aux_edges(fg: &mut Fg, g: GId) {
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        let save_in = std::mem::take(&mut fg.nodes[id].in_); // position.c:213
        let save_out = std::mem::take(&mut fg.nodes[id].out); // position.c:214
        fg.nodes[id].save_in = save_in;
        fg.nodes[id].save_out = save_out;
        fg.nodes[id].in_ = Vec::new(); // position.c:218 — alloc_elist, wiped
        fg.nodes[id].out = Vec::new(); // position.c:219 — alloc_elist, wiped
    }
}

/// `make_LR_constraints` (position.c:224-336) — the x-coordination aux
/// edges: 0-weight ordering edges between rank neighbors, flat-edge
/// constraints (label vnodes + endpoint spacing) and the self-edge width
/// inflation. Returns `-1` when an aux edge length overflows.
fn make_lr_constraints(fg: &mut Fg, g: GId) -> Result<(), i32> {
    let nodesep_g = fg.graphs[g].nodesep;
    let flip = fg.graphs[g].rankdir.flip();
    // position.c:235-241 — smaller separation on odd ranks when the root
    // has edge labels
    let sep: [i32; 2] = if fg.graphs[fg.root_g()].has_labels & EDGE_LABEL != 0 {
        [nodesep_g, 5]
    } else {
        [nodesep_g, nodesep_g]
    };
    let minr = fg.graphs[g].minrank;
    let maxr = fg.graphs[g].maxrank;
    for i in minr..=maxr {
        // position.c:244 — leftmost node seeds x = 0 (ND_rank doubles as the
        // x variable during this phase)
        // Ranks that only exist as cluster scaffolding can be empty before
        // the cluster expansion lands; C assumes a node is present.
        let Some(&leftmost) = rank_row(fg, g, i).v.first() else {
            continue;
        };
        fg.nodes[leftmost].rank = 0;
        let mut last: f64 = 0.0;
        let nodesep = sep[(i & 1) as usize]; // position.c:245
        let row_v = rank_nodes(fg, g, i); // prefix: `get(j+1)` peeks past the end
        for (j, &u) in row_v.iter().enumerate() {
            fg.nodes[u].mval = fg.nodes[u].rw; // position.c:248 — keep it safe
            if !fg.nodes[u].other.is_empty() {
                // position.c:249-265 — self-edge width inflation (persists
                // for the whole phase; undone later by resetRW in dotsplines)
                let mut sw = 0.0;
                for &e in &fg.nodes[u].other {
                    if fg.edges[e].tail == fg.edges[e].head {
                        sw += self_right_space(fg, e);
                    }
                }
                fg.nodes[u].rw += sw; // position.c:264
            }
            if let Some(&v) = row_v.get(j + 1) {
                // position.c:266-274 — hard ordering constraint
                let width = fg.nodes[u].rw + fg.nodes[v].lw + nodesep as f64;
                if make_aux_edge(fg, u, v, width, 0).is_none() {
                    return Err(-1);
                }
                // position.c:273 — int truncation, and `last` continues from
                // the truncated value
                let nl = (last + width) as i32;
                fg.nodes[v].rank = nl;
                last = nl as f64;
            }

            // position.c:277-296 — constraints from the label of a flat edge
            // on the previous rank (u is the label vnode)
            if let Some(e) = flat::nd_alg(fg, u) {
                let e0 = fg.nodes[u].save_out[0];
                let e1 = fg.nodes[u].save_out[1];
                // position.c:280-282 — e0 = the LEFT flat endpoint
                let (e0, e1) =
                    if fg.nodes[fg.edges[e0].head].order > fg.nodes[fg.edges[e1].head].order {
                        (e1, e0)
                    } else {
                        (e0, e1)
                    };
                let m0 = fg.edges[e].minlen * nodesep_g / 2; // position.c:283 — int div
                let (t0, h0) = (fg.edges[e0].tail, fg.edges[e0].head);
                let m1 = m0 as f64 + fg.nodes[h0].rw + fg.nodes[t0].lw; // position.c:284
                // position.c:285-290 — guards because "flat edges work very
                // poorly with cluster layout"
                if !canreach(fg, t0, h0)
                    && make_aux_edge(fg, h0, t0, m1, fg.edges[e].weight).is_none() {
                        return Err(-1);
                    }
                let (t1, h1) = (fg.edges[e1].tail, fg.edges[e1].head);
                let m1 = m0 as f64 + fg.nodes[t1].rw + fg.nodes[h1].lw; // position.c:291
                if !canreach(fg, h1, t1)
                    && make_aux_edge(fg, t1, h1, m1, fg.edges[e].weight).is_none() {
                        return Err(-1);
                    }
            }

            // position.c:299-332 — position flat edge endpoints
            for &e in fg.nodes[u].flat_out.clone().iter() {
                let (t0, h0) = {
                    let (et, eh) = (fg.edges[e].tail, fg.edges[e].head);
                    if fg.nodes[et].order < fg.nodes[eh].order {
                        (et, eh)
                    } else {
                        (eh, et)
                    }
                };
                let width = fg.nodes[t0].rw + fg.nodes[h0].lw; // position.c:309
                // position.c:310 — int m0, truncating conversion
                let mut m0 = ((fg.edges[e].minlen * nodesep_g) as f64 + width) as i32;
                if let Some(e0) = find_fast_edge(fg, t0, h0) {
                    // position.c:312-319 — flat edge between adjacent
                    // neighbors: strengthen the ordering edge found above
                    let dist = flat::edge_dist(fg, e, flip); // ED_dist(e)
                    let boosted = width + nodesep_g as f64 + geom::round_i(dist) as f64;
                    m0 = (m0 as f64).max(boosted) as i32; // position.c:316 — trunc
                    fg.edges[e0].minlen = fg.edges[e0].minlen.max(m0); // position.c:317
                    fg.edges[e0].weight = fg.edges[e0].weight.max(fg.edges[e].weight); // L318
                } else if fg.edges[e].label.is_none() {
                    // position.c:320-328 — unlabeled flat edge between
                    // non-neighbors (labeled ones are constrained via the
                    // label vnode above)
                    if make_aux_edge(fg, t0, h0, m0 as f64, fg.edges[e].weight).is_none() {
                        return Err(-1);
                    }
                }
            }
        }
    }
    Ok(())
}

/// `make_edge_pairs` (position.c:341-370) — soft "keep endpoints ordered"
/// pairs: one slacknode per original out edge, joined to both endpoints with
/// the edge's own weight.
fn make_edge_pairs(fg: &mut Fg, g: GId) -> Result<(), i32> {
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        // position.c:349 — fresh slacknodes are prepended; following the
        // current node's ND_next never revisits them
        n = fg.nodes[id].next;
        if fg.nodes[id].save_out.is_empty() {
            continue; // position.c:347
        }
        for i in 0..fg.nodes[id].save_out.len() {
            let e = fg.nodes[id].save_out[i];
            let sn = virtual_node(fg, g);
            fg.nodes[sn].node_type = SLACKNODE; // position.c:350
            // position.c:351 — double → int truncation
            let raw = (fg.edges[e].head_port.p.x - fg.edges[e].tail_port.p.x) as i32;
            let (m0, m1) = if raw > 0 { (raw, 0) } else { (0, -raw) }; // L352-357
            let t = fg.edges[e].tail;
            let h = fg.edges[e].head;
            let wt = fg.edges[e].weight;
            if make_aux_edge(fg, sn, t, (m0 + 1) as f64, wt).is_none() {
                return Err(-1); // position.c:358-360
            }
            if make_aux_edge(fg, sn, h, (m1 + 1) as f64, wt).is_none() {
                return Err(-1); // position.c:361-363
            }
            // position.c:364-366 — tentative x
            let rt = fg.nodes[t].rank - m0 - 1;
            let rh = fg.nodes[h].rank - m1 - 1;
            fg.nodes[sn].rank = rt.min(rh);
        }
    }
    Ok(())
}

/// `agcontains(g, n)` — cgraph subgraph membership of a (real) node in g's
/// closure (used by `vnode_not_related_to`, position.c:394-397).
fn agcontains(fg: &Fg, g: GId, n: NId) -> bool {
    if g == fg.root_g() {
        return true;
    }
    if fg.graphs[g].nodes_order.contains(&n) {
        return true;
    }
    fg.graphs[g].clust.iter().any(|&c| agcontains(fg, c, n))
}

/// `contain_clustnodes` (position.c:372-386) — recurse children first, then
/// tie each cluster's own `ln`→`rn` with the weight-128 compaction edge.
fn contain_clustnodes(fg: &mut Fg, g: GId, state: &mut PosState) {
    let root = fg.root_g();
    if g != root {
        contain_nodes(fg, g, state); // position.c:378
        let (ln, rn) = state.ln_rn[&g];
        if let Some(e) = find_fast_edge(fg, ln, rn) {
            // position.c:379-380 — maybe from make_lrvn's label-width edge
            fg.edges[e].weight += 128;
        } else {
            // position.c:382 — clust compaction edge
            let _ = make_aux_edge(fg, ln, rn, 1.0, 128);
        }
    }
    for c in fg.graphs[g].clust.clone() {
        // position.c:384-385
        contain_clustnodes(fg, c, state);
    }
}

/// `vnode_not_related_to` (position.c:388-399) — `v` is a vnode whose
/// original edge lies wholly outside `g`.
fn vnode_not_related_to(fg: &Fg, g: GId, v: NId) -> bool {
    if fg.nodes[v].node_type != NodeType::Virtual {
        return false; // position.c:391-392
    }
    let mut e = fg.nodes[v].save_out[0]; // position.c:393
    while let Some(orig) = fg.edges[e].to_orig {
        e = orig;
    }
    !agcontains(fg, g, fg.edges[e].tail) && !agcontains(fg, g, fg.edges[e].head)
}

/// `keepout_othernodes` (position.c:410-442) — push nodes adjacent to (but
/// outside) the cluster's occupied span away by `margin`. When `g` is the
/// root the scans fall outside the row and only the subcluster recursion
/// matters (§7.2 of the spec).
fn keepout_othernodes(fg: &mut Fg, g: GId, state: &mut PosState) {
    let root = fg.root_g();
    let margin = gmargin(state, g); // position.c:415
    for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
        let row_v = rank_nodes(fg, g, r); // the cluster's own rows
        if row_v.is_empty() {
            continue; // position.c:417-418
        }
        let v = row_v[0]; // position.c:419 — leftmost node of the cluster row
        let vorder = fg.nodes[v].order;
        let root_row = rank_nodes(fg, root, r); // root's rank row
        // position.c:422-429 — scan left on the root's row
        let mut i = vorder - 1;
        while i >= 0 {
            let u = root_row[i as usize];
            if fg.nodes[u].node_type == NodeType::Normal || vnode_not_related_to(fg, g, u) {
                let (ln, _) = state.ln_rn[&g];
                let len = margin as f64 + fg.nodes[u].rw;
                let _ = make_aux_edge(fg, u, ln, len, 0);
                break;
            }
            i -= 1;
        }
        // position.c:430-437 — scan right of the cluster's span
        let mut i = (vorder + rank_row(fg, g, r).n as i32) as usize;
        while i < root_row.len() {
            let u = root_row[i];
            if fg.nodes[u].node_type == NodeType::Normal || vnode_not_related_to(fg, g, u) {
                let (_, rn) = state.ln_rn[&g];
                let len = margin as f64 + fg.nodes[u].lw;
                let _ = make_aux_edge(fg, rn, u, len, 0);
                break;
            }
            i += 1;
        }
    }
    for c in fg.graphs[g].clust.clone() {
        // position.c:440-441
        keepout_othernodes(fg, c, state);
    }
}

/// `contain_subclust` (position.c:449-465) — keep subcluster boxes inside
/// g's box, including any left/right label margins.
fn contain_subclust(fg: &mut Fg, g: GId, state: &mut PosState) {
    let margin = gmargin(state, g); // position.c:454
    make_lrvn(fg, g, state); // position.c:455
    for c in fg.graphs[g].clust.clone() {
        make_lrvn(fg, c, state); // position.c:458
        let (ln_g, rn_g) = state.ln_rn[&g];
        let (ln_c, rn_c) = state.ln_rn[&c];
        let border = fg.graphs[g].border;
        // position.c:459-462 (LEFT_IX = 3, RIGHT_IX = 1)
        let _ = make_aux_edge(fg, ln_g, ln_c, margin as f64 + border[3].x, 0);
        let _ = make_aux_edge(fg, rn_c, rn_g, margin as f64 + border[1].x, 0);
        contain_subclust(fg, c, state); // position.c:463
    }
}

/// `separate_subclust` (position.c:472-502) — keep sibling clusters apart
/// where their rank spans overlap.
fn separate_subclust(fg: &mut Fg, g: GId, state: &mut PosState) {
    let margin = gmargin(state, g); // position.c:478
    let clust = fg.graphs[g].clust.clone();
    for &c in &clust {
        make_lrvn(fg, c, state); // position.c:479-480
    }
    for i in 0..clust.len() {
        for j in (i + 1)..clust.len() {
            let (mut low, mut high) = (clust[i], clust[j]);
            if fg.graphs[low].minrank > fg.graphs[high].minrank {
                std::mem::swap(&mut low, &mut high); // position.c:485-487
            }
            if fg.graphs[low].maxrank < fg.graphs[high].minrank {
                continue; // position.c:488-489 — disjoint spans
            }
            let lo_v0 = rank_row(fg, low, fg.graphs[high].minrank).v[0]; // L490
            let hi_v0 = rank_row(fg, high, fg.graphs[high].minrank).v[0]; // L491
            let (left, right) = if fg.nodes[lo_v0].order < fg.nodes[hi_v0].order {
                (low, high)
            } else {
                (high, low)
            };
            let (_, rn_left) = state.ln_rn[&left];
            let (ln_right, _) = state.ln_rn[&right];
            let _ = make_aux_edge(fg, rn_left, ln_right, margin as f64, 0); // L498
        }
        separate_subclust(fg, clust[i], state); // position.c:500 — inside the i loop
    }
}

/// `pos_clusters` (position.c:509-517).
fn pos_clusters(fg: &mut Fg, g: GId, state: &mut PosState) {
    if !fg.graphs[g].clust.is_empty() {
        contain_clustnodes(fg, g, state);
        keepout_othernodes(fg, g, state);
        contain_subclust(fg, g, state);
        separate_subclust(fg, g, state);
    }
}

/// `compress_graph` (position.c:519-541) — `ratio=compress` becomes a
/// weight-1000 width pin between the root's `ln`/`rn`.
fn compress_graph(fg: &mut Fg, g: GId, state: &mut PosState) {
    if state.drawing.ratio_kind != RatioKind::Compress {
        return; // position.c:524-525
    }
    let p = state.drawing.size;
    if p.x * p.y <= 1.0 {
        return; // position.c:527-528
    }
    contain_nodes(fg, g, state); // position.c:529
    let (ln, rn) = state.ln_rn[&g];
    let x = if !fg.graphs[g].rankdir.flip() {
        p.x
    } else {
        p.y
    }; // L530-533
    let x = x.min(USHRT_MAX); // position.c:539
    let _ = make_aux_edge(fg, ln, rn, x, 1000); // position.c:540
}

/// `create_aux_edges` (position.c:543-560).
fn create_aux_edges(fg: &mut Fg, g: GId, state: &mut PosState) -> Result<(), i32> {
    allocate_aux_edges(fg, g);
    make_lr_constraints(fg, g)?;
    make_edge_pairs(fg, g)?;
    pos_clusters(fg, g, state);
    compress_graph(fg, g, state);
    Ok(())
}

/// `remove_aux_edges` (position.c:562-595) — restore the saved fast lists,
/// then unlink every SLACKNODE from `GD_nlist`. (Aux edge objects stay in
/// the arena but become unreachable, which is the Rust equivalent of the C
/// frees; the two passes cannot be merged — the unlink pass needs the
/// stable `ND_next` chain, position.c:578.)
fn remove_aux_edges(fg: &mut Fg, g: GId) {
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        // position.c:568-576 — free aux edges; restore saved lists
        let save_out = std::mem::take(&mut fg.nodes[id].save_out);
        let save_in = std::mem::take(&mut fg.nodes[id].save_in);
        fg.nodes[id].out = save_out;
        fg.nodes[id].in_ = save_in;
    }
    // position.c:579-594 — unlink & drop slacknodes
    let mut nprev: Option<NId> = None;
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        let nnext = fg.nodes[id].next;
        if fg.nodes[id].node_type == NodeType::Slacknode {
            match nprev {
                Some(p) => fg.nodes[p].next = nnext,
                None => fg.graphs[g].nlist = nnext,
            }
            if let Some(nx) = nnext {
                fg.nodes[nx].prev = nprev;
            }
        } else {
            nprev = Some(id);
        }
        n = nnext;
    }
}

/// `set_xcoords` (position.c:598-610) — copy the simplex x out of `ND_rank`
/// into `ND_coord.x` and restore `ND_rank` = rank index. Nodes not installed
/// in rank arrays (the cluster `ln`/`rn` vnodes) keep their simplex x, which
/// `dot_compute_bb` reads at position.c:895-896.
fn set_xcoords(fg: &mut Fg, g: GId) {
    let minr = fg.graphs[g].minrank;
    let maxr = fg.graphs[g].maxrank;
    for i in minr..=maxr {
        let row_v = rank_nodes(fg, g, i);
        for &v in &row_v {
            fg.nodes[v].coord.x = fg.nodes[v].rank as f64; // position.c:606
            fg.nodes[v].rank = i; // position.c:607
        }
    }
}

/// `adjustSimple` (position.c:622-649) — expand cluster height by `delta`,
/// shifting affected rank-leftmost y's when the expansion exceeds the global
/// rank heights. `margin_total` (accumulated ancestor margins) converts the
/// global rank `ht1/ht2` back to this cluster's frame.
fn adjust_simple(fg: &mut Fg, g: GId, delta: f64, margin_total: i32) {
    let root = fg.root_g();
    let maxr = fg.graphs[g].maxrank;
    let minr = fg.graphs[g].minrank;
    let bottom = (delta + 1.0) / 2.0; // position.c:630
    let delbottom =
        fg.graphs[g].ht1 + bottom - (rank_row(fg, root, maxr).ht1 - margin_total as f64);
    
    let deltop = if delbottom > 0.0 {
        let mut r = maxr;
        while r >= minr {
            // position.c:633-636
            if rank_row(fg, root, r).n > 0 {
                let v0 = rank_row(fg, root, r).v[0];
                fg.nodes[v0].coord.y += delbottom;
            }
            r -= 1;
        }
        fg.graphs[g].ht2 + (delta - bottom) + delbottom
            - (rank_row(fg, root, minr).ht2 - margin_total as f64)
    } else {
        fg.graphs[g].ht2 + (delta - bottom)
            - (rank_row(fg, root, minr).ht2 - margin_total as f64)
    };
    if deltop > 0.0 {
        let minroot = fg.graphs[root].minrank;
        let mut r = minr - 1;
        while r >= minroot {
            // position.c:642-645
            if rank_row(fg, root, r).n > 0 {
                let v0 = rank_row(fg, root, r).v[0];
                fg.nodes[v0].coord.y += deltop;
            }
            r -= 1;
        }
    }
    fg.graphs[g].ht2 += delta - bottom; // position.c:647
    fg.graphs[g].ht1 += bottom; // position.c:648
}

/// `adjustRanks` (position.c:656-701) — recursively adjust ranks for wide
/// cluster labels when rankdir=LR (only invoked with `lbl && flip`).
fn adjust_ranks(fg: &mut Fg, g: GId, margin_total: i32, state: &PosState) {
    let root = fg.root_g();
    let margin: i32 = if g == root { 0 } else { gmargin(state, g) }; // L665-668
    let mut ht1 = fg.graphs[g].ht1;
    let mut ht2 = fg.graphs[g].ht2;
    for c in fg.graphs[g].clust.clone() {
        adjust_ranks(fg, c, margin + margin_total, state); // L675
        if fg.graphs[c].maxrank == fg.graphs[g].maxrank {
            ht1 = ht1.max(fg.graphs[c].ht1 + margin as f64); // L677
        }
        if fg.graphs[c].minrank == fg.graphs[g].minrank {
            ht2 = ht2.max(fg.graphs[c].ht2 + margin as f64); // L679
        }
    }
    fg.graphs[g].ht1 = ht1; // L682-683
    fg.graphs[g].ht2 = ht2;
    if g != root && fg.graphs[g].label.is_some() {
        let border = fg.graphs[g].border;
        let lht = border[3].y.max(border[1].y); // L686 — LEFT_IX/RIGHT_IX y
        let maxr = fg.graphs[g].maxrank;
        let minr = fg.graphs[g].minrank;
        let y_min = fg.nodes[rank_row(fg, root, minr).v[0]].coord.y;
        let y_max = fg.nodes[rank_row(fg, root, maxr).v[0]].coord.y;
        let rht = y_min - y_max; // L689
        let delta = lht - (rht + ht1 + ht2); // L690
        if delta > 0.0 {
            adjust_simple(fg, g, delta, margin_total); // L692
        }
    }
    // update the global ranks (L697-700)
    if g != root {
        let (minr, maxr) = (fg.graphs[g].minrank, fg.graphs[g].maxrank);
        let (h1, h2) = (fg.graphs[g].ht1, fg.graphs[g].ht2);
        let cur2 = rank_row(fg, root, minr).ht2;
        rank_row_mut(fg, root, minr).ht2 = cur2.max(h2);
        let cur1 = rank_row(fg, root, maxr).ht1;
        rank_row_mut(fg, root, maxr).ht1 = cur1.max(h1);
    }
}

/// `clust_ht` (position.c:708-753) — recursively fold subcluster
/// half-heights and (TB) cluster-label heights into `GD_ht1/GD_ht2` and the
/// root's global rank half-heights; returns "some cluster has a label".
fn clust_ht(fg: &mut Fg, g: GId, state: &PosState) -> bool {
    let root = fg.root_g();
    let margin: f64 = if g == root {
        CL_OFFSET // position.c:716-717 — the root's own margin is ignored
    } else {
        gmargin(state, g) as f64
    };
    let mut ht1 = fg.graphs[g].ht1;
    let mut ht2 = fg.graphs[g].ht2;
    let mut have_clust_label = false;
    for c in fg.graphs[g].clust.clone() {
        have_clust_label |= clust_ht(fg, c, state); // children first (L727)
        if fg.graphs[c].maxrank == fg.graphs[g].maxrank {
            ht1 = ht1.max(fg.graphs[c].ht1 + margin); // L728-729
        }
        if fg.graphs[c].minrank == fg.graphs[g].minrank {
            ht2 = ht2.max(fg.graphs[c].ht2 + margin); // L730-731
        }
    }
    // room for the root graph label is handled in dotneato_postprocess (L735)
    if g != root && fg.graphs[g].label.is_some() {
        have_clust_label = true;
        if !fg.graphs[root].rankdir.flip() {
            // TB only (L738-741); border = [bottom, right, top, left]
            ht1 += fg.graphs[g].border[0].y; // BOTTOM_IX
            ht2 += fg.graphs[g].border[2].y; // TOP_IX
        }
    }
    fg.graphs[g].ht1 = ht1; // L743-744
    fg.graphs[g].ht2 = ht2;
    if g != root {
        // propagate into the shared global rank array (L747-750)
        let (minr, maxr) = (fg.graphs[g].minrank, fg.graphs[g].maxrank);
        let cur2 = rank_row(fg, root, minr).ht2;
        rank_row_mut(fg, root, minr).ht2 = cur2.max(ht2);
        let cur1 = rank_row(fg, root, maxr).ht1;
        rank_row_mut(fg, root, maxr).ht1 = cur1.max(ht1);
    }
    have_clust_label
}

/// `set_ycoords` (position.c:756-848) — y coordinates, a rank at a time.
/// `ND_rank` still holds rank indices here; y grows upward (maxrank is the
/// bottom rank).
fn set_ycoords(fg: &mut Fg, g: GId, state: &PosState) {
    let minr = fg.graphs[g].minrank;
    let maxr = fg.graphs[g].maxrank;

    // ---- Phase A — scan ranks for tallest nodes (L763-795) ----
    for r in minr..=maxr {
        let row_v = rank_nodes(fg, g, r);
        for n in row_v {
            // assumes symmetry, ht1 = ht2 (L767-768)
            let mut ht2 = fg.nodes[n].ht / 2.0;
            // high self-edge labels can exceed the node half-height (L772-778)
            if !fg.nodes[n].other.is_empty() {
                for &e in &fg.nodes[n].other {
                    if fg.edges[e].tail == fg.edges[e].head
                        && let Some(l) = fg.edges[e].label {
                            ht2 = ht2.max(fg.labels[l].dimen.y / 2.0);
                        }
                }
            }
            // update global rank ht (L781-784). `pht1/pht2` track the
            // *primitive* node heights; `ht1/ht2` start equal but clusters and
            // labels raise them below, and set_ycoords uses both.
            {
                let row = rank_row_mut(fg, g, r);
                if row.pht2 < ht2 {
                    row.pht2 = ht2;
                }
                if row.pht1 < ht2 {
                    row.pht1 = ht2;
                }
                if row.ht2 < ht2 {
                    row.ht2 = ht2;
                }
                if row.ht1 < ht2 {
                    row.ht1 = ht2;
                }
            }
            // update nearest enclosing cluster half-heights (L787-793)
            if let Some(clust) = fg.nodes[n].clust {
                let yoff = if clust == g { 0 } else { gmargin(state, clust) };
                if fg.nodes[n].rank == fg.graphs[clust].minrank {
                    fg.graphs[clust].ht2 = fg.graphs[clust].ht2.max(ht2 + yoff as f64);
                }
                if fg.nodes[n].rank == fg.graphs[clust].maxrank {
                    fg.graphs[clust].ht1 = fg.graphs[clust].ht1.max(ht2 + yoff as f64);
                }
            }
        }
    }

    // ---- Phase B — recursive cluster heights (L798) ----
    let lbl = clust_ht(fg, g, state);

    // ---- Phase C — initial y for the leftmost node of each rank,
    //      bottom-up (L801-816) ----
    let mut maxht = 0.0f64;
    {
        // Cluster graphs whose rank arrays are not yet expanded by mincross
        // (clusters are only partly wired) have nothing to place here; skip
        // rather than index an empty rank.
        let Some(&v0) = rank_row(fg, g, maxr).v.first() else {
            return;
        };
        let ht1 = rank_row(fg, g, maxr).ht1;
        fg.nodes[v0].coord.y = ht1; // L803
    }
    let mut r = maxr;
    loop {
        r -= 1; // while (--r >= GD_minrank(g))
        if r < minr {
            break;
        }
        // C L805-806: d0 uses the primitive heights (`pht`), d1 the
        // cluster/label-augmented ones (`ht`).
        let (pht2_next, pht1_cur) = (rank_row(fg, g, r + 1).pht2, rank_row(fg, g, r).pht1);
        let (ht2_next, ht1_cur) = (rank_row(fg, g, r + 1).ht2, rank_row(fg, g, r).ht1);
        let d0 = pht2_next + pht1_cur + fg.graphs[g].ranksep as f64; // L805 — prim sep
        let d1 = ht2_next + ht1_cur + CL_OFFSET; // L806 — cluster sep
        let delta = d0.max(d1);
        if rank_row(fg, g, r).n > 0 {
            // empty ranks "reflect some problem" (L808)
            let yprev = fg.nodes[rank_row(fg, g, r + 1).v[0]].coord.y;
            let v0 = rank_row(fg, g, r).v[0];
            fg.nodes[v0].coord.y = yprev + delta; // L809
        }
        maxht = maxht.max(delta); // L815
    }

    // ---- Phase D — rotated (LR) cluster labels (L823-836) ----
    if lbl && fg.graphs[g].rankdir.flip() {
        adjust_ranks(fg, g, 0, state); // L824
        if fg.graphs[g].exact_ranksep {
            // recompute maxht from actual gaps (L825-835)
            maxht = 0.0;
            let mut r = maxr;
            let mut d0 = fg.nodes[rank_row(fg, g, r).v[0]].coord.y;
            loop {
                r -= 1;
                if r < minr {
                    break;
                }
                let d1 = fg.nodes[rank_row(fg, g, r).v[0]].coord.y;
                maxht = maxht.max(d1 - d0);
                d0 = d1;
            }
        }
    }

    // ---- Phase E — re-assign if ranks are equally spaced (L839-843) ----
    if fg.graphs[g].exact_ranksep {
        let mut r = maxr - 1;
        while r >= minr {
            if rank_row(fg, g, r).n > 0 {
                let yprev = fg.nodes[rank_row(fg, g, r + 1).v[0]].coord.y;
                let v0 = rank_row(fg, g, r).v[0];
                fg.nodes[v0].coord.y = yprev + maxht;
            }
            r -= 1;
        }
    }

    // ---- Phase F — copy y from leftmost nodes to all nodes (L846-847) ----
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        let r = fg.nodes[id].rank; // rank index at this point
        let y = fg.nodes[rank_row(fg, g, r).v[0]].coord.y;
        fg.nodes[id].coord.y = y;
    }
}

/// `dot_compute_bb` (position.c:857-902) — bounding box of `g`. For the root
/// the x extent comes from the leftmost/rightmost NORMAL node per rank,
/// widened by every child cluster bbox ± CL_OFFSET; for a cluster it comes
/// from the simplex x values kept in `ND_rank(ln)`/`ND_rank(rn)`.
fn dot_compute_bb(fg: &mut Fg, g: GId, root: GId, state: &PosState) {
    let (ll_x, ur_x);
    if g == root {
        let mut llx = i32::MAX as f64; // L865
        let mut urx = -(i32::MAX as f64); // L866 — note: -INT_MAX, not INT_MIN
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let (rnkn, row_v) = {
                let row = rank_row(fg, g, r);
                (row.n, row.v[..row.n.min(row.v.len())].to_vec())
            };
            if rnkn == 0 {
                continue; // L869-870
            }
            let mut v = row_v[0]; // L871
            let mut c = 1usize;
            while fg.nodes[v].node_type != NodeType::Normal && c < rnkn {
                v = row_v[c]; // L873-874 — first NORMAL from the left
                c += 1;
            }
            if fg.nodes[v].node_type == NodeType::Normal {
                let x = fg.nodes[v].coord.x - fg.nodes[v].lw;
                llx = llx.min(x); // L876-877
            } else {
                continue; // L879 — the rank has no NORMAL node
            }
            let mut v = row_v[rnkn - 1]; // L881 — first NORMAL from the right
            let mut c = rnkn as i32 - 2;
            while fg.nodes[v].node_type != NodeType::Normal {
                v = row_v[c as usize];
                c -= 1;
            }
            let x = fg.nodes[v].coord.x + fg.nodes[v].rw;
            urx = urx.max(x); // L884-885
        }
        let offset = CL_OFFSET; // L887
        for c in fg.graphs[g].clust.clone() {
            let cbb = fg.graphs[c].bb;
            llx = llx.min(cbb.ll.x - offset); // L889-890
            urx = urx.max(cbb.ur.x + offset); // L891-892
        }
        ll_x = llx;
        ur_x = urx;
    } else {
        let (ln, rn) = state.ln_rn[&g];
        ll_x = fg.nodes[ln].rank as f64; // L895 — simplex x
        ur_x = fg.nodes[rn].rank as f64; // L896 — simplex x
    }
    let y_bot = fg.nodes[rank_row(fg, root, fg.graphs[g].maxrank).v[0]]
        .coord
        .y; // L898
    let y_top = fg.nodes[rank_row(fg, root, fg.graphs[g].minrank).v[0]]
        .coord
        .y; // L899
    let ll = PointF::new(ll_x, y_bot - fg.graphs[g].ht1);
    let ur = PointF::new(ur_x, y_top + fg.graphs[g].ht2);
    fg.graphs[g].bb = BoxF { ll, ur }; // L900-901
}

/// `rec_bb` (position.c:904-910) — post-order bounding boxes.
fn rec_bb(fg: &mut Fg, g: GId, root: GId, state: &PosState) {
    for c in fg.graphs[g].clust.clone() {
        rec_bb(fg, c, root, state);
    }
    dot_compute_bb(fg, g, root, state);
}

/// `scale_bb` (position.c:915-924) — children first, purely multiplicative.
fn scale_bb(fg: &mut Fg, g: GId, xf: f64, yf: f64) {
    for c in fg.graphs[g].clust.clone() {
        scale_bb(fg, c, xf, yf);
    }
    let bb = &mut fg.graphs[g].bb;
    bb.ll.x *= xf;
    bb.ll.y *= yf;
    bb.ur.x *= xf;
    bb.ur.y *= yf;
}

/// `idealsize` (position.c:1130-1159) — set `size` to a whole-page multiple;
/// returns whether the drawing is to be scaled and filled.
fn idealsize(fg: &Fg, state: &mut PosState, minallowed: f64) -> bool {
    let relpage0 = state.drawing.page;
    if relpage0.x < 0.001 || relpage0.y < 0.001 {
        return false; // no page was specified (L1137-1138)
    }
    let margin = state.drawing.margin;
    let relpage = relpage0.sub(margin).sub(margin); // L1139-1141
    let b = PointF::new(
        fg.graphs[fg.root_g()].bb.ur.x,
        fg.graphs[fg.root_g()].bb.ur.y,
    );
    let xf = relpage.x / b.x;
    let yf = relpage.y / b.y;
    if xf >= 1.0 && yf >= 1.0 {
        return false; // fits on one page (L1146-1147)
    }
    let f = xf.min(yf);
    let xf = f.max(minallowed);
    let yf = f.max(minallowed);
    let r1 = (xf * b.x / relpage.x).ceil(); // whole pages (L1152-1153)
    let xf = r1 * relpage.x / b.x;
    let r2 = (yf * b.y / relpage.y).ceil();
    let yf = r2 * relpage.y / b.y;
    state.drawing.size = PointF::new(b.x * xf, b.y * yf);
    true
}

/// `set_aspect` (position.c:930-998) — compute bounding boxes and, if
/// `ratio` is set, rescale the graph. `R_COMPRESS` is *not* scaled here (it
/// became a constraint edge in `compress_graph`); node coordinates round
/// with C99 `round()` (half away from zero), unlike the `ROUND` macro.
fn set_aspect(fg: &mut Fg, g: GId, state: &mut PosState) {
    rec_bb(fg, g, fg.root_g(), state); // L935
    if fg.graphs[g].maxrank > 0 && state.drawing.ratio_kind != RatioKind::None {
        let bb = fg.graphs[g].bb;
        let mut sz = PointF::new(bb.ur.x - bb.ll.x, bb.ur.y - bb.ll.y); // L937
        let flip = fg.graphs[g].rankdir.flip();
        if flip {
            sz = PointF::new(sz.y, sz.x); // exch_xyf (L938-940)
        }
        let mut scale_it = true;
        let filled = if state.drawing.ratio_kind == RatioKind::Auto {
            idealsize(fg, state, 0.5) // L943
        } else {
            state.drawing.ratio_kind == RatioKind::Fill
        };
        let mut xf = 0.0f64;
        let mut yf = 0.0f64;
        if filled {
            // fill is weird because both X and Y can stretch (L946-962)
            if state.drawing.size.x <= 0.0 {
                scale_it = false;
            } else {
                xf = state.drawing.size.x / sz.x;
                yf = state.drawing.size.y / sz.y;
                if xf < 1.0 || yf < 1.0 {
                    if xf < yf {
                        yf /= xf;
                        xf = 1.0;
                    } else {
                        xf /= yf;
                        yf = 1.0;
                    }
                }
            }
        } else if state.drawing.ratio_kind == RatioKind::Expand {
            // L963-974 — note: divides by bb.UR, not by sz
            if state.drawing.size.x <= 0.0 {
                scale_it = false;
            } else {
                xf = state.drawing.size.x / bb.ur.x;
                yf = state.drawing.size.y / bb.ur.y;
                if xf > 1.0 && yf > 1.0 {
                    let s = xf.min(yf);
                    xf = s;
                    yf = s;
                } else {
                    scale_it = false;
                }
            }
        } else if state.drawing.ratio_kind == RatioKind::Value {
            let desired = state.drawing.ratio;
            let actual = sz.y / sz.x;
            if actual < desired {
                yf = desired / actual;
                xf = 1.0;
            } else {
                xf = actual / desired;
                yf = 1.0;
            }
        } else {
            scale_it = false; // R_COMPRESS lands here (L985-986)
        }
        if scale_it {
            if flip {
                std::mem::swap(&mut xf, &mut yf); // L988-990
            }
            let mut n = fg.graphs[g].nlist;
            while let Some(id) = n {
                n = fg.nodes[id].next;
                fg.nodes[id].coord.x = geom::round(fg.nodes[id].coord.x * xf); // L992
                fg.nodes[id].coord.y = geom::round(fg.nodes[id].coord.y * yf); // L993
            }
            scale_bb(fg, g, xf, yf); // L995
        }
    }
}

/// `make_leafslots` (position.c:1001-1028) — make space for leaf nodes of
/// each rank. `ND_ranktype == LEAFSET` is never assigned anywhere in modern
/// dotgen, so the expansion branch (which would leave NULL holes in the
/// rank rows) is dead code; the port keeps the order re-assignment and
/// panics on the unreachable hole case rather than silently reordering.
fn make_leafslots(fg: &mut Fg, g: GId) {
    let minr = fg.graphs[g].minrank;
    let maxr = fg.graphs[g].maxrank;
    for r in minr..=maxr {
        let mut j = 0i32;
        let row_v = rank_nodes(fg, g, r);
        for &v in &row_v {
            fg.nodes[v].order = j; // L1010
            if fg.nodes[v].ranktype == super::model::RankType::LeafSet {
                j += fg.nodes[v].uf_size as i32; // L1012 — leave a hole
            } else {
                j += 1;
            }
        }
        if j <= row_v.len() as i32 {
            continue; // L1016-1017 — no expansion needed
        }
        // L1018-1027 — expansion (dead branch; see above)
        let mut new_v: Vec<Option<NId>> = vec![None; j as usize];
        for &v in row_v.iter().rev() {
            // back to front (L1019-1022)
            let o = fg.nodes[v].order as usize;
            new_v[o] = Some(v);
        }
        let new_v = new_v
            .into_iter()
            .map(|s| s.expect("LEAFSET hole (dead branch: do_leaves removed upstream)"))
            .collect::<Vec<NId>>();
        let n = new_v.len();
        let row = rank_row_mut(fg, g, r);
        row.n = n; // L1023
        row.v = new_v; // new_v[j] = NULL is the Vec length
    }
}

/// `expand_leaves` (position.c:1041-1063).
///
/// **Replicates a live no-op bug**: at position.c:1051 the historical
/// `ND_rank(aghead(e)) - ND_rank(agtail(e))` became
/// `ND_rank(aghead(e)) - ND_rank(aghead(e))`, which is identically 0, so
/// every iteration hits `continue` and the whole `ND_other` loop never
/// merges leaves. Ported as written — do not "fix" silently.
//
// `clippy::eq_op` is allowed rather than the expression "fixed": the
// self-subtraction *is* the point, and rewriting it would diverge from
// upstream Graphviz (docs/graphviz-specs/position.md §10.3).
#[allow(clippy::eq_op)]
fn expand_leaves(fg: &mut Fg, g: GId) {
    make_leafslots(fg, g); // L1047
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        if !fg.nodes[id].other.is_empty() {
            let mut i = 0usize;
            while i < fg.nodes[id].other.len() {
                let e = fg.nodes[id].other[i];
                // position.c:1051 — both operands are aghead(e); d ≡ 0
                let head = fg.edges[e].head;
                let d: i32 = fg.nodes[head].rank - fg.nodes[head].rank;
                if d == 0 {
                    i += 1; // the C for-loop increment still runs on continue
                    continue; // position.c:1052 — always taken
                }
                // — unreachable below (d ≡ 0), kept for shape —
                let f = fg.edges[e].to_orig; // L1053
                if let Some(f) = f
                    && !ports_eq(fg, e, f)
                {
                    zapinlist(&mut fg.nodes[id].other, e); // L1055
                    if d == 1 {
                        fast_edge(fg, e); // L1057
                    }
                    /* else unitize(e); ### (L1058) */
                    i = i.saturating_sub(1); // i-- (L1059): re-examine slot
                }
                i += 1;
            }
        }
    }
}

/// `make_lrvn` (position.c:1078-1096) — add left/right slacknodes holding
/// the x of the cluster bbox sides; idempotent. A labeled cluster (TB only)
/// is forced at least label-wide via a 0-weight `ln`→`rn` edge.
fn make_lrvn(fg: &mut Fg, g: GId, state: &mut PosState) {
    if state.ln_rn.contains_key(&g) {
        return; // L1082-1083
    }
    let root = fg.root_g();
    let ln = virtual_node(fg, root); // L1084 — dot_root(g)
    fg.nodes[ln].node_type = SLACKNODE; // L1085
    let rn = virtual_node(fg, root); // L1086
    fg.nodes[rn].node_type = SLACKNODE; // L1087
    if fg.graphs[g].label.is_some() && g != root && !fg.graphs[root].rankdir.flip() {
        // L1089-1092 — border = [bottom, right, top, left]
        let border = fg.graphs[g].border;
        let w = border[0].x.max(border[2].x); // BOTTOM_IX / TOP_IX x
        let _ = make_aux_edge(fg, ln, rn, w, 0);
    }
    state.ln_rn.insert(g, (ln, rn)); // L1094-1095
}

/// `contain_nodes` (position.c:1101-1125) — constrain the leftmost/rightmost
/// node of every rank inside the cluster's `ln`/`rn`.
fn contain_nodes(fg: &mut Fg, g: GId, state: &mut PosState) {
    let margin = gmargin(state, g); // L1106
    make_lrvn(fg, g, state);
    let (ln, rn) = state.ln_rn[&g];
    let border = fg.graphs[g].border;
    for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
        let row_v = rank_nodes(fg, g, r);
        if row_v.is_empty() {
            continue; // L1111-1112
        }
        let v = row_v[0];
        // L1114-1118 (v == NULL with an != error) cannot occur: the arena
        // keeps v.len() == n, so rows are either empty or fully populated.
        let len = fg.nodes[v].lw + margin as f64 + border[3].x; // LEFT_IX (L1119-1120)
        let _ = make_aux_edge(fg, ln, v, len, 0);
        let vlast = row_v[row_v.len() - 1];
        let ren = fg.nodes[vlast].rw + margin as f64 + border[1].x; // RIGHT_IX (L1122-1123)
        let _ = make_aux_edge(fg, vlast, rn, ren, 0);
    }
}

/// `mark_lowclusters` (cluster.c:399-417) — zap previous cluster labelings
/// on root nodes and their chain vnodes, then re-derive the lowest
/// containing cluster bottom-up.
pub(crate) fn mark_lowclusters(fg: &mut Fg, root: GId) {
    // first, zap any previous cluster labelings (cluster.c:404-415)
    for &n in fg.graphs[root].nodes_order.clone().iter() {
        fg.nodes[n].clust = None;
        for &orig in fg.input_out[n].clone().iter() {
            let mut e = fg.edges[orig].to_virt;
            while let Some(eid) = e {
                let h = fg.edges[eid].head;
                if fg.nodes[h].node_type != NodeType::Virtual {
                    break;
                }
                fg.nodes[h].clust = None;
                e = fg.nodes[h].out.first().copied();
            }
        }
    }
    mark_lowcluster_basic(fg, root);
}

/// `mark_lowcluster_basic` (cluster.c:419-444) — children first, so each
/// node/vnode ends up with the *lowest* containing cluster.
fn mark_lowcluster_basic(fg: &mut Fg, g: GId) {
    for c in fg.graphs[g].clust.clone() {
        mark_lowcluster_basic(fg, c);
    }
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        if fg.nodes[n].clust.is_none() {
            fg.nodes[n].clust = Some(g);
        }
        for &orig in fg.input_out[n].clone().iter() {
            if !fg.graphs[g].owns_input_edge(fg, orig) {
                continue; // agfstout(g, n) iterates only g's own edges
            }
            let mut e = fg.edges[orig].to_virt;
            while let Some(eid) = e {
                let h = fg.edges[eid].head;
                if fg.nodes[h].node_type != NodeType::Virtual {
                    break;
                }
                if fg.nodes[h].clust.is_none() {
                    fg.nodes[h].clust = Some(g);
                }
                e = fg.nodes[h].out.first().copied();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dotgen::{Measured, build, classes, rank};
    use crate::graph::parser::parse;

    fn approx(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-6, "{a} != {b}");
    }

    /// Build the arena the way the real pipeline does up to position:
    /// parse → build → dot1_rank (ranks) → class2 (fast graph + vnodes),
    /// then hand-install the rank arrays (mincross's job, another package).
    /// Rank rows follow nlist order; orders are dense per row.
    fn setup(src: &str, edge_labels: &[Option<(f64, f64)>]) -> Fg {
        let graph = parse(src).unwrap();
        let measured = Measured {
            label: Vec::new(),
            node: vec![(54.0, 36.0); graph.nodes.len()],
            edge_label: edge_labels.to_vec(),
            measure: None,
        };
        let mut fg = build(&graph, &measured);
        rank::dot_rank(&mut fg, 0);
        classes::class2(&mut fg, 0);
        install_ranks(&mut fg, 0);
        fg
    }

    fn install_ranks(fg: &mut Fg, g: GId) {
        let (minr, maxr) = (fg.graphs[g].minrank, fg.graphs[g].maxrank);
        let mut rows: Vec<Vec<NId>> = vec![Vec::new(); (maxr - minr + 1) as usize];
        let mut next = fg.graphs[g].nlist;
        while let Some(n) = next {
            rows[(fg.nodes[n].rank - minr) as usize].push(n);
            next = fg.nodes[n].next;
        }
        for row in rows {
            for (j, &v) in row.iter().enumerate() {
                fg.nodes[v].order = j as i32;
            }
            let n = row.len();
            fg.graphs[g].rank.push(Rank {
                v: row,
                n,
                ..Default::default()
            });
        }
    }

    fn name_id(fg: &Fg, name: &str) -> NId {
        fg.nodes.iter().position(|n| n.name == name).unwrap()
    }

    /// (a) `digraph { a -> b -> c }`: y(a) > y(b) > y(c) with gaps of
    /// ranksep(36) + 18 + 18 = 72. Single-node ranks admit no same-rank
    /// ordering constraint, so the weight-1 slacknode pairs
    /// (position.c:341-370) collapse the chain onto one x — the vertical
    /// stroke real dot draws. ND_rank is restored to rank indices.
    #[test]
    fn chain_layout_y_and_x() {
        let fg = &mut setup("digraph { a -> b -> c; }", &[]);
        dot_position(fg, 0).unwrap();
        let (a, b, c) = (name_id(fg, "a"), name_id(fg, "b"), name_id(fg, "c"));
        assert!(fg.nodes[a].coord.y > fg.nodes[b].coord.y);
        assert!(fg.nodes[b].coord.y > fg.nodes[c].coord.y);
        approx(fg.nodes[a].coord.y - fg.nodes[b].coord.y, 72.0);
        approx(fg.nodes[b].coord.y - fg.nodes[c].coord.y, 72.0);
        assert_eq!(fg.nodes[a].rank, 0);
        assert_eq!(fg.nodes[b].rank, 1);
        assert_eq!(fg.nodes[c].rank, 2);
        approx(fg.nodes[b].coord.x, fg.nodes[a].coord.x);
        approx(fg.nodes[c].coord.x, fg.nodes[a].coord.x);
        let bb = fg.graphs[0].bb;
        assert!(bb.ur.x > bb.ll.x && bb.ur.y > bb.ll.y);
    }

    /// (b) two independent nodes on one rank end up ≥ nodesep apart in x and
    /// at identical y.
    #[test]
    fn nodesep_separates_same_rank_nodes() {
        let fg = &mut setup("digraph { a -> c; b -> c; }", &[]);
        dot_position(fg, 0).unwrap();
        let (a, b) = (name_id(fg, "a"), name_id(fg, "b"));
        assert_eq!(fg.nodes[a].rank, 0);
        assert_eq!(fg.nodes[b].rank, 0);
        approx(fg.nodes[a].coord.y, fg.nodes[b].coord.y);
        let gap = (fg.nodes[a].coord.x - fg.nodes[b].coord.x).abs();
        assert!(gap >= 72.0 - 1e-6, "x gap {gap} < nodesep width 72");
    }

    /// (c) minlen=3 spreads ranks; the two chain vnodes (half-height 0.5)
    /// give y gaps 54.5 + 37 + 54.5 = 146, and the x simplex collapses the
    /// chain onto the endpoints (position.c:341-370 slacknode pairs).
    #[test]
    fn minlen_spreads_ranks() {
        let fg = &mut setup("digraph { a -> b [minlen=3]; }", &[]);
        assert_eq!(fg.graphs[0].maxrank - fg.graphs[0].minrank, 3);
        dot_position(fg, 0).unwrap();
        let (a, b) = (name_id(fg, "a"), name_id(fg, "b"));
        approx(fg.nodes[a].coord.y - fg.nodes[b].coord.y, 146.0);
        let vnodes: Vec<NId> = (0..fg.nodes.len())
            .filter(|&n| fg.nodes[n].node_type == NodeType::Virtual)
            .collect();
        assert_eq!(vnodes.len(), 2);
        for &v in &vnodes {
            assert!(fg.nodes[a].coord.y > fg.nodes[v].coord.y);
            assert!(fg.nodes[v].coord.y > fg.nodes[b].coord.y);
            approx(fg.nodes[v].coord.x, fg.nodes[a].coord.x);
        }
        approx(fg.nodes[vnodes[0]].coord.x, fg.nodes[vnodes[1]].coord.x);
    }

    /// (d) rankdir=LR: dot lays out internally TB — the screen-axis flip
    /// happens later in dotneato_postprocess, not in position. GD_flip shows
    /// up here only through the swapped node half-widths (lw = rw = h/2),
    /// which is why the y gap is 27+27+36 = 90, not 72.
    #[test]
    fn rankdir_lr_keeps_internal_y_axis() {
        let fg = &mut setup("digraph { rankdir=LR; a -> b; }", &[]);
        dot_position(fg, 0).unwrap();
        let (a, b) = (name_id(fg, "a"), name_id(fg, "b"));
        assert!(fg.nodes[a].coord.y > fg.nodes[b].coord.y);
        approx(fg.nodes[a].coord.y - fg.nodes[b].coord.y, 90.0); // 27+27+36
        approx(fg.nodes[b].coord.x, fg.nodes[a].coord.x);
    }

    /// flat_edges returns true only when a label vnode was created.
    #[test]
    fn flat_edges_reset_flag() {
        let fg = &mut setup(
            "digraph { x -> a; { rank=same; a -> m; a -> c [label=\"AAA\"]; } }",
            &[None, None, Some((30.0, 14.0))],
        );
        assert!(flat::flat_edges(fg, 0));
        let fg = &mut setup("digraph { a -> b -> c; }", &[]);
        assert!(!flat::flat_edges(fg, 0));
    }

    /// A labeled non-adjacent flat edge (a→c spanning over m) gets a virtual
    /// label vnode on rank(a)-1 (flat.c:136-182) sharing the edge's label
    /// record, and x constraints of m0 + widths = 60 per side
    /// (position.c:283-295) place it between the endpoints. (Edge labels
    /// double minlens upstream, so x sits on rank 0 and a/m/c on rank 2.)
    #[test]
    fn labeled_flat_edge_gets_label_vnode() {
        let fg = &mut setup(
            "digraph { x -> a; { rank=same; a -> m; a -> c [label=\"AAA\"]; } }",
            &[None, None, Some((30.0, 14.0))],
        );
        dot_position(fg, 0).unwrap();
        let (a, c, x) = (name_id(fg, "a"), name_id(fg, "c"), name_id(fg, "x"));
        assert_eq!(fg.nodes[x].rank, 0);
        assert_eq!(fg.nodes[a].rank, 2);
        let e_ac = fg
            .edges
            .iter()
            .position(|e| e.tail == a && e.head == c)
            .unwrap();
        let label = fg.edges[e_ac].label.unwrap();
        // the label vnode: virtual, on rank(a)-1, identified structurally
        let vn = (0..fg.nodes.len())
            .find(|&n| fg.nodes[n].node_type == NodeType::Virtual && flat::nd_alg(fg, n).is_some())
            .unwrap();
        assert_eq!(fg.nodes[vn].rank, 1);
        assert_eq!(fg.nodes[vn].label, Some(label)); // shared record (flat.c:166)
        approx(fg.nodes[vn].ht, 14.0); // ND_ht = dimen.y (flat.c:163)
        approx(fg.nodes[vn].lw, 15.0); // lw = rw = dimen.x/2 (flat.c:165)
        approx(fg.nodes[vn].rw, 15.0);
        assert_eq!(flat::nd_alg(fg, vn), Some(e_ac));
        // rank 1 got the vnode slotted in with orders fixed up (flat.c:20-39)
        let row1 = &fg.graphs[0].rank[1];
        assert_eq!(row1.n, 2);
        assert_eq!(fg.nodes[vn].order, 0);
        // m0 = minlen(2) * nodesep(18) / 2 = 18; m1 = 18 + rw + lw = 60
        assert!(fg.nodes[vn].coord.x >= fg.nodes[c].coord.x + 60.0 - 1e-6);
        assert!(fg.nodes[a].coord.x >= fg.nodes[vn].coord.x + 60.0 - 1e-6);
        // the second set_ycoords ran (reset); the vnode owns rank 1's y:
        // y = y(rank2.v[0]) + rank2.ht2 + rank1.ht1(= h2 = 7) + ranksep(18)
        approx(fg.nodes[vn].coord.y, 61.0);
        assert!(fg.nodes[vn].coord.y > fg.nodes[c].coord.y);
        assert!(fg.nodes[vn].coord.y < fg.nodes[x].coord.y);
    }

    /// Self loops: ND_rw grows by selfRightSpace = 18 + label width
    /// (position.c:249-265 with splines.c:1137) while ND_mval keeps the
    /// original, and a tall self-loop label raises the rank half-height
    /// (position.c:772-778). (Edge labels double minlens, so x is on rank 0
    /// and a/c on rank 2.)
    #[test]
    fn self_loop_space_and_label_height() {
        // edges: x->a (0), x->c (1), a->c (2, flat), a->a (3, labeled self)
        let fg = &mut setup(
            "digraph { x -> a; x -> c; { rank=same; a -> c; a -> a [label=\"S\"]; } }",
            &[None, None, None, Some((9.0, 80.0))],
        );
        let (a, c) = (name_id(fg, "a"), name_id(fg, "c"));
        // make a the LEFT node of rank 2 so its inflated rw feeds the
        // ordering + flat-endpoint constraints
        {
            let row = &mut fg.graphs[0].rank[2];
            row.v = vec![a, c];
            row.n = 2;
        }
        fg.nodes[a].order = 0;
        fg.nodes[c].order = 1;
        dot_position(fg, 0).unwrap();
        approx(fg.nodes[a].rw, 27.0 + 18.0 + 9.0);
        approx(fg.nodes[a].mval, 27.0); // ND_mval backup (position.c:248)
        // flat-endpoint constraint (position.c:310-318): m0 = 2*18 + (54+27)
        // strengthened onto the ordering edge with weight 1 pins the gap
        approx(fg.nodes[c].coord.x - fg.nodes[a].coord.x, 117.0);
        // the 80pt self-loop label raises rank 2's half-heights to 40
        approx(fg.graphs[0].rank[2].ht1, 40.0);
        approx(fg.graphs[0].rank[2].ht2, 40.0);
        approx(fg.nodes[a].coord.y, fg.nodes[c].coord.y);
        // y(x) - y(a) = (40 + 0.5 + 18) + (0.5 + 18 + 18) = 95
        approx(
            fg.nodes[name_id(fg, "x")].coord.y - fg.nodes[a].coord.y,
            95.0,
        );
    }

    /// `ranksep=equally` (GD_exact_ranksep) re-assigns every rank gap to the
    /// maximum observed gap (position.c:839-843).
    #[test]
    fn exact_ranksep_equalizes_gaps() {
        let fg = &mut setup("digraph { a -> b [minlen=3]; }", &[]);
        fg.graphs[0].exact_ranksep = true;
        dot_position(fg, 0).unwrap();
        // rows are ordered top (minrank) → bottom (maxrank); y grows upward
        let ys: Vec<f64> = (0..4)
            .map(|r| fg.nodes[rank_row(fg, 0, r).v[0]].coord.y)
            .collect();
        // maxht = max(54.5, 37, 54.5) = 54.5
        for w in ys.windows(2) {
            approx(w[0] - w[1], 54.5);
        }
    }

    /// set_aspect with `ratio=R_VALUE` scales node coordinates by
    /// `round(coord * f)` (C99 round) and bboxes multiplicatively
    /// (position.c:975-995). The chain box is 54 wide × 180 tall, so
    /// actual = 10/3 < desired = 5 → yf = 1.5, xf = 1.
    #[test]
    fn set_aspect_ratio_value_scales() {
        let fg = &mut setup("digraph { a -> b -> c; }", &[]);
        dot_position(fg, 0).unwrap();
        let b = name_id(fg, "b");
        let bb0 = fg.graphs[0].bb;
        let yb = fg.nodes[b].coord.y;
        let actual = (bb0.ur.y - bb0.ll.y) / (bb0.ur.x - bb0.ll.x);
        assert!((actual - 10.0 / 3.0).abs() < 1e-9);
        let yf = 5.0 / actual; // = 1.5
        let mut state = PosState::default();
        state.drawing.ratio_kind = RatioKind::Value;
        state.drawing.ratio = 5.0;
        set_aspect(fg, 0, &mut state);
        approx(fg.nodes[b].coord.y, geom::round(yb * yf)); // round(90 * 1.5)
        approx(fg.graphs[0].bb.ur.y, bb0.ur.y * yf); // scale_bb: 180 * 1.5
    }
}
