//! Flat-edge processing — a faithful port of `lib/dotgen/flat.c`
//! (Graphviz main @ `2e92c7f`, 336 lines).
//!
//! `flat_edges` is invoked from `position.c:133`. It
//!
//! 1. marks which flat edges have *adjacent* endpoints
//!    (`check_flat_adjacent`, flat.c:208-238),
//! 2. inserts a new empty bottom rank when a labeled non-adjacent flat edge
//!    sits on rank 0 and its label node needs somewhere to go
//!    (`abomination`, flat.c:184-202),
//! 3. creates a virtual label node on rank `r-1` for every labeled
//!    non-adjacent flat edge (`flat_node`, flat.c:136-182) and stores the
//!    label width of adjacent flat edges in `ED_dist`.
//!
//! It returns `true` ("reset") when any label node was created, which makes
//! `dot_position` re-run `set_ycoords` (`position.c:133-134`).
//!
//! **Deliberately not ported here** (they live in `mincross.c`, a separate
//! work package): `checkLabelOrder` (mincross.c:297-326) and
//! `rec_reset_vlists` (mincross.c:930-953), called at flat.c:332-334, and the
//! `rec_save_vlists` snapshot at flat.c:294. `flat_breakcycles` /
//! `flat_search` / `flat_reorder` likewise belong to mincross.
//!
//! Two C fields have no arena counterpart and are reproduced without extra
//! storage:
//!
//! * `ND_alg(vn)` — a label vnode is identified structurally: it is
//!   `VIRTUAL` and its (pre-aux) out list is exactly the two `FLATORDER`
//!   edges created by [`flat_node`], both carrying `ED_to_orig == e` (the
//!   C invariant "virtual **and** ND_alg != NULL", flat.c:131-135). See
//!   [`nd_alg`].
//! * `ED_dist(e)` — recomputable at read time from the immutable label
//!   records and the `ND_other` equivalent edges (flat.c:302 and the
//!   flat.c:321 MAX-accumulation). See [`edge_dist`].

use super::classes::virtual_edge;
use super::classes::virtual_node;
use super::geom::PointF;
use super::model::{EId, EdgeType, Fg, GId, NId, NodeType};
use super::position::{rank_row, rank_row_mut};

/// `flat.c:41` — hard left bound slot.
const HLB: usize = 0;
/// `flat.c:42` — hard right bound slot.
const HRB: usize = 1;
/// `flat.c:43` — soft left bound slot.
const SLB: usize = 2;
/// `flat.c:44` — soft right bound slot.
const SRB: usize = 3;

/// `findlr` (flat.c:46-56) — the two nodes' orders as an ordered pair
/// `(l, r)` with `l <= r`.
fn findlr(fg: &Fg, u: NId, v: NId) -> (i32, i32) {
    let l = fg.nodes[u].order;
    let r = fg.nodes[v].order;
    if l > r { (r, l) } else { (l, r) }
}

/// `setbounds` (flat.c:58-102) — tighten `bounds` using node `v` on the
/// previous rank and the target interval `[lpos, rpos]` (the flat edge's
/// endpoint orders, left first). Only `VIRTUAL` nodes do anything.
fn setbounds(fg: &Fg, v: NId, bounds: &mut [i32; 4], lpos: i32, rpos: i32) {
    if fg.nodes[v].node_type != NodeType::Virtual {
        return; // flat.c:63
    }
    let ord = fg.nodes[v].order; // flat.c:64
    if fg.nodes[v].in_.is_empty() {
        // flat.c:65 — "flat": a label node of another flat edge
        debug_assert_eq!(fg.nodes[v].out.len(), 2); // flat.c:66
        let h0 = fg.edges[fg.nodes[v].out[0]].head;
        let h1 = fg.edges[fg.nodes[v].out[1]].head;
        let (l, r) = findlr(fg, h0, h1); // flat.c:67-68
        if r <= lpos {
            // the other flat edge lies wholly to the left (flat.c:70-71)
            bounds[SLB] = ord;
            bounds[HLB] = ord;
        } else if l >= rpos {
            // wholly to the right (flat.c:72-73)
            bounds[SRB] = ord;
            bounds[HRB] = ord;
        } else if l < lpos && r > rpos {
            // spanning this one — ignore (flat.c:75)
        } else {
            // intersecting ranges (flat.c:77-82)
            if l < lpos || (l == lpos && r < rpos) {
                bounds[SLB] = ord;
            }
            if r > rpos || (r == rpos && l > lpos) {
                bounds[SRB] = ord;
            }
        }
    } else {
        // flat.c:83-100 — "forward": a virtual node of a long edge chain
        let mut onleft = false;
        let mut onright = false;
        for &f in &fg.nodes[v].out {
            let h = fg.edges[f].head;
            if fg.nodes[h].order <= lpos {
                onleft = true;
                continue;
            }
            if fg.nodes[h].order >= rpos {
                onright = true;
                continue;
            }
        }
        if onleft && !onright {
            bounds[HLB] = ord + 1; // flat.c:96-97
        }
        if onright && !onleft {
            bounds[HRB] = ord - 1; // flat.c:98-99
        }
    }
}

/// `flat_limits` (flat.c:104-129) — insertion index (order) for the label
/// node of flat edge `e`, scanning rank `rank(tail)-1` inward from both
/// ends; the result is the midpoint of the first feasible hard (else soft)
/// interval.
fn flat_limits(fg: &Fg, g: GId, e: EId) -> i32 {
    let r = fg.nodes[fg.edges[e].tail].rank - 1; // flat.c:108
    let rank_v = super::position::rank_nodes(fg, g, r); // flat.c:109
    let mut lnode = 0i32; // flat.c:110
    let mut rnode = rank_v.len() as i32 - 1; // flat.c:111
    let mut bounds = [0i32; 4];
    bounds[HLB] = lnode - 1; // flat.c:112
    bounds[SLB] = lnode - 1;
    bounds[HRB] = rnode + 1; // flat.c:113
    bounds[SRB] = rnode + 1;
    let (lpos, rpos) = findlr(fg, fg.edges[e].tail, fg.edges[e].head); // flat.c:114
    while lnode <= rnode {
        // flat.c:115-123 — inward scan, hard bounds first
        setbounds(fg, rank_v[lnode as usize], &mut bounds, lpos, rpos);
        if lnode != rnode {
            setbounds(fg, rank_v[rnode as usize], &mut bounds, lpos, rpos);
        }
        lnode += 1;
        rnode -= 1;
        if bounds[HRB] - bounds[HLB] <= 1 {
            break;
        }
    }
    if bounds[HLB] <= bounds[HRB] {
        (bounds[HLB] + bounds[HRB] + 1) / 2 // flat.c:124-125 — integer division
    } else {
        (bounds[SLB] + bounds[SRB] + 1) / 2 // flat.c:126-127
    }
}

/// `make_vn_slot` (flat.c:20-39) — insert a fresh virtual node into rank `r`
/// at index `pos`, shifting the tail of the rank right and bumping the
/// shifted nodes' orders. (The C `av == v` assert at flat.c:25 is trivially
/// true — the arena keeps one `Vec`; the `v[n] = NULL` re-termination at
/// flat.c:37 is implicit in the `Vec` length.)
fn make_vn_slot(fg: &mut Fg, g: GId, r: i32, pos: i32) -> NId {
    let idx = (r - fg.graphs[g].minrank) as usize;
    let n = fg.graphs[g].rank[idx].n; // flat.c:26 (old count)
    {
        let rank = &mut fg.graphs[g].rank[idx];
        rank.v.resize(n + 1, NId::MAX); // gv_recalloc flat.c:26-27
        let v = &mut rank.v;
        let mut i = n;
        while i > pos as usize {
            // flat.c:30-33
            v[i] = v[i - 1];
            fg.nodes[v[i]].order += 1;
            i -= 1;
        }
    }
    let vn = virtual_node(fg, g); // flat.c:34 — also prepends to GD_nlist
    let rank = &mut fg.graphs[g].rank[idx];
    rank.v[pos as usize] = vn;
    rank.n = n + 1; // flat.c:37 (v[++n] = NULL)
    fg.nodes[vn].order = pos; // flat.c:35
    fg.nodes[vn].rank = r; // flat.c:36
    vn
}

/// `flat_node` (flat.c:136-182) — create the virtual label node for a
/// labeled, non-adjacent flat edge `e` on rank `rank(tail)-1`.
fn flat_node(fg: &mut Fg, e: EId) {
    if fg.edges[e].label.is_none() {
        return; // flat.c:145-146
    }
    let g = fg.root_g(); // flat.c:147 — dot_root(agtail(e))
    let r = fg.nodes[fg.edges[e].tail].rank; // flat.c:148

    let place = flat_limits(fg, g, e); // flat.c:150
    // flat.c:151-157 — grab ypos = LL.y of the label box *before* the slot
    // insertion possibly re-orders rank r-1.
    let ypos = {
        let prev = rank_row(fg, g, r - 1);
        match prev.v.first().copied() {
            Some(n) => fg.nodes[n].coord.y - prev.ht1,
            None => {
                // empty rank r-1: fall back to rank r (flat.c:154-156)
                let cur = rank_row(fg, g, r);
                let n = cur.v[0];
                fg.nodes[n].coord.y + cur.ht2 + fg.graphs[g].ranksep as f64
            }
        }
    };
    let vn = make_vn_slot(fg, g, r - 1, place); // flat.c:158
    let dimen = fg.labels[fg.edges[e].label.unwrap()].dimen;
    let dimen = if fg.graphs[g].rankdir.flip() {
        // flat.c:160-162 — GD_flip swaps the label dimensions
        PointF::new(dimen.y, dimen.x)
    } else {
        dimen
    };
    fg.nodes[vn].ht = dimen.y; // flat.c:163
    let h2 = fg.nodes[vn].ht / 2.0; // flat.c:164
    fg.nodes[vn].lw = dimen.x / 2.0; // flat.c:165
    fg.nodes[vn].rw = dimen.x / 2.0;
    fg.nodes[vn].label = fg.edges[e].label; // flat.c:166 — shared label record
    fg.nodes[vn].coord.y = ypos + h2; // flat.c:167 (.x comes from the simplex)
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    let ve = virtual_edge(fg, vn, t, Some(e)); // flat.c:168 — FLATORDER to tail
    fg.edges[ve].tail_port.p.x = -fg.nodes[vn].lw; // flat.c:169
    fg.edges[ve].head_port.p.x = fg.nodes[t].rw; // flat.c:170
    fg.edges[ve].edge_type = EdgeType::FlatOrder; // flat.c:171
    let ve = virtual_edge(fg, vn, h, Some(e)); // flat.c:172 — FLATORDER to head
    fg.edges[ve].tail_port.p.x = fg.nodes[vn].rw; // flat.c:173
    fg.edges[ve].head_port.p.x = fg.nodes[h].lw; // flat.c:174
    fg.edges[ve].edge_type = EdgeType::FlatOrder; // flat.c:175
    {
        // flat.c:177-180 — claim h2 above and below the rank r-1 centerline
        let row = rank_row_mut(fg, g, r - 1);
        if row.ht1 < h2 {
            row.ht1 = h2;
        }
        if row.ht2 < h2 {
            row.ht2 = h2;
        }
    }
    // flat.c:181 — ND_alg(vn) = e. The arena has no ND_alg field; the vnode
    // is identified structurally by `nd_alg` (two FLATORDER out-edges whose
    // ED_to_orig is e), reproducing the flat.c:131-135 invariant.
}

/// `abomination` (flat.c:184-202) — insert one new empty rank below rank 0
/// so a labeled non-adjacent flat edge on rank 0 has a rank −1 for its label
/// node. The C grows the rank array and rebases the pointer
/// (`GD_rank(g) = rptr + 1`, flat.c:193), keeping every existing rank at its
/// absolute index; the arena `rank` Vec is dense from `minrank`, so the same
/// result is an empty `Rank` inserted at index 0 plus `minrank -= 1`.
fn abomination(fg: &mut Fg, g: GId) {
    debug_assert_eq!(fg.graphs[g].minrank, 0); // flat.c:188
    let new_rank = super::model::Rank {
        n: 0,
        v: Vec::new(), // flat.c:196-197 (C callocs 2 sentinel slots)
        flat: None,    // flat.c:198
        ht1: 1.0,      // flat.c:199
        ht2: 1.0,
        pht1: 1.0, // flat.c:200
        pht2: 1.0,
        // flat.c:200 — pht1 = pht2 = 1; the arena has no pht fields (they
        // coincide with ht1/ht2 at every read — see position.rs).
        ..Default::default()
    };
    fg.graphs[g].rank.insert(0, new_rank);
    fg.graphs[g].minrank -= 1; // flat.c:201
}

/// `checkFlatAdjacent` (flat.c:208-238) — mark `ED_adjacent` on `e` and its
/// whole `ED_to_virt` chain when no NORMAL node and no labeled virtual node
/// (i.e. another flat edge's label node) sits strictly between the endpoints
/// on their rank.
fn check_flat_adjacent(fg: &mut Fg, e: EId) {
    let (tn, hn) = (fg.edges[e].tail, fg.edges[e].head);
    let (lo, hi) = if fg.nodes[tn].order < fg.nodes[hn].order {
        (fg.nodes[tn].order, fg.nodes[hn].order) // flat.c:217-220
    } else {
        (fg.nodes[hn].order, fg.nodes[tn].order) // flat.c:222-224
    };
    let r = fg.nodes[tn].rank;
    let root = fg.root_g(); // flat.c:225 — dot_root(tn)
    let rank_v = super::position::rank_nodes(fg, root, r);
    let mut i = lo + 1;
    while i < hi {
        let n = rank_v[i as usize];
        if (fg.nodes[n].node_type == NodeType::Virtual && fg.nodes[n].label.is_some())
            || fg.nodes[n].node_type == NodeType::Normal
        {
            break; // flat.c:228-230
        }
        i += 1;
    }
    if i == hi {
        // flat.c:232-237 — adjacent: mark e, then everything ED_to_vert reaches
        let mut cur = Some(e);
        while let Some(eid) = cur {
            fg.edges[eid].adjacent = true;
            cur = fg.edges[eid].to_virt;
        }
    }
}

/// `flat_edges` (flat.c:256-336) — public entry point. Returns the "reset"
/// flag: `true` iff any label node was created, i.e. the caller must re-run
/// `set_ycoords` (`position.c:133-134`).
pub fn flat_edges(fg: &mut Fg, g: GId) -> bool {
    // ---- Phase 1: adjacency marking (flat.c:261-272) ----------------------
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        if !fg.nodes[id].flat_out.is_empty() {
            // flat.c:262-266
            for &e in fg.nodes[id].flat_out.clone().iter() {
                check_flat_adjacent(fg, e);
            }
        }
        for j in 0..fg.nodes[id].other.len() {
            // flat.c:267-271
            let e = fg.nodes[id].other[j];
            if fg.nodes[fg.edges[e].tail].rank == fg.nodes[fg.edges[e].head].rank {
                check_flat_adjacent(fg, e);
            }
        }
    }

    // ---- Phase 2: maybe insert rank -1 (flat.c:274-292) -------------------
    // Gate: the rank-0 flat-order matrix (created by mincross
    // `flat_breakcycles`) exists, or the graph has clusters.
    let row0_flat = rank_row(fg, g, 0).flat.is_some();
    if row0_flat || !fg.graphs[g].clust.is_empty() {
        let mut found = false;
        'outer: for &n in super::position::rank_nodes(fg, g, 0).iter() {
            // flat.c:277-283 — ND_flat_in of each rank-0 node
            for &e in &fg.nodes[n].flat_in {
                if fg.edges[e].label.is_some() && !fg.edges[e].adjacent {
                    abomination(fg, g);
                    found = true;
                    break 'outer;
                }
            }
            // flat.c:284-290 — ND_other equivalents
            for &e in &fg.nodes[n].other {
                if fg.edges[e].label.is_some() && !fg.edges[e].adjacent {
                    abomination(fg, g);
                    found = true;
                    break 'outer;
                }
            }
        }
        let _ = found;
    }

    // flat.c:294 — rec_save_vlists(g) (mincross.c scope; not ported here —
    // it only matters for re-anchoring cluster rank vlists later).

    // ---- Phase 3: label nodes / ED_dist (flat.c:296-330) ------------------
    // (GD_flip(g) is consumed by `edge_dist` at read time.)
    let mut reset = false;
    let mut n = fg.graphs[g].nlist;
    while let Some(id) = n {
        n = fg.nodes[id].next;
        /* if n is the tail of any flat edge, one will be in flat_out */
        if !fg.nodes[id].flat_out.is_empty() {
            for i in 0..fg.nodes[id].flat_out.len() {
                let e = fg.nodes[id].flat_out[i];
                if let Some(lbl) = fg.edges[e].label {
                    if fg.edges[e].adjacent {
                        // flat.c:302 — ED_dist(e) = flip ? dimen.y : dimen.x.
                        // The arena has no ED_dist field; the value (and the
                        // flat.c:321 MAX-accumulation over equivalents) is
                        // recomputed on demand by `edge_dist`.
                        let _ = lbl;
                    } else {
                        reset = true;
                        flat_node(fg, e); // flat.c:305-306
                    }
                }
            }
            /* look for other flat edges with labels (flat.c:310-328) */
            for j in 0..fg.nodes[id].other.len() {
                let e = fg.nodes[id].other[j];
                if fg.nodes[fg.edges[e].tail].rank != fg.nodes[fg.edges[e].head].rank {
                    continue; // flat.c:313 — only truly flat
                }
                if fg.edges[e].tail == fg.edges[e].head {
                    continue; // flat.c:314 — skip loops
                }
                let mut le = e;
                while let Some(v) = fg.edges[le].to_virt {
                    le = v; // flat.c:316 — chase to the representative
                }
                fg.edges[e].adjacent = fg.edges[le].adjacent; // flat.c:317
                if fg.edges[e].label.is_some() {
                    if fg.edges[e].adjacent {
                        // flat.c:320-321 — ED_dist(le) = MAX(lw, ED_dist(le));
                        // recomputed by `edge_dist`.
                    } else {
                        reset = true;
                        flat_node(fg, e); // flat.c:324-326 — one node per edge
                    }
                }
            }
        }
    }
    if reset {
        // flat.c:332-334 — checkLabelOrder(g) + rec_reset_vlists(g) live in
        // mincross.c (mincross.c:297-326 / 930-953), another work package;
        // not called here.
    }
    reset // flat.c:335
}

/// Structural replacement for `ND_alg` (flat.c:131-135, read at
/// `position.c:277`). A flat-edge label node is a `VIRTUAL` node whose out
/// list is exactly the two `FLATORDER` edges made by [`flat_node`], both
/// carrying `ED_to_orig = e`; this returns that `e`.
///
/// The two edges live in `out` before `allocate_aux_edges`
/// (position.c:206-221) and in `save_out` afterwards; both states are
/// recognized.
pub fn nd_alg(fg: &Fg, n: NId) -> Option<EId> {
    if fg.nodes[n].node_type != NodeType::Virtual {
        return None;
    }
    let list: &[EId] = if !fg.nodes[n].save_out.is_empty() {
        &fg.nodes[n].save_out
    } else {
        &fg.nodes[n].out
    };
    if list.len() != 2 {
        return None;
    }
    let (e0, e1) = (list[0], list[1]);
    if fg.edges[e0].edge_type != EdgeType::FlatOrder
        || fg.edges[e1].edge_type != EdgeType::FlatOrder
    {
        return None;
    }
    fg.edges[e0].to_orig
}

/// `ED_dist` (flat.c:302, flat.c:321) — the largest label width among an
/// adjacent flat edge and its equivalents.
///
/// The C stores this double on the edge; the arena `DEdge` has no such
/// field, and the value is a pure function of the (immutable) label records
/// plus the `ND_other` equivalent edges, so it is recomputed at read time
/// (the only reader is `position.c:316`, `make_LR_constraints`).
///
/// Reproduces flat.c:302 (representative's own label width, adjacent case)
/// and flat.c:311-321 (MAX over labeled adjacent `ND_other` equivalents
/// whose `ED_to_virt` representative is `e`). flat.c:313's rank-equality
/// filter is subsumed here: any `ND_other` edge whose representative is a
/// flat edge is itself flat (class2 only routes flat multi-edge duplicates
/// through `merge_oneway` + `ND_other`), and self loops can never resolve to
/// a `flat_out` representative. `flip` is `GD_flip(g)` of the root graph.
pub fn edge_dist(fg: &Fg, e: EId, flip: bool) -> f64 {
    let label_width = |eid: EId| -> Option<f64> {
        fg.edges[eid].label.map(|l| {
            let d = fg.labels[l].dimen;
            if flip { d.y } else { d.x }
        })
    };
    // flat.c:302 — the representative's own width (0 when unlabeled)
    let mut dist = label_width(e).unwrap_or(0.0);
    let t = fg.edges[e].tail;
    for &f in &fg.nodes[t].other {
        // flat.c:314 — skip self loops (their representative is a loop, never e)
        if fg.edges[f].tail == fg.edges[f].head {
            continue;
        }
        // flat.c:316 — representative of f
        let mut le = f;
        while let Some(v) = fg.edges[le].to_virt {
            le = v;
        }
        if le != e {
            continue;
        }
        // flat.c:318-321 — labeled + adjacent equivalents widen the max
        if let Some(lw) = label_width(f) {
            if fg.edges[f].adjacent {
                dist = dist.max(lw);
            }
        }
    }
    dist
}
