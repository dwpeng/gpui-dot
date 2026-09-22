//! `lib/dotgen/cluster.c` — the cluster half of the `dot` pipeline.
//!
//! A cluster is first *collapsed* during the rank phase ([`super::rank`]'s
//! `collapse_cluster` + [`super::classes`]' `build_skeleton`): its members are
//! unioned onto a leader and a chain of `CLUSTER` virtual nodes (the cluster's
//! *skeleton*) spans its ranks. `mincross` then expands each cluster once via
//! `MinCross::expand_cluster`, which drives this module:
//!
//! ```text
//! class2(subg) → allocate_ranks(subg) → build_ranks(subg, 0)
//!              → merge_ranks(subg)     ← here
//!              → interclexp(subg)      ← here
//!              → remove_rankleaders(subg) ← here
//! ```
//!
//! **Aliasing model.** C re-points `GD_rank(subg)[r].v` into the root's row
//! (`GD_rank(root)[r].v + ipos`) in `merge_ranks`, so cluster and root share
//! the ordering storage and every later swap through either view is visible
//! to both. Rust has no aliasing `Vec`, so this port keeps a *copy* in
//! `DGraph.rank`, records the slice start in `DGraph.rank_offset[r]`, and
//! refreshes the copies whenever the root's row changes
//! ([`refresh_expanded_clusters`], called from `MinCross::exchange` and
//! `MinCross::restore_best`). Reads then agree with C at every point.

use super::classes::{
    delete_fast_edge, delete_fast_node, fast_node, find_fast_edge, find_flat_edge, flat_edge,
    merge_chain, merge_oneway, mergeable, other_edge, ports_eq, safe_other_edge, virtual_edge,
    virtual_node,
};
use super::model::{EdgeType, EId, Fg, GId, NId, NodeType};
use super::position::rank_row_index;

/// `NULL` slot sentinel (`mincross::NO_NODE`).
const NO_NODE: NId = usize::MAX;

/// The root row slot where the cluster's slice for rank `r` starts, if the
/// cluster is expanded and spans `r`.
fn row_offset(fg: &Fg, g: GId, r: i32) -> Option<usize> {
    let dg = &fg.graphs[g];
    if !dg.expanded || r < dg.minrank || r > dg.maxrank {
        return None;
    }
    let i = rank_row_index(fg, g, r);
    let off = *dg.rank_offset.get(i)?;
    (off >= 0).then_some(off as usize)
}

/// Mirrors a write to the root's row `r` into every expanded cluster whose
/// slice covers it — the Rust stand-in for C's pointer aliasing.
pub fn refresh_expanded_clusters(fg: &mut Fg, r: i32) {
    let root = fg.root_g();
    let root_slot = rank_row_index(fg, root, r);
    if root_slot >= fg.graphs[root].rank.len() {
        return;
    }
    for g in 1..fg.graphs.len() {
        let Some(ipos) = row_offset(fg, g, r) else {
            continue;
        };
        let slot = rank_row_index(fg, g, r);
        let d = fg.graphs[g].rank[slot].n;
        if d == 0 {
            continue;
        }
        let src: Vec<NId> = fg.graphs[root].rank[root_slot].v.clone();
        let row = &mut fg.graphs[g].rank[slot].v;
        for i in 0..d {
            if let Some(&v) = src.get(ipos + i) {
                row[i] = v;
            }
        }
    }
}

/// `make_slots` (cluster.c:29-51) — open `d - 1` free slots at `pos` in the
/// root's row `r` (or close `d - 1` when `d <= 0`), shifting the nodes right
/// and renumbering their `order`.
fn make_slots(fg: &mut Fg, root: GId, r: i32, pos: usize, d: i32) {
    let slot = rank_row_index(fg, root, r);
    let n = fg.graphs[root].rank[slot].n;
    let v = fg.graphs[root].rank[slot].v.clone();
    let root_minrank = fg.graphs[root].minrank;
    if d <= 0 {
        let dd = (-d) as usize;
        let mut newv = v.clone();
        for i in (pos - dd + 1)..n {
            let node = v[i];
            newv[i - dd] = node;
            fg.nodes[node].order = (i - dd) as i32;
        }
        for i in (n - dd + 1)..n {
            newv[i] = NO_NODE;
        }
        newv.truncate(n.saturating_sub(dd) + 1);
        fg.graphs[root].rank[slot].v = newv;
        fg.graphs[root].rank[slot].n = (n as i32 + d - 1).max(0) as usize;
        let _ = root_minrank;
        return;
    }
    let du = d as usize;
    let mut newv = v.clone();
    for _ in 1..du {
        newv.insert(pos + 1, NO_NODE);
    }
    for i in (pos + 1)..n {
        let node = v[i];
        fg.nodes[node].order = (i + du - 1) as i32;
    }
    fg.graphs[root].rank[slot].n = n + du - 1;
    let new_n = fg.graphs[root].rank[slot].n;
    newv.truncate(new_n + 1);
    while newv.len() <= new_n {
        newv.push(NO_NODE);
    }
    newv[new_n] = NO_NODE;
    fg.graphs[root].rank[slot].v = newv;
}

/// `clone_vn` (cluster.c:53-66) — duplicate a virtual node one slot to its
/// right in the root's row.
fn clone_vn(fg: &mut Fg, g: GId, vn: NId) -> NId {
    let r = fg.nodes[vn].rank;
    let order = fg.nodes[vn].order as usize;
    make_slots(fg, g, r, order, 2);
    let (lw, rw) = (fg.nodes[vn].lw, fg.nodes[vn].rw);
    let rv = virtual_node(fg, g);
    fg.nodes[rv].lw = lw;
    fg.nodes[rv].rw = rw;
    fg.nodes[rv].rank = r;
    fg.nodes[rv].order = order as i32 + 1;
    let slot = rank_row_index(fg, g, r);
    fg.graphs[g].rank[slot].v[order + 1] = rv;
    rv
}

/// `map_interclust_node` (cluster.c:21-27) — an unexpanded cluster's member is
/// represented by its rank leader (the skeleton node).
fn map_interclust_node(fg: &Fg, n: NId) -> NId {
    match fg.nodes[n].clust {
        Some(c) if !fg.graphs[c].expanded => {
            let r = fg.nodes[n].rank.max(0) as usize;
            fg.graphs[c]
                .rankleader
                .get(r)
                .copied()
                .flatten()
                .unwrap_or(n)
        }
        _ => n,
    }
}

/// `map_path` (cluster.c:68-131) — rebuild `orig`'s virtual chain between
/// `from` and `to` after the cluster expansion changed the fast graph.
fn map_path(fg: &mut Fg, from: NId, to: NId, orig: EId, mut ve: EId, etype: EdgeType) {
    debug_assert!(fg.nodes[from].rank < fg.nodes[to].rank);
    if fg.edges[ve].tail == from && fg.edges[ve].head == to {
        return;
    }
    if fg.edges[ve].count > 1 {
        fg.edges[orig].to_virt = None;
        if fg.nodes[to].rank - fg.nodes[from].rank == 1 {
            if let Some(e) = find_fast_edge(fg, from, to) {
                if ports_eq(fg, orig, e) {
                    merge_oneway(fg, orig, e);
                    if fg.nodes[from].node_type == NodeType::Normal
                        && fg.nodes[to].node_type == NodeType::Normal
                    {
                        other_edge(fg, orig);
                    }
                    return;
                }
            }
        }
        let mut u = from;
        let mut r = fg.nodes[from].rank;
        while r < fg.nodes[to].rank {
            let v = if r < fg.nodes[to].rank - 1 {
                let head = fg.edges[ve].head;
                clone_vn(fg, fg.root_g(), head)
            } else {
                to
            };
            let e = virtual_edge(fg, u, v, Some(orig));
            fg.edges[e].edge_type = etype;
            u = v;
            fg.edges[ve].count -= 1;
            let next = fg.nodes[fg.edges[ve].head].out[0];
            ve = next;
            r += 1;
        }
        return;
    }
    if fg.nodes[to].rank - fg.nodes[from].rank == 1 {
        let found = find_fast_edge(fg, from, to).filter(|&e| ports_eq(fg, orig, e));
        match found {
            Some(ve2) => {
                fg.edges[orig].to_virt = Some(ve2);
                fg.edges[ve2].edge_type = etype;
                fg.edges[ve2].count += 1;
                if fg.nodes[from].node_type == NodeType::Normal
                    && fg.nodes[to].node_type == NodeType::Normal
                {
                    other_edge(fg, orig);
                }
            }
            None => {
                fg.edges[orig].to_virt = None;
                let ve2 = virtual_edge(fg, from, to, Some(orig));
                fg.edges[ve2].edge_type = etype;
            }
        }
        return;
    }
    let mut e;
    if fg.edges[ve].tail != from {
        fg.edges[orig].to_virt = None;
        let head = fg.edges[ve].head;
        e = virtual_edge(fg, from, head, Some(orig));
        fg.edges[orig].to_virt = Some(e);
        delete_fast_edge(fg, ve);
    } else {
        e = ve;
    }
    while fg.nodes[fg.edges[e].head].rank != fg.nodes[to].rank {
        e = fg.nodes[fg.edges[e].head].out[0];
    }
    if fg.edges[e].head != to {
        let stale = e;
        let tail = fg.edges[e].tail;
        e = virtual_edge(fg, tail, to, Some(orig));
        fg.edges[e].edge_type = etype;
        delete_fast_edge(fg, stale);
    }
}

/// `make_interclust_chain` (cluster.c:140-150).
fn make_interclust_chain(fg: &mut Fg, from: NId, to: NId, orig: EId) {
    let u = map_interclust_node(fg, from);
    let v = map_interclust_node(fg, to);
    let newtype = if u == from && v == to {
        EdgeType::Virtual
    } else {
        EdgeType::ClusterEdge
    };
    let ve = fg.edges[orig]
        .to_virt
        .expect("inter-cluster edge has a virtual chain");
    map_path(fg, u, v, orig, ve, newtype);
}

/// `interclexp` (cluster.c:152-213) — attach and install the edges that leave
/// the expanded cluster (`class2` for interclust edges).
pub fn interclexp(fg: &mut Fg, subg: GId) {
    let g = fg.root_g();
    for &n in fg.graphs[subg].nodes_order.clone().iter() {
        let mut prev: Option<EId> = None;
        // `agfstedge(root, n)`: n's out-edges in declaration order, then its
        // in-edges (`agnxtedge` switches lists when the out list runs out).
        let mut incident: Vec<EId> = fg.input_out[n].clone();
        incident.extend(fg.input_in[n].iter().copied());
        for &e in incident.iter() {
            // AGMKOUT(e): our edges are always stored in their declared
            // direction, so the edge is already canonical.
            if fg.graphs[subg].owns_input_edge(fg, e) {
                continue; // agcontains(subg, e) — internal to the cluster
            }
            if mergeable(fg, prev, Some(e)) {
                let p = prev.expect("mergeable pair");
                if fg.nodes[fg.edges[e].tail].rank == fg.nodes[fg.edges[e].head].rank {
                    fg.edges[e].to_virt = fg.edges[p].to_virt;
                } else {
                    fg.edges[e].to_virt = None;
                }
                let Some(pv) = fg.edges[p].to_virt else {
                    continue; // internal edge
                };
                fg.edges[e].to_virt = None;
                merge_chain(fg, subg, e, pv, false);
                safe_other_edge(fg, e);
                continue;
            }
            // flat edges
            if fg.nodes[fg.edges[e].tail].rank == fg.nodes[fg.edges[e].head].rank {
                let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
                match find_flat_edge(fg, t, h) {
                    None => {
                        if fg.edges[e].to_virt.is_none() {
                            flat_edge(fg, g, e);
                        }
                        prev = Some(e);
                    }
                    Some(fe) => {
                        if e != fe {
                            safe_other_edge(fg, e);
                            if fg.edges[e].to_virt.is_none() {
                                merge_oneway(fg, e, fe);
                            }
                        }
                    }
                }
                continue;
            }
            // forward and backward edges
            let (from, to) = if fg.nodes[fg.edges[e].head].rank > fg.nodes[fg.edges[e].tail].rank {
                (fg.edges[e].tail, fg.edges[e].head)
            } else {
                (fg.edges[e].head, fg.edges[e].tail)
            };
            make_interclust_chain(fg, from, to, e);
            prev = Some(e);
        }
    }
}

/// `merge_ranks` (cluster.c:215-243) — splice the cluster's ranked nodes into
/// the root's rows, record the slice offsets and mark the cluster expanded.
pub fn merge_ranks(fg: &mut Fg, subg: GId) {
    let root = fg.root_g();
    let minrank = fg.graphs[subg].minrank;
    let maxrank = fg.graphs[subg].maxrank;
    if minrank > 0 {
        let i = rank_row_index(fg, root, minrank - 1);
        fg.graphs[root].rank[i].valid = false;
    }
    fg.graphs[subg].rank_offset = vec![-1; fg.graphs[subg].rank.len()];
    for r in minrank..=maxrank {
        let slot = rank_row_index(fg, subg, r);
        let d = fg.graphs[subg].rank[slot].n;
        let leader = fg.graphs[subg].rankleader[r as usize].expect("cluster rankleader");
        let ipos = fg.nodes[leader].order as usize;
        make_slots(fg, root, r, ipos, d as i32);
        for i in 0..d {
            let v = fg.graphs[subg].rank[slot].v[i];
            let root_slot = rank_row_index(fg, root, r);
            fg.graphs[root].rank[root_slot].v[ipos + i] = v;
            fg.nodes[v].order = (ipos + i) as i32;
            delete_fast_node(fg, subg, v);
            fast_node(fg, root, v);
        }
        // Keep the port's copy of the slice in step with the root's row.
        let root_slot = rank_row_index(fg, root, r);
        let src: Vec<NId> = fg.graphs[root].rank[root_slot].v.clone();
        {
            let row = &mut fg.graphs[subg].rank[slot];
            for i in 0..d {
                row.v[i] = src[ipos + i];
            }
            row.n = d;
        }
        fg.graphs[subg].rank[slot].v.truncate(d + 1);
        if fg.graphs[subg].rank[slot].v.len() <= d {
            fg.graphs[subg].rank[slot].v.push(NO_NODE);
        }
        fg.graphs[subg].rank_offset[slot] = ipos as i32;
        fg.graphs[root].rank[root_slot].valid = false;
    }
    // C L241-242: `if (r < GD_maxrank(root)) GD_rank(root)[r].valid = false;`
    let after = maxrank + 1;
    if after < fg.graphs[root].maxrank {
        let i = rank_row_index(fg, root, after);
        fg.graphs[root].rank[i].valid = false;
    }
    fg.graphs[subg].expanded = true;
}

/// `remove_rankleaders` (cluster.c:245-270) — delete the cluster's skeleton
/// chain from the fast graph; the cluster is expanded, so its ranks now hold
/// the real nodes.
pub fn remove_rankleaders(fg: &mut Fg, g: GId) {
    let minrank = fg.graphs[g].minrank;
    let maxrank = fg.graphs[g].maxrank;
    for r in minrank..=maxrank {
        let Some(v) = fg.graphs[g].rankleader.get(r as usize).copied().flatten() else {
            continue;
        };
        while let Some(e) = fg.nodes[v].out.first().copied() {
            delete_fast_edge(fg, e);
        }
        while let Some(e) = fg.nodes[v].in_.first().copied() {
            delete_fast_edge(fg, e);
        }
        delete_fast_node(fg, fg.root_g(), v);
        fg.nodes[v].in_.clear();
        fg.nodes[v].out.clear();
        fg.graphs[g].rankleader[r as usize] = None;
    }
}
