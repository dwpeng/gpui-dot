//! `lib/dotgen/conc.c` — merge parallel edges with a common endpoint onto one
//! chain when `concentrate=true`.
//!
//! Run by `dot_position` right after the first `set_ycoords` (position.c:126-131):
//! the ranks and the within-rank order are known, but the auxiliary x-graph has
//! not been built yet, so collapsing a run of identical virtual nodes in a rank
//! removes the duplicate channels before the splines are routed (C prints one
//! edge per concentrator, so `a->b` three times draws a single curve).

use super::classes::{delete_fast_edge, delete_fast_node, merge_oneway, virtual_edge};
use super::dot_splines::portcmp;
use super::model::{EId, EdgeType, Fg, GId, NId, NodeType, RankType};
use super::position::rank_row_mut;

/// `NULL` slot sentinel (see `mincross::NO_NODE`).
const NO_NODE: NId = usize::MAX;

const UP: i32 = 0;
const DOWN: i32 = 1;

/// `samedir` (conc.c:22-37) — both chains run the same way (and neither is
/// flagged as having an opposing edge).
fn samedir(fg: &Fg, e: EId, f: EId) -> bool {
    let walk = |mut x: EId| -> Option<EId> {
        while fg.edges[x].edge_type != EdgeType::Normal {
            {
                let o = fg.edges[x].to_orig?;
                x = o
            }
        }
        Some(x)
    };
    let (Some(e0), Some(f0)) = (walk(e), walk(f)) else {
        return false;
    };
    if fg.edges[e0].conc_opp_flag || fg.edges[f0].conc_opp_flag {
        return false;
    }
    let er = fg.nodes[fg.edges[e0].tail].rank - fg.nodes[fg.edges[e0].head].rank;
    let fr = fg.nodes[fg.edges[f0].tail].rank - fg.nodes[fg.edges[f0].head].rank;
    fr * er > 0
}

/// `downcandidate` (conc.c:39-43).
fn downcandidate(fg: &Fg, v: NId) -> bool {
    fg.nodes[v].node_type == NodeType::Virtual
        && fg.nodes[v].in_.len() == 1
        && fg.nodes[v].out.len() == 1
        && fg.nodes[v].label.is_none()
}

/// `upcandidate` (conc.c:58-62).
fn upcandidate(fg: &Fg, v: NId) -> bool {
    fg.nodes[v].node_type == NodeType::Virtual
        && fg.nodes[v].out.len() == 1
        && fg.nodes[v].in_.len() == 1
        && fg.nodes[v].label.is_none()
}

/// `bothdowncandidates` (conc.c:45-56).
fn bothdowncandidates(fg: &Fg, u: NId, v: NId) -> bool {
    let e = fg.nodes[u].in_[0];
    let f = fg.nodes[v].in_[0];
    downcandidate(fg, v)
        && fg.edges[e].tail == fg.edges[f].tail
        && samedir(fg, e, f)
        && portcmp(&fg.edges[e].tail_port, &fg.edges[f].tail_port) == 0
}

/// `bothupcandidates` (conc.c:64-75).
fn bothupcandidates(fg: &Fg, u: NId, v: NId) -> bool {
    let e = fg.nodes[u].out[0];
    let f = fg.nodes[v].out[0];
    upcandidate(fg, v)
        && fg.edges[e].head == fg.edges[f].head
        && samedir(fg, e, f)
        && portcmp(&fg.edges[e].head_port, &fg.edges[f].head_port) == 0
}

/// `mergevirtual` (conc.c:77-127) — merge `row[lpos+1..=rpos]` into `row[lpos]`.
fn mergevirtual(fg: &mut Fg, g: GId, r: i32, lpos: usize, rpos: usize, dir: i32) {
    let left = {
        let row = rank_row_mut(fg, g, r);
        row.v[lpos]
    };
    for i in (lpos + 1)..=rpos {
        let right = rank_row_mut(fg, g, r).v[i];
        if dir == DOWN {
            while let Some(e) = fg.nodes[right].out.first().copied() {
                let ehead = fg.edges[e].head;
                let mut f = None;
                for &cand in fg.nodes[left].out.iter() {
                    if fg.edges[cand].head == ehead {
                        f = Some(cand);
                        break;
                    }
                }
                let f = match f {
                    Some(f) => f,
                    None => virtual_edge(fg, left, ehead, Some(e)),
                };
                while let Some(e0) = fg.nodes[right].in_.first().copied() {
                    merge_oneway(fg, e0, f);
                    delete_fast_edge(fg, e0);
                }
                delete_fast_edge(fg, e);
            }
        } else {
            while let Some(e) = fg.nodes[right].in_.first().copied() {
                let etail = fg.edges[e].tail;
                let mut f = None;
                for &cand in fg.nodes[left].in_.iter() {
                    if fg.edges[cand].tail == etail {
                        f = Some(cand);
                        break;
                    }
                }
                let f = match f {
                    Some(f) => f,
                    None => virtual_edge(fg, etail, left, Some(e)),
                };
                while let Some(e0) = fg.nodes[right].out.first().copied() {
                    merge_oneway(fg, e0, f);
                    delete_fast_edge(fg, e0);
                }
                delete_fast_edge(fg, e);
            }
        }
        debug_assert!(
            fg.nodes[right].in_.is_empty() && fg.nodes[right].out.is_empty(),
            "mergevirtual left edges on the merged node"
        );
        delete_fast_node(fg, g, right);
    }
    let mut k = lpos + 1;
    let slot = super::position::rank_row_index(fg, g, r);
    let n = fg.graphs[g].rank[slot].n;
    for i in (rpos + 1)..n {
        let node = rank_row_mut(fg, g, r).v[i];
        rank_row_mut(fg, g, r).v[k] = node;
        fg.nodes[node].order = k as i32;
        k += 1;
    }
    let row = rank_row_mut(fg, g, r);
    row.n = k;
    if k < row.v.len() {
        row.v[k] = NO_NODE;
    }
}

/// `infuse` (conc.c:129-137).
fn infuse(fg: &mut Fg, g: GId, n: NId) {
    let r = fg.nodes[n].rank as usize;
    let lead = fg.graphs[g].rankleader.get(r).copied().flatten();
    if (lead.is_none() || fg.nodes[lead.unwrap()].order > fg.nodes[n].order)
        && r < fg.graphs[g].rankleader.len()
    {
        fg.graphs[g].rankleader[r] = Some(n);
    }
}

/// `rebuild_vlists` (conc.c:139-200) — re-derive each cluster's rank slice from
/// the root's rows after the merges.
fn rebuild_vlists(fg: &mut Fg, g: GId) -> Result<(), i32> {
    for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
        let i = r as usize; // rankleader is indexed by absolute rank
        if i < fg.graphs[g].rankleader.len() {
            fg.graphs[g].rankleader[i] = None;
        }
    }
    super::rank::dot_scan_ranks(fg, g);
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        infuse(fg, g, n);
        for &e in fg.input_out[n].clone().iter() {
            let mut rep = e;
            while let Some(v) = fg.edges[rep].to_virt {
                rep = v;
            }
            while fg.nodes[fg.edges[rep].head].rank < fg.nodes[fg.edges[e].head].rank {
                let h = fg.edges[rep].head;
                infuse(fg, g, h);
                match fg.nodes[h].out.first().copied() {
                    Some(next) => rep = next,
                    None => break,
                }
            }
        }
    }
    let root = fg.root_g();
    for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
        let Some(lead) = fg.graphs[g].rankleader.get(r as usize).copied().flatten() else {
            return Err(-1);
        };
        let Some(&at) = fg.graphs[root].rank[super::position::rank_row_index(fg, root, r)]
            .v
            .get(fg.nodes[lead].order.max(0) as usize)
        else {
            return Err(-1);
        };
        if at != lead {
            return Err(-1);
        }
        let off = fg.nodes[lead].order.max(0) as usize;
        let n_total = fg.graphs[root].rank[super::position::rank_row_index(fg, root, r)].n;
        let root_row: Vec<NId> = fg.graphs[root].rank[super::position::rank_row_index(fg, root, r)]
            .v
            .clone();
        let mut maxi: i64 = -1;
        let mut row_nodes: Vec<NId> = Vec::new();
        for i in 0..(n_total.saturating_sub(off)) {
            let n = root_row[off + i];
            if n == NO_NODE {
                break;
            }
            row_nodes.push(n);
            if fg.nodes[n].node_type == NodeType::Normal {
                if g == root || fg.nodes[n].clust == Some(g) {
                    maxi = i as i64;
                } else {
                    break;
                }
            } else {
                // a virtual node of an edge of this cluster
                let mut e = fg.nodes[n].in_.first().copied();
                while let Some(cur) = e {
                    match fg.edges[cur].to_orig {
                        Some(o) if fg.edges[cur].edge_type != EdgeType::Normal => {
                            e = Some(o);
                        }
                        _ => break,
                    }
                }
                if let Some(e) = e {
                    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
                    let inside = |x: NId| g == root || fg.nodes[x].clust == Some(g);
                    if inside(t) && inside(h) {
                        maxi = i as i64;
                    }
                }
            }
        }
        let count = if maxi < 0 { 0 } else { maxi as usize + 1 };
        {
            let slot = super::position::rank_row_index(fg, g, r);
            let row = &mut fg.graphs[g].rank[slot];
            row.v[..count].copy_from_slice(&row_nodes[..count]);
            row.n = count;
        }
        let slot = super::position::rank_row_index(fg, g, r);
        fg.graphs[g].rank_offset[slot] = off as i32;
        fg.graphs[g].expanded = true;
    }
    for c in fg.graphs[g].clust.clone().iter() {
        rebuild_vlists(fg, *c)?;
    }
    Ok(())
}

/// `dot_concentrate` (conc.c:202-248) — the two passes (down on candidate
/// ranks, then up) plus the cluster vlist rebuild. Returns `-1` when a
/// cluster's vlist cannot be rebuilt (C warns and lets the caller decide).
pub fn dot_concentrate(fg: &mut Fg, g: GId) -> Result<(), i32> {
    if fg.graphs[g].maxrank - fg.graphs[g].minrank <= 1 {
        return Ok(());
    }
    let maxrank = fg.graphs[g].maxrank;
    // Downward pass: r is a candidate rank while the next rank has nodes.
    let mut r = 1;
    while r < maxrank + 1 {
        let slot = super::position::rank_row_index(fg, g, r);
        let next = super::position::rank_row_index(fg, g, r + 1);
        if fg.graphs[g].rank.get(next).map(|row| row.n).unwrap_or(0) == 0 {
            break;
        }
        let mut leftpos = 0usize;
        while leftpos < fg.graphs[g].rank[slot].n {
            let left = fg.graphs[g].rank[slot].v[leftpos];
            if !downcandidate(fg, left) {
                leftpos += 1;
                continue;
            }
            let mut rightpos = leftpos + 1;
            while rightpos < fg.graphs[g].rank[slot].n {
                let right = fg.graphs[g].rank[slot].v[rightpos];
                if !bothdowncandidates(fg, left, right) {
                    break;
                }
                rightpos += 1;
            }
            if rightpos - leftpos > 1 {
                mergevirtual(fg, g, r, leftpos, rightpos - 1, DOWN);
            }
            leftpos += 1;
        }
        r += 1;
    }
    // Upward pass: `r` continues from the down pass (C: `while (r > 0)`).
    while r > 0 {
        let slot = super::position::rank_row_index(fg, g, r);
        let mut leftpos = 0usize;
        while leftpos < fg.graphs[g].rank[slot].n {
            let left = fg.graphs[g].rank[slot].v[leftpos];
            if !upcandidate(fg, left) {
                leftpos += 1;
                continue;
            }
            let mut rightpos = leftpos + 1;
            while rightpos < fg.graphs[g].rank[slot].n {
                let right = fg.graphs[g].rank[slot].v[rightpos];
                if !bothupcandidates(fg, left, right) {
                    break;
                }
                rightpos += 1;
            }
            if rightpos - leftpos > 1 {
                mergevirtual(fg, g, r, leftpos, rightpos - 1, UP);
            }
            leftpos += 1;
        }
        r -= 1;
    }
    for c in fg.graphs[g].clust.clone().iter() {
        if rebuild_vlists(fg, *c).is_err() {
            return Err(-1);
        }
    }
    let _ = RankType::Normal;
    Ok(())
}
