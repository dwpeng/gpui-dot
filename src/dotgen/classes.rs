//! Fast-graph operations — a direct port of `lib/dotgen/fastgr.c` plus the
//! classification passes `acyclic.c`, `class1.c`, `class2.c` and the cluster
//! helpers `mark_clusters` / `build_skeleton` from `cluster.c`.
//!
//! Iteration orders matter: `class1`/`class2` walk the cgraph dictionaries
//! (`agfstnode` × `agfstout` — input order), *not* the fast node list; the
//! port mirrors that via `DGraph::nodes_order` and `Fg::input_out`.

use super::model::{CL_CROSS, EId, EdgeType, Fg, GId, MC_SCALE, NId, RankType, SLACKNODE, VIRTUAL};

/// `find_fast_edge` — searches the fast out/in lists.
pub fn find_fast_edge(fg: &Fg, u: NId, v: NId) -> Option<EId> {
    let out = &fg.nodes[u].out;
    let inn = &fg.nodes[v].in_;
    if out.is_empty() || inn.is_empty() {
        return None;
    }
    if out.len() < inn.len() {
        out.iter().copied().find(|&e| fg.edges[e].head == v)
    } else {
        inn.iter().copied().find(|&e| fg.edges[e].tail == u)
    }
}

/// `find_flat_edge`.
pub fn find_flat_edge(fg: &Fg, u: NId, v: NId) -> Option<EId> {
    let out = &fg.nodes[u].flat_out;
    let inn = &fg.nodes[v].flat_in;
    if out.is_empty() || inn.is_empty() {
        return None;
    }
    if out.len() < inn.len() {
        out.iter().copied().find(|&e| fg.edges[e].head == v)
    } else {
        inn.iter().copied().find(|&e| fg.edges[e].tail == u)
    }
}

/// `fast_edge` — attach e to both endpoint lists.
pub fn fast_edge(fg: &mut Fg, e: EId) -> EId {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    fg.nodes[t].out.push(e);
    fg.nodes[h].in_.push(e);
    e
}

/// `zapinlist` — remove e from list, filling the hole with the last member
/// (order-destroying, exactly like fastgr.c).
pub fn zapinlist(list: &mut Vec<EId>, e: EId) {
    if let Some(i) = list.iter().position(|&x| x == e) {
        let last = list.len() - 1;
        list.swap(i, last);
        list.pop();
    }
}

/// `delete_fast_edge` — disconnect e from the graph (edge object retained).
pub fn delete_fast_edge(fg: &mut Fg, e: EId) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    zapinlist(&mut fg.nodes[t].out, e);
    zapinlist(&mut fg.nodes[h].in_, e);
}

/// `other_edge`.
pub fn other_edge(fg: &mut Fg, e: EId) {
    let t = fg.edges[e].tail;
    fg.nodes[t].other.push(e);
}

/// `safe_other_edge` — append only if not already a member.
pub fn safe_other_edge(fg: &mut Fg, e: EId) {
    let t = fg.edges[e].tail;
    if !fg.nodes[t].other.contains(&e) {
        fg.nodes[t].other.push(e);
    }
}

/// `new_virtual_edge` — create the edge object; `orig` supplies weights and
/// ports and receives the `ED_to_virt` backlink when it has none yet.
pub fn new_virtual_edge(fg: &mut Fg, u: NId, v: NId, orig: Option<EId>) -> EId {
    let seq = fg.seq;
    fg.seq += 1;
    fg.edges.push(super::model::DEdge {
        tail: u,
        head: v,
        edge_type: EdgeType::Virtual,
        seq,
        ..Default::default()
    });
    let e = fg.edges.len() - 1;
    if let Some(orig) = orig {
        let (ot, oh) = (fg.edges[orig].tail, fg.edges[orig].head);
        let (otp, ohp, oseq, ocount, oxp, ow, oml, oto_virt) = {
            let o = &fg.edges[orig];
            (
                o.tail_port,
                o.head_port,
                o.seq,
                o.count,
                o.xpenalty,
                o.weight,
                o.minlen,
                o.to_virt,
            )
        };
        {
            let te = &mut fg.edges[e];
            te.seq = oseq;
            te.count = ocount;
            te.xpenalty = oxp;
            te.weight = ow;
            te.minlen = oml;
            if u == ot {
                te.tail_port = otp;
            } else if u == oh {
                te.tail_port = ohp;
            }
            if v == oh {
                te.head_port = ohp;
            } else if v == ot {
                te.head_port = otp;
            }
            te.to_orig = Some(orig);
        }
        if oto_virt.is_none() {
            fg.edges[orig].to_virt = Some(e);
        }
    } else {
        let te = &mut fg.edges[e];
        te.weight = 1;
        te.xpenalty = 1;
        te.count = 1;
        te.minlen = 1;
    }
    e
}

/// `virtual_edge` — create and install.
pub fn virtual_edge(fg: &mut Fg, u: NId, v: NId, orig: Option<EId>) -> EId {
    let e = new_virtual_edge(fg, u, v, orig);
    fast_edge(fg, e)
}

/// `fast_node` — prepend n to g's fast node list.
pub fn fast_node(fg: &mut Fg, g: GId, n: NId) {
    let head = fg.graphs[g].nlist;
    fg.nodes[n].next = head;
    if let Some(h) = head {
        fg.nodes[h].prev = Some(n);
    }
    fg.nodes[n].prev = None;
    fg.graphs[g].nlist = Some(n);
}

/// `delete_fast_node` — unlink n from g's fast node list.
pub fn delete_fast_node(fg: &mut Fg, g: GId, n: NId) {
    let (p, nx) = (fg.nodes[n].prev, fg.nodes[n].next);
    if let Some(nx) = nx {
        fg.nodes[nx].prev = p;
    }
    match p {
        Some(p) => fg.nodes[p].next = nx,
        None => fg.graphs[g].nlist = nx,
    }
}

/// `virtual_node` — fresh dummy with lw=rw=ht=1.
pub fn virtual_node(fg: &mut Fg, g: GId) -> NId {
    let n = super::model::DNode {
        name: format!("%virtual{}", fg.nodes.len()),
        node_type: VIRTUAL,
        lw: 1.0,
        rw: 1.0,
        ht: 1.0,
        uf_size: 1,
        ..Default::default()
    };
    fg.nodes.push(n);
    let n = fg.nodes.len() - 1;
    fast_node(fg, g, n);
    n
}

/// `flat_edge` — attach e to the flat lists; flags both the root and the
/// current graph (C: `GD_has_flat_edges(dot_root(g)) = GD_has_flat_edges(g) = true`).
pub fn flat_edge(fg: &mut Fg, g: GId, e: EId) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    fg.nodes[t].flat_out.push(e);
    fg.nodes[h].flat_in.push(e);
    fg.graphs[0].has_flat_edges = true;
    fg.graphs[g].has_flat_edges = true;
}

/// `delete_flat_edge`.
pub fn delete_flat_edge(fg: &mut Fg, e: EId) {
    let orig = fg.edges[e].to_orig;
    if let Some(orig) = orig
        && fg.edges[orig].to_virt == Some(e)
    {
        fg.edges[orig].to_virt = None;
    }
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    zapinlist(&mut fg.nodes[t].flat_out, e);
    zapinlist(&mut fg.nodes[h].flat_in, e);
}

/// `basic_merge` (fastgr.c).
fn basic_merge(fg: &mut Fg, e: EId, rep: EId) {
    if fg.edges[rep].minlen < fg.edges[e].minlen {
        fg.edges[rep].minlen = fg.edges[e].minlen;
    }
    let (count, xpenalty, weight) = (fg.edges[e].count, fg.edges[e].xpenalty, fg.edges[e].weight);
    let mut rep = Some(rep);
    while let Some(r) = rep {
        fg.edges[r].count += count;
        fg.edges[r].xpenalty += xpenalty;
        fg.edges[r].weight += weight;
        rep = fg.edges[r].to_virt;
    }
}

/// `merge_oneway` — merge e into representative rep.
pub fn merge_oneway(fg: &mut Fg, e: EId, rep: EId) {
    if Some(rep) == fg.edges[e].to_virt || Some(e) == fg.edges[rep].to_virt {
        // "merge_oneway glitch" warning in C
        return;
    }
    debug_assert!(fg.edges[e].to_virt.is_none(), "merge_oneway: to_virt set");
    fg.edges[e].to_virt = Some(rep);
    basic_merge(fg, e, rep);
}

/// `merge_chain` (class2.c) — merge a long edge e into the chain starting at
/// f, widening every virtual node on the way.
pub fn merge_chain(fg: &mut Fg, g: GId, e: EId, f: EId, update_count: bool) {
    debug_assert!(fg.edges[e].to_virt.is_none());
    let (et, eh) = (fg.edges[e].tail, fg.edges[e].head);
    let lastrank = fg.nodes[et].rank.max(fg.nodes[eh].rank);
    fg.edges[e].to_virt = Some(f);
    let (e_count, e_xpenalty, e_weight) =
        (fg.edges[e].count, fg.edges[e].xpenalty, fg.edges[e].weight);
    let mut rep = Some(f);
    while let Some(r) = rep {
        if update_count {
            fg.edges[r].count += e_count;
        }
        fg.edges[r].xpenalty += e_xpenalty;
        fg.edges[r].weight += e_weight;
        let head = fg.edges[r].head;
        if fg.nodes[head].rank == lastrank {
            break;
        }
        incr_width(fg, g, head);
        rep = fg.nodes[head].out.first().copied();
    }
}

/// `ports_eq` (position.c:1030) — both ports undefined, or both defined with
/// identical positions.
pub fn ports_eq(fg: &Fg, e: EId, f: EId) -> bool {
    let (et, eh) = (&fg.edges[e].tail_port, &fg.edges[e].head_port);
    let (ft, fh) = (&fg.edges[f].tail_port, &fg.edges[f].head_port);
    port_eq(et, ft) && port_eq(eh, fh)
}

fn port_eq(a: &super::model::Port, b: &super::model::Port) -> bool {
    if a.defined != b.defined {
        return false;
    }
    if a.defined {
        (a.p.x - b.p.x).abs() < 1e-9 && (a.p.y - b.p.y).abs() < 1e-9
    } else {
        true
    }
}

/// `mergeable` (class2.c) — same endpoints, same label record, same ports.
pub fn mergeable(fg: &Fg, e: Option<EId>, f: Option<EId>) -> bool {
    match (e, f) {
        (Some(e), Some(f)) => {
            fg.edges[e].tail == fg.edges[f].tail
                && fg.edges[e].head == fg.edges[f].head
                && fg.edges[e].label == fg.edges[f].label
                && ports_eq(fg, e, f)
        }
        _ => false,
    }
}

/// `incr_width` — widen a virtual node for merged chains.
pub fn incr_width(fg: &mut Fg, g: GId, v: NId) {
    let width = fg.graphs[g].nodesep as f64 / 2.0;
    fg.nodes[v].lw += width;
    fg.nodes[v].rw += width;
}

// ---------------------------------------------------------------------------
// acyclic.c
// ---------------------------------------------------------------------------

/// `reverse_edge` — turn e around inside the fast graph.
pub fn reverse_edge(fg: &mut Fg, e: EId) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    delete_fast_edge(fg, e);
    if let Some(f) = find_fast_edge(fg, h, t) {
        merge_oneway(fg, e, f);
    } else {
        virtual_edge(fg, h, t, Some(e));
    }
}

fn dfs_acyclic(fg: &mut Fg, n: NId, stamp: usize) {
    if fg.nodes[n].mark == stamp {
        return;
    }
    fg.nodes[n].mark = stamp;
    fg.nodes[n].onstack = true;
    let mut i = 0;
    while i < fg.nodes[n].out.len() {
        let e = fg.nodes[n].out[i];
        let w = fg.edges[e].head;
        if fg.nodes[w].onstack {
            reverse_edge(fg, e);
            // the reversed edge left this node's out list; a replacement
            // slid into slot i — re-examine it (C does i--).
        } else {
            if fg.nodes[w].mark != stamp {
                dfs_acyclic(fg, w, stamp);
            }
            i += 1;
        }
    }
    fg.nodes[n].onstack = false;
}

/// `acyclic` — break cycles per component.
pub fn acyclic(fg: &mut Fg, g: GId) {
    for &comp in fg.graphs[g].comp.clone().iter() {
        let stamp = fg.stamp_next;
        fg.stamp_next += 1;
        let mut next = Some(comp);
        while let Some(n) = next {
            dfs_acyclic(fg, n, stamp);
            next = fg.nodes[n].next;
        }
    }
}

// ---------------------------------------------------------------------------
// cluster.c helpers needed by the classification passes
// ---------------------------------------------------------------------------

/// `mark_clusters` — set ND_clust for every node directly in one of g's
/// child clusters, and mark the virtual chains of intra-cluster edges.
pub fn mark_clusters(fg: &mut Fg, g: GId) {
    // remove sub-cluster marks below this level (input nodes of g only)
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        if fg.nodes[n].ranktype == RankType::Cluster {
            fg.uf_singleton(n);
        }
        fg.nodes[n].clust = None;
    }

    for &clust in fg.graphs[g].clust.clone().iter() {
        for &n in fg.graphs[clust].nodes_order.clone().iter() {
            if fg.nodes[n].ranktype != RankType::Normal {
                continue; // warning in C: already in a rankset
            }
            let leader = fg.graphs[clust].leader.expect("cluster leader");
            fg.uf_setname(n, leader);
            fg.nodes[n].clust = Some(clust);
            fg.nodes[n].ranktype = RankType::Cluster;

            // mark the vnodes of edges declared in the cluster
            for orig in fg.input_out[n].clone() {
                if orig >= fg.n_orig_edges || !fg.graphs[clust].owns_input_edge(fg, orig) {
                    continue;
                }
                if let Some(mut e) = fg.edges[orig].to_virt {
                    while fg.nodes[fg.edges[e].head].node_type == VIRTUAL {
                        fg.nodes[fg.edges[e].head].clust = Some(clust);
                        match fg.nodes[fg.edges[e].head].out.first().copied() {
                            Some(nx) => e = nx,
                            None => break,
                        }
                    }
                }
            }
        }
    }
}

/// `build_skeleton` — chain of cluster rank leaders across the cluster's
/// rank span (cluster.c).
pub fn build_skeleton(fg: &mut Fg, g: GId, subg: GId) {
    let mut prev: Option<NId> = None;
    let minrank = fg.graphs[subg].minrank;
    let maxrank = fg.graphs[subg].maxrank;
    fg.graphs[subg].rankleader = vec![None; (maxrank as usize) + 2];
    for r in minrank..=maxrank {
        let v = virtual_node(fg, g);
        fg.nodes[v].rank = r;
        fg.nodes[v].ranktype = RankType::Cluster;
        fg.nodes[v].clust = Some(subg);
        if let Some(prev) = prev {
            let e = virtual_edge(fg, prev, v, None);
            fg.edges[e].xpenalty *= CL_CROSS;
        }
        fg.graphs[subg].rankleader[r as usize] = Some(v);
        prev = Some(v);
    }

    // set the counts on the virtual edges of the cluster skeleton
    for &v in fg.graphs[subg].nodes_order.clone().iter() {
        let rank = fg.nodes[v].rank;
        let rl = fg.graphs[subg].rankleader[rank as usize].expect("rankleader");
        fg.nodes[rl].uf_size += 1;
        for e in fg.input_out[v].clone() {
            if !fg.graphs[subg].owns_input_edge(fg, e) {
                continue;
            }
            let (tr, hr) = (
                fg.nodes[fg.edges[e].tail].rank,
                fg.nodes[fg.edges[e].head].rank,
            );
            let mut r = tr;
            while r < hr {
                let first = fg.nodes[rl].out.first().copied().expect("skeleton edge");
                fg.edges[first].count += 1;
                r += 1;
            }
        }
    }
    for r in minrank..=maxrank {
        let rl = fg.graphs[subg].rankleader[r as usize].expect("rankleader");
        if fg.nodes[rl].uf_size > 1 {
            fg.nodes[rl].uf_size -= 1;
        }
    }
}

// ---------------------------------------------------------------------------
// class1.c
// ---------------------------------------------------------------------------

/// `nonconstraint_edge` — `constraint=false` edges are ignored for ranking.
pub fn nonconstraint_edge(fg: &Fg, e: EId) -> bool {
    match fg.edges[e].input {
        Some(i) => fg.nonconstraint[i],
        None => false,
    }
}

/// `make_aux_edge` (rank.c) — plain weighted aux edge.
pub fn make_aux_edge(fg: &mut Fg, u: NId, v: NId, len: i32, wt: i32) -> EId {
    let e = virtual_edge(fg, u, v, None);
    fg.edges[e].minlen = len;
    fg.edges[e].weight = wt;
    e
}

/// `interclust1` — inter-cluster edges become a slack node + two aux edges.
fn interclust1(fg: &mut Fg, g: GId, t: NId, h: NId, e: EId) {
    let t_rank = match fg.nodes[t].clust {
        Some(c) => {
            let leader = fg.graphs[c].leader.expect("leader");
            fg.nodes[t].rank - fg.nodes[leader].rank
        }
        None => 0,
    };
    let h_rank = match fg.nodes[h].clust {
        Some(c) => {
            let leader = fg.graphs[c].leader.expect("leader");
            fg.nodes[h].rank - fg.nodes[leader].rank
        }
        None => 0,
    };
    let offset = fg.edges[e].minlen + t_rank - h_rank;
    let (t_len, h_len) = if offset > 0 {
        (0, offset)
    } else {
        (-offset, 0)
    };

    let v = virtual_node(fg, g);
    fg.nodes[v].node_type = SLACKNODE;
    let t0 = fg.uf_find(t);
    let h0 = fg.uf_find(h);
    let weight = fg.edges[e].weight;
    let rt = make_aux_edge(fg, v, t0, t_len, super::model::CL_BACK * weight);
    let rh = make_aux_edge(fg, v, h0, h_len, weight);
    fg.edges[rt].to_orig = Some(e);
    fg.edges[rh].to_orig = Some(e);
}

/// `class1` — build the ranking fast graph.
pub fn class1(fg: &mut Fg, g: GId) {
    mark_clusters(fg, g);
    for n in fg.graphs[g].nodes_order.clone() {
        for e in fg.input_out[n].clone() {
            if !fg.graphs[g].owns_input_edge(fg, e) {
                continue;
            }
            // skip edges already processed
            if fg.edges[e].to_virt.is_some() {
                continue;
            }
            // skip edges that we want to ignore in this phase
            if nonconstraint_edge(fg, e) {
                continue;
            }
            let t = fg.uf_find(fg.edges[e].tail);
            let h = fg.uf_find(fg.edges[e].head);
            // skip self, flat, and intra-cluster edges
            if t == h {
                continue;
            }
            // inter-cluster edges require special treatment
            if fg.nodes[t].clust.is_some() || fg.nodes[h].clust.is_some() {
                interclust1(fg, g, fg.edges[e].tail, fg.edges[e].head, e);
                continue;
            }
            if let Some(rep) = find_fast_edge(fg, t, h) {
                merge_oneway(fg, e, rep);
            } else {
                virtual_edge(fg, t, h, Some(e));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// class2.c
// ---------------------------------------------------------------------------

/// `label_vnode` — a virtual node that reserves room for an edge label.
fn label_vnode(fg: &mut Fg, g: GId, orig: EId) -> NId {
    let label = fg.edges[orig].label.expect("label");
    let dimen = fg.labels[label].dimen;
    let v = virtual_node(fg, g);
    fg.nodes[v].label = Some(label);
    fg.nodes[v].lw = fg.graphs[g].nodesep as f64;
    let flip = fg.graphs[g].rankdir.flip();
    if !fg.edges[orig].label_ontop {
        if flip {
            fg.nodes[v].ht = dimen.x;
            fg.nodes[v].rw = dimen.y;
        } else {
            fg.nodes[v].ht = dimen.y;
            fg.nodes[v].rw = dimen.x;
        }
    }
    v
}

/// `plain_vnode`.
fn plain_vnode(fg: &mut Fg, g: GId) -> NId {
    let v = virtual_node(fg, g);
    incr_width(fg, g, v);
    v
}

/// `leader_of` (class2.c).
fn leader_of(fg: &Fg, v: NId) -> NId {
    if fg.nodes[v].ranktype != RankType::Cluster {
        fg.uf_find(v)
    } else {
        let clust = fg.nodes[v].clust.expect("clust");
        fg.graphs[clust].rankleader[fg.nodes[v].rank as usize].expect("rankleader")
    }
}

/// `make_chain` — create the virtual-node chain for `orig` between from/to.
fn make_chain(fg: &mut Fg, g: GId, from: NId, to: NId, orig: EId) {
    debug_assert!(fg.edges[orig].to_virt.is_none());
    let mut u = from;
    let label_rank = if fg.edges[orig].label.is_some() {
        (fg.nodes[from].rank + fg.nodes[to].rank) / 2
    } else {
        -1
    };
    let from_rank = fg.nodes[from].rank;
    let to_rank = fg.nodes[to].rank;
    let mut r = from_rank + 1;
    while r <= to_rank {
        let v = if r < to_rank {
            let v = if r == label_rank {
                label_vnode(fg, g, orig)
            } else {
                plain_vnode(fg, g)
            };
            fg.nodes[v].rank = r;
            v
        } else {
            to
        };
        let e = virtual_edge(fg, u, v, Some(orig));
        virtual_weight(fg, e);
        u = v;
        r += 1;
    }
    debug_assert!(fg.edges[orig].to_virt.is_some());
}

/// `virtual_weight` (mincross.c) — chain-edge weight scaled by the span.
pub fn virtual_weight(fg: &mut Fg, e: EId) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    let r = (fg.nodes[h].rank - fg.nodes[t].rank) as i64;
    let v = r * r * fg.edges[e].weight as i64 * MC_SCALE;
    fg.edges[e].weight = i32::try_from(v).expect("virtual_weight overflow");
}

/// `interclrep`.
fn interclrep(fg: &mut Fg, g: GId, e: EId) {
    let mut t = leader_of(fg, fg.edges[e].tail);
    let mut h = leader_of(fg, fg.edges[e].head);
    if fg.nodes[t].rank > fg.nodes[h].rank {
        std::mem::swap(&mut t, &mut h);
    }
    if fg.nodes[t].clust != fg.nodes[h].clust {
        if let Some(ve) = find_fast_edge(fg, t, h) {
            merge_chain(fg, g, e, ve, true);
            return;
        }
        if fg.nodes[t].rank == fg.nodes[h].rank {
            return;
        }
        make_chain(fg, g, t, h, e);

        // mark as cluster edge
        let mut ve = fg.edges[e].to_virt;
        while let Some(v) = ve {
            let head = fg.edges[v].head;
            if fg.nodes[head].rank > fg.nodes[h].rank {
                break;
            }
            fg.edges[v].edge_type = EdgeType::ClusterEdge;
            ve = fg.nodes[head].out.first().copied();
        }
    }
    // else ignore intra-cluster edges at this point
}

fn is_cluster_edge(fg: &Fg, e: EId) -> bool {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    fg.nodes[t].ranktype == RankType::Cluster || fg.nodes[h].ranktype == RankType::Cluster
}

/// `class2` — classify edges for mincross/position/splines using given ranks.
pub fn class2(fg: &mut Fg, g: GId) {
    fg.graphs[g].nlist = None;

    mark_clusters(fg, g);
    for c in fg.graphs[g].clust.clone() {
        build_skeleton(fg, g, c);
    }

    // weight_class counting
    for n in fg.graphs[g].nodes_order.clone() {
        for e in fg.input_out[n].clone() {
            if !fg.graphs[g].owns_input_edge(fg, e) {
                continue;
            }
            let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
            if fg.nodes[h].weight_class <= 2 {
                fg.nodes[h].weight_class += 1;
            }
            if fg.nodes[t].weight_class <= 2 {
                fg.nodes[t].weight_class += 1;
            }
        }
    }

    for n in fg.graphs[g].nodes_order.clone() {
        if fg.nodes[n].clust.is_none() && fg.uf_find(n) == n {
            fast_node(fg, g, n);
        }
        let mut prev: Option<EId> = None;
        for e in fg.input_out[n].clone() {
            if !fg.graphs[g].owns_input_edge(fg, e) {
                continue;
            }

            // already processed
            if fg.edges[e].to_virt.is_some() {
                prev = Some(e);
                continue;
            }

            // edges involving sub-clusters of g
            if is_cluster_edge(fg, e) {
                // new cluster multi-edge code
                if mergeable(fg, prev, Some(e)) {
                    let prev_e = prev.unwrap();
                    if fg.edges[prev_e].to_virt.is_some() {
                        merge_chain(fg, g, e, fg.edges[prev_e].to_virt.unwrap(), false);
                        other_edge(fg, e);
                    } else if fg.nodes[fg.edges[e].tail].rank == fg.nodes[fg.edges[e].head].rank {
                        merge_oneway(fg, e, prev_e);
                        other_edge(fg, e);
                    }
                    // else is an intra-cluster edge
                    continue;
                }
                interclrep(fg, g, e);
                prev = Some(e);
                continue;
            }

            // merge multi-edges
            if let Some(prev_e) = prev
                && fg.edges[e].tail == fg.edges[prev_e].tail
                && fg.edges[e].head == fg.edges[prev_e].head
            {
                if fg.nodes[fg.edges[e].tail].rank == fg.nodes[fg.edges[e].head].rank {
                    merge_oneway(fg, e, prev_e);
                    other_edge(fg, e);
                    continue;
                }
                if fg.edges[e].label.is_none()
                    && fg.edges[prev_e].label.is_none()
                    && ports_eq(fg, e, prev_e)
                {
                    if fg.concentrate {
                        fg.edges[e].edge_type = EdgeType::Ignored;
                    } else {
                        merge_chain(fg, g, e, fg.edges[prev_e].to_virt.unwrap(), true);
                        other_edge(fg, e);
                    }
                    continue;
                }
                // parallel edges with different labels fall through
            }

            // self edges
            if fg.edges[e].tail == fg.edges[e].head {
                other_edge(fg, e);
                prev = Some(e);
                continue;
            }

            let t = fg.uf_find(fg.edges[e].tail);
            let h = fg.uf_find(fg.edges[e].head);

            // non-leader leaf nodes
            if fg.edges[e].tail != t || fg.edges[e].head != h {
                continue;
            }

            let (tr, hr) = (
                fg.nodes[fg.edges[e].tail].rank,
                fg.nodes[fg.edges[e].head].rank,
            );

            // flat edges
            if tr == hr {
                flat_edge(fg, g, e);
                prev = Some(e);
                continue;
            }

            // forward edges
            if hr > tr {
                make_chain(fg, g, fg.edges[e].tail, fg.edges[e].head, e);
                prev = Some(e);
                continue;
            }

            // backward edges: find a forward edge they shadow
            let head = fg.edges[e].head;
            let mut merged_into: Option<EId> = None;
            for opp in fg.input_out[head].clone() {
                if !fg.graphs[g].owns_input_edge(fg, opp) {
                    continue;
                }
                // class2.c:262-265 — `aghead(opp) != agtail(e) ||
                // aghead(opp) == aghead(e) || ED_edge_type(opp) == IGNORED`.
                // (A previous revision compared `aghead(opp)` with itself,
                // making the whole backward-edge branch dead code.)
                if fg.edges[opp].head != fg.edges[e].tail
                    || fg.edges[opp].head == fg.edges[e].head
                    || fg.edges[opp].edge_type == EdgeType::Ignored
                {
                    continue;
                }
                // shadows a forward edge
                if fg.edges[opp].to_virt.is_none() {
                    make_chain(fg, g, fg.edges[opp].tail, fg.edges[opp].head, opp);
                }
                if fg.edges[e].label.is_none()
                    && fg.edges[opp].label.is_none()
                    && ports_eq(fg, e, opp)
                {
                    if fg.concentrate {
                        fg.edges[e].edge_type = EdgeType::Ignored;
                        fg.edges[opp].conc_opp_flag = true;
                    } else {
                        other_edge(fg, e);
                        merge_chain(fg, g, e, fg.edges[opp].to_virt.unwrap(), true);
                    }
                    merged_into = Some(opp);
                    break;
                }
            }
            if merged_into.is_some() {
                prev = Some(e);
                continue;
            }
            make_chain(fg, g, fg.edges[e].head, fg.edges[e].tail, e);
            prev = Some(e);
        }
    }
    // since decompose() is not called on subgraphs
    if g != fg.root_g() {
        let nlist = fg.graphs[g].nlist;
        fg.graphs[g].comp = vec![nlist.expect("non-empty cluster")];
    }
}

impl super::model::Fg {
    /// The root graph id.
    pub fn root_g(&self) -> GId {
        0
    }
}

impl super::model::DGraph {
    /// Whether orig edge `e` (an input edge) is a member of this graph.
    /// Root owns every edge; clusters own their induced set (`edges_order`).
    pub fn owns_input_edge(&self, _fg: &super::model::Fg, e: super::model::EId) -> bool {
        if self.input.is_none() {
            return true; // root
        }
        self.edges_order.contains(&e)
    }
}
