//! Ranking phase — a faithful port of `lib/dotgen/rank.c` (`dot1_rank`) and
//! `lib/dotgen/decomp.c` (`decompose`).

use super::classes::{acyclic, class1, delete_fast_node, reverse_edge, virtual_edge};
use super::model::{ClustType, EId, Fg, GId, NId, NodeType, RankType};

const INT_MAX: i32 = i32::MAX;

/// `is_a_cluster` — the root, names starting with `cluster` (case
/// insensitive), or `cluster=true`.
pub(crate) fn is_a_cluster(fg: &Fg, g: GId) -> bool {
    let dg = &fg.graphs[g];
    dg.input.is_none() || dg.is_cluster_name || dg.cluster_flag
}

/// `maptoken("rank", …)` — rank attribute class.
fn rank_set_class(fg: &Fg, g: GId) -> RankType {
    if is_a_cluster(fg, g) {
        return RankType::Cluster;
    }
    match fg.graphs[g].rank_attr.as_deref() {
        Some("same") => RankType::SameRank,
        Some("min") => RankType::MinRank,
        Some("source") => RankType::SourceRank,
        Some("max") => RankType::MaxRank,
        Some("sink") => RankType::SinkRank,
        _ => RankType::Normal, // NORANK
    }
}

/// `decomp.c: decompose` — connected components over fast edges.
pub fn decompose(fg: &mut Fg, g: GId, pass: i32) {
    fg.cmark += 1;
    if fg.cmark == 0 {
        fg.cmark = 1;
    }
    let cmark = fg.cmark;
    fg.graphs[g].comp.clear();

    for &n in fg.graphs[g].nodes_order.clone().iter() {
        let mut v = n;
        if pass > 0 {
            if let Some(subg) = fg.nodes[v].clust {
                v = fg.graphs[subg].rankleader[fg.nodes[v].rank as usize].expect("rankleader");
            }
        } else if fg.uf_find(v) != v {
            continue;
        }
        if fg.nodes[v].mark != cmark {
            search_component(fg, g, v, cmark);
        }
    }
}

/// `search_component` — iterative DFS building one component's node list.
fn search_component(fg: &mut Fg, g: GId, start: NId, cmark: usize) {
    let mut stk: Vec<NId> = Vec::new();
    // push
    fg.nodes[start].mark = cmark + 1;
    stk.push(start);

    let mut last_node: Option<NId> = None;
    let mut comp_head: Option<NId> = None;

    while let Some(n) = stk.pop() {
        if fg.nodes[n].mark == cmark {
            continue;
        }
        // add_to_component
        fg.nodes[n].mark = cmark;
        match last_node {
            Some(prev) => {
                fg.nodes[n].prev = Some(prev);
                fg.nodes[prev].next = Some(n);
            }
            None => {
                // C's `add_to_component`: the first node of a component
                // becomes GD_nlist(g)/GD_comp(g).list[i]
                fg.nodes[n].prev = None;
                comp_head = Some(n);
                fg.graphs[g].nlist = Some(n);
            }
        }
        last_node = Some(n);
        fg.nodes[n].next = None;

        // [flat_in, flat_out, in, out], each scanned in reverse
        let mut ids: Vec<EId> = fg.nodes[n].flat_in.clone();
        ids.extend_from_slice(&fg.nodes[n].flat_out);
        ids.extend_from_slice(&fg.nodes[n].in_);
        ids.extend_from_slice(&fg.nodes[n].out);
        // per-list reverse iteration: flat_in, flat_out, in, out
        let lens = [
            fg.nodes[n].flat_in.len(),
            fg.nodes[n].flat_out.len(),
            fg.nodes[n].in_.len(),
            fg.nodes[n].out.len(),
        ];
        {
            let mut start = 0;
            for len in lens {
                let seg: Vec<EId> = ids[start..start + len].to_vec();
                start += len;
                for &e in seg.iter().rev() {
                let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
                let other = if h == n { t } else { h };
                if fg.nodes[other].mark != cmark && fg.uf_find(other) == other {
                    fg.nodes[other].mark = cmark + 1;
                    stk.push(other);
                }
                }
            }
        }
    }
    if let Some(head) = comp_head {
        fg.graphs[g].comp.push(head);
    }
}

/// `edgelabel_ranks` — double minlen and halve ranksep when edge labels
/// exist (the `GD_has_labels(g) & EDGE_LABEL` flag; only the root has it).
pub fn edgelabel_ranks(fg: &mut Fg, g: GId) {
    const EDGE_LABEL: u8 = 1;
    if fg.graphs[g].has_labels & EDGE_LABEL == 0 {
        return;
    }
    for e in 0..fg.n_orig_edges {
        fg.edges[e].minlen *= 2;
    }
    fg.graphs[g].ranksep = (fg.graphs[g].ranksep + 1) / 2;
}

/// `collapse_rankset` — merge the nodes of a min/max/same rank set.
fn collapse_rankset(fg: &mut Fg, g: GId, subg: GId, kind: RankType) {
    let members = fg.graphs[subg].nodes_order.clone();
    if let Some(&u) = members.first() {
        fg.nodes[u].ranktype = kind;
        for &v in members.iter().skip(1) {
            fg.uf_union(u, v);
            let rt = fg.nodes[u].ranktype;
            fg.nodes[v].ranktype = rt;
        }
        match kind {
            RankType::MinRank | RankType::SourceRank => {
                match fg.graphs[g].minset {
                    None => fg.graphs[g].minset = Some(u),
                    Some(m) => {
                        let r = fg.uf_union(m, u);
                        fg.graphs[g].minset = Some(r);
                    }
                }
            }
            RankType::MaxRank | RankType::SinkRank => {
                match fg.graphs[g].maxset {
                    None => fg.graphs[g].maxset = Some(u),
                    Some(m) => {
                        let r = fg.uf_union(m, u);
                        fg.graphs[g].maxset = Some(r);
                    }
                }
            }
            _ => {}
        }
        match kind {
            RankType::SourceRank => {
                let m = fg.graphs[g].minset.expect("minset");
                fg.nodes[m].ranktype = kind;
            }
            RankType::SinkRank => {
                let m = fg.graphs[g].maxset.expect("maxset");
                fg.nodes[m].ranktype = kind;
            }
            _ => {}
        }
    }
}

/// `node_induce` — pull subgraph nodes/edges into a cluster, enforcing one
/// cluster per node per level.
pub(crate) fn node_induce(fg: &mut Fg, par: GId, g: GId) {
    // enforce that a node is in at most one cluster at this level
    let members = fg.graphs[g].nodes_order.clone();
    let mut kept = Vec::with_capacity(members.len());
    for n in members {
        if fg.nodes[n].ranktype != RankType::Normal {
            continue; // agdelete
        }
        let mut contained = false;
        for &sib in fg.graphs[par].clust.iter() {
            if fg.graphs[sib].nodes_order.contains(&n) {
                contained = true;
                break;
            }
        }
        if contained {
            continue;
        }
        fg.nodes[n].clust = None;
        kept.push(n);
    }
    fg.graphs[g].nodes_order = kept;

    // edges between induced nodes
    let set: std::collections::HashSet<NId> = fg.graphs[g].nodes_order.iter().copied().collect();
    let mut edges = Vec::new();
    for n in fg.graphs[g].nodes_order.clone() {
        for e in fg.input_out[n].clone() {
            if set.contains(&fg.edges[e].head) {
                edges.push(e);
            }
        }
    }
    fg.graphs[g].edges_order = edges;
}

/// `dot_scan_ranks` — min/max/leader from assigned ranks.
pub(crate) fn dot_scan_ranks(fg: &mut Fg, g: GId) {
    let mut minrank = INT_MAX;
    let mut maxrank = -1;
    let mut leader: Option<NId> = None;
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        let r = fg.nodes[n].rank;
        if maxrank < r {
            maxrank = r;
        }
        if minrank > r {
            minrank = r;
        }
        if leader.is_none() {
            leader = Some(n);
        } else if r < fg.nodes[leader.unwrap()].rank {
            leader = Some(n);
        }
    }
    fg.graphs[g].minrank = minrank;
    fg.graphs[g].maxrank = maxrank;
    fg.graphs[g].leader = leader;
}

/// `cluster_leader` — pick the last rank-0 NORMAL node in the fast list.
fn cluster_leader(fg: &mut Fg, clust: GId) {
    let mut leader: Option<NId> = None;
    let mut next = fg.graphs[clust].nlist;
    while let Some(n) = next {
        if fg.nodes[n].rank == 0 && fg.nodes[n].node_type == NodeType::Normal {
            leader = Some(n);
        }
        next = fg.nodes[n].next;
    }
    let leader = leader.expect("cluster leader");
    fg.graphs[clust].leader = Some(leader);

    for &n in fg.graphs[clust].nodes_order.clone().iter() {
        fg.uf_union(n, leader);
        fg.nodes[n].ranktype = RankType::Cluster;
    }
}

/// `make_new_cluster` — register subg as a cluster of g.
pub(crate) fn make_new_cluster(fg: &mut Fg, g: GId, subg: GId) {
    fg.graphs[g].clust.push(subg);
    // do_graph_label(subg): cluster label sizing — handled by the caller's
    // label pipeline; nothing needed for ranking.
}

/// `collapse_cluster`.
fn collapse_cluster(fg: &mut Fg, g: GId, subg: GId, cl_type: ClustType) {
    if fg.graphs[subg].parent.is_some() {
        return;
    }
    fg.graphs[subg].parent = Some(g);
    node_induce(fg, g, subg);
    if fg.graphs[subg].nodes_order.is_empty() {
        return;
    }
    make_new_cluster(fg, g, subg);
    if cl_type == ClustType::Local {
        dot1_rank(fg, subg);
        cluster_leader(fg, subg);
    } else {
        dot_scan_ranks(fg, subg);
    }
}

/// `collapse_sets` — classify subgraphs into clusters/ranksets.
fn collapse_sets(fg: &mut Fg, rg: GId, g: GId, cl_type: ClustType) {
    for subg in fg.graphs[g].children.clone() {
        let c = rank_set_class(fg, subg);
        if c != RankType::Normal {
            if c == RankType::Cluster && cl_type == ClustType::Local {
                collapse_cluster(fg, rg, subg, cl_type);
            } else {
                collapse_rankset(fg, rg, subg, c);
            }
        } else {
            collapse_sets(fg, rg, subg, cl_type);
        }
    }
}

/// `find_clusters` — GLOBAL/NOCLUST clusters are materialized after ranking.
fn find_clusters(fg: &mut Fg, g: GId, cl_type: ClustType) {
    let root_children = fg.graphs[0].children.clone();
    for subg in root_children {
        if fg.graphs[subg].set_type == RankType::Cluster as u8 {
            collapse_cluster(fg, g, subg, cl_type);
        }
    }
}

fn set_minmax(fg: &mut Fg, g: GId) {
    let leader = fg.graphs[g].leader.expect("leader");
    let lr = fg.nodes[leader].rank;
    fg.graphs[g].minrank += lr;
    fg.graphs[g].maxrank += lr;
    for c in fg.graphs[g].clust.clone() {
        set_minmax(fg, c);
    }
}

/// `minmax_edges` — reverse out/in edges of maxset/minset; returns strictness
/// flags (x = source strict, y = sink strict).
fn minmax_edges(fg: &mut Fg, g: GId) -> (i32, i32) {
    let mut slen = (0i32, 0i32);
    if fg.graphs[g].maxset.is_none() && fg.graphs[g].minset.is_none() {
        return slen;
    }
    if let Some(m) = fg.graphs[g].minset {
        fg.graphs[g].minset = Some(fg.uf_find(m));
    }
    if let Some(m) = fg.graphs[g].maxset {
        fg.graphs[g].maxset = Some(fg.uf_find(m));
    }

    if let Some(n) = fg.graphs[g].maxset {
        slen.1 = (fg.nodes[n].ranktype == RankType::SinkRank) as i32;
        while let Some(&e) = fg.nodes[n].out.first() {
            reverse_edge(fg, e);
        }
    }
    if let Some(n) = fg.graphs[g].minset {
        slen.0 = (fg.nodes[n].ranktype == RankType::SourceRank) as i32;
        while let Some(&e) = fg.nodes[n].in_.first() {
            reverse_edge(fg, e);
        }
    }
    slen
}

/// `minmax_edges2` — connect isolated sources/sinks to the min/max sets with
/// zero-weight edges.
fn minmax_edges2(fg: &mut Fg, g: GId, slen: (i32, i32)) -> bool {
    let mut made = false;
    if fg.graphs[g].maxset.is_some() || fg.graphs[g].minset.is_some() {
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            if fg.uf_find(n) != n {
                continue;
            }
            if fg.nodes[n].out.is_empty() && fg.graphs[g].maxset.is_some() {
                let maxset = fg.graphs[g].maxset.unwrap();
                if n != maxset {
                    let e = virtual_edge(fg, n, maxset, None);
                    fg.edges[e].minlen = slen.1;
                    fg.edges[e].weight = 0;
                    made = true;
                }
            }
            if fg.nodes[n].in_.is_empty() && fg.graphs[g].minset.is_some() {
                let minset = fg.graphs[g].minset.unwrap();
                if n != minset {
                    let e = virtual_edge(fg, minset, n, None);
                    fg.edges[e].minlen = slen.0;
                    fg.edges[e].weight = 0;
                    made = true;
                }
            }
        }
    }
    made
}

/// `rank1` — ns per component.
fn rank1(fg: &mut Fg, g: GId) {
    let maxiter = match fg.graphs[g].nslimit1 {
        Some(limit) => super::scale_clamp(fg.graphs[g].nodes_order.len(), limit),
        None => INT_MAX,
    };
    let balance = if fg.graphs[g].clust.is_empty() { 1 } else { 0 };
    let search_size = fg.graphs[g].search_size;
    let tbbalance = fg.graphs[g].tbbalance.clone();
    for comp in fg.graphs[g].comp.clone() {
        let nodes = component_nodes(fg, comp);
        super::ns::rank2(fg, nodes, balance, maxiter, search_size, tbbalance.as_deref())
            .expect("network simplex failed");
    }
}

/// Collects the node chain starting at a component head.
fn component_nodes(fg: &Fg, head: NId) -> Vec<NId> {
    let mut out = Vec::new();
    let mut next = Some(head);
    while let Some(n) = next {
        out.push(n);
        next = fg.nodes[n].next;
    }
    out
}

/// `expand_ranksets` — apply leader offsets, compute min/max ranks.
fn expand_ranksets(fg: &mut Fg, g: GId, cl_type: ClustType) {
    if !fg.graphs[g].nodes_order.is_empty() {
        fg.graphs[g].minrank = INT_MAX;
        fg.graphs[g].maxrank = -1;
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            let leader = fg.uf_find(n);
            if leader != n {
                fg.nodes[n].rank += fg.nodes[leader].rank;
            }
            let r = fg.nodes[n].rank;
            if fg.graphs[g].maxrank < r {
                fg.graphs[g].maxrank = r;
            }
            if fg.graphs[g].minrank > r {
                fg.graphs[g].minrank = r;
            }
            if fg.nodes[n].ranktype != RankType::Normal
                && fg.nodes[n].ranktype != RankType::LeafSet
            {
                fg.uf_singleton(n);
            }
        }
        if g == fg.root_g() {
            if cl_type == ClustType::Local {
                for c in fg.graphs[g].clust.clone() {
                    set_minmax(fg, c);
                }
            } else {
                find_clusters(fg, g, cl_type);
            }
        }
    } else {
        fg.graphs[g].minrank = 0;
        fg.graphs[g].maxrank = 0;
    }
}

/// `cleanup1` — tear down the ranking fast graph. In this arena port the
/// edge objects remain allocated, so the essential effects are: clear the
/// fast in/out lists, unlink slack nodes, clear all `ED_to_virt` backlinks
/// and reset marks/comp.
fn cleanup1(fg: &mut Fg, g: GId) {
    for comp in fg.graphs[g].comp.clone() {
        let mut next = Some(comp);
        while let Some(n) = next {
            next = fg.nodes[n].next;
            fg.nodes[n].in_.clear();
            fg.nodes[n].out.clear();
            fg.nodes[n].mark = 0;
            if fg.nodes[n].node_type == NodeType::Slacknode {
                delete_fast_node(fg, g, n);
            }
        }
    }
    for e in 0..fg.n_orig_edges {
        fg.edges[e].to_virt = None;
    }
    fg.graphs[g].comp.clear();
}

/// `dot1_rank`.
pub fn dot1_rank(fg: &mut Fg, g: GId) {
    edgelabel_ranks(fg, g);
    let cl_type = fg.cl_type;

    collapse_sets(fg, g, g, cl_type);
    class1(fg, g);
    let p = minmax_edges(fg, g);
    decompose(fg, g, 0);
    acyclic(fg, g);
    if minmax_edges2(fg, g, p) {
        decompose(fg, g, 0);
    }

    rank1(fg, g);
    expand_ranksets(fg, g, cl_type);
    cleanup1(fg, g);
}

/// `dot_rank` — dispatch on `newrank`.
pub fn dot_rank(fg: &mut Fg, g: GId) {
    if fg.graphs[g].newrank {
        super::newrank::dot2_rank(fg, g);
    } else {
        dot1_rank(fg, g);
    }
}
