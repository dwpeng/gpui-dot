//! `newrank` ranking — a faithful port of `dot2_rank` and its helpers in
//! `lib/dotgen/rank.c` (the `level.c`-derived code below the `dot1_rank`
//! half).
//!
//! Where `dot1_rank` collapses clusters into a single node and ranks the
//! collapsed graph, `dot2_rank` builds a *separate* constraint graph `Xg`:
//! one node per rank-set representative, strong (`minlen`) constraints for
//! ordinary edges, weak (backward-penalty) ones across compact clusters, and
//! `\177top`/`\177bot` helper nodes per compact cluster. Network simplex then
//! ranks `Xg`, and the resulting levels are copied back onto the real nodes.
//!
//! The auxiliary graph lives in the same arena, appended after the input
//! nodes and edges; everything is truncated again once the levels are read
//! out (`agclose(Xg)` in C).

use super::model::{DEdge, DNode, EId, EdgeType, Fg, GId, NId, NodeType, RankType};
use super::rank::{edgelabel_ranks, is_a_cluster, make_new_cluster, node_induce};
use super::{ns, scale_clamp};

const BACKWARD_PENALTY: i32 = 1000;
const STRONG_CLUSTER_WEIGHT: i32 = 1000;
const INT_MAX: i32 = i32::MAX;
const ROOT: &str = "\u{7f}root";
const TOPNODE: &str = "\u{7f}top";
const BOTNODE: &str = "\u{7f}bot";

/// The auxiliary constraint graph being built (`Xg`).
struct Xg {
    /// Nodes in creation order (`GD_nlist`).
    nodes: Vec<NId>,
    /// `static int id` in C's `weak()`.
    weak_id: usize,
}

impl Xg {
    fn new_node(&mut self, fg: &mut Fg, name: &str) -> NId {
        let id = fg.nodes.len();
        fg.nodes.push(DNode {
            name: name.to_string(),
            node_type: NodeType::Normal,
            ..Default::default()
        });
        self.nodes.push(id);
        id
    }
}

/// `agedge(g, t, h, 0, 1)` — always creates a new edge (no lookup).
fn new_xedge(fg: &mut Fg, t: NId, h: NId) -> EId {
    let id = fg.edges.len();
    fg.edges.push(DEdge {
        tail: t,
        head: h,
        edge_type: EdgeType::Normal,
        ..Default::default()
    });
    fg.nodes[t].out.push(id);
    fg.nodes[h].in_.push(id);
    id
}

/// `agfindedge(g, t, h)` — the edge from `t` to `h`, if any.
fn find_xedge(fg: &Fg, t: NId, h: NId) -> Option<EId> {
    fg.nodes[t]
        .out
        .iter()
        .copied()
        .find(|&e| fg.edges[e].head == h)
}

/// `agdelete(g, e)` — the edge lists are all the simplex walks, so removing
/// it from its endpoints' lists is enough.
fn delete_xedge(fg: &mut Fg, e: EId) {
    let t = fg.edges[e].tail;
    let h = fg.edges[e].head;
    if let Some(i) = fg.nodes[t].out.iter().position(|&x| x == e) {
        fg.nodes[t].out.remove(i);
    }
    if let Some(i) = fg.nodes[h].in_.iter().position(|&x| x == e) {
        fg.nodes[h].in_.remove(i);
    }
    fg.edges[e].edge_type = EdgeType::Ignored;
}

/// `merge` — tighten the length bound and accumulate the weight.
fn merge(fg: &mut Fg, e: EId, minlen: i32, weight: i32) {
    fg.edges[e].minlen = fg.edges[e].minlen.max(minlen);
    fg.edges[e].weight += weight;
}

// ---------------------------------------------------------------------------
// rank sets (`ND_set` union-find)
// ---------------------------------------------------------------------------

/// `find` — path-compressed root of a rank-set union-find.
fn find(fg: &mut Fg, n: NId) -> NId {
    let mut root = n;
    while let Some(s) = fg.nodes[root].set {
        if s == root {
            break;
        }
        root = s;
    }
    // compress
    let mut cur = n;
    while let Some(s) = fg.nodes[cur].set {
        if s == cur {
            break;
        }
        fg.nodes[cur].set = Some(root);
        cur = s;
    }
    fg.nodes[root].set = Some(root);
    root
}

/// `union_one(leader, n)` — attach a rank set to a leader's set.
fn union_one(fg: &mut Fg, leader: NId, n: Option<NId>) -> NId {
    match n {
        Some(n) => {
            let l = find(fg, leader);
            let r = find(fg, n);
            fg.nodes[r].set = Some(l);
            l
        }
        None => leader,
    }
}

/// `union_all` — union every node of the subgraph and return the leader.
fn union_all(fg: &mut Fg, members: &[NId]) -> Option<NId> {
    let first = *members.first()?;
    let leader = find(fg, first);
    for &n in &members[1..] {
        union_one(fg, leader, Some(n));
    }
    Some(leader)
}

/// `rankset_kind` — the `rank` attribute's class (clusters are *not* rank
/// sets here, unlike `rank_set_class`).
fn rankset_kind(fg: &Fg, g: GId) -> RankType {
    match fg.graphs[g].rank_attr.as_deref() {
        Some("min") => RankType::MinRank,
        Some("source") => RankType::SourceRank,
        Some("max") => RankType::MaxRank,
        Some("sink") => RankType::SinkRank,
        Some("same") => RankType::SameRank,
        _ => RankType::Normal, // NORANK
    }
}

/// `is_a_strong_cluster` — the `compact` attribute.
fn is_a_strong_cluster(fg: &Fg, g: GId) -> bool {
    fg.graphs[g].compact
}

/// `is_nonconstraint` — `constraint=false` on an original edge.
fn is_nonconstraint(fg: &Fg, e: EId) -> bool {
    e < fg.n_orig_edges && fg.nonconstraint[e]
}

// ---------------------------------------------------------------------------
// compile_samerank / compile_nodes / compile_edges / compile_clusters
// ---------------------------------------------------------------------------

/// `set_parent(g, p)` — GD_parent, cluster registration and node induction.
///
/// The port's `node_induce` screens a cluster's nodes against the *siblings
/// already registered* in the parent, so it has to run before the cluster is
/// registered (C's version works off cgraph membership instead).
fn set_parent(fg: &mut Fg, g: GId, p: GId) {
    fg.graphs[g].parent = Some(p);
    node_induce(fg, p, g);
    make_new_cluster(fg, p, g);
}

/// `compile_samerank` — walk the subgraph tree, marking clusters and unioning
/// rank sets.
fn compile_samerank(fg: &mut Fg, ug: GId, parent_clust: Option<GId>) {
    // `is_empty`: no nodes anywhere in the subtree.
    if fg.graphs[ug].nodes_order.is_empty() {
        return;
    }
    let is_clust = is_a_cluster(fg, ug);
    let clust = if is_clust {
        match parent_clust {
            Some(p) => {
                fg.graphs[ug].level = fg.graphs[p].level + 1;
                set_parent(fg, ug, p);
            }
            None => fg.graphs[ug].level = 0,
        }
        Some(ug)
    } else {
        parent_clust
    };

    for sub in fg.graphs[ug].children.clone() {
        compile_samerank(fg, sub, clust);
    }

    if is_clust {
        for &n in fg.graphs[ug].nodes_order.clone().iter() {
            if fg.nodes[n].clust.is_none() {
                fg.nodes[n].clust = Some(ug);
            }
        }
    }

    match rankset_kind(fg, ug) {
        RankType::SourceRank | RankType::MinRank => {
            let members = fg.graphs[ug].nodes_order.clone();
            if let Some(leader) = union_all(fg, &members)
                && let Some(cl) = clust {
                    let rep = fg.graphs[cl].minrep;
                    let r = union_one(fg, leader, rep);
                    fg.graphs[cl].minrep = Some(r);
                }
        }
        RankType::SinkRank | RankType::MaxRank => {
            let members = fg.graphs[ug].nodes_order.clone();
            if let Some(leader) = union_all(fg, &members)
                && let Some(cl) = clust {
                    let rep = fg.graphs[cl].maxrep;
                    let r = union_one(fg, leader, rep);
                    fg.graphs[cl].maxrep = Some(r);
                }
        }
        RankType::SameRank => {
            let members = fg.graphs[ug].nodes_order.clone();
            union_all(fg, &members);
        }
        _ => {}
    }

    // a cluster may become degenerate
    if is_clust
        && let Some(minrep) = fg.graphs[ug].minrep
            && Some(minrep) == fg.graphs[ug].maxrep {
                let members = fg.graphs[ug].nodes_order.clone();
                if let Some(up) = union_all(fg, &members) {
                    fg.graphs[ug].minrep = Some(up);
                    fg.graphs[ug].maxrep = Some(up);
                }
            }
}

/// `dot_lca` — lowest common ancestor of two clusters.
fn dot_lca(fg: &Fg, mut c0: GId, mut c1: GId) -> GId {
    while c0 != c1 {
        if fg.graphs[c0].level >= fg.graphs[c1].level {
            match fg.graphs[c0].parent {
                Some(p) => c0 = p,
                None => break,
            }
        } else {
            match fg.graphs[c1].parent {
                Some(p) => c1 = p,
                None => break,
            }
        }
    }
    c0
}

/// `is_internal_to_cluster` — both endpoints inside one cluster subtree.
fn is_internal_to_cluster(fg: &Fg, e: EId) -> bool {
    let ct = fg.nodes[fg.edges[e].tail].clust;
    let ch = fg.nodes[fg.edges[e].head].clust;
    if ct == ch {
        return true;
    }
    let (Some(ct), Some(ch)) = (ct, ch) else {
        return false;
    };
    let par = dot_lca(fg, ct, ch);
    par == ct || par == ch
}

/// `compile_nodes` — one `Xg` variable per rank-set representative.
fn compile_nodes(fg: &mut Fg, g: GId, xg: &mut Xg) {
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        if find(fg, n) == n {
            let name = fg.nodes[n].name.clone();
            let x = xg.new_node(fg, &name);
            fg.nodes[n].rep = Some(x);
        }
    }
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        if fg.nodes[n].rep.is_none() {
            let r = find(fg, n);
            fg.nodes[n].rep = fg.nodes[r].rep;
        }
    }
}

/// `strong` — a `minlen` constraint between two `Xg` variables.
fn strong(fg: &mut Fg, t: NId, h: NId, orig: EId) {
    let e = find_xedge(fg, t, h)
        .or_else(|| find_xedge(fg, h, t))
        .unwrap_or_else(|| new_xedge(fg, t, h));
    let (minlen, weight) = (fg.edges[orig].minlen, fg.edges[orig].weight);
    merge(fg, e, minlen, weight);
}

/// `weak` — a backward-penalty constraint pair through a fresh helper node.
fn weak(fg: &mut Fg, xg: &mut Xg, t: NId, h: NId, orig: EId) {
    for e in fg.nodes[t].in_.clone() {
        let v = fg.edges[e].tail;
        if let Some(f) = fg.nodes[v].out.first().copied()
            && fg.edges[f].head == h {
                return;
            }
    }
    let name = format!("_weak_{}", xg.weak_id);
    xg.weak_id += 1;
    let v = xg.new_node(fg, &name);
    let e = new_xedge(fg, v, t);
    let f = new_xedge(fg, v, h);
    merge(fg, e, 0, fg.edges[orig].weight * BACKWARD_PENALTY);
    let (minlen, weight) = (fg.edges[orig].minlen, fg.edges[orig].weight);
    merge(fg, f, minlen, weight);
}

/// `compile_edges` — turn every constraint edge into `Xg` edges.
fn compile_edges(fg: &mut Fg, ug: GId, xg: &mut Xg) {
    for &n in fg.graphs[ug].nodes_order.clone().iter() {
        let xt = fg.nodes[n].rep.expect("rep");
        for e in fg.input_out[n].clone() {
            if is_nonconstraint(fg, e) {
                continue;
            }
            let head = fg.edges[e].head;
            let hr = find(fg, head);
            let xh = fg.nodes[hr].rep.expect("rep");
            if xt == xh {
                continue;
            }
            let tc = fg.nodes[fg.edges[e].tail].clust;
            let hc = fg.nodes[head].clust;
            if is_internal_to_cluster(fg, e) {
                let tail = fg.edges[e].tail;
                let (mut a, mut b) = (xt, xh);
                // determine if graph requires reversed edge
                let flip = (tc.is_some() && Some(find(fg, tail)) == fg.graphs[tc.unwrap()].maxrep)
                    || (hc.is_some() && Some(find(fg, head)) == fg.graphs[hc.unwrap()].minrep);
                if flip {
                    std::mem::swap(&mut a, &mut b);
                }
                strong(fg, a, b, e);
            } else {
                let strong_cluster = tc.is_some_and(|c| is_a_strong_cluster(fg, c))
                    || hc.is_some_and(|c| is_a_strong_cluster(fg, c));
                if strong_cluster {
                    weak(fg, xg, xt, xh, e);
                } else {
                    strong(fg, xt, xh, e);
                }
            }
        }
    }
}

/// `compile_clusters` — `\177top`/`\177bot` constraints for compact clusters.
fn compile_clusters(fg: &mut Fg, g: GId, xg: &mut Xg, top: Option<NId>, bot: Option<NId>) {
    let mut top = top;
    let mut bot = bot;
    if is_a_cluster(fg, g) && is_a_strong_cluster(fg, g) {
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            if !has_edge_in(fg, g, n) {
                let r = find(fg, n);
                let rep = fg.nodes[r].rep.expect("rep");
                let t = *top.get_or_insert_with(|| xg.new_node(fg, TOPNODE));
                new_xedge(fg, t, rep);
            }
            if !has_edge_out(fg, g, n) {
                let r = find(fg, n);
                let rep = fg.nodes[r].rep.expect("rep");
                let b = *bot.get_or_insert_with(|| xg.new_node(fg, BOTNODE));
                new_xedge(fg, rep, b);
            }
        }
        if let (Some(t), Some(b)) = (top, bot) {
            let e = new_xedge(fg, t, b);
            merge(fg, e, 0, STRONG_CLUSTER_WEIGHT);
        }
    }
    for sub in fg.graphs[g].children.clone() {
        compile_clusters(fg, sub, xg, top, bot);
    }
}

/// `agfstin(g, n) != NULL` — n has an in-edge belonging to g.
fn has_edge_in(fg: &Fg, g: GId, n: NId) -> bool {
    fg.input_in[n]
        .iter()
        .any(|&e| e < fg.n_orig_edges && fg.graphs[g].owns_input_edge(fg, e))
}

/// `agfstout(g, n) != NULL` — n has an out-edge belonging to g.
fn has_edge_out(fg: &Fg, g: GId, n: NId) -> bool {
    fg.input_out[n]
        .iter()
        .any(|&e| e < fg.n_orig_edges && fg.graphs[g].owns_input_edge(fg, e))
}

// ---------------------------------------------------------------------------
// break_cycles / connect_components
// ---------------------------------------------------------------------------

/// `reverse_edge2` — merge the edge into its reverse and delete it.
fn reverse_edge2(fg: &mut Fg, e: EId) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    let rev = find_xedge(fg, h, t).unwrap_or_else(|| new_xedge(fg, h, t));
    let (minlen, weight) = (fg.edges[e].minlen, fg.edges[e].weight);
    merge(fg, rev, minlen, weight);
    delete_xedge(fg, e);
}

/// `dfs` (rank.c) — iterative DFS that reverses back edges.
fn dfs(fg: &mut Fg, start: NId) {
    // frames: (node, next index into its out list)
    let mut stack: Vec<(NId, usize)> = Vec::new();
    fg.nodes[start].mark = 1;
    fg.nodes[start].onstack = true;
    stack.push((start, 0));
    while let Some(&(v, i)) = stack.last() {
        let out = fg.nodes[v].out.clone();
        if i >= out.len() {
            fg.nodes[v].onstack = false;
            stack.pop();
            continue;
        }
        stack.last_mut().unwrap().1 = i + 1;
        let e = out[i];
        if fg.edges[e].edge_type == EdgeType::Ignored {
            continue; // deleted (reversed) earlier in this walk
        }
        let w = fg.edges[e].head;
        if fg.nodes[w].onstack {
            reverse_edge2(fg, e);
        } else if fg.nodes[w].mark == 0 {
            fg.nodes[w].mark = 1;
            fg.nodes[w].onstack = true;
            stack.push((w, 0));
        }
    }
}

/// `break_cycles` — DFS the whole `Xg` and reverse back edges.
fn break_cycles(fg: &mut Fg, xg: &Xg) {
    for &n in &xg.nodes {
        fg.nodes[n].mark = 0;
        fg.nodes[n].onstack = false;
    }
    let nodes = xg.nodes.clone();
    for n in nodes {
        if fg.nodes[n].mark == 0 {
            dfs(fg, n);
        }
    }
}

/// `dfscc` — label one connected component (`ND_comp` overloaded on hops).
fn dfscc(fg: &mut Fg, n: NId, cc: i32) {
    if fg.nodes[n].hops != 0 {
        return;
    }
    fg.nodes[n].hops = cc;
    let out = fg.nodes[n].out.clone();
    for e in out {
        if fg.edges[e].edge_type == EdgeType::Ignored {
            continue;
        }
        let h = fg.edges[e].head;
        dfscc(fg, h, cc);
    }
    let inn = fg.nodes[n].in_.clone();
    for e in inn {
        if fg.edges[e].edge_type == EdgeType::Ignored {
            continue;
        }
        let t = fg.edges[e].tail;
        dfscc(fg, t, cc);
    }
}

/// `connect_components` — add a `\177root` node with one edge per component.
fn connect_components(fg: &mut Fg, xg: &mut Xg) -> i32 {
    let mut cc = 0;
    for &n in &xg.nodes {
        fg.nodes[n].hops = 0;
    }
    let nodes = xg.nodes.clone();
    for n in nodes {
        if fg.nodes[n].hops == 0 {
            cc += 1;
            dfscc(fg, n, cc);
        }
    }
    if cc > 1 {
        let root = xg.new_node(fg, ROOT);
        let mut ncc = 1;
        let nodes = xg.nodes.clone();
        for n in nodes {
            if fg.nodes[n].hops == ncc {
                new_xedge(fg, root, n);
                ncc += 1;
            }
        }
    }
    cc
}

// ---------------------------------------------------------------------------
// readout
// ---------------------------------------------------------------------------

/// `setMinMax` — rank bounds and leader for a graph, recursing into clusters.
fn set_minmax(fg: &mut Fg, g: GId, do_root: bool) {
    for c in fg.graphs[g].clust.clone() {
        set_minmax(fg, c, false);
    }
    if fg.graphs[g].parent.is_none() && !do_root {
        return; // root graph
    }
    fg.graphs[g].minrank = INT_MAX;
    fg.graphs[g].maxrank = -1;
    let mut leader: Option<NId> = None;
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        let v = fg.nodes[n].rank;
        if fg.graphs[g].maxrank < v {
            fg.graphs[g].maxrank = v;
        }
        if fg.graphs[g].minrank > v {
            fg.graphs[g].minrank = v;
            leader = Some(n);
        }
    }
    fg.graphs[g].leader = leader;
}

/// `readout_levels` — copy the `Xg` levels back onto the real nodes.
fn readout_levels(fg: &mut Fg, g: GId, ncc: i32) {
    fg.graphs[g].minrank = INT_MAX;
    fg.graphs[g].maxrank = -1;
    let mut minrk: Option<Vec<i32>> = (ncc > 1).then(|| vec![INT_MAX; ncc as usize + 1]);
    let mut do_root = false;

    for &n in fg.graphs[g].nodes_order.clone().iter() {
        let r = find(fg, n);
        let xn = fg.nodes[r].rep.expect("rep");
        let rank = fg.nodes[xn].rank;
        fg.nodes[n].rank = rank;
        if fg.graphs[g].maxrank < rank {
            fg.graphs[g].maxrank = rank;
        }
        if fg.graphs[g].minrank > rank {
            fg.graphs[g].minrank = rank;
        }
        if let Some(m) = minrk.as_mut() {
            fg.nodes[n].hops = fg.nodes[xn].hops;
            let c = fg.nodes[n].hops as usize;
            m[c] = m[c].min(rank);
        }
    }
    if let Some(m) = minrk {
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            let c = fg.nodes[n].hops as usize;
            fg.nodes[n].rank -= m[c];
        }
        do_root = true;
    } else if fg.graphs[g].minrank > 0 {
        // should never happen
        let delta = fg.graphs[g].minrank;
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            fg.nodes[n].rank -= delta;
        }
        fg.graphs[g].minrank -= delta;
        fg.graphs[g].maxrank -= delta;
    }

    set_minmax(fg, g, do_root);

    // `agclose(Xg)`: nothing in the real graph may keep pointing into it.
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        fg.nodes[n].rep = None;
    }
}

/// `dot2_rank` — the `newrank=true` ranking path.
pub fn dot2_rank(fg: &mut Fg, g: GId) {
    let first_node = fg.nodes.len();
    let first_edge = fg.edges.len();

    edgelabel_ranks(fg, g);
    let maxiter = match fg.graphs[g].nslimit1 {
        Some(s) => scale_clamp(fg.graphs[g].nodes_order.len(), s),
        None => INT_MAX,
    };

    compile_samerank(fg, g, None);
    let mut xg = Xg {
        nodes: Vec::new(),
        weak_id: 0,
    };
    compile_nodes(fg, g, &mut xg);
    compile_edges(fg, g, &mut xg);
    let (top, bot) = (None, None);
    compile_clusters(fg, g, &mut xg, top, bot);
    break_cycles(fg, &xg);
    let ncc = connect_components(fg, &mut xg);
    // `add_fast_edges`: the port keeps ND_out/ND_in current as edges are made.

    let ssize = fg.graphs[g].search_size;
    let nodes = xg.nodes.clone();
    let _ = ns::rank2(fg, nodes, 1, maxiter, ssize, None);

    readout_levels(fg, g, ncc);

    fg.nodes.truncate(first_node);
    fg.edges.truncate(first_edge);
}
