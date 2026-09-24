//! Network simplex ranking — a faithful port of `lib/common/ns.c`.
//!
//! Shared by the rank phase (`rank(g, 1|0, maxiter)`) and the x-coordinate
//! phase (`rank(g, 2, nsiter2)`).
//!
//! The explicit-stack DFS functions mirror the C control flow exactly:
//! `tight_subtree_search`, `inter_tree_edge_search`, `dfs_cutval` and
//! `dfs_range*` keep their frames on the stack with resumable counters
//! (peek-with-counters), while `dfs_enter_*` pop their frames immediately
//! and push children mid-scan (pop-first). `rerank`/`tree_adjust` become
//! worklist walks: the per-node update (`rank ± delta`) is independent per
//! node, so any traversal order produces the identical result.
//!
//! ## Performance model
//!
//! The C original keeps `ND_rank`/`ND_low`/`ND_lim`/`ND_par` and
//! `ED_cutvalue`/`ED_tindex` in the arena structs; this port mirrors them
//! into dense SoA arrays owned by [`NsCtx`] (plus a CSR snapshot of the
//! aux-graph adjacency, which is static for the whole run). The simplex
//! touches these fields millions of times per layout; compact arrays turn
//! every read from a cache miss into a linear scan. The arena is the
//! source of truth only at the boundaries: a snapshot on construction and
//! a write-back before every return ([`Self::sync_back`]).

use super::model::{EId, Fg, NId, NodeType};

const SEARCHSIZE: usize = 30;

/// Sentinel for `None` in the SoA `par` array.
const NO_EDGE: usize = usize::MAX;

#[inline]
fn seq(a: i32, b: i32, c: i32) -> bool {
    a <= b && b <= c
}

/// The simplex context (`network_simplex_ctx_t`) — owns SoA mirrors of the
/// hot state and borrows the arena only for construction/write-back.
pub struct NsCtx<'a> {
    /// Arena (used for snapshots, tree-list protocol, write-back).
    pub fg: &'a mut Fg,
    /// Fast node list in `GD_nlist` order.
    nodes: Vec<NId>,
    tree_edge: Vec<EId>,
    s_i: usize,
    search_size: usize,

    // -- SoA node state (indexed by arena node id). The simplex hot loops
    //    mostly stream ONE field across many nodes (enter_edge's lim scans,
    //    leave_edge's cutvalue scan), so column arrays beat interleaved
    //    records here despite the tree walks touching several fields --
    n_rank: Vec<i32>,
    n_low: Vec<i32>,
    n_lim: Vec<i32>,
    /// `ND_par` — tree edge id, [`NO_EDGE`] = none.
    n_par: Vec<usize>,
    n_mark: Vec<u8>,
    /// static: `ND_node_type == NORMAL`
    n_normal: Vec<bool>,
    /// `ND_priority` — init_rank only.
    n_priority: Vec<i32>,
    // -- SoA edge state (indexed by arena edge id) --
    e_tail: Vec<usize>,
    e_head: Vec<usize>,
    e_minlen: Vec<i32>,
    e_weight: Vec<i32>,
    e_cutvalue: Vec<i32>,
    /// `ED_tindex` — tree membership index, -1 = non-tree.
    e_tidx: Vec<i32>,
    // -- CSR over the aux adjacency (static during the run) --
    out_start: Vec<u32>,
    out_edge: Vec<EId>,
    in_start: Vec<u32>,
    in_edge: Vec<EId>,
    // -- working tree lists (C's ND_tree_in/out elists) --
    tree_out: Vec<Vec<EId>>,
    tree_in: Vec<Vec<EId>>,
    // -- reusable DFS frame stacks (avoids per-pivot allocation churn) --
    scratch_range: Vec<(NId, Option<EId>, i32, usize, usize)>,
    scratch_cutval: Vec<(NId, Option<EId>, usize, usize)>,
    scratch_nodes: Vec<NId>,
}

impl<'a> NsCtx<'a> {
    /// Snapshot the arena's simplex-relevant state into the SoA arrays.
    fn snapshot(&mut self) {
        let nv = self.fg.nodes.len();
        self.n_rank = self.fg.nodes.iter().map(|n| n.rank).collect();
        self.n_low = self.fg.nodes.iter().map(|n| n.low).collect();
        self.n_lim = self.fg.nodes.iter().map(|n| n.lim).collect();
        self.n_par = self
            .fg
            .nodes
            .iter()
            .map(|n| n.par.map_or(NO_EDGE, |e| e))
            .collect();
        self.n_mark = self.fg.nodes.iter().map(|n| n.mark as u8).collect();
        self.n_normal = self
            .fg
            .nodes
            .iter()
            .map(|n| n.node_type == NodeType::Normal)
            .collect();
        self.n_priority = self.fg.nodes.iter().map(|n| n.priority).collect();
        self.e_tail = self.fg.edges.iter().map(|e| e.tail).collect();
        self.e_head = self.fg.edges.iter().map(|e| e.head).collect();
        self.e_minlen = self.fg.edges.iter().map(|e| e.minlen).collect();
        self.e_weight = self.fg.edges.iter().map(|e| e.weight).collect();
        self.e_cutvalue = self.fg.edges.iter().map(|e| e.cutvalue).collect();
        self.e_tidx = self.fg.edges.iter().map(|e| e.tree_index).collect();
        self.tree_out = vec![Vec::new(); nv];
        self.tree_in = vec![Vec::new(); nv];

        self.out_start.clear();
        self.out_start.push(0);
        self.in_start.clear();
        self.in_start.push(0);
        let mut io = 0usize;
        let mut ii = 0usize;
        self.out_edge.clear();
        self.in_edge.clear();
        for n in 0..nv {
            self.out_edge.extend_from_slice(&self.fg.nodes[n].out);
            io += self.fg.nodes[n].out.len();
            self.out_start.push(io as u32);
            self.in_edge.extend_from_slice(&self.fg.nodes[n].in_);
            ii += self.fg.nodes[n].in_.len();
            self.in_start.push(ii as u32);
        }
    }

    /// Write the SoA state back to the arena — every return path of
    /// [`rank2`] funnels through this, so the observable arena state is
    /// exactly what the all-arena version left behind.
    fn sync_back(&mut self) {
        for (i, n) in self.fg.nodes.iter_mut().enumerate() {
            n.rank = self.n_rank[i];
            n.low = self.n_low[i];
            n.lim = self.n_lim[i];
            n.par = if self.n_par[i] == NO_EDGE {
                None
            } else {
                Some(self.n_par[i])
            };
            n.mark = self.n_mark[i] as usize;
            n.priority = self.n_priority[i];
        }
        for (i, e) in self.fg.edges.iter_mut().enumerate() {
            e.cutvalue = self.e_cutvalue[i];
            e.tree_index = self.e_tidx[i];
        }
        // the working tree lists mirror C's elists; the arena copies stay
        // empty during the run and are cleared for the post-run protocol
        for n in self.fg.nodes.iter_mut() {
            n.tree_in.clear();
            n.tree_out.clear();
        }
    }

    #[inline]
    fn length(&self, e: EId) -> i32 {
        self.n_rank[self.e_head[e]] - self.n_rank[self.e_tail[e]]
    }

    #[inline]
    fn slack(&self, e: EId) -> i32 {
        self.length(e) - self.e_minlen[e]
    }

    #[inline]
    fn is_tree_edge(&self, e: EId) -> bool {
        self.e_tidx[e] >= 0
    }

    fn add_tree_edge(&mut self, e: EId) {
        debug_assert!(
            !self.is_tree_edge(e),
            "add_tree_edge: already a tree edge"
        );
        self.e_tidx[e] = self.tree_edge.len() as i32;
        self.tree_edge.push(e);
        let n = self.e_tail[e] as usize;
        self.n_mark[n] = 1;
        self.tree_out[n].push(e);
        let n = self.e_head[e] as usize;
        self.n_mark[n] = 1;
        self.tree_in[n].push(e);
    }

    /// `invalidate_path` — mark ND_low = -1 walking up from to_node toward
    /// lca.
    fn invalidate_path(&mut self, lca: NId, mut to_node: NId) {
        loop {
            if self.n_low[to_node] == -1 {
                break;
            }
            self.n_low[to_node] = -1;
            let e = self.n_par[to_node];
            if e == NO_EDGE {
                break;
            }
            let e = e as EId;
            if self.n_lim[to_node] >= self.n_lim[lca] {
                if to_node != lca {
                    // "invalidate_path: skipped over LCA"
                }
                break;
            }
            to_node = if self.n_lim[self.e_tail[e]] > self.n_lim[self.e_head[e]] {
                self.e_tail[e]
            } else {
                self.e_head[e]
            };
        }
    }

    /// `exchange_tree_edges` — swap-with-last removal, exactly like C.
    fn exchange_tree_edges(&mut self, e: EId, f: EId) {
        let idx = self.e_tidx[e];
        self.e_tidx[f] = idx;
        self.tree_edge[idx as usize] = f;
        self.e_tidx[e] = -1;

        let n = self.e_tail[e];
        {
            let list = &mut self.tree_out[n];
            let i = list.len() - 1;
            let j = list.iter().position(|&x| x == e).expect("tree edge missing");
            list.swap(j, i);
            list.pop();
        }
        let n = self.e_head[e];
        {
            let list = &mut self.tree_in[n];
            let i = list.len() - 1;
            let j = list.iter().position(|&x| x == e).expect("tree edge missing");
            list.swap(j, i);
            list.pop();
        }
        let n = self.e_tail[f];
        self.tree_out[n].push(f);
        let n = self.e_head[f];
        self.tree_in[n].push(f);
    }

    /// `init_rank` — longest-path ranks via topological traversal.
    fn init_rank(&mut self) {
        let mut queue: std::collections::VecDeque<NId> = Default::default();
        for &v in &self.nodes {
            if self.n_priority[v] == 0 {
                queue.push_back(v);
            }
        }
        while let Some(v) = queue.pop_front() {
            self.n_rank[v] = 0;
            for k in self.in_start[v] as usize..self.in_start[v + 1] as usize {
                let e = self.in_edge[k];
                let t = self.e_tail[e];
                let r = self.n_rank[t] + self.e_minlen[e];
                self.n_rank[v] = self.n_rank[v].max(r);
            }
            for k in self.out_start[v] as usize..self.out_start[v + 1] as usize {
                let e = self.out_edge[k];
                let h = self.e_head[e];
                self.n_priority[h] -= 1;
                if self.n_priority[h] <= 0 {
                    queue.push_back(h);
                }
            }
        }
        // ctr != N_nodes → "trouble in init_rank" (cyclic aux graph)
    }

    /// `leave_edge` — round-robin scan for a negative-cut tree edge.
    fn leave_edge(&mut self) -> Option<EId> {
        let mut rv: Option<EId> = None;
        let mut cnt = 0usize;

        let j = self.s_i;
        while self.s_i < self.tree_edge.len() {
            let f = self.tree_edge[self.s_i];
            if self.e_cutvalue[f] < 0 {
                if let Some(r) = rv {
                    if self.e_cutvalue[r] > self.e_cutvalue[f] {
                        rv = Some(f);
                    }
                } else {
                    rv = Some(f);
                }
                cnt += 1;
                if cnt >= self.search_size {
                    return rv;
                }
            }
            self.s_i += 1;
        }
        if j > 0 {
            self.s_i = 0;
            while self.s_i < j {
                let f = self.tree_edge[self.s_i];
                if self.e_cutvalue[f] < 0 {
                    if let Some(r) = rv {
                        if self.e_cutvalue[r] > self.e_cutvalue[f] {
                            rv = Some(f);
                        }
                    } else {
                        rv = Some(f);
                    }
                    cnt += 1;
                    if cnt >= self.search_size {
                        return rv;
                    }
                }
                self.s_i += 1;
            }
        }
        rv
    }

    /// `dfs_enter_outedge` — pop-first DFS over out edges, seeding further
    /// tree-in expansion only while the best slack is positive.
    fn dfs_enter_outedge(&self, start: NId, low: i32, lim: i32) -> Option<EId> {
        let mut enter: Option<EId> = None;
        let mut slack_best = i32::MAX;

        let mut todo = vec![start];
        while let Some(v) = todo.pop() {
            for k in self.out_start[v] as usize..self.out_start[v + 1] as usize {
                let e = self.out_edge[k];
                if !self.is_tree_edge(e) {
                    let l = self.n_lim[self.e_head[e]];
                    if !seq(low, l, lim) {
                        let s = self.slack(e);
                        if s < slack_best || enter.is_none() {
                            enter = Some(e);
                            slack_best = s;
                        }
                    }
                } else if self.n_lim[self.e_head[e]] < self.n_lim[v] {
                    todo.push(self.e_head[e]);
                }
            }
            if slack_best > 0 {
                for i in 0..self.tree_in[v].len() {
                    let e = self.tree_in[v][i];
                    let t = self.e_tail[e];
                    if self.n_lim[t] < self.n_lim[v] {
                        todo.push(t);
                    }
                }
            }
        }
        enter
    }

    /// `dfs_enter_inedge`.
    fn dfs_enter_inedge(&self, start: NId, low: i32, lim: i32) -> Option<EId> {
        let mut enter: Option<EId> = None;
        let mut slack_best = i32::MAX;

        let mut todo = vec![start];
        while let Some(v) = todo.pop() {
            for k in self.in_start[v] as usize..self.in_start[v + 1] as usize {
                let e = self.in_edge[k];
                if !self.is_tree_edge(e) {
                    let l = self.n_lim[self.e_tail[e]];
                    if !seq(low, l, lim) {
                        let s = self.slack(e);
                        if s < slack_best || enter.is_none() {
                            enter = Some(e);
                            slack_best = s;
                        }
                    }
                } else if self.n_lim[self.e_tail[e]] < self.n_lim[v] {
                    todo.push(self.e_tail[e]);
                }
            }
            if slack_best > 0 {
                for i in 0..self.tree_out[v].len() {
                    let e = self.tree_out[v][i];
                    let h = self.e_head[e];
                    if self.n_lim[h] < self.n_lim[v] {
                        todo.push(h);
                    }
                }
            }
        }
        enter
    }

    /// `enter_edge` — search from the down node of tree edge e.
    fn enter_edge(&self, e: EId) -> Option<EId> {
        let (t, h) = (self.e_tail[e], self.e_head[e]);
        let (v, outsearch) = if self.n_lim[t] < self.n_lim[h] {
            (t, false)
        } else {
            (h, true)
        };
        let (low, lim) = (self.n_low[v], self.n_lim[v]);
        if outsearch {
            self.dfs_enter_outedge(v, low, lim)
        } else {
            self.dfs_enter_inedge(v, low, lim)
        }
    }

    // -- tight tree construction ------------------------------------------

    /// `tight_subtree_search` — grows the tight subtree containing v via
    /// zero-slack non-tree edges. Returns the subtree size.
    fn tight_subtree_search(&mut self, v: NId, st: usize, subtree_of: &mut [Option<usize>]) -> i32 {
        let mut rv: i32 = 1;
        subtree_of[v] = Some(st);

        // frames: (v, in_i, out_i, rv)
        let mut todo: Vec<(NId, usize, usize, i32)> = vec![(v, 0, 0, 1)];

        while !todo.is_empty() {
            let top = todo.len() - 1;
            let fv = todo[top].0;
            let mut updated = false;

            // in edges
            while todo[top].1 < self.in_start[fv + 1] as usize - self.in_start[fv] as usize {
                let i = self.in_start[fv] as usize + todo[top].1;
                let e = self.in_edge[i];
                if self.is_tree_edge(e) {
                    todo[top].1 += 1;
                    continue;
                }
                let t = self.e_tail[e];
                if subtree_of[t].is_none() && self.slack(e) == 0 {
                    todo[top].1 += 1;
                    self.add_tree_edge(e);
                    subtree_of[t] = Some(st);
                    todo.push((t, 0, 0, 1));
                    updated = true;
                    break;
                }
                todo[top].1 += 1;
            }
            if updated {
                continue;
            }

            // out edges
            while todo[top].2 < self.out_start[fv + 1] as usize - self.out_start[fv] as usize {
                let i = self.out_start[fv] as usize + todo[top].2;
                let e = self.out_edge[i];
                if self.is_tree_edge(e) {
                    todo[top].2 += 1;
                    continue;
                }
                let h = self.e_head[e];
                if subtree_of[h].is_none() && self.slack(e) == 0 {
                    todo[top].2 += 1;
                    self.add_tree_edge(e);
                    subtree_of[h] = Some(st);
                    todo.push((h, 0, 0, 1));
                    updated = true;
                    break;
                }
                todo[top].2 += 1;
            }
            if updated {
                continue;
            }

            // pop
            let (_, _, _, last_rv) = todo.pop().unwrap();
            match todo.last_mut() {
                None => rv = last_rv,
                Some(parent) => parent.3 += last_rv,
            }
        }
        rv
    }

    /// `STsetFind` over the subtree arena.
    fn st_set_find(subtrees: &[Subtree], mut s: usize) -> usize {
        while let Some(p) = subtrees[s].par {
            if p == s {
                break;
            }
            s = p;
        }
        s
    }

    /// `STheapify` over the subtree arena.
    fn st_heapify(subtrees: &mut [Subtree], heap: &mut [usize], mut i: usize) {
        let size = heap.len();
        loop {
            if i >= size {
                break;
            }
            let left = 2 * (i + 1) - 1;
            let right = 2 * (i + 1);
            let mut smallest = i;
            if left < size && subtrees[heap[left]].size < subtrees[heap[smallest]].size {
                smallest = left;
            }
            if right < size && subtrees[heap[right]].size < subtrees[heap[smallest]].size {
                smallest = right;
            }
            if smallest != i {
                heap.swap(i, smallest);
                subtrees[heap[i]].heap_index = i;
                subtrees[heap[smallest]].heap_index = smallest;
                i = smallest;
            } else {
                break;
            }
        }
    }

    /// `merge_trees` — move the non-heap side, add the entering edge.
    fn merge_trees(
        &mut self,
        e: EId,
        subtrees: &mut [Subtree],
        subtree_of: &mut [Option<usize>],
    ) -> usize {
        debug_assert!(!self.is_tree_edge(e));
        let t0 = Self::st_set_find(subtrees, subtree_of[self.e_tail[e]].unwrap());
        let t1 = Self::st_set_find(subtrees, subtree_of[self.e_head[e]].unwrap());

        if !subtrees[t0].on_heap() {
            let delta = self.slack(e);
            if delta != 0 {
                let rep = subtrees[t0].rep;
                self.tree_adjust(rep, None, delta);
            }
        } else {
            let delta = -self.slack(e);
            if delta != 0 {
                let rep = subtrees[t1].rep;
                self.tree_adjust(rep, None, delta);
            }
        }
        self.add_tree_edge(e);
        Self::st_set_union(subtrees, t0, t1)
    }

    /// `STsetUnion` — on-heap-first root selection, then smaller size.
    fn st_set_union(subtrees: &mut [Subtree], s0: usize, s1: usize) -> usize {
        let mut r0 = s0;
        while let Some(p) = subtrees[r0].par {
            if p == r0 {
                break;
            }
            r0 = p;
        }
        let mut r1 = s1;
        while let Some(p) = subtrees[r1].par {
            if p == r1 {
                break;
            }
            r1 = p;
        }
        if r0 == r1 {
            return r0;
        }
        let r = if !subtrees[r1].on_heap() {
            r0
        } else if !subtrees[r0].on_heap() {
            r1
        } else if subtrees[r1].size < subtrees[r0].size {
            r0
        } else {
            r1
        };
        subtrees[r0].par = Some(r);
        subtrees[r1].par = Some(r);
        subtrees[r].size = subtrees[r0].size + subtrees[r1].size;
        r
    }

    /// `inter_tree_edge_search` — tightest edge leaving the subtree.
    fn inter_tree_edge_search(
        &self,
        v: NId,
        subtree_of: &[Option<usize>],
        subtrees: &[Subtree],
    ) -> Option<EId> {
        // C compares `STsetFind(...)` results — the *current* union-find root
        // of each node, not the original tight-subtree id. Comparing the
        // original ids makes two already-merged subtrees look distinct, which
        // returns an edge inside the merged tree and builds a cycle.
        let root_of = |n: NId| -> usize {
            Self::st_set_find(subtrees, subtree_of[n].expect("node in subtree"))
        };
        let ts0 = root_of(v);
        let mut best: Option<EId> = None;

        // frames: (v, ts, from, out_i, in_i)
        let mut todo: Vec<(NId, usize, Option<NId>, usize, usize)> = vec![(v, ts0, None, 0, 0)];

        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fts, ffrom, _, _) = todo[top];

            if todo[top].3 == 0 && todo[top].4 == 0 {
                if let Some(b) = best {
                    if self.slack(b) == 0 {
                        todo.pop();
                        continue;
                    }
                }
            }

            let mut updated = false;

            // out edges
            let (fo, lo) = (
                self.out_start[fv] as usize,
                self.out_start[fv + 1] as usize,
            );
            while todo[top].3 < lo - fo {
                let e = self.out_edge[fo + todo[top].3];
                if self.is_tree_edge(e) {
                    let h = self.e_head[e];
                    if Some(h) == ffrom {
                        todo[top].3 += 1;
                        continue; // do not search back in tree
                    }
                    todo[top].3 += 1;
                    let hts = root_of(h);
                    todo.push((h, hts, Some(fv), 0, 0)); // search forward in tree
                    updated = true;
                    break;
                }
                let h = self.e_head[e];
                if root_of(h) != fts {
                    // encountered candidate edge
                    if best.is_none() || self.slack(e) < self.slack(best.unwrap()) {
                        best = Some(e);
                    }
                }
                todo[top].3 += 1;
                // else ignore non-tree edge between nodes in the same tree
            }
            if updated {
                continue;
            }

            // in edges (mirrors the above, for in-edges)
            let (fi, li) = (self.in_start[fv] as usize, self.in_start[fv + 1] as usize);
            while todo[top].4 < li - fi {
                let e = self.in_edge[fi + todo[top].4];
                if self.is_tree_edge(e) {
                    let t = self.e_tail[e];
                    if Some(t) == ffrom {
                        todo[top].4 += 1;
                        continue;
                    }
                    todo[top].4 += 1;
                    let tts = root_of(t);
                    todo.push((t, tts, Some(fv), 0, 0));
                    updated = true;
                    break;
                }
                let t = self.e_tail[e];
                if root_of(t) != fts {
                    if best.is_none() || self.slack(e) < self.slack(best.unwrap()) {
                        best = Some(e);
                    }
                }
                todo[top].4 += 1;
            }
            if updated {
                continue;
            }

            todo.pop();
        }
        best
    }

    /// `tree_adjust` — shift the whole subtree containing v by delta.
    ///
    /// C recurses; the per-node update is independent, so a worklist walk
    /// produces the identical ranks (and cannot overflow the stack on deep
    /// trees).
    fn tree_adjust(&mut self, v: NId, from: Option<NId>, delta: i32) {
        // C's recursion marks `from` per frame; encode it as the edge id
        // came-from via a parallel stack of (node, came_from) pairs.
        let mut todo: Vec<(NId, Option<NId>)> = Vec::new();
        todo.push((v, from));
        while let Some((v, from)) = todo.pop() {
            self.n_rank[v] += delta;
            for i in 0..self.tree_in[v].len() {
                let e = self.tree_in[v][i];
                let w = self.e_tail[e];
                if Some(w) != from {
                    todo.push((w, Some(v)));
                }
            }
            for i in 0..self.tree_out[v].len() {
                let e = self.tree_out[v][i];
                let w = self.e_head[e];
                if Some(w) != from {
                    todo.push((w, Some(v)));
                }
            }
        }
    }

    /// `feasible_tree` — construct the initial tight spanning tree and cut
    /// values. Error codes mirror C: 1 = disconnected, 2 = bad.
    fn feasible_tree(&mut self) -> Result<(), i32> {
        let n_nodes = self.nodes.len();
        let mut subtrees: Vec<Subtree> = Vec::new();
        let mut subtree_of: Vec<Option<usize>> = vec![None; self.fg.nodes.len()];
        let mut tree: Vec<usize> = Vec::new();

        // given init_rank, find all tight subtrees
        for &n in self.nodes.clone().iter() {
            if subtree_of[n].is_none() {
                let st = subtrees.len();
                subtrees.push(Subtree {
                    rep: n,
                    size: 0,
                    heap_index: usize::MAX,
                    par: Some(st),
                });
                let size = self.tight_subtree_search(n, st, &mut subtree_of);
                if size < 0 {
                    return Err(2);
                }
                subtrees[st].size = size;
                subtrees[st].par = Some(st);
                tree.push(st);
            }
        }

        // incrementally merge subtrees (STbuildheap: size/2 down to 0)
        let mut heap = tree.clone();
        for (i, &st) in heap.iter().enumerate() {
            subtrees[st].heap_index = i;
        }
        if !heap.is_empty() {
            for i in (0..=heap.len() / 2).rev() {
                Self::st_heapify(&mut subtrees, &mut heap, i);
            }
        }

        while heap.len() > 1 {
            // STextractmin
            let tree0 = heap[0];
            subtrees[tree0].heap_index = usize::MAX;
            let last = heap.len() - 1;
            heap.swap(0, last);
            heap.truncate(last);
            if !heap.is_empty() {
                subtrees[heap[0]].heap_index = 0;
                Self::st_heapify(&mut subtrees, &mut heap, 0);
            }

            let rep = subtrees[tree0].rep;
            let Some(ee) = self.inter_tree_edge_search(rep, &subtree_of, &subtrees) else {
                return Err(1);
            };
            let tree1 = self.merge_trees(ee, &mut subtrees, &mut subtree_of);
            let hi = subtrees[tree1].heap_index;
            Self::st_heapify(&mut subtrees, &mut heap, hi);
        }

        // A disconnected input leaves a *forest* (or, if a merge ever produced
        // a cycle, an over-connected blob): the DFS in `init_cutvalues` cannot
        // terminate on such a structure, and later `treeupdate` walks would
        // fall off a dangling subtree. C asserts a spanning tree here and
        // reports 1 ("graph was not connected") from `rank()` when the heap
        // runs dry; walk the tree lists with a visited set *before* the DFS so
        // a broken tree degrades into the caller's `connectGraph` + retry path
        // instead of looping forever. (A visited set is what makes the walk
        // terminate on a cycle.)
        let root = self.nodes[0];
        let mut seen = vec![false; self.fg.nodes.len()];
        seen[root] = true;
        let mut count = 1usize;
        let mut stack = vec![root];
        while let Some(v) = stack.pop() {
            for i in 0..self.tree_out[v].len() {
                let e = self.tree_out[v][i];
                let h = self.e_head[e];
                if !seen[h] {
                    seen[h] = true;
                    count += 1;
                    stack.push(h);
                }
            }
            for i in 0..self.tree_in[v].len() {
                let e = self.tree_in[v][i];
                let t = self.e_tail[e];
                if !seen[t] {
                    seen[t] = true;
                    count += 1;
                    stack.push(t);
                }
            }
        }
        if count != n_nodes || self.tree_edge.len() != n_nodes.saturating_sub(1) {
            return Err(1);
        }
        self.init_cutvalues();
        Ok(())
    }

    /// `treeupdate` — walk up from v to the LCA with w, updating cutvalues.
    fn treeupdate(&mut self, mut v: NId, w: NId, cutvalue: i32, dir: bool) -> NId {
        while !seq(self.n_low[v], self.n_lim[w], self.n_lim[v]) {
            // A dangling `par` means the tree is a forest (disconnected aux
            // graph); `feasible_tree` already reports that as Err(1).
            let e = self.n_par[v];
            if e == NO_EDGE {
                break;
            }
            let e = e as EId;
            let d = if self.e_tail[e] == v {
                dir
            } else {
                !dir
            };
            if d {
                self.e_cutvalue[e] += cutvalue;
            } else {
                self.e_cutvalue[e] -= cutvalue;
            }
            v = if self.n_lim[self.e_tail[e]] > self.n_lim[self.e_head[e]] {
                self.e_tail[e]
            } else {
                self.e_head[e]
            };
        }
        v
    }

    /// `rerank` — shift v's tree subtree by -delta.
    ///
    /// C recurses; the per-node shift is independent, so a worklist walk
    /// yields the identical ranks without recursion-depth limits.
    fn rerank(&mut self, v: NId, delta: i32) {
        let mut todo = std::mem::take(&mut self.scratch_nodes);
        todo.clear();
        todo.push(v);
        while let Some(v) = todo.pop() {
            self.n_rank[v] -= delta;
            for i in 0..self.tree_out[v].len() {
                let e = self.tree_out[v][i];
                if self.n_par[v] == NO_EDGE || Some(e) != Some(self.n_par[v] as EId) {
                    let h = self.e_head[e];
                    todo.push(h);
                }
            }
            for i in 0..self.tree_in[v].len() {
                let e = self.tree_in[v][i];
                if self.n_par[v] == NO_EDGE || Some(e) != Some(self.n_par[v] as EId) {
                    let t = self.e_tail[e];
                    todo.push(t);
                }
            }
        }
    }

    /// `update` — exchange leaving tree edge e for entering non-tree edge f.
    fn update(&mut self, e: EId, f: EId) -> Result<(), i32> {
        let delta = self.slack(f);
        if delta > 0 {
            let te = self.e_tail[e];
            let he = self.e_head[e];
            let s = self.tree_in[te].len() + self.tree_out[te].len();
            if s == 1 {
                self.rerank(te, delta);
            } else {
                let s = self.tree_in[he].len() + self.tree_out[he].len();
                if s == 1 {
                    self.rerank(he, -delta);
                } else if self.n_lim[te] < self.n_lim[he] {
                    self.rerank(te, delta);
                } else {
                    self.rerank(he, -delta);
                }
            }
        }

        let cutvalue = self.e_cutvalue[e];
        let (ft, fh) = (self.e_tail[f], self.e_head[f]);
        let lca = self.treeupdate(ft, fh, cutvalue, true);
        if self.treeupdate(fh, ft, cutvalue, false) != lca {
            // "update: mismatched lca in treeupdates"
            return Err(2);
        }

        let lca_low = self.n_low[lca];
        self.invalidate_path(lca, fh);
        self.invalidate_path(lca, ft);

        self.e_cutvalue[f] = -cutvalue;
        self.e_cutvalue[e] = 0;
        self.exchange_tree_edges(e, f);
        let par = self.n_par[lca];
        let par = if par == NO_EDGE { None } else { Some(par) };
        self.dfs_range(lca, par, lca_low);
        Ok(())
    }

    /// `x_val` (ns.c).
    fn x_val(&self, e: EId, v: NId, dir: i32) -> i32 {
        let (t, h) = (self.e_tail[e], self.e_head[e]);
        let other = if t == v { h } else { t };
        let (f, mut rv);
        if !seq(self.n_low[v], self.n_lim[other], self.n_lim[v]) {
            f = 1;
            rv = self.e_weight[e];
        } else {
            f = 0;
            rv = if self.is_tree_edge(e) {
                self.e_cutvalue[e]
            } else {
                0
            };
            rv -= self.e_weight[e];
        }
        let mut d = if dir > 0 {
            if h == v {
                1
            } else {
                -1
            }
        } else if t == v {
            1
        } else {
            -1
        };
        if f == 1 {
            d = -d;
        }
        if d < 0 {
            rv = -rv;
        }
        rv
    }

    /// `x_cutval` — set the cut value of f assuming one side is done.
    fn x_cutval(&mut self, f: EId) {
        let (ft, fh) = (self.e_tail[f], self.e_head[f]);
        let (v, dir) = if self.n_par[ft] == f as usize {
            (ft, 1)
        } else {
            (fh, -1)
        };

        let mut sum: i64 = 0;
        for k in self.out_start[v] as usize..self.out_start[v + 1] as usize {
            let e = self.out_edge[k];
            sum += self.x_val(e, v, dir) as i64;
        }
        for k in self.in_start[v] as usize..self.in_start[v + 1] as usize {
            let e = self.in_edge[k];
            sum += self.x_val(e, v, dir) as i64;
        }
        self.e_cutvalue[f] = sum as i32;
    }

    /// `dfs_cutval` — post-order cut value computation (peek-with-counters).
    fn dfs_cutval(&mut self, v: NId, par: Option<EId>) {
        // frames: (v, par, out_i, in_i)
        let mut todo = std::mem::take(&mut self.scratch_cutval);
        todo.clear();
        todo.push((v, par, 0, 0));
        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, _, _) = todo[top];
            let mut updated = false;

            while todo[top].2 < self.tree_out[fv].len() {
                let i = todo[top].2;
                let e = self.tree_out[fv][i];
                todo[top].2 += 1;
                if Some(e) != fpar {
                    let h = self.e_head[e];
                    todo.push((h, Some(e), 0, 0));
                    updated = true;
                    break;
                }
            }
            if updated {
                continue;
            }

            while todo[top].3 < self.tree_in[fv].len() {
                let i = todo[top].3;
                let e = self.tree_in[fv][i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let t = self.e_tail[e];
                    todo.push((t, Some(e), 0, 0));
                    updated = true;
                    break;
                }
            }
            if updated {
                continue;
            }

            let (_, fpar, _, _) = todo.pop().unwrap();
            if let Some(p) = fpar {
                self.x_cutval(p);
            }
        }
        self.scratch_cutval = todo;
    }

    /// `dfs_range_init` — initial par/low/lim assignment (peek-with-counters).
    fn dfs_range_init(&mut self, v: NId) -> i32 {
        let mut lim: i32 = 0;
        self.n_par[v] = NO_EDGE;
        self.n_low[v] = 1;
        // frames: (v, par, lim, tree_out_i, tree_in_i)
        let mut todo = std::mem::take(&mut self.scratch_range);
        todo.clear();
        todo.push((v, None, 1, 0, 0));
        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, flim, _, _) = todo[top];
            let mut pushed_new = false;

            while todo[top].3 < self.tree_out[fv].len() {
                let i = todo[top].3;
                let e = self.tree_out[fv][i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let n = self.e_head[e];
                    self.n_par[n] = e as usize;
                    self.n_low[n] = flim;
                    todo.push((n, Some(e), flim, 0, 0));
                    pushed_new = true;
                    break;
                }
            }
            if pushed_new {
                continue;
            }

            while todo[top].4 < self.tree_in[fv].len() {
                let i = todo[top].4;
                let e = self.tree_in[fv][i];
                todo[top].4 += 1;
                if Some(e) != fpar {
                    let n = self.e_tail[e];
                    self.n_par[n] = e as usize;
                    self.n_low[n] = flim;
                    todo.push((n, Some(e), flim, 0, 0));
                    pushed_new = true;
                    break;
                }
            }
            if pushed_new {
                continue;
            }

            let (pv, _, plim, _, _) = todo.pop().unwrap();
            self.n_lim[pv] = plim;
            lim = plim;
            if let Some(parent) = todo.last_mut() {
                parent.2 = lim + 1;
            }
        }
        self.scratch_range = todo;
        lim + 1
    }

    /// `dfs_range` — incremental par/low/lim update.
    fn dfs_range(&mut self, v: NId, par: Option<EId>, low: i32) -> i32 {
        let mut lim: i32 = 0;

        if self.n_par[v] == par.map_or(NO_EDGE, |e| e as usize) && self.n_low[v] == low {
            return self.n_lim[v] + 1;
        }

        self.n_par[v] = par.map_or(NO_EDGE, |e| e as usize);
        self.n_low[v] = low;
        let mut todo: Vec<(NId, Option<EId>, i32, usize, usize)> = vec![(v, par, low, 0, 0)];

        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, mut flim, _, _) = todo[top];
            let mut processed_child = false;

            while todo[top].3 < self.tree_out[fv].len() {
                let i = todo[top].3;
                let e = self.tree_out[fv][i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let n = self.e_head[e];
                    if self.n_par[n] == e as usize && self.n_low[n] == flim {
                        flim = self.n_lim[n] + 1;
                        todo[top].2 = flim;
                    } else {
                        self.n_par[n] = e as usize;
                        self.n_low[n] = flim;
                        todo.push((n, Some(e), flim, 0, 0));
                    }
                    processed_child = true;
                    break;
                }
            }
            if processed_child {
                continue;
            }

            while todo[top].4 < self.tree_in[fv].len() {
                let i = todo[top].4;
                let e = self.tree_in[fv][i];
                todo[top].4 += 1;
                if Some(e) != fpar {
                    let n = self.e_tail[e];
                    if self.n_par[n] == e as usize && self.n_low[n] == flim {
                        flim = self.n_lim[n] + 1;
                        todo[top].2 = flim;
                    } else {
                        self.n_par[n] = e as usize;
                        self.n_low[n] = flim;
                        todo.push((n, Some(e), flim, 0, 0));
                    }
                    processed_child = true;
                    break;
                }
            }
            if processed_child {
                continue;
            }

            let (pv, _, plim, _, _) = todo.pop().unwrap();
            self.n_lim[pv] = plim;
            lim = plim;
            if let Some(parent) = todo.last_mut() {
                parent.2 = lim + 1;
            }
        }
        self.scratch_range = todo;
        lim + 1
    }

    /// `init_cutvalues`.
    fn init_cutvalues(&mut self) {
        let root = self.nodes[0];
        self.dfs_range_init(root);
        self.dfs_cutval(root, None);
    }

    /// `scan_and_normalize` — shift so minrank(NORMAL) = 0; returns maxrank.
    fn scan_and_normalize(&mut self) -> i32 {
        let mut minrank = i32::MAX;
        let mut maxrank = i32::MIN;
        for &n in &self.nodes {
            if self.n_normal[n] {
                minrank = minrank.min(self.n_rank[n]);
                maxrank = maxrank.max(self.n_rank[n]);
            }
        }
        for &n in &self.nodes {
            self.n_rank[n] -= minrank;
        }
        maxrank - minrank
    }

    /// `freeTreeList`.
    fn free_tree_list(&mut self) {
        for &n in &self.nodes {
            self.tree_in[n].clear();
            self.tree_out[n].clear();
            self.n_mark[n] = 0;
        }
        self.tree_edge.clear();
    }

    /// `LR_balance` (balance == 2).
    fn lr_balance(&mut self) {
        for i in 0..self.tree_edge.len() {
            let e = self.tree_edge[i];
            if self.e_cutvalue[e] == 0 {
                let Some(f) = self.enter_edge(e) else {
                    continue;
                };
                let delta = self.slack(f);
                if delta <= 1 {
                    continue;
                }
                let (te, he) = (self.e_tail[e], self.e_head[e]);
                if self.n_lim[te] < self.n_lim[he] {
                    self.rerank(te, delta / 2);
                } else {
                    self.rerank(he, -delta / 2);
                }
            }
        }
        self.free_tree_list();
    }

    /// `TB_balance` (balance == 1); `tbbalance` mirrors the graph attribute
    /// ("min"/"max" or none).
    fn tb_balance(&mut self, tbbalance: Option<&str>) {
        let mut adj = 0;
        let maxrank = self.scan_and_normalize();
        debug_assert!(maxrank >= 0);

        let mut nrank = vec![0i32; (maxrank.max(0) + 1) as usize];
        if let Some(s) = tbbalance {
            if s == "min" {
                adj = 1;
            } else if s == "max" {
                adj = 2;
            }
            if adj != 0 {
                for &n in &self.nodes {
                    if self.n_normal[n] {
                        let in_empty =
                            self.in_start[n + 1] == self.in_start[n];
                        let out_empty =
                            self.out_start[n + 1] == self.out_start[n];
                        if in_empty && adj == 1 {
                            self.n_rank[n] = 0;
                        }
                        if out_empty && adj == 2 {
                            self.n_rank[n] = maxrank;
                        }
                    }
                }
            }
        }

        let mut tree_node: Vec<NId> = self.nodes.clone();
        if adj > 1 {
            tree_node.sort_by(|a, b| self.n_rank[*b].cmp(&self.n_rank[*a]));
        } else {
            tree_node.sort_by(|a, b| self.n_rank[*a].cmp(&self.n_rank[*b]));
        }
        for &n in &tree_node {
            if self.n_normal[n] {
                nrank[self.n_rank[n] as usize] += 1;
            }
        }
        for &n in tree_node.clone().iter() {
            if !self.n_normal[n] {
                continue;
            }
            let mut inweight = 0i32;
            let mut outweight = 0i32;
            let mut low = 0i32;
            let mut high = maxrank;
            for k in self.in_start[n] as usize..self.in_start[n + 1] as usize {
                let e = self.in_edge[k];
                inweight += self.e_weight[e];
                let r = self.n_rank[self.e_tail[e]] + self.e_minlen[e];
                low = low.max(r);
            }
            for k in self.out_start[n] as usize..self.out_start[n + 1] as usize {
                let e = self.out_edge[k];
                outweight += self.e_weight[e];
                let r = self.n_rank[self.e_head[e]] - self.e_minlen[e];
                high = high.min(r);
            }
            if low < 0 {
                low = 0; // vnodes can have ranks < 0
            }
            if adj != 0 {
                if inweight == outweight {
                    self.n_rank[n] = if adj == 1 { low } else { high };
                }
            } else if inweight == outweight {
                let mut choice = low;
                for i in (low + 1)..=high {
                    if nrank[i as usize] < nrank[choice as usize] {
                        choice = i;
                    }
                }
                nrank[self.n_rank[n] as usize] -= 1;
                nrank[choice as usize] += 1;
                self.n_rank[n] = choice;
            }
            self.tree_in[n].clear();
            self.tree_out[n].clear();
            self.n_mark[n] = 0;
        }
    }

    /// `init_graph` — counts, priorities, feasibility.
    fn init_graph(&mut self) -> bool {
        let mut feasible = true;
        for &n in &self.nodes {
            self.n_mark[n] = 0;
            self.n_priority[n] = 0;
            for k in self.in_start[n] as usize..self.in_start[n + 1] as usize {
                let e = self.in_edge[k];
                self.n_priority[n] += 1;
                self.e_cutvalue[e] = 0;
                self.e_tidx[e] = -1;
                let (t, h) = (self.e_tail[e], self.e_head[e]);
                if self.n_rank[h] - self.n_rank[t] < self.e_minlen[e] {
                    feasible = false;
                }
            }
            self.tree_in[n].clear();
            self.tree_out[n].clear();
        }
        feasible
    }
}

/// Subtree arena entry (`subtree_t`).
struct Subtree {
    rep: NId,
    size: i32,
    /// `usize::MAX` mirrors `SIZE_MAX` = not on the heap.
    heap_index: usize,
    par: Option<usize>,
}

impl Subtree {
    fn on_heap(&self) -> bool {
        self.heap_index != usize::MAX
    }
}

/// `rank2` — apply network simplex to the given fast node list.
///
/// * `balance` — 1 = TB_balance, 2 = LR_balance, other = normalize only.
/// * `maxiter` — iteration cap (`INT_MAX` unless nslimit/nslimit1).
/// * `search_size` — `-1` for the built-in `SEARCHSIZE`.
pub fn rank2(
    fg: &mut Fg,
    nodes: Vec<NId>,
    balance: i32,
    maxiter: i32,
    search_size: i32,
    tbbalance: Option<&str>,
) -> Result<(), i32> {
    if nodes.is_empty() {
        return Ok(());
    }
    let mut ctx = NsCtx {
        fg,
        nodes,
        tree_edge: Vec::new(),
        s_i: 0,
        search_size: if search_size >= 0 {
            search_size as usize
        } else {
            SEARCHSIZE
        },
        n_rank: Vec::new(),
        n_low: Vec::new(),
        n_lim: Vec::new(),
        n_par: Vec::new(),
        n_mark: Vec::new(),
        n_normal: Vec::new(),
        n_priority: Vec::new(),
        e_tail: Vec::new(),
        e_head: Vec::new(),
        e_minlen: Vec::new(),
        e_weight: Vec::new(),
        e_cutvalue: Vec::new(),
        e_tidx: Vec::new(),
        scratch_range: Vec::new(),
        scratch_cutval: Vec::new(),
        scratch_nodes: Vec::new(),
        out_start: Vec::new(),
        out_edge: Vec::new(),
        in_start: Vec::new(),
        in_edge: Vec::new(),
        tree_out: Vec::new(),
        tree_in: Vec::new(),
    };
    ctx.snapshot();

    let run = |ctx: &mut NsCtx| -> Result<(), i32> {
        let feasible = ctx.init_graph();
        if !feasible {
            ctx.init_rank();
        }

        ctx.feasible_tree()?;
        if maxiter <= 0 {
            return Ok(());
        }

        let mut iter = 0;
        let timing = std::env::var_os("GD_TIMING").is_some();
        let (t0, mut next_lap) = (std::time::Instant::now(), 1000usize);
        while let Some(e) = ctx.leave_edge() {
            // C does not test `enter_edge` for NULL; it is guaranteed to find
            // one on a connected aux graph (see `feasible_tree`'s Err(1) path).
            let Some(f) = ctx.enter_edge(e) else {
                break;
            };
            ctx.update(e, f)?;
            iter += 1;
            if timing && iter >= next_lap {
                eprintln!(
                    "[timing]   ns::rank2: {iter} pivots in {:.3}s",
                    t0.elapsed().as_secs_f64()
                );
                next_lap *= 2;
            }
            if iter as i32 >= maxiter {
                break;
            }
        }
        if timing {
            eprintln!(
                "[timing]   ns::rank2: finished {iter} pivots in {:.3}s",
                t0.elapsed().as_secs_f64()
            );
        }
        match balance {
            1 => {
                ctx.tb_balance(tbbalance);
                ctx.tree_edge.clear();
            }
            2 => ctx.lr_balance(),
            _ => {
                ctx.scan_and_normalize();
                ctx.free_tree_list();
            }
        }
        Ok(())
    };

    let result = run(&mut ctx);
    // the arena must observe every state the all-arena version left behind,
    // on success and error paths alike
    ctx.sync_back();
    result
}
