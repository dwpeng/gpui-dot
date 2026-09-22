//! Network simplex ranking — a faithful port of `lib/common/ns.c`.
//!
//! Shared by the rank phase (`rank(g, 1|0, maxiter)`) and the x-coordinate
//! phase (`rank(g, 2, nsiter2)`).
//!
//! The explicit-stack DFS functions mirror the C control flow exactly:
//! `tight_subtree_search`, `inter_tree_edge_search`, `dfs_cutval` and
//! `dfs_range*` keep their frames on the stack with resumable counters
//! (peek-with-counters), while `dfs_enter_*` pop their frames immediately
//! and push children mid-scan (pop-first).

use super::model::{EId, Fg, NId, NodeType};

const SEARCHSIZE: usize = 30;

#[inline]
fn length(fg: &Fg, e: EId) -> i32 {
    fg.nodes[fg.edges[e].head].rank - fg.nodes[fg.edges[e].tail].rank
}

#[inline]
fn slack(fg: &Fg, e: EId) -> i32 {
    length(fg, e) - fg.edges[e].minlen
}

#[inline]
fn seq(a: i32, b: i32, c: i32) -> bool {
    a <= b && b <= c
}

#[inline]
fn tree_edge(fg: &Fg, e: EId) -> bool {
    fg.edges[e].tree_index >= 0
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

/// The simplex context (`network_simplex_ctx_t`) — borrows the arena.
pub struct NsCtx<'a> {
    pub fg: &'a mut Fg,
    /// Fast node list in `GD_nlist` order.
    nodes: Vec<NId>,
    tree_edge: Vec<EId>,
    s_i: usize,
    search_size: usize,
}

impl<'a> NsCtx<'a> {
    fn add_tree_edge(&mut self, e: EId) {
        debug_assert!(!tree_edge(self.fg, e), "add_tree_edge: already a tree edge");
        self.fg.edges[e].tree_index = self.tree_edge.len() as i32;
        self.tree_edge.push(e);
        let n = self.fg.edges[e].tail;
        self.fg.nodes[n].mark = 1;
        self.fg.nodes[n].tree_out.push(e);
        let n = self.fg.edges[e].head;
        self.fg.nodes[n].mark = 1;
        self.fg.nodes[n].tree_in.push(e);
    }

    /// `invalidate_path` — mark ND_low = -1 walking up from to_node toward
    /// lca.
    fn invalidate_path(&mut self, lca: NId, mut to_node: NId) {
        loop {
            if self.fg.nodes[to_node].low == -1 {
                break;
            }
            self.fg.nodes[to_node].low = -1;
            let Some(e) = self.fg.nodes[to_node].par else {
                break;
            };
            if self.fg.nodes[to_node].lim >= self.fg.nodes[lca].lim {
                if to_node != lca {
                    // "invalidate_path: skipped over LCA"
                }
                break;
            }
            to_node = if self.fg.nodes[self.fg.edges[e].tail].lim
                > self.fg.nodes[self.fg.edges[e].head].lim
            {
                self.fg.edges[e].tail
            } else {
                self.fg.edges[e].head
            };
        }
    }

    /// `exchange_tree_edges` — swap-with-last removal, exactly like C.
    fn exchange_tree_edges(&mut self, e: EId, f: EId) {
        let idx = self.fg.edges[e].tree_index;
        self.fg.edges[f].tree_index = idx;
        self.tree_edge[idx as usize] = f;
        self.fg.edges[e].tree_index = -1;

        let n = self.fg.edges[e].tail;
        {
            let list = &mut self.fg.nodes[n].tree_out;
            let i = list.len() - 1;
            let j = list.iter().position(|&x| x == e).expect("tree edge missing");
            list.swap(j, i);
            list.pop();
        }
        let n = self.fg.edges[e].head;
        {
            let list = &mut self.fg.nodes[n].tree_in;
            let i = list.len() - 1;
            let j = list.iter().position(|&x| x == e).expect("tree edge missing");
            list.swap(j, i);
            list.pop();
        }
        let n = self.fg.edges[f].tail;
        self.fg.nodes[n].tree_out.push(f);
        let n = self.fg.edges[f].head;
        self.fg.nodes[n].tree_in.push(f);
    }

    /// `init_rank` — longest-path ranks via topological traversal.
    fn init_rank(&mut self) {
        let mut queue: std::collections::VecDeque<NId> = Default::default();
        for &v in &self.nodes {
            if self.fg.nodes[v].priority == 0 {
                queue.push_back(v);
            }
        }
        while let Some(v) = queue.pop_front() {
            self.fg.nodes[v].rank = 0;
            for i in 0..self.fg.nodes[v].in_.len() {
                let e = self.fg.nodes[v].in_[i];
                let t = self.fg.edges[e].tail;
                let r = self.fg.nodes[t].rank + self.fg.edges[e].minlen;
                self.fg.nodes[v].rank = self.fg.nodes[v].rank.max(r);
            }
            for i in 0..self.fg.nodes[v].out.len() {
                let e = self.fg.nodes[v].out[i];
                let h = self.fg.edges[e].head;
                self.fg.nodes[h].priority -= 1;
                if self.fg.nodes[h].priority <= 0 {
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
            if self.fg.edges[f].cutvalue < 0 {
                if let Some(r) = rv {
                    if self.fg.edges[r].cutvalue > self.fg.edges[f].cutvalue {
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
                if self.fg.edges[f].cutvalue < 0 {
                    if let Some(r) = rv {
                        if self.fg.edges[r].cutvalue > self.fg.edges[f].cutvalue {
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
            for i in 0..self.fg.nodes[v].out.len() {
                let e = self.fg.nodes[v].out[i];
                if !tree_edge(self.fg, e) {
                    let l = self.fg.nodes[self.fg.edges[e].head].lim;
                    if !seq(low, l, lim) {
                        let s = slack(self.fg, e);
                        if s < slack_best || enter.is_none() {
                            enter = Some(e);
                            slack_best = s;
                        }
                    }
                } else if self.fg.nodes[self.fg.edges[e].head].lim < self.fg.nodes[v].lim {
                    todo.push(self.fg.edges[e].head);
                }
            }
            if slack_best > 0 {
                for i in 0..self.fg.nodes[v].tree_in.len() {
                    let e = self.fg.nodes[v].tree_in[i];
                    let t = self.fg.edges[e].tail;
                    if self.fg.nodes[t].lim < self.fg.nodes[v].lim {
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
            for i in 0..self.fg.nodes[v].in_.len() {
                let e = self.fg.nodes[v].in_[i];
                if !tree_edge(self.fg, e) {
                    let l = self.fg.nodes[self.fg.edges[e].tail].lim;
                    if !seq(low, l, lim) {
                        let s = slack(self.fg, e);
                        if s < slack_best || enter.is_none() {
                            enter = Some(e);
                            slack_best = s;
                        }
                    }
                } else if self.fg.nodes[self.fg.edges[e].tail].lim < self.fg.nodes[v].lim {
                    todo.push(self.fg.edges[e].tail);
                }
            }
            if slack_best > 0 {
                for i in 0..self.fg.nodes[v].tree_out.len() {
                    let e = self.fg.nodes[v].tree_out[i];
                    let h = self.fg.edges[e].head;
                    if self.fg.nodes[h].lim < self.fg.nodes[v].lim {
                        todo.push(h);
                    }
                }
            }
        }
        enter
    }

    /// `enter_edge` — search from the down node of tree edge e.
    fn enter_edge(&self, e: EId) -> Option<EId> {
        // NsCtx methods below take &mut; this one is &self in C terms —
        // clone the needed data through a scoped immutable path instead.
        let (t, h) = (self.fg.edges[e].tail, self.fg.edges[e].head);
        let (v, outsearch) = if self.fg.nodes[t].lim < self.fg.nodes[h].lim {
            (t, false)
        } else {
            (h, true)
        };
        let (low, lim) = (self.fg.nodes[v].low, self.fg.nodes[v].lim);
        if outsearch {
            self.dfs_enter_outedge(v, low, lim)
        } else {
            self.dfs_enter_inedge(v, low, lim)
        }
    }

    // -- tight tree construction ------------------------------------------

    /// `tight_subtree_search` — grows the tight subtree containing v via
    /// zero-slack non-tree edges. Returns the subtree size.
    fn tight_subtree_search(
        &mut self,
        v: NId,
        st: usize,
        subtree_of: &mut [Option<usize>],
    ) -> i32 {
        let mut rv: i32 = 1;
        subtree_of[v] = Some(st);

        // frames: (v, in_i, out_i, rv)
        let mut todo: Vec<(NId, usize, usize, i32)> = vec![(v, 0, 0, 1)];

        while !todo.is_empty() {
            let top = todo.len() - 1;
            let fv = todo[top].0;
            let mut updated = false;

            // in edges
            while todo[top].1 < self.fg.nodes[fv].in_.len() {
                let i = todo[top].1;
                let e = self.fg.nodes[fv].in_[i];
                if tree_edge(self.fg, e) {
                    todo[top].1 += 1;
                    continue;
                }
                let t = self.fg.edges[e].tail;
                if subtree_of[t].is_none() && slack(self.fg, e) == 0 {
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
            while todo[top].2 < self.fg.nodes[fv].out.len() {
                let i = todo[top].2;
                let e = self.fg.nodes[fv].out[i];
                if tree_edge(self.fg, e) {
                    todo[top].2 += 1;
                    continue;
                }
                let h = self.fg.edges[e].head;
                if subtree_of[h].is_none() && slack(self.fg, e) == 0 {
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

    /// `STsetFind` — with path compression.
    fn st_set_find(subtrees: &mut [Subtree], mut s0: usize) -> usize {
        loop {
            let p = subtrees[s0].par.expect("subtree root has no parent");
            if p == s0 {
                return s0;
            }
            if let Some(pp) = subtrees[p].par {
                if pp != p {
                    subtrees[s0].par = Some(pp);
                }
            }
            s0 = p;
        }
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
        &mut self,
        v: NId,
        subtree_of: &[Option<usize>],
        subtrees: &[Subtree],
    ) -> Option<EId> {
        // C compares `STsetFind(...)` results — the *current* union-find root
        // of each node, not the original tight-subtree id. Comparing the
        // original ids makes two already-merged subtrees look distinct, which
        // returns an edge inside the merged tree and builds a cycle.
        let root_of = |n: NId| -> usize {
            let mut s = subtree_of[n].expect("node in subtree");
            while let Some(p) = subtrees[s].par {
                if p == s {
                    break;
                }
                s = p;
            }
            s
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
                    if slack(self.fg, b) == 0 {
                        todo.pop();
                        continue;
                    }
                }
            }

            let mut updated = false;

            // out edges
            while todo[top].3 < self.fg.nodes[fv].out.len() {
                let i = todo[top].3;
                let e = self.fg.nodes[fv].out[i];
                if tree_edge(self.fg, e) {
                    let h = self.fg.edges[e].head;
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
                let h = self.fg.edges[e].head;
                if root_of(h) != fts {
                    // encountered candidate edge
                    if best.is_none() || slack(self.fg, e) < slack(self.fg, best.unwrap()) {
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
            while todo[top].4 < self.fg.nodes[fv].in_.len() {
                let i = todo[top].4;
                let e = self.fg.nodes[fv].in_[i];
                if tree_edge(self.fg, e) {
                    let t = self.fg.edges[e].tail;
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
                let t = self.fg.edges[e].tail;
                if root_of(t) != fts {
                    if best.is_none() || slack(self.fg, e) < slack(self.fg, best.unwrap()) {
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
    fn tree_adjust(&mut self, v: NId, from: Option<NId>, delta: i32) {
        self.fg.nodes[v].rank += delta;
        for i in 0..self.fg.nodes[v].tree_in.len() {
            let e = self.fg.nodes[v].tree_in[i];
            let w = self.fg.edges[e].tail;
            if Some(w) != from {
                self.tree_adjust(w, Some(v), delta);
            }
        }
        for i in 0..self.fg.nodes[v].tree_out.len() {
            let e = self.fg.nodes[v].tree_out[i];
            let w = self.fg.edges[e].head;
            if Some(w) != from {
                self.tree_adjust(w, Some(v), delta);
            }
        }
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
        debug_assert!(!tree_edge(self.fg, e));
        let t0 = Self::st_set_find(subtrees, subtree_of[self.fg.edges[e].tail].unwrap());
        let t1 = Self::st_set_find(subtrees, subtree_of[self.fg.edges[e].head].unwrap());

        if !subtrees[t0].on_heap() {
            let delta = slack(self.fg, e);
            if delta != 0 {
                let rep = subtrees[t0].rep;
                self.tree_adjust(rep, None, delta);
            }
        } else {
            let delta = -slack(self.fg, e);
            if delta != 0 {
                let rep = subtrees[t1].rep;
                self.tree_adjust(rep, None, delta);
            }
        }
        self.add_tree_edge(e);
        Self::st_set_union(subtrees, t0, t1)
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
                subtrees.push(Subtree { rep: n, size: 0, heap_index: usize::MAX, par: Some(st) });
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
            for i in 0..self.fg.nodes[v].tree_out.len() {
                let e = self.fg.nodes[v].tree_out[i];
                let h = self.fg.edges[e].head;
                if !seen[h] {
                    seen[h] = true;
                    count += 1;
                    stack.push(h);
                }
            }
            for i in 0..self.fg.nodes[v].tree_in.len() {
                let e = self.fg.nodes[v].tree_in[i];
                let t = self.fg.edges[e].tail;
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
        while !seq(self.fg.nodes[v].low, self.fg.nodes[w].lim, self.fg.nodes[v].lim) {
            // A dangling `par` means the tree is a forest (disconnected aux
            // graph); `feasible_tree` already reports that as Err(1).
            let Some(e) = self.fg.nodes[v].par else {
                break;
            };
            let d = if self.fg.edges[e].tail == v { dir } else { !dir };
            if d {
                self.fg.edges[e].cutvalue += cutvalue;
            } else {
                self.fg.edges[e].cutvalue -= cutvalue;
            }
            v = if self.fg.nodes[self.fg.edges[e].tail].lim
                > self.fg.nodes[self.fg.edges[e].head].lim
            {
                self.fg.edges[e].tail
            } else {
                self.fg.edges[e].head
            };
        }
        v
    }

    /// `rerank` — shift v's tree subtree by -delta.
    fn rerank(&mut self, v: NId, delta: i32) {
        self.fg.nodes[v].rank -= delta;
        for i in 0..self.fg.nodes[v].tree_out.len() {
            let e = self.fg.nodes[v].tree_out[i];
            if Some(e) != self.fg.nodes[v].par {
                let h = self.fg.edges[e].head;
                self.rerank(h, delta);
            }
        }
        for i in 0..self.fg.nodes[v].tree_in.len() {
            let e = self.fg.nodes[v].tree_in[i];
            if Some(e) != self.fg.nodes[v].par {
                let t = self.fg.edges[e].tail;
                self.rerank(t, delta);
            }
        }
    }

    /// `update` — exchange leaving tree edge e for entering non-tree edge f.
    fn update(&mut self, e: EId, f: EId) -> Result<(), i32> {
        let delta = slack(self.fg, f);
        if delta > 0 {
            let te = self.fg.edges[e].tail;
            let he = self.fg.edges[e].head;
            let s = self.fg.nodes[te].tree_in.len() + self.fg.nodes[te].tree_out.len();
            if s == 1 {
                self.rerank(te, delta);
            } else {
                let s = self.fg.nodes[he].tree_in.len() + self.fg.nodes[he].tree_out.len();
                if s == 1 {
                    self.rerank(he, -delta);
                } else if self.fg.nodes[te].lim < self.fg.nodes[he].lim {
                    self.rerank(te, delta);
                } else {
                    self.rerank(he, -delta);
                }
            }
        }

        let cutvalue = self.fg.edges[e].cutvalue;
        let (ft, fh) = (self.fg.edges[f].tail, self.fg.edges[f].head);
        let lca = self.treeupdate(ft, fh, cutvalue, true);
        if self.treeupdate(fh, ft, cutvalue, false) != lca {
            // "update: mismatched lca in treeupdates"
            return Err(2);
        }

        let lca_low = self.fg.nodes[lca].low;
        self.invalidate_path(lca, fh);
        self.invalidate_path(lca, ft);

        self.fg.edges[f].cutvalue = -cutvalue;
        self.fg.edges[e].cutvalue = 0;
        self.exchange_tree_edges(e, f);
        let par = self.fg.nodes[lca].par;
        self.dfs_range(lca, par, lca_low);
        Ok(())
    }

    /// `x_val` (ns.c).
    fn x_val(&self, e: EId, v: NId, dir: i32) -> i32 {
        let (t, h) = (self.fg.edges[e].tail, self.fg.edges[e].head);
        let other = if t == v { h } else { t };
        let (f, mut rv);
        if !seq(self.fg.nodes[v].low, self.fg.nodes[other].lim, self.fg.nodes[v].lim) {
            f = 1;
            rv = self.fg.edges[e].weight;
        } else {
            f = 0;
            rv = if tree_edge(self.fg, e) { self.fg.edges[e].cutvalue } else { 0 };
            rv -= self.fg.edges[e].weight;
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
        let (ft, fh) = (self.fg.edges[f].tail, self.fg.edges[f].head);
        let (v, dir) = if self.fg.nodes[ft].par == Some(f) {
            (ft, 1)
        } else {
            (fh, -1)
        };

        let mut sum: i64 = 0;
        for i in 0..self.fg.nodes[v].out.len() {
            let e = self.fg.nodes[v].out[i];
            sum += self.x_val(e, v, dir) as i64;
        }
        for i in 0..self.fg.nodes[v].in_.len() {
            let e = self.fg.nodes[v].in_[i];
            sum += self.x_val(e, v, dir) as i64;
        }
        self.fg.edges[f].cutvalue = sum as i32;
    }

    /// `dfs_cutval` — post-order cut value computation (peek-with-counters).
    fn dfs_cutval(&mut self, v: NId, par: Option<EId>) {
        // frames: (v, par, out_i, in_i)
        let mut todo: Vec<(NId, Option<EId>, usize, usize)> = vec![(v, par, 0, 0)];
        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, _, _) = todo[top];
            let mut updated = false;

            while todo[top].2 < self.fg.nodes[fv].tree_out.len() {
                let i = todo[top].2;
                let e = self.fg.nodes[fv].tree_out[i];
                todo[top].2 += 1;
                if Some(e) != fpar {
                    let h = self.fg.edges[e].head;
                    todo.push((h, Some(e), 0, 0));
                    updated = true;
                    break;
                }
            }
            if updated {
                continue;
            }

            while todo[top].3 < self.fg.nodes[fv].tree_in.len() {
                let i = todo[top].3;
                let e = self.fg.nodes[fv].tree_in[i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let t = self.fg.edges[e].tail;
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
    }

    /// `dfs_range_init` — initial par/low/lim assignment (peek-with-counters).
    fn dfs_range_init(&mut self, v: NId) -> i32 {
        let mut lim: i32 = 0;
        self.fg.nodes[v].par = None;
        self.fg.nodes[v].low = 1;
        // frames: (v, par, lim, tree_out_i, tree_in_i)
        let mut todo: Vec<(NId, Option<EId>, i32, usize, usize)> = vec![(v, None, 1, 0, 0)];
        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, flim, _, _) = todo[top];
            let mut pushed_new = false;

            while todo[top].3 < self.fg.nodes[fv].tree_out.len() {
                let i = todo[top].3;
                let e = self.fg.nodes[fv].tree_out[i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let n = self.fg.edges[e].head;
                    self.fg.nodes[n].par = Some(e);
                    self.fg.nodes[n].low = flim;
                    todo.push((n, Some(e), flim, 0, 0));
                    pushed_new = true;
                    break;
                }
            }
            if pushed_new {
                continue;
            }

            while todo[top].4 < self.fg.nodes[fv].tree_in.len() {
                let i = todo[top].4;
                let e = self.fg.nodes[fv].tree_in[i];
                todo[top].4 += 1;
                if Some(e) != fpar {
                    let n = self.fg.edges[e].tail;
                    self.fg.nodes[n].par = Some(e);
                    self.fg.nodes[n].low = flim;
                    todo.push((n, Some(e), flim, 0, 0));
                    pushed_new = true;
                    break;
                }
            }
            if pushed_new {
                continue;
            }

            let (pv, _, plim, _, _) = todo.pop().unwrap();
            self.fg.nodes[pv].lim = plim;
            lim = plim;
            if let Some(parent) = todo.last_mut() {
                parent.2 = lim + 1;
            }
        }
        lim + 1
    }

    /// `dfs_range` — incremental par/low/lim update.
    fn dfs_range(&mut self, v: NId, par: Option<EId>, low: i32) -> i32 {
        let mut lim: i32 = 0;

        if self.fg.nodes[v].par == par && self.fg.nodes[v].low == low {
            return self.fg.nodes[v].lim + 1;
        }

        self.fg.nodes[v].par = par;
        self.fg.nodes[v].low = low;
        let mut todo: Vec<(NId, Option<EId>, i32, usize, usize)> = vec![(v, par, low, 0, 0)];

        while !todo.is_empty() {
            let top = todo.len() - 1;
            let (fv, fpar, mut flim, _, _) = todo[top];
            let mut processed_child = false;

            while todo[top].3 < self.fg.nodes[fv].tree_out.len() {
                let i = todo[top].3;
                let e = self.fg.nodes[fv].tree_out[i];
                todo[top].3 += 1;
                if Some(e) != fpar {
                    let n = self.fg.edges[e].head;
                    if self.fg.nodes[n].par == Some(e) && self.fg.nodes[n].low == flim {
                        flim = self.fg.nodes[n].lim + 1;
                        todo[top].2 = flim;
                    } else {
                        self.fg.nodes[n].par = Some(e);
                        self.fg.nodes[n].low = flim;
                        todo.push((n, Some(e), flim, 0, 0));
                    }
                    processed_child = true;
                    break;
                }
            }
            if processed_child {
                continue;
            }

            while todo[top].4 < self.fg.nodes[fv].tree_in.len() {
                let i = todo[top].4;
                let e = self.fg.nodes[fv].tree_in[i];
                todo[top].4 += 1;
                if Some(e) != fpar {
                    let n = self.fg.edges[e].tail;
                    if self.fg.nodes[n].par == Some(e) && self.fg.nodes[n].low == flim {
                        flim = self.fg.nodes[n].lim + 1;
                        todo[top].2 = flim;
                    } else {
                        self.fg.nodes[n].par = Some(e);
                        self.fg.nodes[n].low = flim;
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
            self.fg.nodes[pv].lim = plim;
            lim = plim;
            if let Some(parent) = todo.last_mut() {
                parent.2 = lim + 1;
            }
        }
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
            if self.fg.nodes[n].node_type == NodeType::Normal {
                minrank = minrank.min(self.fg.nodes[n].rank);
                maxrank = maxrank.max(self.fg.nodes[n].rank);
            }
        }
        for &n in &self.nodes {
            self.fg.nodes[n].rank -= minrank;
        }
        maxrank - minrank
    }

    /// `freeTreeList`.
    fn free_tree_list(&mut self) {
        for &n in &self.nodes {
            self.fg.nodes[n].tree_in.clear();
            self.fg.nodes[n].tree_out.clear();
            self.fg.nodes[n].mark = 0;
        }
        self.tree_edge.clear();
    }

    /// `LR_balance` (balance == 2).
    fn lr_balance(&mut self) {
        for i in 0..self.tree_edge.len() {
            let e = self.tree_edge[i];
            if self.fg.edges[e].cutvalue == 0 {
                let Some(f) = self.enter_edge(e) else {
                    continue;
                };
                let delta = slack(self.fg, f);
                if delta <= 1 {
                    continue;
                }
                let (te, he) = (self.fg.edges[e].tail, self.fg.edges[e].head);
                if self.fg.nodes[te].lim < self.fg.nodes[he].lim {
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
                    if self.fg.nodes[n].node_type == NodeType::Normal {
                        if self.fg.nodes[n].in_.is_empty() && adj == 1 {
                            self.fg.nodes[n].rank = 0;
                        }
                        if self.fg.nodes[n].out.is_empty() && adj == 2 {
                            self.fg.nodes[n].rank = maxrank;
                        }
                    }
                }
            }
        }

        let mut tree_node: Vec<NId> = self.nodes.clone();
        if adj > 1 {
            tree_node.sort_by(|a, b| self.fg.nodes[*b].rank.cmp(&self.fg.nodes[*a].rank));
        } else {
            tree_node.sort_by(|a, b| self.fg.nodes[*a].rank.cmp(&self.fg.nodes[*b].rank));
        }
        for &n in &tree_node {
            if self.fg.nodes[n].node_type == NodeType::Normal {
                nrank[self.fg.nodes[n].rank as usize] += 1;
            }
        }
        for &n in tree_node.clone().iter() {
            if self.fg.nodes[n].node_type != NodeType::Normal {
                continue;
            }
            let mut inweight = 0i32;
            let mut outweight = 0i32;
            let mut low = 0i32;
            let mut high = maxrank;
            for i in 0..self.fg.nodes[n].in_.len() {
                let e = self.fg.nodes[n].in_[i];
                inweight += self.fg.edges[e].weight;
                let r = self.fg.nodes[self.fg.edges[e].tail].rank + self.fg.edges[e].minlen;
                low = low.max(r);
            }
            for i in 0..self.fg.nodes[n].out.len() {
                let e = self.fg.nodes[n].out[i];
                outweight += self.fg.edges[e].weight;
                let r = self.fg.nodes[self.fg.edges[e].head].rank - self.fg.edges[e].minlen;
                high = high.min(r);
            }
            if low < 0 {
                low = 0; // vnodes can have ranks < 0
            }
            if adj != 0 {
                if inweight == outweight {
                    self.fg.nodes[n].rank = if adj == 1 { low } else { high };
                }
            } else if inweight == outweight {
                let mut choice = low;
                for i in (low + 1)..=high {
                    if nrank[i as usize] < nrank[choice as usize] {
                        choice = i;
                    }
                }
                nrank[self.fg.nodes[n].rank as usize] -= 1;
                nrank[choice as usize] += 1;
                self.fg.nodes[n].rank = choice;
            }
            self.fg.nodes[n].tree_in.clear();
            self.fg.nodes[n].tree_out.clear();
            self.fg.nodes[n].mark = 0;
        }
    }

    /// `init_graph` — counts, priorities, feasibility.
    fn init_graph(&mut self) -> bool {
        let mut feasible = true;
        for &n in &self.nodes {
            self.fg.nodes[n].mark = 0;
            self.fg.nodes[n].priority = 0;
            for i in 0..self.fg.nodes[n].in_.len() {
                let e = self.fg.nodes[n].in_[i];
                self.fg.nodes[n].priority += 1;
                self.fg.edges[e].cutvalue = 0;
                self.fg.edges[e].tree_index = -1;
                let (t, h) = (self.fg.edges[e].tail, self.fg.edges[e].head);
                if self.fg.nodes[h].rank - self.fg.nodes[t].rank < self.fg.edges[e].minlen {
                    feasible = false;
                }
            }
            self.fg.nodes[n].tree_in.clear();
            self.fg.nodes[n].tree_out.clear();
        }
        feasible
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
        search_size: if search_size >= 0 { search_size as usize } else { SEARCHSIZE },
    };

    let feasible = ctx.init_graph();
    if !feasible {
        ctx.init_rank();
    }

    if let Err(err) = ctx.feasible_tree() {
        ctx.free_tree_list();
        return Err(err);
    }
    if maxiter <= 0 {
        ctx.free_tree_list();
        return Ok(());
    }

    let mut iter = 0;
    let timing = std::env::var_os("GD_TIMING").is_some();
    let (t0, mut next_lap) = (std::time::Instant::now(), 1000usize);
    while let Some(e) = ctx.leave_edge() {
        // C does not test `enter_edge` for NULL; it is guaranteed to find one
        // on a connected aux graph (see `feasible_tree`'s Err(1) path).
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
}
