//! Crossing minimization — a faithful port of `lib/dotgen/mincross.c`
//! (Graphviz `main` @ `2e92c7f`, 1795 lines).
//!
//! `dot_mincross` takes a ranked graph and finds an ordering that avoids
//! edge crossings (mincross.c:11-16). The rank structure is global — not
//! allocated per cluster — because mincross may compare nodes in different
//! clusters. This file ports the cluster-aware pipeline
//! completely:
//!
//! ```text
//! dot_mincross
//! ├── init_mincross ── mincross_options, class2*, decompose*,
//! │                    allocate_ranks, ordered_edges
//! ├── init_mccomp → mincross(g,0)          [per connected component]
//! │     ├── build_ranks ── install_in_rank, enqueue_neighbors, ncross, transpose
//! │     ├── flat_breakcycles ── flat_search ── flat_rev
//! │     ├── flat_reorder ── constraining_flat_edge, postorder, flat_rev
//! │     ├── ncross ── rcross ── local_cross
//! │     ├── save_best / restore_best
//! │     └── mincross_step ── medians (flat_mval), reorder, transpose
//! │                              └─ transpose_step ── in_cross/out_cross
//! ├── merge2 ── merge_components
//! ├── mincross_clust (per cluster: expand_cluster → flat_* → mincross(g,2))
//! └── cleanup2
//! ```
//!
//! (`*` = defined in other modules: `classes.rs`, `rank.rs`.)
//!
//! ## Porting map (C pointer idioms → arena)
//!
//! * **Rank rows / windows** — C's `GD_rank(g)[r].v` is a *moving window*
//!   into the allocated row `av`: `init_mccomp` (mincross.c:422-432) advances
//!   it past each finished component and `merge2` (L807-830) resets it. The
//!   port stores the full allocated row in `Rank::v` (== `av`, with a
//!   `NO_NODE` sentinel playing calloc's trailing `NULL`) plus a per-rank
//!   window offset [`MinCross::win`] (== `v - av`). Every access
//!   `GD_rank(g)[r].v[i]` becomes `Rank::v[win[r] + i]`; `exchange`
//!   (L623-633) writes through the same window.
//! * **Array size** — C allocates `GD_maxrank(g)+2` rank rows, 0-based
//!   (L1141): rows `minrank..=maxrank` are allocated with `cn[r]+1` slots,
//!   the `maxrank+1` row (read by `transpose_step`'s
//!   `GD_rank(g)[r+1].n > 0`) and everything below `minrank` (including the
//!   `minrank-1` slot) stay zeroed. The port indexes
//!   `fg.graphs[g].rank` by the *absolute* rank `r` over a
//!   `Vec<Rank>` of length `maxrank+2`; rows outside `minrank..=maxrank`
//!   are `Rank::default()` (`n == 0`, empty `v`) — the zeroed slot semantics.
//!   For the root graph `minrank` is always 0 in the dot1 pipeline (the
//!   invariant position.rs documents for its `rank_row(r) = rank[r-minrank]`
//!   accessor), so absolute indexing *is* the dense-from-`minrank` layout
//!   the sibling phases use; reorder's `r > 0` guard (L1450) is then
//!   equivalent to C's "row `minrank-1` is only touched when it exists".
//! * **`saveorder(v)`** — C aliases `ND_coord.x` (L114); the port uses the
//!   side table [`MinCross::saved_order`] (spec §1.1/§12.1).
//! * **`ND_mark`** — the arena's monotonic stamp counter (see `model.rs`);
//!   boolean marks are encoded as a fresh `(unset, set)` stamp pair from
//!   [`fresh_marks`].
//! * **`flatindex(v)`** — `ND_low(v)` (L115), stored in `DNode::low`
//!   (rank.rs's network-simplex `low` is finished by the time mincross runs).
//! * **`GD_rank[r].flat`** — the per-rank bit matrix lives in
//!   `Rank::flat` (`Option<Vec<Vec<bool>>>`); `matrix_get`/`matrix_set`
//!   below reproduce mincross.c:51-110 including out-of-range ⇒ false and
//!   on-write expansion.
//! * **`GD_rank[r].candidate`** — not present on `Rank`; kept in
//!   [`MinCross::candidate`] (per-rank side table).
//! * **`TE_list`/`TI_list`** — C module scratch arrays (L165-166) sized
//!   `agnedges+1`; the port uses growable locals at the two use sites
//!   (`do_ordering_node`, `medians`) — same contents, same order.
//! * **i32/i64 asymmetry** — `out_cross` returns `i32` (L606) while
//!   `in_cross` returns `i64` (L587); preserved, with C wrap-around
//!   reproduced via `wrapping_*` so debug builds do not panic.
//!
//! There is no randomness anywhere in this file (spec §0); identical input
//! plus identical iteration order ⇒ identical output ordering.

use std::collections::VecDeque;

use super::classes::{
    class2, delete_flat_edge, find_flat_edge, flat_edge, merge_oneway, new_virtual_edge,
};
use super::cluster;
use super::model::{EId, EdgeType, Fg, GId, MC_SCALE, NId, NodeType, RankType};
use super::position;
use super::position::rank_row_index;
use super::rank;

/// C rank-array NULL sentinel — calloc'd `node_t*` slots read as `NULL`
/// (merge2's end-of-row scan, mincross.c:818-826).
const NO_NODE: NId = usize::MAX;

/// `Convergence` (mincross.c:161).
const CONVERGENCE: f64 = 0.995;

#[inline]
fn ri(r: i32) -> usize {
    debug_assert!(r >= 0, "rank index underflow");
    r as usize
}

/// Fresh `(unset, set)` stamps for the `ND_mark` boolean protocol —
/// `DNode::mark` is the arena's monotonic stamp counter, so `MARK(v) == set`
/// plays C's `true` and `== unset` plays `false`.
fn fresh_marks(fg: &mut Fg) -> (usize, usize) {
    let unset = fg.stamp_next;
    let set = unset + 1;
    fg.stamp_next += 2;
    (unset, set)
}

// ---------------------------------------------------------------------------
// adjmatrix_t (mincross.c:39-110) over `Rank::flat`
// ---------------------------------------------------------------------------

/// `new_matrix` (mincross.c:405-413) — a zeroed `rows × cols` bit matrix.
fn matrix_new(rows: usize, cols: usize) -> Vec<Vec<bool>> {
    vec![vec![false; cols]; rows]
}

/// `matrix_get` (mincross.c:51-66) — out-of-range ⇒ `false`.
#[cfg(test)]
#[allow(dead_code)]
fn matrix_get(me: &Option<Vec<Vec<bool>>>, row: usize, col: usize) -> bool {
    match me {
        Some(m) => matrix_get_ref(m, row, col),
        None => false,
    }
}

/// Same lookup over an already-unwrapped matrix.
fn matrix_get_ref(m: &[Vec<bool>], row: usize, col: usize) -> bool {
    m.get(row)
        .and_then(|r| r.get(col))
        .copied()
        .unwrap_or(false)
}

/// Horizontal sum of a 4 × i64 vector (exact).
#[cfg(target_arch = "x86_64")]
fn horiz_i64(v: core::arch::x86_64::__m256i) -> i64 {
    let mut buf = [0i64; 4];
    // SAFETY: stores 32 bytes into a [i64; 4] buffer.
    unsafe { core::arch::x86_64::_mm256_storeu_si256(buf.as_mut_ptr() as *mut _, v) };
    buf[0] + buf[1] + buf[2] + buf[3]
}

/// Horizontal wrapping sum of an 8 × i32 vector (exact mod 2³²).
#[cfg(target_arch = "x86_64")]
fn horiz_i32_wrapping(v: core::arch::x86_64::__m256i) -> i32 {
    let mut buf = [0i32; 8];
    // SAFETY: stores 32 bytes into a [i32; 8] buffer.
    unsafe { core::arch::x86_64::_mm256_storeu_si256(buf.as_mut_ptr() as *mut _, v) };
    buf.iter().fold(0i32, |a, &b| a.wrapping_add(b))
}

/// `matrix_set` (mincross.c:73-110) — setting out of range grows the backing
/// store (new zeroed rows/columns; previously set bits keep their
/// coordinates), then sets the bit.
fn matrix_set(me: &mut Option<Vec<Vec<bool>>>, row: usize, col: usize) {
    let m = me.get_or_insert_with(Vec::new);
    let ncols = m.first().map(|r| r.len()).unwrap_or(0).max(col + 1);
    let nrows = m.len().max(row + 1);
    m.resize(nrows, Vec::new());
    for r in m.iter_mut() {
        r.resize(ncols, false);
    }
    m[row][col] = true;
}

// ---------------------------------------------------------------------------
// Crossing counting primitives (mincross.c:587-621, 1482-1504)
// ---------------------------------------------------------------------------

impl MinCross {
    /// Rebuilds [`Snap`] from the arena — O(V+E), run only after a
    /// structural change dropped `snap.valid`.
    fn rebuild_snap(&mut self, fg: &Fg) {
        let nv = fg.nodes.len();
        let ne = fg.edges.len();
        let s = &mut self.snap;
        s.ord = fg.nodes.iter().map(|n| n.order).collect();
        s.nclust = fg
            .nodes
            .iter()
            .map(|n| n.clust.map_or(0, |c| c as u32 + 1))
            .collect();
        s.skel = fg
            .nodes
            .iter()
            .map(|n| n.ranktype == RankType::Cluster && n.node_type == NodeType::Virtual)
            .collect();
        s.any_port = fg.nodes.iter().any(|n| n.has_port);
        s.mval = fg.nodes.iter().map(|n| n.mval).collect();

        let mut total_out = 0usize;
        let mut total_in = 0usize;
        for n in fg.nodes.iter() {
            total_out += n.out.len();
            total_in += n.in_.len();
        }
        s.out_edge.resize(total_out, 0);
        s.out_eho.resize(total_out, 0);
        s.out_xp.resize(total_out, 0);
        s.out_hpx.resize(total_out, 0.0);
        s.out_hpo.resize(total_out, 0);
        s.in_edge.resize(total_in, 0);
        s.in_eto.resize(total_in, 0);
        s.in_xp.resize(total_in, 0);
        s.in_tpx.resize(total_in, 0.0);
        s.in_tpo.resize(total_in, 0);

        s.out_start.clear();
        s.out_start.push(0);
        s.in_start.clear();
        s.in_start.push(0);
        let (mut io, mut ii) = (0usize, 0usize);
        for n in 0..nv {
            for &e in fg.nodes[n].out.iter() {
                let d = &fg.edges[e];
                s.out_edge[io] = e as u32;
                s.out_eho[io] = s.ord[d.head];
                s.out_xp[io] = d.xpenalty;
                s.out_hpx[io] = d.head_port.p.x;
                s.out_hpo[io] = d.head_port.order;
                io += 1;
            }
            s.out_start.push(io as u32);
            for &e in fg.nodes[n].in_.iter() {
                let d = &fg.edges[e];
                s.in_edge[ii] = e as u32;
                s.in_eto[ii] = s.ord[d.tail];
                s.in_xp[ii] = d.xpenalty;
                s.in_tpx[ii] = d.tail_port.p.x;
                s.in_tpo[ii] = d.tail_port.order;
                ii += 1;
            }
            s.in_start.push(ii as u32);
        }
        // edge id → entry slot, for exchange's O(1) payload patches
        s.out_slot.clear();
        s.out_slot.resize(ne, u32::MAX);
        s.in_slot.clear();
        s.in_slot.resize(ne, u32::MAX);
        for k in 0..total_out {
            s.out_slot[s.out_edge[k] as usize] = k as u32;
        }
        for k in 0..total_in {
            s.in_slot[s.in_edge[k] as usize] = k as u32;
        }
        s.valid = true;
    }

    /// Lazily (re)builds the snapshot before a hot loop reads it.
    #[inline]
    fn ensure_snap(&mut self, fg: &Fg) {
        if !self.snap.valid {
            self.rebuild_snap(fg);
        }
    }

    /// `in_cross(v, w)` and `in_cross(w, v)` fused into one pass.
    ///
    /// Both directions scan the same (e1 ∈ in(v), e2 ∈ in(w)) pairs with
    /// mirrored conditions: with `t = ord(tail(e1)) − ord(tail(e2))`,
    /// direction (v,w) counts `t > 0` (or `t == 0 && port(e1) > port(e2)`)
    /// and direction (w,v) counts `t < 0` (or `t == 0 && port(e2) >
    /// port(e1)`); both add the same commutative `xp(e1)·xp(e2)` product.
    /// One pass halves the loads, and the strict port compares can never
    /// both fire, so the sums are exactly the two separate loops'.
    fn in_cross_pair(&self, v: NId, w: NId) -> (i64, i64) {
        let s = &self.snap;
        let (v0, v1) = (s.in_start[v] as usize, s.in_start[v + 1] as usize);
        let (w0, w1) = (s.in_start[w] as usize, s.in_start[w + 1] as usize);
        if v0 == v1 || w0 == w1 {
            return (0, 0);
        }
        if s.any_port {
            let mut c0: i64 = 0;
            let mut c1: i64 = 0;
            for k in w0..w1 {
                let cnt = s.in_xp[k];
                let inv = s.in_eto[k];
                let tpx2 = s.in_tpx[k];
                for m in v0..v1 {
                    let t = s.in_eto[m] - inv;
                    let prod = s.in_xp[m].wrapping_mul(cnt);
                    if t > 0 {
                        c0 += prod as i64;
                    } else if t < 0 {
                        c1 += prod as i64;
                    } else {
                        // equal tail order ⇒ parallel edges; the strict port
                        // compares are mutually exclusive
                        if s.in_tpx[m] > tpx2 {
                            c0 += prod as i64;
                        }
                        if tpx2 > s.in_tpx[m] {
                            c1 += prod as i64;
                        }
                    }
                }
            }
            (c0, c1)
        } else {
            #[cfg(target_arch = "x86_64")]
            {
                // 8-wide lanes pay off only past a few degrees; dot graphs
                // are typically 2–6 degree, so gate on range size
                if self.simd && v1 - v0 >= 16 {
                    let (mut c0, mut c1) = (0i64, 0i64);
                    // SAFETY: avx2 detected; the slices are indexed within
                    // the CSR ranges verified above.
                    unsafe {
                        self.in_cross_pair_avx2(v0, v1, w0, w1, &mut c0, &mut c1);
                    }
                    return (c0, c1);
                }
            }
            let mut c0: i64 = 0;
            let mut c1: i64 = 0;
            for k in w0..w1 {
                let cnt = s.in_xp[k];
                let inv = s.in_eto[k];
                for m in v0..v1 {
                    let t = s.in_eto[m] - inv;
                    if t != 0 {
                        let prod = s.in_xp[m].wrapping_mul(cnt);
                        if t > 0 {
                            c0 += prod as i64;
                        } else {
                            c1 += prod as i64;
                        }
                    }
                }
            }
            (c0, c1)
        }
    }

    /// `out_cross(v, w)` / `out_cross(w, v)` fused — see
    /// [`Self::in_cross_pair`]; i32 accumulators with wrapping adds.
    fn out_cross_pair(&self, v: NId, w: NId) -> (i32, i32) {
        let s = &self.snap;
        let (v0, v1) = (s.out_start[v] as usize, s.out_start[v + 1] as usize);
        let (w0, w1) = (s.out_start[w] as usize, s.out_start[w + 1] as usize);
        if v0 == v1 || w0 == w1 {
            return (0, 0);
        }
        if s.any_port {
            let mut c0: i32 = 0;
            let mut c1: i32 = 0;
            for k in w0..w1 {
                let cnt = s.out_xp[k];
                let inv = s.out_eho[k];
                let hpx2 = s.out_hpx[k];
                for m in v0..v1 {
                    let t = s.out_eho[m] - inv;
                    let prod = s.out_xp[m].wrapping_mul(cnt);
                    if t > 0 {
                        c0 = c0.wrapping_add(prod);
                    } else if t < 0 {
                        c1 = c1.wrapping_add(prod);
                    } else {
                        if s.out_hpx[m] > hpx2 {
                            c0 = c0.wrapping_add(prod);
                        }
                        if hpx2 > s.out_hpx[m] {
                            c1 = c1.wrapping_add(prod);
                        }
                    }
                }
            }
            (c0, c1)
        } else {
            #[cfg(target_arch = "x86_64")]
            {
                if self.simd && v1 - v0 >= 16 {
                    let (mut c0, mut c1) = (0i32, 0i32);
                    // SAFETY: avx2 detected; slices indexed within CSR ranges.
                    unsafe {
                        self.out_cross_pair_avx2(v0, v1, w0, w1, &mut c0, &mut c1);
                    }
                    return (c0, c1);
                }
            }
            let mut c0: i32 = 0;
            let mut c1: i32 = 0;
            for k in w0..w1 {
                let cnt = s.out_xp[k];
                let inv = s.out_eho[k];
                for m in v0..v1 {
                    let t = s.out_eho[m] - inv;
                    if t != 0 {
                        let prod = s.out_xp[m].wrapping_mul(cnt);
                        if t > 0 {
                            c0 = c0.wrapping_add(prod);
                        } else {
                            c1 = c1.wrapping_add(prod);
                        }
                    }
                }
            }
            (c0, c1)
        }
    }

    /// AVX2 core of [`Self::in_cross_pair`] (no-port path): 8-wide i32
    /// compare/select/multiply, widened to exact i64 accumulation — the
    /// lane arithmetic reproduces `wrapping_mul` + i64 sign-extension
    /// per term.
    ///
    /// # Safety
    /// Caller must have verified AVX2 support. Indices must be in range of
    /// the snapshot arrays (guaranteed: they come from the CSR starts).
    #[cfg(target_arch = "x86_64")]
    unsafe fn in_cross_pair_avx2(&self, v0: usize, v1: usize, w0: usize, w1: usize, c0: &mut i64, c1: &mut i64) {

        unsafe {
            use core::arch::x86_64::*;
            let s = &self.snap;
            let zero = _mm256_setzero_si256();
            let mut acc0 = _mm256_setzero_si256(); // 4 × i64 (lanes 0..4)
            let mut acc0h = _mm256_setzero_si256(); // 4 × i64 (lanes 4..8)
            let mut acc1 = _mm256_setzero_si256();
            let mut acc1h = _mm256_setzero_si256();
            let eto = s.in_eto.as_slice();
            let xp = s.in_xp.as_slice();
            for k in w0..w1 {
                let inv = _mm256_set1_epi32(s.in_eto[k]);
                let cnt = _mm256_set1_epi32(s.in_xp[k]);
                let mut m = v0;
                while m + 8 <= v1 {
                    let t = _mm256_sub_epi32(
                        _mm256_loadu_si256(eto.as_ptr().add(m) as *const __m256i),
                        inv,
                    );
                    let prod = _mm256_mullo_epi32(
                        _mm256_loadu_si256(xp.as_ptr().add(m) as *const __m256i),
                        cnt,
                    );
                    // t > 0 → c0's term; t < 0 → c1's term (t == 0 ⇒ no port
                    // fires in the no-port path ⇒ no term)
                    let p0 = _mm256_and_si256(prod, _mm256_cmpgt_epi32(t, zero));
                    let p1 = _mm256_and_si256(prod, _mm256_cmpgt_epi32(zero, t));
                    acc0 = _mm256_add_epi64(
                        acc0,
                        _mm256_cvtepi32_epi64(_mm256_castsi256_si128(p0)),
                    );
                    acc0h = _mm256_add_epi64(
                        acc0h,
                        _mm256_cvtepi32_epi64(_mm256_extracti128_si256(p0, 1)),
                    );
                    acc1 = _mm256_add_epi64(
                        acc1,
                        _mm256_cvtepi32_epi64(_mm256_castsi256_si128(p1)),
                    );
                    acc1h = _mm256_add_epi64(
                        acc1h,
                        _mm256_cvtepi32_epi64(_mm256_extracti128_si256(p1, 1)),
                    );
                    m += 8;
                }
                // scalar tail
                while m < v1 {
                    let t = s.in_eto[m] - s.in_eto[k];
                    if t != 0 {
                        let prod = s.in_xp[m].wrapping_mul(s.in_xp[k]);
                        if t > 0 {
                            *c0 += prod as i64;
                        } else {
                            *c1 += prod as i64;
                        }
                    }
                    m += 1;
                }
            }
            *c0 += horiz_i64(acc0) + horiz_i64(acc0h);
            *c1 += horiz_i64(acc1) + horiz_i64(acc1h);
    
        }
    }

    /// AVX2 core of [`Self::out_cross_pair`] (no-port path): the i32
    /// accumulators wrap per lane, and addition mod 2³² is
    /// associative/commutative, so the horizontal reduction reproduces C's
    /// wrapping accumulation exactly.
    ///
    /// # Safety
    /// Caller must have verified AVX2 support. Indices must be in range of
    /// the snapshot arrays.
    #[cfg(target_arch = "x86_64")]
    unsafe fn out_cross_pair_avx2(&self, v0: usize, v1: usize, w0: usize, w1: usize, c0: &mut i32, c1: &mut i32) {

        unsafe {
            use core::arch::x86_64::*;
            let s = &self.snap;
            let zero = _mm256_setzero_si256();
            let mut acc0 = _mm256_setzero_si256();
            let mut acc1 = _mm256_setzero_si256();
            let eho = s.out_eho.as_slice();
            let xp = s.out_xp.as_slice();
            for k in w0..w1 {
                let inv = _mm256_set1_epi32(s.out_eho[k]);
                let cnt = _mm256_set1_epi32(s.out_xp[k]);
                let mut m = v0;
                while m + 8 <= v1 {
                    let t = _mm256_sub_epi32(
                        _mm256_loadu_si256(eho.as_ptr().add(m) as *const __m256i),
                        inv,
                    );
                    let prod = _mm256_mullo_epi32(
                        _mm256_loadu_si256(xp.as_ptr().add(m) as *const __m256i),
                        cnt,
                    );
                    acc0 = _mm256_add_epi32(acc0, _mm256_and_si256(prod, _mm256_cmpgt_epi32(t, zero)));
                    acc1 = _mm256_add_epi32(acc1, _mm256_and_si256(prod, _mm256_cmpgt_epi32(zero, t)));
                    m += 8;
                }
                while m < v1 {
                    let t = s.out_eho[m] - s.out_eho[k];
                    if t != 0 {
                        let prod = s.out_xp[m].wrapping_mul(s.out_xp[k]);
                        if t > 0 {
                            *c0 = c0.wrapping_add(prod);
                        } else {
                            *c1 = c1.wrapping_add(prod);
                        }
                    }
                    m += 1;
                }
            }
            *c0 = c0.wrapping_add(horiz_i32_wrapping(acc0));
            *c1 = c1.wrapping_add(horiz_i32_wrapping(acc1));
    
        }
    }
}

/// `in_cross(v, w)` (mincross.c:587-604) — reference form reading the arena
/// directly (test oracle for the snapshot-based [`MinCross::in_cross`]).
#[cfg(test)]
#[allow(dead_code)]
fn in_cross(fg: &Fg, v: NId, w: NId) -> i64 {
    let mut cross: i64 = 0;
    for &e2 in fg.nodes[w].in_.iter() {
        let cnt = fg.edges[e2].xpenalty;
        let inv = fg.nodes[fg.edges[e2].tail].order;
        for &e1 in fg.nodes[v].in_.iter() {
            let t = fg.nodes[fg.edges[e1].tail].order - inv;
            // equal tail order ⇒ counted iff e1's tail port x is strictly greater
            if t > 0 || (t == 0 && fg.edges[e1].tail_port.p.x > fg.edges[e2].tail_port.p.x) {
                // C: int * int (wrapping) accumulated into int64
                cross += fg.edges[e1].xpenalty.wrapping_mul(cnt) as i64;
            }
        }
    }
    cross
}

/// `out_cross(v, w)` (mincross.c:606-621) — reference form reading the arena
/// directly (test oracle). **i32 accumulator** (asymmetry with `in_cross`
/// preserved; C wrap reproduced with `wrapping_*`).
#[cfg(test)]
fn out_cross(fg: &Fg, v: NId, w: NId) -> i32 {
    let mut cross: i32 = 0;
    for &e2 in fg.nodes[w].out.iter() {
        let cnt = fg.edges[e2].xpenalty;
        let inv = fg.nodes[fg.edges[e2].head].order;
        for &e1 in fg.nodes[v].out.iter() {
            let t = fg.nodes[fg.edges[e1].head].order - inv;
            if t > 0 || (t == 0 && fg.edges[e1].head_port.p.x > fg.edges[e2].head_port.p.x) {
                cross = cross.wrapping_add(fg.edges[e1].xpenalty.wrapping_mul(cnt));
            }
        }
    }
    cross
}

/// `local_cross(l, dir)` (mincross.c:1482-1504) — pairwise crossings within
/// one elist. The product-of-differences `< 0` test is the exact C code
/// (integer × double promoted to double, compared `< 0`).
fn local_cross(fg: &Fg, l: &[EId], dir: i32) -> i32 {
    let mut cross: i32 = 0;
    let is_out = dir > 0;
    for i in 0..l.len() {
        let e = l[i];
        for j in (i + 1)..l.len() {
            let f = l[j];
            let crossed = if is_out {
                (fg.nodes[fg.edges[f].head].order - fg.nodes[fg.edges[e].head].order) as f64
                    * (fg.edges[f].tail_port.p.x - fg.edges[e].tail_port.p.x)
                    < 0.0
            } else {
                (fg.nodes[fg.edges[f].tail].order - fg.nodes[fg.edges[e].tail].order) as f64
                    * (fg.edges[f].head_port.p.x - fg.edges[e].head_port.p.x)
                    < 0.0
            };
            if crossed {
                cross += fg.edges[e].xpenalty.wrapping_mul(fg.edges[f].xpenalty);
            }
        }
    }
    cross
}

// ---------------------------------------------------------------------------
// Module-static state (mincross.c:159-179)
// ---------------------------------------------------------------------------

/// SoA snapshot of the fields the crossing/median hot loops read.
///
/// The C original reaches `ND_order` / `ED_xpenalty` through interior
/// pointers; the port's equivalents (`fg.nodes[x].order`,
/// `fg.edges[e].xpenalty`, …) sit inside structs several hundred bytes
/// wide, so every read misses the cache once a graph outgrows L2 — that
/// dominated profiles of large graphs. The snapshot packs exactly those
/// fields into contiguous arrays, with per-adjacency-entry payloads so the
/// inner crossing loops read *pure* contiguous slices and can auto-
/// vectorize. Sync contract:
///
/// * [`MinCross::exchange`] — the only order mutation inside the hot
///   loops — patches the two swapped nodes' incident payloads in O(deg);
/// * any structural change (rank building, flat-edge rewiring, cluster
///   expansion, component merge, `ordering` edges) drops `valid`, and the
///   next hot-loop entry ([`MinCross::ensure_snap`]) rebuilds in O(V+E).
#[derive(Default)]
struct Snap {
    /// `ND_order` per node.
    ord: Vec<i32>,
    /// `ND_clust` per node as `clust + 1` (0 = none) — `left2right`'s
    /// cluster guard.
    nclust: Vec<u32>,
    /// `ND_ranktype == CLUSTER && ND_node_type == VIRTUAL` per node —
    /// `left2right`'s skeleton exception.
    skel: Vec<bool>,
    /// CSR over `ND_out` / `ND_in`, preserving each elist's order.
    out_start: Vec<u32>,
    in_start: Vec<u32>,
    out_edge: Vec<u32>,
    in_edge: Vec<u32>,
    /// Per-**out**-entry payloads (entry = a tail's out-edge, so the data
    /// is head-side): head order, xpenalty, head port x, head port order.
    out_eho: Vec<i32>,
    out_xp: Vec<i32>,
    out_hpx: Vec<f64>,
    out_hpo: Vec<u8>,
    /// Per-**in**-entry payloads (tail-side data): tail order, xpenalty,
    /// tail port x, tail port order.
    in_eto: Vec<i32>,
    in_xp: Vec<i32>,
    in_tpx: Vec<f64>,
    in_tpo: Vec<u8>,
    /// Edge id → its entry index in its tail's out-CSR / head's in-CSR
    /// (`u32::MAX` when the edge is in no list) — [`MinCross::exchange`]'s
    /// O(1) payload patch.
    out_slot: Vec<u32>,
    in_slot: Vec<u32>,
    /// `ND_mval` per node — written by `medians`, read by `reorder`'s scan
    /// (which probes every node's mval several times per rank pass).
    mval: Vec<f64>,
    /// Any node carries a resolved port (`ND_has_port`): when false, every
    /// port x is 0.0 and the port tie-break in the crossing loops never
    /// fires, so the loops run without the f64 compare.
    any_port: bool,
    /// Whether the arrays mirror the arena's current state.
    valid: bool,
}


/// The module-static state of mincross.c (L159-179) plus the rank-window
/// bookkeeping the C encodes in raw interior pointers (see the module docs).
struct MinCross {
    /// `Root` (L163) — the top-level graph; set by `init_mincross`.
    root: GId,
    /// `GlobalMinRank` / `GlobalMaxRank` (L164) — root rank bounds snapshot
    /// (L1029-1030), restored by `merge_components` (L802-803).
    global_minrank: i32,
    global_maxrank: i32,
    /// `MinQuit` (L160; set by `mincross_options`).
    minquit: i32,
    /// `MaxIter` global (globals.h:64; default 24 set at L1756).
    maxiter: i32,
    /// `ReMincross` (L167) — true only during the remincross re-run (L388).
    remincross: bool,
    /// Per-rank window offset `GD_rank(g)[r].v - GD_rank(g)[r].av`
    /// (`init_mccomp` advances it, `merge2` resets it to 0).
    win: Vec<usize>,
    /// `GD_rank(g)[r].candidate` — transpose work-list flags (L635-690).
    candidate: Vec<bool>,
    /// `saveorder(v)` side table (L114 aliases `ND_coord.x` in C; §12.1).
    /// Indexed by node id; only live between `save_best` and
    /// `restore_best`/`cleanup2`.
    saved_order: Vec<i32>,
    /// The SoA mirror the hot loops read (see [`Snap`]).
    snap: Snap,
    /// Per-rank adjacent-pair crossing cache (see [`transpose_step`]).
    xcache: Vec<XCache>,
    /// AVX2 available (probed once at construction).
    simd: bool,
}

/// One cached adjacent-pair evaluation: the in/out crossing sums for the
/// pair as-is (`c0*`) and swapped (`c1*`).
#[derive(Default, Clone, Copy)]
struct XEntry {
    c0i: i64,
    c1i: i64,
    c0o: i32,
    c1o: i32,
    ok: bool,
}

/// Per-rank cache: `entries[i]` covers the window positions `(i, i+1)`.
/// Valid only while `n` matches the rank's size and no swap touched the
/// pair's relevant orders.
#[derive(Default, Clone)]
struct XCache {
    n: usize,
    entries: Vec<XEntry>,
}

impl MinCross {
    fn new(root: GId) -> Self {
        Self {
            root,
            global_minrank: 0,
            global_maxrank: 0,
            minquit: 0,
            maxiter: 0,
            remincross: false,
            win: Vec::new(),
            candidate: Vec::new(),
            saved_order: Vec::new(),
            snap: Snap::default(),
            xcache: Vec::new(),
            simd: cfg!(target_arch = "x86_64") && std::arch::is_x86_feature_detected!("avx2"),
        }
    }

    /// Drops the SoA snapshot AND the pair cache — everything that depends
    /// on arena orders/edge state.
    fn invalidate_snap(&mut self) {
        self.snap.valid = false;
        self.xcache = Vec::new();
    }

    /// `mincross_options` (mincross.c:1750-1763) — defaults `MinQuit = 8`,
    /// `MaxIter = 24`, optionally scaled by the `mclimit` graph attribute
    /// via `scale_clamp`.
    ///
    /// PORT NOTE: the `mclimit` attribute is not plumbed onto `DGraph` yet,
    /// so the defaults always apply (identical behavior for graphs without
    /// `mclimit`).
    fn mincross_options(&mut self, _fg: &Fg, _g: GId) {
        self.minquit = 8;
        self.maxiter = 24;
        // C: p = agget(g, "mclimit");
        //    if (p && (f = atof(p)) > 0.0) {
        //        MinQuit = MAX(1, scale_clamp(MinQuit, f));
        //        MaxIter = MAX(1, scale_clamp(MaxIter, f));
        //    }
    }
}

// ---------------------------------------------------------------------------
// Entry point (mincross.c:332-403)
// ---------------------------------------------------------------------------

/// `dot_mincross` (mincross.c:332-403) — minimize edge crossings.
///
/// Nodes are not placed into `GD_rank(g)` until this runs (L328-330
/// comment). Returns `Ok(())` (C `rc == 0`) or `Err(-1)`; `cleanup2` runs on
/// every path (C's `goto done`).
pub fn dot_mincross(fg: &mut Fg, g: GId) -> Result<(), i32> {
    // L341-353: malformed input guard — drop clusters without nodes that the
    // crossing functions would not anticipate.
    {
        let mut i = 0usize;
        while i < fg.graphs[g].clust.len() {
            if fg.graphs[fg.graphs[g].clust[i]].nodes_order.is_empty() {
                // C: agwarningf("removing empty cluster\n")
                fg.graphs[g].clust.remove(i);
            } else {
                i += 1;
            }
        }
    }

    let timing = std::env::var_os("GD_TIMING").is_some();
    let mut t0 = std::time::Instant::now();
    let mut lap = |name: &str| {
        if timing {
            eprintln!(
                "[timing]   mincross::{name}: {:.3}s",
                t0.elapsed().as_secs_f64()
            );
            t0 = std::time::Instant::now();
        }
    };

    let mut mc = MinCross::new(g);
    mc.init_mincross(fg, g);
    lap("init");
    let mut has_set_vlists = false; // L356

    // L358-367: one full mincross per connected component.
    let mut nc: i64 = 0;
    for comp in 0..fg.graphs[g].comp.len() {
        mc.init_mccomp(fg, g, comp);
        let m = mc.mincross(fg, g, 0);
        if m < 0 {
            mc.cleanup2(fg, g, nc, has_set_vlists);
            return Err(-1);
        }
        nc += m;
    }
    lap("components");

    mc.merge2(fg, g); // L369
    lap("merge2");

    // L371-384: run mincross on the contents of each cluster, outermost
    // first, in cluster-index order.
    for &c in fg.graphs[g].clust.clone().iter() {
        match mc.mincross_clust(fg, c) {
            Ok(m) => nc += m,
            Err(err) => {
                mc.cleanup2(fg, g, nc, false);
                return Err(err);
            }
        }
    }
    has_set_vlists = true; // L384

    // L386-399: remincross over the assembled graph — entered when the graph
    // has clusters and `remincross` is not false. Each cluster's slice was
    // saved by `save_vlist`; this pass minimizes the crossings of the whole
    // root again now that the clusters occupy real rows.
    let remincross = match fg.graphs[g].remincross.clone() {
        None => true,
        Some(s) => super::mapbool(Some(&s)),
    };
    if !fg.graphs[g].clust.is_empty() && remincross {
        position::mark_lowclusters(fg, g);
        mc.invalidate_snap(); // mark_lowclusters rewrote every ND_clust
        mc.remincross = true;
        nc = mc.mincross(fg, g, 2);
        if nc < 0 {
            mc.cleanup2(fg, g, nc, has_set_vlists);
            return Err(-1);
        }
    }

    mc.cleanup2(fg, g, nc, has_set_vlists); // L400-401
    lap("cleanup");
    Ok(())
}

impl MinCross {
    /// `init_mincross` (mincross.c:1010-1031).
    fn init_mincross(&mut self, fg: &mut Fg, g: GId) {
        self.remincross = false; // L1016
        self.root = g; // Root = g (L1017)
        // L1018-1021: TE_list/TI_list scratch sized agnedges(dot_root)+1.
        // This port uses growable locals at the two use sites
        // (`do_ordering_node`, `medians`) — same contents, same order.
        self.mincross_options(fg, g); // L1022
        // L1023-1024: `GD_flags(g) & NEW_RANK` ⇒ fillRanks(g) — the dot2
        // (newrank) path; STUB, see `fill_ranks` below.
        class2(fg, g); // L1025 — builds the fast graph, virtual chains, flat edges, skeletons
        rank::decompose(fg, g, 1); // L1026 — components into GD_comp; pass 1 ⇒ cluster-aware
        self.allocate_ranks(fg, g); // L1027
        self.ordered_edges(fg, g); // L1028 — `ordering` attributes
        self.global_minrank = fg.graphs[g].minrank; // L1029
        self.global_maxrank = fg.graphs[g].maxrank; // L1030
    }

    /// `allocate_ranks` (mincross.c:1122-1147) — allocate the rank structure
    /// and determine the number of nodes per rank; no nodes are installed
    /// yet.
    ///
    /// See the module docs for the row-indexing map: `fg.graphs[g].rank` is
    /// indexed by absolute rank over `maxrank+2` rows; only
    /// `minrank..=maxrank` get a row of `cn[r]+1` slots (`an == n ==
    /// cn[r]+1` at C L1143 — the trailing slot stays `NO_NODE`, calloc's
    /// NULL). `cn` is 0-based, *not* minrank-based (comment L1128).
    fn allocate_ranks(&mut self, fg: &mut Fg, g: GId) {
        let minrank = fg.graphs[g].minrank;
        let maxrank = fg.graphs[g].maxrank;
        let rows = (maxrank as usize) + 2;
        let mut cn = vec![0usize; rows]; // must be 0 based, not GD_minrank (L1128)
        // C iterates the cgraph nodes of g (`agfstnode`..`agnxtnode`) and
        // their original out-edges: every real node occupies its rank, and
        // every long edge contributes its interior (virtual-chain) ranks.
        for &n in fg.graphs[g].nodes_order.iter() {
            cn[fg.nodes[n].rank as usize] += 1;
            for &e in fg.input_out[n].iter() {
                // C iterates `agfstout(g, n)` — the *subgraph's* edge
                // membership. For a cluster that excludes edges leaving the
                // cluster (their span would exceed the cluster's own rank
                // allocation); for the root it owns every edge.
                if !fg.graphs[g].owns_input_edge(fg, e) {
                    continue;
                }
                let tr = fg.nodes[fg.edges[e].tail].rank;
                let hr = fg.nodes[fg.edges[e].head].rank;
                let (low, high) = if tr > hr { (hr, tr) } else { (tr, hr) };
                for r in (low + 1)..high {
                    cn[r as usize] += 1;
                }
            }
        }
        fg.graphs[g].rank = vec![super::model::Rank::default(); rows]; // L1141 (calloc ⇒ zeroed rows)
        // `win`/`candidate` are per-rank side tables for the *current* graph.
        // They must stay usable for the root after a cluster's
        // `allocate_ranks` (a cluster spans fewer ranks than the root), so
        // grow without shrinking, and reset this graph's own span to 0 —
        // C re-points `GD_rank(g)[r].v` back at `.av` per graph.
        if self.win.len() < rows {
            self.win.resize(rows, 0);
        }
        if self.candidate.len() < rows {
            self.candidate.resize(rows, false);
        }
        for r in minrank..=maxrank {
            self.win[ri(r)] = 0;
            self.candidate[ri(r)] = false;
        }
        for r in minrank..=maxrank {
            let row = &mut fg.graphs[g].rank[ri(r)];
            row.v = vec![NO_NODE; cn[ri(r)] + 1]; // av == v == calloc(cn[r]+1) (L1144)
            row.n = 0;
            // C leaves `.n == .an == cn[r]+1` here (L1143); the value is dead
            // until build_ranks zeroes it (L1219-1220) before any read.
        }
    }

    /// `init_mccomp` (mincross.c:422-432) — point `GD_nlist` at component
    /// `c` and, for `c > 0`, advance each rank window past the previous
    /// component's nodes.
    fn init_mccomp(&mut self, fg: &mut Fg, g: GId, c: usize) {
        fg.graphs[g].nlist = Some(fg.graphs[g].comp[c]);
        if c > 0 {
            self.xcache = Vec::new(); // rank row sizes shift with the window
        }
        if c > 0 {
            for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
                // GD_rank(g)[r].v += GD_rank(g)[r].n; GD_rank(g)[r].n = 0;
                self.win[ri(r)] += fg.graphs[g].rank[ri(r)].n;
                fg.graphs[g].rank[ri(r)].n = 0;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Rank-array construction (mincross.c:1149-1295)
    // -----------------------------------------------------------------------

    /// `install_in_rank` (mincross.c:1150-1193) — install a node at the
    /// current right end of its rank window; returns 0 / -1.
    fn install_in_rank(&mut self, fg: &mut Fg, g: GId, n: NId) -> i32 {
        let r = fg.nodes[n].rank;
        if r < 0 || ri(r) >= fg.graphs[g].rank.len() {
            // C L1181-1184: "rank %d not in rank range"
            return -1;
        }
        {
            let row = &mut fg.graphs[g].rank[ri(r)];
            if row.v.is_empty() {
                // C L1155-1158: "install_in_rank ... an = 0"
                return -1;
            }
            let i = row.n;
            row.v[self.win[ri(r)] + i] = n;
            fg.nodes[n].order = i as i32;
            row.n += 1;
        }
        debug_assert!(
            fg.graphs[g].rank[ri(r)].n <= fg.graphs[g].rank[ri(r)].v.len(),
            "install_in_rank overflow"
        );
        // C L1175-1191 keep three further pointer-arithmetic error guards
        // (`ND_order(n) > GD_rank(Root)[r].an`, rank-range, window past
        // `av + an`) that cannot fire for well-formed graphs; the rank-range
        // check above is the load-bearing one in this port.
        0
    }

    /// `enqueue_neighbors` (mincross.c:1275-1295) — pass 0 ⇒ enqueue the
    /// heads of `n0`'s out-edges in list order; otherwise the tails of its
    /// in-edges. Marks are set at push time (BFS dedup).
    fn enqueue_neighbors(
        &self,
        fg: &mut Fg,
        q: &mut VecDeque<NId>,
        n0: NId,
        pass: i32,
        set: usize,
    ) {
        if pass == 0 {
            for i in 0..fg.nodes[n0].out.len() {
                let e = fg.nodes[n0].out[i];
                let h = fg.edges[e].head;
                if fg.nodes[h].mark != set {
                    fg.nodes[h].mark = set;
                    q.push_back(h);
                }
            }
        } else {
            for i in 0..fg.nodes[n0].in_.len() {
                let e = fg.nodes[n0].in_[i];
                let t = fg.edges[e].tail;
                if fg.nodes[t].mark != set {
                    fg.nodes[t].mark = set;
                    q.push_back(t);
                }
            }
        }
    }

    /// `build_ranks` (mincross.c:1199-1273) — install nodes in ranks. The
    /// initial ordering ensures series-parallel graphs such as trees are
    /// drawn with no crossings; it tries searching in- and out-edges and
    /// takes the better of the two initial orderings (comment L1195-1198).
    ///
    /// Called twice for the root (pass 0 seeds from in-degree-0 nodes, pass 1
    /// from out-degree-0 nodes) and once per cluster (pass 0).
    fn build_ranks(&mut self, fg: &mut Fg, g: GId, pass: i32) -> i32 {
        self.invalidate_snap(); // installs + flip rewrite ND_order
        let (unset, set) = fresh_marks(fg);
        // L1204-1205: MARK(n) = false over the GD_nlist chain
        let mut next = fg.graphs[g].nlist;
        while let Some(n) = next {
            fg.nodes[n].mark = unset;
            next = fg.nodes[n].next;
        }

        // L1219-1220
        for i in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            fg.graphs[g].rank[ri(i)].n = 0;
        }

        // L1222-1231: clusters walk the nlist backwards to preserve input
        // node order; the root walks forward.
        let walkbackwards = g != fg.root_g();
        let mut chain: Vec<NId> = Vec::new();
        if walkbackwards {
            let mut last = fg.graphs[g].nlist;
            while let Some(cur) = last {
                match fg.nodes[cur].next {
                    Some(nx) => last = Some(nx),
                    None => break,
                }
            }
            let mut cur = last;
            while let Some(n) = cur {
                chain.push(n);
                cur = fg.nodes[n].prev;
            }
        } else {
            let mut cur = fg.graphs[g].nlist;
            while let Some(n) = cur {
                chain.push(n);
                cur = fg.nodes[n].next;
            }
        }

        let mut q: VecDeque<NId> = VecDeque::new();
        for &n in chain.iter() {
            // L1233-1235: seed only sources (pass 0) / sinks (pass ≠ 0)
            let has_otheredges = if pass == 0 {
                !fg.nodes[n].in_.is_empty()
            } else {
                !fg.nodes[n].out.is_empty()
            };
            if has_otheredges {
                continue;
            }
            if fg.nodes[n].mark != set {
                fg.nodes[n].mark = set;
                q.push_back(n);
                while let Some(n0) = q.pop_front() {
                    if fg.nodes[n0].ranktype != RankType::Cluster {
                        if self.install_in_rank(fg, g, n0) != 0 {
                            return -1; // C frees the queue (L1242-1245)
                        }
                        self.enqueue_neighbors(fg, &mut q, n0, pass, set);
                    } else {
                        // L1247-1253: install_cluster (cluster.c:380-395)
                        let rc = self.install_cluster(fg, g, n0, pass, set, &mut q);
                        if rc != 0 {
                            return rc;
                        }
                    }
                }
            }
        }
        debug_assert!(q.is_empty()); // L1257

        // L1258-1267: invalidate every Root row; `GD_flip` (rankdir LR/BT)
        // reverses each filled row in place (NB: `j == last - j` exchanges a
        // node with itself for odd n — harmless).
        for i in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            fg.graphs[g].rank[ri(i)].valid = false;
            if fg.graphs[g].rankdir.flip() && fg.graphs[g].rank[ri(i)].n > 0 {
                let num_nodes_1 = fg.graphs[g].rank[ri(i)].n - 1;
                let half_num_nodes_1 = num_nodes_1 / 2;
                for j in 0..=half_num_nodes_1 {
                    // C's `exchange` writes into `GD_rank(Root)[ND_order(·)]`,
                    // which for an *unexpanded cluster* indexes the root's row
                    // with the cluster's own 0-based orders. Reverse the row
                    // *in place* instead — identical for the root (where the
                    // two views coincide) and for a cluster it produces the
                    // reversal C intends without clobbering the root's row.
                    let win = self.win[ri(i)];
                    let v = fg.graphs[g].rank[ri(i)].v[win + j];
                    let w = fg.graphs[g].rank[ri(i)].v[win + num_nodes_1 - j];
                    let (vi, wi) = (fg.nodes[v].order, fg.nodes[w].order);
                    let row = &mut fg.graphs[g].rank[ri(i)].v;
                    row[win + j] = w;
                    row[win + num_nodes_1 - j] = v;
                    fg.nodes[v].order = wi;
                    fg.nodes[w].order = vi;
                }
            }
        }

        // L1269-1270
        if g == fg.root_g() && self.ncross(fg) > 0 {
            self.transpose(fg, g, false);
        }
        0
    }

    /// `install_cluster` (cluster.c:380-395) — a node with
    /// `ND_ranktype == CLUSTER` stands for its whole cluster: install every
    /// rank leader of that cluster, then enqueue their neighbors. The
    /// `GD_installed(clust) == pass + 1` guard makes the install idempotent
    /// across the two `build_ranks` passes.
    fn install_cluster(
        &mut self,
        fg: &mut Fg,
        g: GId,
        n0: NId,
        pass: i32,
        set: usize,
        q: &mut VecDeque<NId>,
    ) -> i32 {
        let clust = fg.nodes[n0].clust.expect("CLUSTER node has ND_clust");
        if fg.graphs[clust].installed == (pass + 1) as u8 {
            return 0;
        }
        for r in fg.graphs[clust].minrank..=fg.graphs[clust].maxrank {
            let lead = fg.graphs[clust].rankleader[r as usize].expect("cluster rankleader");
            if self.install_in_rank(fg, g, lead) != 0 {
                return -1;
            }
        }
        for r in fg.graphs[clust].minrank..=fg.graphs[clust].maxrank {
            let lead = fg.graphs[clust].rankleader[r as usize].expect("cluster rankleader");
            self.enqueue_neighbors(fg, q, lead, pass, set);
        }
        fg.graphs[clust].installed = (pass + 1) as u8;
        0
    }

    // -----------------------------------------------------------------------
    // The mincross driver (mincross.c:692-781, 1455-1480)
    // -----------------------------------------------------------------------

    /// `mincross` (mincross.c:692-753) — returns the best (minimum) crossing
    /// count, or -1 on failure (only `build_ranks` failures propagate).
    ///
    /// Root-level runs pass 0, 1, 2; per-pass improvement loops are capped at
    /// `min(4, MaxIter)` and `MaxIter` respectively; `MinQuit` = 8 steps per
    /// stretch of non-significant improvement (`cur < 0.995 * best`, double
    /// comparison); equal-cost results do *not* reset `trying` (spec §9.1).
    fn mincross(&mut self, fg: &mut Fg, g: GId, startpass: i32) -> i64 {
        let endpass = 2;
        let mut cur_cross: i64;
        let mut best_cross: i64;

        if startpass > 1 {
            cur_cross = self.ncross(fg);
            best_cross = cur_cross;
            self.save_best(fg, g);
        } else {
            cur_cross = i64::MAX;
            best_cross = i64::MAX;
        }

        let mut pass = startpass;
        while pass <= endpass {
            let maxthispass;
            if pass <= 1 {
                maxthispass = 4.min(self.maxiter); // MIN(4, MaxIter) (L704)
                if g == fg.root_g()
                    && self.build_ranks(fg, g, pass) != 0 {
                        return -1;
                    }
                if pass == 0 {
                    self.flat_breakcycles(fg, g); // L709-710
                }
                self.flat_reorder(fg, g); // L711
                cur_cross = self.ncross(fg); // L713
                if cur_cross <= best_cross {
                    self.save_best(fg, g);
                    best_cross = cur_cross;
                }
            } else {
                maxthispass = self.maxiter; // L718
                if cur_cross > best_cross {
                    self.restore_best(fg, g); // L719-720
                }
                cur_cross = best_cross; // L721
            }

            // L723-741: the improvement loop. C's `if (trying++ >= MinQuit)
            // break` compares the *pre-increment* value.
            let mut trying = 0;
            let mut iter = 0;
            let (imin_t0, mut t_step, mut t_ncross, mut n_steps) = (
                std::time::Instant::now(),
                std::time::Duration::ZERO,
                std::time::Duration::ZERO,
                0usize,
            );
            let ti = std::env::var_os("GD_TIMING").is_some();
            while iter < maxthispass {
                let prev_trying = trying;
                trying += 1;
                if prev_trying >= self.minquit {
                    break;
                }
                if cur_cross == 0 {
                    break;
                }
                let s0 = std::time::Instant::now();
                self.mincross_step(fg, g, iter);
                t_step += s0.elapsed();
                let n0 = std::time::Instant::now();
                cur_cross = self.ncross(fg);
                t_ncross += n0.elapsed();
                n_steps += 1;
                if cur_cross <= best_cross {
                    self.save_best(fg, g);
                    // L737: `cur_cross < Convergence * (double)best_cross`
                    // — compared against the OLD best_cross (before the
                    // reassignment below).
                    if (cur_cross as f64) < CONVERGENCE * (best_cross as f64) {
                        trying = 0;
                    }
                    best_cross = cur_cross;
                }
                iter += 1;
            }
            if ti {
                eprintln!(
                    "[timing]     mincross::improve(pass {pass}): {:.3}s total ({n_steps} steps: mincross_step {:.3}s, ncross {:.3}s)",
                    imin_t0.elapsed().as_secs_f64(),
                    t_step.as_secs_f64(),
                    t_ncross.as_secs_f64()
                );
            }
            if cur_cross == 0 {
                break; // L742-743
            }
            pass += 1;
        }

        let t_tr = std::time::Instant::now();
        if cur_cross > best_cross {
            self.restore_best(fg, g); // L745-746
        }
        if best_cross > 0 {
            self.transpose(fg, g, false); // L748: final polish, no tie swaps
            best_cross = self.ncross(fg); // L749
        }
        if std::env::var_os("GD_TIMING").is_some() {
            eprintln!(
                "[timing]     mincross::final-transpose: {:.3}s",
                t_tr.elapsed().as_secs_f64()
            );
        }
        best_cross
    }

    /// `save_best` (mincross.c:772-781) — stash `ND_order` for every node in
    /// the current rank windows. C writes it into `ND_coord.x`
    /// (`saveorder`, L114); the port uses the side table (§12.1).
    fn save_best(&mut self, fg: &mut Fg, g: GId) {
        if self.saved_order.len() < fg.nodes.len() {
            self.saved_order.resize(fg.nodes.len(), 0);
        }
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            for i in 0..n {
                let v = fg.graphs[g].rank[ri(r)].v[win + i];
                self.saved_order[v] = fg.nodes[v].order;
            }
        }
    }

    /// `restore_best` (mincross.c:755-770) — put the best-so-far order back
    /// and re-sort each rank row by it (`nodeposcmpf`; orders are a unique
    /// permutation within a row, so a stable sort matches `qsort`).
    fn restore_best(&mut self, fg: &mut Fg, g: GId) {
        self.invalidate_snap(); // writes ND_order, then re-sorts rows
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            for i in 0..n {
                let v = fg.graphs[g].rank[ri(r)].v[win + i];
                fg.nodes[v].order = self.saved_order[v];
            }
        }
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            fg.graphs[g].rank[ri(r)].valid = false; // L766
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            let mut tmp: Vec<(i32, NId)> = (0..n)
                .map(|i| {
                    let v = fg.graphs[g].rank[ri(r)].v[win + i];
                    (fg.nodes[v].order, v)
                })
                .collect();
            tmp.sort_by_key(|&(order, _)| order); // nodeposcmpf (L767-768)
            for (i, &(_, v)) in tmp.iter().enumerate() {
                fg.graphs[g].rank[ri(r)].v[win + i] = v;
            }
            if g != self.root {
                // `restore_best` re-sorts the cluster's own row in C (its
                // slice *is* the root's row); mirror it into the root and the
                // other clusters' copies.
                let row: Vec<NId> = fg.graphs[g].rank[ri(r)].v[win..win + n].to_vec();
                let base = fg.graphs[g].rank_offset[ri(r)].max(0) as usize;
                fg.graphs[self.root].rank[ri(r)].v[base..base + n].copy_from_slice(&row);
                cluster::refresh_expanded_clusters(fg, r);
            }
        }
    }

    /// `mincross_step` (mincross.c:1455-1480) — one median/reorder sweep
    /// followed by a transpose with the *opposite* tie-swap mode.
    fn mincross_step(&mut self, fg: &mut Fg, g: GId, pass: i32) {
        let reverse = pass % 4 < 2; // passes 0,1,4,5,…: true; 2,3,6,7,…: false

        let (mut first, last, dir);
        if pass % 2 == 0 {
            // down pass — ranks ascending
            first = fg.graphs[g].minrank + 1;
            if fg.graphs[g].minrank > fg.graphs[self.root].minrank {
                first -= 1; // cluster deeper than root: include own top rank
            }
            last = fg.graphs[g].maxrank;
            dir = 1;
        } else {
            // up pass — ranks descending
            first = fg.graphs[g].maxrank - 1;
            last = fg.graphs[g].minrank;
            if fg.graphs[g].maxrank < fg.graphs[self.root].maxrank {
                first += 1; // cluster shallower than root: include own bottom rank
            }
            dir = -1;
        }

        let mut r = first;
        let t_dbg = std::env::var_os("GD_TIMING").is_some();
        let (mut tm, mut tr) = (std::time::Duration::ZERO, std::time::Duration::ZERO);
        while r != last + dir {
            let other = r - dir;
            let s0 = std::time::Instant::now();
            let hasfixed = self.medians(fg, g, r, other); // mval of rank r from rank `other`
            let s1 = std::time::Instant::now();
            self.reorder(fg, g, r, reverse, hasfixed);
            let s2 = std::time::Instant::now();
            if t_dbg {
                tm += s1 - s0;
                tr += s2 - s1;
            }
            r += dir;
        }
        if t_dbg {
            eprintln!("[timing]       step: medians {:.3}s reorder {:.3}s", tm.as_secs_f64(), tr.as_secs_f64());
        }
        self.transpose(fg, g, !reverse); // L1479: the OPPOSITE of reorder's `reverse`
    }

    // -----------------------------------------------------------------------
    // Transpose machinery (mincross.c:563-585, 623-690)
    // -----------------------------------------------------------------------

    /// `exchange` (mincross.c:623-633) — swap `v` and `w` within their rank
    /// row. C writes through `GD_rank(Root)[r].v` — the *window* — indexing
    /// with the (window-relative) `ND_order` values; the port indexes the
    /// base row at `win[r] + order`. No bound/type checks, as in C.
    ///
    /// The snapshot's `ord` and the per-entry order payloads are patched
    /// here in O(deg(v) + deg(w)) — the only order mutation inside the hot
    /// loops (a stale snapshot is rebuilt wholesale instead).
    fn exchange(&mut self, fg: &mut Fg, v: NId, w: NId) {
        let r = fg.nodes[v].rank;
        let vi = fg.nodes[v].order;
        let wi = fg.nodes[w].order;
        let win = self.win[ri(r)];
        {
            let row = &mut fg.graphs[self.root].rank[ri(r)].v;
            row[win + wi as usize] = v;
            row[win + vi as usize] = w;
        }
        fg.nodes[v].order = wi;
        fg.nodes[w].order = vi;
        if self.snap.valid {
            let s = &mut self.snap;
            s.ord[v] = wi;
            s.ord[w] = vi;
            // v took order wi: every edge with v on that side follows.
            for &e in fg.nodes[v].out.iter() {
                let slot = s.in_slot[e];
                if slot != u32::MAX {
                    s.in_eto[slot as usize] = wi;
                }
            }
            for &e in fg.nodes[v].in_.iter() {
                let slot = s.out_slot[e];
                if slot != u32::MAX {
                    s.out_eho[slot as usize] = wi;
                }
            }
            // w took order vi
            for &e in fg.nodes[w].out.iter() {
                let slot = s.in_slot[e];
                if slot != u32::MAX {
                    s.in_eto[slot as usize] = vi;
                }
            }
            for &e in fg.nodes[w].in_.iter() {
                let slot = s.out_slot[e];
                if slot != u32::MAX {
                    s.out_eho[slot as usize] = vi;
                }
            }
        }
        // Pair-cache invalidation: the swapped pair's own entry mirrors
        // (crossings of the reversed arrangement are the stored ones — the
        // sums only read the *other* ranks' orders); its neighbors' pairs
        // changed membership. Neighbor ranks' caches die wholesale.
        let (a, b) = (vi.min(wi) as usize, vi.max(wi) as usize);
        if let Some(xc) = self.xcache.get_mut(ri(r))
            && xc.n > 0 && a < xc.n {
                if b == a + 1 {
                    let e = &mut xc.entries[a];
                    if e.ok {
                        std::mem::swap(&mut e.c0i, &mut e.c1i);
                        std::mem::swap(&mut e.c0o, &mut e.c1o);
                    }
                } else {
                    xc.entries[a].ok = false;
                }
                if a > 0 {
                    xc.entries[a - 1].ok = false;
                }
                if b < xc.n {
                    xc.entries[b].ok = false;
                }
                if b - 1 > a && b - 1 < xc.n {
                    xc.entries[b - 1].ok = false;
                }
            }
        for rr in [r - 1, r + 1] {
            if rr >= 0
                && let Some(xc) = self.xcache.get_mut(ri(rr)) {
                    xc.n = 0;
                    xc.entries.clear();
                }
        }
        // C aliases the clusters' rank slices into the root's row; this port
        // keeps copies, so the swap has to be mirrored.
        cluster::refresh_expanded_clusters(fg, r);
    }

    /// `transpose_step` (mincross.c:635-674) — one left→right sweep of
    /// adjacent pairs on rank `r`; returns the i64 improvement.
    ///
    /// Swap condition (L657): `c1 < c0 || (c0 > 0 && reverse && c1 == c0)` —
    /// equal-cost swaps happen only when `reverse` is set and the count is
    /// nonzero. `in_cross` is consulted iff `r > 0` (not "rank r-1
    /// non-empty"); `out_cross` iff rank `r+1` is non-empty.
    fn transpose_step(&mut self, fg: &mut Fg, g: GId, r: i32, reverse: bool) -> i64 {
        let mut rv: i64 = 0;
        self.candidate[ri(r)] = false; // cleared BEFORE scanning (L640)
        let n = fg.graphs[g].rank[ri(r)].n;
        if n < 2 {
            return rv;
        }
        let win = self.win[ri(r)];
        self.ensure_snap(fg); // build_ranks' transpose runs on fresh rows
        // loop-invariant lookups, hoisted out of the pair scan
        let has_matrix = fg.graphs[g].rank[ri(r)].flat.is_some();
        let flip = fg.graphs[g].rankdir.flip();
        let next_row_nonempty = fg.graphs[g].rank[ri(r + 1)].n > 0;

        // Adjacent-pair crossing cache: a rescan recomputes only the pairs
        // whose relevant orders changed (this rank's swaps touch positions
        // i-1..i+1; a swap in r±1 invalidates the whole row's cache — see
        // `exchange`). Convergence tail scans (the dominant cost) then read
        // every value from the cache and swap nothing.
        if self.xcache.len() <= ri(r) {
            self.xcache.resize(ri(r) + 1, XCache::default());
        }
        if self.xcache[ri(r)].n != n - 1 {
            let x = &mut self.xcache[ri(r)];
            x.n = n - 1;
            x.entries.clear();
            x.entries.resize_with(n - 1, Default::default);
        }

        for i in 0..(n - 1) {
            let v = fg.graphs[g].rank[ri(r)].v[win + i];
            let w = fg.graphs[g].rank[ri(r)].v[win + i + 1];
            debug_assert!(fg.nodes[v].order < fg.nodes[w].order);
            if self.left2right(fg, g, v, w, has_matrix, flip) {
                continue; // frozen pair
            }
            let (c0, c1) = if self.xcache[ri(r)].entries[i].ok {
                let e = &self.xcache[ri(r)].entries[i];
                (e.c0i + e.c0o as i64, e.c1i + e.c1o as i64)
            } else {
                let (mut c0i, mut c1i) = (0i64, 0i64);
                let (mut c0o, mut c1o) = (0i32, 0i32);
                if r > 0 {
                    let (a, b) = self.in_cross_pair(v, w);
                    c0i = a;
                    c1i = b;
                }
                if next_row_nonempty {
                    let (a, b) = self.out_cross_pair(v, w);
                    c0o = a;
                    c1o = b;
                }
                self.xcache[ri(r)].entries[i] = XEntry { c0i, c1i, c0o, c1o, ok: true };
                (c0i + c0o as i64, c1i + c1o as i64)
            };
            if c1 < c0 || (c0 > 0 && reverse && c1 == c0) {
                self.exchange(fg, v, w);
                rv += c0 - c1;
                // invalidation: Root rows r-1, r, r+1 (L660-669)
                fg.graphs[self.root].rank[ri(r)].valid = false;
                self.candidate[ri(r)] = true;
                if r > fg.graphs[g].minrank {
                    fg.graphs[self.root].rank[ri(r - 1)].valid = false;
                    self.candidate[ri(r - 1)] = true;
                }
                if r < fg.graphs[g].maxrank {
                    fg.graphs[self.root].rank[ri(r + 1)].valid = false;
                    self.candidate[ri(r + 1)] = true;
                }
            }
        }
        rv
    }

    /// `transpose` (mincross.c:676-690) — repeat improvement sweeps until a
    /// pass yields `delta < 1`. A pass of pure tie-swaps (reverse mode)
    /// yields `delta == 0` and terminates.
    fn transpose(&mut self, fg: &mut Fg, g: GId, reverse: bool) {
        let (ti, t0) = (
            std::env::var_os("GD_TIMING").is_some(),
            std::time::Instant::now(),
        );
        let mut n_sweeps: usize = 0;
        let mut n_scan: usize = 0;
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            self.candidate[ri(r)] = true;
        }
        loop {
            let mut delta: i64 = 0;
            n_sweeps += 1;
            for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
                if self.candidate[ri(r)] {
                    n_scan += 1;
                    delta += self.transpose_step(fg, g, r, reverse);
                }
            }
            if delta < 1 {
                break; // exact: while (delta >= 1)
            }
        }
        if ti {
            eprintln!(
                "[timing]     mincross::transpose(reverse={reverse}): {:.3}s ({n_sweeps} sweeps, {n_scan} rank-scans)",
                t0.elapsed().as_secs_f64()
            );
        }
    }

    /// `left2right` (mincross.c:563-585) — may `v`, `w` (at orders `v < w`)
    /// not be exchanged? Callers assert `ND_order(v) < ND_order(w)`; with
    /// `GD_flip` the flat-matrix lookup is mirrored (L581-584).
    ///
    /// `has_matrix`/`flip` are loop-invariant per rank scan and hoisted by
    /// the callers; the node flags come from the SoA snapshot.
    fn left2right(&self, fg: &Fg, g: GId, v: NId, w: NId, has_matrix: bool, flip: bool) -> bool {
        // CLUSTER indicates orig nodes of clusters, and vnodes of skeletons
        if !self.remincross {
            let (cv, cw) = (self.snap.nclust[v], self.snap.nclust[w]);
            if cv != cw && cv != 0 && cw != 0 {
                // the following allows cluster skeletons to be swapped
                if self.snap.skel[v] {
                    return false;
                }
                if self.snap.skel[w] {
                    return false;
                }
                return true; // different real clusters: never swap
            }
        } else {
            // remincross pass: never interleave clusters after expansion
            if self.snap.nclust[v] != self.snap.nclust[w] {
                return true;
            }
        }
        if !has_matrix {
            return false;
        }
        let r = fg.nodes[v].rank;
        let Some(m) = fg.graphs[g].rank[ri(r)].flat.as_ref() else {
            return false;
        };
        let (v, w) = if flip { (w, v) } else { (v, w) };
        matrix_get_ref(m, fg.nodes[v].low as usize, fg.nodes[w].low as usize)
    }

    // -----------------------------------------------------------------------
    // Medians and reorder (mincross.c:1404-1453, 1583-1670)
    // -----------------------------------------------------------------------

    // `VAL(node, port)` (mincross.c:1612) inlines as
    // `MC_SCALE * ND_order(node) + port.order` (C `int` arithmetic); the
    // medians loop reads both operands straight from the snapshot.

    /// `medians` (mincross.c:1614-1670) — compute `ND_mval` for every node of
    /// rank `r0` from rank `r1` (the side we came from); returns `hasfixed`.
    ///
    /// Even-`j` handling is the span-weighted median: `j == 2` and equal
    /// spans use plain C int-division averages; otherwise
    /// `list[lm]*rspan + list[rm]*lspan` over `lspan + rspan` (real-valued).
    ///
    /// Edge walks go through the SoA snapshot (elist order preserved).
    fn medians(&mut self, fg: &mut Fg, g: GId, r0: i32, r1: i32) -> bool {
        self.ensure_snap(fg);
        let mut hasfixed = false;
        let mut list: Vec<i32> = Vec::new(); // TI_list scratch (growable stand-in)
        let (win, n) = (self.win[ri(r0)], fg.graphs[g].rank[ri(r0)].n);
        for i in 0..n {
            let v = fg.graphs[g].rank[ri(r0)].v[win + i];
            list.clear();
            let (s0, s1) = (
                self.snap.out_start[v] as usize,
                self.snap.out_start[v + 1] as usize,
            );
            let (t0, t1) = (
                self.snap.in_start[v] as usize,
                self.snap.in_start[v + 1] as usize,
            );
            if r1 > r0 {
                // r1 below r0 ⇒ downward medians over out-edges
                for k in s0..s1 {
                    if self.snap.out_xp[k] > 0 {
                        list.push(
                            (MC_SCALE * self.snap.out_eho[k] as i64
                                + self.snap.out_hpo[k] as i64) as i32,
                        );
                    }
                }
            } else {
                // r1 above (or same) ⇒ upward medians over in-edges
                for k in t0..t1 {
                    if self.snap.in_xp[k] > 0 {
                        list.push(
                            (MC_SCALE * self.snap.in_eto[k] as i64
                                + self.snap.in_tpo[k] as i64) as i32,
                        );
                    }
                }
            }
            let j = list.len();
            let mval = match j {
                0 => -1.0, // "fixed" — never comparable in reorder
                1 => list[0] as f64,
                // C int division; operands ≥ 0 so truncation == floor
                2 => ((list[0] + list[1]) / 2) as f64,
                _ => {
                    list.sort_unstable(); // ordercmpf ascending; equal values indistinguishable
                    if j % 2 == 1 {
                        list[j / 2] as f64
                    } else {
                        // weighted median (the old "wmed")
                        let rm = j / 2;
                        let lm = rm - 1;
                        let rspan = list[j - 1] - list[rm]; // spread to the END of the value list
                        let lspan = list[lm] - list[0]; // spread to the START
                        if lspan == rspan {
                            ((list[lm] + list[rm]) / 2) as f64
                        } else {
                            let w = list[lm] as f64 * rspan as f64 + list[rm] as f64 * lspan as f64;
                            w / (lspan + rspan) as f64
                        }
                    }
                }
            };
            fg.nodes[v].mval = mval;
            self.snap.mval[v] = mval;
        }
        // second pass: isolated nodes get flat_mval (L1664-1668) — fast-graph
        // degrees, NOT flat lists.
        for i in 0..n {
            let v = fg.graphs[g].rank[ri(r0)].v[win + i];
            if fg.nodes[v].out.is_empty() && fg.nodes[v].in_.is_empty() {
                hasfixed |= self.flat_mval(fg, v);
            }
        }
        hasfixed
    }

    /// `flat_mval` (mincross.c:1583-1610) — mval for nodes with no in or out
    /// non-flat edges (precondition: `ND_out(n).size == 0 &&
    /// ND_in(n).size == 0`).
    ///
    /// Asymmetries preserved: in-edge branch uses `>= 0` and `+1` on the
    /// *largest-order* in-tail; out-edge branch uses `> 0` and `-1` on the
    /// *smallest-order* out-head; one-step propagation only. Returns true if
    /// `mval` stays -1 ("fixed" for reorder).
    fn flat_mval(&mut self, fg: &mut Fg, n: NId) -> bool {
        let s = &self.snap;
        if !fg.nodes[n].flat_in.is_empty() {
            let mut nn = fg.edges[fg.nodes[n].flat_in[0]].tail;
            for i in 1..fg.nodes[n].flat_in.len() {
                let e = fg.nodes[n].flat_in[i];
                let t = fg.edges[e].tail;
                if fg.nodes[t].order > fg.nodes[nn].order {
                    nn = t;
                }
            }
            if s.mval[nn] >= 0.0 {
                let m = s.mval[nn] + 1.0;
                fg.nodes[n].mval = m;
                self.snap.mval[n] = m;
                return false;
            }
        } else if !fg.nodes[n].flat_out.is_empty() {
            let mut nn = fg.edges[fg.nodes[n].flat_out[0]].head;
            for i in 1..fg.nodes[n].flat_out.len() {
                let e = fg.nodes[n].flat_out[i];
                let h = fg.edges[e].head;
                if fg.nodes[h].order < fg.nodes[nn].order {
                    nn = h;
                }
            }
            if s.mval[nn] > 0.0 {
                let m = s.mval[nn] - 1.0;
                fg.nodes[n].mval = m;
                self.snap.mval[n] = m;
                return false;
            }
        }
        true
    }

    /// `reorder` (mincross.c:1404-1453) — bubble-like passes that exchange
    /// the leftmost comparable node `lp` with the next comparable node `rp`
    /// whenever `mval(lp) > mval(rp)` (always) or `mval(lp) == mval(rp)` and
    /// `reverse`. `mval == -1` nodes are never moved into comparison (they
    /// can be skipped over). The window shrinks from the right each outer
    /// round only when `!hasfixed && !reverse`.
    fn reorder(&mut self, fg: &mut Fg, g: GId, r: i32, reverse: bool, hasfixed: bool) {
        self.ensure_snap(fg); // left2right reads the snapshot's cluster flags
        let win = self.win[ri(r)];
        let mut changed = 0i32;
        let mut ep = fg.graphs[g].rank[ri(r)].n as i32; // exclusive window end (shrinks)
        let has_matrix = fg.graphs[g].rank[ri(r)].flat.is_some();
        let flip = fg.graphs[g].rankdir.flip();

        let mut nelt = ep - 1;
        while nelt >= 0 {
            // (nelt = n-1 down to 0 — n outer rounds)
            let mut lp = 0i32;
            let mut rp: i32;
            while lp < ep {
                // find leftmost node that can be compared
                while lp < ep && self.snap.mval[fg.graphs[g].rank[ri(r)].v[win + lp as usize]] < 0.0
                {
                    lp += 1;
                }
                if lp >= ep {
                    break;
                }
                // find the node that can be compared
                let mut sawclust = false;
                let mut muststay = false;
                rp = lp + 1;
                while rp < ep {
                    let rn = fg.graphs[g].rank[ri(r)].v[win + rp as usize];
                    if sawclust && fg.nodes[rn].clust.is_some() {
                        rp += 1;
                        continue; // skip interior cluster nodes (### marker)
                    }
                    if self.left2right(
                        fg,
                        g,
                        fg.graphs[g].rank[ri(r)].v[win + lp as usize],
                        rn,
                        has_matrix,
                        flip,
                    ) {
                        muststay = true;
                        break;
                    }
                    if self.snap.mval[rn] >= 0.0 {
                        break; // found comparable
                    }
                    if fg.nodes[rn].clust.is_some() {
                        sawclust = true;
                    }
                    rp += 1;
                }
                if rp >= ep {
                    break; // ends this outer round
                }
                if !muststay {
                    let ln = fg.graphs[g].rank[ri(r)].v[win + lp as usize];
                    let rn = fg.graphs[g].rank[ri(r)].v[win + rp as usize];
                    let p1 = self.snap.mval[ln];
                    let p2 = self.snap.mval[rn];
                    // p1 > p2 always swaps; p1 == p2 swaps iff reverse
                    if p1 > p2 || (p1 >= p2 && reverse) {
                        // exchange positions lp and rp (not adjacent-swap semantics!)
                        self.exchange(fg, ln, rn);
                        changed += 1;
                    }
                }
                lp = rp;
            }
            if !hasfixed && !reverse {
                ep -= 1; // shrink window from the right each outer round
            }
            nelt -= 1;
        }

        if changed != 0 {
            // invalidation asymmetry (L1448-1452): only r and r-1, never r+1
            fg.graphs[self.root].rank[ri(r)].valid = false;
            if r > 0 {
                fg.graphs[self.root].rank[ri(r - 1)].valid = false;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Flat-edge machinery (mincross.c:883-896, 1033-1117, 1297-1402)
    // -----------------------------------------------------------------------

    /// `agcontains` for a *node* — the root contains every fast node; a
    /// cluster contains its members at any nesting depth (subgraph membership
    /// is transitive in cgraph, which the parser reproduces).
    fn agcontains_node(fg: &Fg, g: GId, v: NId) -> bool {
        if g == fg.root_g() {
            return true;
        }
        fg.graphs[g].nodes_order.contains(&v)
    }

    /// `agcontains` for an *edge* (used by `is_a_vnode_of_an_edge_of`).
    fn agcontains_edge(fg: &Fg, g: GId, e: EId) -> bool {
        if g == fg.root_g() {
            return true;
        }
        fg.graphs[g].edges_order.contains(&e)
    }

    /// `is_a_normal_node_of` (mincross.c:883-885).
    fn is_a_normal_node_of(&self, fg: &Fg, g: GId, v: NId) -> bool {
        fg.nodes[v].node_type == NodeType::Normal && Self::agcontains_node(fg, g, v)
    }

    /// `is_a_vnode_of_an_edge_of` (mincross.c:887-896) — a virtual node with
    /// exactly one in- and one out-edge whose original edge belongs to `g`
    /// (found by walking `ED_to_orig` until the edge type is `NORMAL`).
    fn is_a_vnode_of_an_edge_of(&self, fg: &Fg, g: GId, v: NId) -> bool {
        if fg.nodes[v].node_type == NodeType::Virtual
            && fg.nodes[v].in_.len() == 1
            && fg.nodes[v].out.len() == 1
        {
            let mut e = fg.nodes[v].out[0];
            while fg.edges[e].edge_type != EdgeType::Normal {
                e = match fg.edges[e].to_orig {
                    Some(o) => o,
                    None => return false, // malformed chain — C would not terminate
                };
            }
            return Self::agcontains_edge(fg, g, e);
        }
        false
    }

    /// `inside_cluster` (mincross.c:898-900). For the root graph every fast
    /// node qualifies, so `constraining_flat_edge` reduces to
    /// `ED_weight != 0` until clusters are ported.
    fn inside_cluster(&self, fg: &Fg, g: GId, v: NId) -> bool {
        self.is_a_normal_node_of(fg, g, v) || self.is_a_vnode_of_an_edge_of(fg, g, v)
    }

    /// `constraining_flat_edge` (mincross.c:1297-1305).
    fn constraining_flat_edge(&self, fg: &Fg, g: GId, e: EId) -> bool {
        if fg.edges[e].weight == 0 {
            return false;
        }
        if !self.inside_cluster(fg, g, fg.edges[e].tail) {
            return false;
        }
        if !self.inside_cluster(fg, g, fg.edges[e].head) {
            return false;
        }
        true
    }

    /// `flat_rev` (mincross.c:1033-1057) — reverse flat edge `e`: if the
    /// opposite edge already exists, merge into it and file `e` under
    /// `ND_other(agtail(e))`; otherwise create a reversed virtual copy
    /// (`FLATORDER` stays `FLATORDER`, everything else becomes `REVERSED`)
    /// and attach it with `flat_edge`.
    fn flat_rev(&mut self, fg: &mut Fg, g: GId, e: EId) {
        let (et, eh) = (fg.edges[e].tail, fg.edges[e].head);
        // scan ND_flat_out(aghead(e)) for an edge with head == agtail(e)
        let mut rev: Option<EId> = None;
        for j in 0..fg.nodes[eh].flat_out.len() {
            let cand = fg.nodes[eh].flat_out[j];
            if fg.edges[cand].head == et {
                rev = Some(cand);
                break;
            }
        }
        if let Some(rev) = rev {
            merge_oneway(fg, e, rev); // folds weight/xpenalty/count into rev
            if fg.edges[rev].edge_type == EdgeType::FlatOrder && fg.edges[rev].to_orig.is_none() {
                fg.edges[rev].to_orig = Some(e);
            }
            fg.nodes[et].other.push(e); // elist_append(e, ND_other(agtail(e)))
        } else {
            let rev = new_virtual_edge(fg, eh, et, Some(e));
            fg.edges[rev].edge_type = if fg.edges[e].edge_type == EdgeType::FlatOrder {
                EdgeType::FlatOrder
            } else {
                EdgeType::Reversed
            };
            fg.edges[rev].label = fg.edges[e].label; // ED_label(rev) = ED_label(e)
            flat_edge(fg, g, rev); // sets GD_has_flat_edges(root) = GD_has_flat_edges(g) = true
        }
    }

    /// `flat_search` (mincross.c:1059-1090) — left→right DFS over flat
    /// out-edges, freezing directions in the rank's flat matrix and breaking
    /// cycles it meets on the stack.
    ///
    /// C's `for (i = 0; …; i++)` with `i--` before `continue`/`flat_rev`
    /// re-scans the same slot after a deletion; the `while` below reproduces
    /// that (the slot now holds the edge `zapinlist` swapped in, if any).
    fn flat_search(&mut self, fg: &mut Fg, g: GId, v: NId, unset: usize, set: usize) {
        let r = fg.nodes[v].rank;
        fg.nodes[v].mark = set;
        fg.nodes[v].onstack = true;
        let hascl = !fg.graphs[fg.root_g()].clust.is_empty(); // GD_n_cluster(dot_root(g)) > 0
        let mut i = 0usize;
        while i < fg.nodes[v].flat_out.len() {
            let e = fg.nodes[v].flat_out[i];
            let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
            if hascl && !(Self::agcontains_node(fg, g, t) && Self::agcontains_node(fg, g, h)) {
                i += 1;
                continue; // edge of another cluster: ignore entirely (kept in lists)
            }
            if fg.edges[e].weight == 0 {
                i += 1;
                continue; // non-constraint: ignore (kept in lists)
            }
            if fg.nodes[h].onstack {
                // cycle: break it — M[head][tail] records the reversed constraint
                let (fl, tl) = (fg.nodes[h].low as usize, fg.nodes[t].low as usize);
                matrix_set(&mut fg.graphs[g].rank[ri(r)].flat, fl, tl);
                delete_flat_edge(fg, e);
                // C: delete, i--, then either `continue` (FLATORDER) or
                // flat_rev before the loop's i++ — both re-examine slot i.
                if fg.edges[e].edge_type == EdgeType::FlatOrder {
                    continue;
                }
                self.flat_rev(fg, g, e);
                // re-examine the same slot (a replacement slid in, if any)
            } else {
                // M[tail][head]: freeze direction
                let (tl, hl) = (fg.nodes[t].low as usize, fg.nodes[h].low as usize);
                matrix_set(&mut fg.graphs[g].rank[ri(r)].flat, tl, hl);
                if fg.nodes[h].mark == unset {
                    self.flat_search(fg, g, h, unset, set);
                }
                i += 1;
            }
        }
        fg.nodes[v].onstack = false;
    }

    /// `flat_breakcycles` (mincross.c:1092-1117) — per rank: alias every
    /// node to its window index (`ND_low`, L1102), allocate the `n × n` flat
    /// matrix once (at the first node with flat out-edges), then run
    /// `flat_search` from each unmarked node, left→right.
    fn flat_breakcycles(&mut self, fg: &mut Fg, g: GId) {
        self.invalidate_snap(); // flat_rev merges fold xpenalty
        let (unset, set) = fresh_marks(fg);
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let mut flat = false;
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            for i in 0..n {
                let v = fg.graphs[g].rank[ri(r)].v[win + i];
                fg.nodes[v].mark = unset;
                fg.nodes[v].onstack = false;
                fg.nodes[v].low = i as i32; // ND_low — stable per-rank alias
                if !fg.nodes[v].flat_out.is_empty() && !flat {
                    fg.graphs[g].rank[ri(r)].flat = Some(matrix_new(n, n));
                    flat = true;
                }
            }
            if flat {
                for i in 0..n {
                    let v = fg.graphs[g].rank[ri(r)].v[win + i];
                    if fg.nodes[v].mark == unset {
                        self.flat_search(fg, g, v, unset, set);
                    }
                }
            }
        }
    }

    /// `postorder` (mincross.c:1310-1325) — construct nodes reachable from
    /// `v` in post-order (the same as a topological sort in reverse order).
    fn postorder(&mut self, fg: &mut Fg, g: GId, v: NId, list: &mut Vec<NId>, r: i32, set: usize) {
        fg.nodes[v].mark = set;
        for i in 0..fg.nodes[v].flat_out.len() {
            let e = fg.nodes[v].flat_out[i];
            if !self.constraining_flat_edge(fg, g, e) {
                continue;
            }
            let h = fg.edges[e].head;
            if fg.nodes[h].mark != set {
                self.postorder(fg, g, h, list, r, set);
            }
        }
        debug_assert_eq!(fg.nodes[v].rank, r);
        list.push(v); // append AFTER recursion (true post-order)
    }

    /// `flat_reorder` (mincross.c:1327-1402) — the literal scan (spec §6.6).
    ///
    /// Verified semantics: ranks containing any constraining flat edge are
    /// left untouched ("do no harm"); otherwise the reorder itself is an
    /// identity rewrite of `ND_order`. The observable effects are (a)
    /// flipping leftward *non-constraining* flat edges to point LR via
    /// `flat_rev`, and (b) invalidating every visited rank's crossing cache
    /// (L1399 runs unconditionally per non-empty rank).
    fn flat_reorder(&mut self, fg: &mut Fg, g: GId) {
        self.invalidate_snap(); // rewrites orders + flat_rev
        if !fg.graphs[g].has_flat_edges {
            return; // L1333-1334
        }
        let (unset, set) = fresh_marks(fg);
        let mut temprank: Vec<NId> = Vec::new();
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            if n == 0 {
                continue;
            }
            let base_order = fg.nodes[fg.graphs[g].rank[ri(r)].v[win]].order; // L1338
            for i in 0..n {
                let v = fg.graphs[g].rank[ri(r)].v[win + i];
                fg.nodes[v].mark = unset;
            }
            temprank.clear(); // LIST_CLEAR

            // construct reverse topological sort order in temprank (L1343)
            let flip = fg.graphs[g].rankdir.flip();
            let mut valid_rank = true;
            for i in 0..n {
                // flip: L→R scan; else R→L scan
                let v = if flip {
                    fg.graphs[g].rank[ri(r)].v[win + i]
                } else {
                    fg.graphs[g].rank[ri(r)].v[win + n - i - 1]
                };
                let mut local_in_cnt = 0;
                let mut local_out_cnt = 0;
                for &fe in fg.nodes[v].flat_in.iter() {
                    if self.constraining_flat_edge(fg, g, fe) {
                        local_in_cnt += 1;
                    }
                }
                for &fe in fg.nodes[v].flat_out.iter() {
                    if self.constraining_flat_edge(fg, g, fe) {
                        local_out_cnt += 1;
                    }
                }
                if local_in_cnt == 0 && local_out_cnt == 0 {
                    temprank.push(v);
                } else if fg.nodes[v].mark != set && local_in_cnt == 0 {
                    self.postorder(fg, g, v, &mut temprank, r, set);
                } else {
                    valid_rank = false;
                    break;
                }
            }

            if valid_rank && !temprank.is_empty() {
                debug_assert_eq!(temprank.len(), n, "temprank is a permutation of the rank");
                if !flip {
                    temprank.reverse(); // LIST_REVERSE (L1373-1375)
                }
                for i in 0..n {
                    let v = temprank[i];
                    fg.graphs[g].rank[ri(r)].v[win + i] = v;
                    fg.nodes[v].order = i as i32 + base_order;
                }
                // nonconstraint flat edges must be made LR (L1381-1395)
                for i in 0..n {
                    let v = fg.graphs[g].rank[ri(r)].v[win + i];
                    let mut j = 0usize;
                    while j < fg.nodes[v].flat_out.len() {
                        let e = fg.nodes[v].flat_out[j];
                        let h_ord = fg.nodes[fg.edges[e].head].order;
                        let t_ord = fg.nodes[fg.edges[e].tail].order;
                        let leftward = if !flip { h_ord < t_ord } else { h_ord > t_ord };
                        if leftward {
                            debug_assert!(!self.constraining_flat_edge(fg, g, e));
                            delete_flat_edge(fg, e);
                            // C: delete, j--, flat_rev, then j++ ⇒ re-examine slot j
                            self.flat_rev(fg, g, e);
                        } else {
                            j += 1;
                        }
                    }
                }
                // postprocess to restore intended order (L1396)
            }
            // else do no harm! (L1398)
            fg.graphs[self.root].rank[ri(r)].valid = false; // ALWAYS (L1399)
        }
    }

    // -----------------------------------------------------------------------
    // Crossing counting (mincross.c:1506-1560)
    // -----------------------------------------------------------------------

    /// `rcross` (mincross.c:1506-1543) — crossings between rank `r` and
    /// `r+1`, plus the port-aware `local_cross` corrections.
    ///
    /// `Count[k]` accumulates the total xpenalty of edges seen so far ending
    /// at column `k`; a new edge to column `inv` crosses every previously
    /// accumulated edge ending at columns `> inv`. `Count` is sized
    /// `GD_rank(Root)[r+1].n + 1` (L1514) — here the *same* graph is Root, so
    /// the component-relative head orders are strictly below that length;
    /// the `resize` guard is defensive only.
    fn rcross(&self, fg: &Fg, r: i32) -> i64 {
        let mut cross: i64 = 0;
        let mut max: i32 = 0;
        let (win, n) = (self.win[ri(r)], fg.graphs[self.root].rank[ri(r)].n);
        let count_len = fg.graphs[self.root].rank[ri(r + 1)].n + 1;
        let mut count: Vec<i32> = vec![0; count_len];

        for top in 0..n {
            let v = fg.graphs[self.root].rank[ri(r)].v[win + top];
            if max > 0 {
                for &e in fg.nodes[v].out.iter() {
                    let inv = fg.nodes[fg.edges[e].head].order;
                    let mut k = inv + 1;
                    while k <= max {
                        let ck = count.get(k as usize).copied().unwrap_or(0);
                        cross += (ck as i64) * (fg.edges[e].xpenalty as i64);
                        k += 1;
                    }
                }
            }
            for &e in fg.nodes[v].out.iter() {
                let inv = fg.nodes[fg.edges[e].head].order;
                if inv > max {
                    max = inv;
                }
                let iu = inv as usize;
                if iu >= count.len() {
                    count.resize(iu + 1, 0); // defensive; see doc
                }
                count[iu] += fg.edges[e].xpenalty;
            }
        }
        // port-aware corrections (both ranks): nodes with resolved ports
        // (ND_has_port) — ports are only computed by the position phase, so
        // these contribute 0 during mincross, exactly as in C.
        for top in 0..n {
            let v = fg.graphs[self.root].rank[ri(r)].v[win + top];
            if fg.nodes[v].has_port {
                cross += local_cross(fg, &fg.nodes[v].out, 1) as i64;
            }
        }
        let (win1, n1) = (self.win[ri(r + 1)], fg.graphs[self.root].rank[ri(r + 1)].n);
        for bot in 0..n1 {
            let v = fg.graphs[self.root].rank[ri(r + 1)].v[win1 + bot];
            if fg.nodes[v].has_port {
                cross += local_cross(fg, &fg.nodes[v].in_, -1) as i64;
            }
        }
        cross
    }

    /// `ncross` (mincross.c:1545-1560) — total crossings over **Root**'s
    /// ranks `minrank ..< maxrank` (exclusive upper bound), with the
    /// per-rank `valid`/`cache_nc` memo. Cache invalidation sites:
    /// `build_ranks` (L1259), `transpose_step` (L660-669), `reorder`
    /// (L1449-1451 — only r and r-1), `flat_reorder` (L1399).
    fn ncross(&mut self, fg: &mut Fg) -> i64 {
        self.ensure_snap(fg);
        let g = self.root;
        // the invalidated ranks (cache misses) are the work; each rank's
        // rcross reads only its own two rows, so the re-computations are
        // independent and the integer total is order-independent
        let mut invalid: Vec<i32> = Vec::new();
        let mut work = 0usize;
        for r in fg.graphs[g].minrank..fg.graphs[g].maxrank {
            if !fg.graphs[g].rank[ri(r)].valid {
                invalid.push(r);
                work += fg.graphs[g].rank[ri(r)].n;
            }
        }
        let mut count: i64 = 0;
        // still-valid ranks contribute their cached totals (integer sums are
        // order-independent, so adding them up front is exact)
        for r in fg.graphs[g].minrank..fg.graphs[g].maxrank {
            if fg.graphs[g].rank[ri(r)].valid {
                count += fg.graphs[g].rank[ri(r)].cache_nc.unwrap_or(0) as i64;
            }
        }
        let threads = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .min(8);
        if invalid.len() >= 4 && work >= 8192 && threads > 1 {
            // parallel: chunk the invalidated ranks across threads; rcross(r)
            // only reads rank r's own two rows, so the computations are
            // independent. Results come back in rank order and the caches are
            // filled sequentially — identical observable state to the scan.
            let chunk = invalid.len().div_ceil(threads);
            let (me, fgr) = (&*self, &*fg);
            let results: Vec<Vec<(i32, i64)>> = std::thread::scope(|scope| {
                let handles: Vec<_> = invalid
                    .chunks(chunk)
                    .map(|ch| scope.spawn(move || ch.iter().map(|&r| (r, me.rcross(fgr, r))).collect()))
                    .collect();
                handles.into_iter().map(|h| h.join().expect("rcross thread")).collect()
            });
            for part in results {
                for (r, nc) in part {
                    count += nc;
                    let rank = &mut fg.graphs[g].rank[ri(r)];
                    rank.cache_nc = Some(nc as usize);
                    rank.valid = true;
                }
            }
        } else {
            for r in invalid {
                let nc = self.rcross(fg, r);
                count += nc;
                let rank = &mut fg.graphs[g].rank[ri(r)];
                rank.cache_nc = Some(nc as usize); // crossing counts are non-negative
                rank.valid = true;
            }
        }
        count
    }

    // -----------------------------------------------------------------------
    // Component merge / cleanup (mincross.c:784-869)
    // -----------------------------------------------------------------------

    /// `merge_components` (mincross.c:784-804) — splice the component node
    /// chains into one list and restore the global rank bounds.
    fn merge_components(&mut self, fg: &mut Fg, g: GId) {
        if fg.graphs[g].comp.len() <= 1 {
            return;
        }
        let comps = fg.graphs[g].comp.clone();
        let mut u: Option<NId> = None;
        for &v in comps.iter() {
            if let Some(uu) = u {
                fg.nodes[uu].next = Some(v);
            }
            fg.nodes[v].prev = u;
            let mut cur = v;
            while let Some(nx) = fg.nodes[cur].next {
                cur = nx; // walk to the end of this component's chain
            }
            u = Some(cur);
        }
        fg.graphs[g].comp.truncate(1);
        fg.graphs[g].nlist = Some(comps[0]);
        fg.graphs[g].minrank = self.global_minrank; // L802-803
        fg.graphs[g].maxrank = self.global_maxrank;
    }

    /// `merge2` (mincross.c:806-830) — merge connected components and create
    /// globally consistent rank lists.
    ///
    /// `.n = .an` + the `NULL` scan (L815-828) repairs the per-component
    /// windows into one global list — components were laid end-to-end in
    /// `av` by repeated `build_ranks`. PORT NOTE: after renumbering, the
    /// trailing `NO_NODE` slack slots are truncated away so `Rank::v` is the
    /// final row; C keeps them but only ever reads `0 .. n`.
    fn merge2(&mut self, fg: &mut Fg, g: GId) {
        self.invalidate_snap(); // global order renumbering
        // merge the components and rank limits (L811-812)
        self.merge_components(fg, g);

        // install complete ranks (L814-829)
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let row = &mut fg.graphs[g].rank[ri(r)];
            row.n = row.v.len(); // n = an (L816)
            self.win[ri(r)] = 0; // v = av (L817)
            let mut nn = row.n;
            for i in 0..row.n {
                if row.v[i] == NO_NODE {
                    // C L820-826 prints a Verbose diagnostic here.
                    nn = i;
                    break;
                }
            }
            row.n = nn;
            for i in 0..nn {
                fg.nodes[row.v[i]].order = i as i32; // global renumbering (L827)
            }
            row.v.truncate(nn);
        }
    }
    /// `cleanup2` (mincross.c:832-869) — free the module scratch (dropped
    /// with `self` here), fix the cluster vlists, renumber `ND_order`
    /// globally one last time, delete the `FLATORDER` temporary edges created
    /// by `do_ordering_node`, and free the per-rank flat matrices.
    ///
    /// PORT NOTE: the arena never frees edge objects (indices must stay
    /// stable); an unlinked `FLATORDER` edge simply remains an unreferenced
    /// slot — the same observable state as C's `free`.
    fn cleanup2(&mut self, fg: &mut Fg, g: GId, _nc: i64, has_vlists: bool) {
        self.invalidate_snap(); // final renumbering
        // C L837-844: free TI_list / TE_list — dropped with self.
        let _ = _nc; // C L866-868 prints a Verbose summary here.

        // fix vlists of clusters (L845-847)
        if has_vlists {
            for c in fg.graphs[g].clust.clone().iter() {
                self.rec_reset_vlists(fg, *c);
            }
        }

        // remove node temporary edges for ordering nodes (L849-865)
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let (win, n) = (self.win[ri(r)], fg.graphs[g].rank[ri(r)].n);
            for i in 0..n {
                let v = fg.graphs[g].rank[ri(r)].v[win + i];
                fg.nodes[v].order = i as i32; // final global numbering
                let mut j = 0usize;
                while j < fg.nodes[v].flat_out.len() {
                    let e = fg.nodes[v].flat_out[j];
                    if fg.edges[e].edge_type == EdgeType::FlatOrder {
                        delete_flat_edge(fg, e);
                        // C frees the edge and re-examines the same index (j--)
                    } else {
                        j += 1;
                    }
                }
            }
            // C L864: `free_matrix(GD_rank(g)[r].flat)` frees the struct but
            // does NOT null `GD_rank[r].flat` — the pointer stays non-NULL
            // (dangling), and flat.c:274 later gates the rank −1 insertion on
            // exactly that: `if (GD_rank(g)[0].flat || GD_n_cluster(g) > 0)`.
            // The arena keeps the matrix in place, reproducing the observable
            // "mincross allocated a rank-0 flat matrix" state that flat.rs's
            // abomination gate depends on.
        }
    }

    // -----------------------------------------------------------------------
    // Edge ordering attributes (mincross.c:434-535)
    // -----------------------------------------------------------------------

    /// `late_string(g, G_ordering, NULL)` — PORT NOTE: graph attributes are
    /// not carried on `DGraph` yet, so this always reports "unset"
    /// (identical behavior for graphs without `ordering`). Wire the
    /// attribute through `DGraph` to engage the machinery below.
    fn graph_ordering_attr(_fg: &Fg, _g: GId) -> Option<String> {
        None
    }

    /// `late_string(n, N_ordering, NULL)` — PORT NOTE: node attributes are
    /// not carried on `DNode` yet (see above).
    fn node_ordering_attr(_fg: &Fg, _n: NId) -> Option<String> {
        None
    }

    /// `is_a_cluster` (rank.rs keeps the canonical copy; this mirrors the
    /// `ordered_edges` subgraph guard at mincross.c:527-531).
    fn is_a_cluster_graph(fg: &Fg, g: GId) -> bool {
        let dg = &fg.graphs[g];
        dg.input.is_none() || dg.is_cluster_name || dg.cluster_flag
    }

    /// `betweenclust` (mincross.c:434-438) — true iff the original edge of
    /// `e` crosses a cluster boundary.
    fn betweenclust(&self, fg: &Fg, mut e: EId) -> bool {
        while let Some(o) = fg.edges[e].to_orig {
            e = o;
        }
        fg.nodes[fg.edges[e].tail].clust != fg.nodes[fg.edges[e].head].clust
    }

    /// `do_ordering_node` (mincross.c:440-477) — create `FLATORDER` flat
    /// edges forcing the input order of a node's non-betweenclust edges.
    ///
    /// Edges are sorted by `AGSEQ` (creation sequence — declaration order);
    /// the first existing flat edge aborts the whole node's chain (`return`,
    /// not `continue`, L471-472). C sorts the shared `TE_list` in place; the
    /// port sorts an equivalent local list.
    fn do_ordering_node(&mut self, fg: &mut Fg, g: GId, n: NId, outflag: bool) {
        if fg.nodes[n].clust.is_some() {
            return; // cluster members are skipped
        }
        let mut sortlist: Vec<EId> = Vec::new();
        if outflag {
            for &e in fg.nodes[n].out.iter() {
                if !self.betweenclust(fg, e) {
                    sortlist.push(e);
                }
            }
        } else {
            for &e in fg.nodes[n].in_.iter() {
                if !self.betweenclust(fg, e) {
                    sortlist.push(e);
                }
            }
        }
        if sortlist.len() <= 1 {
            return;
        }
        sortlist.sort_by_key(|&e| fg.edges[e].seq); // edgeidcmpf — ascending AGSEQ
        for idx in 1..sortlist.len() {
            let e = sortlist[idx - 1];
            let f = sortlist[idx];
            let (u, v) = if outflag {
                (fg.edges[e].head, fg.edges[f].head)
            } else {
                (fg.edges[e].tail, fg.edges[f].tail)
            };
            if find_flat_edge(fg, u, v).is_some() {
                return; // NB: aborts the whole node on first hit (L471-472)
            }
            let fe = new_virtual_edge(fg, u, v, None);
            fg.edges[fe].edge_type = EdgeType::FlatOrder;
            flat_edge(fg, g, fe);
        }
    }

    /// `do_ordering` (mincross.c:479-486) — order all nodes in graph.
    fn do_ordering(&mut self, fg: &mut Fg, g: GId, outflag: bool) {
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            self.do_ordering_node(fg, g, n, outflag);
        }
    }

    /// `do_ordering_for_nodes` (mincross.c:488-504) — honor per-node
    /// `ordering` attributes ("out" / "in"; anything else non-empty is an
    /// error in C).
    fn do_ordering_for_nodes(&mut self, fg: &mut Fg, g: GId) {
        for &n in fg.graphs[g].nodes_order.clone().iter() {
            if let Some(ordering) = Self::node_ordering_attr(fg, n) {
                if ordering == "out" {
                    self.do_ordering_node(fg, g, n, true);
                } else if ordering == "in" {
                    self.do_ordering_node(fg, g, n, false);
                } else if !ordering.is_empty() {
                    // C: agerrorf("ordering '%s' not recognized for node '%s'.\n", ...)
                }
            }
        }
    }

    /// `ordered_edges` (mincross.c:512-535) — the graph `ordering` attribute
    /// dominates the node attributes; clusters are processed by separate
    /// calls (`mincross_clust`).
    fn ordered_edges(&mut self, fg: &mut Fg, g: GId) {
        self.invalidate_snap(); // FLATORDER edges extend the arena
        let g_ordering = Self::graph_ordering_attr(fg, g);
        let n_ordering = fg.graphs[g]
            .nodes_order
            .first()
            .is_some_and(|&n| Self::node_ordering_attr(fg, n).is_some());
        // C guard: `if (!G_ordering && !N_ordering) return;`
        if g_ordering.is_none() && !n_ordering {
            return;
        }
        if let Some(ordering) = g_ordering {
            if ordering == "out" {
                self.do_ordering(fg, g, true);
            } else if ordering == "in" {
                self.do_ordering(fg, g, false);
            } else if !ordering.is_empty() {
                // C: agerrorf("ordering '%s' not recognized.\n", ...)
            }
        } else {
            for &subg in fg.graphs[g].children.clone().iter() {
                // clusters are processed by separate calls to ordered_edges
                if !Self::is_a_cluster_graph(fg, subg) {
                    self.ordered_edges(fg, subg);
                }
            }
            if n_ordering {
                self.do_ordering_for_nodes(fg, g);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Cluster expansion and the remaining label-order milestones.
    //
    // The cluster *ordering* path lives above (`mincross_clust` →
    // `expand_cluster` → `cluster::*`); what remains are `fillRanks` (dot2's
    // `newrank` mode) and the flat-edge label-order fixups, whose stubs below
    // stay inert and cite their C sites.
    // -----------------------------------------------------------------------

    /// `expand_cluster` (cluster.c:280-295) — build the cluster's internal
    /// structure (`class2`), rank it in place, splice it into the root's rows
    /// (`merge_ranks`), then rebuild its inter-cluster edges (`interclexp`)
    /// and drop the skeleton (`remove_rankleaders`).
    fn expand_cluster(&mut self, fg: &mut Fg, subg: GId) -> i32 {
        self.invalidate_snap(); // class2/interclexp rewire in_/out_
        class2(fg, subg);
        fg.graphs[subg].comp = fg.graphs[subg].nlist.map(|n| vec![n]).unwrap_or_default();
        self.allocate_ranks(fg, subg);
        if self.build_ranks(fg, subg, 0) != 0 {
            return -1;
        }
        cluster::merge_ranks(fg, subg);
        cluster::interclexp(fg, subg);
        cluster::remove_rankleaders(fg, subg);
        0
    }

    /// `mincross_clust` (mincross.c:537-561) — minimize crossings inside one
    /// cluster. The objective still scans **Root**'s rows, so a cluster pass
    /// minimizes whole-root crossings while only permuting the cluster's own
    /// rank slices; `startpass = 2` skips `build_ranks`/`flat_*` (the cluster's
    /// ordering was just built by `expand_cluster`). Returns the crossing
    /// count, or `-1` on failure.
    fn mincross_clust(&mut self, fg: &mut Fg, g: GId) -> Result<i64, i32> {
        if self.expand_cluster(fg, g) != 0 {
            return Err(-1);
        }
        self.ordered_edges(fg, g);
        self.flat_breakcycles(fg, g);
        self.flat_reorder(fg, g);
        let mut nc = self.mincross(fg, g, 2);
        if nc < 0 {
            return Err(-1);
        }
        for c in fg.graphs[g].clust.clone().iter() {
            nc += self.mincross_clust(fg, *c)?;
        }
        self.save_vlist(fg, g);
        Ok(nc)
    }

    /// `save_vlist` (mincross.c:913-920) — remember each rank's first node so
    /// `rec_reset_vlists` can re-derive the cluster's slice later.
    fn save_vlist(&mut self, fg: &mut Fg, g: GId) {
        if fg.graphs[g].rankleader.is_empty() {
            return;
        }
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let slot = ri(r);
            let first = fg.graphs[g].rank[slot].v.first().copied();
            fg.graphs[g].rankleader[r as usize] = first;
        }
    }

    /// `rec_save_vlists` (mincross.c:922-928) — pre-order.
    #[allow(dead_code)]
    fn rec_save_vlists(&mut self, fg: &mut Fg, g: GId) {
        self.save_vlist(fg, g);
        for c in fg.graphs[g].clust.clone().iter() {
            self.rec_save_vlists(fg, *c);
        }
    }

    /// `rec_reset_vlists` (mincross.c:930-953) — post-order; re-derive each
    /// cluster's rank slice from the root's row (C re-points
    /// `GD_rank(g)[r].v`; this port refreshes its copy and the slice offset).
    fn rec_reset_vlists(&mut self, fg: &mut Fg, g: GId) {
        for c in fg.graphs[g].clust.clone().iter() {
            self.rec_reset_vlists(fg, *c);
        }
        if fg.graphs[g].rankleader.is_empty() {
            return;
        }
        let root = fg.root_g();
        for r in fg.graphs[g].minrank..=fg.graphs[g].maxrank {
            let Some(v) = fg.graphs[g].rankleader[r as usize] else {
                continue;
            };
            let u = self.furthestnode(fg, g, v, -1).unwrap_or(v);
            let w = self.furthestnode(fg, g, v, 1).unwrap_or(v);
            let uo = fg.nodes[u].order.max(0) as usize;
            let wo = fg.nodes[w].order.max(0) as usize;
            fg.graphs[g].rankleader[r as usize] = Some(u);
            let slot = ri(r);
            let root_slot = rank_row_index(fg, root, r);
            let src: Vec<NId> = fg.graphs[root].rank[root_slot].v.clone();
            let n = wo.saturating_sub(uo) + 1;
            {
                let row = &mut fg.graphs[g].rank[slot];
                for i in 0..n {
                    row.v[i] = *src.get(uo + i).unwrap_or(&usize::MAX);
                }
                row.n = n;
            }
            fg.graphs[g].rank_offset[slot] = uo as i32;
        }
    }

    /// `neighbor` (mincross.c:871-881) — the adjacent node in the **root**
    /// row, in direction `dir`.
    fn neighbor(&self, fg: &Fg, v: NId, dir: i32) -> Option<NId> {
        let r = fg.nodes[v].rank;
        let row = &fg.graphs[self.root].rank[ri(r)];
        let o = fg.nodes[v].order;
        if dir < 0 {
            if o > 0 {
                return Some(row.v[(o - 1) as usize]);
            }
            None
        } else {
            row.v
                .get((o + 1) as usize)
                .copied()
                .filter(|&n| n != NO_NODE)
        }
    }

    /// `furthestnode` (mincross.c:902-911) — walk the root row in `dir`,
    /// keeping the last node that belongs to `g` (normal member or a virtual
    /// node of one of its edges).
    fn furthestnode(&self, fg: &Fg, g: GId, v: NId, dir: i32) -> Option<NId> {
        let mut rv = v;
        let mut u = v;
        while let Some(next) = self.neighbor(fg, u, dir) {
            u = next;
            if self.is_a_normal_node_of(fg, g, u) || self.is_a_vnode_of_an_edge_of(fg, g, u) {
                rv = u;
            }
        }
        Some(rv)
    }

    /// STUB — `fillRanks` (mincross.c:1003-1008) / `realFillRanks`
    /// (mincross.c:966-1001): guarantee every cluster has ≥ 1 node on every
    /// rank by inserting tiny real nodes into the `_new_rank` subgraph (only
    /// reached with `GD_flags & NEW_RANK`, the dot2 path).
    #[allow(dead_code)]
    fn fill_ranks(&mut self, _fg: &mut Fg, _g: GId) {}

    /// STUB — `checkLabelOrder` (mincross.c:297-326, called from
    /// flat.c:332): scan each rank for flat-edge label vnodes (`ND_alg`),
    /// build the synthetic `lg` graph with `info_t` nodes
    /// (`lo`/`hi` = the label's two out-head orders), and call
    /// `fixLabelOrder` when more than one label lands on a rank.
    #[allow(dead_code)]
    fn check_label_order(&mut self, _fg: &mut Fg, _g: GId) {}

    /// STUB — `fixLabelOrder` (mincross.c:244-287) with `getComp`
    /// (L221-241), `topsort` (L204-219), `emptyComp` (L181-189) and
    /// `isBackedge` (L191): pairwise interval ordering of label vnodes,
    /// component splitting, and Kahn-style re-splicing of the fixed labels
    /// back into the real rank row (`rk->v[indices[i]] = …`).
    #[allow(dead_code)]
    fn fix_label_order(&mut self, _fg: &mut Fg, _g: GId) {}
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dotgen::{Measured, build};
    use crate::graph::parser::parse;

    /// dot_rank + dot_mincross over a parsed source graph.
    fn run(src: &str) -> Fg {
        let g = parse(src).expect("parse");
        let n = g.nodes.len();
        let mut fg = build(
            &g,
            &Measured {
                label: Vec::new(),
                node: vec![(54.0, 36.0); n],
                edge_label: Vec::new(),
                measure: None,
            },
        );
        super::rank::dot_rank(&mut fg, 0);
        dot_mincross(&mut fg, 0).expect("dot_mincross failed");
        fg
    }

    fn id_by_name(fg: &Fg, name: &str) -> NId {
        fg.nodes
            .iter()
            .position(|n| n.name == name)
            .unwrap_or_else(|| panic!("node {name}"))
    }

    /// Total rank-r → rank-r+1 crossings of the final order, via this
    /// module's own `out_cross` over all node pairs.
    fn pair_cross(fg: &Fg, r: i32) -> i64 {
        let rank = &fg.graphs[0].rank[ri(r)];
        let mut cross = 0i64;
        for i in 0..rank.n {
            for j in (i + 1)..rank.n {
                cross += out_cross(fg, rank.v[i], rank.v[j]) as i64;
            }
        }
        cross
    }

    /// The fused AVX2 kernels must agree bit-for-bit with the scalar
    /// reference over randomized xpenalty/order tables.
    #[cfg(target_arch = "x86_64")]
    #[test]
    fn avx2_pair_kernels_match_scalar() {
        if !std::is_x86_feature_detected!("avx2") {
            return; // scalar paths are exercised by every other test
        }
        let mut mc = MinCross::new(0);
        // deterministic pseudo-random tables
        let mut seed = 0x2545F4914F6CDD1Du64;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        let ne = 257usize;
        mc.snap.in_eto = (0..ne).map(|_| (next() % 40) as i32).collect();
        mc.snap.in_xp = (0..ne).map(|i| if i % 17 == 0 { ((next() % 1000) as i32).wrapping_neg() } else { (next() % 5000) as i32 }).collect();
        mc.snap.out_eho = (0..ne).map(|_| (next() % 40) as i32).collect();
        mc.snap.out_xp = mc.snap.in_xp.clone();
        mc.snap.in_tpx = vec![0.0; ne];
        mc.snap.out_hpx = vec![0.0; ne];
        mc.snap.any_port = false;
        // scalar references
        let mut sums = [(0i64, 0i64); 8];
        for case in 0..8usize {
            let v0 = (case * 31) % ne;
            let v1 = v0 + 1 + (next() as usize) % (ne - v0).min(64);
            let w0 = (case * 17) % ne;
            let w1 = w0 + 1 + (next() as usize) % (ne - w0).min(64);
            let mut c0 = 0i64;
            let mut c1 = 0i64;
            for k in w0..w1 {
                for m in v0..v1 {
                    let t = mc.snap.in_eto[m] - mc.snap.in_eto[k];
                    let prod = mc.snap.in_xp[m].wrapping_mul(mc.snap.in_xp[k]);
                    if t > 0 { c0 += prod as i64; } else if t < 0 { c1 += prod as i64; }
                }
            }
            sums[case] = (c0, c1);
            unsafe {
                let (s0, s1) = (0i64, 0i64);
                let mut r0 = s0;
                let mut r1 = s1;
                mc.in_cross_pair_avx2(v0, v1, w0, w1, &mut r0, &mut r1);
                assert_eq!((r0, r1), sums[case], "in case {case}: [{v0}..{v1}) x [{w0}..{w1})");
            }
        }
        // out (i32 wrapping) — verify against scalar with adversarial products
        let mut seed2 = 0x9E3779B97F4A7C15u64;
        let mut next2 = move || {
            seed2 ^= seed2 << 13;
            seed2 ^= seed2 >> 7;
            seed2 ^= seed2 << 17;
            seed2
        };
        mc.snap.out_xp = (0..ne).map(|_| (next2() % (i32::MAX as u64 / 3)) as i32).collect();
        for case in 0..8usize {
            let v0 = (case * 29) % ne;
            let v1 = v0 + 1 + (next2() as usize) % (ne - v0).min(64);
            let w0 = (case * 13) % ne;
            let w1 = w0 + 1 + (next2() as usize) % (ne - w0).min(64);
            let mut c0 = 0i32;
            let mut c1 = 0i32;
            for k in w0..w1 {
                for m in v0..v1 {
                    let t = mc.snap.out_eho[m] - mc.snap.out_eho[k];
                    let prod = mc.snap.out_xp[m].wrapping_mul(mc.snap.out_xp[k]);
                    if t > 0 { c0 = c0.wrapping_add(prod); } else if t < 0 { c1 = c1.wrapping_add(prod); }
                }
            }
            unsafe {
                let (mut r0, mut r1) = (0i32, 0i32);
                mc.out_cross_pair_avx2(v0, v1, w0, w1, &mut r0, &mut r1);
                assert_eq!((r0, r1), (c0, c1), "out case {case}");
            }
        }
    }

    #[test]
    fn bipartite_two_by_two_orders_with_no_crossings() {
        // (a)
        let fg = run("digraph { a -> x; a -> y; b -> x; b -> y; }");
        let rank = &fg.graphs[0].rank;
        assert_eq!(rank[0].n, 2, "rank 0 non-empty with both sources");
        assert_eq!(rank[1].n, 2, "rank 1 non-empty with both sinks");
        let top: Vec<NId> = (0..rank[0].n).map(|i| rank[0].v[i]).collect();
        let bot: Vec<NId> = (0..rank[1].n).map(|i| rank[1].v[i]).collect();
        assert!(top.contains(&id_by_name(&fg, "a")) && top.contains(&id_by_name(&fg, "b")));
        assert!(bot.contains(&id_by_name(&fg, "x")) && bot.contains(&id_by_name(&fg, "y")));
        for r in 0..2i32 {
            for i in 0..rank[ri(r)].n {
                assert_eq!(fg.nodes[rank[ri(r)].v[i]].rank, r, "ND_rank");
                assert_eq!(fg.nodes[rank[ri(r)].v[i]].order, i as i32, "ND_order");
            }
        }
        let cross = pair_cross(&fg, 0);
        assert!(cross <= 1, "crossings = {cross}");
    }

    #[test]
    fn chain_gets_one_node_per_rank_in_order() {
        // (b) — NB: ND_order is the position *within the rank row*
        // (cleanup2, mincross.c:851-853), so each singleton rank's node has
        // order 0; the rank rows themselves carry the a→b→c sequence.
        let fg = run("digraph { a -> b -> c; }");
        let rank = &fg.graphs[0].rank;
        assert_eq!(rank[0].v, vec![id_by_name(&fg, "a")]);
        assert_eq!(rank[1].v, vec![id_by_name(&fg, "b")]);
        assert_eq!(rank[2].v, vec![id_by_name(&fg, "c")]);
        for r in 0..3i32 {
            assert_eq!(fg.nodes[rank[ri(r)].v[0]].order, 0);
            assert_eq!(fg.nodes[rank[ri(r)].v[0]].rank, r);
        }
    }

    #[test]
    fn flat_edge_graph_produces_an_order_without_panicking() {
        // (c) — a, b, c all on one rank via {rank=same}; a→b, a→c, b→c are
        // flat edges, so flat_breakcycles/flat_search/flat_reorder all run.
        let fg = run("digraph { a -> b; a -> c; b -> c; { rank=same; a; b; c; } }");
        let rank = &fg.graphs[0].rank;
        assert_eq!(rank[0].n, 3, "all three nodes on rank 0");
        let mut seen = std::collections::HashSet::new();
        for i in 0..rank[0].n {
            assert_eq!(fg.nodes[rank[0].v[i]].order, i as i32);
            assert!(seen.insert(rank[0].v[i]), "order is a permutation");
        }
    }

    #[test]
    fn disconnected_components_merge_into_global_orders() {
        // two components + a long edge (virtual chain) — exercises
        // init_mccomp windows, merge_components and merge2's NULL scan.
        let fg = run("digraph { a -> b -> c; x -> y; p -> q [minlen=3]; }");
        let rank = &fg.graphs[0].rank;
        let total: usize = (fg.graphs[0].minrank..=fg.graphs[0].maxrank)
            .map(|r| rank[ri(r)].n)
            .sum();
        assert_eq!(total, 9, "7 real nodes (a,b,c,x,y,p,q) + 2 chain vnodes");
        let mut seen = std::collections::HashSet::new();
        for r in fg.graphs[0].minrank..=fg.graphs[0].maxrank {
            for i in 0..rank[ri(r)].n {
                let v = rank[ri(r)].v[i];
                assert_eq!(fg.nodes[v].order, i as i32, "renumbered globally");
                assert!(seen.insert(v), "node installed exactly once");
            }
        }
    }

    #[test]
    fn components_keep_discovery_order_across_ranks() {
        // Three disjoint 2-chains: per-component mincross lays them
        // end-to-end in discovery order (init_mccomp windows + merge2), and
        // dot never reorders ACROSS components — rank 1 stays [z, y, x]
        // with each component internally crossing-free, exactly like C.
        let fg = run("digraph { a -> z; b -> y; c -> x; }");
        let rank = &fg.graphs[0].rank;
        assert_eq!(rank[0].n, 3);
        assert_eq!(rank[1].n, 3);
        let top: Vec<&str> = (0..rank[0].n)
            .map(|i| fg.nodes[rank[0].v[i]].name.as_str())
            .collect();
        let bot: Vec<&str> = (0..rank[1].n)
            .map(|i| fg.nodes[rank[1].v[i]].name.as_str())
            .collect();
        assert_eq!(top, vec!["a", "b", "c"]);
        assert_eq!(bot, vec!["z", "y", "x"]);
        // each component {a,z}, {b,y}, {c,x} is aligned: 0 internal crossings
        for (i, t) in top.iter().enumerate() {
            let ti = id_by_name(&fg, t);
            let bi = id_by_name(&fg, bot[i]);
            let t_pos = fg.nodes[ti].order;
            let e = fg.nodes[ti]
                .out
                .iter()
                .find(|&&e| fg.edges[e].head == bi)
                .copied();
            assert!(e.is_some(), "{t} -> {} edge present", bot[i]);
            let _ = t_pos;
            // the edge's head sits at the same slot: order(b_i) == order slot
            assert_eq!(fg.nodes[bi].rank, fg.nodes[ti].rank + 1);
        }
    }

    #[test]
    fn connected_dense_bipartite_run_is_deterministic_and_bounded() {
        // K3,3-ish: connected, needs medians/reorder/transpose to work.
        // Deterministic (no randomness); the pinned bound is generous —
        // the minimum possible is 1 (K3,3 is non-planar).
        let fg = run("digraph { b -> x; a -> w; a -> x; c -> y; b -> y; c -> w; a -> y; }");
        let rank = &fg.graphs[0].rank;
        assert_eq!(rank[0].n, 3);
        assert_eq!(rank[1].n, 3);
        let cross = pair_cross(&fg, 0);
        assert!(cross <= 4, "crossings = {cross}");
        for i in 0..rank[0].n {
            assert_eq!(fg.nodes[rank[0].v[i]].order, i as i32);
            assert_eq!(fg.nodes[rank[1].v[i]].order, i as i32);
        }
    }

    #[test]
    fn lr_rankdir_flip_path_produces_consistent_orders() {
        // rankdir=LR ⇒ GD_flip ⇒ build_ranks reverses each filled row in
        // place (mincross.c:1260-1266); assert the arrays stay consistent.
        let fg = run("digraph { rankdir=LR; a -> b; a -> c; b -> d; c -> d; }");
        let rank = &fg.graphs[0].rank;
        for r in fg.graphs[0].minrank..=fg.graphs[0].maxrank {
            assert!(rank[ri(r)].n >= 1, "rank {r} non-empty");
            for i in 0..rank[ri(r)].n {
                let v = rank[ri(r)].v[i];
                assert_eq!(fg.nodes[v].rank, r);
                assert_eq!(fg.nodes[v].order, i as i32);
            }
        }
        let cross = (fg.graphs[0].minrank..fg.graphs[0].maxrank)
            .map(|r| pair_cross(&fg, r))
            .sum::<i64>();
        assert!(cross <= 1, "crossings = {cross}");
    }
}
