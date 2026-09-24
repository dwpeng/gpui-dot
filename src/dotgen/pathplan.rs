//! A faithful port of `lib/pathplan` — the visibility-free subset dot uses:
//! `Pshortestpath` (shortest.c: funnel algorithm over an ear-clipped
//! triangulation), `Proutespline` (route.c: Hermite least-squares spline
//! fitting through the shortest path, avoiding barrier edges), the cubic
//! solvers (solvers.c) and `make_polyline` (util.c).
//!
//! The `ccw` helper reproduces triang.c's exact *behavior* (its `ISCCW`/`ISCW`
//! names are swapped relative to mathematical convention — documented in
//! docs/graphviz-specs/splines-fit.md §4.1; do not "fix" it).

use super::geom::PointF;

pub const EPSILON1: f64 = 1e-3;
pub const EPSILON2: f64 = 1e-6;
const EPS: f64 = 1e-7;
const SIZE_MAX: usize = usize::MAX;

// ccw() constants (tri.h) — names are the C ones, swapped vs. convention.
const ISCCW: i32 = 1;
const ISCW: i32 = 2;
const ISON: i32 = 3;

/// triang.c `ccw` — sign of the cross product (p2-p1)×(p3-p2), with the C
/// enum mapping (d>0 → ISCW).
fn ccw(p1: PointF, p2: PointF, p3: PointF) -> i32 {
    let d = (p1.y - p2.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p1.x - p2.x);
    if d > 0.0 {
        ISCW
    } else if d < 0.0 {
        ISCCW
    } else {
        ISON
    }
}

fn dist(a: PointF, b: PointF) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn dist2(a: PointF, b: PointF) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

fn add(a: PointF, b: PointF) -> PointF {
    PointF::new(a.x + b.x, a.y + b.y)
}

fn sub(a: PointF, b: PointF) -> PointF {
    PointF::new(a.x - b.x, a.y - b.y)
}

fn scale(f: f64, a: PointF) -> PointF {
    PointF::new(a.x * f, a.y * f)
}

fn dot(a: PointF, b: PointF) -> f64 {
    a.x * b.x + a.y * b.y
}

// ---------------------------------------------------------------------------
// solvers.c
// ---------------------------------------------------------------------------

fn aeq0(x: f64) -> bool {
    x < EPS && x > -EPS
}

/// solvers.c `solve1`.
fn solve1(coeff: &[f64; 4], roots: &mut [f64]) -> usize {
    let a = coeff[1];
    let b = coeff[0];
    if aeq0(a) {
        if aeq0(b) {
            return 4;
        }
        return 0;
    }
    roots[0] = -b / a;
    1
}

/// solvers.c `solve2`.
fn solve2(coeff: &[f64; 4], roots: &mut [f64]) -> usize {
    let a = coeff[2];
    let b = coeff[1];
    let c = coeff[0];
    if aeq0(a) {
        return solve1(coeff, roots);
    }
    let disc = (b / (2.0 * a)) * (b / (2.0 * a)) - c / a;
    if disc < 0.0 {
        0
    } else if disc > 0.0 {
        let t = -b / (2.0 * a);
        roots[0] = t + disc.sqrt();
        roots[1] = -2.0 * t - roots[0];
        2
    } else {
        roots[0] = -b / (2.0 * a);
        1
    }
}

/// solvers.c `solve3` — cubic root solver; 4 means "identically zero".
fn solve3(coeff: &[f64; 4], roots: &mut [f64; 4]) -> usize {
    let a = coeff[3];
    let b = coeff[2];
    let c = coeff[1];
    let d = coeff[0];
    if aeq0(a) {
        return solve2(coeff, roots);
    }
    let b3 = b / (3.0 * a);
    let ca = c / a;
    let da = d / a;
    let mut p = b3 * b3;
    let q = 2.0 * b3 * p - b3 * ca + da;
    p = ca / 3.0 - p;
    let disc = q * q + 4.0 * p * p * p;
    let rootn;
    if disc < 0.0 {
        let r = 0.5 * (-disc + q * q).sqrt();
        let theta = (-disc).sqrt().atan2(-q);
        let temp = 2.0 * r.cbrt();
        roots[0] = temp * (theta / 3.0).cos();
        roots[1] = temp * ((theta + std::f64::consts::TAU) / 3.0).cos();
        roots[2] = temp * ((theta - std::f64::consts::TAU) / 3.0).cos();
        rootn = 3;
    } else {
        let alpha = 0.5 * (disc.sqrt() - q);
        let beta = -q - alpha;
        roots[0] = alpha.cbrt() + beta.cbrt();
        if disc > 0.0 {
            rootn = 1;
        } else {
            roots[1] = -0.5 * roots[0];
            roots[2] = roots[1];
            rootn = 3;
        }
    }
    for i in 0..rootn {
        roots[i] -= b3;
    }
    rootn
}

/// route.c `points2coeff` — Bézier ordinates to power basis.
fn points2coeff(v0: f64, v1: f64, v2: f64, v3: f64, coeff: &mut [f64; 4]) {
    coeff[3] = v3 + 3.0 * v1 - (v0 + 3.0 * v2);
    coeff[2] = 3.0 * v0 + 3.0 * v2 - 6.0 * v1;
    coeff[1] = 3.0 * (v1 - v0);
    coeff[0] = v0;
}

fn addroot(root: f64, roots: &mut [f64; 4], rootn: &mut usize) {
    if root >= 0.0 && root <= 1.0 {
        roots[*rootn] = root;
        *rootn += 1;
    }
}

// ---------------------------------------------------------------------------
// shortest.c — funnel shortest path in a polygon
// ---------------------------------------------------------------------------

/// A point node with an aliasing `link` (the funnel's parent chain). C uses
/// pointers so the same node may re-enter the deque and overwrite `link`;
/// `Cell` models that aliasing.
struct Pnl {
    pp: PointF,
    link: std::cell::Cell<usize>,
}

struct Tri {
    /// The three edges as (pnl0, pnl1) node pairs.
    e: [(usize, usize); 3],
    right_index: [std::cell::Cell<usize>; 3],
    mark: std::cell::Cell<u8>,
}

struct Deque {
    pnlps: Vec<usize>, // indices into the Pnl arena
    fpnlpi: usize,
    lpnlpi: usize,
    apex: usize,
}

const DQ_FRONT: usize = 1;
const DQ_BACK: usize = 2;

fn add2dq(dq: &mut Deque, side: usize, pnlp: usize, pnls: &[Pnl]) {
    if side == DQ_FRONT {
        if dq.lpnlpi >= dq.fpnlpi {
            pnls[pnlp].link.set(dq.pnlps[dq.fpnlpi]);
        }
        dq.fpnlpi -= 1;
        dq.pnlps[dq.fpnlpi] = pnlp;
    } else {
        if dq.lpnlpi >= dq.fpnlpi {
            pnls[pnlp].link.set(dq.pnlps[dq.lpnlpi]);
        }
        dq.lpnlpi += 1;
        dq.pnlps[dq.lpnlpi] = pnlp;
    }
}

fn splitdq(dq: &mut Deque, side: usize, index: usize) {
    if side == DQ_FRONT {
        dq.lpnlpi = index;
    } else {
        dq.fpnlpi = index;
    }
}

fn finddqsplit(dq: &Deque, pnlp: usize, pnls: &[Pnl]) -> usize {
    let mut index = dq.fpnlpi;
    while index < dq.apex {
        let a = pnls[dq.pnlps[index + 1]].pp;
        let b = pnls[dq.pnlps[index]].pp;
        let c = pnls[pnlp].pp;
        if ccw(a, b, c) == ISCCW {
            return index;
        }
        index += 1;
    }
    let mut index = dq.lpnlpi;
    while index > dq.apex {
        let a = pnls[dq.pnlps[index - 1]].pp;
        let b = pnls[dq.pnlps[index]].pp;
        let c = pnls[pnlp].pp;
        if ccw(a, b, c) == ISCW {
            return index;
        }
        index -= 1;
    }
    dq.apex
}

/// triang.c `between`.
fn between(pa: PointF, pb: PointF, pc: PointF) -> bool {
    if ccw(pa, pb, pc) != ISON {
        return false;
    }
    let pca = (pc.x - pa.x) * (pb.x - pa.x) + (pc.y - pa.y) * (pb.y - pa.y);
    let pba = (pb.x - pa.x) * (pb.x - pa.x) + (pb.y - pa.y) * (pb.y - pa.y);
    pca >= 0.0 && pca * pca <= pba * pba
}

/// triang.c `intersects`.
fn intersects(pa: PointF, pb: PointF, pc: PointF, pd: PointF) -> bool {
    let ccw1 = ccw(pa, pb, pc);
    let ccw2 = ccw(pa, pb, pd);
    let ccw3 = ccw(pc, pd, pa);
    let ccw4 = ccw(pc, pd, pb);
    if ccw1 == ISON || ccw2 == ISON || ccw3 == ISON || ccw4 == ISON {
        return between(pa, pb, pc)
            || between(pa, pb, pd)
            || between(pc, pd, pa)
            || between(pc, pd, pb);
    }
    (ccw1 == ISCCW) != (ccw2 == ISCCW) && (ccw3 == ISCCW) != (ccw4 == ISCCW)
}

/// triang.c `isdiagonal` over an index array.
fn isdiagonal(i: usize, ip2: usize, pts: &[usize], pnls: &[Pnl]) -> bool {
    let n = pts.len();
    let ip1 = (i + 1) % n;
    let im1 = (i + n - 1) % n;
    let p = |k: usize| pnls[pts[k]].pp;
    let res;
    if ccw(p(im1), p(i), p(ip1)) == ISCCW {
        res = ccw(p(i), p(ip2), p(im1)) == ISCCW && ccw(p(ip2), p(i), p(ip1)) == ISCCW;
    } else {
        res = ccw(p(i), p(ip2), p(ip1)) == ISCW;
    }
    if !res {
        return false;
    }
    for j in 0..n {
        let jp1 = (j + 1) % n;
        if !(j == i || jp1 == i || j == ip2 || jp1 == ip2) {
            if intersects(p(i), p(ip2), p(j), p(jp1)) {
                return false;
            }
        }
    }
    true
}

/// shortest.c `loadtriangle`.
fn loadtriangle(a: usize, b: usize, c: usize, tris: &mut Vec<Tri>) {
    tris.push(Tri {
        e: [(a, b), (b, c), (c, a)],
        right_index: [
            std::cell::Cell::new(SIZE_MAX),
            std::cell::Cell::new(SIZE_MAX),
            std::cell::Cell::new(SIZE_MAX),
        ],
        mark: std::cell::Cell::new(0),
    });
}

/// shortest.c `triangulate` — recursive ear clipping; a failure to find a
/// diagonal only prints an error in C and returns 0.
fn triangulate(pts: &mut Vec<usize>, tris: &mut Vec<Tri>, pnls: &[Pnl]) {
    let n = pts.len();
    if n > 3 {
        for pnli in 0..n {
            let pnlip1 = (pnli + 1) % n;
            let pnlip2 = (pnli + 2) % n;
            if isdiagonal(pnli, pnlip2, pts, pnls) {
                loadtriangle(pts[pnli], pts[pnlip1], pts[pnlip2], tris);
                // remove the ear tip
                let removed = pts.remove(pnlip1);
                let _ = removed;
                triangulate(pts, tris, pnls);
                return;
            }
        }
        // "triangulation failed" — fall through like C
    } else {
        loadtriangle(pts[0], pts[1], pts[2], tris);
    }
}

/// shortest.c `connecttris` — link triangles sharing an edge (node identity).
fn connecttris(tri1: usize, tri2: usize, tris: &mut [Tri]) {
    for i in 0..3 {
        for j in 0..3 {
            let (a0, a1) = tris[tri1].e[i];
            let (b0, b1) = tris[tri2].e[j];
            if (a0 == b0 && a1 == b1) || (a0 == b1 && a1 == b0) {
                tris[tri1].right_index[i].set(tri2);
                tris[tri2].right_index[j].set(tri1);
                return;
            }
        }
    }
}

/// shortest.c `marktripath` — recursive DFS marking the triangle strip.
fn marktripath(tris: &[Tri], trii: usize, trij: usize) -> bool {
    if tris[trii].mark.get() != 0 {
        return false;
    }
    tris[trii].mark.set(1);
    if trii == trij {
        return true;
    }
    for ei in 0..3 {
        let r = tris[trii].right_index[ei].get();
        if r != SIZE_MAX && marktripath(tris, r, trij) {
            return true;
        }
    }
    tris[trii].mark.set(0);
    false
}

/// shortest.c `pointintri` — boundary points qualify (collinear counts).
fn pointintri(tri: &Tri, p: PointF, pnls: &[Pnl]) -> bool {
    let mut sum = 0;
    for (a, b) in &tri.e {
        let cc = ccw(pnls[*a].pp, pnls[*b].pp, p);
        if cc != ISCW {
            sum += 1;
        }
    }
    sum == 3 || sum == 0
}

/// pathplan.h `Pshortestpath` — the funnel shortest path inside a simple
/// polygon. `polyp` is the polygon (closed boundary, last point not
/// repeated), `eps` the two endpoints. Returns the open polyline
/// `[eps0, ..., eps1]`, or an error code like C (-1 bad input, -2 internal).
pub fn pshortestpath(polyp: &[PointF], eps: &[PointF; 2]) -> Result<Vec<PointF>, i32> {
    let pn = polyp.len();

    // orientation: make point order CCW-per-this-code, drop dups
    let mut minpi = 0;
    for (i, p) in polyp.iter().enumerate() {
        if p.x < polyp[minpi].x {
            minpi = i;
        }
    }
    let p2 = polyp[minpi];
    let p1 = polyp[(minpi + pn - 1) % pn];
    let p3 = polyp[(minpi + 1) % pn];
    let reverse = (p1.x == p2.x && p2.x == p3.x && p3.y > p2.y) || ccw(p1, p2, p3) != ISCCW;

    // node arena: polygon nodes then the two endpoints (indices pn, pn+1)
    let mut pnls: Vec<Pnl> = Vec::with_capacity(pn + 2);
    if reverse {
        for pi in (0..pn).rev() {
            let prev = if pi + 1 < pn { polyp[pi + 1] } else { polyp[0] };
            if polyp[pi] == prev {
                continue; // dup
            }
            pnls.push(Pnl {
                pp: polyp[pi],
                link: std::cell::Cell::new(SIZE_MAX),
            });
        }
    } else {
        for pi in 0..pn {
            let prev = if pi > 0 { polyp[pi - 1] } else { polyp[pn - 1] };
            if polyp[pi] == prev {
                continue;
            }
            pnls.push(Pnl {
                pp: polyp[pi],
                link: std::cell::Cell::new(SIZE_MAX),
            });
        }
    }
    let pnll = pnls.len();
    let eps0_idx = pnll;
    let eps1_idx = pnll + 1;
    pnls.push(Pnl {
        pp: eps[0],
        link: std::cell::Cell::new(SIZE_MAX),
    });
    pnls.push(Pnl {
        pp: eps[1],
        link: std::cell::Cell::new(SIZE_MAX),
    });

    // triangulate
    let mut tris: Vec<Tri> = Vec::new();
    let mut pts: Vec<usize> = (0..pnll).collect();
    triangulate(&mut pts, &mut tris, &pnls);

    // connect shared edges
    let t = tris.len();
    for trii in 0..t {
        for trij in trii + 1..t {
            connecttris(trii, trij, &mut tris);
        }
    }

    // locate endpoints
    let mut ftrii = SIZE_MAX;
    for (i, tri) in tris.iter().enumerate() {
        if pointintri(tri, eps[0], &pnls) {
            ftrii = i;
            break;
        }
    }
    if ftrii == SIZE_MAX {
        return Err(-1); // "source point not in any triangle"
    }
    let mut ltrii = SIZE_MAX;
    for (i, tri) in tris.iter().enumerate() {
        if pointintri(tri, eps[1], &pnls) {
            ltrii = i;
            break;
        }
    }
    if ltrii == SIZE_MAX {
        return Err(-1); // "destination point not in any triangle"
    }

    // mark the strip
    if !marktripath(&tris, ftrii, ltrii) {
        // "a straight line is better than failing"
        return Ok(vec![eps[0], eps[1]]);
    }
    if ftrii == ltrii {
        return Ok(vec![eps[0], eps[1]]);
    }

    // funnel
    let mut dq = Deque {
        pnlps: vec![SIZE_MAX; 2 * (pnll + 2)],
        fpnlpi: pnll + 2,
        lpnlpi: pnll + 1,
        apex: 0,
    };
    add2dq(&mut dq, DQ_FRONT, eps0_idx, &pnls);
    dq.apex = dq.fpnlpi;

    let mut trii = ftrii;
    while trii != SIZE_MAX {
        tris[trii].mark.set(2);
        // find the exiting edge: one whose right neighbor is still marked 1
        let mut ei = 3;
        for e in 0..3 {
            let r = tris[trii].right_index[e].get();
            if r != SIZE_MAX && tris[r].mark.get() == 1 {
                ei = e;
                break;
            }
        }
        let (lpnlp, rpnlp);
        if ei == 3 {
            // last triangle: pair deque end with eps[1]
            let front = pnls[dq.pnlps[dq.fpnlpi]].pp;
            let back = pnls[dq.pnlps[dq.lpnlpi]].pp;
            if ccw(eps[1], front, back) == ISCCW {
                lpnlp = dq.pnlps[dq.lpnlpi];
                rpnlp = eps1_idx;
            } else {
                lpnlp = eps1_idx;
                rpnlp = dq.pnlps[dq.lpnlpi]; // NOTE: back, not front (C quirk)
            }
        } else {
            let pnlp = tris[trii].e[(ei + 1) % 3].1;
            let e0 = tris[trii].e[ei].0;
            let e1 = tris[trii].e[ei].1;
            if ccw(pnls[e0].pp, pnls[pnlp].pp, pnls[e1].pp) == ISCCW {
                lpnlp = e1;
                rpnlp = e0;
            } else {
                lpnlp = e0;
                rpnlp = e1;
            }
        }

        // deque update
        if trii == ftrii {
            add2dq(&mut dq, DQ_BACK, lpnlp, &pnls);
            add2dq(&mut dq, DQ_FRONT, rpnlp, &pnls);
        } else {
            let front = dq.pnlps[dq.fpnlpi];
            let back = dq.pnlps[dq.lpnlpi];
            if front != rpnlp && back != rpnlp {
                // right point is new
                let splitindex = finddqsplit(&dq, rpnlp, &pnls);
                splitdq(&mut dq, DQ_BACK, splitindex);
                add2dq(&mut dq, DQ_FRONT, rpnlp, &pnls);
                if splitindex > dq.apex {
                    dq.apex = splitindex;
                }
            } else {
                // left point is new
                let splitindex = finddqsplit(&dq, lpnlp, &pnls);
                splitdq(&mut dq, DQ_FRONT, splitindex);
                add2dq(&mut dq, DQ_BACK, lpnlp, &pnls);
                if splitindex < dq.apex {
                    dq.apex = splitindex;
                }
            }
        }

        // advance to the next strip triangle
        let mut next = SIZE_MAX;
        for e in 0..3 {
            let r = tris[trii].right_index[e].get();
            if r != SIZE_MAX && tris[r].mark.get() == 1 {
                next = r;
                break;
            }
        }
        trii = next;
    }

    // output: walk the link chain from epnls[1], fill ops in reverse
    let mut count = 0usize;
    let mut pnlp = eps1_idx;
    loop {
        count += 1;
        pnlp = pnls[pnlp].link.get();
        if pnlp == SIZE_MAX {
            break;
        }
    }
    let mut ops = vec![PointF::ZERO; count];
    let mut i = count;
    let mut pnlp = eps1_idx;
    loop {
        i -= 1;
        ops[i] = pnls[pnlp].pp;
        pnlp = pnls[pnlp].link.get();
        if pnlp == SIZE_MAX {
            break;
        }
    }
    Ok(ops)
}

/// `make_polyline` (util.c) — expand a polyline into degenerate cubics
/// (3·pn−2 points, 3-stride).
pub fn make_polyline(line: &[PointF]) -> Vec<PointF> {
    let mut out = Vec::with_capacity(3 * line.len().saturating_sub(2) + 2);
    if line.is_empty() {
        return out;
    }
    out.push(line[0]);
    out.push(line[0]);
    for p in line.iter().take(line.len().saturating_sub(1)).skip(1) {
        out.push(*p);
        out.push(*p);
        out.push(*p);
    }
    let last = *line.last().unwrap();
    out.push(last);
    out.push(last);
    out
}

// ---------------------------------------------------------------------------
// route.c — Proutespline
// ---------------------------------------------------------------------------

/// route.c `normv`.
fn normv(v: PointF) -> PointF {
    if v.x * v.x + v.y * v.y > 1e-6 {
        let d = (v.x * v.x + v.y * v.y).sqrt();
        PointF::new(v.x / d, v.y / d)
    } else {
        v
    }
}

/// route.c `dist_n` — polyline length.
fn dist_n(p: &[PointF]) -> f64 {
    let mut rv = 0.0;
    for i in 1..p.len() {
        rv += dist(p[i], p[i - 1]);
    }
    rv
}

// Bernstein helpers (route.c:462-495)
#[inline]
fn b0(t: f64) -> f64 {
    let x = 1.0 - t;
    x * x * x
}
#[inline]
fn b1(t: f64) -> f64 {
    3.0 * t * (1.0 - t) * (1.0 - t)
}
#[inline]
fn b2(t: f64) -> f64 {
    3.0 * t * t * (1.0 - t)
}
#[inline]
fn b3(t: f64) -> f64 {
    t * t * t
}
#[inline]
fn b01(t: f64) -> f64 {
    let x = 1.0 - t;
    x * x * (x + 3.0 * t)
}
#[inline]
fn b23(t: f64) -> f64 {
    t * t * (3.0 * (1.0 - t) + t)
}

/// `Proutespline` — fit a spline through the shortest path inside the
/// barriers. Returns the 3k+1 control-point array.
pub fn proutespline(
    barriers: &[(PointF, PointF)],
    input_route: &[PointF],
    endpoint_slopes: &[PointF; 2],
) -> Result<Vec<PointF>, i32> {
    if input_route.is_empty() {
        return Err(-1);
    }
    let ev0 = normv(endpoint_slopes[0]);
    let ev1 = normv(endpoint_slopes[1]);
    let mut ops: Vec<PointF> = Vec::with_capacity(4 * input_route.len());
    ops.push(input_route[0]);
    let mut ctx = RouteCtx { barriers, ops };
    reallyroutespline(&mut ctx, input_route, ev0, ev1)?;
    Ok(ctx.ops)
}

struct RouteCtx<'b> {
    barriers: &'b [(PointF, PointF)],
    ops: Vec<PointF>,
}

/// route.c `reallyroutespline`.
fn reallyroutespline(
    ctx: &mut RouteCtx,
    inps: &[PointF],
    ev0: PointF,
    ev1: PointF,
) -> Result<(), i32> {
    let inpn = inps.len();
    debug_assert!(inpn > 0);
    let mut tnas: Vec<(f64, [PointF; 2])> = vec![(0.0, [PointF::ZERO; 2]); inpn];
    for i in 1..inpn {
        tnas[i].0 = tnas[i - 1].0 + dist(inps[i], inps[i - 1]);
    }
    for i in 1..inpn {
        tnas[i].0 /= tnas[inpn - 1].0;
    }
    for i in 0..inpn {
        tnas[i].1[0] = scale(b1(tnas[i].0), ev0);
        tnas[i].1[1] = scale(b2(tnas[i].0), ev1);
    }
    let (p1, v1, p2, v2) = mkspline(inps, &tnas, ev0, ev1);
    let fit = splinefits(ctx, p1, v1, p2, v2, inps);
    if fit > 0 {
        return Ok(());
    }
    if fit < 0 {
        return Err(-1);
    }

    // split at the input vertex farthest from the LSQ curve
    let cp1 = add(p1, scale(1.0 / 3.0, v1));
    let cp2 = sub(p2, scale(1.0 / 3.0, v2));
    let mut maxi: i64 = -1;
    let mut maxd = -1.0f64;
    for i in 1..inpn.saturating_sub(1) {
        let t = tnas[i].0;
        let p = add(
            add(scale(b0(t), p1), scale(b1(t), cp1)),
            add(scale(b2(t), cp2), scale(b3(t), p2)),
        );
        let d = dist(p, inps[i]);
        if d > maxd {
            maxd = d;
            maxi = i as i64;
        }
    }
    let spliti = maxi;
    if spliti < 0 {
        // unreachable per C (inpn>=3 here); bail like an alloc failure
        return Err(-1);
    }
    let spliti = spliti as usize;
    let splitv1 = normv(sub(inps[spliti], inps[spliti - 1]));
    let splitv2 = normv(sub(inps[spliti + 1], inps[spliti]));
    let splitv = normv(add(splitv1, splitv2));
    reallyroutespline(ctx, &inps[..spliti + 1], ev0, splitv)?;
    reallyroutespline(ctx, &inps[spliti..], splitv, ev1)?;
    Ok(())
}

/// route.c `mkspline` — least-squares handle magnitudes.
fn mkspline(
    inps: &[PointF],
    tnas: &[(f64, [PointF; 2])],
    ev0: PointF,
    ev1: PointF,
) -> (PointF, PointF, PointF, PointF) {
    let inpn = inps.len();
    let mut c00 = 0.0;
    let mut c01 = 0.0;
    let mut c11 = 0.0;
    let mut x0 = 0.0;
    let mut x1 = 0.0;
    for i in 0..inpn {
        let t = tnas[i].0;
        let (a0, a1) = (tnas[i].1[0], tnas[i].1[1]);
        c00 += dot(a0, a0);
        c01 += dot(a0, a1);
        c11 += dot(a1, a1);
        let tmp = sub(
            inps[i],
            add(scale(b01(t), inps[0]), scale(b23(t), inps[inpn - 1])),
        );
        x0 += dot(a0, tmp);
        x1 += dot(a1, tmp);
    }
    let c10 = c01;
    let det01 = c00 * c11 - c10 * c01;
    let det0x = c00 * x1 - c01 * x0;
    let detx1 = x0 * c11 - x1 * c01;
    let mut scale0 = 0.0;
    let mut scale3 = 0.0;
    if det01.abs() >= 1e-6 {
        scale0 = detx1 / det01;
        scale3 = det0x / det01;
    }
    if det01.abs() < 1e-6 || scale0 <= 0.0 || scale3 <= 0.0 {
        let d01 = dist(inps[0], inps[inpn - 1]) / 3.0;
        scale0 = d01;
        scale3 = d01;
    }
    (
        inps[0],
        scale(scale0, ev0),
        inps[inpn - 1],
        scale(scale3, ev1),
    )
}

/// route.c `splinefits` — 1 = fitted, 0 = no fit, -1 = failure.
fn splinefits(
    ctx: &mut RouteCtx,
    pa: PointF,
    va: PointF,
    pb: PointF,
    vb: PointF,
    inps: &[PointF],
) -> i32 {
    let inpn = inps.len();
    let forceflag = inpn == 2;
    let mut a = 4.0f64;
    let mut first = true;
    loop {
        let mut sps = [PointF::ZERO; 4];
        sps[0] = pa;
        sps[1] = add(pa, scale(a / 3.0, va));
        sps[2] = sub(pb, scale(a / 3.0, vb));
        sps[3] = pb;
        if first && dist_n(&sps) < dist_n(inps) - EPSILON1 {
            return 0; // shortcuts not allowed
        }
        first = false;
        if splineisinside(ctx.barriers, &sps) {
            ctx.ops.push(sps[1]);
            ctx.ops.push(sps[2]);
            ctx.ops.push(sps[3]);
            return 1;
        }
        if a < 0.005 {
            if forceflag {
                ctx.ops.push(sps[1]);
                ctx.ops.push(sps[2]);
                ctx.ops.push(sps[3]);
                return 1;
            }
            break;
        }
        if a > 0.01 {
            a /= 2.0;
        } else {
            a = 0.0;
        }
    }
    0
}

/// route.c `splineisinside`.
fn splineisinside(barriers: &[(PointF, PointF)], sps: &[PointF; 4]) -> bool {
    for (a, b) in barriers {
        let lps = [*a, *b];
        let mut roots = [0.0f64; 4];
        let rootn = splineintersectsline(sps, &lps, &mut roots);
        if rootn == 4 {
            continue;
        }
        for t in roots.iter().take(rootn) {
            if *t < EPSILON2 || *t > 1.0 - EPSILON2 {
                continue;
            }
            let ta = (1.0 - t) * (1.0 - t) * (1.0 - t);
            let tb = 3.0 * t * (1.0 - t) * (1.0 - t);
            let tc = 3.0 * t * t * (1.0 - t);
            let td = t * t * t;
            let ip = add(
                add(scale(ta, sps[0]), scale(tb, sps[1])),
                add(scale(tc, sps[2]), scale(td, sps[3])),
            );
            if dist2(ip, lps[0]) < EPSILON1 || dist2(ip, lps[1]) < EPSILON1 {
                continue; // touching a barrier endpoint is OK
            }
            return false;
        }
    }
    true
}

/// route.c `splineintersectsline` — returns the root count, or 4 for
/// "coincident".
fn splineintersectsline(sps: &[PointF; 4], lps: &[PointF; 2], roots: &mut [f64; 4]) -> usize {
    let xcoeff = [lps[0].x, lps[1].x - lps[0].x];
    let ycoeff = [lps[0].y, lps[1].y - lps[0].y];
    let mut rootn = 0usize;

    if xcoeff[1] == 0.0 && ycoeff[1] == 0.0 {
        let mut xroots = [0.0f64; 4];
        let mut yroots = [0.0f64; 4];
        let mut xc = [0.0f64; 4];
        points2coeff(sps[0].x, sps[1].x, sps[2].x, sps[3].x, &mut xc);
        xc[0] -= xcoeff[0];
        let xrootn = solve3(&xc, &mut xroots);
        let mut yc = [0.0f64; 4];
        points2coeff(sps[0].y, sps[1].y, sps[2].y, sps[3].y, &mut yc);
        yc[0] -= ycoeff[0];
        let yrootn = solve3(&yc, &mut yroots);
        if xrootn == 4 && yrootn == 4 {
            return 4;
        }
        if xrootn == 4 {
            for r in yroots.iter().take(yrootn) {
                addroot(*r, roots, &mut rootn);
            }
        } else if yrootn == 4 {
            for r in xroots.iter().take(xrootn) {
                addroot(*r, roots, &mut rootn);
            }
        } else {
            for xr in xroots.iter().take(xrootn) {
                for yr in yroots.iter().take(yrootn) {
                    if xr == yr {
                        addroot(*xr, roots, &mut rootn);
                        break;
                    }
                }
            }
        }
        return rootn;
    }

    if xcoeff[1] == 0.0 {
        // vertical
        let mut xroots = [0.0f64; 4];
        let mut xc = [0.0f64; 4];
        points2coeff(sps[0].x, sps[1].x, sps[2].x, sps[3].x, &mut xc);
        xc[0] -= xcoeff[0];
        let xrootn = solve3(&xc, &mut xroots);
        if xrootn == 4 {
            return 4;
        }
        for tv in xroots.iter().take(xrootn) {
            if *tv < 0.0 || *tv > 1.0 {
                continue;
            }
            // the power-basis evaluation like C: c0 + tv(c1 + tv(c2 + tv c3))
            let sv = yc_eval(sps, *tv, 1);
            let s = (sv - ycoeff[0]) / ycoeff[1];
            if (0.0..=1.0).contains(&s) {
                addroot(*tv, roots, &mut rootn);
            }
        }
        return rootn;
    }

    // general case
    let rat = ycoeff[1] / xcoeff[1];
    let mut scoeff = [0.0f64; 4];
    points2coeff(sps[0].y, sps[1].y, sps[2].y, sps[3].y, &mut scoeff);
    let mut xpc = [0.0f64; 4];
    points2coeff(sps[0].x, sps[1].x, sps[2].x, sps[3].x, &mut xpc);
    for i in 0..4 {
        scoeff[i] -= rat * xpc[i];
    }
    scoeff[0] += rat * xcoeff[0] - ycoeff[0];
    let mut sroots = [0.0f64; 4];
    let srootn = solve3(&scoeff, &mut sroots);
    if srootn == 4 {
        return 4;
    }
    for tv in sroots.iter().take(srootn) {
        if *tv < 0.0 || *tv > 1.0 {
            continue;
        }
        let sv = yc_eval_x(sps, *tv);
        let s = (sv - xcoeff[0]) / xcoeff[1];
        if (0.0..=1.0).contains(&s) {
            addroot(*tv, roots, &mut rootn);
        }
    }
    rootn
}

/// evaluates the spline's y via the power basis (c0 + t(c1 + t(c2 + t c3)))
fn yc_eval(sps: &[PointF; 4], t: f64, _axis: usize) -> f64 {
    let mut coeff = [0.0f64; 4];
    points2coeff(sps[0].y, sps[1].y, sps[2].y, sps[3].y, &mut coeff);
    coeff[0] + t * (coeff[1] + t * (coeff[2] + t * coeff[3]))
}

fn yc_eval_x(sps: &[PointF; 4], t: f64) -> f64 {
    let mut coeff = [0.0f64; 4];
    points2coeff(sps[0].x, sps[1].x, sps[2].x, sps[3].x, &mut coeff);
    coeff[0] + t * (coeff[1] + t * (coeff[2] + t * coeff[3]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve3_finds_roots() {
        // (t-0.5)(t-1)(t-0.25) = t^3 - 1.75 t^2 + 0.875 t - 0.125
        let coeff = [-0.125, 0.875, -1.75, 1.0];
        let mut roots = [0.0f64; 4];
        let n = solve3(&coeff, &mut roots);
        assert_eq!(n, 3);
        let mut sorted = roots[..3].to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 0.25).abs() < 1e-6, "{sorted:?}");
        assert!((sorted[1] - 0.5).abs() < 1e-6);
        assert!((sorted[2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn shortest_path_in_a_square() {
        // square polygon, endpoints on the bottom edge
        let poly = vec![
            PointF::new(0.0, 0.0),
            PointF::new(100.0, 0.0),
            PointF::new(100.0, 100.0),
            PointF::new(0.0, 100.0),
        ];
        let eps = [PointF::new(10.0, 0.0), PointF::new(90.0, 0.0)];
        let path = pshortestpath(&poly, &eps).expect("path");
        assert_eq!(path.len(), 2);
        assert!(dist(path[0], eps[0]) < 1e-9);
        assert!(dist(path[1], eps[1]) < 1e-9);
    }

    #[test]
    fn shortest_path_routes_around_a_reflex() {
        // L-shaped polygon: the straight line dips outside, so the funnel
        // must hug the reflex vertex.
        let poly = vec![
            PointF::new(0.0, 0.0),
            PointF::new(100.0, 0.0),
            PointF::new(100.0, 40.0),
            PointF::new(40.0, 40.0),
            PointF::new(40.0, 100.0),
            PointF::new(0.0, 100.0),
        ];
        let eps = [PointF::new(80.0, 10.0), PointF::new(10.0, 80.0)];
        let path = pshortestpath(&poly, &eps).expect("path");
        assert!(dist(path[0], eps[0]) < 1e-9);
        assert!(dist(path[path.len() - 1], eps[1]) < 1e-9);
        // every intermediate vertex must be a polygon vertex (the funnel
        // path bends only at polygon corners)
        for p in &path[1..path.len() - 1] {
            assert!(poly.iter().any(|q| dist(*q, *p) < 1e-9), "{p:?}");
        }
        // and it must actually bend around (40,40)
        assert!(
            path.iter()
                .any(|p| dist(*p, PointF::new(40.0, 40.0)) < 1e-9)
        );
    }

    #[test]
    fn routespline_fits_through_a_channel() {
        // an S-free channel: shortest path with a corner, slopes vertical
        let barriers = vec![];
        let input = vec![
            PointF::new(0.0, 0.0),
            PointF::new(0.0, 50.0),
            PointF::new(50.0, 50.0),
        ];
        let slopes = [PointF::new(0.0, 1.0), PointF::new(1.0, 0.0)];
        let ops = proutespline(&barriers, &input, &slopes).expect("spline");
        // 3k+1 control points; endpoints match the input
        assert_eq!((ops.len() - 1) % 3, 0);
        assert!(dist(ops[0], input[0]) < 1e-9);
        assert!(dist(*ops.last().unwrap(), *input.last().unwrap()) < 1e-9);
    }

    #[test]
    fn make_polyline_expands() {
        let line = vec![
            PointF::ZERO,
            PointF::new(10.0, 0.0),
            PointF::new(10.0, 10.0),
        ];
        let out = make_polyline(&line);
        assert_eq!(out.len(), 3 * 3 - 2);
        // segment 0 = (p0, p0, p1, p1)
        assert_eq!(out[0], line[0]);
        assert_eq!(out[1], line[0]);
        assert_eq!(out[2], line[1]);
        assert_eq!(out[3], line[1]);
    }
}
