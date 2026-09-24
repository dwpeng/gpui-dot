//! Spline routing core — the `lib/common/routespl.c` + `lib/common/splines.c`
//! layer dot's edge router funnels through: channel-box validation/repair
//! (`checkpath`), the channel-polygon → shortest-path → Hermite-spline
//! pipeline (`routesplines_`), box space reclaiming (`limitBoxes`), and the
//! final endpoint clipping + arrow adjustments (`clip_and_install`).

use super::arrows::{arrow_clip, bezier_eval};
use super::geom::{BoxF, PointF, approx_eqpt};

fn approx_eqpt_milli(a: PointF, b: PointF) -> bool {
    approx_eqpt(a, b) || (a.x - b.x).abs() < MILLIPOINT && (a.y - b.y).abs() < MILLIPOINT
}
use super::model::{EdgeType, Fg, NId};
use super::pathplan::{make_polyline, proutespline, pshortestpath};

const FUDGE: f64 = 0.0001;
const INIT_DELTA: f64 = 10.0;
const LOOP_TRIES: usize = 15;
const MILLIPOINT: f64 = 0.001;
const DBL_MAX: f64 = f64::MAX;

/// An edge endpoint (`path.start` / `path.end`).
#[derive(Debug, Clone, Copy, Default)]
pub struct EndPoint {
    pub p: PointF,
    pub theta: f64,
    pub constrained: bool,
}

/// The routing path (`path` struct): channel boxes tail-end first plus the
/// two endpoints.
#[derive(Debug, Clone, Default)]
pub struct Path {
    pub start: EndPoint,
    pub end: EndPoint,
    pub boxes: Vec<BoxF>,
    /// The fast edge being routed.
    pub data: usize,
}

/// routespl.c `overlap`.
fn overlap(i0: f64, i1: f64, j0: f64, j1: f64) -> f64 {
    if i1 < j0 || j1 < i0 {
        return 0.0;
    }
    if i0 <= j0 && i1 >= j1 {
        return i1 - i0;
    }
    if j0 <= i0 && j1 >= i1 {
        return j1 - j0;
    }
    if j0 <= i0 && i0 <= j1 {
        j1 - i0
    } else {
        i1 - j0
    }
}

/// routespl.c `checkpath` — validate and repair the channel boxes in place.
pub fn checkpath(boxes: &mut Vec<BoxF>, path: &mut Path) -> Result<(), ()> {
    // 1. drop degenerate boxes
    let mut kept: Vec<BoxF> = Vec::with_capacity(boxes.len());
    for b in boxes.iter() {
        if (b.ll.y - b.ur.y).abs() < 0.01 {
            continue;
        }
        if (b.ll.x - b.ur.x).abs() < 0.01 {
            continue;
        }
        kept.push(*b);
    }
    *boxes = kept;
    if boxes.is_empty() {
        return Err(()); // "all bounding boxes are below threshold"
    }

    // 2-4. adjacency repair + overlap resolution
    for bi in 0..boxes.len().saturating_sub(1) {
        let (mut ba, mut bb) = (boxes[bi], boxes[bi + 1]);
        let mut l = ba.ur.x < bb.ll.x;
        let mut r = ba.ll.x > bb.ur.x;
        let mut d = ba.ur.y < bb.ll.y;
        let mut u = ba.ll.y > bb.ur.y;
        let mut errs = [l, r, d, u].iter().filter(|&&x| x).count();
        if errs > 0 {
            // first violating axis: swap the offending coordinates
            if l {
                std::mem::swap(&mut ba.ur.x, &mut bb.ll.x);
            } else if r {
                std::mem::swap(&mut ba.ll.x, &mut bb.ur.x);
            } else if d {
                std::mem::swap(&mut ba.ur.y, &mut bb.ll.y);
            } else if u {
                std::mem::swap(&mut ba.ll.y, &mut bb.ur.y);
            }
            errs -= 1;
            // remaining violations: meet at midpoint + 0.5
            while errs > 0 {
                l = ba.ur.x < bb.ll.x;
                r = ba.ll.x > bb.ur.x;
                d = ba.ur.y < bb.ll.y;
                u = ba.ll.y > bb.ur.y;
                if l {
                    let m = (ba.ur.x + bb.ll.x) / 2.0 + 0.5;
                    ba.ur.x = m;
                    bb.ll.x = m;
                } else if r {
                    let m = (ba.ll.x + bb.ur.x) / 2.0 + 0.5;
                    ba.ll.x = m;
                    bb.ur.x = m;
                } else if d {
                    let m = (ba.ur.y + bb.ll.y) / 2.0 + 0.5;
                    ba.ur.y = m;
                    bb.ll.y = m;
                } else if u {
                    let m = (ba.ll.y + bb.ur.y) / 2.0 + 0.5;
                    ba.ll.y = m;
                    bb.ur.y = m;
                }
                errs -= 1;
            }
        }

        // overlap resolution
        let xoverlap = overlap(ba.ll.x, ba.ur.x, bb.ll.x, bb.ur.x);
        let yoverlap = overlap(ba.ll.y, ba.ur.y, bb.ll.y, bb.ur.y);
        if xoverlap > 0.0 && yoverlap > 0.0 {
            if xoverlap < yoverlap {
                if ba.ur.x - ba.ll.x > bb.ur.x - bb.ll.x {
                    if ba.ur.x < bb.ur.x {
                        ba.ur.x = bb.ll.x;
                    } else {
                        ba.ll.x = bb.ur.x;
                    }
                } else if ba.ur.x < bb.ur.x {
                    bb.ll.x = ba.ur.x;
                } else {
                    bb.ur.x = ba.ll.x;
                }
            } else if ba.ur.y - ba.ll.y > bb.ur.y - bb.ll.y {
                if ba.ur.y < bb.ur.y {
                    ba.ur.y = bb.ll.y;
                } else {
                    ba.ll.y = bb.ur.y;
                }
            } else if ba.ur.y < bb.ur.y {
                bb.ll.y = ba.ur.y;
            } else {
                bb.ur.y = ba.ll.y;
            }
        }
        boxes[bi] = ba;
        boxes[bi + 1] = bb;
    }

    // 5. clamp endpoints into first/last box
    let first = boxes[0];
    if !first.contains(path.start.p) {
        path.start.p.x = path.start.p.x.min(first.ur.x).max(first.ll.x);
        path.start.p.y = path.start.p.y.min(first.ur.y).max(first.ll.y);
    }
    let last = *boxes.last().unwrap();
    if !last.contains(path.end.p) {
        path.end.p.x = path.end.p.x.min(last.ur.x).max(last.ll.x);
        path.end.p.y = path.end.p.y.min(last.ur.y).max(last.ll.y);
    }
    Ok(())
}

/// routespl.c `limitBoxes` — bound each box's x extent by sampling the spline.
fn limit_boxes(boxes: &mut [BoxF], pps: &[PointF], delta: f64) {
    let boxn = boxes.len();
    let pn = pps.len();
    let num_div = delta * boxn as f64;
    let mut splinepi = 0;
    while splinepi + 3 < pn {
        let sp0 = [
            pps[splinepi],
            pps[splinepi + 1],
            pps[splinepi + 2],
            pps[splinepi + 3],
        ];
        for si in 0..=(num_div as usize) {
            let t = si as f64 / num_div;
            let mut sp = sp0;
            // De Casteljau at t, in C's exact order (routespl.c:244-255): the
            // six lerps collapse sp[0] onto the curve point. Updating only
            // sp[3]/sp[2]/sp[1] (as this port did) leaves sp[0] at the first
            // control point, so only the box containing p0 is ever sampled,
            // the `bounded` test never succeeds, and every edge pays the full
            // 15 doubling retries — 17s on 1.dot instead of 0.6s.
            sp[1].x += t * (sp[2].x - sp[1].x);
            sp[1].y += t * (sp[2].y - sp[1].y);
            sp[2].x += t * (sp[3].x - sp[2].x);
            sp[2].y += t * (sp[3].y - sp[2].y);
            sp[0].x += t * (sp[1].x - sp[0].x);
            sp[0].y += t * (sp[1].y - sp[0].y);
            sp[1].x += t * (sp[2].x - sp[1].x);
            sp[1].y += t * (sp[2].y - sp[1].y);
            sp[0].x += t * (sp[1].x - sp[0].x);
            sp[0].y += t * (sp[1].y - sp[0].y);
            let pt = sp[0];
            for b in boxes.iter_mut() {
                if pt.y <= b.ur.y + FUDGE && pt.y >= b.ll.y - FUDGE {
                    b.ll.x = b.ll.x.min(pt.x);
                    b.ur.x = b.ur.x.max(pt.x);
                }
            }
        }
        splinepi += 3;
    }
}

/// routespl.c `routesplines_` — route one edge through its channel boxes.
/// Returns the 3k+1 Bézier control points, or None on failure (the caller
/// skips the edge, like C).
pub fn routesplines_(path: &mut Path, polyline: bool) -> Option<Vec<PointF>> {
    let mut boxes = path.boxes.clone();
    if checkpath(&mut boxes, path).is_err() {
        return None;
    }
    path.boxes = boxes.clone();
    let boxn = boxes.len();

    // y-flip if the channel runs upward
    let mut flip = false;
    if boxn > 1 && boxes[0].ll.y > boxes[1].ll.y {
        flip = true;
        for b in boxes.iter_mut() {
            let v = b.ur.y;
            b.ur.y = -b.ll.y;
            b.ll.y = -v;
        }
    }

    // build the channel polygon
    let mut polypoints: Vec<PointF> = Vec::with_capacity(8 * boxn);
    let emit = |polypoints: &mut Vec<PointF>, x: f64, y: f64| {
        polypoints.push(PointF::new(x, y));
    };

    // forward pass: left boundary, bi = 0..boxn
    for bi in 0..boxn {
        let prev: i32 = if bi > 0 {
            if boxes[bi].ll.y > boxes[bi - 1].ll.y {
                -1
            } else {
                1
            }
        } else {
            0
        };
        let next: i32 = if bi + 1 < boxn {
            if boxes[bi + 1].ll.y > boxes[bi].ll.y {
                1
            } else {
                -1
            }
        } else {
            0
        };
        if prev != next {
            if next == -1 || prev == 1 {
                emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ur.y);
                emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ll.y);
            } else {
                emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ll.y);
                emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ur.y);
            }
        } else if prev == 0 {
            emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ur.y);
            emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ll.y);
        } else {
            // stacked boxes: nothing in the forward pass
        }
    }

    // backward pass: right boundary, bi = boxn-1 .. 0
    for bi in (0..boxn).rev() {
        let prev: i32 = if bi + 1 < boxn {
            if boxes[bi].ll.y > boxes[bi + 1].ll.y {
                -1
            } else {
                1
            }
        } else {
            0
        };
        let next: i32 = if bi > 0 {
            if boxes[bi - 1].ll.y > boxes[bi].ll.y {
                1
            } else {
                -1
            }
        } else {
            0
        };
        if prev != next {
            if next == -1 || prev == 1 {
                emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ur.y);
                emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ll.y);
            } else {
                emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ll.y);
                emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ur.y);
            }
        } else if prev == 0 {
            emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ll.y);
            emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ur.y);
        } else {
            // degenerate/stacked box: traverse fully
            emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ll.y);
            emit(&mut polypoints, boxes[bi].ur.x, boxes[bi].ur.y);
            emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ur.y);
            emit(&mut polypoints, boxes[bi].ll.x, boxes[bi].ll.y);
        }
    }
    let pi = polypoints.len();

    // un-flip
    if flip {
        for b in boxes.iter_mut() {
            let v = b.ur.y;
            b.ur.y = -b.ll.y;
            b.ll.y = -v;
        }
        for p in polypoints.iter_mut() {
            p.y *= -1.0;
        }
    }

    // reset box x extents and run the shortest path
    for b in boxes.iter_mut() {
        b.ll.x = DBL_MAX;
        b.ur.x = -DBL_MAX;
    }
    let eps = [path.start.p, path.end.p];
    let pl = pshortestpath(&polypoints, &eps).ok()?;
    if std::env::var_os("GD_POLYDUMP").is_some() {
        eprintln!("POLY eps={:?}", eps);
        for p in &polypoints {
            eprintln!("POLY {:.3} {:.3}", p.x, p.y);
        }
        for p in &pl {
            eprintln!("PATH {:.3} {:.3}", p.x, p.y);
        }
        eprintln!("POLYEND");
    }

    // convert polyline to Bézier control points
    let spl: Vec<PointF> = if polyline {
        make_polyline(&pl)
    } else {
        let mut edges = Vec::with_capacity(pi);
        for i in 0..pi {
            edges.push((polypoints[i], polypoints[(i + 1) % pi]));
        }
        let ev0 = if path.start.constrained {
            PointF::new(path.start.theta.cos(), path.start.theta.sin())
        } else {
            PointF::ZERO
        };
        let ev1 = if path.end.constrained {
            PointF::new(-path.end.theta.cos(), -path.end.theta.sin())
        } else {
            PointF::ZERO
        };
        proutespline(&edges, &pl, &[ev0, ev1]).ok()?
    };

    // reclaim box x-space
    let is_horizontal = !spl.is_empty() && spl.iter().all(|p| (spl[0].y - p.y).abs() <= FUDGE);
    let is_vertical = !spl.is_empty() && spl.iter().all(|p| (spl[0].x - p.x).abs() <= FUDGE);
    let mut unbounded = true;
    if is_horizontal || is_vertical {
        for b in boxes.iter_mut() {
            b.ll.x = spl[0].x;
            b.ur.x = spl[0].x;
        }
        unbounded = false;
    }
    if unbounded {
        let mut delta = INIT_DELTA;
        for _ in 0..LOOP_TRIES {
            limit_boxes(&mut boxes, &spl, delta);
            let bounded = boxes
                .iter()
                .all(|b| b.ll.x != DBL_MAX && b.ur.x != -DBL_MAX);
            if bounded {
                unbounded = false;
                break;
            }
            delta *= 2.0;
        }
    }
    if unbounded {
        // "Unable to reclaim box space" — fall back to the shortest path
        let polyspl = make_polyline(&pl);
        limit_boxes(&mut boxes, &polyspl, INIT_DELTA);
    }
    path.boxes = boxes;
    Some(spl)
}

// ---------------------------------------------------------------------------
// splines.c — clip_and_install + shape clipping
// ---------------------------------------------------------------------------

/// Node shape inside test (node-relative coordinates, y up). Mirrors the
/// `insidefn` contract for the shapes this port sizes: rectangles (box
/// family), ellipses and diamonds (as the polygon they are).
pub fn shape_inside(fg: &Fg, n: NId, p: PointF) -> bool {
    shape_inside_bp(fg, n, None, p)
}

/// [`shape_inside`] with an explicit port rectangle: record (and HTML) ports
/// carry the field's box, and the inside test then uses that box instead of
/// the whole node (`inside_context->s.bp`, shapes.c:3762-3785).
pub fn shape_inside_bp(fg: &Fg, n: NId, bp: Option<super::geom::BoxF>, p: PointF) -> bool {
    let node = &fg.nodes[n];
    if let Some(record) = fg.records.get(&n) {
        // record_inside: rotate the point into the node frame, then test the
        // field box (or the record's root box) grown by penwidth/2.
        let rankdir = fg.graphs[fg.root_g()].rankdir;
        let rank = match rankdir {
            super::model::RankDir::Tb => 0,
            super::model::RankDir::Lr => 1,
            super::model::RankDir::Bt => 2,
            super::model::RankDir::Rl => 3,
        };
        let p = super::shapes::ccwrotatepf(p, rank * 90);
        let penwidth = if node.penwidth > 0.0 {
            node.penwidth
        } else {
            super::shapes::DEFAULT_NODEPENWIDTH
        };
        return super::shapes::record_inside_test(record, bp, p, penwidth);
    }
    let kind = if node.shape_info.kind.is_empty() {
        "box"
    } else {
        node.shape_info.kind.as_str()
    };
    let desc = super::shapes::resolved_desc(&super::shapes::shape_of(kind), &node.shape_info);
    // The cached `poly_init` vertices were built from the *unflipped* box;
    // for LR/BT the layout frame swaps a node's dimensions, so drop the cache
    // and let the test rebuild the rings from the current lw/rw/ht — the very
    // box the spline has to stop at.
    let mut info = node.shape_info.clone();
    info.vertices = None;
    super::shapes::poly_inside_test(&desc, node.lw, node.rw, node.ht, &info, None, p)
}

/// splines.c `shape_clip0` — clip the curve's start to the node boundary.
pub(crate) fn shape_clip0(
    fg: &Fg,
    n: NId,
    curve: &mut [PointF; 4],
    coord: PointF,
    left_inside: bool,
) {
    // node-relative coordinates
    for p in curve.iter_mut() {
        p.x -= coord.x;
        p.y -= coord.y;
    }
    let mut sp = *curve;
    let inside = |p: PointF| shape_inside(fg, n, p);
    if left_inside {
        let p0 = sp[0];
        let _ = p0;
        super::arrows::bezier_clip(&mut sp, inside, true);
    } else {
        super::arrows::bezier_clip(&mut sp, inside, false);
    }
    for p in sp.iter_mut() {
        p.x += coord.x;
        p.y += coord.y;
    }
    *curve = sp;
}

/// splines.c `clip_and_install` — finalize one routed Bézier: clip the ends
/// to the node shapes, skip degenerate segments, pull back for arrows and
/// store the result on the original edge, updating the graph bounding box.
#[allow(clippy::too_many_arguments)]
pub fn clip_and_install(
    fg: &mut Fg,
    fe: usize,
    hn: NId,
    ps: &mut [PointF],
    swap_ends: bool,
    is_ortho: bool,
) {
    let pn = ps.len();
    if pn < 4 {
        return;
    }
    let tn = fg.edges[fe].tail;
    let (mut tn, mut hn) = (tn, hn);

    // reversed flat edge? swap which node is "head" for clipping
    if fg.nodes[tn].rank == fg.nodes[hn].rank && fg.nodes[tn].order > fg.nodes[hn].order {
        std::mem::swap(&mut tn, &mut hn);
    }

    // walk to the original edge for ports
    let mut orig = fe;
    while let Some(o) = fg.edges[orig].to_orig {
        if fg.edges[orig].edge_type == EdgeType::Normal {
            break;
        }
        orig = o;
    }
    // An explicit port (`tailport`/`headport`, compass or `_`) turns clipping
    // off for that end: the endpoint must stay exactly on the port, not be
    // pulled back to wherever the spline happens to cross the outline.
    let (clip_tail, clip_head) = if tn == fg.edges[orig].tail {
        (fg.edges[orig].tail_port.clip, fg.edges[orig].head_port.clip)
    } else {
        // the fast edge runs opposite to the original
        (fg.edges[orig].head_port.clip, fg.edges[orig].tail_port.clip)
    };

    // tail-end shape clipping
    let mut start = 0usize;
    if clip_tail {
        while start < pn - 4 {
            let p2 = PointF::new(
                ps[start + 3].x - fg.nodes[tn].coord.x,
                ps[start + 3].y - fg.nodes[tn].coord.y,
            );
            if !shape_inside(fg, tn, p2) {
                break;
            }
            start += 3;
        }
        let mut curve = [ps[start], ps[start + 1], ps[start + 2], ps[start + 3]];
        shape_clip0(fg, tn, &mut curve, fg.nodes[tn].coord, true);
        ps[start..start + 4].copy_from_slice(&curve);
    }

    // head-end shape clipping (mirror)
    let mut end = pn - 4;
    if clip_head {
        while end > 0 {
            let p2 = PointF::new(
                ps[end].x - fg.nodes[hn].coord.x,
                ps[end].y - fg.nodes[hn].coord.y,
            );
            if !shape_inside(fg, hn, p2) {
                break;
            }
            end -= 3;
        }
        let mut curve = [ps[end], ps[end + 1], ps[end + 2], ps[end + 3]];
        shape_clip0(fg, hn, &mut curve, fg.nodes[hn].coord, false);
        ps[end..end + 4].copy_from_slice(&curve);
    }

    // skip degenerate end segments
    while start < pn - 4 && approx_eqpt_milli(ps[start], ps[start + 3]) {
        start += 3;
    }
    while end > 0 && approx_eqpt_milli(ps[end], ps[end + 3]) {
        end -= 3;
    }
    let _ = orig;

    // arrow clipping — the clip reports the true tip anchors (sp/ep)
    let mut spl_sp = PointF::ZERO;
    let mut spl_ep = PointF::ZERO;
    let (sflag, eflag) = arrow_clip(
        fg,
        fe,
        ps,
        &mut start,
        &mut end,
        &mut spl_sp,
        &mut spl_ep,
        swap_ends,
        false,
        false,
        is_ortho,
    );

    // copy surviving points into the new bezier. When the two node shapes
    // swallow the whole polyline, the tail and head clip walks can pass each
    // other and land `end` below `start`; clamp so the slice stays valid and
    // the edge keeps its single surviving segment instead of underflowing
    // `end - start + 4` (usize wrap → slice panic).
    let end = end.max(start);
    let size = end - start + 4;
    let pts: Vec<PointF> = ps[start..start + size].to_vec();
    // 3-stride cubic segments sharing endpoints
    let mut segs: Vec<[PointF; 4]> = Vec::with_capacity((size - 1) / 3);
    let mut k = 0usize;
    while k + 3 < size {
        segs.push([pts[k], pts[k + 1], pts[k + 2], pts[k + 3]]);
        k += 3;
    }
    // update the graph bounding box
    let mut bb = fg.graphs[0].bb;
    for seg in &segs {
        update_bb_bz(&mut bb, seg);
    }
    fg.graphs[0].bb = bb;

    // store on the original edge
    let mut orig = fe;
    while let Some(o) = fg.edges[orig].to_orig {
        orig = o;
    }
    fg.edges[orig].spl = Some(super::model::Splines {
        list: segs,
        sp: spl_sp,
        ep: spl_ep,
    });
    fg.edges[orig].sflag = sflag;
    fg.edges[orig].eflag = eflag;
}

/// emit.c `update_bb_bz` — grow the bounding box by a Bézier segment
/// (flat segments expand directly; curved ones split at t=0.5 and recurse).
pub fn update_bb_bz(bb: &mut BoxF, cp: &[PointF; 4]) {
    let inside = |p: PointF| bb.contains(p);
    if inside(cp[0]) && inside(cp[1]) && inside(cp[2]) && inside(cp[3]) {
        return;
    }
    // flatness: distance of control points from the chord
    let pt_to_line2 = |a: PointF, b: PointF, p: PointF| -> f64 {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let a2 = ((p.y - a.y) * dx - (p.x - a.x) * dy).powi(2);
        if a2 < 1e-10 {
            return 0.0;
        }
        a2 / (dx * dx + dy * dy)
    };
    const HW: f64 = 2.0;
    if pt_to_line2(cp[0], cp[3], cp[1]) < HW * HW && pt_to_line2(cp[0], cp[3], cp[2]) < HW * HW {
        for p in cp {
            bb.ll.x = bb.ll.x.min(p.x);
            bb.ll.y = bb.ll.y.min(p.y);
            bb.ur.x = bb.ur.x.max(p.x);
            bb.ur.y = bb.ur.y.max(p.y);
        }
        return;
    }
    let mut left = [PointF::ZERO; 4];
    let mut right = [PointF::ZERO; 4];
    let _mid = bezier_eval(cp, 0.5, Some(&mut left), Some(&mut right));
    update_bb_bz(bb, &left);
    update_bb_bz(bb, &right);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `routespl.c limitBoxes` must sample every point of the spline, not just
    /// its first control point. The port this test was written for updated
    /// only `sp[3]`/`sp[2]`/`sp[1]` in its de Casteljau loop, leaving `sp[0]`
    /// pinned at `pps[splinepi]`; no box except the one containing the start
    /// point was ever sampled, so `routesplines_`'s "bounded" test never
    /// succeeded and every edge paid the full 15 doubling retries (17s on
    /// 1.dot, and bogus box x-extents for the drawn spline).
    #[test]
    fn limit_boxes_samples_the_whole_curve() {
        // Three stacked boxes with disjoint y bands; a monotone cubic rising
        // from y=0 to y=30 sweeps through all three.
        let mut boxes = [
            BoxF::new(PointF::new(DBL_MAX, 0.0), PointF::new(-DBL_MAX, 10.0)),
            BoxF::new(PointF::new(DBL_MAX, 10.0), PointF::new(-DBL_MAX, 20.0)),
            BoxF::new(PointF::new(DBL_MAX, 20.0), PointF::new(-DBL_MAX, 30.0)),
        ];
        let curve = [
            PointF::new(5.0, 0.0),
            PointF::new(5.0, 10.0),
            PointF::new(7.0, 20.0),
            PointF::new(7.0, 30.0),
        ];
        limit_boxes(&mut boxes, &curve, 10.0);
        for (i, b) in boxes.iter().enumerate() {
            assert!(
                b.ll.x != DBL_MAX && b.ur.x != -DBL_MAX,
                "box {i} was never sampled (ll.x={}, ur.x={})",
                b.ll.x,
                b.ur.x
            );
            // every sample lies within x in [5,7]
            assert!(b.ll.x >= 5.0 - 1e-9 && b.ur.x <= 7.0 + 1e-9);
        }
    }

    /// The sampled point must be the curve point at `t`, i.e. for a cubic that
    /// is a straight horizontal segment at y=5 the third box (y in 20..30)
    /// must stay unbounded.
    #[test]
    fn limit_boxes_respects_the_y_band() {
        let mut boxes = [
            BoxF::new(PointF::new(DBL_MAX, 0.0), PointF::new(-DBL_MAX, 10.0)),
            BoxF::new(PointF::new(DBL_MAX, 20.0), PointF::new(-DBL_MAX, 30.0)),
        ];
        let curve = [
            PointF::new(5.0, 5.0),
            PointF::new(6.0, 5.0),
            PointF::new(7.0, 5.0),
            PointF::new(8.0, 5.0),
        ];
        limit_boxes(&mut boxes, &curve, 10.0);
        assert_eq!(boxes[0].ll.x, 5.0);
        assert_eq!(boxes[0].ur.x, 8.0);
        assert_eq!(boxes[1].ll.x, DBL_MAX);
        assert_eq!(boxes[1].ur.x, -DBL_MAX);
    }
}
