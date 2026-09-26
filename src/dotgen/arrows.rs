//! Edge arrowheads — a faithful port of `lib/common/arrows.c` plus
//! `arrow_clip`/`bezier_clip` from `lib/common/splines.c`.
//!
//! Flags are packed like C: up to 4 arrowheads × 8 bits; the low 4 bits of
//! each slot are the shape type, the upper 4 the modifiers. Slot 0 is the
//! head closest to the node.

use super::geom::PointF;
use super::model::{DEdge, EId, Fg};

pub const EPSILON: f64 = 0.0001;
pub const ARROW_LENGTH: f64 = 10.0;
const NUMB_OF_ARROW_HEADS: u32 = 4;
const BITS_PER_ARROW: u32 = 8;
const BITS_PER_ARROW_TYPE: u32 = 4;

pub const ARR_TYPE_NONE: u32 = 0;
pub const ARR_TYPE_NORM: u32 = 1;
pub const ARR_TYPE_CROW: u32 = 2;
pub const ARR_TYPE_TEE: u32 = 3;
pub const ARR_TYPE_BOX: u32 = 4;
pub const ARR_TYPE_DIAMOND: u32 = 5;
pub const ARR_TYPE_DOT: u32 = 6;
pub const ARR_TYPE_CURVE: u32 = 7;
pub const ARR_TYPE_GAP: u32 = 8;

const ARR_MOD_OPEN: u32 = 1 << BITS_PER_ARROW_TYPE;
const ARR_MOD_INV: u32 = 1 << (BITS_PER_ARROW_TYPE + 1);
const ARR_MOD_LEFT: u32 = 1 << (BITS_PER_ARROW_TYPE + 2);
const ARR_MOD_RIGHT: u32 = 1 << (BITS_PER_ARROW_TYPE + 3);

const TYPE_MASK: u32 = (1 << BITS_PER_ARROW_TYPE) - 1;
const SLOT_MASK: u32 = (1 << BITS_PER_ARROW) - 1;

/// One rendered arrowhead piece.
#[derive(Debug, Clone)]
pub enum ArrowShape {
    /// Filled or outlined polygon.
    Polygon(Vec<PointF>, bool),
    /// Stroked polyline.
    Polyline(Vec<PointF>),
    /// Ellipse given by opposite corners; filled flag.
    Ellipse(PointF, PointF, bool),
    /// Cubic Bezier (4 control points).
    Bezier([PointF; 4]),
}

/// The pieces of one rendered arrowhead, in paint order.
#[derive(Debug, Clone, Default)]
pub struct ArrowOut {
    pub shapes: Vec<ArrowShape>,
}

fn sub(a: PointF, b: PointF) -> PointF {
    PointF::new(a.x - b.x, a.y - b.y)
}

fn add(a: PointF, b: PointF) -> PointF {
    PointF::new(a.x + b.x, a.y + b.y)
}

fn scale(f: f64, a: PointF) -> PointF {
    PointF::new(a.x * f, a.y * f)
}

fn hypot(x: f64, y: f64) -> f64 {
    x.hypot(y)
}

/// `miter_shape` (arrows.c) — the SVG line-join triangle at a stroke join.
fn miter_shape(base_left: PointF, p: PointF, base_right: PointF, penwidth: f64) -> [PointF; 3] {
    if (base_left.x == p.x && base_left.y == p.y) || (base_right.x == p.x && base_right.y == p.y) {
        return [p, p, p];
    }
    let dx_a = p.x - base_left.x;
    let dy_a = p.y - base_left.y;
    let hypot_a = hypot(dx_a, dy_a);
    let cos_alpha = dx_a / hypot_a;
    let alpha = if dy_a > 0.0 {
        cos_alpha.acos()
    } else {
        -cos_alpha.acos()
    };
    let p1 = PointF::new(
        p.x - penwidth / 2.0 * alpha.sin(),
        p.y + penwidth / 2.0 * cos_alpha,
    );
    let dx_b = base_right.x - p.x;
    let dy_b = base_right.y - p.y;
    let hypot_b = hypot(dx_b, dy_b);
    let cos_beta = dx_b / hypot_b;
    let beta = if dy_b > 0.0 {
        cos_beta.acos()
    } else {
        -cos_beta.acos()
    };
    let beta_rev = beta - std::f64::consts::PI;
    let mut theta = beta_rev - alpha;
    if theta <= -std::f64::consts::PI {
        theta += 2.0 * std::f64::consts::PI;
    }
    debug_assert!((0.0..=std::f64::consts::PI).contains(&theta));
    let stroke_miterlimit = 4.0;
    let normalized_miter_length = 1.0 / (theta / 2.0).sin();
    let sin_beta_minus_pi = -(dy_b / hypot_b);
    let cos_beta_minus_pi = -(dx_b / hypot_b);
    let p2 = PointF::new(
        p.x + penwidth / 2.0 * sin_beta_minus_pi,
        p.y - penwidth / 2.0 * cos_beta_minus_pi,
    );
    if normalized_miter_length > stroke_miterlimit {
        let pbevel = PointF::new((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
        return [pbevel, p1, p2];
    }
    let l = penwidth / 2.0 / (theta / 2.0).tan();
    let p3 = PointF::new(p1.x + l * cos_alpha, p1.y + l * alpha.sin());
    [p3, p1, p2]
}

/// `arrow_type_normal0` — the normal (triangle) arrow; returns the visual
/// start point q and fills the 5-vertex outline.
fn arrow_type_normal0(
    p: PointF,
    u: PointF,
    penwidth: f64,
    flag: u32,
    a: &mut [PointF; 5],
) -> PointF {
    let mut arrowwidth = 0.35;
    if penwidth > 4.0 {
        arrowwidth *= penwidth / 4.0;
    }
    let v = PointF::new(-u.y * arrowwidth, u.x * arrowwidth);
    let mut q = add(p, u);
    let origin = PointF::ZERO;
    let v_inv = scale(-1.0, v);
    let normal_left = if flag & ARR_MOD_RIGHT != 0 {
        origin
    } else {
        v_inv
    };
    let normal_right = if flag & ARR_MOD_LEFT != 0 { origin } else { v };
    let base_left = if flag & ARR_MOD_INV != 0 {
        normal_right
    } else {
        normal_left
    };
    let base_right = if flag & ARR_MOD_INV != 0 {
        normal_left
    } else {
        normal_right
    };
    let normal_tip = scale(-1.0, u);
    let inv_tip = u;
    let pt = if flag & ARR_MOD_INV != 0 {
        inv_tip
    } else {
        normal_tip
    };
    let mut delta_base = PointF::ZERO;
    let mut delta_tip = PointF::ZERO;
    if u.x != 0.0 || u.y != 0.0 {
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        let phi = if pt.y > 0.0 {
            cos_phi.acos()
        } else {
            -cos_phi.acos()
        };
        if flag & ARR_MOD_LEFT != 0 {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            let p1 = shape[1];
            let dx = p1.x - pt.x;
            let dy = p1.y - pt.y;
            let h = hypot(dx, dy);
            let cos_alpha = dx / h;
            let alpha = if dy > 0.0 {
                cos_alpha.acos()
            } else {
                -cos_alpha.acos()
            };
            let gamma = alpha - phi;
            let len = h * gamma.cos();
            delta_tip = PointF::new(len * cos_phi, len * sin_phi);
        } else if flag & ARR_MOD_RIGHT != 0 {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            let p2 = shape[2];
            let dx = p2.x - pt.x;
            let dy = p2.y - pt.y;
            let h = hypot(dx, dy);
            let cos_alpha = dx / h;
            let alpha = if dy > 0.0 {
                cos_alpha.acos()
            } else {
                -cos_alpha.acos()
            };
            let gamma = alpha - phi;
            let len = h * gamma.cos();
            delta_tip = PointF::new(len * cos_phi, len * sin_phi);
        } else {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            delta_tip = sub(shape[0], pt);
        }
        delta_base = PointF::new(penwidth / 2.0 * cos_phi, penwidth / 2.0 * sin_phi);
    }
    let mut p = p;
    if flag & ARR_MOD_INV != 0 {
        p = add(p, delta_base);
        q = add(q, delta_base);
        a[0] = p;
        a[4] = p;
        a[1] = sub(p, v);
        a[2] = q;
        a[3] = add(p, v);
        q = add(q, delta_tip);
    } else {
        p = sub(p, delta_tip);
        q = sub(q, delta_tip);
        a[0] = q;
        a[4] = q;
        a[1] = sub(q, v);
        a[2] = p;
        a[3] = add(q, v);
        q = sub(q, delta_base);
    }
    q
}

/// `arrow_type_crow0` — crow/vee arrow.
fn arrow_type_crow0(
    p: PointF,
    u: PointF,
    arrowsize: f64,
    penwidth: f64,
    flag: u32,
    a: &mut [PointF; 9],
) -> PointF {
    let mut arrowwidth = 0.45;
    if penwidth > 4.0 * arrowsize && flag & ARR_MOD_INV != 0 {
        arrowwidth *= penwidth / (4.0 * arrowsize);
    }
    let mut shaftwidth = 0.0;
    if penwidth > 1.0 && flag & ARR_MOD_INV != 0 {
        shaftwidth = 0.05 * (penwidth - 1.0) / arrowsize;
    }
    let v = PointF::new(-u.y * arrowwidth, u.x * arrowwidth);
    let w = PointF::new(-u.y * shaftwidth, u.x * shaftwidth);
    let mut q = add(p, u);
    let m = PointF::new(p.x + u.x * 0.5, p.y + u.y * 0.5);
    let origin = PointF::ZERO;
    let v_inv = scale(-1.0, v);
    let normal_left = if flag & ARR_MOD_RIGHT != 0 { origin } else { v };
    let normal_right = if flag & ARR_MOD_LEFT != 0 {
        origin
    } else {
        v_inv
    };
    let base_left = if flag & ARR_MOD_INV != 0 {
        normal_right
    } else {
        normal_left
    };
    let base_right = if flag & ARR_MOD_INV != 0 {
        normal_left
    } else {
        normal_right
    };
    let normal_tip = u;
    let inv_tip = scale(-1.0, u);
    let pt = if flag & ARR_MOD_INV != 0 {
        inv_tip
    } else {
        normal_tip
    };
    let mut delta_base = PointF::ZERO;
    let mut delta_tip = PointF::ZERO;
    if u.x != 0.0 || u.y != 0.0 {
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        let phi = if pt.y > 0.0 {
            cos_phi.acos()
        } else {
            -cos_phi.acos()
        };
        if (flag & ARR_MOD_LEFT != 0 && flag & ARR_MOD_INV != 0)
            || (flag & ARR_MOD_RIGHT != 0 && flag & ARR_MOD_INV == 0)
        {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            let p2 = shape[2];
            let dx = p2.x - pt.x;
            let dy = p2.y - pt.y;
            let h = hypot(dx, dy);
            let cos_alpha = dx / h;
            let alpha = if dy > 0.0 {
                cos_alpha.acos()
            } else {
                -cos_alpha.acos()
            };
            let gamma = alpha - phi;
            let len = h * gamma.cos();
            delta_tip = PointF::new(len * cos_phi, len * sin_phi);
        } else if (flag & ARR_MOD_LEFT != 0 && flag & ARR_MOD_INV == 0)
            || (flag & ARR_MOD_RIGHT != 0 && flag & ARR_MOD_INV != 0)
        {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            let p1 = shape[1];
            let dx = p1.x - pt.x;
            let dy = p1.y - pt.y;
            let h = hypot(dx, dy);
            let cos_alpha = dx / h;
            let alpha = if dy > 0.0 {
                cos_alpha.acos()
            } else {
                -cos_alpha.acos()
            };
            let gamma = alpha - phi;
            let len = h * gamma.cos();
            delta_tip = PointF::new(len * cos_phi, len * sin_phi);
        } else {
            let shape = miter_shape(base_left, pt, base_right, penwidth);
            delta_tip = sub(shape[0], pt);
        }
        if flag & ARR_MOD_INV != 0 {
            delta_base = PointF::new(penwidth / 2.0 * cos_phi, penwidth / 2.0 * sin_phi);
        } else {
            let toe_base_left = add(sub(m, q), w);
            let toe_base_right = origin;
            let toe_p = sub(v, u);
            let toe = miter_shape(toe_base_left, toe_p, toe_base_right, penwidth);
            let p1 = toe[1];
            let dx = p1.x - toe_p.x;
            let dy = p1.y - toe_p.y;
            let h = hypot(dx, dy);
            let cos_alpha = dx / h;
            let alpha = if dy > 0.0 {
                cos_alpha.acos()
            } else {
                -cos_alpha.acos()
            };
            let gamma = alpha - phi;
            let len = -h * gamma.cos();
            delta_base = PointF::new(len * cos_phi, len * sin_phi);
        }
    }
    let mut p = p;
    if flag & ARR_MOD_INV != 0 {
        p = sub(p, delta_tip);
        q = sub(q, delta_tip);
        a[0] = p;
        a[8] = p;
        a[1] = sub(q, v);
        a[2] = sub(m, w);
        a[3] = sub(q, w);
        a[4] = q;
        a[5] = add(q, w);
        a[6] = add(m, w);
        a[7] = add(q, v);
        q = sub(q, delta_base);
    } else {
        p = add(p, delta_base);
        q = add(q, delta_base);
        a[0] = q;
        a[8] = q;
        a[1] = sub(p, v);
        a[2] = sub(m, w);
        a[3] = add(p, delta_base);
        a[4] = add(p, delta_base);
        a[5] = add(p, delta_base);
        a[6] = add(m, w);
        a[7] = add(p, v);
        q = add(q, delta_tip);
    }
    q
}

/// `arrow_type_gap` — `none`: a bare 2-point polyline (spacer).
fn arrow_type_gap(p: PointF, u: PointF, out: &mut ArrowOut) -> PointF {
    let q = add(p, u);
    out.shapes.push(ArrowShape::Polyline(vec![p, q]));
    q
}

/// `arrow_type_tee`.
fn arrow_type_tee(p: PointF, u: PointF, penwidth: f64, flag: u32, out: &mut ArrowOut) -> PointF {
    let v = PointF::new(-u.y, u.x);
    let q = add(p, u);
    let m = PointF::new(p.x + u.x * 0.2, p.y + u.y * 0.2);
    let n = PointF::new(p.x + u.x * 0.6, p.y + u.y * 0.6);
    let length = hypot(u.x, u.y);
    let extend = penwidth / 2.0 - 0.2 * length;
    let mut p = p;
    if length > 0.0 && extend > 0.0 {
        let pt = scale(-1.0, u);
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        let delta = PointF::new(extend * cos_phi, extend * sin_phi);
        p = sub(p, delta);
        let m2 = sub(m, delta);
        let n2 = sub(n, delta);
        let q2 = sub(q, delta);
        return tee_polygon(p, m2, n2, q2, v, flag, out);
    }
    tee_polygon(p, m, n, q, v, flag, out)
}

fn tee_polygon(
    p: PointF,
    m: PointF,
    n: PointF,
    q: PointF,
    v: PointF,
    flag: u32,
    out: &mut ArrowOut,
) -> PointF {
    let mut a = [add(m, v), sub(m, v), sub(n, v), add(n, v)];
    if flag & ARR_MOD_LEFT != 0 {
        a[0] = m;
        a[3] = n;
    } else if flag & ARR_MOD_RIGHT != 0 {
        a[1] = m;
        a[2] = n;
    }
    out.shapes.push(ArrowShape::Polygon(a.to_vec(), true));
    out.shapes.push(ArrowShape::Polyline(vec![p, q]));
    q
}

/// `arrow_type_box`.
fn arrow_type_box(p: PointF, u: PointF, penwidth: f64, flag: u32, out: &mut ArrowOut) -> PointF {
    let v = PointF::new(-u.y * 0.4, u.x * 0.4);
    let mut q = add(p, u);
    let m = PointF::new(p.x + u.x * 0.8, p.y + u.y * 0.8);
    let mut delta = PointF::ZERO;
    if u.x != 0.0 || u.y != 0.0 {
        let pt = scale(-1.0, u);
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        delta = PointF::new(penwidth / 2.0 * cos_phi, penwidth / 2.0 * sin_phi);
    }
    let p = sub(p, delta);
    let m = sub(m, delta);
    q = sub(q, delta);
    let mut a = [add(p, v), sub(p, v), sub(m, v), add(m, v)];
    if flag & ARR_MOD_LEFT != 0 {
        a[0] = p;
        a[3] = m;
    } else if flag & ARR_MOD_RIGHT != 0 {
        a[1] = p;
        a[2] = m;
    }
    out.shapes
        .push(ArrowShape::Polygon(a.to_vec(), flag & ARR_MOD_OPEN == 0));
    out.shapes.push(ArrowShape::Polyline(vec![m, q]));
    q
}

/// `arrow_type_diamond0`.
fn arrow_type_diamond0(
    p: PointF,
    u: PointF,
    penwidth: f64,
    flag: u32,
    a: &mut [PointF; 5],
) -> PointF {
    let v = PointF::new(-u.y / 3.0, u.x / 3.0);
    let mut r = PointF::new(p.x + u.x / 2.0, p.y + u.y / 2.0);
    let q0 = add(p, u);
    let origin = PointF::ZERO;
    let unmod_left = sub(scale(-0.5, u), v);
    let unmod_right = add(scale(-0.5, u), v);
    let base_left = if flag & ARR_MOD_RIGHT != 0 {
        origin
    } else {
        unmod_left
    };
    let base_right = if flag & ARR_MOD_LEFT != 0 {
        origin
    } else {
        unmod_right
    };
    let tip = scale(-1.0, u);
    let shape = miter_shape(base_left, tip, base_right, penwidth);
    let delta = sub(shape[0], tip);
    let p = sub(p, delta);
    r = sub(r, delta);
    let q = sub(q0, delta);
    a[0] = q;
    a[4] = q;
    a[1] = add(r, v);
    a[2] = p;
    a[3] = sub(r, v);
    sub(q, delta)
}

/// `arrow_type_dot`.
fn arrow_type_dot(p: PointF, u: PointF, penwidth: f64, flag: u32, out: &mut ArrowOut) -> PointF {
    let r = hypot(u.x, u.y) / 2.0;
    let mut delta = PointF::ZERO;
    let mut p = p;
    if u.x != 0.0 || u.y != 0.0 {
        let pt = scale(-1.0, u);
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        delta = PointF::new(penwidth / 2.0 * cos_phi, penwidth / 2.0 * sin_phi);
        p = sub(p, delta);
    }
    let af0 = PointF::new(p.x + u.x / 2.0 - r, p.y + u.y / 2.0 - r);
    let af1 = PointF::new(p.x + u.x / 2.0 + r, p.y + u.y / 2.0 + r);
    out.shapes
        .push(ArrowShape::Ellipse(af0, af1, flag & ARR_MOD_OPEN == 0));
    let mut q = add(p, u);
    q = sub(q, delta);
    q
}

/// utils.c `Bezier` — de Casteljau evaluation at t, optionally emitting the
/// subdivided left/right control polygons.
pub fn bezier_eval(
    v: &[PointF; 4],
    t: f64,
    left: Option<&mut [PointF; 4]>,
    right: Option<&mut [PointF; 4]>,
) -> PointF {
    // C's Bezier() builds the full de Casteljau triangle Vtemp[i][j], then
    // returns Left[j] = Vtemp[j][0] and Right[j] = Vtemp[degree-j][j].
    let mut vt = [[PointF::ZERO; 4]; 4];
    vt[0] = *v;
    for i in 1..=3 {
        for j in 0..=3 - i {
            vt[i][j] = PointF::new(
                (1.0 - t) * vt[i - 1][j].x + t * vt[i - 1][j + 1].x,
                (1.0 - t) * vt[i - 1][j].y + t * vt[i - 1][j + 1].y,
            );
        }
    }
    if let Some(left) = left {
        for j in 0..=3 {
            left[j] = vt[j][0];
        }
    }
    if let Some(right) = right {
        for j in 0..=3 {
            right[j] = vt[3 - j][j];
        }
    }
    vt[3][0]
}

/// splines.c `bezier_clip` — binary-search the curve for the point where
/// `inside` starts to hold, converging within 0.5 pt.
pub fn bezier_clip<F>(sp: &mut [PointF; 4], inside: F, left_inside: bool)
where
    F: Fn(PointF) -> bool,
{
    let (pt0, idir_is_low);
    if left_inside {
        pt0 = sp[0];
        idir_is_low = true;
    } else {
        pt0 = sp[3];
        idir_is_low = false;
    }
    let mut found = false;
    let (mut low, mut high) = (0.0f64, 1.0f64);
    let mut best = *sp;
    let mut pt = pt0;
    loop {
        let opt = pt;
        let t = (high + low) / 2.0;
        let (mut l, mut r) = ([PointF::ZERO; 4], [PointF::ZERO; 4]);
        let eval = bezier_eval(sp, t, Some(&mut l), Some(&mut r));
        let seg = if left_inside { r } else { l };
        pt = eval;
        if inside(pt) {
            best = seg;
            found = true;
            if idir_is_low {
                low = t;
            } else {
                high = t;
            }
        } else if idir_is_low {
            high = t;
        } else {
            low = t;
        }
        if (opt.x - pt.x).abs() <= 0.5 && (opt.y - pt.y).abs() <= 0.5 {
            break;
        }
    }
    let _ = found;
    *sp = best;
}

fn dist(a: PointF, b: PointF) -> f64 {
    hypot(a.x - b.x, a.y - b.y)
}

/// `arrowEndClip` — pull the spline's head end back by the arrow length.
pub fn arrow_end_clip(
    spl_ep: &mut PointF,
    ps: &mut [PointF],
    startp: usize,
    mut endp: usize,
    _eflag: u32,
    elen: f64,
) -> usize {
    *spl_ep = ps[endp + 3];
    if endp > startp && dist(ps[endp], ps[endp + 3]) < elen {
        endp -= 3;
    }
    // C builds the curve REVERSED so that `sp[0]` is the tip (inside the
    // arrow-length disc) and passes `left_inside = true`; without the
    // reversal the clip keeps the wrong half and the line never pulls back to
    // the arrow's base.
    let mut sp = [*spl_ep, ps[endp + 2], ps[endp + 1], ps[endp]];
    if elen > 0.0 {
        let tip = *spl_ep;
        bezier_clip(&mut sp, |p| dist(p, tip) <= elen, true);
    }
    ps[endp] = sp[3];
    ps[endp + 1] = sp[2];
    ps[endp + 2] = sp[1];
    ps[endp + 3] = sp[0];
    endp
}

/// `arrowStartClip`.
pub fn arrow_start_clip(
    spl_sp: &mut PointF,
    ps: &mut [PointF],
    mut startp: usize,
    endp: usize,
    _sflag: u32,
    slen: f64,
) -> usize {
    *spl_sp = ps[startp];
    if endp > startp && dist(ps[startp], ps[startp + 3]) < slen {
        startp += 3;
    }
    let mut sp = [ps[startp + 3], ps[startp + 2], ps[startp + 1], *spl_sp];
    if slen > 0.0 {
        let tip = *spl_sp;
        bezier_clip(&mut sp, |p| dist(p, tip) <= slen, false);
    }
    // C writes the clipped window back mirrored (sp holds the curve reversed,
    // tip last), so the arrow base lands at `ps[startp]` and the segment keeps
    // its original direction. Writing it back in sp order reversed the whole
    // start window — visible on every layout-reversed back edge.
    ps[startp] = sp[3];
    ps[startp + 1] = sp[2];
    ps[startp + 2] = sp[1];
    ps[startp + 3] = sp[0];
    startp
}

/// `arrow_clip` (splines.c:64) — adjust the spline endpoints for arrows.
/// `fe` is the fast edge; `swap_ends` mirrors `info->swapEnds(e)`;
/// `spline_merge_*` mirror `info->splineMerge(node)`.
#[allow(clippy::too_many_arguments)]
pub fn arrow_clip(
    fg: &Fg,
    fe: EId,
    ps: &mut [PointF],
    startp: &mut usize,
    endp: &mut usize,
    spl_sp: &mut PointF,
    spl_ep: &mut PointF,
    swap_ends: bool,
    spline_merge_tail: bool,
    spline_merge_head: bool,
    is_ortho: bool,
) -> (u32, u32) {
    let e = orig_of(fg, fe);
    let (mut sflag, mut eflag) = resolved_flags(fg, e);
    if spline_merge_head {
        eflag = ARR_TYPE_NONE;
    }
    if spline_merge_tail {
        sflag = ARR_TYPE_NONE;
    }
    let (sflag, eflag) = if swap_ends {
        (eflag, sflag)
    } else {
        (sflag, eflag)
    };
    if is_ortho {
        // ortho routing clips on straight segments only; ported with the
        // splines=ortho phase
        (sflag, eflag)
    } else {
        if sflag != 0 {
            let slen = arrow_length_flags(fg, e, sflag);
            *startp = arrow_start_clip(spl_sp, ps, *startp, *endp, sflag, slen);
        }
        if eflag != 0 {
            let elen = arrow_length_flags(fg, e, eflag);
            *endp = arrow_end_clip(spl_ep, ps, *startp, *endp, eflag, elen);
        }
        (sflag, eflag)
    }
}

fn orig_of(fg: &Fg, mut e: EId) -> EId {
    while let Some(o) = fg.edges[e].to_orig {
        e = o;
    }
    e
}

/// Resolved arrow flags for an original edge, following `arrow_flags`:
/// `dir` picks the base pair, `arrowhead`/`arrowtail` override NORM slots,
/// `conc_opp_flag` ORs in the opposing edge's head flag.
pub fn resolved_flags(fg: &Fg, e: EId) -> (u32, u32) {
    let mut sflag = ARR_TYPE_NONE;
    let mut eflag = ARR_TYPE_NORM;
    let d = &fg.edges[e];
    match d.dir_attr.as_deref() {
        Some("back") => {
            sflag = ARR_TYPE_NORM;
            eflag = ARR_TYPE_NONE;
        }
        Some("both") => {
            sflag = ARR_TYPE_NORM;
            eflag = ARR_TYPE_NORM;
        }
        Some("none") => {
            sflag = ARR_TYPE_NONE;
            eflag = ARR_TYPE_NONE;
        }
        _ => {} // forward (default)
    }
    if eflag == ARR_TYPE_NORM
        && let Some(name) = &d.arrowhead_attr
            && !name.is_empty() {
                eflag = arrow_match_name(name);
            }
    if sflag == ARR_TYPE_NORM
        && let Some(name) = &d.arrowtail_attr
            && !name.is_empty() {
                sflag = arrow_match_name(name);
            }
    if d.conc_opp_flag {
        // pick up arrowhead of opposing edge
        if let Some(opp) = find_opposite(fg, e) {
            let (s0, e0) = resolved_flags(fg, opp);
            eflag |= e0 | s0;
            let _ = s0;
        }
    }
    (sflag, eflag)
}

fn find_opposite(fg: &Fg, e: EId) -> Option<EId> {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    fg.input_out
        .get(h)?
        .iter()
        .copied()
        .find(|&o| fg.edges[o].tail == t && fg.edges[o].head == h)
}

/// `arrow_length` — total length pulled back for the given packed flags.
pub fn arrow_length_flags(fg: &Fg, e: EId, flag: u32) -> f64 {
    let d = &fg.edges[e];
    arrow_pull_len(flag, d.arrowsize, d.penwidth)
}

/// Total spline pull-back for `flag` without a layout context — the same sum
/// [`arrow_length_flags`] computes, for drag-time re-routing where only the
/// edge's own `arrowsize`/`penwidth` are known.
pub fn arrow_pull_len(flag: u32, arrowsize: f64, penwidth: f64) -> f64 {
    if arrowsize == 0.0 {
        return 0.0;
    }
    let mut length = 0.0;
    for i in 0..NUMB_OF_ARROW_HEADS {
        let f = (flag >> (i * BITS_PER_ARROW)) & TYPE_MASK;
        let arrow_flag = (flag >> (i * BITS_PER_ARROW)) & SLOT_MASK;
        match f {
            ARR_TYPE_NORM => length += arrow_length_normal(1.0, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_CROW => length += arrow_length_crow(1.0, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_TEE => length += arrow_length_tee(0.5, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_BOX => length += arrow_length_box(1.0, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_DIAMOND => {
                length += arrow_length_diamond(1.2, arrowsize, penwidth, arrow_flag)
            }
            ARR_TYPE_DOT => length += arrow_length_dot(0.8, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_CURVE => length += arrow_length_curve(1.0, arrowsize, penwidth, arrow_flag),
            ARR_TYPE_GAP => length += arrow_length_generic(0.5, arrowsize, penwidth, arrow_flag),
            _ => {}
        }
    }
    length
}

fn arrow_length_generic(lenfact: f64, arrowsize: f64, _penwidth: f64, _flag: u32) -> f64 {
    lenfact * arrowsize * ARROW_LENGTH
}

fn arrow_length_normal(lenfact: f64, arrowsize: f64, penwidth: f64, flag: u32) -> f64 {
    let mut a = [PointF::ZERO; 5];
    let p = PointF::ZERO;
    let u = PointF::new(lenfact * arrowsize * ARROW_LENGTH, 0.0);
    let q = arrow_type_normal0(p, u, penwidth, flag, &mut a);
    let (base1, base2, tip) = (a[1], a[3], a[2]);
    let full_length = q.x;
    let nominal_length = (base1.x - tip.x).abs();
    let nominal_base_width = base2.y - base1.y;
    let full_base_width = nominal_base_width * full_length / nominal_length;
    let overlap_at_base = penwidth / 2.0;
    let length_where_width_is_penwidth = full_length * penwidth / full_base_width;
    let overlap = if flag & ARR_MOD_INV != 0 {
        length_where_width_is_penwidth
    } else {
        overlap_at_base
    };
    full_length - overlap
}

fn arrow_length_crow(lenfact: f64, arrowsize: f64, penwidth: f64, flag: u32) -> f64 {
    let mut a = [PointF::ZERO; 9];
    let p = PointF::ZERO;
    let u = PointF::new(lenfact * arrowsize * ARROW_LENGTH, 0.0);
    let q = arrow_type_crow0(p, u, arrowsize, penwidth, flag, &mut a);
    let (base1, base2, tip, shaft1) = (a[1], a[7], a[0], a[3]);
    let full_length = q.x;
    let full_length_without_shaft = full_length - (base1.x - shaft1.x);
    let nominal_length = (base1.x - tip.x).abs();
    let nominal_base_width = base2.y - base1.y;
    let full_base_width = nominal_base_width * full_length_without_shaft / nominal_length;
    let overlap_at_base = penwidth / 2.0;
    let length_where_width_is_penwidth = full_length_without_shaft * penwidth / full_base_width;
    let overlap = if flag & ARR_MOD_INV != 0 {
        overlap_at_base
    } else {
        length_where_width_is_penwidth
    };
    full_length - overlap
}

fn arrow_length_tee(lenfact: f64, arrowsize: f64, penwidth: f64, _flag: u32) -> f64 {
    let nominal_length = lenfact * arrowsize * ARROW_LENGTH;
    let mut length = nominal_length;
    let at_start = penwidth / 2.0 - (1.0 - 0.6) * nominal_length;
    if at_start > 0.0 {
        length += at_start;
    }
    // bug-compatible with arrows.c:1243-1251 — the second guard re-tests
    // the *start* condition while adding the *end* extension.
    let at_end = penwidth / 2.0 - 0.2 * nominal_length;
    if at_start > 0.0 {
        length += at_end;
    }
    length
}

fn arrow_length_box(lenfact: f64, arrowsize: f64, penwidth: f64, _flag: u32) -> f64 {
    lenfact * arrowsize * ARROW_LENGTH + penwidth / 2.0
}

fn arrow_length_diamond(lenfact: f64, arrowsize: f64, penwidth: f64, flag: u32) -> f64 {
    let mut a = [PointF::ZERO; 5];
    let p = PointF::ZERO;
    let u = PointF::new(lenfact * arrowsize * ARROW_LENGTH, 0.0);
    let q = arrow_type_diamond0(p, u, penwidth, flag, &mut a);
    let (base1, base2, tip) = (a[3], a[1], a[2]);
    let full_length = q.x / 2.0;
    let nominal_length = (base1.x - tip.x).abs();
    let nominal_base_width = base2.y - base1.y;
    let full_base_width = nominal_base_width * full_length / nominal_length;
    let length_where_width_is_penwidth = full_length * penwidth / full_base_width;
    2.0 * full_length - length_where_width_is_penwidth
}

fn arrow_length_dot(lenfact: f64, arrowsize: f64, penwidth: f64, _flag: u32) -> f64 {
    lenfact * arrowsize * ARROW_LENGTH + penwidth
}

fn arrow_length_curve(lenfact: f64, arrowsize: f64, penwidth: f64, _flag: u32) -> f64 {
    lenfact * arrowsize * ARROW_LENGTH + penwidth / 2.0
}

// -- name resolution ---------------------------------------------------------

const ARROWSYNONYMS: &[(&str, u32)] = &[("invempty", ARR_TYPE_NORM | ARR_MOD_INV | ARR_MOD_OPEN)];

const ARROWMODS: &[(&str, u32)] = &[
    ("o", ARR_MOD_OPEN),
    ("r", ARR_MOD_RIGHT),
    ("l", ARR_MOD_LEFT),
    ("e", ARR_MOD_OPEN),
    ("half", ARR_MOD_LEFT),
];

const ARROWNAMES: &[(&str, u32)] = &[
    ("normal", ARR_TYPE_NORM),
    ("crow", ARR_TYPE_CROW),
    ("tee", ARR_TYPE_TEE),
    ("box", ARR_TYPE_BOX),
    ("diamond", ARR_TYPE_DIAMOND),
    ("dot", ARR_TYPE_DOT),
    ("none", ARR_TYPE_GAP),
    ("inv", ARR_TYPE_NORM | ARR_MOD_INV),
    ("vee", ARR_TYPE_CROW | ARR_MOD_INV),
    ("pen", ARR_TYPE_CROW | ARR_MOD_INV),
    ("mpty", ARR_TYPE_NORM),
    ("curve", ARR_TYPE_CURVE),
    ("icurve", ARR_TYPE_CURVE | ARR_MOD_INV),
];

fn match_frag<'a>(name: &'a str, table: &[(&str, u32)], flag: &mut u32) -> &'a str {
    for (n, f) in table {
        if let Some(rest) = name.strip_prefix(n) {
            *flag |= f;
            return rest;
        }
    }
    name
}

fn arrow_match_shape(name: &str) -> (&str, u32) {
    let mut flag = ARR_TYPE_NONE;
    let mut rest = match_frag(name, ARROWSYNONYMS, &mut flag);
    if rest == name {
        loop {
            let next = rest;
            rest = match_frag(next, ARROWMODS, &mut flag);
            if next == rest {
                break;
            }
        }
        rest = match_frag(rest, ARROWNAMES, &mut flag);
    }
    if flag != 0 && flag & TYPE_MASK == 0 {
        flag |= ARR_TYPE_NORM;
    }
    (rest, flag)
}

/// `arrow_match_name` — parse a possibly multi-head arrow name.
pub fn arrow_match_name(name: &str) -> u32 {
    let mut packed = 0u32;
    let mut rest = name;
    let mut i = 0;
    while !rest.is_empty() && i < NUMB_OF_ARROW_HEADS {
        let (next_rest, mut f) = arrow_match_shape(rest);
        if f == ARR_TYPE_NONE {
            // "Arrow type unknown - ignoring"
            return 0;
        }
        if f == ARR_TYPE_GAP && i == NUMB_OF_ARROW_HEADS - 1 {
            f = ARR_TYPE_NONE;
        }
        if f == ARR_TYPE_GAP && i == 0 && next_rest.is_empty() {
            f = ARR_TYPE_NONE;
        }
        if f != ARR_TYPE_NONE {
            packed |= f << (i * BITS_PER_ARROW);
        }
        rest = next_rest;
        i += 1;
    }
    packed
}

/// `arrow_gen` — render the arrowhead(s) at p pointing toward tip u_target.
pub fn arrow_gen(p: PointF, tip: PointF, arrowsize: f64, penwidth: f64, flag: u32) -> ArrowOut {
    let mut out = ArrowOut::default();
    let mut u = sub(tip, p);
    let s = ARROW_LENGTH / (hypot(u.x, u.y) + EPSILON);
    u.x += if u.x >= 0.0 { EPSILON } else { -EPSILON };
    u.y += if u.y >= 0.0 { EPSILON } else { -EPSILON };
    u.x *= s;
    u.y *= s;
    let mut p = p;
    for i in 0..NUMB_OF_ARROW_HEADS {
        let f = (flag >> (i * BITS_PER_ARROW)) & SLOT_MASK;
        if f == ARR_TYPE_NONE {
            break;
        }
        p = arrow_gen_type(p, u, arrowsize, penwidth, f, &mut out);
    }
    out
}

fn arrow_gen_type(
    p: PointF,
    mut u: PointF,
    arrowsize: f64,
    penwidth: f64,
    flag: u32,
    out: &mut ArrowOut,
) -> PointF {
    let f = flag & TYPE_MASK;
    let (lenfact, genfn): (f64, fn(PointF, PointF, ArrowGenArgs) -> PointF) = match f {
        ARR_TYPE_NORM => (1.0, |p, u, args| {
            let mut a = [PointF::ZERO; 5];
            let q = arrow_type_normal0(p, u, args.penwidth, args.flag, &mut a);
            let poly: Vec<PointF> = if args.flag & ARR_MOD_LEFT != 0 {
                a[..3].to_vec()
            } else if args.flag & ARR_MOD_RIGHT != 0 {
                a[2..].to_vec()
            } else {
                a[1..4].to_vec()
            };
            args.out
                .shapes
                .push(ArrowShape::Polygon(poly, args.flag & ARR_MOD_OPEN == 0));
            q
        }),
        ARR_TYPE_CROW => (1.0, |p, u, args| {
            let mut a = [PointF::ZERO; 9];
            let q = arrow_type_crow0(p, u, args.arrowsize, args.penwidth, args.flag, &mut a);
            let poly: Vec<PointF> = if args.flag & ARR_MOD_LEFT != 0 {
                a[..5].to_vec()
            } else if args.flag & ARR_MOD_RIGHT != 0 {
                a[4..].to_vec()
            } else {
                a[..8].to_vec()
            };
            args.out.shapes.push(ArrowShape::Polygon(poly, true));
            q
        }),
        ARR_TYPE_TEE => (0.5, |p, u, args| {
            arrow_type_tee(p, u, args.penwidth, args.flag, args.out)
        }),
        ARR_TYPE_BOX => (1.0, |p, u, args| {
            arrow_type_box(p, u, args.penwidth, args.flag, args.out)
        }),
        ARR_TYPE_DIAMOND => (1.2, |p, u, args| {
            let mut a = [PointF::ZERO; 5];
            let q = arrow_type_diamond0(p, u, args.penwidth, args.flag, &mut a);
            let poly: Vec<PointF> = if args.flag & ARR_MOD_LEFT != 0 {
                a[2..].to_vec()
            } else if args.flag & ARR_MOD_RIGHT != 0 {
                a[..3].to_vec()
            } else {
                a[..4].to_vec()
            };
            args.out
                .shapes
                .push(ArrowShape::Polygon(poly, args.flag & ARR_MOD_OPEN == 0));
            q
        }),
        ARR_TYPE_DOT => (0.8, |p, u, args| {
            arrow_type_dot(p, u, args.penwidth, args.flag, args.out)
        }),
        ARR_TYPE_CURVE => (1.0, |p, u, args| {
            arrow_type_curve(p, u, args.penwidth, args.flag, args.out)
        }),
        ARR_TYPE_GAP => (0.5, |p, u, args| arrow_type_gap(p, u, args.out)),
        _ => return p,
    };
    u = scale(lenfact * arrowsize, u);
    genfn(
        p,
        u,
        ArrowGenArgs {
            arrowsize,
            penwidth,
            flag,
            out,
        },
    )
}

struct ArrowGenArgs<'o> {
    arrowsize: f64,
    penwidth: f64,
    flag: u32,
    out: &'o mut ArrowOut,
}

/// `arrow_type_curve` — the curved (parenthesis) arrow.
fn arrow_type_curve(p: PointF, u: PointF, penwidth: f64, flag: u32, out: &mut ArrowOut) -> PointF {
    let arrowwidth = if penwidth > 4.0 {
        0.5 * penwidth / 4.0
    } else {
        0.5
    };
    let mut p = p;
    if flag & ARR_MOD_INV == 0 && (u.x != 0.0 || u.y != 0.0) {
        let pt = scale(-1.0, u);
        let cos_phi = pt.x / hypot(pt.x, pt.y);
        let sin_phi = pt.y / hypot(pt.x, pt.y);
        let delta = PointF::new(penwidth / 2.0 * cos_phi, penwidth / 2.0 * sin_phi);
        p = sub(p, delta);
    }
    let q = add(p, u);
    let v = PointF::new(-u.y * arrowwidth, u.x * arrowwidth);
    let w = PointF::new(v.y, -v.x);
    let mut af = [PointF::ZERO; 4];
    af[0] = add(add(p, v), w);
    af[3] = add(sub(p, v), w);
    if flag & ARR_MOD_INV != 0 {
        af[1] = PointF::new(
            p.x + 0.95 * v.x + w.x + w.x * 4.0 / 3.0,
            af[0].y + w.y * 4.0 / 3.0,
        );
        af[2] = PointF::new(
            p.x - 0.95 * v.x + w.x + w.x * 4.0 / 3.0,
            af[3].y + w.y * 4.0 / 3.0,
        );
    } else {
        af[1] = PointF::new(
            p.x + 0.95 * v.x + w.x - w.x * 4.0 / 3.0,
            af[0].y - w.y * 4.0 / 3.0,
        );
        af[2] = PointF::new(
            p.x - 0.95 * v.x + w.x - w.x * 4.0 / 3.0,
            af[3].y - w.y * 4.0 / 3.0,
        );
    }
    out.shapes.push(ArrowShape::Polyline(vec![p, q]));
    let mut curve = af;
    if flag & ARR_MOD_LEFT != 0 {
        let mut left = [PointF::ZERO; 4];
        bezier_eval(&af, 0.5, Some(&mut left), None);
        curve = left;
    } else if flag & ARR_MOD_RIGHT != 0 {
        let mut right = [PointF::ZERO; 4];
        bezier_eval(&af, 0.5, None, Some(&mut right));
        curve = right;
    }
    out.shapes.push(ArrowShape::Bezier(curve));
    q
}

/// Resolves arrow flags at build time for an original edge (kept on DEdge).
pub fn resolve_into(fg: &mut Fg, e: EId) {
    let (sflag, eflag) = resolved_flags(fg, e);
    fg.edges[e].sflag = sflag;
    fg.edges[e].eflag = eflag;
}

#[allow(unused)]
fn dedge_ref(fg: &Fg, e: EId) -> &DEdge {
    &fg.edges[e]
}
