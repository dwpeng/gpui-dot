use gpui_kit::component::ActiveTheme as _;
use gpui_kit::{
    App, Bounds, Corners, Edges, Hsla, PathBuilder, Pixels, Point, SharedString, TextAlign, Window,
    px,
};
use gpui_kit::{Background, Font, ShapedLine, Size, point, size};

/// Maximum deviation (screen px) between a sampled chord and the true
/// arc/ellipse it approximates. Tuning this down keeps corners smooth even
/// at high zoom, where a fixed angular step would produce visible facets.
const CHORD_TOLERANCE: f32 = 0.25;
/// Bounds on how many chords a corner arc is split into.
const MIN_ARC_STEPS: usize = 8;
const MAX_ARC_STEPS: usize = 128;
/// Corner fillet radius (screen px) for dashed strokes only: the dash walk
/// rounds hairpin corners by half the pen width so dashes meet cleanly.
/// Node polygons themselves render exactly — sharp edges.
const DASH_CORNER_RADIUS_FACTOR: f32 = 0.5;

/// Number of chords needed to approximate an arc of `sweep` radians and
/// radius `r` so that every chord stays within [`CHORD_TOLERANCE`] of the
/// arc: chord deviation is `r·(1 − cos(θ/2))`, inverted for θ.
fn arc_steps(sweep: f32, r: f32) -> usize {
    if r <= 1e-6 || sweep.abs() <= 1e-9 {
        return 1;
    }
    let ratio = (CHORD_TOLERANCE / r).clamp(0.0, 1.0);
    let theta = 2.0 * ratio.acos();
    let n = (sweep.abs() / theta).ceil() as usize;
    n.clamp(MIN_ARC_STEPS, MAX_ARC_STEPS)
}

/// Snaps a screen-space rectangle onto the device pixel grid (one device
/// pixel = `1 / scale` logical px): the outer corners are rounded to whole
/// device pixels instead of keeping the fractional position a zoom/pan lands
/// them at. Quad borders are shaded with a binary ±0.5px SDF threshold rather
/// than smooth coverage, so a border whose edge sits between two pixel rows
/// paints one row more or less than the opposite edge of the very same rect —
/// the "four sides, four thicknesses" artifact. Snapping the bounds makes
/// every side land exactly on a pixel boundary and render identically.
pub fn snap_bounds(bounds: Bounds<Pixels>, scale: f32) -> Bounds<Pixels> {
    if scale <= 0.0 {
        return bounds;
    }
    let snap = |v: Pixels| -> Pixels { px((v.as_f32() * scale).round() / scale) };
    let x0 = snap(bounds.origin.x);
    let y0 = snap(bounds.origin.y);
    let x1 = snap(bounds.origin.x + bounds.size.width);
    let y1 = snap(bounds.origin.y + bounds.size.height);
    Bounds {
        origin: point(x0, y0),
        size: size(
            (x1 - x0).max(px(1.0 / scale)),
            (y1 - y0).max(px(1.0 / scale)),
        ),
    }
}

pub fn rounded_rect(
    window: &mut Window,
    bounds: Bounds<Pixels>,
    radius: Pixels,
    fill: impl Into<Background>,
    border: Option<(Hsla, Pixels)>,
) {
    let (border_widths, border_color) = border
        .map(|(color, width)| (Edges::all(width), color))
        .unwrap_or((Edges::all(px(0.0)), Hsla::default()));
    window.paint_quad(gpui_kit::PaintQuad {
        bounds,
        corner_radii: Corners::all(radius),
        background: fill.into(),
        border_widths,
        border_color,
        border_style: Default::default(),
    });
}

/// If `points` outline an axis-aligned rectangle — in any order, and with
/// the ring optionally repeating its first point — returns the rectangle's
/// bounds. Used by [`fill_poly`] to paint such polygons directly as quads.
fn rect_bounds_of(points: &[Point<Pixels>]) -> Option<Bounds<Pixels>> {
    let mut unique: Vec<Point<Pixels>> = Vec::with_capacity(points.len());
    for &p in points {
        if !unique.contains(&p) {
            unique.push(p);
        }
    }
    if unique.len() != 4 {
        return None;
    }
    let (mut min_x, mut min_y) = (f32::INFINITY, f32::INFINITY);
    let (mut max_x, mut max_y) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for p in &unique {
        min_x = min_x.min(p.x.as_f32());
        min_y = min_y.min(p.y.as_f32());
        max_x = max_x.max(p.x.as_f32());
        max_y = max_y.max(p.y.as_f32());
    }
    if max_x - min_x <= 1e-3 || max_y - min_y <= 1e-3 {
        return None; // degenerate: zero width or height
    }
    // Every distinct point must sit on one of the bounding box's corners;
    // anything else (a diamond, a pentagon, …) falls back to the path fill.
    const EPS: f32 = 0.5;
    let corners = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ];
    if unique.iter().any(|p| {
        let (x, y) = (p.x.as_f32(), p.y.as_f32());
        !corners
            .iter()
            .any(|&(cx, cy)| (x - cx).abs() <= EPS && (y - cy).abs() <= EPS)
    }) {
        return None;
    }
    Some(Bounds {
        origin: point(px(min_x), px(min_y)),
        size: Size::new(px(max_x - min_x), px(max_y - min_y)),
    })
}

/// A polygon parsed as part of a **node shape** (diamond, pentagon, …).
/// Rendered exactly like an arrow decoration: fills and outline rings keep
/// every corner sharp — no joint smoothing.
pub struct NodePoly {
    points: Vec<Point<Pixels>>,
}

/// A polygon parsed as part of an **arrow or edge decoration** (arrowhead
/// triangles, vee/box arrow shapes, …). Rendered exactly: every corner stays
/// pointy — no joint smoothing.
pub struct ArrowPoly {
    points: Vec<Point<Pixels>>,
}

impl NodePoly {
    pub fn new(points: Vec<Point<Pixels>>) -> Self {
        Self { points }
    }

    /// Fills the polygon, corners sharp.
    pub fn fill(&self, window: &mut Window, color: Hsla) {
        fill_poly(window, &self.points, color);
    }

    /// Strokes the polygon's outline; every corner stays sharp.
    pub fn outline(&self, window: &mut Window, width: Pixels, color: Hsla) {
        stroke_ring(window, &self.points, width, color);
    }

    /// Strokes the polygon's outline with a dash pattern.
    pub fn outline_dashed(
        &self,
        window: &mut Window,
        width: Pixels,
        dash_len: Pixels,
        gap_len: Pixels,
        color: Hsla,
    ) {
        let mut ring = self.points.clone();
        if let Some(first) = ring.first().copied() {
            ring.push(first); // close the loop
        }
        stroke_dashed(window, &ring, width, dash_len, gap_len, color);
    }
}

impl ArrowPoly {
    pub fn new(points: Vec<Point<Pixels>>) -> Self {
        Self { points }
    }

    /// Fills the polygon, corners kept exactly pointy.
    pub fn fill(&self, window: &mut Window, color: Hsla) {
        fill_poly(window, &self.points, color);
    }

    /// Strokes the polygon's outline, corners kept exactly pointy.
    pub fn outline(&self, window: &mut Window, width: Pixels, color: Hsla) {
        stroke_ring(window, &self.points, width, color);
    }
}

/// Shared polygon fill: axis-aligned rectangles go through [`rounded_rect`]
/// (the quad's corners are exact); everything else is tessellated through
/// `paint_path`, corners sharp.
fn fill_poly(window: &mut Window, points: &[Point<Pixels>], color: Hsla) {
    if points.len() < 3 {
        return;
    }
    if let Some(bounds) = rect_bounds_of(points) {
        let radius = bounds.size.width.min(bounds.size.height).min(px(4.0)) * 0.5;
        rounded_rect(window, bounds, radius, color, None);
        return;
    }
    let mut builder = PathBuilder::fill();
    builder.move_to(points[0]);
    for p in &points[1..] {
        builder.line_to(*p);
    }
    builder.close();
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// Shared closed-ring stroke. The ring is traced exactly: corners sharp.
fn stroke_ring(window: &mut Window, ring: &[Point<Pixels>], width: Pixels, color: Hsla) {
    if ring.len() < 2 {
        return;
    }
    // A ring may repeat its first point at the end; strip it so the traced
    // path is a clean loop.
    let closed = ring.first() == ring.last();
    let base: &[Point<Pixels>] = if closed {
        &ring[..ring.len() - 1]
    } else {
        ring
    };
    if base.len() < 2 {
        return;
    }
    let mut builder = PathBuilder::stroke(width);
    builder.move_to(base[0]);
    for p in &base[1..] {
        builder.line_to(*p);
    }
    builder.line_to(base[0]);
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// True when the direction change from `from` to `to` (unit vectors) exceeds
/// `threshold` radians — a corner sharp enough that the stroke's miter join
/// at it would protrude past the vertex as a little spike ("小头头").
fn sharp_turn(from: (f32, f32), to: (f32, f32), threshold: f32) -> bool {
    let cross = from.0 * to.1 - from.1 * to.0;
    let dot = from.0 * to.0 + from.1 * to.1;
    cross.abs().atan2(dot) > threshold
}

/// Appends the rounded replacement for one vertex `v` (between its incoming
/// neighbour `a` and outgoing neighbour `b`) to `out`: the bare vertex when
/// the turn is gentle, or the tangent points plus a sampled arc when the
/// corner is sharp enough to poke out as a pointy angle.
fn fillet_corner(
    out: &mut Vec<Point<Pixels>>,
    v: Point<Pixels>,
    a: Point<Pixels>,
    b: Point<Pixels>,
    radius: f32,
) {
    const SHARP_TURN: f32 = 25.0_f32.to_radians();
    let (ax, ay) = ((v.x - a.x).as_f32(), (v.y - a.y).as_f32());
    let (bx, by) = ((b.x - v.x).as_f32(), (b.y - v.y).as_f32());
    let la = (ax * ax + ay * ay).sqrt();
    let lb = (bx * bx + by * by).sqrt();
    if la < 1e-6 || lb < 1e-6 {
        out.push(v);
        return;
    }
    let (uax, uay) = (ax / la, ay / la);
    let (ubx, uby) = (bx / lb, by / lb);
    if !sharp_turn((uax, uay), (ubx, uby), SHARP_TURN) {
        out.push(v);
        return;
    }
    let theta = sharp_turn_angle((uax, uay), (ubx, uby));
    debug_assert!(theta > SHARP_TURN);
    let half = theta * 0.5;
    let (sin_h, cos_h) = half.sin_cos();
    // Tangent points sit `d` on either side of the vertex; clamp the
    // radius so they stay on both segments.
    let tan_h = sin_h / cos_h;
    let r = radius.min(la * tan_h).min(lb * tan_h);
    let d = r * cos_h / sin_h;
    // Centre sits on the corner bisector (normalised u_next − u_prev).
    let bm = ((ubx - uax).powi(2) + (uby - uay).powi(2)).sqrt();
    let (bcx, bcy) = ((ubx - uax) / bm, (uby - uay) / bm);
    let reach = r / sin_h;
    let (cx_, cy_) = (v.x.as_f32() + bcx * reach, v.y.as_f32() + bcy * reach);
    let (t1x, t1y) = (v.x.as_f32() - uax * d, v.y.as_f32() - uay * d);
    let (t2x, t2y) = (v.x.as_f32() + ubx * d, v.y.as_f32() + uby * d);
    out.push(point(px(t1x), px(t1y)));
    // Sweep from the incoming to the outgoing tangent through the corner.
    let a1 = (t1y - cy_).atan2(t1x - cx_);
    let a2 = (t2y - cy_).atan2(t2x - cx_);
    // Always take the short way around: the true corner sweep is π − θ
    // (< 180°), but the raw `a2 − a1` can wrap past ±π when the arc crosses
    // the branch cut (e.g. a rect ring's first vertex, dot's top-left
    // corner) — sampling that raw difference loops 270° the wrong way and
    // sprouts a bulge out of the corner.
    let mut sweep = a2 - a1;
    if sweep > std::f32::consts::PI {
        sweep -= std::f32::consts::TAU;
    } else if sweep < -std::f32::consts::PI {
        sweep += std::f32::consts::TAU;
    }
    let steps = arc_steps(sweep.abs(), r);
    for k in 1..=steps {
        let t = a1 + sweep * (k as f32 / steps as f32);
        out.push(point(px(cx_ + r * t.cos()), px(cy_ + r * t.sin())));
    }
}

/// Absolute turn angle (radians) between two unit directions, in (0, π].
fn sharp_turn_angle(from: (f32, f32), to: (f32, f32)) -> f32 {
    let cross = from.0 * to.1 - from.1 * to.0;
    let dot = from.0 * to.0 + from.1 * to.1;
    cross.abs().atan2(dot)
}

/// Replaces every vertex where the polyline turns more sharply than 25° with
/// a circular fillet of `radius`, tangent to both incident segments. Paths
/// drawn from the result have smooth, 圆滑 corners instead of pointy angles.
/// When `closed`, the ring is treated as cyclic (a repeated closing point is
/// dropped) and every vertex — the seam included — is filleted; the caller
/// re-closes the output.
fn round_joints(points: &[Point<Pixels>], radius: f32, closed: bool) -> Vec<Point<Pixels>> {
    let n = if closed && points.len() > 1 && points.last() == points.first() {
        points.len() - 1
    } else {
        points.len()
    };
    if n < 2 || radius <= 1e-3 {
        return points.to_vec();
    }
    let mut out = Vec::with_capacity(n + 16);
    if closed {
        for i in 0..n {
            let v = points[i];
            let a = points[(i + n - 1) % n];
            let b = points[(i + 1) % n];
            fillet_corner(&mut out, v, a, b, radius);
        }
    } else {
        out.push(points[0]);
        for i in 1..n.saturating_sub(1) {
            fillet_corner(&mut out, points[i], points[i - 1], points[i + 1], radius);
        }
        if n >= 2 {
            out.push(points[n - 1]);
        }
    }
    out
}

fn stroke_dashed(
    window: &mut Window,
    points: &[Point<Pixels>],
    width: Pixels,
    dash_len: Pixels,
    gap_len: Pixels,
    color: Hsla,
) {
    if points.len() < 2 {
        return;
    }
    // Round the sharp corners first so the stroke joins are smooth; closed
    // rings are filleted all around, then re-closed so the dash walk covers
    // the closing edge.
    let closed = points.len() > 1 && points.first() == points.last();
    let mut points = round_joints(points, width.as_f32() * DASH_CORNER_RADIUS_FACTOR, closed);
    if closed {
        points.push(points[0]);
    }
    // A degenerate pattern is a solid line.
    if dash_len.as_f32() <= 0.0 || gap_len.as_f32() <= 0.0 {
        let mut builder = PathBuilder::stroke(width);
        builder.move_to(points[0]);
        for p in &points[1..] {
            builder.line_to(*p);
        }
        if let Ok(path) = builder.build() {
            window.paint_path(path, color);
        }
        return;
    }
    const SNAP: f32 = 0.5;
    let mut builder = PathBuilder::stroke(width);
    let mut pen_down = true;
    let mut remaining = dash_len.as_f32();
    builder.move_to(points[0]);
    for pair in points.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let dx = (b.x - a.x).as_f32();
        let dy = (b.y - a.y).as_f32();
        let seg_len = (dx * dx + dy * dy).sqrt();
        if seg_len < 1e-6 {
            continue;
        }
        let (ux, uy) = (dx / seg_len, dy / seg_len);
        let mut t = 0.0_f32;
        while seg_len - t > remaining {
            let boundary = t + remaining;
            // Snap to the vertex when the toggle would land on (or within
            // half a pixel of) the corner: the current run then ends exactly
            // at the corner instead of a hair-invisible amount past it.
            let snap = boundary >= seg_len - SNAP;
            let p = if snap {
                b
            } else {
                point(a.x + px(ux * boundary), a.y + px(uy * boundary))
            };
            if pen_down {
                builder.line_to(p);
            } else {
                builder.move_to(p);
            }
            pen_down = !pen_down;
            remaining = if pen_down {
                dash_len.as_f32()
            } else {
                gap_len.as_f32()
            };
            t = if snap { seg_len } else { boundary };
        }
        remaining -= seg_len - t;
        if pen_down {
            builder.line_to(b);
        }
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// Builds the closed loop of a rounded rectangle outline. Every corner is a
/// complete quarter circle emitted from its tangency point on the incoming
/// edge to its tangency point on the outgoing edge — never a chopped arc —
/// and every straight side is exactly the chord between two tangency points.
/// The loop therefore has G1-continuous corners, stays inside the rectangle's
/// outer bounds (no protruding geometry) and ends where it starts.
fn rounded_rect_outline(bounds: Bounds<Pixels>, radius: f32) -> Vec<Point<Pixels>> {
    let x = bounds.origin.x.as_f32();
    let y = bounds.origin.y.as_f32();
    let w = bounds.size.width.as_f32();
    let h = bounds.size.height.as_f32();
    let r = radius.min(w / 2.0).min(h / 2.0);
    let mut out = Vec::with_capacity(64);

    // Emit one corner arc, both tangency points included.
    let corner = |out: &mut Vec<Point<Pixels>>, cx: f32, cy: f32, from: f32, to: f32| {
        if r <= 1e-3 {
            out.push(point(px(cx), px(cy))); // sharp corner
            return;
        }
        let sweep = to - from;
        let steps = arc_steps(sweep, r);
        for i in 0..=steps {
            let t = from + sweep * (i as f32 / steps as f32);
            out.push(point(px(cx + r * t.cos()), px(cy + r * t.sin())));
        }
    };

    // Clockwise, starting at the top-right corner's top tangency point:
    // the top edge is the closing chord back to `out[0]`.
    corner(
        &mut out,
        x + w - r,
        y + r,
        -std::f32::consts::FRAC_PI_2,
        0.0,
    );
    corner(
        &mut out,
        x + w - r,
        y + h - r,
        0.0,
        std::f32::consts::FRAC_PI_2,
    );
    corner(
        &mut out,
        x + r,
        y + h - r,
        std::f32::consts::FRAC_PI_2,
        std::f32::consts::PI,
    );
    // Ends exactly at `(x + r, y)`, which the caller closes back to `out[0]`.
    corner(
        &mut out,
        x + r,
        y + r,
        std::f32::consts::PI,
        std::f32::consts::PI + std::f32::consts::FRAC_PI_2,
    );
    out
}

/// Strokes a dashed rounded-rectangle outline — the drop-target indicator the
/// canvas paints while a node is being dragged. `dash_len` / `gap_len` are
/// screen pixels; the stroke width is fixed, independent of zoom.
pub fn dashed_rounded_rect(
    window: &mut Window,
    bounds: Bounds<Pixels>,
    radius: Pixels,
    width: Pixels,
    dash_len: Pixels,
    gap_len: Pixels,
    color: Hsla,
) {
    let mut outline = rounded_rect_outline(bounds, radius.as_f32());
    if let Some(first) = outline.first().copied() {
        outline.push(first); // close the loop
    }
    stroke_dashed(window, &outline, width, dash_len, gap_len, color);
}

/// Strokes *disjoint* line segments — each `(start, end)` pair becomes its
/// own subpath. This is what the background grid and record dividers use:
/// one `PathBuilder`, many `move_to` subpaths. Consecutive segments that
/// share an endpoint are merged into one continuous chain so their joins
/// have no gaps; no joint rounding happens here — node rings get it via
/// [`NodePoly::outline`], arrows and decorations stay exact.
pub fn stroke_segments(
    window: &mut Window,
    segments: &[(Point<Pixels>, Point<Pixels>)],
    width: Pixels,
    color: Hsla,
) {
    if segments.is_empty() {
        return;
    }
    let mut builder = PathBuilder::stroke(width);
    let mut i = 0;
    while i < segments.len() {
        // Merge consecutive segments that share an endpoint into one chain.
        let mut end = i;
        while end + 1 < segments.len() && segments[end].1 == segments[end + 1].0 {
            end += 1;
        }
        let closed = segments[end].1 == segments[i].0;
        if end == i {
            builder.move_to(segments[i].0);
            builder.line_to(segments[i].1);
        } else {
            let mut pts: Vec<Point<Pixels>> = Vec::with_capacity(end - i + 2);
            pts.push(segments[i].0);
            pts.extend(segments[i..=end].iter().map(|s| s.1));
            for (k, p) in pts.iter().enumerate() {
                if k == 0 {
                    builder.move_to(*p);
                } else {
                    builder.line_to(*p);
                }
            }
            if closed {
                builder.line_to(pts[0]);
            }
        }
        i = end + 1;
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// The font for canvas-drawn text: the theme's UI family, which the app
/// points at the embedded MiSans Latin typeface, on top of the window's
/// default text style (sizes, line metrics).
pub fn font(window: &Window, cx: &App) -> Font {
    let mut font = window.text_style().font();
    font.family = cx.theme().font_family.clone();
    font
}

/// Shapes one line of text with the canvas font; the shaped line's
/// [`ShapedLine::width`] is the advance the caller can lay out with.
pub fn shape_line(
    window: &Window,
    cx: &App,
    text: SharedString,
    font_size: Pixels,
    color: Hsla,
) -> ShapedLine {
    let len = text.len();
    let run = gpui_kit::TextRun {
        len,
        font: font(window, cx),
        color,
        background_color: None,
        underline: None,
        strikethrough: None,
    };
    window
        .text_system()
        .shape_line(text, font_size, &[run], None)
}

/// Paints a shaped line with its top-left corner at `origin`.
pub fn paint_line(
    line: &ShapedLine,
    origin: Point<Pixels>,
    line_height: Pixels,
    window: &mut Window,
    cx: &mut App,
) {
    let _ = line.paint(origin, line_height, TextAlign::Left, None, window, cx);
}

/// Flattens one cubic Bézier segment into a polyline whose chords deviate
/// from the true curve by less than `tolerance` screen pixels, using
/// de Casteljau subdivision with the flatness test. [`flatten_cubic_into`]
/// writes into a caller-owned buffer so a long spline is flattened without
/// per-segment allocations.
pub fn flatten_cubic(
    p0: (f32, f32),
    p1: (f32, f32),
    p2: (f32, f32),
    p3: (f32, f32),
    tolerance: f32,
) -> Vec<(f32, f32)> {
    let mut out = Vec::new();
    flatten_cubic_into(p0, p1, p2, p3, tolerance, &mut Vec::new(), &mut out);
    out
}

/// [`flatten_cubic`] into `out`; `stack` is scratch reused across calls (a
/// hot paint pass flattens hundreds of segments into the same buffers).
pub fn flatten_cubic_into(
    p0: (f32, f32),
    p1: (f32, f32),
    p2: (f32, f32),
    p3: (f32, f32),
    tolerance: f32,
    stack: &mut Vec<[(f32, f32); 4]>,
    out: &mut Vec<(f32, f32)>,
) {
    stack.clear();
    stack.push([p0, p1, p2, p3]);
    while let Some([p0, p1, p2, p3]) = stack.pop() {
        // Squared flatness: both "control-point drift" terms together.
        let (ux, uy) = (
            3.0 * p1.0 - 2.0 * p0.0 - p3.0,
            3.0 * p1.1 - 2.0 * p0.1 - p3.1,
        );
        let (vx, vy) = (
            3.0 * p2.0 - 2.0 * p3.0 - p0.0,
            3.0 * p2.1 - 2.0 * p3.1 - p0.1,
        );
        let flatness2 = (ux * ux).max(vx * vx) + (uy * uy).max(vy * vy);
        if flatness2 <= tolerance * tolerance {
            out.push(p3);
            continue;
        }
        let (a, b) = (
            ((p0.0 + p1.0) / 2.0, (p0.1 + p1.1) / 2.0),
            ((p1.0 + p2.0) / 2.0, (p1.1 + p2.1) / 2.0),
        );
        let c = ((p2.0 + p3.0) / 2.0, (p2.1 + p3.1) / 2.0);
        let (d, e) = (
            ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0),
            ((b.0 + c.0) / 2.0, (b.1 + c.1) / 2.0),
        );
        let mid = ((d.0 + e.0) / 2.0, (d.1 + e.1) / 2.0);
        // Push the right half first: the stack is LIFO, so the left half is
        // flattened (and appended) before the right one.
        stack.push([mid, e, c, p3]);
        stack.push([p0, a, d, mid]);
    }
}

/// Chord tolerance (screen px) for flattening routed edge splines.
pub const BEZIER_TOLERANCE: f32 = 0.35;

/// Strokes dot's routed edge splines — each `[p0, p1, p2, p3]` cubic is
/// flattened and appended to one stroked path, so an edge's segments join
/// seamlessly. Takes any iterable of *screen-space* segments, so callers
/// transform world geometry lazily instead of materializing a `Vec` per edge.
pub fn stroke_splines<I>(window: &mut Window, splines: I, width: Pixels, color: Hsla)
where
    I: IntoIterator<Item = [Point<Pixels>; 4]>,
{
    let mut builder = PathBuilder::stroke(width);
    let mut first = true;
    let mut stack: Vec<[(f32, f32); 4]> = Vec::new();
    let mut samples: Vec<(f32, f32)> = Vec::new();
    for [p0, p1, p2, p3] in splines {
        if first {
            builder.move_to(p0);
            first = false;
        }
        // Only this segment's fresh samples: `flatten_cubic_into` appends
        // without clearing, so a stale buffer would replay earlier segments
        // (and draw a stray line back to their start) on every following one.
        samples.clear();
        flatten_cubic_into(
            (p0.x.as_f32(), p0.y.as_f32()),
            (p1.x.as_f32(), p1.y.as_f32()),
            (p2.x.as_f32(), p2.y.as_f32()),
            (p3.x.as_f32(), p3.y.as_f32()),
            BEZIER_TOLERANCE,
            &mut stack,
            &mut samples,
        );
        for &(x, y) in &samples {
            builder.line_to(point(px(x), px(y)));
        }
    }
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// Strokes a polyline with a dash pattern (screen pixels) — the same
/// arc-length walk as the rectangle outlines use, for dashed/dotted edges.
pub fn stroke_dashed_polyline(
    window: &mut Window,
    points: &[Point<Pixels>],
    width: Pixels,
    dash_len: Pixels,
    gap_len: Pixels,
    color: Hsla,
) {
    stroke_dashed(window, points, width, dash_len, gap_len, color);
}

/// Samples the outline of the ellipse (center `(cx, cy)`, radii `rx`/`ry`)
/// so every chord stays within `tolerance` screen pixels of the true curve.
/// Subdivision adapts to local curvature, so a wide flat ellipse is not
/// over-sampled and the sharply curved ends are not under-sampled.
fn sample_ellipse(cx: f32, cy: f32, rx: f32, ry: f32, tolerance: f32) -> Vec<(f32, f32)> {
    const MAX_DEPTH: u32 = 24;
    fn point_at(cx: f32, cy: f32, rx: f32, ry: f32, t: f32) -> (f32, f32) {
        let a = std::f32::consts::TAU * t;
        (cx + rx * a.cos(), cy + ry * a.sin())
    }
    fn subdivide(
        out: &mut Vec<(f32, f32)>,
        cx: f32,
        cy: f32,
        rx: f32,
        ry: f32,
        t0: f32,
        t1: f32,
        depth: u32,
        tolerance: f32,
    ) {
        let tm = (t0 + t1) * 0.5;
        let (x0, y0) = point_at(cx, cy, rx, ry, t0);
        let (x1, y1) = point_at(cx, cy, rx, ry, t1);
        let (xm, ym) = point_at(cx, cy, rx, ry, tm);
        // Deviation of the curve's midpoint from the chord's midpoint.
        let dev = ((xm - (x0 + x1) * 0.5).powi(2) + (ym - (y0 + y1) * 0.5).powi(2)).sqrt();
        if depth >= MAX_DEPTH || dev <= tolerance {
            out.push((x1, y1));
        } else {
            subdivide(out, cx, cy, rx, ry, t0, tm, depth + 1, tolerance);
            subdivide(out, cx, cy, rx, ry, tm, t1, depth + 1, tolerance);
        }
    }
    let mut out = Vec::new();
    out.push(point_at(cx, cy, rx, ry, 0.0));
    subdivide(&mut out, cx, cy, rx, ry, 0.0, 1.0, 0, tolerance);
    out
}

/// Strokes a screen-space ellipse outline (no fill) — the true ellipse
/// inscribed in `bounds`, not a corner-rounded rectangle, so the ring never
/// bulges outward on the flat sides the way a "capsule" would.
pub fn stroke_ellipse(window: &mut Window, bounds: Bounds<Pixels>, width: Pixels, color: Hsla) {
    let w = bounds.size.width.as_f32();
    let h = bounds.size.height.as_f32();
    if w <= 1e-3 || h <= 1e-3 {
        return;
    }
    let (rx, ry) = (w * 0.5, h * 0.5);
    let (cx, cy) = (bounds.origin.x.as_f32() + rx, bounds.origin.y.as_f32() + ry);
    let samples = sample_ellipse(cx, cy, rx, ry, CHORD_TOLERANCE);
    let mut builder = PathBuilder::stroke(width);
    builder.move_to(point(px(samples[0].0), px(samples[0].1)));
    // `samples` ends back at its start; skip the closing duplicate.
    for &(x, y) in samples.iter().take(samples.len() - 1).skip(1) {
        builder.line_to(point(px(x), px(y)));
    }
    builder.close();
    if let Ok(path) = builder.build() {
        window.paint_path(path, color);
    }
}

/// Strokes a dashed ellipse outline: the same adaptive sampling as
/// [`stroke_ellipse`], walked by the shared arc-length dash routine. The
/// dashed variant of a node's ellipse family (dotted doublecircle &c).
pub fn stroke_dashed_ellipse(
    window: &mut Window,
    bounds: Bounds<Pixels>,
    width: Pixels,
    dash_len: Pixels,
    gap_len: Pixels,
    color: Hsla,
) {
    let w = bounds.size.width.as_f32();
    let h = bounds.size.height.as_f32();
    if w <= 1e-3 || h <= 1e-3 {
        return;
    }
    let (rx, ry) = (w * 0.5, h * 0.5);
    let (cx, cy) = (bounds.origin.x.as_f32() + rx, bounds.origin.y.as_f32() + ry);
    let samples = sample_ellipse(cx, cy, rx, ry, CHORD_TOLERANCE);
    let pts: Vec<Point<Pixels>> = samples
        .iter()
        .take(samples.len() - 1) // drop the closing duplicate
        .map(|&(x, y)| point(px(x), px(y)))
        .collect();
    stroke_dashed(window, &pts, width, dash_len, gap_len, color);
}

/// Fills a screen-space ellipse (the true ellipse inscribed in `bounds`,
/// sampled the same way [`stroke_ellipse`] is) — `rounded_rect` with a
/// min-side radius would draw a capsule, not an ellipse.
pub fn fill_ellipse(window: &mut Window, bounds: Bounds<Pixels>, color: Hsla) {
    let w = bounds.size.width.as_f32();
    let h = bounds.size.height.as_f32();
    if w <= 1e-3 || h <= 1e-3 {
        return;
    }
    let (rx, ry) = (w * 0.5, h * 0.5);
    let (cx, cy) = (bounds.origin.x.as_f32() + rx, bounds.origin.y.as_f32() + ry);
    let samples = sample_ellipse(cx, cy, rx, ry, CHORD_TOLERANCE);
    let pts: Vec<Point<Pixels>> = samples
        .iter()
        .take(samples.len() - 1) // drop the closing duplicate
        .map(|&(x, y)| point(px(x), px(y)))
        .collect();
    fill_poly(window, &pts, color);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bezier_flattening_stays_within_the_control_hull() {
        let (p0, p1, p2, p3) = ((0.0, 0.0), (0.0, 100.0), (100.0, 100.0), (100.0, 0.0));
        let pts = flatten_cubic(p0, p1, p2, p3, 0.35);
        assert!(pts.len() > 2, "a curved segment must be subdivided");
        assert_eq!(*pts.last().unwrap(), p3);
        for (x, y) in pts {
            assert!(
                (0.0..=100.0).contains(&x) && (0.0..=100.0).contains(&y),
                "({x},{y})"
            );
        }
    }

    #[test]
    fn straight_segment_flattens_to_one_point() {
        let pts = flatten_cubic((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), 0.35);
        assert_eq!(pts, vec![(3.0, 0.0)]);
    }

    #[test]
    fn rounded_rect_outline_hits_every_tangency_point() {
        let bounds = Bounds {
            origin: point(px(0.0), px(0.0)),
            size: Size::new(px(100.0), px(60.0)),
        };
        let outline = rounded_rect_outline(bounds, 10.0);
        let near = |p: &Point<Pixels>, x: f32, y: f32| {
            (p.x.as_f32() - x).abs() < 1e-3 && (p.y.as_f32() - y).abs() < 1e-3
        };
        // Starts at the top-right corner's top tangency and runs clockwise
        // through every corner tangency, finishing at the top-left's.
        assert!(outline.len() > 16, "must sample the corner arcs");
        assert!(
            near(&outline[0], 90.0, 0.0),
            "start tangency {:?}",
            outline[0]
        );
        for (x, y) in [
            (100.0, 10.0), // top-right corner's right tangency
            (100.0, 50.0), // bottom-right corner's right tangency
            (90.0, 60.0),  // bottom-right corner's bottom tangency
            (10.0, 60.0),  // bottom-left corner's bottom tangency
            (0.0, 50.0),   // bottom-left corner's left tangency
            (0.0, 10.0),   // top-left corner's left tangency
        ] {
            assert!(
                outline.iter().any(|p| near(p, x, y)),
                "missing tangency point ({x}, {y})"
            );
        }
        assert_eq!(*outline.last().unwrap(), point(px(10.0), px(0.0)));
    }

    #[test]
    fn rounded_rect_outline_never_protrudes_the_bounds() {
        let bounds = Bounds {
            origin: point(px(10.0), px(20.0)),
            size: Size::new(px(80.0), px(40.0)),
        };
        for radius in [0.0, 5.0, 20.0] {
            let outline = rounded_rect_outline(bounds, radius);
            for p in &outline {
                assert!(
                    (9.999..=90.001).contains(&p.x.as_f32()),
                    "x={}",
                    p.x.as_f32()
                );
                assert!(
                    (19.999..=60.001).contains(&p.y.as_f32()),
                    "y={}",
                    p.y.as_f32()
                );
            }
        }
    }

    #[test]
    fn rounded_rect_outline_has_no_duplicate_seam() {
        let bounds = Bounds {
            origin: point(px(0.0), px(0.0)),
            size: Size::new(px(100.0), px(60.0)),
        };
        let outline = rounded_rect_outline(bounds, 10.0);
        for pair in outline.windows(2) {
            assert_ne!(pair[0], pair[1], "consecutive duplicate points");
        }
    }

    #[test]
    fn ellipse_sampling_stays_on_the_outline() {
        let (cx, cy, rx, ry) = (50.0f32, 30.0, 100.0, 20.0);
        let samples = sample_ellipse(cx, cy, rx, ry, 0.25);
        assert!(samples.len() > 32, "must subdivide");
        let (first_x, first_y) = samples[0];
        let (last_x, last_y) = *samples.last().unwrap();
        assert!(
            (last_x - first_x).abs() < 1e-3 && (last_y - first_y).abs() < 1e-3,
            "closed loop: start ({first_x}, {first_y}) vs end ({last_x}, {last_y})"
        );
        for &(x, y) in &samples {
            // The parametric point can be a hair outside the axis-aligned
            // bounds only within float error.
            let ex = ((x - cx) / rx).powi(2);
            let ey = ((y - cy) / ry).powi(2);
            assert!(
                ex + ey <= 1.0 + 1e-3,
                "point ({x}, {y}) off the ellipse: {ex} + {ey}"
            );
        }
    }

    #[test]
    fn sharp_turn_detects_corner_spikes() {
        // 90° and hairpin turns must split; gentle bends must not.
        assert!(sharp_turn((1.0, 0.0), (0.0, 1.0), 0.44), "right angle");
        assert!(sharp_turn((1.0, 0.0), (-1.0, 0.0), 0.44), "hairpin");
        assert!(!sharp_turn((1.0, 0.0), (0.985, 0.174), 0.44), "10° bend");
        // Colinear continuation is not a turn at all.
        assert!(!sharp_turn((1.0, 0.0), (1.0, 0.0), 0.0));
    }

    #[test]
    fn round_joints_fillets_sharp_corners() {
        fn near(out: &[Point<Pixels>], x: f32, y: f32) -> bool {
            out.iter()
                .any(|p| (p.x.as_f32() - x).abs() < 1e-2 && (p.y.as_f32() - y).abs() < 1e-2)
        }
        // An L-shape with a 90° corner at (100, 0) and fillet radius 4:
        // the vertex is replaced by an arc running from (96, 0) to (100, 4).
        let pts = Vec::from([
            point(px(0.0), px(0.0)),
            point(px(100.0), px(0.0)),
            point(px(100.0), px(100.0)),
        ]);
        let out = round_joints(&pts, 4.0, false);
        assert!(
            !out.contains(&point(px(100.0), px(0.0))),
            "vertex must be filleted"
        );
        assert!(out.contains(&point(px(96.0), px(0.0))), "incoming tangency");
        assert!(
            out.contains(&point(px(100.0), px(4.0))),
            "outgoing tangency"
        );
        // Mid-arc sample bulges into the corner region (98.83, 1.17).
        assert!(near(&out, 98.83, 1.17), "arc must sweep through the corner");

        // Turns the other way fillet on the other side.
        let pts = Vec::from([
            point(px(0.0), px(0.0)),
            point(px(100.0), px(0.0)),
            point(px(100.0), px(-100.0)),
        ]);
        let out = round_joints(&pts, 4.0, false);
        assert!(out.contains(&point(px(96.0), px(0.0))));
        assert!(out.contains(&point(px(100.0), px(-4.0))));
        assert!(near(&out, 98.83, -1.17));

        // A gentle 10° bend is left untouched (just the vertex passes through).
        let c = 10.0_f32.to_radians();
        let pts = Vec::from([
            point(px(0.0), px(0.0)),
            point(px(100.0), px(0.0)),
            point(px(100.0 + 100.0 * c.cos()), px(100.0 * c.sin())),
        ]);
        assert_eq!(round_joints(&pts, 4.0, false).len(), 3);

        // Zero radius and short inputs pass through unchanged.
        assert_eq!(round_joints(&pts, 0.0, false).len(), 3);
        assert_eq!(round_joints(&pts[..2], 4.0, false).len(), 2);
    }

    #[test]
    fn round_joints_fillets_closed_rings_including_the_seam() {
        // A closed square ring — the repeated closing point included —
        // filleted at radius 4: every corner, the seam vertex included, is
        // replaced by an arc, and the output is an open ring the caller
        // re-closes.
        let mut square = Vec::from([
            point(px(0.0), px(0.0)),
            point(px(100.0), px(0.0)),
            point(px(100.0), px(100.0)),
            point(px(0.0), px(100.0)),
        ]);
        square.push(square[0]);
        let out = round_joints(&square, 4.0, true);
        assert!(
            !out.contains(&point(px(0.0), px(0.0))),
            "seam vertex must be filleted"
        );
        assert!(!out.contains(&point(px(100.0), px(0.0))));
        assert!(!out.contains(&point(px(100.0), px(100.0))));
        assert!(!out.contains(&point(px(0.0), px(100.0))));
        // Every corner contributes both of its tangency points.
        for (x, y) in [
            (96.0, 0.0),
            (100.0, 4.0), // (100, 0)
            (100.0, 96.0),
            (96.0, 100.0), // (100, 100)
            (4.0, 100.0),
            (0.0, 96.0), // (0, 100)
            (0.0, 4.0),
            (4.0, 0.0), // (0, 0) — the seam
        ] {
            assert!(
                out.iter()
                    .any(|p| (p.x.as_f32() - x).abs() < 1e-2 && (p.y.as_f32() - y).abs() < 1e-2),
                "missing tangency point ({x}, {y})"
            );
        }
        // An open call on the same points still treats only interior
        // vertices: both ends survive untouched.
        let open = round_joints(&square[..4], 4.0, false);
        assert_eq!(open[0], point(px(0.0), px(0.0)));
        assert_eq!(*open.last().unwrap(), point(px(0.0), px(100.0)));
    }

    #[test]
    fn fillet_arcs_never_wrap_the_long_way_round() {
        // Regression: the top-left corner of a closed rect ring sits on the
        // atan2 branch cut, so its sweep wrapped to 270° and looped deep
        // into the shape — visible as a bulge ("箭头") sprouting from every
        // node's top-left corner. Every fillet sample must stay inside the
        // border band of width `r` around the square's perimeter.
        let mut square = Vec::from([
            point(px(0.0), px(0.0)),
            point(px(100.0), px(0.0)),
            point(px(100.0), px(100.0)),
            point(px(0.0), px(100.0)),
        ]);
        square.push(square[0]);
        let r = 4.0;
        let out = round_joints(&square, r, true);
        for p in &out {
            let (x, y) = (p.x.as_f32(), p.y.as_f32());
            assert!(
                x <= r + 1e-2 || x >= 100.0 - r - 1e-2 || y <= r + 1e-2 || y >= 100.0 - r - 1e-2,
                "point ({x}, {y}) left the border band — arc wrapped the long way"
            );
        }
    }

    #[test]
    fn rect_detection_finds_axis_aligned_rectangles() {
        let a = point(px(0.0), px(0.0));
        let b = point(px(100.0), px(0.0));
        let c = point(px(100.0), px(50.0));
        let d = point(px(0.0), px(50.0));

        // Any vertex order works…
        let bounds = rect_bounds_of(&Vec::from([c, a, b, d])).expect("rect in any order");
        assert_eq!(bounds.origin, a);
        assert_eq!(bounds.size, Size::new(px(100.0), px(50.0)));

        // …and so does a ring that repeats its first point.
        let mut closed = Vec::from([a, b, c, d]);
        closed.push(a);
        assert!(rect_bounds_of(&closed).is_some(), "closed ring");
    }

    #[test]
    fn rect_detection_rejects_non_rectangles() {
        let a = point(px(0.0), px(0.0));
        let b = point(px(100.0), px(0.0));
        let c = point(px(100.0), px(50.0));

        assert!(rect_bounds_of(&[a, b, c]).is_none(), "triangle");
        assert!(
            rect_bounds_of(&[
                point(px(50.0), px(0.0)),
                point(px(100.0), px(50.0)),
                point(px(50.0), px(100.0)),
                point(px(0.0), px(50.0)),
            ])
            .is_none(),
            "rotated square is not axis-aligned"
        );
        assert!(
            rect_bounds_of(&[
                point(px(0.0), px(0.0)),
                point(px(100.0), px(0.0)),
                point(px(50.0), px(0.0)),
                point(px(80.0), px(0.0)),
            ])
            .is_none(),
            "degenerate zero-height"
        );
    }
}
