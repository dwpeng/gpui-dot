//! Graph painting: composes the primitives into the drawn canvas — the
//! background grid, cluster boxes, edges (dot's cubic splines with their real
//! arrowheads and labels), node shapes (polygons or ellipses) with labels, the
//! hover tooltip and the empty-state message. Everything is painted in screen
//! space through the [`ViewTransform`].

use std::rc::Rc;

use gpui_kit::component::ActiveTheme as _;
use gpui_kit::{App, Bounds, Hsla, Pixels, Point, SharedString, Window, point, px, size};

use crate::document::Document;
use crate::viz::dotview::{ArrowPart, NodeShape, ViewEdge, ViewLabel, ViewNode};
use crate::viz::layout::NodeBox;

use super::primitives;
use super::transform::{ViewTransform, grid_step};

/// Hover tooltip (node attributes) geometry, all in screen pixels.
const TOOLTIP_OFFSET: f32 = 12.0;
const TOOLTIP_TITLE_FONT: f32 = 12.5;
const TOOLTIP_ROW_FONT: f32 = 11.5;
const CHIP_PAD_X: f32 = 8.0;
const CHIP_PAD_Y: f32 = 6.0;
const CHIP_MARGIN: f32 = 8.0;

const GRID_OPACITY: f32 = 0.3;
const GRID_STROKE: f32 = 1.0;

/// Screen-pixel margin added around the viewport before culling off-screen
/// graph items, so shapes that only poke into view (labels, wide strokes,
/// arrowheads) are still painted.
const CULL_MARGIN_PX: f32 = 32.0;

/// The fallback color for graph ink when an element carries no `color`.
fn ink(cx: &App) -> Hsla {
    cx.theme().foreground
}

pub(crate) struct Frame<'a> {
    pub document: &'a Rc<Document>,
    /// Live drag override (canvas view state), if any.
    pub node_drag: Option<super::NodeDrag>,
    pub selection: Option<usize>,
    pub hovered: Option<usize>,
    pub mouse: Option<Point<Pixels>>,
    pub show_node_labels: bool,
    pub show_edge_labels: bool,
    pub label_scale: f32,
    pub transform: ViewTransform,
    pub bounds: Bounds<Pixels>,
}

pub(crate) fn graph(frame: &Frame<'_>, window: &mut Window, cx: &mut App) {
    let document = frame.document;
    let transform = frame.transform;
    // Copy the theme colors out up front so `cx` stays free for the text
    // shaping calls below.
    let (popover, border, primary, theme_radius) = {
        let theme = cx.theme();
        (
            theme.popover,
            theme.border,
            theme.primary,
            theme.radius.as_f32(),
        )
    };
    let edge_ink = ink(cx);
    let edge_ink_faded = edge_ink.opacity(0.6);
    // Hairline floor: one *device* pixel. Below that, coverage anti-aliasing
    // turns stroke intensity into a sub-pixel lottery (a line's apparent
    // thickness then depends on where it lands between pixels — some edges
    // look bolder, the short ones fade out entirely), so every zoomed-out
    // edge is clamped to at least a full device pixel instead of 0.75 logical px.
    let hairline = 1.0 / window.scale_factor();

    // A node drag is not committed to `document.offsets` until the button is
    // released, so its live position is merged in here: the dragged node,
    // its label and every edge (with its label) touching it follow the
    // cursor.
    let live_offsets = frame
        .node_drag
        .map(|drag| document.offsets_with_live_drag(Some((drag.node, (drag.pos.x, drag.pos.y)))));
    let offsets: &[(f32, f32)] = live_offsets
        .as_deref()
        .unwrap_or(document.offsets.as_slice());
    // Borrows the routed edges whenever no node is moved; clones them (with
    // the drag re-routing) only while a drag is live.
    let edges = document.view.edges_with_offsets(offsets);

    // The visible world rectangle with a margin, used to cull everything the
    // canvas can't show before any geometry is transformed.
    let margin = CULL_MARGIN_PX / transform.zoom;
    let (wx0, wy0) = transform.to_world(frame.bounds.origin);
    let (wx1, wy1) = transform.to_world(point(
        frame.bounds.origin.x + frame.bounds.size.width,
        frame.bounds.origin.y + frame.bounds.size.height,
    ));
    let visible = (
        wx0.min(wx1) - margin,
        wy0.min(wy1) - margin,
        wx0.max(wx1) + margin,
        wy0.max(wy1) + margin,
    );
    let node_box = |i: usize| -> NodeBox {
        let n = &document.view.nodes[i];
        let (dx, dy) = offsets.get(i).copied().unwrap_or((0.0, 0.0));
        NodeBox {
            x: n.x + dx,
            y: n.y + dy,
            w: n.w,
            h: n.h,
        }
    };

    // Clusters sit behind everything.
    for cluster in &document.view.clusters {
        let (x, y, w, h) = cluster.rect;
        if !overlaps(cluster.rect, visible) {
            continue;
        }
        let bounds = primitives::snap_bounds(
            Bounds {
                origin: transform.to_screen(x, y),
                size: transform.to_screen_size(w, h),
            },
            window.scale_factor(),
        );
        let radius = px(theme_radius * transform.zoom);
        if let Some(fill) = cluster.fill {
            primitives::rounded_rect(window, bounds, radius, fill, None);
        }
        let width = px(1.0);
        if cluster.dashed {
            primitives::dashed_rounded_rect(
                window,
                bounds,
                radius,
                width,
                px(5.0),
                px(4.0),
                cluster.stroke,
            );
        } else {
            primitives::rounded_rect(
                window,
                bounds,
                radius,
                Hsla::transparent_black(),
                Some((cluster.stroke, width)),
            );
        }
        if frame.show_node_labels
            && let Some(label) = &cluster.label
        {
            paint_label(label, frame, None, (0.0, 0.0), window, cx);
        }
    }

    // Edges: dot's cubic splines, then their arrowheads on top. `colors`
    // borrows the edge's parsed colorList (no per-frame clone); the theme ink
    // at edge opacity stands in when the attribute carries no usable color.
    let fallback = [edge_ink_faded];
    for edge in edges.iter() {
        if !overlaps(edge_bounds(edge), visible) {
            continue;
        }
        let width = px((edge.penwidth * transform.zoom).max(hairline));
        let colors: &[Hsla] = if edge.color.a > 0.0 {
            &edge.colors
        } else {
            &fallback
        };
        paint_edge(edge, frame, width, colors, window);
    }
    for edge in edges.iter() {
        if !overlaps(edge_bounds(edge), visible) {
            continue;
        }
        // arrows take the colorList's last color (emit.c)
        let arrow_color = if edge.color.a > 0.0 {
            edge.colors.last().copied().unwrap_or(edge_ink_faded)
        } else {
            edge_ink_faded
        };
        paint_arrows(&edge.arrows, frame, arrow_color, window);
        if frame.show_edge_labels
            && let Some(label) = &edge.label
        {
            paint_label(label, frame, None, (0.0, 0.0), window, cx);
        }
    }

    // Nodes: fill, outline, then the label — the dragged node paints last so
    // it floats above its neighbours while moving.
    for (index, node) in document.view.nodes.iter().enumerate() {
        if frame.node_drag.is_some_and(|drag| drag.node == index) {
            continue;
        }
        let box_ = node_box(index);
        if !overlaps((box_.x, box_.y, box_.w, box_.h), visible) {
            continue;
        }
        paint_node(
            frame, index, node, &box_, popover, border, primary, window, cx,
        );
    }
    if let Some(drag) = frame.node_drag
        && let Some(index) = document.view.nodes.get(drag.node).map(|_| drag.node)
    {
        let box_ = node_box(index);
        if overlaps((box_.x, box_.y, box_.w, box_.h), visible) {
            paint_node(
                frame,
                index,
                &document.view.nodes[index],
                &box_,
                popover,
                border,
                primary,
                window,
                cx,
            );
        }
    }

    // The dragged node's drop preview is gone: the node itself already
    // follows the cursor exactly, so there is nothing to preview.

    if let (Some(index), Some(mouse)) = (frame.hovered, frame.mouse)
        && frame.node_drag.is_none()
    {
        paint_node_tooltip(index, mouse, document, &frame.bounds, window, cx);
    }
}

/// Paints one routed edge: the flattened spline, dashed or solid. A
/// colorList (`color="red:blue"`) paints segment `i` with `colors[i %
/// len]` (emit.c cycles the list along the spline).
fn paint_edge(
    edge: &ViewEdge,
    frame: &Frame<'_>,
    width: Pixels,
    colors: &[Hsla],
    window: &mut Window,
) {
    if edge.segments.is_empty() {
        return;
    }
    let transform = frame.transform;
    // World → screen, transformed lazily per use (no per-edge buffer).
    let seg4 = |s: &[(f32, f32); 4]| -> [Point<Pixels>; 4] {
        [
            transform.to_screen(s[0].0, s[0].1),
            transform.to_screen(s[1].0, s[1].1),
            transform.to_screen(s[2].0, s[2].1),
            transform.to_screen(s[3].0, s[3].1),
        ]
    };
    if edge.dashed || edge.dotted {
        let (dash, gap) = if edge.dotted {
            (width, width * 1.8)
        } else {
            (px(6.0), px(4.0))
        };
        // One pair of scratch buffers for the whole edge: each segment is
        // flattened into them and dash-walked, no per-segment allocation.
        let mut stack: Vec<[(f32, f32); 4]> = Vec::new();
        let mut samples: Vec<(f32, f32)> = Vec::new();
        let mut pts: Vec<Point<Pixels>> = Vec::new();
        for (i, s) in edge.segments.iter().enumerate() {
            let [p0, p1, p2, p3] = seg4(s);
            pts.clear();
            pts.push(p0);
            samples.clear();
            primitives::flatten_cubic_into(
                (p0.x.as_f32(), p0.y.as_f32()),
                (p1.x.as_f32(), p1.y.as_f32()),
                (p2.x.as_f32(), p2.y.as_f32()),
                (p3.x.as_f32(), p3.y.as_f32()),
                primitives::BEZIER_TOLERANCE,
                &mut stack,
                &mut samples,
            );
            pts.extend(samples.iter().map(|&(x, y)| point(px(x), px(y))));
            primitives::stroke_dashed_polyline(
                window,
                &pts,
                width,
                dash,
                gap,
                colors[i % colors.len()],
            );
        }
        return;
    }
    if colors.len() == 1 {
        primitives::stroke_splines(window, edge.segments.iter().map(seg4), width, colors[0]);
        return;
    }
    for (i, s) in edge.segments.iter().enumerate() {
        primitives::stroke_splines(
            window,
            std::iter::once(seg4(s)),
            width,
            colors[i % colors.len()],
        );
    }
}

/// The world-space bounding box of a routed edge: the Bézier control-point
/// hull covers the curve, and the arrowheads sit at its ends.
fn edge_bounds(edge: &ViewEdge) -> (f32, f32, f32, f32) {
    let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for s in &edge.segments {
        for &(x, y) in s {
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
    }
    (x0, y0, x1 - x0, y1 - y0)
}

/// True when a world-space `(x, y, w, h)` box intersects the visible rect.
fn overlaps(box_: (f32, f32, f32, f32), visible: (f32, f32, f32, f32)) -> bool {
    let (x, y, w, h) = box_;
    x <= visible.2 && x + w >= visible.0 && y <= visible.3 && y + h >= visible.1
}

/// Paints dot's arrowhead pieces.
fn paint_arrows(parts: &[ArrowPart], frame: &Frame<'_>, color: Hsla, window: &mut Window) {
    let width = px((1.0 * frame.transform.zoom).max(1.0 / window.scale_factor()));
    for part in parts {
        match part {
            ArrowPart::Polygon(points, filled) => {
                let screen: Vec<Point<Pixels>> = points
                    .iter()
                    .map(|&(x, y)| frame.transform.to_screen(x, y))
                    .collect();
                let poly = primitives::ArrowPoly::new(screen);
                if *filled {
                    poly.fill(window, color);
                } else {
                    poly.outline(window, width, color);
                }
            }
            ArrowPart::Polyline(points) => {
                let segments: Vec<(Point<Pixels>, Point<Pixels>)> = points
                    .windows(2)
                    .map(|w| {
                        (
                            frame.transform.to_screen(w[0].0, w[0].1),
                            frame.transform.to_screen(w[1].0, w[1].1),
                        )
                    })
                    .collect();
                primitives::stroke_segments(window, &segments, width, color);
            }
            ArrowPart::Ellipse(a, b, filled) => {
                let p1 = frame.transform.to_screen(a.0, a.1);
                let p2 = frame.transform.to_screen(b.0, b.1);
                let bounds = Bounds {
                    origin: point(p1.x.min(p2.x), p1.y.min(p2.y)),
                    size: size(
                        (p2.x - p1.x).abs().max(px(1.0)),
                        (p2.y - p1.y).abs().max(px(1.0)),
                    ),
                };
                if *filled {
                    if (bounds.size.width - bounds.size.height).abs() <= px(0.5) {
                        let radius = bounds.size.width.min(bounds.size.height) / 2.0;
                        primitives::rounded_rect(window, bounds, radius, color, None);
                    } else {
                        primitives::fill_ellipse(window, bounds, color);
                    }
                } else {
                    primitives::stroke_ellipse(window, bounds, width, color);
                }
            }
            ArrowPart::Bezier(b) => {
                let screen = [
                    frame.transform.to_screen(b[0].0, b[0].1),
                    frame.transform.to_screen(b[1].0, b[1].1),
                    frame.transform.to_screen(b[2].0, b[2].1),
                    frame.transform.to_screen(b[3].0, b[3].1),
                ];
                let poly = primitives::flatten_cubic(
                    (screen[0].x.as_f32(), screen[0].y.as_f32()),
                    (screen[1].x.as_f32(), screen[1].y.as_f32()),
                    (screen[2].x.as_f32(), screen[2].y.as_f32()),
                    (screen[3].x.as_f32(), screen[3].y.as_f32()),
                    0.35,
                );
                let pts: Vec<Point<Pixels>> = std::iter::once(screen[0])
                    .chain(poly.into_iter().map(|(x, y)| point(px(x), px(y))))
                    .collect();
                let segments: Vec<(Point<Pixels>, Point<Pixels>)> =
                    pts.windows(2).map(|w| (w[0], w[1])).collect();
                primitives::stroke_segments(window, &segments, width, color);
            }
        }
    }
}

/// Draws one node: its shape (fill + outline rings), selection/hover
/// highlight and its label.
#[allow(clippy::too_many_arguments)]
fn paint_node(
    frame: &Frame<'_>,
    index: usize,
    node: &ViewNode,
    box_: &super::layout::NodeBox,
    popover: Hsla,
    border: Hsla,
    primary: Hsla,
    window: &mut Window,
    cx: &mut App,
) {
    let transform = frame.transform;
    let live = frame.transform.to_screen(box_.x, box_.y);
    let rect = Bounds {
        origin: live,
        size: transform.to_screen_size(box_.w, box_.h),
    };
    // The shape outline is stored at the laid-out position; a dragged node is
    // painted at its live box, so shift every world point by the same delta
    // (the transform is uniform, so this is a screen-space translation).
    let shift = transform.to_screen_size(box_.x - node.x, box_.y - node.y);
    let width = px((node.penwidth * transform.zoom).max(1.0 / window.scale_factor()));
    // Hovering a node must not repaint its border — only an explicit
    // selection highlights.
    let highlight = if frame.selection == Some(index) {
        Some(primary)
    } else {
        None
    };

    match &node.shape {
        NodeShape::None => {}
        NodeShape::Rect(rings) => {
            // One rounded-rect quad per periphery ring: fill and border in a
            // single paint_quad (innermost ring only carries the fill).
            for (i, &(rx, ry, rw, rh)) in rings.iter().enumerate() {
                let origin = transform.to_screen(rx, ry);
                let bounds = primitives::snap_bounds(
                    Bounds {
                        origin: point(origin.x + shift.width, origin.y + shift.height),
                        size: transform.to_screen_size(rw, rh),
                    },
                    window.scale_factor(),
                );
                let radius = if node.rounded {
                    // graphviz `style=rounded`: RBCONST(12)·RBCURVE(0.5) world
                    // points, clamped to a sixth of the shorter side.
                    let r = (6.0_f32).min(rw.min(rh) / 6.0);
                    px(r * transform.zoom)
                } else {
                    px(2.0) // plain box: a hair of rounding, screen px
                }
                .min(bounds.size.width / 2.0)
                .min(bounds.size.height / 2.0);
                let fill = if i == 0 { node.fill } else { None };
                let stroke = if highlight.is_some() && i + 1 == rings.len() {
                    highlight.unwrap_or(node.stroke)
                } else {
                    node.stroke
                };
                if node.dashed || node.dotted {
                    if let Some(fill) = fill {
                        primitives::rounded_rect(window, bounds, radius, fill, None);
                    }
                    let (dash, gap) = if node.dotted {
                        (width, width * 1.8)
                    } else {
                        (px(5.0), px(4.0))
                    };
                    primitives::dashed_rounded_rect(
                        window, bounds, radius, width, dash, gap, stroke,
                    );
                } else {
                    primitives::rounded_rect(
                        window,
                        bounds,
                        radius,
                        fill.unwrap_or(Hsla::transparent_black()),
                        Some((stroke, width)),
                    );
                }
            }
            let _ = (popover, border);
        }
        NodeShape::Ellipse(rings) => {
            // One true ellipse per periphery ring (innermost first; the node
            // box is the outermost ring). The fill covers the innermost one.
            // A circle (`rx == ry`) paints through a quad instead.
            let center = point(
                live.x + rect.size.width / 2.0,
                live.y + rect.size.height / 2.0,
            );
            for (i, &(rx, ry)) in rings.iter().enumerate() {
                let size = transform.to_screen_size(rx * 2.0, ry * 2.0);
                let ring_bounds = primitives::snap_bounds(
                    Bounds {
                        origin: point(center.x - size.width / 2.0, center.y - size.height / 2.0),
                        size,
                    },
                    window.scale_factor(),
                );
                let is_circle = (size.width - size.height).abs() <= px(0.5);
                let radius = size.width.min(size.height) / 2.0;
                if i == 0
                    && let Some(fill) = node.fill
                {
                    if is_circle {
                        primitives::rounded_rect(window, ring_bounds, radius, fill, None);
                    } else {
                        primitives::fill_ellipse(window, ring_bounds, fill);
                    }
                }
                let stroke = if highlight.is_some() && i + 1 == rings.len() {
                    highlight.unwrap_or(node.stroke)
                } else {
                    node.stroke
                };
                if node.dashed || node.dotted {
                    let (dash, gap) = if node.dotted {
                        (width, width * 1.8)
                    } else {
                        (px(5.0), px(4.0))
                    };
                    primitives::stroke_dashed_ellipse(
                        window,
                        ring_bounds,
                        width,
                        dash,
                        gap,
                        stroke,
                    );
                } else if is_circle {
                    primitives::rounded_rect(
                        window,
                        ring_bounds,
                        radius,
                        Hsla::transparent_black(),
                        Some((stroke, width)),
                    );
                } else {
                    primitives::stroke_ellipse(window, ring_bounds, width, stroke);
                }
            }
            let _ = (popover, border);
        }
        NodeShape::Polygons(rings) => {
            // Only the first `peripheries` rings are drawn ink: any extra
            // final ring is the penwidth/2 spline-clip outline, not a border.
            // Polygons render exactly — every corner stays sharp.
            for (i, ring) in rings.iter().enumerate().take(node.peripheries.max(1)) {
                let screen: Vec<Point<Pixels>> = ring
                    .iter()
                    .map(|&(x, y)| {
                        let p = transform.to_screen(x, y);
                        point(p.x + shift.width, p.y + shift.height)
                    })
                    .collect();
                let poly = primitives::NodePoly::new(screen);
                if i == 0
                    && let Some(fill) = node.fill
                {
                    poly.fill(window, fill);
                }
                let stroke = if highlight.is_some() && i + 1 == rings.len() {
                    highlight.unwrap_or(node.stroke)
                } else {
                    node.stroke
                };
                if node.dashed || node.dotted {
                    let (dash, gap) = if node.dotted {
                        (width, width * 1.8)
                    } else {
                        (px(5.0), px(4.0))
                    };
                    poly.outline_dashed(window, width, dash, gap, stroke);
                } else {
                    poly.outline(window, width, stroke);
                }
            }
            let _ = (popover, border);
        }
    }

    // A record draws one label per field (centred in its own box) instead of
    // a single centred label, plus the segments separating the siblings.
    if let Some(record) = &node.record {
        let stroke = highlight.unwrap_or(node.stroke);
        let segments: Vec<(Point<Pixels>, Point<Pixels>)> = record
            .dividers
            .iter()
            .map(|(a, b)| {
                let pa = transform.to_screen(a.0, a.1);
                let pb = transform.to_screen(b.0, b.1);
                (
                    point(pa.x + shift.width, pa.y + shift.height),
                    point(pb.x + shift.width, pb.y + shift.height),
                )
            })
            .collect();
        primitives::stroke_segments(window, &segments, width, stroke);
        if frame.show_node_labels {
            let font_size = label_font_size(frame, index);
            let color = node
                .label
                .as_ref()
                .and_then(|l| l.color)
                .unwrap_or_else(|| cx.theme().foreground);
            for (x, y, w, h, text) in record.fields.iter() {
                if text.is_empty() {
                    continue;
                }
                let line = primitives::shape_line(window, cx, text.clone(), font_size, color);
                let anchor = transform.to_screen(x + w / 2.0, y + h / 2.0);
                let origin = point(
                    anchor.x + shift.width - line.width() / 2.0,
                    anchor.y + shift.height - font_size * 1.2 / 2.0,
                );
                primitives::paint_line(&line, origin, px(font_size.as_f32() * 1.2), window, cx);
            }
        }
    } else if frame.show_node_labels
        && let Some(label) = &node.label
    {
        // The label is stored at the laid-out position; follow the node's
        // live box so it moves with a dragged (or dropped) node.
        paint_label(
            label,
            frame,
            None,
            (box_.x - node.x, box_.y - node.y),
            window,
            cx,
        );
    }
}

/// The node label font size in screen pixels (`label.scale` from settings).
fn label_font_size(frame: &Frame<'_>, _index: usize) -> Pixels {
    px(14.0 * frame.label_scale * frame.transform.zoom)
}

/// Paints a label's lines centered on its anchor point, `offset` being an
/// extra world-space translation (a node's drag offset).
fn paint_label(
    label: &ViewLabel,
    frame: &Frame<'_>,
    color_override: Option<Hsla>,
    offset: (f32, f32),
    window: &mut Window,
    cx: &mut App,
) {
    if label.text.is_empty() {
        return;
    }
    let transform = frame.transform;
    let scale = frame.label_scale;
    let font_size = px(label.font_size * scale * transform.zoom);
    let line_height = px(label.font_size * scale * transform.zoom * 1.2);
    let color = color_override
        .or(label.color)
        .unwrap_or_else(|| cx.theme().foreground);
    let lines: Vec<&str> = label.text.split('\n').collect();
    let shaped: Vec<gpui_kit::ShapedLine> = lines
        .iter()
        .map(|text| primitives::shape_line(window, cx, (*text).into(), font_size, color))
        .collect();
    // `emit_label` (labels.c:245-269): a left/right line is aligned against the
    // label's whole box (its widest line), not its own width.
    let space_width = if label.space > 0.0 {
        label.space * scale * transform.zoom
    } else {
        shaped
            .iter()
            .map(|l| l.width().as_f32())
            .fold(0.0f32, f32::max)
    };
    let total_height = px(line_height.as_f32() * lines.len() as f32);
    let follow = transform.to_screen_size(offset.0, offset.1);
    let anchor = transform.to_screen(label.center.0, label.center.1);
    let anchor = point(anchor.x + follow.width, anchor.y + follow.height);
    let top = anchor.y - total_height / 2.0;
    for (i, line) in shaped.iter().enumerate() {
        let just = label.just.get(i).copied().unwrap_or('n');
        let x = match just {
            'l' => anchor.x - px(space_width) / 2.0,
            'r' => anchor.x + px(space_width) / 2.0 - line.width(),
            _ => anchor.x - line.width() / 2.0,
        };
        let origin = point(x, top + px(line_height.as_f32() * i as f32));
        primitives::paint_line(line, origin, line_height, window, cx);
    }
}

/// Paints the background grid: a world-anchored lattice of adaptive-spaced
/// hairline lines covering the visible canvas, behind the graph. Line
/// positions sit on world multiples of the step, so they stay put while
/// panning and shift between powers-of-two spacings while zooming.
pub(crate) fn grid(
    transform: ViewTransform,
    bounds: &Bounds<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    let step = grid_step(transform.zoom);
    let (wx0, wy0) = transform.to_world(bounds.origin);
    let (wx1, wy1) = transform.to_world(point(
        bounds.origin.x + bounds.size.width,
        bounds.origin.y + bounds.size.height,
    ));
    let color = cx.theme().border.opacity(GRID_OPACITY);
    let (cols, rows) = (
        ((wx1 / step).floor() - (wx0 / step).floor()).max(0.0) as usize + 2,
        ((wy1 / step).floor() - (wy0 / step).floor()).max(0.0) as usize + 2,
    );
    let mut segments: Vec<(Point<Pixels>, Point<Pixels>)> = Vec::with_capacity(cols + rows);
    for k in (wx0 / step).floor() as i64..=(wx1 / step).floor() as i64 {
        let x = transform.to_screen(k as f32 * step, wy0).x;
        segments.push((
            point(x, bounds.origin.y),
            point(x, bounds.origin.y + bounds.size.height),
        ));
    }
    for k in (wy0 / step).floor() as i64..=(wy1 / step).floor() as i64 {
        let y = transform.to_screen(wx0, k as f32 * step).y;
        segments.push((
            point(bounds.origin.x, y),
            point(bounds.origin.x + bounds.size.width, y),
        ));
    }
    primitives::stroke_segments(window, &segments, px(GRID_STROKE), color);
}

/// A message centered on the empty canvas (hint or error text).
pub(crate) fn centered_message(
    message: &SharedString,
    bounds: &Bounds<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    let font_size = px(15.0);
    let line_height = px(font_size.as_f32() * 1.4);
    let color = cx.theme().muted_foreground;
    let line = primitives::shape_line(window, cx, message.clone(), font_size, color);
    let origin = point(
        bounds.origin.x + px((bounds.size.width - line.width()).as_f32() / 2.0),
        bounds.origin.y + px(bounds.size.height.as_f32() / 2.0) - font_size,
    );
    primitives::paint_line(&line, origin, line_height, window, cx);
}

/// Paints the node-attribute tooltip next to the cursor: the node's label,
/// its DOT id (when distinct), and its remaining attributes.
fn paint_node_tooltip(
    index: usize,
    mouse: Point<Pixels>,
    document: &Document,
    bounds: &Bounds<Pixels>,
    window: &mut Window,
    cx: &mut App,
) {
    let theme = cx.theme();
    let node = &document.graph.nodes[index];
    let title = node.label();
    let id_is_title = node.attrs.get("label").is_none_or(|label| label.is_empty());

    let mut lines: Vec<(SharedString, Pixels, Hsla)> = Vec::new();
    if !title.is_empty() {
        lines.push((title.into(), px(TOOLTIP_TITLE_FONT), theme.foreground));
    }
    if !id_is_title {
        lines.push((
            format!("id: {}", node.name).into(),
            px(TOOLTIP_ROW_FONT),
            theme.muted_foreground,
        ));
    }
    const MAX_ATTR_ROWS: usize = 12;
    let attrs: Vec<(&String, &String)> = node
        .attrs
        .iter()
        .filter(|(key, _)| key.as_str() != "label")
        .collect();
    let overflow = attrs.len().saturating_sub(MAX_ATTR_ROWS);
    for (key, value) in attrs.iter().take(MAX_ATTR_ROWS) {
        lines.push((
            format!("{key} = {value}").into(),
            px(TOOLTIP_ROW_FONT),
            theme.muted_foreground,
        ));
    }
    if overflow > 0 {
        lines.push((
            format!("+{overflow} more attribute(s)").into(),
            px(TOOLTIP_ROW_FONT),
            theme.muted_foreground,
        ));
    }
    if lines.is_empty() {
        lines.push((
            SharedString::from(node.name.as_str()),
            px(TOOLTIP_ROW_FONT),
            theme.foreground,
        ));
    }

    let mut shaped: Vec<(gpui_kit::ShapedLine, Pixels)> = Vec::with_capacity(lines.len());
    let mut width = 0.0_f32;
    for (text, font_size, color) in &lines {
        let line = primitives::shape_line(window, cx, text.clone(), *font_size, *color);
        width = width.max(line.width().as_f32());
        shaped.push((line, px(font_size.as_f32() * 1.4)));
    }
    let chip_size = size(
        px(width + CHIP_PAD_X * 2.0),
        px(shaped.iter().map(|(_, h)| h.as_f32()).sum::<f32>() + CHIP_PAD_Y * 2.0),
    );
    let mut origin = point(mouse.x + px(TOOLTIP_OFFSET), mouse.y + px(TOOLTIP_OFFSET));
    if origin.x + chip_size.width > bounds.right() - px(CHIP_MARGIN) {
        origin.x =
            (mouse.x - px(TOOLTIP_OFFSET) - chip_size.width).max(bounds.origin.x + px(CHIP_MARGIN));
    }
    if origin.y + chip_size.height > bounds.bottom() - px(CHIP_MARGIN) {
        origin.y = (mouse.y - px(TOOLTIP_OFFSET) - chip_size.height)
            .max(bounds.origin.y + px(CHIP_MARGIN));
    }
    primitives::rounded_rect(
        window,
        Bounds {
            origin,
            size: chip_size,
        },
        px(theme.radius.as_f32()),
        theme.popover,
        Some((theme.border, px(1.0))),
    );
    let mut y = origin.y + px(CHIP_PAD_Y);
    for (line, line_height) in &shaped {
        primitives::paint_line(
            line,
            point(origin.x + px(CHIP_PAD_X), y),
            *line_height,
            window,
            cx,
        );
        y += *line_height;
    }
}
