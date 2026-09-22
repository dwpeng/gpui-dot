//! Text-aware measurement feeding the layout.
//!
//! Node and edge labels are shaped with the app's text system — the same
//! shaping the canvas paints with — and handed to [`crate::dotgen`] as label
//! extents in points. The engine then applies Graphviz' own sizing rules
//! (`poly_init`: label + padding, attribute lower bounds, shape fits), so the
//! laid-out geometry matches the rendered glyphs.

use gpui_kit::{Hsla, SharedString, TextRun, WindowTextSystem, font, px};

use crate::dotgen::Measured;
use crate::graph::Graph;

/// Graphviz' default font size for node and edge labels, in points.
pub const DEFAULT_FONT_SIZE: f64 = 14.0;
/// `LINESPACING` (const.h) — label line height as a multiple of the font size.
pub const LINE_SPACING: f64 = 1.2;

/// Measures every node and edge label of `graph` at the base font size.
pub fn measure(graph: &Graph, text_system: &WindowTextSystem, family: &str) -> Measured {
    measure_scaled(graph, text_system, family, 1.0)
}

/// Like [`measure`], but at `label_scale`× the base font size — the "Label
/// scale" setting. Every label extent scales by the same factor, so the
/// engine's sizing and the painted glyphs stay in step.
pub fn measure_scaled(
    graph: &Graph,
    text_system: &WindowTextSystem,
    family: &str,
    label_scale: f32,
) -> Measured {
    let scale = label_scale.clamp(0.1, 10.0) as f64;
    let nodes: Vec<(f64, f64)> = graph
        .nodes
        .iter()
        .enumerate()
        .map(|(i, node)| {
            let font_size = font_size_of(&node.attrs, scale);
            text_extent_lines(
                &node_lines(graph, i),
                text_system,
                family,
                font_size,
            )
        })
        .collect();
    let edge_labels: Vec<Option<(f64, f64)>> = graph
        .edges
        .iter()
        .enumerate()
        .map(|(i, edge)| {
            edge.label().map(|_| {
                let font_size = font_size_of(&edge.attrs, scale);
                let lines = graph
                    .edge_label_lines(i)
                    .unwrap_or_default()
                    .into_iter()
                    .map(|l| l.text)
                    .collect::<Vec<_>>();
                text_extent_lines(&lines, text_system, family, font_size)
            })
        })
        .collect();
    // Record field sizing needs a per-string measurer: the engine parses the
    // record label into fields and asks for each one's extent, so `record`
    // nodes are sized from their real glyph widths.
    let family_owned = family.to_string();
    // The measurer outlives this call (it is stored in `Measured`), so it
    // owns its own cheap shaping wrapper around the shared font database.
    let text_system = WindowTextSystem::new(std::sync::Arc::clone(&*text_system));
    let measure = crate::dotgen::TextMeasure(std::rc::Rc::new(move |text: &str| {
        let (w, h) = text_extent(text, &text_system, &family_owned, DEFAULT_FONT_SIZE * scale);
        crate::dotgen::geom::PointF::new(w, h)
    }));
    Measured {
        // The engine sizes boxes itself; `node` stays empty so the label
        // extents drive `poly_init`.
        node: Vec::new(),
        label: nodes,
        edge_label: edge_labels,
        measure: Some(measure),
    }
}

fn font_size_of(attrs: &crate::graph::model::Attributes, scale: f64) -> f64 {
    attrs
        .get("fontsize")
        .and_then(|v| v.trim().parse::<f64>().ok())
        .filter(|v| *v > 0.0)
        .unwrap_or(DEFAULT_FONT_SIZE)
        * scale
}

/// The label's lines, escape-processed (`make_simple_label`).
fn node_lines(graph: &Graph, index: usize) -> Vec<String> {
    graph
        .node_label_lines(index)
        .into_iter()
        .map(|l| l.text)
        .collect()
}

/// The extent of a multi-line label in points: the widest line, and
/// `lines × fontsize × LINESPACING` tall — dot's `textspan` model.
fn text_extent_lines(
    lines: &[String],
    text_system: &WindowTextSystem,
    family: &str,
    font_size: f64,
) -> (f64, f64) {
    let width = lines
        .iter()
        .map(|line| {
            let run = TextRun {
                len: line.len(),
                font: font(family),
                color: Hsla::default(),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            text_system
                .shape_line(SharedString::from(line.clone()), px(font_size as f32), &[run], None)
                .width()
                .as_f32() as f64
        })
        .fold(0.0_f64, f64::max);
    let height = lines.len().max(1) as f64 * font_size * LINE_SPACING;
    (width, height)
}

/// The extent of a (possibly multi-line) label in points: the widest line,
/// and `lines × fontsize × LINESPACING` tall — dot's `textspan` model.
fn text_extent(
    label: &str,
    text_system: &WindowTextSystem,
    family: &str,
    font_size: f64,
) -> (f64, f64) {
    let lines: Vec<&str> = label.split('\n').collect();
    let width = lines
        .iter()
        .map(|line| {
            let run = TextRun {
                len: line.len(),
                font: font(family),
                color: Hsla::default(),
                background_color: None,
                underline: None,
                strikethrough: None,
            };
            text_system
                .shape_line(SharedString::from(*line), px(font_size as f32), &[run], None)
                .width()
                .as_f32() as f64
        })
        .fold(0.0_f64, f64::max);
    let height = lines.len().max(1) as f64 * font_size * LINE_SPACING;
    (width, height)
}
