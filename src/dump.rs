//! Headless dump mode for the oracle-diff harness: `dotv --dump <file.dot>`
//! parses, lays out and prints JSON — no window, no GPU.

use std::path::Path;

use crate::dotgen::{self, Measured};
use crate::graph::parser;

/// Analytic node size estimate for headless runs (no text system): dot's
/// default box is 0.75×0.5 in; long labels grow the box.
/// The node's label with escapes processed (`\N` etc.) and line breaks turned
/// into real newlines, for the analytic estimator.
fn label_plain(graph: &crate::graph::model::Graph, index: usize, raw: &str) -> String {
    let lines = graph.node_label_lines(index);
    if lines.is_empty() {
        return raw.to_string();
    }
    lines.iter().map(|l| l.text.as_str()).collect::<Vec<_>>().join("\n")
}

/// The edge's label with escapes processed.
fn label_plain_edge(graph: &crate::graph::model::Graph, index: usize, raw: &str) -> String {
    match graph.edge_label_lines(index) {
        Some(lines) if !lines.is_empty() => {
            lines.iter().map(|l| l.text.as_str()).collect::<Vec<_>>().join("\n")
        }
        _ => raw.to_string(),
    }
}

fn font_size(attrs: &crate::graph::model::Attributes) -> f64 {
    attrs
        .get("fontsize")
        .and_then(|v| v.trim().parse::<f64>().ok())
        .filter(|v| *v > 0.0)
        .unwrap_or(14.0)
}

/// Analytic label extents (no text system in headless mode): ~0.5 em per
/// glyph at the label's own font size, and dot's 1.2 line spacing. The engine
/// then applies Graphviz' `poly_init` sizing, so the boxes track real `dot`
/// output closely for Latin labels.
fn estimate_measured(graph: &crate::graph::model::Graph) -> Measured {
    let node = graph
        .nodes
        .iter()
        .enumerate()
        .map(|(i, n)| {
            let text = label_plain(graph, i, &n.label());
            let lines: usize = text.chars().filter(|&c| c == '\n').count() + 1;
            let f = font_size(&n.attrs);
            let w = text
                .split('\n')
                .map(|l| l.chars().count() as f64 * f * 0.5)
                .fold(0.0f64, f64::max);
            (w, lines as f64 * f * 1.2)
        })
        .collect();
    let edge_label = graph
        .edges
        .iter()
        .enumerate()
        .map(|(i, e)| {
            e.label().map(|text| {
                let text = label_plain_edge(graph, i, text);
                let lines: usize = text.matches('\n').count() + 1;
                let f = font_size(&e.attrs);
                let w = text
                    .split('\n')
                    .map(|l| l.chars().count() as f64 * f * 0.5)
                    .fold(0.0f64, f64::max);
                (w, lines as f64 * f * 1.2)
            })
        })
        .collect();
    // Label mode: the engine runs `poly_init` from these text extents.
    Measured {
        node: Vec::new(),
        label: node,
        edge_label,
        measure: None, // analytic record-field sizing inside the engine
    }
}

/// Optional node sizes (inches) taken from `dot -Tplain`: feeding Graphviz'
/// own measurements back in removes font-metric differences, so any remaining
/// positional deviation is an engine difference.
fn read_sizes(path: &Path) -> std::collections::HashMap<String, (f64, f64)> {
    let mut map = std::collections::HashMap::new();
    if let Ok(text) = std::fs::read_to_string(path) {
        for line in text.lines() {
            let p: Vec<&str> = line.split_whitespace().collect();
            if p.first() == Some(&"node") && p.len() >= 6 {
                let name = p[1].trim_matches('"').to_string();
                if let (Ok(w), Ok(h)) = (p[4].parse::<f64>(), p[5].parse::<f64>()) {
                    map.insert(name, (w * 72.0, h * 72.0));
                }
            }
        }
    }
    map
}

pub fn run_with_sizes(path: &Path, sizes: Option<&Path>) {
    let source = match std::fs::read_to_string(path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("dotv --dump: cannot read {}: {e}", path.display());
            std::process::exit(2);
        }
    };
    let graph = match parser::parse(&source) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("dotv --dump: parse error: {e}");
            std::process::exit(3);
        }
    };
    let mut measured = estimate_measured(&graph);
    if let Some(sizes) = sizes {
        let table = read_sizes(sizes);
        if !table.is_empty() {
            // Final-size mode with Graphviz' own box sizes.
            measured.node = graph
                .nodes
                .iter()
                .map(|n| table.get(&n.name).copied().unwrap_or((54.0, 36.0)))
                .collect();
            measured.label = Vec::new();
        }
    }
    let layout = dotgen::layout(&graph, &measured);

    println!("{{");
    println!("  \"directed\": {},", graph.kind == crate::graph::model::GraphKind::Directed);
    println!("  \"nodes\": [");
    for (i, node) in graph.nodes.iter().enumerate() {
        let (x, y) = (layout.coords[i].x, layout.coords[i].y);
        let (w, h) = layout.sizes[i];
        let comma = if i + 1 < graph.nodes.len() { "," } else { "" };
        println!(
            "    {{ \"name\": {:?}, \"rank\": {}, \"x\": {:.3}, \"y\": {:.3}, \"w\": {:.3}, \"h\": {:.3} }}{}",
            node.name, layout.ranks[i], x, y, w, h, comma
        );
    }
    println!("  ],");
    println!("  \"edges\": [");
    for (i, e) in layout.edges.iter().enumerate() {
        let segs: Vec<String> = e
            .segments
            .iter()
            .map(|s| {
                format!(
                    "[[{:.2},{:.2}],[{:.2},{:.2}],[{:.2},{:.2}],[{:.2},{:.2}]]",
                    s[0].x, s[0].y, s[1].x, s[1].y, s[2].x, s[2].y, s[3].x, s[3].y
                )
            })
            .collect();
        let comma = if i + 1 < layout.edges.len() { "," } else { "" };
        println!(
            "    {{ \"tail\": {}, \"head\": {}, \"sflag\": {}, \"eflag\": {}, \"sp\": [{:.2},{:.2}], \"ep\": [{:.2},{:.2}], \"label\": {}, \"label_pos\": [{:.2},{:.2}], \"segs\": [{}] }}{}",
            e.tail, e.head, e.sflag, e.eflag, e.sp.x, e.sp.y, e.ep.x, e.ep.y,
            e.label.as_ref().map(|l| format!("{:?}", l.text)).unwrap_or_else(|| "null".into()),
            e.label.as_ref().map(|l| l.pos.x).unwrap_or(0.0),
            e.label.as_ref().map(|l| l.pos.y).unwrap_or(0.0),
            segs.join(","), comma
        );
    }
    println!("  ]");
    println!("}}");
}
