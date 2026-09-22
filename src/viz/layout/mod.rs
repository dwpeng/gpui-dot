//! Layout-facing types the rest of the app talks to.
//!
//! The layout engine itself now lives in [`crate::dotgen`] — a port of
//! Graphviz' `dot`. This module keeps the small vocabulary the app shell and
//! interaction layer use (`RankDir`, `NodeBox`) plus the text-measurement
//! step that feeds the engine.

pub mod measure;

use crate::dotgen::DotLayout;
use crate::viz::dotview::DotView;

/// World-unit spacing the background grid steps between (see
/// [`super::transform::grid_step`]).
pub const GRID: f32 = 20.0;

/// Which axis the layers run along.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RankDir {
    /// Layers stack top-to-bottom (DOT default).
    #[default]
    TB,
    /// Layers run left-to-right.
    LR,
}

impl RankDir {
    /// The `rankdir` attribute value.
    pub fn as_dot(self) -> &'static str {
        match self {
            RankDir::TB => "TB",
            RankDir::LR => "LR",
        }
    }

    /// Parses a `rankdir` value; anything unknown falls back to `TB`.
    pub fn parse(value: &str) -> Self {
        match value.to_uppercase().as_str() {
            "LR" => RankDir::LR,
            _ => RankDir::TB,
        }
    }
}

/// World-space geometry of one node (top-left corner + size).
#[derive(Debug, Clone, Copy)]
pub struct NodeBox {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl NodeBox {
    pub fn right(&self) -> f32 {
        self.x + self.w
    }
    pub fn bottom(&self) -> f32 {
        self.y + self.h
    }
    /// True when the point (world coordinates) is inside the box.
    pub fn contains(&self, px: f32, py: f32) -> bool {
        px >= self.x && px <= self.right() && py >= self.y && py <= self.bottom()
    }
}

/// Builds the render view for a finished dot layout.
pub fn view(layout: &DotLayout, graph: &crate::graph::Graph, rank_dir: RankDir) -> DotView {
    DotView::build(layout, graph, rank_dir)
}

/// The tight world-space box around every node box plus every committed drag
/// offset — what "fit to view" frames.
pub fn effective_bounds(view: &DotView, offsets: &[(f32, f32)]) -> (f32, f32, f32, f32) {
    let mut bounds = view.bounds;
    for (i, node) in view.nodes.iter().enumerate() {
        let (dx, dy) = offsets.get(i).copied().unwrap_or((0.0, 0.0));
        bounds.0 = bounds.0.min(node.x + dx);
        bounds.1 = bounds.1.min(node.y + dy);
        bounds.2 = bounds.2.max(node.x + dx + node.w);
        bounds.3 = bounds.3.max(node.y + dy + node.h);
    }
    bounds
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rankdir_parses_case_insensitively() {
        assert_eq!(RankDir::parse("lr"), RankDir::LR);
        assert_eq!(RankDir::parse("TB"), RankDir::TB);
        assert_eq!(RankDir::parse("weird"), RankDir::TB);
    }
}
