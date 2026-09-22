//! The application document: a loaded [`Graph`] bundled with its computed
//! dot layout (as a render-ready [`DotView`]) and the user's drag offsets.
//! Shared (via [`Rc`]) between the view state and the canvas element, so a
//! snapshot is cheap to clone and rebuild every render.

use std::path::PathBuf;
use std::rc::Rc;

use gpui_kit::WindowTextSystem;

use crate::dotgen;
use crate::fonts::FONT_FAMILY;
use crate::graph::{self, Graph};
use crate::viz::dotview::DotView;
use crate::viz::layout::{self, NodeBox, RankDir};

/// An immutable snapshot of a loaded DOT document.
#[derive(Debug)]
pub struct Document {
    pub path: Option<PathBuf>,
    pub graph: Rc<Graph>,
    /// The dot layout in render form (world coordinates, y down).
    pub view: Rc<DotView>,
    /// Per-node drag offsets from the laid-out positions (parallel to
    /// [`Graph::nodes`], world units, top-left corner). The layout itself
    /// stays as computed; the offsets are what free node dragging commits,
    /// and edge geometry re-anchors to the moved boxes at paint time
    /// ([`DotView::edges_with_offsets`]). A settings-driven re-layout
    /// positions every node from scratch, so it resets these.
    pub offsets: Rc<Vec<(f32, f32)>>,
}

impl Document {
    pub fn file_name(&self) -> String {
        self.path
            .as_deref()
            .and_then(|p| p.file_name())
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Untitled graph".into())
    }

    /// The layer direction the layout was computed with.
    pub fn rank_dir(&self) -> RankDir {
        self.view.rank_dir
    }

    /// The laid-out box of one node, before any drag offset.
    pub fn node_box(&self, index: usize) -> Option<NodeBox> {
        self.view.nodes.get(index).map(|n| NodeBox {
            x: n.x,
            y: n.y,
            w: n.w,
            h: n.h,
        })
    }

    /// The laid-out node boxes with every committed drag offset applied — the
    /// boxes the canvas paints and hit-tests against.
    pub fn effective_boxes(&self) -> Vec<NodeBox> {
        self.effective_boxes_with(&self.offsets)
    }

    /// [`Self::effective_boxes`] with an explicit offset table — paint passes
    /// [`Self::offsets_with_live_drag`] here while a node is being dragged.
    pub fn effective_boxes_with(&self, offsets: &[(f32, f32)]) -> Vec<NodeBox> {
        self.view
            .boxes_with_offsets(offsets)
            .into_iter()
            .map(|(x, y, w, h)| NodeBox { x, y, w, h })
            .collect()
    }

    /// The tight world box covering the laid-out drawing plus drag offsets —
    /// what "fit to view" frames.
    pub fn effective_bounds(&self) -> (f32, f32, f32, f32) {
        layout::effective_bounds(&self.view, &self.offsets)
    }

    /// Returns the document with `node` moved to `offset` — the world-space
    /// position of the node's top-left corner measured from its laid-out
    /// position (the dropped position while dragging). Returns `None` when the
    /// offset would not move the node (a click without a drag) or the node is
    /// out of range.
    pub fn with_node_offset(&self, node: usize, offset: (f32, f32)) -> Option<Document> {
        let current = *self.offsets.get(node)?;
        if (current.0 - offset.0).abs() < 1e-3 && (current.1 - offset.1).abs() < 1e-3 {
            return None;
        }
        let mut offsets = (*self.offsets).clone();
        offsets[node] = offset;
        Some(Document {
            path: self.path.clone(),
            graph: self.graph.clone(),
            view: self.view.clone(),
            offsets: Rc::new(offsets),
        })
    }

    /// The per-node drag offsets with a *live* (uncommitted) node drag merged
    /// in: `drag` is the node index plus the free world position its top-left
    /// corner currently sits at. Painting uses this so the dragged node, its
    /// label and every edge touching it follow the cursor; the committed
    /// offsets only change on release.
    pub fn offsets_with_live_drag(
        &self,
        drag: Option<(usize, (f32, f32))>,
    ) -> Vec<(f32, f32)> {
        let Some((node, pos)) = drag else {
            return (*self.offsets).clone();
        };
        let mut offsets = (*self.offsets).clone();
        if let (Some(base), Some(slot)) = (self.node_box(node), offsets.get_mut(node)) {
            *slot = (pos.0 - base.x, pos.1 - base.y);
        }
        offsets
    }

    /// Returns the document with `node`'s committed drag offset cleared, so it
    /// snaps back to its laid-out position. Returns `None` when the node was
    /// never moved or is out of range (nothing to undo).
    pub fn clear_node_offset(&self, node: usize) -> Option<Document> {
        if self.offsets.get(node).copied() == Some((0.0, 0.0)) || node >= self.offsets.len() {
            return None;
        }
        let mut offsets = (*self.offsets).clone();
        offsets[node] = (0.0, 0.0);
        Some(Document {
            path: self.path.clone(),
            graph: self.graph.clone(),
            view: self.view.clone(),
            offsets: Rc::new(offsets),
        })
    }

    /// Returns the document with *every* committed drag offset cleared, so the
    /// whole drawing snaps back to its computed layout. Returns `None` when no
    /// node has been moved (nothing to undo).
    pub fn clear_offsets(&self) -> Option<Document> {
        if !self.offsets.iter().any(|&(dx, dy)| dx != 0.0 || dy != 0.0) {
            return None;
        }
        Some(Document {
            path: self.path.clone(),
            graph: self.graph.clone(),
            view: self.view.clone(),
            offsets: Rc::new(vec![(0.0, 0.0); self.offsets.len()]),
        })
    }

    /// Returns the document re-measured with the labels at `label_scale`×
    /// their size and re-laid-out in the forced `rank_dir`. The graph and the
    /// path are shared; only the view changes, and drag offsets reset — the
    /// fresh layout positions every node from scratch.
    pub fn relayout(
        &self,
        rank_dir: RankDir,
        label_scale: f32,
        text_system: &WindowTextSystem,
    ) -> Document {
        let graph = self.graph.clone();
        let measured = layout::measure::measure_scaled(&graph, text_system, FONT_FAMILY, label_scale);
        let view = Rc::new(build_view(&graph, &measured, rank_dir));
        let offsets = Rc::new(vec![(0.0, 0.0); graph.nodes.len()]);
        Document {
            path: self.path.clone(),
            graph,
            view,
            offsets,
        }
    }
}

/// Runs the dot engine and builds the render view, honoring a forced
/// direction (the settings "Direction" choice) over the graph's own
/// `rankdir` attribute.
fn build_view(
    graph: &Graph,
    measured: &dotgen::Measured,
    rank_dir: RankDir,
) -> DotView {
    let mut graph_for_layout: Graph = graph.clone();
    graph_for_layout.set_graph_attr("rankdir", rank_dir.as_dot());
    let layout = dotgen::layout(&graph_for_layout, measured);
    layout::view(&layout, graph, rank_dir)
}

/// Reads a DOT file from disk, parses it, measures its labels with the given
/// text system and computes its layout with the dot engine.
pub fn load_document(
    path: impl Into<PathBuf>,
    text_system: &WindowTextSystem,
) -> Result<Document, graph::LoadError> {
    let (path, graph) = graph::load(path)?;
    let graph = Rc::new(graph);
    let rank_dir = RankDir::parse(
        graph
            .graph_attrs()
            .get("rankdir")
            .map(String::as_str)
            .unwrap_or("TB"),
    );
    let measured = layout::measure::measure(&graph, text_system, FONT_FAMILY);
    let view = Rc::new(build_view(&graph, &measured, rank_dir));
    let offsets = Rc::new(vec![(0.0, 0.0); graph.nodes.len()]);
    Ok(Document {
        path: Some(path),
        graph,
        view,
        offsets,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every routed edge must be paintable: finite control points, segments
    /// that share their endpoints (no gaps), and a label that sits on its own
    /// line. This is the regression guard for "some lines are not rendered".
    #[test]
    fn routed_edges_are_all_paintable() {
        // A layered graph with long edges, fan-in, fan-out and labels — the
        // shapes a real build graph produces.
        let mut source = String::from("digraph {\n");
        for layer in 0..6 {
            for i in 0..6 {
                source.push_str(&format!("n{layer}_{i} [label=\"node {layer}.{i}\"];\n"));
                if layer > 0 {
                    source.push_str(&format!(
                        "n{}_{} -> n{layer}_{i} [label=\" e{layer}\"];\n",
                        layer - 1,
                        (i * 5 + layer) % 6
                    ));
                }
            }
        }
        source.push_str("n0_0 -> n5_5;\nn0_1 -> n5_0;\n}\n");
        let graph = Rc::new(graph::parser::parse(&source).expect("parse"));
        let measured = dotgen::Measured {
            label: graph
                .nodes
                .iter()
                .map(|n| (n.label().chars().count() as f64 * 5.0, 14.0))
                .collect(),
            node: Vec::new(),
            edge_label: graph
                .edges
                .iter()
                .map(|e| e.label().map(|l| (l.chars().count() as f64 * 5.0, 11.0)))
                .collect(),
                    measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        assert!(view.edges.len() > 30);
        for (i, edge) in view.edges.iter().enumerate() {
            assert!(!edge.segments.is_empty(), "edge {i} was not routed");
            for seg in &edge.segments {
                for p in seg {
                    assert!(p.0.is_finite() && p.1.is_finite(), "edge {i} not finite");
                }
            }
            for w in edge.segments.windows(2) {
                let a = w[0][3];
                let b = w[1][0];
                let gap = ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
                assert!(gap < 0.01, "edge {i} has a {gap:.3}pt gap between segments");
            }
            if let Some(label) = &edge.label {
                let (mut x0, mut x1, mut y0, mut y1) =
                    (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
                for s in &edge.segments {
                    for p in s {
                        x0 = x0.min(p.0);
                        x1 = x1.max(p.0);
                        y0 = y0.min(p.1);
                        y1 = y1.max(p.1);
                    }
                }
                let pad = 14.0;
                assert!(
                    label.center.0 >= x0 - pad
                        && label.center.0 <= x1 + pad
                        && label.center.1 >= y0 - pad
                        && label.center.1 <= y1 + pad,
                    "edge {i} label {:?} is off its edge",
                    label.text
                );
            }
        }
    }

    /// End-to-end regression for the repo's real `1.dot` sample (whatever the
    /// current file is — it changed from the 125-node ninja graph to the
    /// graphviz color wheel, so expectations are derived, not hardcoded). It
    /// exercises both bugs that made this file take 17s and draw wrong lines:
    ///
    /// * the position-phase network simplex built a *cyclic* tree because
    ///   `inter_tree_edge_search` compared original tight-subtree ids instead
    ///   of live union-find roots — the following `init_cutvalues` DFS then
    ///   never terminated; and
    /// * `limit_boxes` sampled only the first control point, so box reclaim
    ///   failed on every edge and took all 15 doubling retries.
    ///
    /// Both are now fixed, so the assertions below are cheap.
    #[test]
    fn ninja_build_graph_lays_out_and_routes_every_edge() {
        let source = include_str!("../1.dot");
        let graph = Rc::new(graph::parser::parse(source).expect("parse 1.dot"));
        // Every parsed node must keep a unique name (parser dedup by name).
        let unique = graph
            .nodes
            .iter()
            .map(|n| n.name.as_str())
            .collect::<std::collections::HashSet<_>>()
            .len();
        assert_eq!(graph.nodes.len(), unique);
        assert!(!graph.edges.is_empty());
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: graph
                .edges
                .iter()
                .map(|e| e.label().map(|l| (l.chars().count() as f64 * 5.0, 11.0)))
                .collect(),
                    measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::LR);

        assert_eq!(view.edges.len(), graph.edges.len());
        for (i, edge) in view.edges.iter().enumerate() {
            assert!(!edge.segments.is_empty(), "edge {i} was not routed");
            for seg in &edge.segments {
                for p in seg {
                    assert!(p.0.is_finite() && p.1.is_finite(), "edge {i} not finite");
                }
            }
            for w in edge.segments.windows(2) {
                let a = w[0][3];
                let b = w[1][0];
                let gap = ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
                assert!(gap < 0.01, "edge {i} has a {gap:.3}pt gap between segments");
            }
        }
    }

    /// A document laid out from DOT source with analytic (unmeasured) sizes.
    fn document(source: &str) -> Document {
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
                    measure: None,
        };
        let view = Rc::new(build_view(&graph, &measured, RankDir::TB));
        let n = graph.nodes.len();
        Document {
            path: None,
            graph,
            view,
            offsets: Rc::new(vec![(0.0, 0.0); n]),
        }
    }

    #[test]
    fn node_drag_offsets_move_boxes_and_reanchor_edges() {
        let document = document("digraph { a -> b; }");
        let boxes = document.effective_boxes();
        assert_eq!(boxes.len(), 2);
        // A click without a drag commits nothing.
        assert!(document.with_node_offset(1, (0.0, 0.0)).is_none());
        let moved = document
            .with_node_offset(1, (60.0, 20.0))
            .expect("the drop moves the node");
        let effective = moved.effective_boxes();
        assert!((effective[1].x - (boxes[1].x + 60.0)).abs() < 1e-3);
        assert!((effective[1].y - (boxes[1].y + 20.0)).abs() < 1e-3);
        assert!((effective[0].x - boxes[0].x).abs() < 1e-3);
        // The edge touching the moved node is re-routed: it ends at the
        // arrow's *base* just short of the moved box, the arrowhead bridging
        // the gap onto the border.
        let edges = moved.view.edges_with_offsets(&moved.offsets);
        assert_eq!(edges.len(), 1);
        let edge = &edges[0];
        let last = edge.segments.last().expect("segments")[3];
        let pull = edge.arrow_len(edge.eflag) as f32 + 1.0;
        assert!(
            last.0 >= effective[1].x - pull
                && last.0 <= effective[1].right() + pull
                && last.1 >= effective[1].y - pull
                && last.1 <= effective[1].bottom() + pull,
            "head end {last:?} not at the arrow base by the moved box {:?}",
            effective[1]
        );
        // ... and the arrowhead still lands on the border (the apex is set
        // back a couple of pixels by arrows.c's miter delta_tip, hence 2px).
        let crate::viz::dotview::ArrowPart::Polygon(pts, _) = &edge.arrows[0] else {
            panic!("polygon arrowhead expected");
        };
        let apex = pts[1];
        assert!(
            apex.0 >= effective[1].x - 2.0
                && apex.0 <= effective[1].right() + 2.0
                && apex.1 >= effective[1].y - 2.0
                && apex.1 <= effective[1].bottom() + 2.0,
            "head apex {apex:?} not on the moved box {:?}",
            effective[1]
        );
    }

    /// A live (uncommitted) node drag must move the painted boxes and re-anchor
    /// the touching edges without touching the document, so the node follows
    /// the cursor while the button is down; the committed offsets only change
    /// on release.
    #[test]
    fn live_drag_offsets_move_boxes_without_committing() {
        let document = document("digraph { a -> b; }");
        let base = document.effective_boxes();
        let (bx, by) = (base[1].x, base[1].y);

        // No drag in flight: the helper is the identity.
        assert_eq!(document.offsets_with_live_drag(None), *document.offsets);

        let live = document.offsets_with_live_drag(Some((1, (bx + 75.0, by - 40.0))));
        let effective = document.effective_boxes_with(&live);
        assert!((effective[1].x - (bx + 75.0)).abs() < 1e-3);
        assert!((effective[1].y - (by - 40.0)).abs() < 1e-3);
        // The committed document is untouched.
        assert_eq!(*document.offsets, vec![(0.0, 0.0), (0.0, 0.0)]);
        assert_eq!(document.effective_boxes()[1].x, bx);

        // ... and the edge re-anchors to the live box — its arrow base stops
        // just short of the box (the arrowhead bridges the gap) — so the line
        // tracks the dragging node instead of snapping only on release.
        let edges = document.view.edges_with_offsets(&live);
        let edge = &edges[0];
        let last = edge.segments.last().expect("segments")[3];
        let pull = edge.arrow_len(edge.eflag) as f32 + 1.0;
        assert!(
            last.0 >= effective[1].x - pull
                && last.0 <= effective[1].right() + pull
                && last.1 >= effective[1].y - pull
                && last.1 <= effective[1].bottom() + pull,
            "head end {last:?} does not follow the live box {:?}",
            effective[1]
        );
    }

    /// Explicit ports must decide where the spline attaches: `headport=s`
    /// puts the head end on the *south* border of the head node (routed
    /// around it), and `tailport=e` the tail end on the east border.
    #[test]
    fn explicit_ports_decide_the_attachment_side() {
        let source = r#"digraph {
            node [shape=box];
            a -> b [headport=s];
            c -> d [tailport=e];
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        let node_box = |name: &str| {
            let index = graph.nodes.iter().position(|n| n.name == name).expect("node");
            view.nodes[index].clone()
        };
        // headport=s: the routed head end sits below b's centre.
        let b = node_box("b");
        let edge = &view.edges[0];
        let head = edge.segments.last().expect("segments")[3];
        // world coordinates are y-down: south is *below* the centre.
        assert!(
            head.1 > b.y + b.h / 2.0,
            "headport=s must arrive on the south side: head {head:?}, b {b:?}"
        );
        // tailport=e: the routed tail end sits right of c's centre.
        let c = node_box("c");
        let edge = &view.edges[1];
        let tail = edge.segments.first().expect("segments")[0];
        assert!(
            tail.0 > c.x + c.w / 2.0,
            "tailport=e must leave from the east side: tail {tail:?}, c {c:?}"
        );
    }

    /// `newrank=true` (rank.c `dot2_rank`): a `rank=same` set spanning two
    /// clusters is a *global* constraint, so the two cluster heads land in one
    /// rank even though the clusters rank their members locally.
    #[test]
    fn newrank_ranks_across_clusters() {
        let source = r#"digraph {
            newrank=true;
            subgraph cluster_a { a1 -> a2; }
            subgraph cluster_b { b1 -> b2; }
            {rank=same; a1; b1;}
            a2 -> b2;
        }"#;
        let ranks = |src: &str| -> Vec<i32> {
            let graph = graph::parser::parse(src).expect("parse");
            let measured = dotgen::Measured {
                node: vec![(54.0, 36.0); graph.nodes.len()],
                label: Vec::new(),
                edge_label: vec![None; graph.edges.len()],
                measure: None,
            };
            dotgen::layout(&graph, &measured).ranks
        };
        let rank_of = |ranks: &[i32], name: &str| {
            let graph = graph::parser::parse(source).expect("parse");
            let i = graph.nodes.iter().position(|n| n.name == name).expect("node");
            ranks[i]
        };
        let on = ranks(source);
        // The set is honoured globally, so `a2 -> b2` then pushes b2 a rank
        // below a2 (b1 stays with a1).
        assert_eq!(rank_of(&on, "a1"), 0);
        assert_eq!(rank_of(&on, "b1"), 0, "rank=same across clusters");
        assert_eq!(rank_of(&on, "a2"), 1);
        assert_eq!(rank_of(&on, "b2"), 2);
        // Without newrank each cluster ranks locally, so b2 stays with a2.
        let off = ranks(&source.replace("newrank=true;", ""));
        assert_eq!(rank_of(&off, "b2"), rank_of(&off, "a2"));
    }

    /// `newrank=true` must not leak its auxiliary constraint graph into the
    /// real node/edge arena (`Xg` is truncated again in `dot2_rank`).
    #[test]
    fn newrank_leaves_no_auxiliary_nodes() {
        let source = r#"digraph {
            newrank=true;
            subgraph cluster_a { compact=true; a1 -> a2; }
            {rank=same; a1; b1;}
            a2 -> b1;
            b1 -> c;
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let n_edges = graph.edges.len();
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; n_edges],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        assert_eq!(view.nodes.len(), graph.nodes.len(), "no auxiliary nodes survive ranking");
        assert_eq!(view.edges.len(), n_edges, "no auxiliary edges survive");
        let layout = dotgen::layout(&graph, &measured);
        assert!(layout.ranks.iter().all(|r| *r >= 0));
        assert!(layout.coords.iter().all(|c| c.x.is_finite() && c.y.is_finite()));
    }

    /// `concentrate=true`: parallel edges between the same endpoints are
    /// merged into one concentrator, so only the surviving edge is routed.
    #[test]
    fn concentrate_merges_parallel_edges() {
        let source = r#"digraph {
            concentrate=true;
            a -> b; a -> b; a -> b;
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        assert_eq!(view.edges.len(), 3, "every input edge keeps its record");
        let routed = view
            .edges
            .iter()
            .filter(|e| !e.segments.is_empty())
            .count();
        assert_eq!(routed, 1, "only the concentrator is routed");
    }

    /// A backward edge that shadows a forward edge is folded into it (class2's
    /// backward-edge branch): it gets no spline of its own.
    #[test]
    fn backward_edge_shadows_a_forward_edge() {
        let source = r#"digraph { a -> b; c -> b; b -> a; }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        for edge in view.edges.iter() {
            assert!(!edge.segments.is_empty(), "every edge is routed");
        }
        // ... and the shadowed pair shares one curve (the reverse edge runs
        // along the forward edge, not on its own channel).
        let ab = &view.edges[0];
        let ba = &view.edges[2];
        assert_eq!(ab.segments.len(), ba.segments.len());
    }

    /// `make_simple_label`'s output reaches the painter: object escapes are
    /// resolved, lines are split at `\n`/`\l`/`\r`, and each line keeps the
    /// justification the painter aligns it with (labels.c:245-269).
    #[test]
    fn label_escapes_reach_the_view_per_line() {
        let source = r#"digraph G {
            a [label="one\ltwo\rthree\N"];
            a -> b [label="\T->\H"];
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        let label = view.nodes[0].label.as_ref().expect("node label");
        assert_eq!(label.text, "one\ntwo\nthreea");
        assert_eq!(label.just, vec!['l', 'r', 'n']);
        let edge = view.edges[0].label.as_ref().expect("edge label");
        assert_eq!(edge.text, "a->b");
        assert_eq!(edge.just, vec!['n']);
        // An HTML label keeps its source text untouched.
        let html = r#"digraph { a [label=<<b>x</b>>]; }"#;
        let graph = Rc::new(graph::parser::parse(html).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: Vec::new(),
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        let label = view.nodes[0].label.as_ref().expect("html label");
        assert_eq!(label.text, "<b>x</b>");
    }

    /// A record node lays its fields out inside its box and exposes the
    /// painted parts (one box + text per leaf, a divider per boundary).
    #[test]
    fn record_nodes_expose_their_fields() {
        let source = r#"digraph {
            node [shape=record];
            a [label="<one> one|<two> two"];
            b [shape=box];
            a:one -> b;
            a:two -> b;
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
            measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        let a = view.nodes[0].clone();
        let record = a.record.as_ref().expect("record node exposes its fields");
        assert_eq!(record.fields.len(), 2, "two leaf fields");
        assert_eq!(record.dividers.len(), 1, "one divider between them");
        for (x, y, w, h, text) in record.fields.iter() {
            assert!(!text.is_empty(), "field text");
            assert!(
                *x >= a.x - 1.0
                    && *x + *w <= a.x + a.w + 1.0
                    && *y >= a.y - 1.0
                    && *y + *h <= a.y + a.h + 1.0,
                "field ({x}, {y}, {w}x{h}) escapes the record box {:?}",
                (a.x, a.y, a.w, a.h)
            );
        }
        // The two edges leave from different fields. In a TB layout the
        // record's fields sit side by side, so the two ports differ in x.
        let tails: Vec<f32> = view
            .edges
            .iter()
            .map(|e| e.segments.first().expect("segments")[0].0)
            .collect();
        assert!(
            (tails[0] - tails[1]).abs() > 1.0,
            "the two field ports must attach at different points: {tails:?}"
        );
    }

    /// A cluster laid out in LR must also expand: the rankdir flip path
    /// (`build_ranks`'s in-place row reversal) used to corrupt the root's rank
    /// rows for flipped graphs once clusters were enabled.
    #[test]
    fn cluster_expands_under_lr_too() {
        let source = r#"digraph {
            rankdir=LR;
            subgraph cluster_x { label="X"; p -> q; }
            subgraph cluster_y { label="Y"; r -> s; }
            q -> r;
            p -> s;
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
                    measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::LR);
        assert_eq!(view.clusters.len(), 2);
        // LR ranks are columns: the four nodes must land in four distinct
        // columns, in input order.
        let mut cols: Vec<f32> = view.nodes.iter().map(|n| n.x).collect();
        cols.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
        cols.dedup_by(|a, b| (*a - *b).abs() < 0.5);
        assert_eq!(cols.len(), 4, "expected four rank columns, got {cols:?}");
        for (name, expected) in [("p", 0usize), ("q", 1), ("r", 2), ("s", 3)] {
            let index = graph.nodes.iter().position(|n| n.name == name).expect("node");
            assert!((view.nodes[index].x - cols[expected]).abs() < 0.5, "{name} in the wrong column");
        }
        for edge in view.edges.iter() {
            assert!(!edge.segments.is_empty(), "edge not routed");
        }
    }

    /// Clusters must expand into real boxes: one box per `cluster_*` subgraph,
    /// each enclosing every member node, with the cluster's members on
    /// contiguous ranks (no foreign node between them). This is the regression
    /// guard for `expand_cluster` / `merge_ranks` / `interclexp`.
    #[test]
    fn cluster_boxes_contain_their_members() {
        let source = r#"digraph {
            subgraph cluster_a { label="alpha"; a -> b; }
            subgraph cluster_b { label="beta"; c -> d; }
            b -> c;
        }"#;
        let graph = Rc::new(graph::parser::parse(source).expect("parse"));
        let measured = dotgen::Measured {
            node: vec![(54.0, 36.0); graph.nodes.len()],
            label: Vec::new(),
            edge_label: vec![None; graph.edges.len()],
                    measure: None,
        };
        let view = build_view(&graph, &measured, RankDir::TB);
        assert_eq!(view.clusters.len(), 2, "one box per cluster subgraph");

        let node_box = |name: &str| {
            let index = graph
                .nodes
                .iter()
                .position(|n| n.name == name)
                .unwrap_or_else(|| panic!("node {name}"));
            view.nodes[index].clone()
        };
        let cluster_box = |label: &str| {
            view.clusters
                .iter()
                .find(|c| c.label.as_ref().is_some_and(|l| l.text == label))
                .unwrap_or_else(|| panic!("cluster labelled {label}"))
        };

        for (label, members) in [("alpha", ["a", "b"]), ("beta", ["c", "d"])] {
            let (x, y, w, h) = cluster_box(label).rect;
            assert!(w > 0.0 && h > 0.0, "{label} box is degenerate");
            for member in members {
                let n = node_box(member);
                assert!(
                    n.x >= x - 12.0
                        && n.x + n.w <= x + w + 12.0
                        && n.y >= y - 12.0
                        && n.y + n.h <= y + h + 12.0,
                    "node {member} ({}, {}, {}x{}) escapes the {label} box ({x}, {y}, {w}x{h})",
                    n.x,
                    n.y,
                    n.w,
                    n.h
                );
            }
        }

        // The two clusters occupy disjoint bands of ranks.
        let a = cluster_box("alpha").rect;
        let b = cluster_box("beta").rect;
        assert!(
            a.1 + a.3 <= b.1 + 1.0 || b.1 + b.3 <= a.1 + 1.0,
            "cluster boxes overlap: alpha {a:?} beta {b:?}"
        );
    }

    /// The "reset positions" escape hatch clears every drag offset at once and
    /// reports when there was nothing to undo.
    #[test]
    fn clearing_every_offset_restores_the_whole_layout() {
        let document = document("digraph { a -> b; c -> d; }");
        assert!(document.clear_offsets().is_none());

        let moved = document
            .with_node_offset(1, (40.0, 10.0))
            .and_then(|document| document.with_node_offset(2, (-30.0, 25.0)))
            .expect("both drops move their nodes");
        let base = document.effective_boxes();
        let reset = moved.clear_offsets().expect("moved nodes can be reset");
        for (node, box_) in reset.effective_boxes().iter().enumerate() {
            assert!((box_.x - base[node].x).abs() < 1e-3);
            assert!((box_.y - base[node].y).abs() < 1e-3);
        }
    }

    /// Double-click undo: clearing a node's offset restores its laid-out box,
    /// and clearing an unmoved node is a no-op.
    #[test]
    fn clearing_a_node_offset_restores_the_layout_position() {
        let document = document("digraph { a -> b; }");
        let base = document.effective_boxes();
        assert!(document.clear_node_offset(0).is_none());

        let moved = document
            .with_node_offset(1, (60.0, 20.0))
            .expect("the drop moves the node");
        let reset = moved
            .clear_node_offset(1)
            .expect("a moved node has an offset to clear");
        let boxes = reset.effective_boxes();
        assert!((boxes[1].x - base[1].x).abs() < 1e-3);
        assert!((boxes[1].y - base[1].y).abs() < 1e-3);
        // Out-of-range indices are ignored rather than panicking.
        assert!(document.clear_node_offset(99).is_none());
    }

    /// The head arrow's tip must sit on the target node's outline, and the
    /// line itself must stop short of it (dot pulls the visible end back to
    /// the arrow's base before `clip_and_install` stores the spline).
    #[test]
    fn arrow_tip_lands_on_the_node_border() {
        let document = document("digraph { a -> b; }");
        let b = &document.view.nodes[1];
        let (cx, cy) = (b.x + b.w / 2.0, b.y + b.h / 2.0);
        let edge = &document.view.edges[0];
        assert!(edge.eflag != 0, "edge should carry a head arrow");

        // The arrow apex is the polygon vertex that reaches furthest along
        // the edge's direction of travel (downward here).
        let mut tip: Option<(f32, f32)> = None;
        for part in &edge.arrows {
            if let crate::viz::dotview::ArrowPart::Polygon(points, _) = part {
                for p in points {
                    if tip.is_none_or(|t| p.1 > t.1) {
                        tip = Some(*p);
                    }
                }
            }
        }
        let tip = tip.expect("arrow polygon");
        // node b's outline: its top edge (TB layout), i.e. y = b.y
        assert!(
            (tip.1 - b.y).abs() < 2.0,
            "arrow tip {tip:?} is not on b's top border (y = {})",
            b.y
        );
        assert!(
            (tip.0 - cx).abs() < b.w / 2.0,
            "arrow tip {tip:?} is outside b's horizontal extent"
        );

        // The visible line stops at the arrow's base, i.e. short of the
        // border (the edge arrives from above, so a smaller y).
        let end = edge.segments.last().expect("segments")[3];
        assert!(
            end.1 < b.y - 1.0,
            "line end {end:?} ran past the arrow base (node top y = {})",
            b.y
        );
        // ... and the gap is about one arrow length.
        assert!(
            b.y - end.1 < 16.0,
            "line end {end:?} stopped too far from the border (y = {})",
            b.y
        );
        let _ = cy;
    }

    #[test]
    fn every_edge_is_routed_with_arrows() {
        let document = document("digraph { a -> b; b -> c; }");
        for edge in document.view.edges.iter() {
            assert!(!edge.segments.is_empty(), "edge not routed");
            assert!(!edge.arrows.is_empty(), "edge has no arrowhead");
        }
    }
}
