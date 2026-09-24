//! The visualization layer: everything that turns a laid-out [`Document`]
//! into pixels and pointer behavior, split by concern —
//!
//! - [`primitives`]: graphic primitives (rounded rects, circles, rounded
//!   polylines, polygons, text);
//! - [`layout`]: the layered layout algorithm and its text measurement;
//! - [`transform`]: the world ↔ screen transform and grid spacing;
//! - [`interact`]: pointer behavior (pan, zoom, drag, hover, selection);
//! - [`paint`]: composing primitives into the drawn graph frame;
//! - this module: the self-drawn GPUI [`Element`] that ties them together.
//!
//! No input state lives in the element — every change is written back into
//! the owning view through its weak handle.

pub mod dotview;
mod interact;
pub mod layout;
mod paint;
mod primitives;
mod transform;
mod x11colors;

use std::rc::Rc;

use gpui_kit::{
    App, Bounds, ContentMask, CursorStyle, Element, ElementId, GlobalElementId, Hitbox,
    HitboxBehavior, InspectorElementId, IntoElement, LayoutId, Pixels, Point, SharedString, Size,
    Style, WeakEntity, Window, point, px,
};

use gpui_kit::component::ActiveTheme as _;

use crate::app::GraphView;
use crate::document::Document;

use interact::CanvasCells;
pub use interact::NodeDrag;
use paint::Frame;
use transform::ViewTransform;

/// Zoom clamp shared with the zoom controls.
pub const MIN_ZOOM: f32 = 0.1;
pub const MAX_ZOOM: f32 = 8.0;

/// A snapshot of the graph plus the view state needed to draw it. Rebuilt by
/// the view on every render; cheap because the document is shared.
///
/// A self-contained GPUI *component*: built with [`Self::new`] plus the
/// builder methods, paints itself in [`Element::paint`], and reports
/// interaction back to the owning view through its weak handle. It is a
/// custom [`Element`] rather than a `div` tree on purpose — a large graph is
/// thousands of shapes per frame, and direct `paint_quad`/`paint_path` calls
/// skip the per-node Taffy layout and wrapping a `div`-based "shape" component
/// would pay for.
pub struct GraphCanvas {
    document: Option<Rc<Document>>,
    /// Message shown when there is no document (hint or error text).
    message: Option<SharedString>,
    /// Pan in screen pixels: where the world origin lands on the canvas.
    pan: Point<Pixels>,
    /// Zoom factor, clamped to [`MIN_ZOOM`], [`MAX_ZOOM`].
    zoom: f32,
    /// Index of the currently selected node, if any.
    selection: Option<usize>,
    /// The node being dragged, if any (see [`NodeDrag`]).
    node_drag: Option<NodeDrag>,
    /// Whether node labels are painted (settings "Show node labels").
    show_node_labels: bool,
    /// Whether edge labels are painted (settings "Show edge labels").
    show_edge_labels: bool,
    /// The label scale applied when painting text (settings "Label scale";
    /// the layout was measured at this same scale, so geometry matches).
    label_scale: f32,
    /// Whether the background grid is painted behind the graph.
    show_grid: bool,
    view: WeakEntity<GraphView>,
}

impl GraphCanvas {
    /// A canvas showing `document` and reporting interaction back to `view`.
    /// The view-dependent knobs are set with the builder methods below.
    pub fn new(document: Option<Rc<Document>>, view: WeakEntity<GraphView>) -> Self {
        Self {
            document,
            message: None,
            pan: point(px(0.0), px(0.0)),
            zoom: 1.0,
            selection: None,
            node_drag: None,
            show_node_labels: true,
            show_edge_labels: true,
            label_scale: 1.0,
            show_grid: false,
            view,
        }
    }

    /// Message shown when there is no document (hint or error text).
    pub fn message(mut self, message: Option<SharedString>) -> Self {
        self.message = message;
        self
    }

    /// Pan in screen pixels: where the world origin lands on the canvas.
    pub fn pan(mut self, pan: Point<Pixels>) -> Self {
        self.pan = pan;
        self
    }

    /// Zoom factor, clamped to [`MIN_ZOOM`], [`MAX_ZOOM`].
    pub fn zoom(mut self, zoom: f32) -> Self {
        self.zoom = zoom;
        self
    }

    /// Index of the currently selected node, if any.
    pub fn selection(mut self, selection: Option<usize>) -> Self {
        self.selection = selection;
        self
    }

    /// The node being dragged, if any (see [`NodeDrag`]).
    pub fn node_drag(mut self, node_drag: Option<NodeDrag>) -> Self {
        self.node_drag = node_drag;
        self
    }

    /// Whether node labels are painted (settings "Show node labels").
    pub fn show_node_labels(mut self, show: bool) -> Self {
        self.show_node_labels = show;
        self
    }

    /// Whether edge labels are painted (settings "Show edge labels").
    pub fn show_edge_labels(mut self, show: bool) -> Self {
        self.show_edge_labels = show;
        self
    }

    /// The label scale applied when painting text (settings "Label scale";
    /// the layout was measured at this same scale, so geometry matches).
    pub fn label_scale(mut self, label_scale: f32) -> Self {
        self.label_scale = label_scale;
        self
    }

    /// Whether the background grid is painted behind the graph.
    pub fn show_grid(mut self, show: bool) -> Self {
        self.show_grid = show;
        self
    }
}

impl Element for GraphCanvas {
    type RequestLayoutState = ();
    type PrepaintState = Option<Hitbox>;

    fn id(&self) -> Option<ElementId> {
        Some(ElementId::Name("graph-canvas".into()))
    }

    fn source_location(&self) -> Option<&'static std::panic::Location<'static>> {
        None
    }

    fn request_layout(
        &mut self,
        _: Option<&GlobalElementId>,
        _: Option<&InspectorElementId>,
        window: &mut Window,
        cx: &mut App,
    ) -> (LayoutId, Self::RequestLayoutState) {
        let style = Style {
            size: Size::full(),
            ..Default::default()
        };
        (window.request_layout(style, None, cx), ())
    }

    fn prepaint(
        &mut self,
        _: Option<&GlobalElementId>,
        _: Option<&InspectorElementId>,
        bounds: Bounds<Pixels>,
        _: &mut Self::RequestLayoutState,
        window: &mut Window,
        cx: &mut App,
    ) -> Self::PrepaintState {
        // Report the canvas size back to the view, which uses it to fit the
        // graph to the window. Idempotent per frame; ignores a dead view.
        let _ = self
            .view
            .update(cx, |view, _| view.canvas_bounds = Some(bounds));
        Some(window.insert_hitbox(bounds, HitboxBehavior::Normal))
    }

    fn paint(
        &mut self,
        global_id: Option<&GlobalElementId>,
        _: Option<&InspectorElementId>,
        bounds: Bounds<Pixels>,
        _: &mut Self::RequestLayoutState,
        prepaint: &mut Self::PrepaintState,
        window: &mut Window,
        cx: &mut App,
    ) {
        let Some(hitbox) = prepaint.as_ref().cloned() else {
            return;
        };

        let transform = ViewTransform {
            origin: bounds.origin,
            pan: self.pan,
            zoom: self.zoom,
        };

        // Element-local interaction state from previous dispatches: the node
        // under the cursor (if any), the cursor position for the tooltip and
        // whether a pan drag is in progress. A node drag is view state.
        let (hovered, mouse, dragging) = global_id
            .map(|id| {
                window.with_element_state::<CanvasCells, (Option<usize>, Option<Point<Pixels>>, bool)>(
                    id,
                    |prev, _| {
                        let cells = prev.unwrap_or_default();
                        (
                            (
                                cells.hovered.get(),
                                cells.mouse.get(),
                                cells.drag.get().is_some(),
                            ),
                            cells,
                        )
                    },
                )
            })
            .unwrap_or((None, None, false));
        let dragging = dragging || self.node_drag.is_some();

        // Cursor styles can only be registered during paint in this gpui
        // version; the window applies the hovered hitbox's request after the
        // frame draws (and re-applies it on hit-test changes). Closed hand
        // while dragging — window-wide so it holds even if the pointer drags
        // off the canvas — otherwise open hand over a draggable node.
        if dragging {
            window.set_window_cursor_style(CursorStyle::ClosedHand);
        } else {
            window.set_cursor_style(
                if hovered.is_some() {
                    CursorStyle::OpenHand
                } else {
                    CursorStyle::Arrow
                },
                &hitbox,
            );
        }

        // Everything visual is clipped to the canvas' own bounds: panning and
        // free node dragging routinely move graph content past this rectangle,
        // and the mask keeps the graph layer strictly *below* the surrounding
        // chrome (title bar above, status bar below) — the canvas can never
        // paint outside its layout box.
        window.with_content_mask(Some(ContentMask { bounds }), |window| {
            // Background (the graph's `bgcolor` when it sets one), then the
            // (optional) background grid, then the graph on top of both.
            let background = self
                .document
                .as_ref()
                .and_then(|d| crate::viz::dotview::color_of(d.graph.graph_attrs().get("bgcolor")))
                .unwrap_or_else(|| cx.theme().background);
            primitives::rounded_rect(window, bounds, px(0.0), background, None);
            if self.show_grid {
                paint::grid(transform, &bounds, window, cx);
            }
            match &self.document {
                Some(document) => {
                    let frame = Frame {
                        document,
                        node_drag: self.node_drag,
                        selection: self.selection,
                        hovered,
                        mouse,
                        show_node_labels: self.show_node_labels,
                        show_edge_labels: self.show_edge_labels,
                        label_scale: self.label_scale,
                        transform,
                        bounds,
                    };
                    paint::graph(&frame, window, cx);
                }
                None => {
                    if let Some(message) = &self.message {
                        paint::centered_message(message, &bounds, window, cx);
                    }
                }
            }
        });

        // Pointer behavior: wheel zoom/pan, node drag, empty-space pan, hover.
        let Some(cells) = global_id.map(|id| {
            window.with_element_state::<CanvasCells, CanvasCells>(id, |prev, _| {
                let cells = prev.unwrap_or_default();
                (cells.clone(), cells)
            })
        }) else {
            return;
        };
        interact::register(
            interact::Interaction {
                hitbox: &hitbox,
                transform,
                view: self.view.clone(),
                document: self.document.clone(),
                cells,
            },
            window,
        );
    }
}

impl IntoElement for GraphCanvas {
    type Element = Self;

    fn into_element(self) -> Self::Element {
        self
    }
}
