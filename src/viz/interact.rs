//! Canvas interaction: wheel zoom/pan, node drag, middle-button pan,
//! double-click reset, click selection and hover.
//!
//! The gestures are:
//!
//! * Ctrl/⌘ + wheel — zoom anchored at the cursor; a plain wheel pans;
//! * left-drag on a node — move it (past [`DRAG_THRESHOLD`], so an ordinary
//!   click selects without displacing the node); it follows the cursor freely,
//!   with no grid snapping;
//! * left- or middle-drag on empty space — pan;
//! * double-click on a moved node — send it back to its laid-out position;
//! * `escape` (the `ClearSelection` action) — drop the selection.
//!
//! No input state lives here — every handler writes its changes back into the
//! owning [`GraphView`] through its weak handle, and the element-local cells
//! ([`CanvasCells`], kept alive across frames by the canvas) hold only
//! transient dispatch state.

use std::rc::Rc;

use gpui_kit::{
    App, Hitbox, MouseButton, MouseDownEvent, MouseMoveEvent, MouseUpEvent, Pixels, Point,
    ScrollDelta, ScrollWheelEvent, WeakEntity, Window, point, px,
};

use crate::app::GraphView;
use crate::document::Document;

use super::transform::ViewTransform;
use super::{MAX_ZOOM, MIN_ZOOM};

/// One shared state cell: `Rc` so the paint pass and every handler clone see
/// the *same* value (a bare `Cell` would be copied per clone).
type Cell<T> = Rc<std::cell::Cell<T>>;

/// Element-local state cells shared between the paint-installed mouse
/// handlers and this frame's paint calls. Kept alive across frames by
/// [`Window::with_element_state`]. The `Rc`s matter: the paint pass clones
/// this struct into every handler, and the handlers must all read and write
/// the *same* cells (a plain `Cell` would be copied per clone, splitting the
/// press and move handlers' view of the state).
#[derive(Clone, Default)]
pub(crate) struct CanvasCells {
    /// `(mouse position, pan)` at the moment the left button went down on
    /// empty space (a pan drag in progress).
    pub drag: Cell<Option<(Point<Pixels>, Point<Pixels>)>>,
    /// The node index currently under the cursor.
    pub hovered: Cell<Option<usize>>,
    /// The cursor position, for painting the hover tooltip next to it.
    pub mouse: Cell<Option<Point<Pixels>>>,
}

/// A node being dragged freely: its graph index, the grab offset of the
/// pointer inside the box, and its live top-left position — both offsets in
/// world coordinates. `pos` follows the cursor without constraint (no grid
/// snapping) and is what the canvas paints and what a drop commits as a
/// permanent drag offset on release.
#[derive(Clone, Copy, Debug)]
pub struct NodeDrag {
    pub node: usize,
    pub grab: Point<f32>,
    pub pos: Point<f32>,
    /// Whether the pointer moved past [`DRAG_THRESHOLD`] since the press. A
    /// press/release that stays inside the threshold is a *click*: it selects
    /// the node and must not commit a drop offset (clicks jitter by a pixel
    /// or two, which would visibly shift the node).
    pub moved: bool,
    /// Screen position of the press, for the click/drag threshold.
    pub press: Point<Pixels>,
}

/// How far (screen pixels) the pointer must travel from the press before the
/// gesture counts as a drag rather than a click. Without it, a couple of
/// pixels of jitter during a click would commit a drop offset and visibly
/// shift the node.
pub(crate) const DRAG_THRESHOLD: f32 = 3.0;

/// Whether the pointer travelled far enough from `press` to be a drag.
fn past_drag_threshold(press: Point<Pixels>, now: Point<Pixels>) -> bool {
    (now.x - press.x)
        .as_f32()
        .abs()
        .max((now.y - press.y).as_f32().abs())
        > DRAG_THRESHOLD
}

/// Everything the handlers need from this frame's paint pass.
pub(crate) struct Interaction<'a> {
    pub hitbox: &'a Hitbox,
    pub transform: ViewTransform,
    pub view: WeakEntity<GraphView>,
    pub document: Option<Rc<Document>>,
    pub cells: CanvasCells,
}

/// Registers the canvas' mouse handlers. Called from the element's `paint` —
/// the only place this gpui version accepts event registration.
pub(crate) fn register(interaction: Interaction<'_>, window: &mut Window) {
    register_wheel(&interaction, window);
    register_press(&interaction, window);
    register_move(&interaction, window);
    register_release(&interaction, window);
}

/// Wheel: Ctrl/⌘ zooms at the cursor; a plain wheel pans.
fn register_wheel(interaction: &Interaction<'_>, window: &mut Window) {
    let hitbox = interaction.hitbox.clone();
    let transform = interaction.transform;
    let view = interaction.view.clone();
    let hover_cell = interaction.cells.hovered.clone();
    let mouse_cell = interaction.cells.mouse.clone();
    window.on_mouse_event(
        move |e: &ScrollWheelEvent, phase, window: &mut Window, cx: &mut App| {
            if !phase.bubble() || !hitbox.is_hovered(window) {
                return;
            }
            let delta = match e.delta {
                ScrollDelta::Pixels(delta) => delta,
                ScrollDelta::Lines(lines) => point(px(lines.x * 40.0), px(lines.y * 40.0)),
            };
            if e.modifiers.control || e.modifiers.platform {
                // Zoom anchored at the cursor: keep the world point under the
                // cursor fixed, pan' = cursor - origin - world * zoom'.
                let (wx, wy) = transform.to_world(e.position);
                let Some(view) = view.upgrade() else {
                    return;
                };
                view.update(cx, |view, cx| {
                    let factor = 1.0 + delta.y.as_f32() * 0.0018;
                    let Some(tab) = view.active_mut() else { return };
                    let next_zoom = (tab.zoom * factor).clamp(MIN_ZOOM, MAX_ZOOM);
                    if (next_zoom - tab.zoom).abs() < 1e-4 {
                        return;
                    }
                    tab.pan = point(
                        e.position.x - transform.origin.x - px(wx * next_zoom),
                        e.position.y - transform.origin.y - px(wy * next_zoom),
                    );
                    tab.zoom = next_zoom;
                    // The world moved under a stationary cursor: the node
                    // under it (and the tooltip that follows it) may differ.
                    let now = ViewTransform {
                        pan: tab.pan,
                        zoom: tab.zoom,
                        ..transform
                    };
                    hover_cell.set(hover_at(tab.document.as_ref(), &now, e.position));
                    mouse_cell.set(Some(e.position));
                    cx.notify();
                });
            } else {
                let Some(view) = view.upgrade() else {
                    return;
                };
                view.update(cx, |view, cx| {
                    // Plain wheel pans. The direction is a setting: the
                    // traditional direction (scroll down reveals what is
                    // below) versus the macOS-style "natural" scrolling
                    // where the content follows the scroll.
                    let (dx, dy) = if view.settings.natural_scroll {
                        (delta.x, delta.y)
                    } else {
                        (-delta.x, -delta.y)
                    };
                    let Some(tab) = view.active_mut() else { return };
                    tab.pan = point(tab.pan.x + dx, tab.pan.y + dy);
                    let now = ViewTransform {
                        pan: tab.pan,
                        ..transform
                    };
                    hover_cell.set(hover_at(tab.document.as_ref(), &now, e.position));
                    mouse_cell.set(Some(e.position));
                    cx.notify();
                });
            }
        },
    );
}

/// Pointer down: start dragging the node under the cursor (selecting it), or
/// pan when the press lands on empty space. Left-button drags nodes, middle-
/// button always pans, and a double-click on a moved node sends it back to its
/// laid-out position. The pointer is captured so the drag keeps working even
/// when it leaves the canvas.
fn register_press(interaction: &Interaction<'_>, window: &mut Window) {
    let hitbox = interaction.hitbox.clone();
    let transform = interaction.transform;
    let view = interaction.view.clone();
    let document = interaction.document.clone();
    let drag_cell = interaction.cells.drag.clone();
    let hover_cell = interaction.cells.hovered.clone();
    let mouse_cell = interaction.cells.mouse.clone();
    window.on_mouse_event(
        move |e: &MouseDownEvent, phase, window: &mut Window, cx: &mut App| {
            if !phase.bubble() || !hitbox.is_hovered(window) {
                return;
            }
            if e.button != MouseButton::Left && e.button != MouseButton::Middle {
                return;
            }
            window.prevent_default();
            window.capture_pointer(hitbox.id);
            // The effective boxes (committed drag offsets applied) decide the
            // hit and the grab, so re-grabbing a moved node does not jump it
            // back to its laid-out spot.
            let boxes = document.as_ref().map(|document| document.effective_boxes());
            // Middle-button panning never grabs a node; only the left button
            // selects and drags.
            let hit = if e.button == MouseButton::Left {
                hit_test(boxes.as_deref().unwrap_or(&[]), &transform, e.position)
            } else {
                None
            };
            // A double-click on a node that has a committed drag offset puts
            // it back on its laid-out position — the only way to undo a move.
            let reset = e.button == MouseButton::Left
                && e.click_count >= 2
                && hit.is_some_and(|index| {
                    document
                        .as_ref()
                        .and_then(|document| document.offsets.get(index).copied())
                        .is_some_and(|offset| offset != (0.0, 0.0))
                });
            // The grab offset inside the box (world units) keeps the node
            // under the cursor while it moves.
            let node_drag = if reset {
                None
            } else {
                hit.and_then(|index| {
                    let node_box = boxes.as_ref()?.get(index).copied()?;
                    let (wx, wy) = transform.to_world(e.position);
                    Some(NodeDrag {
                        node: index,
                        grab: point(wx - node_box.x, wy - node_box.y),
                        pos: point(node_box.x, node_box.y),
                        moved: false,
                        press: e.position,
                    })
                })
            };
            let Some(view) = view.upgrade() else {
                return;
            };
            view.update(cx, |view, cx| {
                let Some(tab) = view.active_mut() else { return };
                if reset {
                    if let Some(index) = hit
                        && let Some(document) = tab.document.clone()
                        && let Some(reset) = document.clear_node_offset(index)
                    {
                        tab.document = Some(Rc::new(reset));
                    }
                    tab.selection = hit;
                    tab.node_drag = None;
                    drag_cell.set(None);
                    hover_cell.set(hover_at(tab.document.as_ref(), &transform, e.position));
                    mouse_cell.set(Some(e.position));
                    cx.notify();
                    return;
                }
                tab.selection = hit;
                tab.node_drag = node_drag;
                if node_drag.is_none() {
                    drag_cell.set(Some((e.position, tab.pan)));
                }
                cx.notify();
            });
        },
    );
}

/// Move: drag the node (live position), pan while dragging empty space, and
/// otherwise track hover for the cursor and the tooltip.
fn register_move(interaction: &Interaction<'_>, window: &mut Window) {
    let hitbox = interaction.hitbox.clone();
    let transform = interaction.transform;
    let view = interaction.view.clone();
    let document = interaction.document.clone();
    let drag_cell = interaction.cells.drag.clone();
    let hover_cell = interaction.cells.hovered.clone();
    let mouse_cell = interaction.cells.mouse.clone();
    window.on_mouse_event(
        move |e: &MouseMoveEvent, phase, window: &mut Window, cx: &mut App| {
            if !phase.bubble() {
                return;
            }
            let Some(view) = view.upgrade() else {
                return;
            };
            view.update(cx, |view, cx| {
                let Some(tab) = view.active_mut() else { return };
                if let Some(node_drag) = tab.node_drag.as_mut() {
                    let (wx, wy) = transform.to_world(e.position);
                    let pos = point(wx - node_drag.grab.x, wy - node_drag.grab.y);
                    // A press/release that never left the threshold is a
                    // click, not a drop: only a real move may commit an
                    // offset on release.
                    if past_drag_threshold(node_drag.press, e.position) {
                        node_drag.moved = true;
                    }
                    node_drag.pos = pos;
                    tab.selection = Some(node_drag.node);
                    mouse_cell.set(Some(e.position));
                    cx.notify();
                } else if let Some((start_mouse, start_pan)) = drag_cell.get() {
                    let delta = point(e.position.x - start_mouse.x, e.position.y - start_mouse.y);
                    tab.pan = point(start_pan.x + delta.x, start_pan.y + delta.y);
                    // The graph slides under a stationary cursor while
                    // panning, so no node stays hovered: hide the tooltip and
                    // let the release (or the next move) re-hit-test.
                    hover_cell.set(None);
                    mouse_cell.set(None);
                    cx.notify();
                } else if hitbox.is_hovered(window) {
                    let boxes = document.as_ref().map(|document| document.effective_boxes());
                    let hover = hit_test(boxes.as_deref().unwrap_or(&[]), &transform, e.position);
                    let previous = hover_cell.get();
                    // Repaint on hover changes, and on every move while a node
                    // is hovered so the tooltip follows the cursor.
                    if previous != hover || hover.is_some() {
                        hover_cell.set(hover);
                        mouse_cell.set(Some(e.position));
                        window.refresh();
                    }
                } else if hover_cell.get().is_some() {
                    hover_cell.set(None);
                    mouse_cell.set(None);
                    window.refresh();
                }
            });
        },
    );
}

/// Pointer up ends either drag: a node drop commits the dragged position as a
/// permanent offset (edges follow at paint time), an empty drop ends panning.
/// The selection persists either way. A press/release without motion is only a
/// click — it must not move the node.
fn register_release(interaction: &Interaction<'_>, window: &mut Window) {
    let transform = interaction.transform;
    let view = interaction.view.clone();
    let drag_cell = interaction.cells.drag.clone();
    let hover_cell = interaction.cells.hovered.clone();
    let mouse_cell = interaction.cells.mouse.clone();
    window.on_mouse_event(
        move |e: &MouseUpEvent, phase, window: &mut Window, cx: &mut App| {
            if !phase.bubble() {
                return;
            }
            if e.button != MouseButton::Left && e.button != MouseButton::Middle {
                return;
            }
            let Some(view) = view.upgrade() else {
                return;
            };
            view.update(cx, |view, cx| {
                let Some(tab) = view.active_mut() else { return };
                if let Some(drag) = tab.node_drag.take() {
                    if drag.moved {
                        // Commit the free drop position (exactly where the
                        // cursor left the node — no snapping), as an offset
                        // from its laid-out box; every edge touching it
                        // re-anchors to the moved border when the next frame
                        // paints.
                        if let Some(document) = tab.document.clone()
                            && let Some(base) = document.node_box(drag.node)
                            && let Some(updated) = document.with_node_offset(
                                drag.node,
                                (drag.pos.x - base.x, drag.pos.y - base.y),
                            )
                        {
                            tab.document = Some(Rc::new(updated));
                        }
                        cx.notify();
                    }
                } else if drag_cell.take().is_some() {
                    // Panning ended: the graph may have moved under the
                    // cursor, so re-hit-test before the next frame picks the
                    // cursor style (and the tooltip) again.
                    cx.notify();
                }
                // Refresh hover for the drop/pan end position: the pointer is
                // where it was released, but the boxes underneath it moved.
                hover_cell.set(hover_at(tab.document.as_ref(), &transform, e.position));
                mouse_cell.set(Some(e.position));
                window.refresh();
            });
        },
    );
}

/// Finds the node under a cursor position, in screen space. `boxes` are the
/// effective node boxes (committed drag offsets applied), so hover, grab and
/// selection all agree with what is painted.
///
/// Nodes are painted in index order, so the *last* box containing the point
/// is the one visible on top; hit-testing the first match would grab the node
/// hidden underneath whenever boxes overlap (e.g. after dragging one node
/// onto another).
pub(crate) fn hit_test(
    boxes: &[super::layout::NodeBox],
    transform: &ViewTransform,
    mouse: Point<Pixels>,
) -> Option<usize> {
    let (wx, wy) = transform.to_world(mouse);
    boxes.iter().rposition(|node| node.contains(wx, wy))
}

/// The node under `position` for the given document snapshot, or `None` when
/// the pointer is off every box (or there is no document). Used to refresh the
/// hover/tooltip state after the view moved under a stationary cursor
/// (zoom, pan, drop).
fn hover_at(
    document: Option<&Rc<Document>>,
    transform: &ViewTransform,
    position: Point<Pixels>,
) -> Option<usize> {
    let boxes = document.map(|document| document.effective_boxes());
    hit_test(boxes.as_deref().unwrap_or(&[]), transform, position)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viz::layout::NodeBox;
    use gpui_kit::px;

    fn transform() -> ViewTransform {
        ViewTransform {
            origin: point(px(0.0), px(0.0)),
            pan: point(px(0.0), px(0.0)),
            zoom: 1.0,
        }
    }

    /// Overlapping boxes are painted in index order, so the last one is the
    /// visible (topmost) node: a hit test that returned the first match would
    /// grab a node hidden underneath.
    #[test]
    fn hit_test_prefers_the_topmost_node() {
        let boxes = [
            NodeBox {
                x: 0.0,
                y: 0.0,
                w: 100.0,
                h: 100.0,
            },
            NodeBox {
                x: 50.0,
                y: 50.0,
                w: 100.0,
                h: 100.0,
            },
        ];
        let t = transform();
        // Inside both: the later (painted-on-top) node wins.
        assert_eq!(hit_test(&boxes, &t, point(px(60.0), px(60.0))), Some(1));
        // Inside only the first.
        assert_eq!(hit_test(&boxes, &t, point(px(10.0), px(10.0))), Some(0));
        // Inside neither.
        assert_eq!(hit_test(&boxes, &t, point(px(300.0), px(300.0))), None);
    }

    /// A click is a press/release that stays inside [`DRAG_THRESHOLD`]; only a
    /// gesture that travels past it counts as a drag (and therefore commits a
    /// snapped drop offset). Without the threshold, the pixel or two of jitter
    /// in an ordinary click would shift the node.
    #[test]
    fn click_and_drag_are_separated_by_the_threshold() {
        let at = |x: f32, y: f32| point(px(x), px(y));
        let press = at(100.0, 100.0);
        assert!(!past_drag_threshold(press, at(100.0, 100.0)));
        assert!(!past_drag_threshold(press, at(102.0, 101.0)), "jitter");
        assert!(!past_drag_threshold(
            press,
            at(100.0 + DRAG_THRESHOLD, 100.0)
        ));
        assert!(past_drag_threshold(
            press,
            at(100.0 + DRAG_THRESHOLD + 0.5, 100.0)
        ));
        assert!(
            past_drag_threshold(press, at(100.0, 92.0)),
            "vertical travel"
        );
    }

    /// The hit test runs in screen space through the view transform, so pan
    /// and zoom must be honored.
    #[test]
    fn hit_test_follows_pan_and_zoom() {
        let boxes = [NodeBox {
            x: 10.0,
            y: 20.0,
            w: 40.0,
            h: 30.0,
        }];
        let t = ViewTransform {
            origin: point(px(5.0), px(7.0)),
            pan: point(px(100.0), px(50.0)),
            zoom: 2.0,
        };
        // world (10,20) → screen (5+100+20, 7+50+40) = (125, 97)
        assert_eq!(hit_test(&boxes, &t, point(px(125.0), px(97.0))), Some(0));
        assert_eq!(hit_test(&boxes, &t, point(px(124.0), px(96.0))), None);
    }
}
