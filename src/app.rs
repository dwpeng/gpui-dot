//! The application shell: the [`GraphView`] owns the current document and the
//! navigation state (pan / zoom / selection) that the self-drawn canvas reads
//! and writes. The chrome lives in [`crate::ui`], the settings in
//! [`crate::settings`], the canvas in [`crate::viz`].

use std::path::{Path, PathBuf};
use std::rc::Rc;

use gpui_kit::WindowTextSystem;
use gpui_kit::{
    App, AppContext as _, AsyncApp, Bounds, Context, FocusHandle, Focusable, InteractiveElement,
    IntoElement, PathPromptOptions, Pixels, Point, Render, SharedString, WeakEntity, Window,
    point, px,
};

use crate::actions::{ClearSelection, FitGraph, OpenGraph, ZoomIn, ZoomOut};
use crate::document::{Document, load_document};
use crate::file_dialog;
use crate::settings::Settings;
use crate::ui::{status_bar, title_bar, zoom_cluster};
use crate::viz::GraphCanvas;
use crate::viz::NodeDrag;
use crate::viz::layout::RankDir;

/// Root view of the viewer.
pub struct GraphView {
    /// The loaded document, or `None` while empty or after a failed load.
    pub document: Option<Rc<Document>>,
    /// A user-facing message describing a load or dialog failure, if any.
    pub error: Option<String>,
    /// Pan in screen pixels: where the world origin lands on the canvas.
    pub pan: Point<Pixels>,
    /// Zoom factor, clamped to `MIN_ZOOM`..`MAX_ZOOM`.
    pub zoom: f32,
    /// Index of the currently selected node, if any.
    pub selection: Option<usize>,
    /// The node currently being dragged, if any.
    pub node_drag: Option<NodeDrag>,
    /// Canvas bounds in screen pixels, reported back by the canvas each frame.
    pub canvas_bounds: Option<Bounds<Pixels>>,
    /// Whether the graph should be auto-fitted once the canvas reports bounds.
    fit_pending: bool,
    /// The viewer's settings, edited from the title-bar settings card.
    pub settings: Settings,
    /// The ("Direction", "Label scale") pair the current `document` was last
    /// measured and laid out with. Kept in sync on load and on settings
    /// re-layout, so a settings change can skip the recompute when it would
    /// not change anything.
    applied_layout: Option<(RankDir, f32)>,
    /// Owned focus handle so the root keeps keyboard focus (actions, keys).
    focus_handle: FocusHandle,
}

impl GraphView {
    /// Creates the view and kicks off the load of `file` if one was given.
    /// Initial keyboard focus is applied by the caller (which owns the window).
    pub fn new(file: Option<PathBuf>, cx: &mut Context<Self>) -> Self {
        let focus_handle = cx.focus_handle();
        let mut view = Self {
            document: None,
            error: None,
            pan: point(px(0.0), px(0.0)),
            zoom: 1.0,
            selection: None,
            node_drag: None,
            canvas_bounds: None,
            fit_pending: true,
            settings: Settings::default(),
            applied_layout: None,
            focus_handle,
        };
        if let Some(path) = file {
            view.load_path(path, cx);
        }
        view
    }

    /// Loads a DOT file off the main thread and applies the result on the
    /// next frame. Fails softly: the error message is shown on the canvas.
    pub fn load_path(&mut self, path: PathBuf, cx: &mut Context<Self>) {
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            // Measure labels with the app's text system (a cheap per-window
            // shaping wrapper around the shared font database), so the layout
            // always matches the fonts the canvas renders with.
            let result = cx.update(|app: &mut App| {
                let text_system = WindowTextSystem::new(app.text_system().clone());
                load_document(path.clone(), &text_system)
            });
            let _ = this.update(cx, |view, cx| match result {
                Ok(document) => {
                    // The card's direction follows the file's declared
                    // `rankdir`, and the fresh measurements are at the base
                    // label scale; if the user's "Label scale" preference
                    // differs, the relayout below re-measures and re-lays-out
                    // so the drawing matches the setting.
                    let loaded_dir = document.rank_dir();
                    view.settings.rank_dir = match loaded_dir {
                        RankDir::LR => "LR".into(),
                        RankDir::TB => "TB".into(),
                    };
                    view.document = Some(Rc::new(document));
                    view.error = None;
                    view.selection = None;
                    view.node_drag = None;
                    view.fit_pending = true;
                    view.applied_layout = Some((loaded_dir, 1.0));
                    view.relayout_for_settings(cx);
                    cx.notify();
                }
                Err(err) => {
                    view.document = None;
                    view.error = Some(format!("Failed to open {}: {err}", path.display()));
                    cx.notify();
                }
            });
        })
        .detach();
    }

    /// Opens a file picker and loads the picked file.
    ///
    /// Outside WSL this is the platform dialog (`prompt_for_paths`). Inside
    /// WSL that dialog is the XDG file-chooser portal, which under WSLg has
    /// no session D-Bus to talk to, so nothing ever opens. There the WSL
    /// files are a Windows network share (`\\wsl.localhost\<distro>\...`),
    /// and the picker is the Windows common dialog run through interop (see
    /// [`crate::file_dialog`]); the portal stays as the last resort.
    pub(crate) fn open_graph(&mut self, cx: &mut Context<Self>) {
        if file_dialog::running_in_wsl() {
            self.open_graph_via_windows(cx);
            return;
        }
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            Self::open_graph_via_portal(this, cx, None).await;
        })
        .detach();
    }

    /// Opens the Windows file dialog (the WSL fallback) and loads the picked
    /// file.
    fn open_graph_via_windows(&mut self, cx: &mut Context<Self>) {
        let start_dir = self
            .document
            .as_ref()
            .and_then(|document| document.path.as_ref())
            .and_then(|path| path.parent())
            .map(Path::to_path_buf);
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            // Reaching the Windows side (spawning powershell.exe through
            // interop) can take seconds on a cold VM: keep it off the UI
            // thread.
            let picked = cx.update(|cx: &mut App| {
                cx.background_spawn(async move {
                    file_dialog::pick_file("Select a DOT file", start_dir.as_deref())
                })
            });
            match picked.await {
                Ok(Some(path)) => {
                    let _ = this.update(cx, |view, cx| view.load_path(path, cx));
                }
                Ok(None) => {} // the user closed the Windows dialog
                Err(message) => {
                    // Interop itself failed (no powershell.exe): try the
                    // portal dialog, which reports its failure on the canvas.
                    let failure = format!("Windows file dialog failed: {message}");
                    Self::open_graph_via_portal(this, cx, Some(failure)).await;
                }
            }
        })
        .detach();
    }

    /// Shows the platform file dialog and loads the picked file. On
    /// platforms without a working dialog (e.g. some Linux setups) the
    /// relayed error is shown on the canvas instead, prefixed by
    /// `error_prefix` when a prior attempt already failed.
    async fn open_graph_via_portal(
        this: WeakEntity<Self>,
        cx: &mut AsyncApp,
        error_prefix: Option<String>,
    ) {
        let receiver = cx.update(|cx: &mut App| {
            cx.prompt_for_paths(PathPromptOptions {
                files: true,
                directories: false,
                multiple: false,
                prompt: Some("Select a DOT file".into()),
            })
        });
        let picked = match receiver.await {
            Ok(Ok(Some(mut paths))) if !paths.is_empty() => paths.remove(0),
            Ok(Err(err)) => {
                let mut message = String::new();
                if let Some(prefix) = error_prefix {
                    message.push_str(&prefix);
                    message.push('\n');
                }
                message.push_str(&format!(
                    "System file dialog unavailable: {err}\nOpen a file from the command line: dotv <file>"
                ));
                let _ = this.update(cx, |view, cx| {
                    view.error = Some(message);
                    cx.notify();
                });
                return;
            }
            _ => return, // cancelled
        };
        let _ = this.update(cx, |view, cx| view.load_path(picked, cx));
    }

    /// Undoes every manual node move: the drawing snaps back to the positions
    /// the layout computed. No-op when nothing has been dragged.
    pub fn reset_positions(&mut self, cx: &mut Context<Self>) {
        let Some(document) = self.document.clone() else {
            return;
        };
        if let Some(cleared) = document.clear_offsets() {
            self.document = Some(Rc::new(cleared));
            self.node_drag = None;
            cx.notify();
        }
    }

    pub fn zoom_in(&mut self, cx: &mut Context<Self>) {
        self.set_zoom(self.zoom * 1.25, cx);
    }

    pub fn zoom_out(&mut self, cx: &mut Context<Self>) {
        self.set_zoom(self.zoom / 1.25, cx);
    }

    /// Applies the layout-affecting settings ("Direction", "Label scale") to
    /// the loaded document: re-measures the labels at the chosen scale and
    /// re-runs the layout in the chosen direction, off the main thread, then
    /// swaps the new snapshot in and refits. No-op when there is no document
    /// or when the settings already match what the document was built with
    /// (tracked in [`Self::applied_layout`]).
    pub fn relayout_for_settings(&mut self, cx: &mut Context<Self>) {
        let Some(document) = self.document.clone() else {
            return;
        };
        let rank_dir = match self.settings.rank_dir.as_str() {
            "LR" => RankDir::LR,
            _ => RankDir::TB,
        };
        let label_scale = self.settings.label_scale as f32;
        if self.applied_layout == Some((rank_dir, label_scale)) {
            return;
        }
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            let result = cx.update(|app: &mut App| {
                let text_system = WindowTextSystem::new(app.text_system().clone());
                document.relayout(rank_dir, label_scale, &text_system)
            });
            let _ = this.update(cx, |view, cx| {
                // Ignore a stale result: the settings may have changed again
                // while the relayout was running. The newest relayout spawns
                // from the newest settings, so only let it land if it matches
                // what the user currently has selected.
                let current_dir = match view.settings.rank_dir.as_str() {
                    "LR" => RankDir::LR,
                    _ => RankDir::TB,
                };
                if current_dir != rank_dir
                    || (view.settings.label_scale as f32 - label_scale).abs() > 1e-3
                {
                    return;
                }
                view.document = Some(Rc::new(result));
                view.applied_layout = Some((rank_dir, label_scale));
                // The fresh layout positions every node from scratch (and
                // resets its drag offsets); a drag in flight would fight it.
                view.node_drag = None;
                view.fit_pending = true;
                cx.notify();
            });
        })
        .detach();
    }

    fn set_zoom(&mut self, target: f32, cx: &mut Context<Self>) {
        let next = target.clamp(crate::viz::MIN_ZOOM, crate::viz::MAX_ZOOM);
        if (next - self.zoom).abs() > 1e-6 {
            self.zoom = next;
            cx.notify();
        }
    }

    /// Fits the graph inside the canvas; when the canvas has not reported its
    /// bounds yet, defers the fit by keeping `fit_pending` set.
    pub fn fit_graph(&mut self, cx: &mut Context<Self>) {
        if self.compute_fit() {
            self.fit_pending = false;
            cx.notify();
        } else {
            self.fit_pending = true;
        }
    }

    /// Computes pan/zoom so the whole layout fits the canvas with a margin.
    /// Returns `false` when there is nothing to fit yet (no document or no
    /// canvas bounds). Does not notify — callers decide whether to repaint.
    /// The fit box covers the laid-out bounds plus every node a drag offset
    /// has moved (see [`effective_bounds`]), so dragged nodes stay visible.
    fn compute_fit(&mut self) -> bool {
        let Some(document) = &self.document else {
            return false;
        };
        let Some(bounds) = self.canvas_bounds else {
            return false;
        };
        let (min_x, min_y, max_x, max_y) = document.effective_bounds();
        let layout_w = (max_x - min_x).max(1.0);
        let layout_h = (max_y - min_y).max(1.0);
        let inset = 24.0_f32;
        let canvas_w = (bounds.size.width.as_f32() - inset * 2.0).max(1.0);
        let canvas_h = (bounds.size.height.as_f32() - inset * 2.0).max(1.0);
        let fitted = (canvas_w / layout_w).min(canvas_h / layout_h);
        self.zoom = fitted.clamp(crate::viz::MIN_ZOOM, crate::viz::MAX_ZOOM);
        // Center the content's actual box (its bounds center, which is not
        // guaranteed to sit at the world origin) on the canvas center.
        let canvas_cx = bounds.size.width.as_f32() / 2.0;
        let canvas_cy = bounds.size.height.as_f32() / 2.0;
        let content_cx = (min_x + max_x) / 2.0;
        let content_cy = (min_y + max_y) / 2.0;
        self.pan = point(
            px(canvas_cx - content_cx * self.zoom),
            px(canvas_cy - content_cy * self.zoom),
        );
        true
    }

    /// Message shown on the canvas when there is no graph to draw.
    fn canvas_message(&self) -> Option<SharedString> {
        if let Some(error) = &self.error {
            return Some(SharedString::from(error.as_str()));
        }
        self.document
            .is_none()
            .then(|| SharedString::from("Open a DOT file to get started (Ctrl+O · or dotv <file>)"))
    }
}

impl Focusable for GraphView {
    fn focus_handle(&self, _cx: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl Render for GraphView {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        // On the first frame after a load, fit once the canvas has real bounds.
        if self.fit_pending && self.compute_fit() {
            self.fit_pending = false;
        }
        let weak = cx.weak_entity();

        use gpui_kit::base::StyledExt as _;
        use gpui_kit::component::Root;
        use gpui_kit::{ParentElement, Styled, div};

        div()
            .id("dotv-app")
            .size_full()
            .v_flex()
            .track_focus(&self.focus_handle)
            .on_action(cx.listener(|this, _: &OpenGraph, _window, cx| this.open_graph(cx)))
            .on_action(cx.listener(|this, _: &ZoomIn, _window, cx| this.zoom_in(cx)))
            .on_action(cx.listener(|this, _: &ZoomOut, _window, cx| this.zoom_out(cx)))
            .on_action(cx.listener(|this, _: &FitGraph, _window, cx| this.fit_graph(cx)))
            .on_action(cx.listener(|this, _: &ClearSelection, _window, cx| {
                // Escape drops the selection (and any drag in flight); the
                // committed node positions stay where the user put them.
                if this.selection.take().is_some() || this.node_drag.take().is_some() {
                    cx.notify();
                }
            }))
            .child(title_bar(self, cx))
            .child(
                div()
                    .id("dotv-canvas")
                    .relative()
                    .flex_1()
                    .min_h_0()
                    // Clip the canvas area to its layout box, which sits
                    // strictly between the title bar and the status bar.
                    .overflow_hidden()
                    .child(
                        GraphCanvas::new(self.document.clone(), weak)
                            .message(self.canvas_message())
                            .pan(self.pan)
                            .zoom(self.zoom)
                            .selection(self.selection)
                            .node_drag(self.node_drag)
                            .show_node_labels(self.settings.show_node_labels)
                            .show_edge_labels(self.settings.show_edge_labels)
                            .label_scale(self.applied_layout.map_or(1.0, |(_, scale)| scale))
                            .show_grid(self.settings.show_grid),
                    )
                    .child(zoom_cluster(self, cx)),
            )
            .child(status_bar(self, cx))
            .children(Root::render_dialog_layer(window, cx))
            .children(Root::render_sheet_layer(window, cx))
            .children(Root::render_notification_layer(window, cx))
    }
}
