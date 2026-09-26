//! The application shell: the [`GraphView`] owns the open tabs, and every
//! tab (a [`GraphTab`]) owns its document and navigation state (pan / zoom /
//! selection) that the self-drawn canvas reads and writes. The chrome lives
//! in [`crate::ui`], the settings in [`crate::settings`], the canvas in
//! [`crate::viz`].

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::time::Duration;

use gpui_kit::WindowTextSystem;
use gpui_kit::component::WindowExt as _;
use gpui_kit::component::TitleBar;
use gpui_kit::component::notification::Notification;
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{
    App, AppContext as _, AsyncApp, Bounds, ClipboardEntry, Context, DragMoveEvent, ExternalPaths,
    FileDropEvent, FocusHandle, Focusable, InteractiveElement, IntoElement, PathPromptOptions,
    Pixels, Point, Render, SharedString, Size, TestSupportExt as _, TitlebarOptions,
    WeakEntity, Window, WindowBounds, WindowDecorations, WindowOptions, point, px,
};

use crate::actions::{
    ClearSelection, CloseTab, FitGraph, NextTab, OpenGraph, PasteGraph, PrevTab, SearchTabs,
    ToggleFullscreen, ZoomIn, ZoomOut,
};
use crate::document::{Document, LoadOutput, load_parts, relayout_graph};
use crate::file_dialog::{self, parse_pasted_paths};
use crate::graph::LoadError;
use crate::settings::{Settings, SettingsStore};
use crate::ui::{
    TabDrag, drop_overlay, empty_state, loading_overlay, status_bar, title_bar, zoom_cluster,
};
use crate::viz::GraphCanvas;
use crate::viz::NodeDrag;
use crate::viz::dotview::DotView;
use crate::viz::layout::RankDir;

/// The options every dotv window opens with: the client-decorated title bar the
/// app draws itself, so the tab strip and the window's own controls share a row.
/// `size` and `origin` are the window's, in screen coordinates.
pub fn window_options(size: Size<Pixels>, origin: Point<Pixels>) -> WindowOptions {
    WindowOptions {
        window_bounds: Some(WindowBounds::Windowed(Bounds { origin, size })),
        titlebar: Some(TitlebarOptions {
            title: Some("dotv — DOT Graph Viewer".into()),
            ..TitleBar::title_bar_options()
        }),
        window_decorations: Some(WindowDecorations::Client),
        ..TitleBar::window_options()
    }
}

/// One open file: its loaded snapshot plus the navigation state the canvas
/// manipulates. Every field is per-tab, so switching tabs restores exactly
/// where the user left that drawing.
/// Cloned when a tab is detached: the clone shares the loaded document (it is
/// behind an `Rc`) and copies the navigation state, so the receiving window can
/// be built before this one gives the tab up.
#[derive(Clone)]
pub struct GraphTab {
    /// Stable identity for element ids and async loads (survives reordering).
    pub id: u64,
    /// The file this tab shows, as opened.
    pub path: PathBuf,
    /// The file name, for the tab label.
    pub title: String,
    /// The loaded document, or `None` while loading or after a failed load.
    pub document: Option<Rc<Document>>,
    /// A user-facing message describing a load failure, if any.
    pub error: Option<String>,
    /// Whether an off-thread load or re-layout for this tab is still running.
    /// A first load (no document yet) covers the canvas with the loading
    /// animation; a re-layout keeps the previous drawing visible and the
    /// status bar shows the wait.
    pub loading: bool,
    /// Pan in screen pixels: where the world origin lands on the canvas.
    pub pan: Point<Pixels>,
    /// Zoom factor, clamped to `MIN_ZOOM`..`MAX_ZOOM`.
    pub zoom: f32,
    /// Index of the currently selected node, if any.
    pub selection: Option<usize>,
    /// The node currently being dragged, if any.
    pub node_drag: Option<NodeDrag>,
    /// Whether the graph should be auto-fitted once the canvas reports bounds.
    pub fit_pending: bool,
    /// The ("Direction", "Label scale") pair this tab's `document` was last
    /// measured and laid out with. Kept in sync on load and on settings
    /// re-layout, so a settings change can skip the recompute when it would
    /// not change anything.
    pub applied_layout: Option<(RankDir, f32)>,
}

impl GraphTab {
    /// An empty tab for `path`; the document arrives when the load finishes.
    fn new(id: u64, path: PathBuf) -> Self {
        Self {
            id,
            title: path
                .file_name()
                .map(|name| name.to_string_lossy().into_owned())
                .unwrap_or_else(|| path.display().to_string()),
            path,
            document: None,
            error: None,
            loading: false,
            pan: point(px(0.0), px(0.0)),
            zoom: 1.0,
            selection: None,
            node_drag: None,
            fit_pending: true,
            applied_layout: None,
        }
    }
}

/// An error queued for the centered alert dialog. The async load and
/// re-layout completions run through `AsyncApp`, which cannot reach the
/// window, so a failure parks here and [`GraphView::render`] — which always
/// has the window — opens the dialog once and clears the queue.
struct PendingError {
    /// Dialog title: what failed, naming the file.
    title: String,
    /// The error detail plus the recovery hint shown in the dialog body.
    description: String,
}

/// Root view of the viewer: a strip of [`GraphTab`]s, one visible at a time.
pub struct GraphView {
    /// The open tabs, in strip order. Empty while no file is open.
    tabs: Vec<GraphTab>,
    /// Index into [`Self::tabs`] of the visible tab.
    active: usize,
    /// Monotonic source of [`GraphTab::id`]s.
    next_tab_id: u64,
    /// Canvas bounds in screen pixels, reported back by the canvas each frame.
    pub canvas_bounds: Option<Bounds<Pixels>>,
    /// A file-dialog failure shown on the empty canvas when no tab exists to
    /// carry it; cleared as soon as a file opens.
    pub(crate) dialog_error: Option<String>,
    /// Load or re-layout failures queued for the centered alert dialog,
    /// one dialog per frame ([`PendingError`]). A queue rather than a
    /// slot: two failures landing in the same frame — a re-layout and a
    /// load, say — must not overwrite each other.
    pending_errors: VecDeque<PendingError>,
    /// Whether the tab list's search popup is open. The popup is a controlled
    /// [`gpui_kit::component::popover::Popover`], which owns the rendering;
    /// this only remembers the state so the shortcut can open it and picking a
    /// tab can close it.
    pub(crate) tab_menu_open: bool,
    /// Whether files dragged from the desktop are currently over the window;
    /// shows the drop-target overlay over the canvas.
    file_drag_hover: bool,
    /// Whether the viewer is in fullscreen mode: only the canvas shows, the
    /// title/tab/status bars are hidden. Esc (or F11) exits.
    fullscreen: bool,
    /// The window's outer size captured before entering fullscreen, restored
    /// once the platform has left fullscreen again. Needed because some
    /// Wayland compositors (WSLg's Weston) answer `unset_fullscreen` with a
    /// 0×0 "client decides" configure, which gpui's backend accepts as "keep
    /// the fullscreen size" — without restoring, the window stays screen-sized
    /// and the compositor re-places it (the reported jump in size/position).
    pre_fullscreen_size: Option<Size<Pixels>>,
    /// Bumped each time a fullscreen exit starts a restore, so a slow restore
    /// scheduled by an earlier toggle can never apply a stale size.
    fullscreen_restore_epoch: u64,
    /// The viewer's settings, edited from the status bar's settings card
    /// and quick toggles, and shared by every tab.
    pub settings: Settings,
    /// Where those settings are remembered between runs. `None` when the
    /// platform has no config directory to put them in; the viewer then runs
    /// on its defaults and writes nothing.
    settings_store: Option<SettingsStore>,
    /// Where the pointer was during the last tab drag, so a detached tab's
    /// window can open under it. Written without notifying: only the drop that
    /// ends the gesture reads it, and a repaint per pointer move would cost far
    /// more than the position is worth.
    tab_drag_position: Option<Point<Pixels>>,
    /// Owned focus handle so the root keeps keyboard focus (actions, keys).
    focus_handle: FocusHandle,
}

impl GraphView {
    /// Creates a view with no tabs, over `settings_store`: where it reads the
    /// settings it starts from.
    fn empty(settings_store: Option<SettingsStore>, cx: &mut Context<Self>) -> Self {
        let focus_handle = cx.focus_handle();
        let settings = settings_store
            .as_ref()
            .map(SettingsStore::load)
            .unwrap_or_default();
        Self {
            tabs: Vec::new(),
            active: 0,
            next_tab_id: 0,
            canvas_bounds: None,
            dialog_error: None,
            tab_menu_open: false,
            pending_errors: VecDeque::new(),
            file_drag_hover: false,
            fullscreen: false,
            pre_fullscreen_size: None,
            fullscreen_restore_epoch: 0,
            tab_drag_position: None,
            settings,
            settings_store,
            focus_handle,
        }
    }

    /// Creates a view over `settings_store` and kicks off the load of `file` if
    /// one was given. Initial keyboard focus is applied by the caller (which owns
    /// the window).
    pub fn new(
        file: Option<PathBuf>,
        settings_store: Option<SettingsStore>,
        cx: &mut Context<Self>,
    ) -> Self {
        let mut view = Self::empty(settings_store, cx);
        if let Some(path) = file {
            view.open_path_in_tab(path, cx);
        }
        view
    }

    /// Creates a view whose only tab is `tab` — the receiving end of
    /// [`Self::detach_tab`]. The tab arrives with its document, its pan, zoom and
    /// selection, so a detached window opens exactly as the tab looked. Its id is
    /// carried over, and the new view's id counter starts above it, so a file
    /// opened here later cannot collide with the tab it was given.
    pub fn with_tab(
        tab: GraphTab,
        settings_store: Option<SettingsStore>,
        cx: &mut Context<Self>,
    ) -> Self {
        let next_tab_id = tab.id + 1;
        let mut view = Self::empty(settings_store, cx);
        view.next_tab_id = next_tab_id;
        view.tabs.push(tab);
        view
    }

    /// The visible tab, if any file is open.
    pub fn active(&self) -> Option<&GraphTab> {
        self.tabs.get(self.active)
    }

    /// The open tabs, in strip order (for the tab strip).
    pub fn tabs(&self) -> impl Iterator<Item = &GraphTab> {
        self.tabs.iter()
    }

    /// Index of the visible tab in the strip.
    pub fn active_index(&self) -> usize {
        self.active
    }

    /// The visible tab, mutably. Canvas interaction writes through this.
    pub(crate) fn active_mut(&mut self) -> Option<&mut GraphTab> {
        self.tabs.get_mut(self.active)
    }

    /// The active tab's document, if one is loaded.
    pub fn active_document(&self) -> Option<Rc<Document>> {
        self.active().and_then(|tab| tab.document.clone())
    }

    /// Opens `path` in its own tab and makes it visible — or just activates
    /// the tab that already shows this file. The load itself runs off the
    /// main thread.
    pub fn open_path_in_tab(&mut self, path: PathBuf, cx: &mut Context<Self>) {
        self.dialog_error = None;
        if let Some(index) = self.tabs.iter().position(|tab| tab.path == path) {
            self.activate(index, cx);
            return;
        }
        let id = self.next_tab_id;
        self.next_tab_id += 1;
        self.tabs.push(GraphTab::new(id, path.clone()));
        self.activate(self.tabs.len() - 1, cx);
        self.load_into_tab(id, path, cx);
    }

    /// Makes `index` the visible tab. A tab last laid out under older
    /// settings re-lays-out on activation.
    pub(crate) fn activate(&mut self, index: usize, cx: &mut Context<Self>) {
        if index < self.tabs.len() && index != self.active {
            self.active = index;
            self.relayout_for_settings(cx);
            cx.notify();
        }
    }

    /// Closes the tab with `tab_id`; the neighbor that takes its slot stays
    /// active. Closing the last tab returns to the empty window.
    pub(crate) fn close_tab(&mut self, tab_id: u64, cx: &mut Context<Self>) {
        let Some(index) = self.tabs.iter().position(|tab| tab.id == tab_id) else {
            return;
        };
        self.tabs.remove(index);
        self.active = active_after_close(self.active, self.tabs.len() + 1, index);
        // The tab that takes the closed one's slot was laid out under
        // whatever settings were current back then, which need not be the
        // current ones: re-apply them, exactly as `activate` does. Without
        // this the drawing disagrees with the settings the toggles show.
        self.relayout_for_settings(cx);
        cx.notify();
    }

    /// Makes the open tab with `tab_id` visible, if it is still open. The
    /// strip works by index; the tab-list dropdown holds ids, which survive
    /// a tab being closed while its menu is open.
    pub(crate) fn activate_tab_id(&mut self, tab_id: u64, cx: &mut Context<Self>) {
        if let Some(index) = self.tabs.iter().position(|tab| tab.id == tab_id) {
            self.activate(index, cx);
        }
    }

    /// Hands the tab with `tab_id` to a window of its own, opened where the drag
    /// that asked for it left off.
    ///
    /// The document moves rather than reloading: a detach must not depend on the
    /// file still being readable, and it must not re-run a layout. The tab is
    /// cloned into the new window first and dropped from this one only once that
    /// window exists, so a platform that refuses to open one leaves the tab where
    /// it was rather than losing it.
    pub(crate) fn detach_tab(&mut self, tab_id: u64, window: &mut Window, cx: &mut Context<Self>) {
        let Some(index) = self.tabs.iter().position(|tab| tab.id == tab_id) else {
            return;
        };
        // A tab whose drawing has not arrived yet has nothing to hand over, and
        // the load it is waiting for belongs to this view.
        if self.tabs[index].document.is_none() {
            return;
        }

        let window_bounds = window.bounds();
        // Put the new window where the gesture ended, with the pointer over the
        // slot its tab will take, rather than letting the compositor place it.
        let origin = self
            .tab_drag_position
            .map(|pointer| window_bounds.origin + pointer - point(px(156.), px(17.)))
            .unwrap_or(window_bounds.origin);

        let detached = self.tabs[index].clone();
        let settings_store = self.settings_store.clone();
        let opened = cx.open_window(
            window_options(window_bounds.size, origin),
            move |window, cx| {
                let view = cx.new(|cx| GraphView::with_tab(detached, settings_store, cx));
                let focus_handle = view.focus_handle(cx);
                window.focus(&focus_handle, cx);
                cx.new(|cx| gpui_kit::component::Root::new(view, window, cx))
            },
        );
        if opened.is_err() {
            eprintln!("dotv: could not open a window for the detached tab; it stays here");
            return;
        }

        // The window is up: the tab belongs to it now.
        self.tabs.remove(index);
        self.active = active_after_close(self.active, self.tabs.len() + 1, index);
        if self.tabs.is_empty() {
            // Nothing left to show, and an empty window is only noise: this is
            // what Chrome does when its last tab leaves for another window.
            window.remove_window();
        } else {
            self.relayout_for_settings(cx);
            cx.notify();
        }
    }

    /// Moves the tab with `tab_id` into slot `to` of the strip.
    ///
    /// `active` is a position, so it has to be re-pointed at whatever tab it
    /// named before the move: dragging a tab must never change which drawing is
    /// on screen.
    pub(crate) fn move_tab(&mut self, tab_id: u64, to: usize, cx: &mut Context<Self>) {
        let Some(from) = self.tabs.iter().position(|tab| tab.id == tab_id) else {
            return;
        };
        let to = to.min(self.tabs.len().saturating_sub(1));
        if from == to {
            return;
        }
        let visible = self.active().map(|tab| tab.id);
        let tab = self.tabs.remove(from);
        self.tabs.insert(to, tab);
        if let Some(visible) = visible
            && let Some(index) = self.tabs.iter().position(|tab| tab.id == visible)
        {
            self.active = index;
        }
        cx.notify();
    }

    /// Steps the active tab by `step` (wrapping around both ends).
    pub(crate) fn cycle_tab(&mut self, step: isize, cx: &mut Context<Self>) {
        if self.tabs.is_empty() {
            return;
        }
        let next = (self.active as isize + step).rem_euclid(self.tabs.len() as isize) as usize;
        self.activate(next, cx);
    }

    /// Enters or leaves fullscreen: the canvas alone fills the window, and
    /// the platform window follows, so the viewer state and the actual
    /// window state stay in sync (Esc, F11 and the title-bar button can all
    /// change either side).
    pub fn set_fullscreen(
        &mut self,
        fullscreen: bool,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        self.fullscreen = fullscreen;
        if window.is_fullscreen() != fullscreen {
            if fullscreen {
                // Remember the pre-fullscreen size so the exit can restore it
                // (see [`Self::restore_pre_fullscreen_size`]). A maximized
                // window stores `None` instead: the compositor restores the
                // maximize state and its geometry itself, and leaving a
                // value from an earlier session here would let a stale size
                // win later.
                self.pre_fullscreen_size =
                    (!window.is_maximized()).then(|| window.bounds().size);
            }
            window.toggle_fullscreen();
            if !fullscreen {
                self.restore_pre_fullscreen_size(window, cx);
            }
        }
        cx.notify();
    }

    /// Puts the window back to its pre-fullscreen size after leaving
    /// fullscreen. The Wayland exit is asynchronous — the compositor confirms
    /// `unset_fullscreen` with a configure a frame or two later — and some
    /// compositors (WSLg's Weston) send a 0×0 "client decides" size, which
    /// gpui's backend accepts as "stay at the fullscreen size". So we wait
    /// for the exit to settle and, if the window didn't come back to the
    /// recorded size, resize it ourselves; on compositors that restore
    /// correctly this is a no-op. Bounded so a stuck platform can't spin.
    fn restore_pre_fullscreen_size(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(saved) = self.pre_fullscreen_size else {
            return;
        };
        self.fullscreen_restore_epoch += 1;
        let epoch = self.fullscreen_restore_epoch;
        cx.spawn_in(window, async move |view, cx| {
            for _ in 0..120 {
                cx.background_executor()
                    .timer(Duration::from_millis(15))
                    .await;
                let mut settled = false;
                let _ = view.update_in(cx, |this, window, _cx| {
                    // A newer toggle owns the restore now; this one retires.
                    if this.fullscreen_restore_epoch != epoch {
                        settled = true;
                        return;
                    }
                    // Wait for the platform to process the exit configure
                    // (`is_fullscreen` lags the request by one round-trip).
                    // While maximized, the compositor's own restore path
                    // governs the size and we stay out of its way.
                    if window.is_fullscreen() || window.is_maximized() {
                        return;
                    }
                    if window.bounds().size != saved {
                        window.resize(saved);
                    }
                    this.pre_fullscreen_size = None;
                    settled = true;
                });
                if settled {
                    return;
                }
            }
        })
        .detach();
    }

    /// Loads `path` on the background executor into the tab with `tab_id`
    /// and applies the result on a later frame. The tab shows the loading
    /// animation until then; a failure reports through the centered error
    /// dialog and stays on the tab as canvas text.
    fn load_into_tab(&mut self, tab_id: u64, path: PathBuf, cx: &mut Context<Self>) {
        if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.id == tab_id) {
            tab.loading = true;
        }
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            // The shared font database is picked up on the main thread; label
            // shaping on the background thread runs in its own thread-safe
            // layout cache around it, so the layout always matches the fonts
            // the canvas renders with.
            let text_system = cx.update(|app: &mut App| app.text_system().clone());
            let computed = cx
                .update(|app: &mut App| {
                    app.background_spawn(async move {
                        let text_system = WindowTextSystem::new(text_system);
                        load_parts(path.clone(), &text_system)
                    })
                })
                .await;
            let _ = this.update(cx, |view, cx| view.load_finished(tab_id, computed, cx));
        })
        .detach();
    }

    /// Applies a finished load to the tab with `tab_id`: the drawing
    /// replaces the loading animation, or the failure is queued as the
    /// centered error dialog and recorded on the tab. A result for a tab
    /// that was closed meanwhile is dropped.
    fn load_finished(
        &mut self,
        tab_id: u64,
        result: Result<LoadOutput, LoadError>,
        cx: &mut Context<Self>,
    ) {
        let Some(index) = self.tabs.iter().position(|tab| tab.id == tab_id) else {
            return;
        };
        self.tabs[index].loading = false;
        match result {
            Ok(parts) => {
                // The fresh measurements are at the base label scale; if the
                // user's "Label scale" preference differs, the relayout below
                // re-measures and re-lays-out so the drawing matches the
                // setting.
                let document = Rc::new(Document::from_parts(parts));
                let dir = document.rank_dir();
                let tab = &mut self.tabs[index];
                tab.document = Some(document);
                tab.error = None;
                tab.selection = None;
                tab.node_drag = None;
                tab.fit_pending = true;
                tab.applied_layout = Some((dir, 1.0));
                // The direction preference follows the file the user just
                // opened — but only while it is still the tab on screen; a
                // tab opened in the background keeps the current preference
                // until it is activated.
                if self.active == index {
                    // The direction follows the file that was just opened, and is
                    // remembered like any other setting. Leaving it out of the
                    // store instead would make the stored direction nearly
                    // meaningless: opening any file would overwrite the user's
                    // pick in memory, and the next session would start from a
                    // value nothing on screen ever showed. This way the toggle
                    // reads the same at the start of a session as it did at the
                    // end of the last one.
                    self.update_settings(|settings| {
                        settings.rank_dir = match dir {
                            RankDir::LR => "LR".into(),
                            RankDir::TB => "TB".into(),
                        };
                    });
                    // Only the tab on screen follows the settings here; a
                    // background tab keeps the base layout until it is
                    // activated, which re-applies them (`activate`).
                    // Re-laying out the *active* tab on a background
                    // completion would touch the wrong document.
                    self.relayout_for_settings(cx);
                }
            }
            Err(err) => {
                let file = self.tabs[index].path.display().to_string();
                self.tabs[index].error = Some(format!("Failed to open {file}: {err}"));
                // The title names the file; the body says what went wrong and
                // how to recover, without repeating the path the title
                // carries.
                let hint = match &err {
                    LoadError::Parse(_) | LoadError::Layout(_) => {
                        "Check the DOT source and open the file again."
                    }
                    LoadError::Io(_) => "Check that the file exists and is readable.",
                };
                self.pending_errors.push_back(PendingError {
                    title: format!(r#"Couldn’t open “{file}”"#),
                    description: format!("{err} {hint}"),
                });
            }
        }
        cx.notify();
    }

    /// Opens a file picker and loads the picked file in a new tab.
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
            .active_document()
            .and_then(|document| document.path.clone())
            .and_then(|path| path.parent().map(Path::to_path_buf));
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
                    let _ = this.update(cx, |view, cx| view.open_path_in_tab(path, cx));
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
                    // With no tab open there is nothing to carry the failure,
                    // so it shows on the empty canvas instead.
                    match view.active_mut() {
                        Some(tab) => tab.error = Some(message),
                        None => view.dialog_error = Some(message),
                    }
                    cx.notify();
                });
                return;
            }
            _ => return, // cancelled
        };
        let _ = this.update(cx, |view, cx| view.open_path_in_tab(picked, cx));
    }

    /// Opens files dropped from the desktop onto the window.
    pub fn handle_dropped_paths(
        &mut self,
        paths: &ExternalPaths,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        self.load_first_dot(paths.clone(), window, cx);
    }

    /// Opens the .dot file whose path sits on the clipboard, as text or as a
    /// native clipboard file entry. This is the reliable substitute for
    /// dragging a file in from Windows: WSLg's RDP clipboard forwards text
    /// (and, when running natively on Windows, file entries) but has no
    /// drag-and-drop channel, so Explorer's Ctrl+C + dotv's Ctrl+V is the
    /// cross-system path in.
    pub fn paste_graph(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(item) = cx.read_from_clipboard() else {
            self.notify_paste_empty(window, cx);
            return;
        };
        let paths = item
            .entries
            .iter()
            .flat_map(|entry| match entry {
                ClipboardEntry::ExternalPaths(paths) => paths.paths().to_vec(),
                ClipboardEntry::String(text) => parse_pasted_paths(text.text()),
                _ => Vec::new(),
            })
            .collect::<Vec<_>>();
        if paths.is_empty() {
            self.notify_paste_empty(window, cx);
            return;
        }
        self.load_first_dot(ExternalPaths(paths.into()), window, cx);
    }

    /// Warning shown when the clipboard holds no usable file path.
    fn notify_paste_empty(&mut self, window: &mut Window, _cx: &mut Context<Self>) {
        window.push_notification(
            Notification::warning(
                "No file path on the clipboard — copy a .dot or .gv file (Ctrl+C) and paste here",
            ),
            _cx,
        );
    }

    /// Opens the first path that names a DOT source file, or leaves the
    /// current tabs alone and says so, as a notification.
    fn load_first_dot(
        &mut self,
        paths: ExternalPaths,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        match Self::pick_dot_file(&paths) {
            Some(path) => self.open_path_in_tab(path, cx),
            None => window.push_notification(
                Notification::warning("Not a DOT file — drop a .dot or .gv file to open it"),
                cx,
            ),
        }
    }

    /// The first dropped path that names a DOT source file, if any.
    /// Extensions are case-insensitive (`graph.DOT` counts); directories and
    /// other files never match, so an unsupported drop leaves the current
    /// document untouched.
    fn pick_dot_file(paths: &ExternalPaths) -> Option<PathBuf> {
        paths
            .paths()
            .iter()
            .find(|path| {
                matches!(
                    path.extension().and_then(|ext| ext.to_str()),
                    Some(ext) if ext.eq_ignore_ascii_case("dot") || ext.eq_ignore_ascii_case("gv")
                )
            })
            .cloned()
    }

    /// Undoes every manual node move in the visible tab: the drawing snaps
    /// back to the positions the layout computed. No-op when nothing has
    /// been dragged.
    pub fn reset_positions(&mut self, cx: &mut Context<Self>) {
        let Some(tab) = self.active_mut() else {
            return;
        };
        let Some(document) = tab.document.clone() else {
            return;
        };
        if let Some(cleared) = document.clear_offsets() {
            tab.document = Some(Rc::new(cleared));
            tab.node_drag = None;
            cx.notify();
        }
    }

    pub fn zoom_in(&mut self, cx: &mut Context<Self>) {
        let zoom = self.active().map_or(1.0, |tab| tab.zoom);
        self.set_zoom(zoom * 1.25, cx);
    }

    pub fn zoom_out(&mut self, cx: &mut Context<Self>) {
        let zoom = self.active().map_or(1.0, |tab| tab.zoom);
        self.set_zoom(zoom / 1.25, cx);
    }

    /// Applies `change` to the settings and writes them out if it moved
    /// anything.
    ///
    /// Every settings mutation goes through here — the status bar toggles, the
    /// settings card, whatever is added next — so that "the viewer remembers
    /// what you set" cannot be forgotten at one call site. `Settings` compares
    /// by value, so a control that reports the value it already had does not
    /// rewrite the file.
    pub(crate) fn update_settings(&mut self, change: impl FnOnce(&mut Settings)) {
        let before = self.settings.clone();
        change(&mut self.settings);
        if self.settings != before {
            self.persist_settings();
        }
    }

    /// Writes the settings out, or reports why it could not. A config
    /// directory that is missing and cannot be created, or read-only, costs
    /// the viewer its memory of the settings and nothing else — so the failure
    /// is reported and otherwise swallowed rather than interrupting the user.
    fn persist_settings(&self) {
        let Some(store) = self.settings_store.as_ref() else {
            return;
        };
        if let Err(err) = store.save(&self.settings) {
            eprintln!(
                "dotv: could not save settings to {}: {err}",
                store.path().display()
            );
        }
    }

    /// Applies the layout-affecting settings ("Direction", "Label scale") to
    /// the visible tab's document: re-measures the labels at the chosen
    /// scale and re-runs the layout in the chosen direction on the background
    /// executor, then swaps the new snapshot in and refits. The previous
    /// drawing stays visible and interactive meanwhile; the status bar shows
    /// the wait. No-op when there is no document or when the settings already
    /// match what the document was built with (tracked per tab in
    /// [`GraphTab::applied_layout`]).
    pub fn relayout_for_settings(&mut self, cx: &mut Context<Self>) {
        let Some(document) = self.active_document() else {
            return;
        };
        let Some(tab_id) = self.active().map(|tab| tab.id) else {
            return;
        };
        let rank_dir = match self.settings.rank_dir.as_str() {
            "LR" => RankDir::LR,
            _ => RankDir::TB,
        };
        let label_scale = self.settings.label_scale as f32;
        if self.active().and_then(|tab| tab.applied_layout) == Some((rank_dir, label_scale)) {
            return;
        }
        // The engine lays out an owned copy of the graph: `Rc` cannot cross
        // a thread boundary.
        let graph = (*document.graph).clone();
        if let Some(tab) = self.tabs.iter_mut().find(|tab| tab.id == tab_id) {
            tab.loading = true;
        }
        cx.notify();
        cx.spawn(async move |this, cx: &mut AsyncApp| {
            // The shared font database is picked up on the main thread; label
            // shaping on the background thread runs in its own thread-safe
            // layout cache around it.
            let text_system = cx.update(|app: &mut App| app.text_system().clone());
            let computed = cx
                .update(|app: &mut App| {
                    app.background_spawn(async move {
                        let text_system = WindowTextSystem::new(text_system);
                        relayout_graph(&graph, rank_dir, label_scale, &text_system)
                    })
                })
                .await;
            let _ = this.update(cx, |view, cx| {
                view.relayout_finished(tab_id, rank_dir, label_scale, computed, cx)
            });
        })
        .detach();
    }

    /// Applies a finished re-layout to the tab with `tab_id`. A result whose
    /// parameters no longer match the current settings is stale (the user
    /// changed them again while this ran) and is dropped — the newest
    /// relayout replaces it. A failure keeps the previous drawing and is
    /// reported in the centered error dialog.
    fn relayout_finished(
        &mut self,
        tab_id: u64,
        rank_dir: RankDir,
        label_scale: f32,
        result: Result<DotView, LoadError>,
        cx: &mut Context<Self>,
    ) {
        // Ignore a stale result: the settings may have changed again while
        // the relayout was running. The newest relayout spawns from the
        // newest settings, so only let it land if it matches what the user
        // currently has selected.
        let current_dir = match self.settings.rank_dir.as_str() {
            "LR" => RankDir::LR,
            _ => RankDir::TB,
        };
        if current_dir != rank_dir || (self.settings.label_scale as f32 - label_scale).abs() > 1e-3
        {
            return;
        }
        let Some(index) = self.tabs.iter().position(|tab| tab.id == tab_id) else {
            return;
        };
        self.tabs[index].loading = false;
        match result {
            Ok(view) => {
                let Some(document) = self.tabs[index].document.clone() else {
                    return;
                };
                let tab = &mut self.tabs[index];
                tab.document = Some(Rc::new(document.with_view(view)));
                tab.applied_layout = Some((rank_dir, label_scale));
                // The fresh layout positions every node from scratch (and
                // resets its drag offsets); a drag in flight would fight it.
                tab.node_drag = None;
                tab.fit_pending = true;
            }
            Err(err) => {
                let file = self.tabs[index].path.display().to_string();
                // The previous drawing stays on screen; the dialog says why
                // the settings choice could not be applied.
                self.pending_errors.push_back(PendingError {
                    title: format!(r#"Couldn’t lay out “{file}”"#),
                    description: format!("{err} The previous drawing is still shown."),
                });
            }
        }
        cx.notify();
    }

    fn set_zoom(&mut self, target: f32, cx: &mut Context<Self>) {
        let Some(tab) = self.active_mut() else {
            return;
        };
        let next = target.clamp(crate::viz::MIN_ZOOM, crate::viz::MAX_ZOOM);
        if (next - tab.zoom).abs() > 1e-6 {
            tab.zoom = next;
            cx.notify();
        }
    }

    /// Fits the graph inside the canvas; when the canvas has not reported its
    /// bounds yet, defers the fit by keeping `fit_pending` set.
    pub fn fit_graph(&mut self, cx: &mut Context<Self>) {
        if self.compute_fit() {
            if let Some(tab) = self.active_mut() {
                tab.fit_pending = false;
            }
            cx.notify();
        } else if let Some(tab) = self.active_mut() {
            tab.fit_pending = true;
        }
    }

    /// Computes pan/zoom so the whole layout fits the canvas with a margin.
    /// Returns `false` when there is nothing to fit yet (no document or no
    /// canvas bounds). Does not notify — callers decide whether to repaint.
    /// The fit box covers the laid-out bounds plus every node a drag offset
    /// has moved (see [`effective_bounds`]), so dragged nodes stay visible.
    fn compute_fit(&mut self) -> bool {
        let Some(bounds) = self.canvas_bounds else {
            return false;
        };
        let Some(tab) = self.active_mut() else {
            return false;
        };
        let Some(document) = tab.document.as_ref() else {
            return false;
        };
        let (min_x, min_y, max_x, max_y) = document.effective_bounds();
        let layout_w = (max_x - min_x).max(1.0);
        let layout_h = (max_y - min_y).max(1.0);
        let inset = 24.0_f32;
        let canvas_w = (bounds.size.width.as_f32() - inset * 2.0).max(1.0);
        let canvas_h = (bounds.size.height.as_f32() - inset * 2.0).max(1.0);
        let fitted = (canvas_w / layout_w).min(canvas_h / layout_h);
        tab.zoom = fitted.clamp(crate::viz::MIN_ZOOM, crate::viz::MAX_ZOOM);
        // Center the content's actual box (its bounds center, which is not
        // guaranteed to sit at the world origin) on the canvas center.
        let canvas_cx = bounds.size.width.as_f32() / 2.0;
        let canvas_cy = bounds.size.height.as_f32() / 2.0;
        let content_cx = (min_x + max_x) / 2.0;
        let content_cy = (min_y + max_y) / 2.0;
        tab.pan = point(
            px(canvas_cx - content_cx * tab.zoom),
            px(canvas_cy - content_cy * tab.zoom),
        );
        true
    }

    /// Error message shown on the canvas when the visible tab failed to
    /// load. Empty and loading states render the empty-state overlay instead.
    fn canvas_message(&self) -> Option<SharedString> {
        self.active()?
            .error
            .as_ref()
            .map(|error| SharedString::from(error.as_str()))
    }
}

/// The active index after closing the tab at `removed` from a strip of `len`
/// tabs: a removed tab before the active one shifts it left, a removed active
/// (or later) tab keeps the successor (or the same index) visible.
fn active_after_close(active: usize, len: usize, removed: usize) -> usize {
    let Some(last) = len.checked_sub(2) else {
        return 0; // nothing left open
    };
    match removed.cmp(&active) {
        std::cmp::Ordering::Less => active - 1,
        _ => active.min(last),
    }
}

impl Focusable for GraphView {
    fn focus_handle(&self, _cx: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl Render for GraphView {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        // A queued load or re-layout failure opens the centered alert dialog
        // here, where the window is at hand — the async completions that
        // enqueue it cannot reach the window. Taking the entry keeps this a
        // one-shot side effect, so no notify loop can form.
        if let Some(failure) = self.pending_errors.pop_front() {
            let title = SharedString::from(failure.title);
            let description = SharedString::from(failure.description);
            window.open_alert_dialog(cx, move |alert, _, _| {
                alert.title(title.clone()).description(description.clone())
            });
            // One dialog per frame, so make sure the frame that shows
            // the rest of the queue actually arrives.
            if !self.pending_errors.is_empty() {
                cx.notify();
            }
        }
        // On the first frame after a load, fit once the canvas has real bounds.
        if self.active().is_some_and(|tab| tab.fit_pending)
            && self.compute_fit()
            && let Some(tab) = self.active_mut()
        {
            tab.fit_pending = false;
        }
        let weak = cx.weak_entity();
        let active = self.tabs.get(self.active);
        let fullscreen = self.fullscreen;
        // The loading overlay takes over while a first load is still parsing
        // and laying out; the empty overlay takes over when the visible tab
        // has nothing to show and no error to report.
        let loading_tab = active.filter(|tab| tab.loading && tab.document.is_none());
        let show_empty = active
            .is_some_and(|tab| tab.document.is_none() && tab.error.is_none() && !tab.loading)
            || active.is_none();

        use gpui_kit::base::StyledExt as _;
        use gpui_kit::component::Root;
        use gpui_kit::{ParentElement, Styled, div};

        div()
            .id("dotv-app")
            .size_full()
            .v_flex()
            .track_focus(&self.focus_handle)
            // The whole window is a drop target: files dragged from the
            // desktop open wherever they land. GPUI translates the platform
            // file-drag into an `ExternalPaths` drag, so the overlay state
            // rides the normal drag-move / drop / exit events.
            .on_drag_move(
                cx.listener(|this, _: &DragMoveEvent<ExternalPaths>, _, cx| {
                    if !this.file_drag_hover {
                        this.file_drag_hover = true;
                        cx.notify();
                    }
                }),
            )
            .on_file_drop_exit(cx.listener(|this, _: &FileDropEvent, _, cx| {
                if this.file_drag_hover {
                    this.file_drag_hover = false;
                    cx.notify();
                }
            }))
            .on_drop(cx.listener(|this, paths: &ExternalPaths, window, cx| {
                this.file_drag_hover = false;
                this.handle_dropped_paths(paths, window, cx);
            }))
            // A tab dragged out of the strip and let go anywhere else gets a
            // window of its own — what dropping a tab on the page does in
            // Chrome. The payload's own type keeps this apart from the file
            // drop above, and the drag-move below only records where the
            // pointer is, so the window opens where the gesture ended.
            .on_drag_move(cx.listener(|this, event: &DragMoveEvent<TabDrag>, _, _| {
                this.tab_drag_position = Some(event.event.position);
            }))
            .on_drop(cx.listener(|this, drag: &TabDrag, window, cx| {
                this.detach_tab(drag.tab_id, window, cx);
            }))
            .on_action(cx.listener(|this, _: &OpenGraph, _window, cx| this.open_graph(cx)))
            .on_action(cx.listener(|this, _: &PasteGraph, window, cx| this.paste_graph(window, cx)))
            .on_action(cx.listener(|this, _: &SearchTabs, _window, cx| {
                this.tab_menu_open = true;
                cx.notify();
            }))
            .on_action(cx.listener(|this, _: &ToggleFullscreen, window, cx| {
                this.set_fullscreen(!this.fullscreen, window, cx)
            }))
            .on_action(cx.listener(|this, _: &CloseTab, _window, cx| {
                if let Some(tab) = this.active() {
                    let id = tab.id;
                    this.close_tab(id, cx);
                }
            }))
            .on_action(cx.listener(|this, _: &NextTab, _window, cx| this.cycle_tab(1, cx)))
            .on_action(cx.listener(|this, _: &PrevTab, _window, cx| this.cycle_tab(-1, cx)))
            .on_action(cx.listener(|this, _: &ZoomIn, _window, cx| this.zoom_in(cx)))
            .on_action(cx.listener(|this, _: &ZoomOut, _window, cx| this.zoom_out(cx)))
            .on_action(cx.listener(|this, _: &FitGraph, _window, cx| this.fit_graph(cx)))
            .on_action(cx.listener(|this, _: &ClearSelection, window, cx| {
                // Esc means "leave fullscreen" first; inside the normal
                // chrome it drops the selection (and any drag in flight),
                // while committed node positions stay where the user put
                // them.
                if this.fullscreen {
                    this.set_fullscreen(false, window, cx);
                    return;
                }
                if let Some(tab) = this.active_mut()
                    && (tab.selection.take().is_some() || tab.node_drag.take().is_some())
                {
                    cx.notify();
                }
            }))
            // Fullscreen shows the canvas alone; the chrome returns on Esc.
            // The title bar and the tab strip share this one row, so the
            // strip no longer needs a row of its own above the canvas. The
            // strip offers its `+` only once a file is open; an empty window
            // uses the canvas' own open button instead.
            .when(!fullscreen, |app| app.child(title_bar(self, window, cx)))
            .child(
                div()
                    .id("dotv-canvas")
                    .relative()
                    .flex_1()
                    .min_h_0()
                    // Clip the canvas area to its layout box, which sits
                    // strictly between the tab strip and the status bar.
                    .overflow_hidden()
                    .child(
                        GraphCanvas::new(active.and_then(|tab| tab.document.clone()), weak)
                            .message(self.canvas_message())
                            .pan(active.map_or(point(px(0.0), px(0.0)), |tab| tab.pan))
                            .zoom(active.map_or(1.0, |tab| tab.zoom))
                            .selection(active.and_then(|tab| tab.selection))
                            .node_drag(active.and_then(|tab| tab.node_drag))
                            .show_node_labels(self.settings.show_node_labels)
                            .show_edge_labels(self.settings.show_edge_labels)
                            .label_scale(
                                active
                                    .and_then(|tab| tab.applied_layout)
                                    .map_or(1.0, |(_, scale)| scale),
                            )
                            .show_grid(self.settings.show_grid),
                    )
                    .when(show_empty, |canvas| canvas.child(empty_state(self, cx)))
                    .when_some(loading_tab, |canvas, tab| {
                        canvas.child(loading_overlay(tab, cx))
                    })
                    // The zoom controls only make sense with a drawing on
                    // screen: in the empty state, and under the first-load
                    // veil, they would sit on top of it — clickable, and
                    // reading a zoom for a graph that is not there.
                    .when(active.is_some_and(|tab| tab.document.is_some()), |canvas| {
                        canvas.child(zoom_cluster(self, cx))
                    })
                    .when(self.file_drag_hover, |canvas| {
                        canvas.child(drop_overlay(cx))
                    }),
            )
            .when(!fullscreen, |app| app.child(status_bar(self, cx)))
            .children(Root::render_dialog_layer(window, cx))
            .children(Root::render_sheet_layer(window, cx))
            .children(Root::render_notification_layer(window, cx))
            // Registered so the UI tests can drop a dragged tab on the
            // window itself: the drawing is the detach zone.
            .test_support()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The Kit imports for the UI integration tests are scoped here and
    // deliberately not glob imports (a gpui-kit glob would shadow Rust's
    // `#[test]`).
    use gpui_kit::TestAppContext;
    use gpui_kit::component::Root;
    use gpui_kit::size;
    use gpui_kit::test::TestWindowExt as _;
    use gpui_kit::{AnyWindowHandle, Entity};

    /// The scratch settings path a test's fixture name maps to. Nothing is
    /// read or written here, so a test that wants to inspect what a viewer
    /// wrote can ask for the same path without clearing it.
    fn scratch_settings_path(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("dotv-settings-test-{name}.json"))
    }

    /// A settings file for one test, cleared first so a leftover from an
    /// earlier run cannot change what the test starts from — and on a scratch
    /// path, so no test ever reads or rewrites the real user's settings.
    fn scratch_settings(name: &str) -> SettingsStore {
        let path = scratch_settings_path(name);
        let _ = std::fs::remove_file(&path);
        SettingsStore::at(path)
    }

    /// Opens a headless window over the production root view, loading
    /// `source` written to a fixture file.
    fn open_viewer(
        source: &str,
        name: &str,
        cx: &mut TestAppContext,
    ) -> (Entity<GraphView>, AnyWindowHandle) {
        let path = std::env::temp_dir().join(name);
        std::fs::write(&path, source).expect("write the DOT fixture");
        cx.update(gpui_kit::init);
        let mut view = None;
        let window = cx.open_window(size(px(640.), px(480.)), |window, cx| {
            let root = cx.new(|cx| {
                GraphView::new(Some(path.clone()), Some(scratch_settings(name)), cx)
            });
            view = Some(root.clone());
            Root::new(root, window, cx)
        });
        (view.expect("the view is built"), window.into())
    }

    fn paths(paths: &[&str]) -> ExternalPaths {
        ExternalPaths(paths.iter().map(PathBuf::from).collect())
    }

    #[test]
    fn drop_picks_dot_files_case_insensitively() {
        assert_eq!(
            GraphView::pick_dot_file(&paths(&["/tmp/graph.dot"])),
            Some(PathBuf::from("/tmp/graph.dot"))
        );
        assert_eq!(
            GraphView::pick_dot_file(&paths(&["/tmp/graph.GV"])),
            Some(PathBuf::from("/tmp/graph.GV"))
        );
    }

    #[test]
    fn drop_skips_other_files_and_picks_the_dot_among_many() {
        assert_eq!(GraphView::pick_dot_file(&paths(&["/tmp/notes.txt"])), None);
        assert_eq!(
            GraphView::pick_dot_file(&paths(&["/tmp/notes.txt", "/tmp/readme.md", "/tmp/a.dot"])),
            Some(PathBuf::from("/tmp/a.dot"))
        );
    }

    #[test]
    fn drop_rejects_extension_less_paths_so_directories_fail_softly() {
        assert_eq!(GraphView::pick_dot_file(&paths(&["/tmp"])), None);
        assert_eq!(GraphView::pick_dot_file(&ExternalPaths::default()), None);
    }

    #[test]
    fn closing_a_tab_keeps_a_sensible_neighbor_active() {
        // Closing the active tab: the successor takes its slot…
        assert_eq!(active_after_close(1, 3, 1), 1);
        // …or the previous one when the last tab was closed.
        assert_eq!(active_after_close(2, 3, 2), 1);
        // A tab closed before the active one shifts the index left.
        assert_eq!(active_after_close(2, 3, 0), 1);
        assert_eq!(active_after_close(1, 3, 0), 0);
        // A tab closed after the active one changes nothing.
        assert_eq!(active_after_close(0, 3, 2), 0);
        // Closing the last open tab empties the strip.
        assert_eq!(active_after_close(0, 1, 0), 0);
    }

    /// A valid file loads through the background pipeline: the loading
    /// animation covers the canvas while the layout runs, ends the frame the
    /// layout lands, and no error dialog ever opens.
    #[gpui_kit::test]
    fn a_good_file_shows_the_loading_animation_until_the_layout_lands(cx: &mut TestAppContext) {
        let (view, window) =
            open_viewer("digraph { a -> b; b -> c; }", "dotv-ui-test-good.dot", cx);

        // Before the background load lands, the tab is loading: the
        // animation covers the canvas and nothing is reported.
        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.try_find("dotv-loading-overlay").is_some(),
                "the loading animation covers the canvas during the first load"
            );
            assert!(!window.has_active_dialog(cx));
        })
        .unwrap();

        // The layout lands: the animation ends and the drawing takes over.
        cx.run_until_parked();
        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.try_find("dotv-loading-overlay").is_none(),
                "the loading animation ends when the layout completes"
            );
            assert!(!window.has_active_dialog(cx));
        })
        .unwrap();
        cx.update(|cx| {
            let tab = view.read(cx).active().expect("the tab stays open");
            assert!(tab.document.is_some(), "the drawing replaced the animation");
            assert!(!tab.loading);
        });
    }

    /// A file with a syntax error reports the failure in a centered dialog
    /// naming the file; dismissing it keeps the error recorded on the tab.
    #[gpui_kit::test]
    fn a_broken_file_reports_in_a_centered_dialog(cx: &mut TestAppContext) {
        let (view, window) = open_viewer("digraph { a -> ", "dotv-ui-test-broken.dot", cx);

        // The load lands on the background executor; the failure then opens
        // the dialog on the next frame.
        cx.run_until_parked();
        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.has_active_dialog(cx),
                "the parse failure opens a dialog"
            );
            assert!(
                window.try_find("dotv-loading-overlay").is_none(),
                "no animation covers the canvas once the failure is reported"
            );
            // Dismiss through the dialog's OK button.
            window.within("dialog").click("ok", cx);
            window.render_frame(cx);
            assert!(!window.has_active_dialog(cx), "OK dismisses the dialog");
        })
        .unwrap();

        // The tab carries the failure after the dialog is gone, and no
        // document replaced the animation.
        cx.update(|cx| {
            let tab = view.read(cx).active().expect("the broken tab stays open");
            assert!(tab.document.is_none());
            assert!(tab.error.is_some(), "the tab keeps the failure");
            assert!(!tab.loading);
        });
    }

    /// The tab strip shares the title bar's row, so a click on a control up
    /// there must still reach that control: the strip's hitboxes block the
    /// mouse deliberately, which keeps the title bar's caption/drag region
    /// from claiming the press (see `ui::tab_bar`). A close button is the
    /// sharpest probe — it is an ordinary button sitting inside a tab, and
    /// whether it fires is directly observable in the tab count. Clicking
    /// the one on an *idle* tab also pins that a background tab can be closed
    /// without being switched to first.
    #[gpui_kit::test]
    fn a_tab_control_in_the_title_bar_receives_clicks(cx: &mut TestAppContext) {
        let (view, window) =
            open_viewer("digraph { a -> b; }", "dotv-ui-test-tab-a.dot", cx);
        cx.run_until_parked();

        // A second file opens its own tab and takes over as the active one.
        let second = std::env::temp_dir().join("dotv-ui-test-tab-b.dot");
        std::fs::write(&second, "digraph { b -> c; }").expect("write the fixture");
        cx.update(|cx| {
            view.update(cx, |view, cx| view.open_path_in_tab(second, cx));
        });
        cx.run_until_parked();
        let (idle, active) = cx.update(|cx| {
            let view = view.read(cx);
            let mut tabs = view.tabs();
            let idle = tabs.next().expect("the first tab").id;
            let active = tabs.next().expect("the second tab").id;
            assert_eq!(view.active_index(), 1, "the opened tab is active");
            (idle, active)
        });

        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window
                    .try_find(SharedString::from(format!("tab-close-{idle}")))
                    .is_some(),
                "an idle tab carries a close button too"
            );
            assert!(
                window
                    .try_find(SharedString::from(format!("tab-close-{active}")))
                    .is_some(),
                "the active tab's close button is laid out in the title bar"
            );
            // Closing the idle one must close it without switching to it.
            window.click(SharedString::from(format!("tab-close-{idle}")), cx);
            window.render_frame(cx);
        })
        .unwrap();

        cx.update(|cx| {
            let view = view.read(cx);
            assert_eq!(view.tabs().count(), 1, "the close button closed its tab");
            assert_eq!(
                view.tabs().next().expect("the survivor").id,
                active,
                "the tab that was active is the one left"
            );
        });
    }

    /// Opens a window over two tabs whose titles differ wildly in length and
    /// returns both tabs' rendered widths.
    fn two_tab_widths(cx: &mut TestAppContext, window_width: f32, name: &str) -> (Pixels, Pixels) {
        let short = std::env::temp_dir().join(format!("{name}-t.dot"));
        let long = std::env::temp_dir().join(format!("{name}-a-very-long-graph-name.dot"));
        std::fs::write(&short, "digraph { a -> b; }").expect("write the short fixture");
        std::fs::write(&long, "digraph { b -> c; }").expect("write the long fixture");

        let mut built = None;
        let handle = cx.open_window(size(px(window_width), px(600.)), |window, cx| {
            let root = cx.new(|cx| {
                GraphView::new(Some(short.clone()), Some(scratch_settings(name)), cx)
            });
            built = Some(root.clone());
            Root::new(root, window, cx)
        });
        let view = built.expect("the view is built");
        cx.run_until_parked();
        cx.update(|cx| {
            view.update(cx, |view, cx| view.open_path_in_tab(long.clone(), cx));
        });
        cx.run_until_parked();

        let (short_id, long_id) = cx.update(|cx| {
            let mut tabs = view.read(cx).tabs();
            let short_id = tabs.next().expect("the short tab").id;
            let long_id = tabs.next().expect("the long tab").id;
            (short_id, long_id)
        });

        cx.update_window(handle.into(), |_, window, cx| {
            window.render_frame(cx);
            let width = |id: u64| {
                window
                    .find(SharedString::from(format!("dotv-tab-{id}")))
                    .bounds()
                    .size
                    .width
            };
            (width(short_id), width(long_id))
        })
        .unwrap()
    }

    /// Tabs are sized to each other, not to their titles: a short file name and
    /// a very long one lay out to the same width, and the long one truncates
    /// instead of widening its tab. The width they agree on follows the window,
    /// so a wide window gets wider tabs than a narrow one — up to the cap.
    #[gpui_kit::test]
    fn tabs_are_all_the_same_width(cx: &mut TestAppContext) {
        cx.update(gpui_kit::init);

        let (wide_short, wide_long) = two_tab_widths(cx, 900.0, "uniform-wide");
        assert_eq!(
            wide_short, wide_long,
            "a short title and a long one are laid out the same width"
        );
        assert_eq!(
            wide_short,
            px(240.),
            "a wide window stops at the cap instead of stretching the tabs"
        );

        let (narrow_short, narrow_long) = two_tab_widths(cx, 400.0, "uniform-narrow");
        assert_eq!(
            narrow_short, narrow_long,
            "the tabs are equal in a narrow window too"
        );
        assert!(
            narrow_short < wide_short,
            "a narrow window gets narrower tabs: {narrow_short:?} against {wide_short:?}"
        );
    }

    /// Dragging a tab out of the strip and letting go over the drawing gives it
    /// a window of its own, and the strip gives it up.
    #[gpui_kit::test]
    fn dragging_a_tab_out_detaches_it(cx: &mut TestAppContext) {
        let (view, window) = open_viewer("digraph { a -> b; }", "dotv-ui-test-detach.dot", cx);
        cx.run_until_parked();
        let second = std::env::temp_dir().join("dotv-ui-test-detach-b.dot");
        std::fs::write(&second, "digraph { b -> c; }").expect("write the fixture");
        cx.update(|cx| view.update(cx, |view, cx| view.open_path_in_tab(second, cx)));
        cx.run_until_parked();

        let (dragged, staying) = cx.update(|cx| {
            let mut tabs = view.read(cx).tabs();
            let dragged = tabs.next().expect("the first tab").id;
            let staying = tabs.next().expect("the second tab").id;
            (dragged, staying)
        });
        let windows_before = cx.windows().len();

        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            // Let go over the drawing, away from every tab.
            window.drag_to(
                SharedString::from(format!("dotv-tab-{dragged}")),
                "dotv-app",
                cx,
            );
            window.render_frame(cx);
        })
        .unwrap();

        assert_eq!(
            cx.windows().len(),
            windows_before + 1,
            "the tab got a window of its own"
        );
        cx.update(|cx| {
            let view = view.read(cx);
            assert_eq!(view.tabs().count(), 1, "the strip gave the tab up");
            assert_eq!(
                view.tabs().next().expect("the survivor").id,
                staying,
                "the tab that stayed is the one that was not dragged"
            );
        });
    }

    /// Dragging a tab onto another's slot moves it there, and the drawing on
    /// screen stays the drawing on screen: `active` is a position, so it has to
    /// follow the tab it named.
    #[gpui_kit::test]
    fn dragging_a_tab_reorders_the_strip(cx: &mut TestAppContext) {
        let (view, window) = open_viewer("digraph { a -> b; }", "dotv-ui-test-reorder.dot", cx);
        cx.run_until_parked();
        for name in ["dotv-ui-test-reorder-b.dot", "dotv-ui-test-reorder-c.dot"] {
            let path = std::env::temp_dir().join(name);
            std::fs::write(&path, "digraph { b -> c; }").expect("write the fixture");
            cx.update(|cx| view.update(cx, |view, cx| view.open_path_in_tab(path, cx)));
            cx.run_until_parked();
        }

        let (first, second, visible) = cx.update(|cx| {
            let tabs: Vec<u64> = view.read(cx).tabs().map(|tab| tab.id).collect();
            assert_eq!(tabs.len(), 3, "three tabs are open");
            assert_eq!(view.read(cx).active_index(), 2, "the last opened is visible");
            (tabs[0], tabs[1], tabs[2])
        });

        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            // Drag the first tab onto the visible one's slot.
            window.drag_to(
                SharedString::from(format!("dotv-tab-{first}")),
                SharedString::from(format!("dotv-tab-{visible}")),
                cx,
            );
            window.render_frame(cx);
        })
        .unwrap();

        cx.update(|cx| {
            let view = view.read(cx);
            let ids: Vec<u64> = view.tabs().map(|tab| tab.id).collect();
            assert_eq!(
                ids,
                vec![second, visible, first],
                "the dragged tab took the slot it was dropped on"
            );
            assert_eq!(
                view.tabs().nth(view.active_index()).map(|tab| tab.id),
                Some(visible),
                "the tab that was visible is still the visible one"
            );
        });
    }

    /// The point of the store: a setting changed in one window is what the
    /// next one starts from.
    #[gpui_kit::test]
    fn settings_survive_a_restart(cx: &mut TestAppContext) {
        let (view, _window) =
            open_viewer("digraph { a -> b; }", "dotv-ui-test-persist.dot", cx);
        cx.run_until_parked();

        cx.update(|cx| {
            view.update(cx, |view, cx| {
                view.update_settings(|settings| settings.show_grid = false);
                view.update_settings(|settings| settings.label_scale = 1.25);
                cx.notify();
            });
        });

        // The same path the viewer was pointed at — but not cleared, so the
        // file it just wrote is still there to read.
        let store = SettingsStore::at(scratch_settings_path("dotv-ui-test-persist.dot"));
        let written = store.load();
        assert!(!written.show_grid, "the change reached the file");
        assert_eq!(written.label_scale, 1.25);

        // A viewer built over that store starts from what was written.
        let mut reopened = None;
        cx.open_window(size(px(640.), px(480.)), |window, cx| {
            let root = cx.new(|cx| GraphView::new(None, Some(store.clone()), cx));
            reopened = Some(root.clone());
            Root::new(root, window, cx)
        });
        cx.update(|cx| {
            let settings = &reopened.expect("the second view is built").read(cx).settings;
            assert!(!settings.show_grid, "the reopened viewer starts grid-less");
            assert_eq!(settings.label_scale, 1.25);
        });
    }

    /// Typing in the panel's field filters the list on the document's name
    /// and its path.
    #[gpui_kit::test]
    fn the_tab_panel_filters_as_you_type(cx: &mut TestAppContext) {
        let (view, window) =
            open_viewer("digraph { a -> b; }", "dotv-ui-test-filter-a.dot", cx);
        cx.run_until_parked();

        let second = std::env::temp_dir().join("dotv-ui-test-filter-b.dot");
        std::fs::write(&second, "digraph { b -> c; }").expect("write the fixture");
        cx.update(|cx| {
            view.update(cx, |view, cx| view.open_path_in_tab(second, cx));
        });
        cx.run_until_parked();
        let first = cx.update(|cx| view.read(cx).tabs().next().expect("first tab").id);
        let second_id = cx.update(|cx| {
            let mut tabs = view.read(cx).tabs();
            tabs.next();
            tabs.next().expect("second tab").id
        });

        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            window.click("dotv-tab-list", cx);
            window.render_frame(cx);
            assert!(
                window
                    .try_find(SharedString::from(format!("dotv-tab-row-{first}")))
                    .is_some(),
                "both tabs are listed to begin with"
            );

            window.click("dotv-tab-search-field", cx);
            window.render_frame(cx);
            window.input("-filter-b", cx);
            window.render_frame(cx);

            assert!(
                window
                    .try_find(SharedString::from(format!("dotv-tab-row-{first}")))
                    .is_none(),
                "the query drops the tab whose path does not match"
            );
            assert!(
                window
                    .try_find(SharedString::from(format!("dotv-tab-row-{second_id}")))
                    .is_some(),
                "the matching tab stays listed"
            );
        })
        .unwrap();
    }

    /// The tab list at the far left of the strip opens the search panel, and
    /// picking a row from it brings that tab forward and closes the panel. It
    /// is the only control that opens a popover *inside* the title bar's
    /// caption area, so it is worth pinning down.
    #[gpui_kit::test]
    fn the_tab_list_searches_and_switches_tabs(cx: &mut TestAppContext) {
        let (view, window) =
            open_viewer("digraph { a -> b; }", "dotv-ui-test-tab-list.dot", cx);
        cx.run_until_parked();

        let second = std::env::temp_dir().join("dotv-ui-test-tab-list-b.dot");
        std::fs::write(&second, "digraph { b -> c; }").expect("write the fixture");
        cx.update(|cx| {
            view.update(cx, |view, cx| view.open_path_in_tab(second, cx));
        });
        cx.run_until_parked();
        let first = cx.update(|cx| view.read(cx).tabs().next().expect("first tab").id);

        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.try_find("dotv-tab-menu-panel").is_none(),
                "the panel starts closed"
            );
            window.click("dotv-tab-list", cx);
            window.render_frame(cx);
            assert!(
                window.try_find("dotv-tab-menu-panel").is_some(),
                "clicking the chevron opens the search panel"
            );
            window.click(SharedString::from(format!("dotv-tab-row-{first}")), cx);
            window.render_frame(cx);
        })
        .unwrap();

        cx.update(|cx| {
            assert_eq!(
                view.read(cx).active_index(),
                0,
                "picking the row brought that tab forward"
            );
        });
        cx.update_window(window, |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.try_find("dotv-tab-menu-panel").is_none(),
                "picking a tab closes the panel"
            );
        })
        .unwrap();
    }

    /// A missing file reports through the dialog too (the I/O path), and the
    /// layout error path produces the same treatment for a graph the engine
    /// cannot draw.
    #[gpui_kit::test]
    fn a_missing_file_reports_in_a_centered_dialog(cx: &mut TestAppContext) {
        let path = std::env::temp_dir().join("dotv-ui-test-missing.dot");
        let _ = std::fs::remove_file(&path);

        cx.update(gpui_kit::init);
        let handle = cx.open_window(size(px(640.), px(480.)), |window, cx| {
            let view = cx.new(|cx| {
                GraphView::new(Some(path.clone()), Some(scratch_settings("missing")), cx)
            });
            Root::new(view, window, cx)
        });

        cx.run_until_parked();
        cx.update_window(handle.into(), |_, window, cx| {
            window.render_frame(cx);
            assert!(
                window.has_active_dialog(cx),
                "the read failure opens a dialog"
            );
        })
        .unwrap();
    }
}
