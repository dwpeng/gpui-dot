//! Application-wide actions and their default key bindings.

use gpui_kit::{App, KeyBinding, actions};

// Actions are registered under the `dotv` namespace, so their full names are
// `dotv::open_graph`, `dotv::zoom_in`, etc.
actions!(
    dotv,
    [
        OpenGraph,
        PasteGraph,
        ToggleFullscreen,
        CloseTab,
        NextTab,
        PrevTab,
        ZoomIn,
        ZoomOut,
        FitGraph,
        ClearSelection,
        SearchTabs
    ]
);

/// Binds the default key shortcuts for the whole application.
///
/// Uses the `secondary` modifier (Command on macOS, Ctrl everywhere else) so
/// the same bindings work on every platform:
/// - `secondary-o`  → open a DOT file
/// - `secondary-v`  → open the .dot file path on the clipboard (the WSLg
///   substitute for dragging a file in from Windows, which WSLg's RDP
///   clipboard never forwards)
/// - `secondary-w`            → close the current tab
/// - `secondary-tab`          → next tab
/// - `secondary-shift-tab`    → previous tab
/// - `f11`          → toggle fullscreen (Esc also exits)
/// - `secondary-=`  → zoom in
/// - `secondary--`  → zoom out
/// - `secondary-0`  → fit the graph to the window
/// - `secondary-shift-a` → open the tab list's search field
/// - `escape`       → clear the current node selection
pub fn bind_keys(cx: &mut App) {
    cx.bind_keys([
        KeyBinding::new("secondary-o", OpenGraph, None),
        KeyBinding::new("secondary-v", PasteGraph, None),
        KeyBinding::new("secondary-w", CloseTab, None),
        KeyBinding::new("secondary-tab", NextTab, None),
        KeyBinding::new("secondary-shift-tab", PrevTab, None),
        KeyBinding::new("f11", ToggleFullscreen, None),
        KeyBinding::new("secondary-=", ZoomIn, None),
        KeyBinding::new("secondary--", ZoomOut, None),
        KeyBinding::new("secondary-0", FitGraph, None),
        KeyBinding::new("secondary-shift-a", SearchTabs, None),
        KeyBinding::new("escape", ClearSelection, None),
    ]);
}
