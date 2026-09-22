//! Application-wide actions and their default key bindings.

use gpui_kit::{App, KeyBinding, actions};

// Actions are registered under the `dotv` namespace, so their full names are
// `dotv::open_graph`, `dotv::zoom_in`, etc.
actions!(dotv, [OpenGraph, ZoomIn, ZoomOut, FitGraph, ClearSelection]);

/// Binds the default key shortcuts for the whole application.
///
/// Uses the `secondary` modifier (Command on macOS, Ctrl everywhere else) so
/// the same bindings work on every platform:
/// - `secondary-o`  → open a DOT file
/// - `secondary-=`  → zoom in
/// - `secondary--`  → zoom out
/// - `secondary-0`  → fit the graph to the window
/// - `escape`       → clear the current node selection
pub fn bind_keys(cx: &mut App) {
    cx.bind_keys([
        KeyBinding::new("secondary-o", OpenGraph, None),
        KeyBinding::new("secondary-=", ZoomIn, None),
        KeyBinding::new("secondary--", ZoomOut, None),
        KeyBinding::new("secondary-0", FitGraph, None),
        KeyBinding::new("escape", ClearSelection, None),
    ]);
}
