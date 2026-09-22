//! The settings module: the [`Settings`] model, the generic settings-card UI
//! ([`card`]) and the app's own settings rows ([`panel`]).

mod card;
pub(crate) mod panel;

use gpui_kit::SharedString;

pub use card::{SettingRow, SettingsCard};

/// The viewer's settings, edited from the title-bar settings card and the
/// status-bar quick toggles.
///
/// All values are applied when drawing: the booleans gate what the canvas
/// paints (`show_node_labels`, `show_edge_labels`, `show_grid`,
/// `natural_scroll`), and the choice/value settings (`rank_dir`,
/// `label_scale`) drive a re-measure and re-layout of the loaded document, so
/// the drawing always matches them.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// A boolean setting: whether node labels are painted.
    pub show_node_labels: bool,
    /// A boolean setting: whether edge labels are painted.
    pub show_edge_labels: bool,
    /// A boolean setting: whether the background grid is painted on the canvas.
    pub show_grid: bool,
    /// A boolean setting: the plain-wheel pan direction — `true` is the
    /// macOS-style "natural" scrolling where the content follows the scroll
    /// (the fingers), `false` the traditional Windows-style direction where
    /// scrolling down reveals what is below.
    pub natural_scroll: bool,
    /// A choice setting: the layout direction of the graph.
    pub rank_dir: SharedString,
    /// A value setting: the scale of node labels.
    pub label_scale: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            show_node_labels: true,
            show_edge_labels: true,
            show_grid: true,
            natural_scroll: true,
            rank_dir: "TB".into(),
            label_scale: 1.0,
        }
    }
}
