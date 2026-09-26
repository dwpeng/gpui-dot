//! The settings module: the [`Settings`] model, where it is persisted
//! ([`store`]), the generic settings-card UI ([`card`]) and the app's own
//! settings rows ([`panel`]).

mod card;
pub(crate) mod panel;
mod store;

use gpui_kit::SharedString;

pub use card::{SettingRow, SettingsCard};
pub use store::SettingsStore;

/// The range the "Label scale" control offers, the step between its values, and
/// the clamp applied when a settings file names a scale outside it.
pub const LABEL_SCALE_MIN: f64 = 0.5;
pub const LABEL_SCALE_MAX: f64 = 2.0;
pub const LABEL_SCALE_STEP: f64 = 0.05;

/// The viewer's settings, edited from the settings card and the quick
/// toggles, both in the status bar, and remembered between runs by
/// [`SettingsStore`].
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
    /// A choice setting: the layout direction of the graph. Only `"TB"` and
    /// `"LR"` are meaningful; anything else reads as `"TB"`.
    pub rank_dir: SharedString,
    /// A value setting: the scale of node labels, clamped to
    /// [`LABEL_SCALE_MIN`]`..=`[`LABEL_SCALE_MAX`].
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

impl Settings {
    /// The settings as JSON, ready to write to the store.
    pub fn to_json(&self) -> String {
        serde_json::json!({
            "show_node_labels": self.show_node_labels,
            "show_edge_labels": self.show_edge_labels,
            "show_grid": self.show_grid,
            "natural_scroll": self.natural_scroll,
            "rank_dir": self.rank_dir.as_ref(),
            "label_scale": self.label_scale,
        })
        .to_string()
    }

    /// Reads settings from JSON.
    ///
    /// Every field is taken on its own: one that is absent, or of the wrong
    /// type, keeps its default rather than sinking the whole file, and the
    /// choice and value settings are clamped to what the viewer can act on.
    /// That tolerance is deliberate — settings written by an older or newer
    /// build, or hand-edited into a typo, should still start the viewer.
    pub fn from_json(text: &str) -> Self {
        let fallback = Self::default();
        let Ok(value) = serde_json::from_str::<serde_json::Value>(text) else {
            return fallback;
        };
        // Only the two directions the engine is given exist; a third value in
        // the file would otherwise reach the layout as "not LR" anyway, and
        // then be written back out as a lie.
        let rank_dir = match value.get("rank_dir").and_then(serde_json::Value::as_str) {
            Some("LR") => SharedString::from("LR"),
            _ => fallback.rank_dir,
        };
        let label_scale = value
            .get("label_scale")
            .and_then(serde_json::Value::as_f64)
            .unwrap_or(fallback.label_scale)
            .clamp(LABEL_SCALE_MIN, LABEL_SCALE_MAX);

        Self {
            show_node_labels: bool_at(&value, "show_node_labels", fallback.show_node_labels),
            show_edge_labels: bool_at(&value, "show_edge_labels", fallback.show_edge_labels),
            show_grid: bool_at(&value, "show_grid", fallback.show_grid),
            natural_scroll: bool_at(&value, "natural_scroll", fallback.natural_scroll),
            rank_dir,
            label_scale,
        }
    }
}

/// The boolean at `key`, or `default` when it is absent or not a boolean.
fn bool_at(value: &serde_json::Value, key: &str, default: bool) -> bool {
    value
        .get(key)
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(default)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_round_trips_every_setting() {
        let settings = Settings {
            show_node_labels: false,
            show_edge_labels: false,
            show_grid: false,
            natural_scroll: false,
            rank_dir: "LR".into(),
            label_scale: 1.35,
        };
        assert_eq!(Settings::from_json(&settings.to_json()), settings);
    }

    /// Nothing to read is not a failure: the viewer starts on its defaults.
    #[test]
    fn unreadable_text_falls_back_to_defaults() {
        assert_eq!(Settings::from_json(""), Settings::default());
        assert_eq!(Settings::from_json("not json at all"), Settings::default());
        assert_eq!(Settings::from_json("[1, 2, 3]"), Settings::default());
    }

    /// A file written by a build that knew fewer settings, or more, still
    /// loads: the fields it has are taken and the rest keep their defaults.
    #[test]
    fn missing_and_unknown_fields_keep_their_defaults() {
        let partial = Settings::from_json(r#"{"show_grid": false}"#);
        assert!(!partial.show_grid);
        assert!(
            partial.show_node_labels,
            "an absent field keeps its default"
        );
        assert_eq!(partial.rank_dir, "TB");

        let extended = Settings::from_json(
            r#"{"show_grid": false, "from_the_future": 7, "theme": "dracula"}"#,
        );
        assert!(
            !extended.show_grid,
            "a known field beside unknown ones loads"
        );
    }

    #[test]
    fn wrongly_typed_fields_keep_their_defaults() {
        let settings =
            Settings::from_json(r#"{"show_grid": "no", "rank_dir": 3, "label_scale": "big"}"#);
        assert!(settings.show_grid, "a non-boolean keeps the default");
        assert_eq!(settings.rank_dir, "TB", "a non-string falls back");
        assert_eq!(settings.label_scale, 1.0, "a non-number falls back");
    }

    /// The choice and value settings are clamped to what the viewer can act on,
    /// so a hand-edited file cannot ask for a scale nothing can draw.
    #[test]
    fn choice_and_value_settings_are_clamped() {
        assert_eq!(
            Settings::from_json(r#"{"rank_dir": "sideways"}"#).rank_dir,
            "TB"
        );
        assert_eq!(
            Settings::from_json(r#"{"label_scale": 99.0}"#).label_scale,
            LABEL_SCALE_MAX
        );
        assert_eq!(
            Settings::from_json(r#"{"label_scale": -4.0}"#).label_scale,
            LABEL_SCALE_MIN
        );
    }
}
