//! The app's settings-card rows: a snapshot of [`Settings`] turned into
//! [`SettingRow`]s that write changes back into the owning [`GraphView`].

use gpui_kit::WeakEntity;

use crate::app::GraphView;

use super::{LABEL_SCALE_MAX, LABEL_SCALE_MIN, LABEL_SCALE_STEP, SettingRow, Settings};

/// Builds the settings-card rows from a snapshot of [`Settings`], writing
/// changes back into the owning [`GraphView`] through the weak handle.
pub fn rows(settings: &Settings, weak: &WeakEntity<GraphView>) -> Vec<SettingRow> {
    vec![
        SettingRow::bool(
            "natural-scroll",
            "Natural scrolling",
            None,
            settings.natural_scroll,
            {
                let weak = weak.clone();
                move |value, _window, cx| {
                    let _ = weak.update(cx, |view, cx| {
                        view.update_settings(|settings| settings.natural_scroll = value);
                        cx.notify();
                    });
                }
            },
        ),
        SettingRow::value(
            "label-scale",
            "Label scale",
            None,
            settings.label_scale,
            LABEL_SCALE_MIN,
            LABEL_SCALE_MAX,
            LABEL_SCALE_STEP,
            {
                let weak = weak.clone();
                move |value, _window, cx| {
                    let _ = weak.update(cx, |view, cx| {
                        view.update_settings(|settings| settings.label_scale = value);
                        view.relayout_for_settings(cx);
                        cx.notify();
                    });
                }
            },
        ),
    ]
}
