//! The status bar: document counts, quick toggles for the label and
//! direction settings, and interaction hints. The toggles double as state
//! display and one-click switches — highlighted when the feature is on,
//! dimmed when it is off; the direction button shows the current layout
//! axis and flips it on click.

use gpui_kit::base::StyledExt as _;

use gpui_kit::component::button::{Button, ButtonVariants as _, Toggle};
use gpui_kit::component::popover::Popover;
use gpui_kit::component::{ActiveTheme as _, Icon, Sizable as _};
use gpui_kit::{
    Anchor, App, Context, InteractiveElement, IntoElement, ParentElement, Styled, Window, div, px,
};

use crate::actions::ToggleFullscreen;
use crate::app::GraphView;
use crate::icons::IconName;
use crate::settings::SettingsCard;
use crate::settings::panel as settings_panel;
use crate::ui::loading::loading_spinner;
use crate::ui::zoom_cluster;

/// Renders the status bar for `view`.
pub fn status_bar(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    // Zooming a drawing that is not there has no meaning — in the empty
    // state, and under the first-load veil, the controls are withheld
    // rather than reading a zoom for a graph that has not arrived.
    let has_document = view.active().is_some_and(|tab| tab.document.is_some());
    // Built here rather than in the builder chain below: `zoom_cluster`
    // answers a mutable `cx`, and `cx.theme()` owns a borrow of `*cx` for
    // the rest of the body.
    let zoom = has_document.then(|| zoom_cluster(view, cx));

    let theme = cx.theme();
    let weak = cx.weak_entity();
    let settings = view.settings.clone();
    let settings_trigger = {
        let weak = weak.clone();
        Popover::new("status-settings")
            .anchor(Anchor::BottomLeft)
            .trigger(
                Button::new("status-settings-trigger")
                    .icon(Icon::new(IconName::Settings))
                    .ghost()
                    .xsmall()
                    .tooltip("Settings"),
            )
            .content(move |_, _, _| SettingsCard::new(settings_panel::rows(&settings, &weak)))
    };

    let node_labels_toggle = {
        let weak = weak.clone();
        Toggle::new("status-node-labels")
            .icon(Icon::new(IconName::NodeLabels))
            .checked(view.settings.show_node_labels)
            .xsmall()
            .tooltip("Node labels")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.update_settings(|settings| settings.show_node_labels = *checked);
                    cx.notify();
                });
            })
    };
    let edge_labels_toggle = {
        let weak = weak.clone();
        Toggle::new("status-edge-labels")
            .icon(Icon::new(IconName::EdgeLabels))
            .checked(view.settings.show_edge_labels)
            .xsmall()
            .tooltip("Edge labels")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.update_settings(|settings| settings.show_edge_labels = *checked);
                    cx.notify();
                });
            })
    };

    let direction_icon = if view.settings.rank_dir == "LR" {
        IconName::DirectionHorizontal
    } else {
        IconName::DirectionVertical
    };

    let direction_toggle = {
        let weak = weak.clone();
        Button::new("status-direction")
            .icon(Icon::new(direction_icon))
            .ghost()
            .xsmall()
            .tooltip("Layout direction")
            .on_click(move |_, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    let next = if view.settings.rank_dir == "LR" {
                        "TB"
                    } else {
                        "LR"
                    };
                    view.update_settings(|settings| settings.rank_dir = next.into());
                    view.relayout_for_settings(cx);
                    cx.notify();
                });
            })
    };

    let grid_toggle = {
        let weak = weak.clone();
        Toggle::new("status-grid")
            .icon(Icon::new(IconName::Grid))
            .checked(view.settings.show_grid)
            .xsmall()
            .tooltip("Show grid line")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.update_settings(|settings| settings.show_grid = *checked);
                    cx.notify();
                });
            })
    };

    // Fullscreen used to ride the title bar. It moved here when that row
    // became the tab strip — which leaves the title bar as pure document
    // chrome — and it belongs with the other view toggles anyway. The
    // action is dispatched rather than handled locally, so this button,
    // F11 and Esc all run the same viewer-state + platform-window path.
    let fullscreen_toggle = Button::new("status-fullscreen")
        .icon(Icon::new(IconName::Maximize))
        .ghost()
        .xsmall()
        .tooltip("Fullscreen (F11)")
        .on_click(|_, window: &mut Window, cx: &mut App| {
            window.dispatch_action(Box::new(ToggleFullscreen), cx);
        });

    let selected = view.active().and_then(|tab| {
        tab.selection.and_then(|index| {
            let document = tab.document.as_ref()?;
            let node = document.graph.nodes.get(index)?;
            let label = node.label();
            Some(if label.is_empty() {
                node.name.clone()
            } else {
                label
            })
        })
    });

    // A re-layout (a settings change) runs on the background executor; the
    // previous drawing stays visible, and this chip explains the wait. A
    // first load covers the canvas with the loading overlay instead.
    let relaying_out = view
        .active()
        .is_some_and(|tab| tab.loading && tab.document.is_some());

    div()
        .id("dotv-status")
        .h_flex()
        .gap_2()
        .items_center()
        .px_3()
        .py_1()
        .border_t_1()
        .border_color(theme.border)
        .text_xs()
        .text_color(theme.muted_foreground)
        .child(
            div()
                .flex_1()
                .h_flex()
                .gap_2()
                .items_center()
                .child(settings_trigger)
                // Zoom stays beside settings: both are view controls that
                // act on the drawing above rather than on the document.
                .children(zoom)
                .child(direction_toggle)
                .children(relaying_out.then(|| {
                    div()
                        .h_flex()
                        .gap_1p5()
                        .items_center()
                        .child(loading_spinner(
                            "dotv-status-spinner",
                            px(12.),
                            theme.muted_foreground,
                        ))
                        .child("Laying out…")
                }))
                .children(selected.map(|name| div().truncate().child(format!("selected: {name}")))),
        )
        .child(
            div()
                .h_flex()
                .gap_0p5()
                .items_center()
                .child(node_labels_toggle)
                .child(edge_labels_toggle)
                .child(grid_toggle)
                .child(fullscreen_toggle),
        )
}
