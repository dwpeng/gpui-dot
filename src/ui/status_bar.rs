//! The status bar: document counts, quick toggles for the label and
//! direction settings, and interaction hints. The toggles double as state
//! display and one-click switches — highlighted when the feature is on,
//! dimmed when it is off; the direction button shows the current layout
//! axis and flips it on click.

use gpui_kit::base::StyledExt as _;

use gpui_kit::assets::IconName;
use gpui_kit::component::button::{Button, ButtonVariants as _, Toggle};
use gpui_kit::component::{ActiveTheme as _, Icon, Sizable as _};
use gpui_kit::{App, Context, InteractiveElement, IntoElement, ParentElement, Styled, Window, div};

use crate::app::GraphView;

/// Renders the status bar for `view`.
pub fn status_bar(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();
    let weak = cx.weak_entity();
    let node_labels_toggle = {
        let weak = weak.clone();
        Toggle::new("status-node-labels")
            .icon(Icon::new(IconName::SquareText))
            .checked(view.settings.show_node_labels)
            .xsmall()
            .tooltip("Node labels")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.settings.show_node_labels = *checked;
                    cx.notify();
                });
            })
    };
    let edge_labels_toggle = {
        let weak = weak.clone();
        Toggle::new("status-edge-labels")
            .icon(Icon::new(IconName::Baseline))
            .checked(view.settings.show_edge_labels)
            .xsmall()
            .tooltip("Edge labels")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.settings.show_edge_labels = *checked;
                    cx.notify();
                });
            })
    };

    let direction_icon = if view.settings.rank_dir == "LR" {
        IconName::AlignCenterHorizontal
    } else {
        IconName::AlignCenterVertical
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
                    view.settings.rank_dir = if view.settings.rank_dir == "LR" {
                        "TB".into()
                    } else {
                        "LR".into()
                    };
                    view.relayout_for_settings(cx);
                    cx.notify();
                });
            })
    };

    let grid_toggle = {
        let weak = weak.clone();
        Toggle::new("status-edge-grid")
            .icon(Icon::new(IconName::Frame))
            .checked(view.settings.show_grid)
            .xsmall()
            .tooltip("Show grid line")
            .on_click(move |checked, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| {
                    view.settings.show_grid = *checked;
                    cx.notify();
                });
            })
    };

    // Counts and the selection read-out: the status bar is where the user
    // checks what is loaded and what a click selected.
    let summary = view.document.as_ref().map(|document| {
        format!(
            "{} nodes · {} edges",
            document.graph.nodes.len(),
            document.graph.edges.len()
        )
    });
    let selected = view
        .selection
        .and_then(|index| view.document.as_ref().map(|document| (index, document)))
        .and_then(|(index, document)| {
            let node = document.graph.nodes.get(index)?;
            let label = node.label();
            Some(if label.is_empty() {
                node.name.clone()
            } else {
                label
            })
        });

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
                .child(direction_toggle)
                .children(summary.map(|summary| div().child(summary)))
                .children(selected.map(|name| div().truncate().child(format!("selected: {name}")))),
        )
        .child(
            div()
                .h_flex()
                .gap_0p5()
                .items_center()
                .child(node_labels_toggle)
                .child(edge_labels_toggle)
                .child(grid_toggle),
        )
}
