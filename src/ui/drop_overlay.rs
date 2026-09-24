//! The drop-target overlay: a quiet veil across the canvas shown while the
//! user drags files from the desktop over the window, so the landing zone and
//! the accepted formats are visible before the drop. Pure view builder over
//! the owning [`GraphView`](crate::app::GraphView) — no state of its own.

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::{ActiveTheme as _, Icon, Sizable as _};
use gpui_kit::{Context, InteractiveElement, IntoElement, ParentElement, Styled, div};

use crate::app::GraphView;
use crate::icons::IconName;

/// Renders the drop overlay over the canvas. Call only while a file drag is
/// over the window (`GraphView::file_drag_hover`); the parent composites it
/// on top of the canvas like the zoom cluster.
pub fn drop_overlay(cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();
    div()
        .id("dotv-drop-overlay")
        .absolute()
        .inset_3()
        .rounded_lg()
        .border_2()
        .border_dashed()
        .border_color(theme.primary)
        .bg(theme.background.opacity(0.8))
        .h_flex()
        .items_center()
        .justify_center()
        // The veil sits over the canvas during a file drag: swallow pointer
        // events so the drag cursor stays clean and no canvas hover fires.
        .occlude()
        .child(
            div()
                .v_flex()
                .items_center()
                .gap_2()
                .text_center()
                .child(
                    Icon::new(IconName::OpenFile)
                        .large()
                        .text_color(theme.primary),
                )
                .child(
                    div()
                        .text_sm()
                        .font_medium()
                        .text_color(theme.foreground)
                        .child("Drop to open"),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(theme.muted_foreground)
                        .child("DOT and Graphviz files (.dot · .gv)"),
                ),
        )
}
