//! Floating zoom controls pinned to the canvas' bottom-left corner: zoom
//! out, the current percentage, zoom in, and fit-to-window.

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::button::{Button, ButtonVariants as _};
use gpui_kit::component::{ActiveTheme as _, Sizable as _};
use gpui_kit::{ClickEvent, Context, InteractiveElement, IntoElement, ParentElement, Styled, div};

use crate::app::GraphView;
use crate::icons::IconName;

pub fn zoom_cluster(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();
    div()
        .id("dotv-zoom-cluster")
        .absolute()
        .bottom_3()
        .left_3()
        .h_flex()
        .items_center()
        .gap_0p5()
        .px_1()
        .py_0p5()
        .rounded_md()
        .bg(theme.popover)
        .border_1()
        .border_color(theme.border)
        .occlude()
        .child(
            Button::new("zoom-out")
                .icon(IconName::ZoomOut)
                .ghost()
                .xsmall()
                .tooltip("Zoom out (Ctrl+-)")
                .on_click(cx.listener(|this, _: &ClickEvent, _window, cx| this.zoom_out(cx))),
        )
        .child(
            div()
                .w_11()
                .text_center()
                .text_xs()
                .text_color(theme.muted_foreground)
                .child(format!(
                    "{}%",
                    (view.active().map_or(1.0, |tab| tab.zoom) * 100.0).round() as i32
                )),
        )
        .child(
            Button::new("zoom-in")
                .icon(IconName::ZoomIn)
                .ghost()
                .xsmall()
                .tooltip("Zoom in (Ctrl+=)")
                .on_click(cx.listener(|this, _: &ClickEvent, _window, cx| this.zoom_in(cx))),
        )
        .child(
            Button::new("fit")
                .icon(IconName::FitView)
                .ghost()
                .xsmall()
                .tooltip("Fit to window (Ctrl+0)")
                .on_click(cx.listener(|this, _: &ClickEvent, _window, cx| this.fit_graph(cx))),
        )
        // Undo every manual node move. Dragged nodes are otherwise permanent
        // (only a re-layout, a double-click on each node or a restart clears
        // them), so the escape hatch sits with the other view controls.
        .child(
            Button::new("reset-positions")
                .icon(IconName::ResetLayout)
                .ghost()
                .xsmall()
                .tooltip("Reset moved nodes to their layout positions")
                .on_click(
                    cx.listener(|this, _: &ClickEvent, _window, cx| this.reset_positions(cx)),
                ),
        )
}
