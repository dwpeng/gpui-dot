//! The loading overlay: what the canvas shows while a freshly opened file is
//! parsed and laid out on the background executor — an indeterminate
//! spinner, the file being loaded and what is happening, centered over the
//! canvas. It disappears the frame the layout lands. Pure view builder over
//! the owning [`GraphView`](crate::app::GraphView) — no state of its own.

use std::time::Duration;

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::{ActiveTheme as _, Icon};
use gpui_kit::{
    Animation, AnimationExt as _, Context, Hsla, InteractiveElement as _, IntoElement,
    ParentElement, Pixels, Role, StatefulInteractiveElement as _, Styled, TestSupportExt as _,
    Transformation, div, percentage, px,
};

use crate::app::{GraphTab, GraphView};
use crate::icons::IconName;

/// The dotv loading spinner: the `loader-circle` arc rotating at a constant
/// speed. Linear on purpose — the framework `Spinner`'s default ease-in-out
/// restarts the turn every cycle (the speed dips to zero each revolution),
/// which reads as a stutter rather than a spin, and its fast 0.8 s turn on a
/// spoked icon strobes. One turn per 1.1 s at constant speed is the calm,
/// unambiguous "working" motion, driven by vsync (`request_animation_frame`)
/// with the angle sampled from the wall clock, and it freezes under the
/// system's reduced-motion preference.
pub(crate) fn loading_spinner(id: &'static str, size: Pixels, color: Hsla) -> impl IntoElement {
    Icon::new(IconName::LoaderCircle)
        .size(size)
        .text_color(color)
        .with_animation(
            id,
            Animation::new(Duration::from_secs_f64(1.1)).repeat(),
            |icon, delta| icon.transform(Transformation::rotate(percentage(delta))),
        )
}

/// Renders the loading overlay over the canvas. Call only while the visible
/// tab is loading its first document (`GraphTab::loading` with no document
/// yet); the parent composites it on top of the canvas like the zoom
/// cluster.
pub fn loading_overlay(tab: &GraphTab, cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();
    div()
        .id("dotv-loading-overlay")
        .absolute()
        .inset_0()
        .bg(theme.background.opacity(0.8))
        .h_flex()
        .items_center()
        .justify_center()
        // Announced as a live region: what is loading, named by file.
        .role(Role::Status)
        .aria_label(format!(
            "Loading {} — parsing and laying out the graph",
            tab.title
        ))
        // The veil covers the canvas while the graph is being built: swallow
        // pointer events so nothing under it reacts (there is nothing to
        // interact with until the drawing arrives anyway).
        .occlude()
        // Registered so the UI integration tests can assert that the
        // animation appears with a first load and ends when it lands.
        .test_support()
        .child(
            div()
                .v_flex()
                .items_center()
                .gap_2()
                .text_center()
                .child(loading_spinner(
                    "dotv-loading-spinner",
                    px(24.),
                    theme.primary,
                ))
                .child(
                    div()
                        .text_sm()
                        .font_medium()
                        .text_color(theme.foreground)
                        .child(tab.title.clone()),
                )
                .child(
                    div()
                        .text_xs()
                        .text_color(theme.muted_foreground)
                        .child("Parsing and laying out the graph…"),
                ),
        )
}
