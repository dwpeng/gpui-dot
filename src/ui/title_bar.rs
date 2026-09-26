//! The title bar: the one row the tab strip and the window's own controls share.
//!
//! This row is ours rather than the kit's `TitleBar`, for a reason that keeps
//! paying off. The kit marks the whole row as the window's caption area
//! (`WindowControlArea::Drag`), and a caption area belongs to the platform: it
//! answers the press and the drag before the app ever sees them, which is why a
//! tab drag turned into a window drag and a dropdown click went nowhere. Drawing
//! the row ourselves makes it only as much of a caption as we say it is — the
//! tab strip keeps its own presses, and the empty stretch between the `+` and
//! the window controls is the part that moves the window.
//!
//! It settles the geometry too. The strip's lane is laid out against whatever
//! the row leaves after the tab list and the window controls, so a tab's width
//! is a share of its own container rather than a number we derive from the
//! window width and keep in step with the chrome by hand.

use gpui_kit::StatefulInteractiveElement as _;
use gpui_kit::base::InteractiveElementExt as _;
use gpui_kit::component::ActiveTheme as _;
use gpui_kit::component::{Icon, Sizable as _};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{
    App, Context, Hsla, InteractiveElement as _, IntoElement, MouseButton, MouseDownEvent,
    ParentElement, SharedString, Styled, Window, div, px,
};

use crate::app::GraphView;
use crate::icons::IconName;
use crate::ui::tab_bar;

/// The row's height.
const ROW_HEIGHT: f32 = 34.0;
/// The row's left inset, on everything but macOS.
const ROW_LEFT_INSET: f32 = 6.0;
/// The footprint of one window control.
const CONTROL_SIZE: f32 = 34.0;
/// The gap that always stands between the strip and the window controls, so the
/// `+` never touches them however full the strip gets.
const GAP_BEFORE_CONTROLS: f32 = 64.0;

/// Whether a press on the row is waiting for a movement to become a window move.
/// Retained per window so a plain click, or a double click, on the row still
/// works instead of starting one.
struct RowDrag {
    armed: bool,
}

/// Renders the title bar: the tab strip, then the window's own controls.
pub fn title_bar(
    view: &GraphView,
    window: &mut Window,
    cx: &mut Context<GraphView>,
) -> impl IntoElement {
    // Chrome's light frame grey (#d4d4d4), from the theme's palette: dark
    // enough for the white active tab to read clearly on it, light enough that
    // the theme's own hover tints stay visible on top of it.
    let frame = cx.theme().tokens.secondary_active;
    let drag = window.use_keyed_state("dotv-row-drag", cx, |_, _| RowDrag { armed: false });

    div()
        .id("dotv-title-bar")
        .h(px(ROW_HEIGHT))
        .flex_shrink_0()
        .flex()
        .items_center()
        .when(!cfg!(target_os = "macos"), |row| row.pl(px(ROW_LEFT_INSET)))
        .bg(frame)
        // Pressed and then moved, the window follows the pointer — but only
        // after an actual movement, so clicking and double-clicking the row
        // still do what they should. The strip stops its own presses before
        // they reach these handlers, and everything after the `+` is theirs.
        .on_mouse_down(MouseButton::Left, {
            let drag = drag.clone();
            move |_, _, cx| {
                drag.update(cx, |drag, _| drag.armed = true);
            }
        })
        .on_mouse_up(MouseButton::Left, {
            let drag = drag.clone();
            move |_, _, cx| {
                drag.update(cx, |drag, _| drag.armed = false);
            }
        })
        .on_mouse_move({
            let drag = drag.clone();
            move |_, window, cx| {
                if drag.read(cx).armed {
                    drag.update(cx, |drag, _| drag.armed = false);
                    window.start_window_move();
                }
            }
        })
        .on_double_click(|_, window, _| window.zoom_window())
        // The window menu the platform offers, where one is offered: the kit's
        // title bar raised it on a right press, and nothing else will now.
        .when(cfg!(target_os = "linux"), |row| {
            row.on_mouse_down(MouseButton::Right, |event: &MouseDownEvent, window, _| {
                window.show_window_menu(event.position);
            })
        })
        .child(tab_bar(view, cx))
        .child(window_controls(cx))
}

/// The window's own controls: minimize, zoom and close.
///
/// Plain buttons, each acting on its own click, rather than elements marked
/// with `WindowControlArea` — the platform gets no say in this row's hit
/// testing, which is the point of drawing the row ourselves.
fn window_controls(cx: &App) -> impl IntoElement {
    let ink = cx.theme().foreground;
    let hover = cx.theme().secondary_hover;
    let close_hover = cx.theme().danger;

    div()
        .id("dotv-window-controls")
        .h_full()
        .flex_shrink_0()
        .flex()
        .items_center()
        .ml(px(GAP_BEFORE_CONTROLS))
        .child(control(
            IconName::Minimize,
            "Minimize",
            ink,
            hover,
            |window, _| window.minimize_window(),
        ))
        .child(control(
            IconName::Maximize,
            "Maximize",
            ink,
            hover,
            |window, _| window.zoom_window(),
        ))
        .child(control(
            IconName::CloseTab,
            "Close",
            ink,
            close_hover,
            |window, _| window.remove_window(),
        ))
}

/// One window control: a control-sized icon button that acts on its click.
///
/// A plain div rather than a `Button`, because a button carries a hover style of
/// its own and gpui asserts there is only one — and we need the close button to
/// hover in the theme's danger colour. It has no tooltip: every browser's
/// caption buttons are understood without one.
fn control(
    icon: IconName,
    label: &'static str,
    ink: Hsla,
    hover: Hsla,
    action: impl Fn(&mut Window, &mut App) + 'static,
) -> impl IntoElement {
    div()
        .id(SharedString::from(format!("dotv-window-{label}")))
        .w(px(CONTROL_SIZE))
        .h_full()
        .flex()
        .justify_center()
        .items_center()
        .text_color(ink)
        .hover(move |style| style.bg(hover))
        .on_click(move |_, window, cx| action(window, cx))
        .child(Icon::new(icon).small())
}
