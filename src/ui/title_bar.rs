//! The title bar: window chrome and the fullscreen toggle on the left. The
//! open action lives in the tab strip's `+` button, settings live in the
//! status bar, and the open file is named by its own tab.

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::TitleBar;
use gpui_kit::component::button::{Button, ButtonVariants as _};
use gpui_kit::component::{Icon, Sizable as _};
use gpui_kit::{App, Context, IntoElement, ParentElement, Styled, Window, div};

use crate::actions::ToggleFullscreen;
use crate::app::GraphView;
use crate::icons::IconName;

/// Renders the title bar for `view`.
pub fn title_bar(_view: &GraphView, _cx: &mut Context<GraphView>) -> impl IntoElement {
    // Dispatches the app action, so the button, F11 and Esc all run the same
    // fullscreen path (viewer state + platform window sync) in the view.
    let fullscreen = Button::new("fullscreen")
        .icon(Icon::new(IconName::Maximize))
        .ghost()
        .small()
        .tooltip("Fullscreen")
        .on_click(|_, window: &mut Window, cx: &mut App| {
            window.dispatch_action(Box::new(ToggleFullscreen), cx);
        });

    TitleBar::new().child(
        div()
            .h_full()
            .h_flex()
            .items_center()
            .gap_0p5()
            .child(fullscreen),
    )
}
