//! The title bar. It shares its single row with the tab strip — Chrome's
//! arrangement — so the open documents, the `+` that opens another and the
//! window's own controls all sit on one line, and the document area gets the
//! row back.
//!
//! The row wears the *tab strip's frame* colour rather than the window's. That
//! is the load-bearing part of Chrome's design: the frame is a distinctly
//! darker surface, and the active tab is a lighter shape floating inside it,
//! which is what makes a tab read as a tab rather than as a slightly lighter
//! rectangle. The bar keeps its own bottom divider, because the tabs float
//! clear of that edge rather than merging into the drawing below.
//!
//! The window controls are drawn by [`TitleBar`] itself on the right; the tab
//! strip fills everything to their left and shows the frame through where it
//! has no tab of its own. The component keeps that region as the window's
//! caption area, so dragging the empty part of the row still moves the window
//! and double-clicking it still maximizes; the strip's own controls opt out of
//! the marker (see [`super::tab_bar`]).

use gpui_kit::component::ActiveTheme as _;
use gpui_kit::component::TitleBar;
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{Background, Context, IntoElement, ParentElement, Styled, Window, px};

use crate::app::GraphView;
use crate::ui::tab_bar;

/// The row's left inset, on everything but macOS. Shared with the tab strip,
/// which subtracts it when working out how wide a tab can be.
pub(crate) const ROW_LEFT_INSET: f32 = 6.0;

/// Renders the title bar, with the tab strip folded into it.
pub fn title_bar(
    view: &GraphView,
    window: &mut Window,
    cx: &mut Context<GraphView>,
) -> impl IntoElement {
    // Chrome's light frame grey (#d4d4d4), taken from the theme's palette. It
    // is dark enough for the white active tab to read clearly on it, and light
    // enough that the kit's hover tints — which all sit a shade lighter, on the
    // window controls and on ghost buttons alike — stay visible on top of it.
    let frame: Background = cx.theme().tokens.secondary_active.into();
    TitleBar::new()
        .bg(frame)
        // The component insets the row by 12px, which is room for a platform
        // window menu this window does not draw; Chrome tucks the tab list in
        // close to the edge instead. macOS keeps the component's 80px, which
        // reserves the space the traffic lights sit in.
        .when(!cfg!(target_os = "macos"), |bar| bar.pl(px(ROW_LEFT_INSET)))
        .child(tab_bar(view, window, cx))
}
