//! The tab strip: one tab per open DOT file, the active one highlighted and
//! each tab carrying its own close button. Pure view builder over the owning
//! [`GraphView`](crate::app::GraphView) — no state of its own.

use gpui_kit::component::Sizable as _;
use gpui_kit::component::button::{Button, ButtonVariants as _};
use gpui_kit::component::tab::{Tab, TabBar};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{App, Context, IntoElement, SharedString, Styled, Window, px};

use crate::app::GraphView;
use crate::icons::IconName;

/// Renders the tab strip for `view`.
pub fn tab_bar(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let weak = cx.weak_entity();
    // The `+` opens the file browser. It rides the tab strip's suffix slot,
    // which sits outside the scrolling region, so it stays pinned at the
    // right edge no matter how many tabs are open. With no tabs the strip
    // stays bare — the empty canvas below carries the open action instead.
    let has_tabs = view.tabs().next().is_some();
    let add = {
        let weak = weak.clone();
        Button::new("tab-add")
            .icon(IconName::NewTab)
            .ghost()
            .small()
            .tooltip("Open a DOT file (Ctrl+O)")
            .on_click(move |_, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| view.open_graph(cx));
            })
    };
    let tabs = view
        .tabs()
        .map(|tab| {
            let tab_id = tab.id;
            // The close button stops propagation: a click on it must close
            // the tab, not also switch to it through the tab bar's handler.
            let close = {
                let weak = weak.clone();
                Button::new(SharedString::from(format!("tab-close-{tab_id}")))
                    .icon(IconName::CloseTab)
                    .ghost()
                    .xsmall()
                    .pr_1()
                    .tooltip("Close tab (Ctrl+W)")
                    .on_click(move |_, _window: &mut Window, cx: &mut App| {
                        cx.stop_propagation();
                        let _ = weak.update(cx, |view, cx| view.close_tab(tab_id, cx));
                    })
            };
            Tab::new()
                .label(SharedString::from(tab.title.clone()))
                .suffix(close)
        })
        .collect::<Vec<_>>();

    TabBar::new("dotv-tabs")
        .children(tabs)
        .selected_index(view.active_index())
        .max_width(px(220.0))
        .when(has_tabs, |bar| bar.suffix(add))
        .on_click(move |index, _window, cx: &mut App| {
            let index = *index;
            let _ = weak.update(cx, |view, cx| view.activate(index, cx));
        })
}
