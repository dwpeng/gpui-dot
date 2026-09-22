//! The title bar: a settings card on the left, the file name plus a
//! statistics popover in the middle, and the standard window controls
//! (minimize / maximize / close) rendered by the component on the right.

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::button::{Button, ButtonVariants as _};
use gpui_kit::component::popover::Popover;
use gpui_kit::component::{ActiveTheme as _, IconName, Sizable as _, TitleBar};
use gpui_kit::{
    Anchor, App, Context, InteractiveElement, IntoElement, ParentElement, Styled, Window, div, px,
};

use crate::app::GraphView;
use crate::settings::SettingsCard;
use crate::settings::panel as settings_panel;

pub fn title_bar(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();

    // Left: open-file, then the gear that opens the settings card. The open
    // button matters on Linux setups where the title bar is the only chrome —
    // without it the only way in is the Ctrl+O shortcut or a command-line
    // argument.
    let weak = cx.weak_entity();
    let open_trigger = {
        let weak = weak.clone();
        Button::new("open-file")
            .icon(IconName::FolderOpen)
            .ghost()
            .small()
            .tooltip("Open a DOT file (Ctrl+O)")
            .on_click(move |_, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| view.open_graph(cx));
            })
    };
    let settings = view.settings.clone();
    let settings_trigger = Popover::new("settings")
        .anchor(Anchor::TopLeft)
        .trigger(
            Button::new("settings")
                .icon(IconName::Settings)
                .ghost()
                .small()
                .tooltip("Settings"),
        )
        .content(move |_, _, _| SettingsCard::new(settings_panel::rows(&settings, &weak)));

    // Middle: the current file name, with a small statistics icon that opens
    // a popover when a document is loaded.
    let file_name = div()
        .id("file-name")
        .flex_none()
        .max_w(px(360.0))
        .truncate()
        .text_sm()
        .text_color(theme.foreground)
        .child(match &view.document {
            Some(document) => document.file_name(),
            None => "No file open".to_string(),
        });

    TitleBar::new()
        .child(
            div()
                .h_full()
                .h_flex()
                .items_center()
                .gap_0p5()
                .child(open_trigger)
                .child(settings_trigger),
        )
        .child(
            div()
                .flex_1()
                .h_full()
                .h_flex()
                .items_center()
                .justify_center()
                .gap_1p5()
                .child(file_name),
        )
        // Balanced spacer so the file name stays centred between the left
        // controls and the window buttons.
        .child(div().w_12())
}
