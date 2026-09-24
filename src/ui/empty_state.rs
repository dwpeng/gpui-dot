//! The empty state: what the canvas shows when no document is loaded — a
//! large invitation icon, the open action and the alternative ways in.
//! Pure view builder over the owning [`GraphView`](crate::app::GraphView).

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::button::Button;
use gpui_kit::component::empty::{Empty, EmptyContent, EmptyDescription, EmptyHeader, EmptyMedia};
use gpui_kit::component::{ActiveTheme as _, Icon, Sizable as _};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{
    App, Context, InteractiveElement, IntoElement, ParentElement, Styled, Window, div, px,
};

use crate::app::GraphView;
use crate::icons::IconName;

pub fn empty_state(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let theme = cx.theme();
    let weak = cx.weak_entity();
    let open = {
        let weak = weak.clone();
        Button::new("empty-open")
            .label("Open a DOT file")
            .outline()
            .on_click(move |_, _window: &mut Window, cx: &mut App| {
                let _ = weak.update(cx, |view, cx| view.open_graph(cx));
            })
    };
    let (description, description_color) = match view.dialog_error.as_deref() {
        Some(error) => (error.to_string(), Some(theme.danger)),
        None => (
            "Open a DOT or Graphviz file to view its graph here.".into(),
            None,
        ),
    };

    div().v_flex().absolute().inset_0().id("dotv-empty").child(
        Empty::new()
            .header(
                EmptyHeader::new()
                    .media(
                        EmptyMedia::new().child(
                            Icon::new(IconName::OpenFile)
                                .with_size(px(48.0))
                                .text_color(theme.muted_foreground),
                        ),
                    )
                    .description(
                        EmptyDescription::new().child(
                            div()
                                .child(description)
                                .when_some(description_color, |text, color| text.text_color(color)),
                        ),
                    ),
            )
            .content(
                EmptyContent::new()
                    .child(open)
                    .child(div().text_xs().text_color(theme.muted_foreground)),
            ),
    )
}
