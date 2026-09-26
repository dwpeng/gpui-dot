//! The tab strip: one tab per open DOT file, a tab list at the far left and
//! the `+` that opens another file right behind the last tab. Pure view
//! builder over the owning [`GraphView`](crate::app::GraphView).
//!
//! It is rendered *inside* the title bar (see [`super::title_bar`]) so the two
//! share one row, and follows the current Chrome tab design:
//!
//! * every control in the row — the tab list, a tab's close button, the `+` —
//!   sits on one horizontal centre line, so a tab is centred in the strip and
//!   the label, the `✕` and the neighbouring buttons all line up;
//! * a tab is a shape that *floats* in the strip: rounded on all four corners
//!   with a gap above and below it, rather than a block welded to the bottom
//!   edge. That is why the title bar keeps its own divider underneath;
//! * the active tab is painted with the surface the drawing itself sits on,
//!   idle tabs are transparent, and hovering one lifts it;
//! * a hairline separator stands between two idle neighbours, inset from both
//!   ends, and is left out on either side of the active tab;
//! * every tab is the same width, and every tab carries its own close button:
//!   a name too long for that width is truncated rather than widening its
//!   tab, so the strip reads as a row of equal tabs. The width is an equal
//!   share of the row, so it answers to the window. A middle click closes the
//!   tab under the cursor, and hovering one names the file's whole path;
//! * a tab is dragged by its body to another slot, which moves it there: the
//!   gesture lives entirely in the payload and the drop target, so it needs no
//!   drag state in the view;
//! * tabs are adjacent — no gap — and shrink together once they outrun the
//!   row, down to a floor, so the row never pushes the `+` or the window
//!   controls out. Whatever does not fit is still reachable from the tab list
//!   and from `Ctrl+Tab`.
//!
//! The tab list opens a search panel ([`tab_menu`]) rather than a plain menu:
//! the field filters on the file name and its path, each row names the document
//! and where it lives, and a row's `✕` closes it without leaving the panel.
//!
//! The strip is drawn by hand rather than with the kit's `TabBar` because
//! `Tab::render` applies its own corner radius *after* any the caller set, so a
//! `Tab` cannot be given Chrome's rounded shape.
//!
//! Every interactive part of the strip blocks the mouse. The title bar marks
//! the region it hands to the window manager as a caption
//! (`WindowControlArea::Drag`), and a hitbox that blocks stops that marker from
//! claiming the click — which is what keeps a tab click from being read as
//! "move the window": on Linux it keeps the title bar's own drag handlers from
//! arming, and on Windows it drops the caption area out of the hit test. The
//! empty stretch of row after the `+` stays unblocked on purpose, so dragging
//! there still moves the window.

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::ActiveTheme as _;
use gpui_kit::component::Icon;
use gpui_kit::component::Sizable as _;
use gpui_kit::component::button::{Button, ButtonVariants as _};
use gpui_kit::component::input::{Input, InputEvent, InputState};
use gpui_kit::component::popover::Popover;
use gpui_kit::component::scroll::ScrollableElement as _;
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{
    Anchor, App, AppContext as _, Background, Context, Entity, Hsla, InteractiveElement as _,
    IntoElement, MouseButton, ParentElement, Render, SharedString, StatefulInteractiveElement as _,
    Styled, Subscription, TestSupportExt as _, WeakEntity, Window, div, px, relative,
};

use crate::app::GraphView;
use crate::icons::IconName;

/// A tab floats inside the taller title bar: this tall, which leaves an equal
/// gap above and below and keeps every control in the row on one centre line.
const TAB_HEIGHT: f32 = 24.0;
/// Chrome rounds a tab on all four corners; it is a floating shape, not a
/// block with a squared-off foot.
const TAB_RADIUS: f32 = 8.0;
/// The widest a tab grows, as a share of the lane it sits in. A share is what
/// the layouter measures, so a tab's width follows its own container rather
/// than a width we work out from the window; it also keeps a single tab from
/// eating the whole row.
const TAB_MAX_SHARE: f32 = 0.2;
/// Where equal shares stop shrinking; past this the lane clips, and the
/// tab list is the way back to what scrolled out.
const TAB_MIN_WIDTH: f32 = 42.0;
/// The footprint of the two controls beside the tabs: Chrome gives the tab
/// list and the `+` the same small square, a shade lighter on hover.
const CONTROL_SIZE: f32 = 26.0;
const CONTROL_RADIUS: f32 = 6.0;
/// The search panel's width, and how tall its list grows before it scrolls.
const MENU_WIDTH: f32 = 300.0;
const MENU_LIST_MAX_HEIGHT: f32 = 360.0;

/// The retained state of the search panel's field. Held in a keyed state so the
/// query survives the re-render each keystroke causes.
struct TabSearch {
    input: Entity<InputState>,
    _subscriptions: Vec<Subscription>,
}

/// One row of the search panel: the tab's id, its name and where it lives.
type TabEntry = (u64, SharedString, SharedString);

/// Renders the tab strip for `view`: the tab list, the tabs, then the `+`.
pub fn tab_bar(view: &GraphView, cx: &mut Context<GraphView>) -> impl IntoElement {
    let weak = cx.weak_entity();
    let theme = cx.theme();
    let active_surface: Hsla = theme.background;
    let hover_tint: Background = theme.tokens.secondary_hover.into();
    let active_ink: Hsla = theme.tab_active_foreground;
    let idle_ink: Hsla = theme.tab_foreground;
    let active_index = view.active_index();
    let active_id = view.active().map(|tab| tab.id);
    let entries: Vec<TabEntry> = view
        .tabs()
        .map(|tab| {
            (
                tab.id,
                SharedString::from(tab.title.clone()),
                SharedString::from(tab.path.display().to_string()),
            )
        })
        .collect();
    let has_tabs = !entries.is_empty();

    // The tab list, at the far left: it opens the search panel below the button.
    let tab_list = {
        let weak = weak.clone();
        let entries = entries.clone();
        div()
            .id("dotv-tab-list")
            .flex_shrink_0()
            .occlude()
            .test_support()
            .child(
                Popover::new("dotv-tab-menu")
                    // Top-left: the panel's top-left corner hangs off the
                    // trigger, so the panel opens downwards into the window.
                    .anchor(Anchor::TopLeft)
                    .open(view.tab_menu_open)
                    .on_open_change({
                        let weak = weak.clone();
                        move |open, _window, cx| {
                            let _ = weak.update(cx, |view, cx| {
                                if view.tab_menu_open != *open {
                                    view.tab_menu_open = *open;
                                    cx.notify();
                                }
                            });
                        }
                    })
                    .trigger(
                        Button::new("tab-list")
                            .icon(Icon::new(IconName::ChevronDown))
                            .ghost()
                            .small()
                            .w(px(CONTROL_SIZE))
                            .h(px(CONTROL_SIZE))
                            .rounded(px(CONTROL_RADIUS))
                            .tooltip("Search tabs (Ctrl+Shift+A)"),
                    )
                    .content(move |_, window, cx| tab_menu(&weak, &entries, active_id, window, cx)),
            )
    };

    let add = {
        let weak = weak.clone();
        div().id("dotv-tab-add").flex_shrink_0().occlude().child(
            Button::new("tab-add")
                .icon(IconName::NewTab)
                .ghost()
                .small()
                .w(px(CONTROL_SIZE))
                .h(px(CONTROL_SIZE))
                .rounded(px(CONTROL_RADIUS))
                .tooltip("Open a DOT file (Ctrl+O)")
                .on_click(move |_, _window: &mut Window, cx: &mut App| {
                    let _ = weak.update(cx, |view, cx| view.open_graph(cx));
                }),
        )
    };

    let tabs = view
        .tabs()
        .enumerate()
        .map(|(index, tab)| {
            let tab_id = tab.id;
            let is_active = index == active_index;
            let fill = if is_active {
                active_surface
            } else {
                Hsla::transparent_black()
            };

            div()
                .id(SharedString::from(format!("dotv-tab-{tab_id}")))
                .relative()
                .occlude()
                .test_support()
                .min_w(px(TAB_MIN_WIDTH))
                .max_w(relative(TAB_MAX_SHARE))
                .tooltip({
                    let path = SharedString::from(tab.path.display().to_string());
                    move |_window, cx| cx.new(|_| TabTooltip { path: path.clone() }).into()
                })
                .on_drag(
                    TabDrag {
                        tab_id,
                        title: SharedString::from(tab.title.clone()),
                    },
                    |drag: &TabDrag, _offset, _window, cx| {
                        cx.new(|_| TabDragChip {
                            title: drag.title.clone(),
                        })
                    },
                )
                .drag_over::<TabDrag>(move |style, _drag, _window, _cx| {
                    style.bg(hover_tint).rounded(px(TAB_RADIUS))
                })
                .on_drop({
                    let weak = weak.clone();
                    move |drag: &TabDrag, _window, cx| {
                        let _ = weak.update(cx, |view, cx| view.move_tab(drag.tab_id, index, cx));
                    }
                })
                .h(px(TAB_HEIGHT))
                .flex()
                .items_center()
                .gap_1()
                .when_else(index == 0, |this| this.ml_0(), |this| this.ml_1())
                .mr_1()
                .px_2()
                .rounded(px(TAB_RADIUS))
                .text_sm()
                .text_color(if is_active { active_ink } else { idle_ink })
                .bg(fill)
                .when(!is_active, |this| {
                    this.hover(move |style| style.bg(hover_tint))
                })
                .child(
                    div()
                        .flex_1()
                        .min_w_0()
                        .truncate()
                        .child(SharedString::from(tab.title.clone())),
                )
                // Every tab carries one, so a tab in the background can be
                // closed without being switched to first.
                .child(close_button(tab_id, &weak))
                // The press stops here. The row above is the window's caption
                // area, and its own mouse-down arms a window move on the first
                // movement — which would turn a tab drag into a window drag.
                // A click would still work without this; a drag would not.
                .on_mouse_down(MouseButton::Left, |_, _, cx| cx.stop_propagation())
                // A middle click closes the tab under the cursor, as it does in
                // every browser: the shortcut for an idle tab, which no longer
                // shows a button of its own.
                .on_mouse_down(MouseButton::Middle, {
                    let weak = weak.clone();
                    move |_, _window: &mut Window, cx: &mut App| {
                        let _ = weak.update(cx, |view, cx| view.close_tab(tab_id, cx));
                    }
                })
                .on_click({
                    let weak = weak.clone();
                    move |_, _window: &mut Window, cx: &mut App| {
                        let _ = weak.update(cx, |view, cx| view.activate(index, cx));
                    }
                })
        })
        .collect::<Vec<_>>();

    div()
        .h_full()
        .flex_1()
        .min_w_0()
        .h_flex()
        .gap_1()
        .when(has_tabs, |strip| {
            strip.child(tab_list).child(
                div()
                    .h_full()
                    .min_w_0()
                    .flex_1()
                    .flex()
                    .items_center()
                    .overflow_hidden()
                    .children(tabs)
                    .child(add),
            )
        })
}

/// What a dragged tab hands to whatever it is dropped on: its identity, and
/// the name to paint on the chip that follows the cursor. The strip is
/// rebuilt every frame, so this payload is the whole of the gesture's state —
/// none of it has to live in the view.
#[derive(Clone)]
pub(crate) struct TabDrag {
    pub(crate) tab_id: u64,
    pub(crate) title: SharedString,
}

/// The chip that follows the cursor while a tab is dragged: the tab's own
/// name on the surface a tab is painted with, so the gesture reads as "this
/// tab is moving" rather than as a generic drag.
struct TabDragChip {
    title: SharedString,
}

impl Render for TabDragChip {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .px_2()
            .py_1()
            .rounded(px(TAB_RADIUS))
            .bg(cx.theme().background)
            .border_1()
            .border_color(cx.theme().border)
            .text_sm()
            .text_color(cx.theme().tab_active_foreground)
            .child(self.title.clone())
    }
}

/// What hovering a tab shows: where the document lives. The tab itself
/// carries only the file's name, which a narrow window truncates and which
/// two files can share; the path answers both, and the vertical slice this
/// renders is whatever the native tooltip overlay holds it in.
struct TabTooltip {
    path: SharedString,
}

impl Render for TabTooltip {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .px_2()
            .py_1()
            .rounded_md()
            .bg(cx.theme().popover)
            .border_1()
            .border_color(cx.theme().border)
            .text_xs()
            .text_color(cx.theme().popover_foreground)
            .child(self.path.clone())
    }
}

/// A tab's close button, with the observed wrapper the UI tests click through.
fn close_button(tab_id: u64, weak: &WeakEntity<GraphView>) -> impl IntoElement {
    let weak = weak.clone();
    div()
        .id(SharedString::from(format!("tab-close-{tab_id}")))
        .flex_shrink_0()
        .occlude()
        .test_support()
        // The tab around it is a drag handle; pressing the button must not
        // start dragging it.
        .on_mouse_down(MouseButton::Left, |_, _, cx| cx.stop_propagation())
        .child(
            Button::new(SharedString::from(format!("tab-close-button-{tab_id}")))
                .icon(IconName::CloseTab)
                .ghost()
                .xsmall()
                .tooltip("Close tab (Ctrl+W)")
                .on_click(move |_, _window: &mut Window, cx: &mut App| {
                    cx.stop_propagation();
                    let _ = weak.update(cx, |view, cx| view.close_tab(tab_id, cx));
                }),
        )
}

/// The tab search panel: a field over a list of every open document.
///
/// The field filters on the document's name and on its path, so two files that
/// share a name stay tellable apart. Picking a row brings that tab forward and
/// closes the panel; a row's `✕` closes the document and leaves the panel open,
/// which is what Chrome does and what makes the panel usable for tidying up.
///
/// The entries are a snapshot taken when the strip was built: the panel is
/// rebuilt on every view render, so closing a tab from here refreshes the list
/// on the same frame.
fn tab_menu(
    weak: &WeakEntity<GraphView>,
    entries: &[TabEntry],
    active_id: Option<u64>,
    window: &mut Window,
    cx: &mut App,
) -> gpui_kit::AnyElement {
    let theme = cx.theme();
    let muted = theme.muted_foreground;
    let active_row: Background = theme.tokens.list_active.into();
    let hover_row: Background = theme.tokens.list_hover.into();
    let strong = theme.foreground;

    // The query lives in a keyed state: retained across renders, so typing
    // survives the repaint each keystroke triggers.
    let search = {
        let weak = weak.clone();
        window.use_keyed_state("dotv-tab-search", cx, move |window, cx| {
            let input = cx.new(|cx| InputState::new(window, cx).placeholder("Search tabs"));
            let _subscriptions = vec![cx.subscribe_in(&input, window, {
                let weak = weak.clone();
                move |_: &mut TabSearch, _input, event: &InputEvent, _window, cx| {
                    if let InputEvent::Change = event {
                        let _ = weak.update(cx, |_view, cx| cx.notify());
                    }
                }
            })];
            TabSearch {
                input,
                _subscriptions,
            }
        })
    };
    let query = search.read(cx).input.read(cx).value().to_lowercase();

    let mut rows: Vec<gpui_kit::AnyElement> = Vec::new();
    for (tab_id, title, path) in entries {
        if !query.is_empty()
            && !title.to_lowercase().contains(&query)
            && !path.to_lowercase().contains(&query)
        {
            continue;
        }
        let is_active = active_id == Some(*tab_id);
        rows.push(
            div()
                .id(SharedString::from(format!("dotv-tab-row-{tab_id}")))
                .w_full()
                .h_flex()
                .items_center()
                .gap_2()
                .px_2()
                .py_1()
                .rounded_md()
                .occlude()
                .test_support()
                .when(is_active, |this| this.bg(active_row))
                .when(!is_active, |this| {
                    this.hover(move |style| style.bg(hover_row))
                })
                .child(Icon::new(IconName::File).size_4().text_color(muted))
                .child(
                    div()
                        .flex_1()
                        .min_w_0()
                        .v_flex()
                        .child(
                            div()
                                .truncate()
                                .text_sm()
                                .text_color(if is_active { strong } else { muted })
                                .child(title.clone()),
                        )
                        .child(
                            div()
                                .truncate()
                                .text_xs()
                                .text_color(muted)
                                .child(path.clone()),
                        ),
                )
                .child(close_button(*tab_id, weak))
                .on_click({
                    let weak = weak.clone();
                    let tab_id = *tab_id;
                    move |_, _window: &mut Window, cx: &mut App| {
                        let _ = weak.update(cx, |view, cx| {
                            view.activate_tab_id(tab_id, cx);
                            // Picking a tab is what the panel is for: close it.
                            view.tab_menu_open = false;
                            cx.notify();
                        });
                    }
                })
                .into_any_element(),
        );
    }
    if rows.is_empty() {
        rows.push(
            div()
                .px_2()
                .py_4()
                .text_sm()
                .text_color(muted)
                .child("No tab matches that")
                .into_any_element(),
        );
    }

    div()
        .id("dotv-tab-menu-panel")
        .test_support()
        .v_flex()
        .w(px(MENU_WIDTH))
        .gap_1()
        .child(
            div()
                .h_flex()
                .items_center()
                .gap_2()
                .child(
                    // The wrapper is what the UI tests focus: clicking it
                    // puts the caret in the field inside.
                    div()
                        .id("dotv-tab-search-field")
                        .flex_1()
                        .min_w_0()
                        .occlude()
                        .test_support()
                        .child(
                            Input::new(&search.read(cx).input)
                                .prefix(Icon::new(IconName::Search).size_3p5())
                                .cleanable(true)
                                .appearance(false),
                        ),
                )
                .child(
                    div()
                        .flex_shrink_0()
                        .text_xs()
                        .text_color(muted)
                        .child("Ctrl+Shift+A"),
                ),
        )
        .child(div().text_xs().text_color(muted).child("Open tabs"))
        .child(
            div()
                .v_flex()
                .gap_0p5()
                .max_h(px(MENU_LIST_MAX_HEIGHT))
                .overflow_y_scrollbar()
                .children(rows),
        )
        .into_any_element()
}
