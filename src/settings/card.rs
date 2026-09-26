use std::rc::Rc;

use gpui_kit::base::StyledExt as _;
use gpui_kit::component::input::{InputEvent, InputState, NumberInput};
use gpui_kit::component::separator::Separator;
use gpui_kit::component::switch::Switch;
use gpui_kit::component::{ActiveTheme as _, Sizable as _};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::{
    AnyElement, App, AppContext, Entity, InteractiveElement, IntoElement, ParentElement,
    RenderOnce, SharedString, Styled, Subscription, Window, div,
};

/// Callback shape shared by every control kind.
type OnChange<T> = Rc<dyn Fn(T, &mut Window, &mut App)>;

/// One row of the settings card: a title, an optional description, and a
/// control of one of the supported kinds ([`SettingKind`]).
#[derive(Clone)]
pub struct SettingRow {
    /// Stable element id, unique per row.
    id: &'static str,
    /// The row title, shown next to the control.
    title: SharedString,
    /// Optional one-line description under the title.
    description: Option<SharedString>,
    /// The control itself.
    kind: SettingKind,
}

impl SettingRow {
    /// A boolean toggle row.
    pub fn bool(
        id: &'static str,
        title: impl Into<SharedString>,
        description: Option<SharedString>,
        value: bool,
        on_change: impl Fn(bool, &mut Window, &mut App) + 'static,
    ) -> Self {
        Self {
            id,
            title: title.into(),
            description,
            kind: SettingKind::Bool {
                value,
                on_change: Rc::new(on_change),
            },
        }
    }

    /// A numeric value row with a min/max/step range.
    #[allow(clippy::too_many_arguments)]
    pub fn value(
        id: &'static str,
        title: impl Into<SharedString>,
        description: Option<SharedString>,
        value: f64,
        min: f64,
        max: f64,
        step: f64,
        on_change: impl Fn(f64, &mut Window, &mut App) + 'static,
    ) -> Self {
        Self {
            id,
            title: title.into(),
            description,
            kind: SettingKind::Value {
                value,
                min,
                max,
                step,
                on_change: Rc::new(on_change),
            },
        }
    }
}

/// The kinds of settings control the card supports.
#[derive(Clone)]
enum SettingKind {
    /// A boolean toggle switch.
    Bool {
        value: bool,
        on_change: OnChange<bool>,
    },
    /// A numeric value with a min/max/step.
    Value {
        value: f64,
        min: f64,
        max: f64,
        step: f64,
        on_change: OnChange<f64>,
    },
}

impl SettingRow {
    fn render(self, window: &mut Window, cx: &mut App) -> AnyElement {
        // Copy the color tokens out up front so no borrow of `cx` outlives the
        // mutable calls below (`use_keyed_state`, `state_entity.update`).
        let theme = cx.theme();
        let foreground = theme.foreground;
        let muted_foreground = theme.muted_foreground;
        let SettingRow {
            id,
            title,
            description,
            kind,
        } = self;

        let control = match kind {
            SettingKind::Bool { value, on_change } => Switch::new(id)
                .checked(value)
                .small()
                .on_change({
                    let on_change = on_change.clone();
                    move |checked, window, cx| on_change(*checked, window, cx)
                })
                .into_any_element(),
            SettingKind::Value {
                value,
                min,
                max,
                step,
                on_change,
            } => {
                // A value control needs a retained `Entity<InputState>`
                // (stepping, min/max clamping and the displayed text). It is
                // created once per row with a keyed state, mirroring the
                // crate's own NumberField; later renders only re-sync it.
                let state_entity = window.use_keyed_state(
                    SharedString::from(format!("settings-{id}-value")),
                    cx,
                    move |window, cx| {
                        let input = cx.new(|cx| {
                            InputState::new(window, cx)
                                .default_value(value.to_string())
                                .step(step)
                                .min(min)
                                .max(max)
                        });
                        let _subscriptions = vec![cx.subscribe_in(&input, window, {
                            let on_change = on_change.clone();
                            move |state: &mut ValueState, input, event: &InputEvent, window, cx| {
                                if let InputEvent::Change = event {
                                    input.update(cx, |input, cx| {
                                        let text = input.value();
                                        // Unparsable intermediates are left
                                        // alone; out-of-range text stays too
                                        // and is clamped on blur.
                                        if let Ok(parsed) = text.parse::<f64>() {
                                            let clamped = parsed.clamp(min, max);
                                            // Compare numerically, not as text:
                                            // "1.50" and "1.5" are one value
                                            // and must not fire a change.
                                            if (clamped - state.initial_value).abs() > 1e-9 {
                                                on_change(clamped, window, cx);
                                                state.initial_value = clamped;
                                            }
                                        }
                                    });
                                }
                            }
                        })];
                        ValueState {
                            input,
                            initial_value: value,
                            _subscriptions,
                        }
                    },
                );

                // Keep the engine's bounds/step and the displayed text in sync
                // with the model value on every render.
                state_entity.update(cx, |state, cx| {
                    state.input.update(cx, |input, cx| {
                        input.set_step(Some(step.into()), window, cx);
                        input.set_min(Some(min), window, cx);
                        input.set_max(Some(max), window, cx);
                    });
                    if state.initial_value != value {
                        state.initial_value = value;
                        state.input.update(cx, |input, cx| {
                            input.set_value(SharedString::from(value.to_string()), window, cx);
                        });
                    }
                });

                let state = state_entity.read(cx);
                NumberInput::new(&state.input)
                    .small()
                    .w_32()
                    .into_any_element()
            }
        };

        div()
            .id(format!("settings-row-{id}"))
            .w_full()
            .h_flex()
            .items_center()
            .gap_3()
            .py_1p5()
            .child(
                div()
                    .v_flex()
                    .flex_1()
                    .gap_0p5()
                    .child(div().text_sm().text_color(foreground).child(title))
                    .when_some(description, |this, desc| {
                        this.child(div().text_xs().text_color(muted_foreground).child(desc))
                    }),
            )
            .child(control)
            .into_any_element()
    }
}

/// The retained state of a value row's number input.
struct ValueState {
    input: Entity<InputState>,
    initial_value: f64,
    _subscriptions: Vec<Subscription>,
}

/// The settings card: a fixed-width column of setting rows.
#[derive(IntoElement)]
pub struct SettingsCard {
    rows: Vec<SettingRow>,
}

impl SettingsCard {
    /// Create a card containing the given rows.
    pub fn new(rows: Vec<SettingRow>) -> Self {
        Self { rows }
    }
}

impl RenderOnce for SettingsCard {
    fn render(self, window: &mut Window, cx: &mut App) -> impl IntoElement {
        let mut children: Vec<AnyElement> = Vec::new();
        for (i, row) in self.rows.into_iter().enumerate() {
            if i > 0 {
                children.push(Separator::horizontal().into_any_element());
            }
            children.push(row.render(window, cx));
        }

        div()
            .id("settings-card")
            .w_64()
            .v_flex()
            .gap_0p5()
            .children(children)
    }
}
