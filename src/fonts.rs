use std::borrow::Cow;

use gpui_kit::App;
use gpui_kit::component::Theme;

/// The font family name declared by the embedded MiSans faces.
pub const FONT_FAMILY: &str = "MiSans";

/// Loads the embedded fonts into the text system's font database.
///
/// Call this before [`gpui_kit::init`] so the faces are registered when the
/// theme resolves its default font.
pub fn register(cx: &mut App) {
    let fonts: Vec<Cow<'static, [u8]>> = vec![
        include_bytes!("../assets/fonts/MiSans-Normal.ttf")
            .as_slice()
            .into(),
    ];
    cx.text_system()
        .add_fonts(fonts)
        .expect("failed to register the embedded MiSans font");
}

/// Points the active theme's font families at the embedded typeface.
///
/// Call this after [`gpui_kit::init`], which creates the theme global. The
/// default theme ships no explicit font family, so this override survives
/// later theme / appearance changes.
pub fn apply_to_theme(cx: &mut App) {
    let theme = Theme::global_mut(cx);
    theme.font_family = FONT_FAMILY.into();
    // The app never renders monospace text; reusing the embedded family keeps
    // every text run on a face we guarantee is present.
    theme.mono_font_family = FONT_FAMILY.into();
    Theme::sync_base(cx);
}
