//! The application's own icon set.
//!
//! SVGs live in `assets/icons/` — copied from the gpui-kit icon catalog and
//! kept in-repo, so dotv owns the icons it renders and can replace any of
//! them without touching a dependency. [`AppAssets`] embeds the folder with
//! rust-embed and falls back to GPUI Kit's default component bundle for the
//! icons that framework components draw themselves (window controls, menus,
//! dialogs). [`IconName`] is the business-facing catalog: it plugs into
//! `Icon::new`, `Button::icon` and `Toggle::icon` through `IconNamed`, and
//! renders directly as a child element.

use std::borrow::Cow;

use gpui_kit::assets::Assets as DefaultAssets;
use gpui_kit::component::IconNamed;
use gpui_kit::{
    App, AssetSource, IntoElement, RenderOnce, Result, SharedString, Styled, Window, svg,
};
use rust_embed::RustEmbed;

/// The icons dotv names by business meaning. Each variant points at a file
/// under `assets/icons/`; several may share one source SVG (the tab strip's
/// `+` and zoom-in are both `plus.svg`).
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, IntoElement)]
pub enum IconName {
    /// Close a tab (`close.svg`).
    CloseTab,
    /// Layout direction: left-to-right ranks (`align-center-horizontal.svg`).
    DirectionHorizontal,
    /// Layout direction: top-to-bottom ranks (`align-center-vertical.svg`).
    DirectionVertical,
    /// Toggle edge labels (`baseline.svg`).
    EdgeLabels,
    /// Fit the graph to the window (`locate-fixed.svg`).
    FitView,
    /// Toggle the canvas grid (`layout-grid.svg`).
    Grid,
    /// The loading spinner: a ~300° arc whose rotation reads as one
    /// continuous motion (`loader-circle.svg`).
    LoaderCircle,
    /// Toggle fullscreen (`maximize.svg`).
    Maximize,
    /// Open a new tab / another file (`plus.svg`).
    NewTab,
    /// Toggle node labels (`square-text.svg`).
    NodeLabels,
    /// The empty-state / drop-target invitation to open a file
    /// (`file-up.svg`).
    OpenFile,
    /// Reset manually moved nodes to their layout positions (`undo-2.svg`).
    ResetLayout,
    /// Open the settings card (`settings.svg`).
    Settings,
    /// Zoom in (`plus.svg`).
    ZoomIn,
    /// Zoom out (`minus.svg`).
    ZoomOut,
}

impl IconNamed for IconName {
    fn path(self) -> SharedString {
        match self {
            IconName::CloseTab => "icons/close.svg",
            IconName::DirectionHorizontal => "icons/align-center-horizontal.svg",
            IconName::DirectionVertical => "icons/align-center-vertical.svg",
            IconName::EdgeLabels => "icons/baseline.svg",
            IconName::FitView => "icons/locate-fixed.svg",
            IconName::Grid => "icons/layout-grid.svg",
            IconName::LoaderCircle => "icons/loader-circle.svg",
            IconName::Maximize => "icons/maximize.svg",
            IconName::NewTab | IconName::ZoomIn => "icons/plus.svg",
            IconName::NodeLabels => "icons/square-text.svg",
            IconName::OpenFile => "icons/file-up.svg",
            IconName::ResetLayout => "icons/undo-2.svg",
            IconName::Settings => "icons/settings.svg",
            IconName::ZoomOut => "icons/minus.svg",
        }
        .into()
    }
}

// Mirrors the framework's own `IconName` rendering: sized to and colored by
// the surrounding text style, so `.child(IconName::X)` tracks the local text.
// Explicit themed sizes and transformations belong to the consumer's `Icon`.
impl RenderOnce for IconName {
    fn render(self, window: &mut Window, _: &mut App) -> impl IntoElement {
        let text_style = window.text_style();
        svg()
            .path(self.path())
            .flex_shrink_0()
            .size(text_style.font_size.to_pixels(window.rem_size()))
            .text_color(text_style.color)
    }
}

/// The repo's `assets/icons/*.svg`, embedded into the binary. The
/// `debug-embed` feature keeps debug builds identical to release ones —
/// icons come from the binary, not the working directory.
#[derive(RustEmbed)]
#[folder = "assets"]
#[include = "icons/*.svg"]
struct IconFiles;

/// Asset source for dotv: the repo's icons first, then GPUI Kit's default
/// component bundle for the icons framework components request on their own.
#[derive(Clone, Copy, Debug, Default)]
pub struct AppAssets;

impl AssetSource for AppAssets {
    fn load(&self, path: &str) -> Result<Option<Cow<'static, [u8]>>> {
        if path.is_empty() {
            return Ok(None);
        }

        if let Some(file) = IconFiles::get(path) {
            return Ok(Some(file.data));
        }
        DefaultAssets.load(path)
    }

    fn list(&self, path: &str) -> Result<Vec<SharedString>> {
        let mut paths = DefaultAssets.list(path)?;
        paths.extend(IconFiles::iter().filter_map(|p| p.starts_with(path).then(|| p.into())));
        paths.sort();
        paths.dedup();
        Ok(paths)
    }
}
