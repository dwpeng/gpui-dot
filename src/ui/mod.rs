//! The application chrome around the canvas: title bar, floating zoom
//! controls and status bar. Pure view builders over the owning
//! [`GraphView`](crate::app::GraphView) — no state of their own.

mod drop_overlay;
mod empty_state;
mod loading;
mod status_bar;
mod tab_bar;
mod title_bar;
mod zoom_cluster;

pub use drop_overlay::drop_overlay;
pub use empty_state::empty_state;
pub use loading::loading_overlay;
pub use status_bar::status_bar;
pub use tab_bar::tab_bar;
pub use title_bar::title_bar;
pub use zoom_cluster::zoom_cluster;
