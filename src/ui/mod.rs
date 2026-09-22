//! The application chrome around the canvas: title bar, floating zoom
//! controls and status bar. Pure view builders over the owning
//! [`GraphView`](crate::app::GraphView) — no state of their own.

mod status_bar;
mod title_bar;
mod zoom_cluster;

pub use status_bar::status_bar;
pub use title_bar::title_bar;
pub use zoom_cluster::zoom_cluster;
