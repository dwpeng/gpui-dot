//! The world ↔ screen transform and the adaptive background-grid spacing.

use gpui_kit::{Pixels, Point, Size, point, px, size};

use super::layout::GRID;

/// The world → screen transform used by every paint and hit-test operation:
/// world coordinates scaled by `zoom` and shifted by `pan` (screen pixels,
/// relative to the canvas `origin`).
#[derive(Clone, Copy)]
pub(crate) struct ViewTransform {
    /// Origin of the canvas in window coordinates.
    pub origin: Point<Pixels>,
    /// Where the world origin lands on the canvas.
    pub pan: Point<Pixels>,
    pub zoom: f32,
}

impl ViewTransform {
    pub fn to_screen(self, x: f32, y: f32) -> Point<Pixels> {
        point(
            px(x * self.zoom) + self.pan.x + self.origin.x,
            px(y * self.zoom) + self.pan.y + self.origin.y,
        )
    }

    pub fn to_screen_size(self, w: f32, h: f32) -> Size<Pixels> {
        size(px(w * self.zoom), px(h * self.zoom))
    }

    pub fn to_world(self, mouse: Point<Pixels>) -> (f32, f32) {
        (
            (mouse.x - self.origin.x - self.pan.x).as_f32() / self.zoom,
            (mouse.y - self.origin.y - self.pan.y).as_f32() / self.zoom,
        )
    }
}

/// The world-unit grid spacing at `zoom`. Starting from [`GRID`], the step
/// doubles while its on-screen spacing would fall below [`GRID_MIN_SCREEN`]
/// pixels and halves while it would reach [`GRID_MIN_SCREEN`]×2, so the drawn
/// spacing always lands in `[GRID_MIN_SCREEN, GRID_MIN_SCREEN×2)`. Because the
/// result is a power-of-two multiple (or subdivision) of [`GRID`], every step
/// is a multiple of every finer step — grid lines stay at the same world
/// positions as the spacing changes.
pub(crate) fn grid_step(zoom: f32) -> f32 {
    if !zoom.is_finite() || zoom <= 0.0 {
        return GRID;
    }
    let mut step = GRID;
    while step * zoom < GRID_MIN_SCREEN {
        step *= 2.0;
    }
    while step * zoom >= GRID_MIN_SCREEN * 2.0 {
        step /= 2.0;
    }
    step
}

/// Minimum on-screen distance between adjacent grid lines, in pixels.
const GRID_MIN_SCREEN: f32 = 20.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_step_keeps_screen_spacing_in_band() {
        // At every zoom in the app's working range the on-screen spacing
        // between adjacent grid lines stays within [GRID_MIN_SCREEN, 2×).
        for zoom in [0.1, 0.25, 0.5, 0.75, 1.0, 1.6, 2.0, 3.0, 5.0, 8.0] {
            let step = grid_step(zoom);
            let screen = step * zoom;
            assert!(
                (GRID_MIN_SCREEN..GRID_MIN_SCREEN * 2.0).contains(&screen),
                "zoom {zoom}: step {step} gives {screen}px, outside [{GRID_MIN_SCREEN}, {}px)",
                GRID_MIN_SCREEN * 2.0
            );
        }
    }

    #[test]
    fn grid_step_zooms_between_power_of_two_steps() {
        assert_eq!(grid_step(1.0), GRID);
        // Zoomed out the step grows (320 world units ≈ 32 px at 10% zoom).
        assert_eq!(grid_step(0.1), GRID * 16.0);
        // Zoomed in the step subdivides (2.5 world units ≈ 20 px at 800%).
        assert_eq!(grid_step(8.0), GRID / 8.0);
        // Non-positive or non-finite zoom is guarded and returns the base.
        assert_eq!(grid_step(0.0), GRID);
        assert_eq!(grid_step(-1.0), GRID);
        assert_eq!(grid_step(f32::NAN), GRID);
    }

    #[test]
    fn grid_step_is_a_power_of_two_factor_of_base() {
        // Every step is GRID × 2^k, so coarser steps are exact multiples of
        // finer ones: lines already on the grid stay on it as the spacing
        // changes across zoom levels (no line ever jumps to a new position).
        for zoom in [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0] {
            let step = grid_step(zoom);
            let factor = step / GRID;
            assert!(factor.is_finite() && factor > 0.0);
            assert_eq!(
                factor.log2().round(),
                factor.log2(),
                "step {step} at zoom {zoom} is not GRID × 2^k"
            );
        }
    }
}
