//! Geometry primitives for the dot engine — points and boxes in graphviz
//! coordinate conventions: units are PostScript points (1/72 inch), and the
//! layout's y axis points **up** (the renderer flips once at the end).

/// A point in dot space.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointF {
    pub x: f64,
    pub y: f64,
}

impl PointF {
    pub const ZERO: PointF = PointF { x: 0.0, y: 0.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn add(self, other: PointF) -> PointF {
        PointF::new(self.x + other.x, self.y + other.y)
    }

    pub fn sub(self, other: PointF) -> PointF {
        PointF::new(self.x - other.x, self.y - other.y)
    }

    pub fn mid(self, other: PointF) -> PointF {
        PointF::new((self.x + other.x) / 2.0, (self.y + other.y) / 2.0)
    }
}

/// A box given by its lower-left and upper-right corners (`boxf`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BoxF {
    pub ll: PointF,
    pub ur: PointF,
}

impl BoxF {
    pub fn new(ll: PointF, ur: PointF) -> Self {
        Self { ll, ur }
    }

    pub fn contains(&self, p: PointF) -> bool {
        p.x >= self.ll.x && p.x <= self.ur.x && p.y >= self.ll.y && p.y <= self.ur.y
    }
}

/// C `round()`: round half away from zero (Rust's `f64::round` matches).
pub fn round(v: f64) -> f64 {
    v.round()
}

/// C `ROUND` macro in dotgen: `(int)(f < 0 ? f - .5 : f + .5)` — truncation
/// toward the nearest integer with ties away from zero.
pub fn round_i(v: f64) -> i32 {
    (if v < 0.0 { v - 0.5 } else { v + 0.5 }) as i32
}

pub fn approx_eq(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-4
}

pub fn approx_eqpt(a: PointF, b: PointF) -> bool {
    approx_eq(a.x, b.x) && approx_eq(a.y, b.y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_matches_c_semantics() {
        assert_eq!(round_i(0.5), 1);
        assert_eq!(round_i(-0.5), -1);
        assert_eq!(round_i(1.4), 1);
        assert_eq!(round_i(-1.6), -2);
    }
}
