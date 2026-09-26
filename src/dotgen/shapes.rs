//! A faithful Rust port of Graphviz node-shape geometry
//! (`lib/common/shapes.c` @ 2e92c7f), covering everything dot layout and
//! rendering need: the `Shapes[]` registry, `poly_init` sizing, the
//! `poly_inside`/`point_inside`/`star_inside`/`record_inside` hit tests,
//! compass ports (`compassPort`), the record (`record`/`Mrecord`) field
//! parser/layout, and the `round_corners` corner transforms used for
//! painting outlines.
//!
//! Every function and constant carries a `shapes.c:LINE` citation. Units
//! follow the C exactly: node `width`/`height` attributes and
//! [`PolyInit::width_in`] are **inches**; label dims, vertices, ports and
//! returned sizes are **points** (1 in = 72 pt). Shape geometry lives in
//! the *node frame* (+y up, origin at node center) — rankdir frame
//! conversions (`ccwrotatepf`/`cwrotatepf`) are the caller's job, except
//! inside [`compass_port`] where the `flip` flag performs them (see its
//! doc comment).
//!
//! Known, deliberate deviations (all documented inline):
//!
//! * `IS_PLAIN` (shapes.c:209-211) compares `polygon == &p_plain`, so in
//!   current C main only `shape=plain` skips label padding — `plaintext`
//!   and `none` *do* get `PAD`. This port treats the whole
//!   plain/plaintext/none family as plain, per the project spec
//!   (docs/graphviz-specs/shapes-arrows.md §B.4) and its plaintext
//!   no-padding test. Flip [`PolyDesc::plain`] on `p_plaintext` for strict
//!   C-main parity.
//! * Image sizing (`shapefile`/`image` → `gvusershape_size`,
//!   shapes.c:2020-2052) is not implemented; `imagesize` stays `(0,0)`.
//! * SBOLv glyphs (promoter…lpromoter) resolve to their descriptors for
//!   sizing/hit-testing (convex 4-gon, like C) but render as plain
//!   polygons — the glyph geometry is render-only and not ported.

#![allow(dead_code)] // wired up by the splines/render phases

use std::collections::BTreeMap;

use super::geom::{BoxF, PointF};
use super::model::{Port, RankDir, ShapeInfo};

// ===========================================================================
// Constants (verbatim from the C sources)
// ===========================================================================

/// `GAP` (const.h:251) — whitespace in points around labels and between
/// peripheries. `PAD` adds `4*GAP` horizontally and `2*GAP` vertically
/// (macros.h:27-29), i.e. +16 pt x / +8 pt y in total.
pub const GAP: f64 = 4.0;

/// `RBCONST` (shapes.c:30) — max corner radius (points) for rounded /
/// diagonal corners.
pub const RBCONST: f64 = 12.0;

/// `RBCURVE` (shapes.c:31) — fraction of `t` used for the Bézier corner
/// approach in `rounded_draw`.
pub const RBCURVE: f64 = 0.5;

/// `DEF_POINT` (shapes.c:42) — default `shape=point` diameter, inches
/// (0.05 in = 3.6 pt).
pub const DEF_POINT: f64 = 0.05;

/// `MIN_POINT` (shapes.c:47) — minimum point diameter, inches (0.02 pt).
pub const MIN_POINT: f64 = 0.0003;

/// `SQRT2` (arith.h:45).
// Allowed rather than replaced by `std::f64::consts::SQRT_2`: the
// literal *is* that constant to full `f64` precision, and spelling it
// out keeps the one-to-one correspondence with the C source that the
// rest of this port maintains.
// `excessive_precision` too: the literal carries all 20 digits arith.h
// spells out, which is more than an `f64` can represent.
#[allow(clippy::approx_constant, clippy::excessive_precision)]
pub const SQRT2: f64 = 1.41421356237309504880;

/// `MC_SCALE` (const.h:99) — port `order` granularity (0..=256).
pub const MC_SCALE: f64 = 256.0;

/// `POINTS_PER_INCH` (geom.h:58).
pub const POINTS_PER_INCH: f64 = 72.0;

/// `DEFAULT_NODEWIDTH` (const.h:74), inches.
pub const DEFAULT_NODEWIDTH: f64 = 0.75;
/// `DEFAULT_NODEHEIGHT` (const.h:72), inches.
pub const DEFAULT_NODEHEIGHT: f64 = 0.5;
/// `MIN_NODEWIDTH` (const.h:75), inches.
pub const MIN_NODEWIDTH: f64 = 0.01;
/// `MIN_NODEHEIGHT` (const.h:73), inches.
pub const MIN_NODEHEIGHT: f64 = 0.02;
/// `DEFAULT_NODEPENWIDTH` (const.h:77).
pub const DEFAULT_NODEPENWIDTH: f64 = 1.0;
/// `MIN_NODEPENWIDTH` (const.h:78).
pub const MIN_NODEPENWIDTH: f64 = 0.0;

// Side bits (const.h:111-120).
pub const BOTTOM: u8 = 1 << 0;
pub const RIGHT: u8 = 1 << 1;
pub const TOP: u8 = 1 << 2;
pub const LEFT: u8 = 1 << 3;
/// `BOTTOM|RIGHT|TOP|LEFT`.
pub const ALL_SIDES: u8 = BOTTOM | RIGHT | TOP | LEFT;

// Polygon `option.shape` codes (const.h:192-219).
pub const DOGEAR: u8 = 1;
pub const TAB: u8 = 2;
pub const FOLDER: u8 = 3;
pub const BOX3D: u8 = 4;
pub const COMPONENT: u8 = 5;
pub const PROMOTER: u8 = 6;
pub const CDS: u8 = 7;
pub const TERMINATOR: u8 = 8;
pub const UTR: u8 = 9;
pub const PRIMERSITE: u8 = 10;
pub const RESTRICTIONSITE: u8 = 11;
pub const FIVEPOVERHANG: u8 = 12;
pub const THREEPOVERHANG: u8 = 13;
pub const NOVERHANG: u8 = 14;
pub const ASSEMBLY: u8 = 15;
pub const SIGNATURE: u8 = 16;
pub const INSULATOR: u8 = 17;
pub const RIBOSITE: u8 = 18;
pub const RNASTAB: u8 = 19;
pub const PROTEASESITE: u8 = 20;
pub const PROTEINSTAB: u8 = 21;
pub const RPROMOTER: u8 = 22;
pub const RARROW: u8 = 23;
pub const LARROW: u8 = 24;
pub const LPROMOTER: u8 = 25;
pub const CYLINDER: u8 = 26;

const PI: f64 = std::f64::consts::PI;

// ===========================================================================
// Shape descriptors — `polygon_t` bases (shapes.c:96-203) + `shape_desc`
// table (shapes.c:292-360)
// ===========================================================================

/// Which C function table a shape binds to (`shape_functions`,
/// shapes.c:243-290).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ShapeFns {
    /// `poly_fns` — generic polygons/ellipses.
    #[default]
    Poly,
    /// `point_fns` — `shape=point` (`point_init`/`point_inside`).
    Point,
    /// `record_fns` — `record`/`Mrecord`.
    Record,
    /// `epsf_fns` — external images (unsupported sizing; box behavior).
    Epsf,
    /// `star_fns` — non-convex star (`poly_init` + `star_inside`).
    Star,
    /// `cylinder_fns` — cylinder (`poly_init` + `poly_inside`).
    Cylinder,
}

/// Which custom vertex generator the base polygon carries
/// (`poly_desc_t`, shapes.c:33-36).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexGen {
    /// `star_gen` (shapes.c:4034-4087).
    Star,
    /// `cylinder_gen` (shapes.c:4149-4197).
    Cylinder,
}

/// Static base polygon descriptor — mirrors the `polygon_t` initializers
/// at shapes.c:96-203 (fields left out of a C initializer are zero/false).
#[derive(Debug, Clone, Copy, Default)]
pub struct PolyDesc {
    /// `regular`: force width == height.
    pub regular: bool,
    /// `peripheries`: number of drawn outlines (rings).
    pub peripheries: i32,
    /// `sides`: 1-2 = ellipse, 3+ = polygon, 0 = fully user-controlled
    /// (`shape=polygon` takes sides/skew/distortion attributes).
    pub sides: i32,
    /// `orientation` in degrees (+CCW).
    pub orientation: f64,
    /// `distortion`.
    pub distortion: f64,
    /// `skew`.
    pub skew: f64,
    /// `option.shape` code (DOGEAR/TAB/…/CYLINDER); 0 = none.
    pub shape_code: u8,
    /// `option.diagonals` (Mdiamond/Msquare/Mcircle).
    pub diagonals: bool,
    /// `option.auxlabels`.
    pub auxlabels: bool,
    /// `option.underline`.
    pub underline: bool,
    /// Custom size/vertex generator (star, cylinder).
    pub vertex_gen: Option<VertexGen>,
    /// True for the plain family — `width=height=0`, no label padding.
    /// NOTE: C `IS_PLAIN` (shapes.c:209-211) is a pointer compare against
    /// `&p_plain`, so only `plain` matches there; we flag the whole
    /// plaintext family per the project spec (see module docs).
    pub plain: bool,
}

/// `shape_desc` (types.h:189-194) resolved for a node.
#[derive(Debug, Clone)]
pub struct ShapeDesc {
    /// Resolved table name (unknown names fall back to `"box"`).
    pub name: &'static str,
    /// Bound function table.
    pub fns: ShapeFns,
    /// Base polygon descriptor (`polygon == NULL` for record/epsf → zero).
    pub poly: PolyDesc,
    /// `usershape` flag (only meaningful for the "custom" shape).
    pub usershape: bool,
}

/// The all-zero `polygon_t` (record/epsf have `polygon == NULL`).
const P_ZERO: PolyDesc = PolyDesc {
    regular: false,
    peripheries: 0,
    sides: 0,
    orientation: 0.0,
    distortion: 0.0,
    skew: 0.0,
    shape_code: 0,
    diagonals: false,
    auxlabels: false,
    underline: false,
    vertex_gen: None,
    plain: false,
};

/// `p_polygon` — user-controlled (shapes.c:96).
const P_POLYGON: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 0,
    ..P_ZERO
};
const P_ELLIPSE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 1,
    ..P_ZERO
};
const P_CIRCLE: PolyDesc = PolyDesc {
    regular: true,
    peripheries: 1,
    sides: 1,
    ..P_ZERO
};
const P_EGG: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 1,
    distortion: -0.3,
    ..P_ZERO
};
const P_TRIANGLE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 3,
    ..P_ZERO
};
/// `p_box` (shapes.c:103).
pub const P_BOX: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    ..P_ZERO
};
const P_SQUARE: PolyDesc = PolyDesc {
    regular: true,
    peripheries: 1,
    sides: 4,
    ..P_ZERO
};
const P_PLAINTEXT: PolyDesc = PolyDesc {
    sides: 4,
    plain: true,
    ..P_ZERO
};
const P_DIAMOND: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    orientation: 45.0,
    ..P_ZERO
};
const P_TRAPEZIUM: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    distortion: -0.4,
    ..P_ZERO
};
const P_PARALLELOGRAM: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    skew: 0.6,
    ..P_ZERO
};
const P_HOUSE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 5,
    distortion: -0.64,
    ..P_ZERO
};
const P_PENTAGON: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 5,
    ..P_ZERO
};
const P_HEXAGON: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 6,
    ..P_ZERO
};
const P_SEPTAGON: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 7,
    ..P_ZERO
};
const P_OCTAGON: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 8,
    ..P_ZERO
};
const P_NOTE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: DOGEAR,
    ..P_ZERO
};
const P_TAB: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: TAB,
    ..P_ZERO
};
const P_FOLDER: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: FOLDER,
    ..P_ZERO
};
const P_BOX3D: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: BOX3D,
    ..P_ZERO
};
const P_COMPONENT: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: COMPONENT,
    ..P_ZERO
};
const P_UNDERLINE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    underline: true,
    ..P_ZERO
};
const P_CYLINDER: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 19,
    shape_code: CYLINDER,
    vertex_gen: Some(VertexGen::Cylinder),
    ..P_ZERO
};
const P_DOUBLECIRCLE: PolyDesc = PolyDesc {
    regular: true,
    peripheries: 2,
    sides: 1,
    ..P_ZERO
};
const P_INVTRIANGLE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 3,
    orientation: 180.0,
    ..P_ZERO
};
const P_INVTRAPEZIUM: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    orientation: 180.0,
    distortion: -0.4,
    ..P_ZERO
};
const P_INVHOUSE: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 5,
    orientation: 180.0,
    distortion: -0.64,
    ..P_ZERO
};
const P_DOUBLEOCTAGON: PolyDesc = PolyDesc {
    peripheries: 2,
    sides: 8,
    ..P_ZERO
};
const P_TRIPLEOCTAGON: PolyDesc = PolyDesc {
    peripheries: 3,
    sides: 8,
    ..P_ZERO
};
const P_MDIAMOND: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    orientation: 45.0,
    diagonals: true,
    auxlabels: true,
    ..P_ZERO
};
const P_MSQUARE: PolyDesc = PolyDesc {
    regular: true,
    peripheries: 1,
    sides: 4,
    diagonals: true,
    ..P_ZERO
};
const P_MCIRCLE: PolyDesc = PolyDesc {
    regular: true,
    peripheries: 1,
    sides: 1,
    diagonals: true,
    auxlabels: true,
    ..P_ZERO
};
const P_STAR: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 10,
    vertex_gen: Some(VertexGen::Star),
    ..P_ZERO
};
/// SBOLv 4-sided bases (shapes.c:163-203) — sizing/hit-test only.
const P_SBOLV: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    ..P_ZERO
};
const P_SBOLV_WITH: PolyDesc = PolyDesc {
    peripheries: 1,
    sides: 4,
    shape_code: PROMOTER,
    ..P_ZERO
};

const fn sbolv(code: u8) -> PolyDesc {
    PolyDesc {
        shape_code: code,
        ..P_SBOLV_WITH
    }
}

/// `Shapes[]` (shapes.c:292-360), in table order. The first entry is the
/// fallback for unknown names (`bind_shape`/`user_shape`,
/// shapes.c:3949-3990).
pub const SHAPES: &[(&str, ShapeFns, PolyDesc)] = &[
    ("box", ShapeFns::Poly, P_BOX),
    ("polygon", ShapeFns::Poly, P_POLYGON),
    ("ellipse", ShapeFns::Poly, P_ELLIPSE),
    ("oval", ShapeFns::Poly, P_ELLIPSE),
    ("circle", ShapeFns::Poly, P_CIRCLE),
    ("point", ShapeFns::Point, P_CIRCLE),
    ("egg", ShapeFns::Poly, P_EGG),
    ("triangle", ShapeFns::Poly, P_TRIANGLE),
    ("none", ShapeFns::Poly, P_PLAINTEXT),
    ("plaintext", ShapeFns::Poly, P_PLAINTEXT),
    ("plain", ShapeFns::Poly, P_PLAINTEXT),
    ("diamond", ShapeFns::Poly, P_DIAMOND),
    ("trapezium", ShapeFns::Poly, P_TRAPEZIUM),
    ("parallelogram", ShapeFns::Poly, P_PARALLELOGRAM),
    ("house", ShapeFns::Poly, P_HOUSE),
    ("pentagon", ShapeFns::Poly, P_PENTAGON),
    ("hexagon", ShapeFns::Poly, P_HEXAGON),
    ("septagon", ShapeFns::Poly, P_SEPTAGON),
    ("octagon", ShapeFns::Poly, P_OCTAGON),
    ("note", ShapeFns::Poly, P_NOTE),
    ("tab", ShapeFns::Poly, P_TAB),
    ("folder", ShapeFns::Poly, P_FOLDER),
    ("box3d", ShapeFns::Poly, P_BOX3D),
    ("component", ShapeFns::Poly, P_COMPONENT),
    ("cylinder", ShapeFns::Cylinder, P_CYLINDER),
    ("rect", ShapeFns::Poly, P_BOX),
    ("rectangle", ShapeFns::Poly, P_BOX),
    ("square", ShapeFns::Poly, P_SQUARE),
    ("doublecircle", ShapeFns::Poly, P_DOUBLECIRCLE),
    ("doubleoctagon", ShapeFns::Poly, P_DOUBLEOCTAGON),
    ("tripleoctagon", ShapeFns::Poly, P_TRIPLEOCTAGON),
    ("invtriangle", ShapeFns::Poly, P_INVTRIANGLE),
    ("invtrapezium", ShapeFns::Poly, P_INVTRAPEZIUM),
    ("invhouse", ShapeFns::Poly, P_INVHOUSE),
    ("underline", ShapeFns::Poly, P_UNDERLINE),
    ("Mdiamond", ShapeFns::Poly, P_MDIAMOND),
    ("Msquare", ShapeFns::Poly, P_MSQUARE),
    ("Mcircle", ShapeFns::Poly, P_MCIRCLE),
    ("promoter", ShapeFns::Poly, sbolv(PROMOTER)),
    ("cds", ShapeFns::Poly, sbolv(CDS)),
    ("terminator", ShapeFns::Poly, sbolv(TERMINATOR)),
    ("utr", ShapeFns::Poly, sbolv(UTR)),
    ("insulator", ShapeFns::Poly, sbolv(INSULATOR)),
    ("ribosite", ShapeFns::Poly, sbolv(RIBOSITE)),
    ("rnastab", ShapeFns::Poly, sbolv(RNASTAB)),
    ("proteasesite", ShapeFns::Poly, sbolv(PROTEASESITE)),
    ("proteinstab", ShapeFns::Poly, sbolv(PROTEINSTAB)),
    ("primersite", ShapeFns::Poly, sbolv(PRIMERSITE)),
    ("restrictionsite", ShapeFns::Poly, sbolv(RESTRICTIONSITE)),
    ("fivepoverhang", ShapeFns::Poly, sbolv(FIVEPOVERHANG)),
    ("threepoverhang", ShapeFns::Poly, sbolv(THREEPOVERHANG)),
    ("noverhang", ShapeFns::Poly, sbolv(NOVERHANG)),
    ("assembly", ShapeFns::Poly, sbolv(ASSEMBLY)),
    ("signature", ShapeFns::Poly, sbolv(SIGNATURE)),
    ("rpromoter", ShapeFns::Poly, sbolv(RPROMOTER)),
    ("larrow", ShapeFns::Poly, sbolv(RARROW)),
    ("rarrow", ShapeFns::Poly, sbolv(LARROW)),
    ("lpromoter", ShapeFns::Poly, sbolv(LPROMOTER)),
    ("record", ShapeFns::Record, P_ZERO),
    ("Mrecord", ShapeFns::Record, P_ZERO),
    ("epsf", ShapeFns::Epsf, P_ZERO),
    ("star", ShapeFns::Star, P_STAR),
];

/// `shape_of`/`bind_shape` (shapes.c:3970-3990): exact (case-sensitive)
/// table lookup; unknown names fall back to `Shapes[0]` = box, and a
/// `shapefile`-driven "custom" node is the caller's concern (it behaves as
/// box here).
pub fn shape_of(name: &str) -> ShapeDesc {
    for (n, fns, poly) in SHAPES {
        if *n == name {
            return ShapeDesc {
                name: n,
                fns: *fns,
                poly: *poly,
                usershape: false,
            };
        }
    }
    // user_shape(): copy of Shapes[0] (shapes.c:3958)
    ShapeDesc {
        name: "box",
        fns: ShapeFns::Poly,
        poly: P_BOX,
        usershape: false,
    }
}

/// `IS_BOX` (shapes.c:205-207): the base polygon is the box descriptor.
pub fn is_box(desc: &ShapeDesc) -> bool {
    desc.poly.sides == P_BOX.sides
        && desc.poly.peripheries == P_BOX.peripheries
        && desc.poly.orientation == 0.0
        && desc.poly.skew == 0.0
        && desc.poly.distortion == 0.0
        && desc.poly.regular == P_BOX.regular
        && desc.fns == ShapeFns::Poly
        && matches!(desc.name, "box" | "rect" | "rectangle")
}

// ===========================================================================
// Attribute helpers (late_double / late_int / mapbool / sscanf)
// ===========================================================================

/// `strtod` prefix parse: optional whitespace, sign, digits with optional
/// `.`/exponent. Returns the value and the number of bytes consumed.
fn c_strtod_at(b: &[u8], mut i: usize) -> Option<(f64, usize)> {
    let start = i;
    while i < b.len() && b[i].is_ascii_whitespace() {
        i += 1;
    }
    let num_start = i;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let mut mant = false;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
        mant = true;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            mant = true;
        }
    }
    if !mant {
        return None;
    }
    let mut end = i;
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            end = j;
        }
    }
    let text = std::str::from_utf8(&b[num_start..end]).ok()?;
    let v: f64 = text.parse().ok()?;
    Some((v, end - start))
}

/// `late_double` (utils.c:55-69): attribute parse with default and minimum.
fn attr_f64(attrs: &BTreeMap<String, String>, key: &str, default: f64, minimum: f64) -> f64 {
    match attrs.get(key) {
        None => default,
        Some(s) if s.is_empty() => default,
        Some(s) => match c_strtod_at(s.as_bytes(), 0) {
            None => default, // invalid double format
            Some((v, _)) => {
                if v < minimum {
                    minimum
                } else {
                    v
                }
            }
        },
    }
}

/// `late_int` (utils.c:40-53): strtol parse with default and minimum.
fn attr_i32(attrs: &BTreeMap<String, String>, key: &str, default: i32, minimum: i32) -> i32 {
    match attrs.get(key) {
        None => default,
        Some(s) if s.is_empty() => default,
        Some(s) => {
            let t = s.trim_start();
            let mut end = 0;
            let bytes = t.as_bytes();
            if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
                end += 1;
            }
            let digits = end;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
            }
            if end == digits {
                return default; // invalid int format
            }
            match t[..end].parse::<i64>() {
                Ok(v) if v > i32::MAX as i64 => default,
                Ok(v) if (v as i32) < minimum => minimum,
                Ok(v) => v as i32,
                Err(_) => default,
            }
        }
    }
}

/// `mapBool`/`mapbool` (utils.c:326-342).
fn mapbool(v: Option<&str>) -> bool {
    let Some(s) = v else { return false };
    if s.is_empty() {
        return false;
    }
    match s.to_ascii_lowercase().as_str() {
        "false" | "no" => false,
        "true" | "yes" => true,
        _ => {
            let b = s.as_bytes();
            if b[0].is_ascii_digit() {
                // atoi prefix
                let mut end = 0;
                if b[0] == b'+' || b[0] == b'-' {
                    end = 1;
                }
                while end < b.len() && b[end].is_ascii_digit() {
                    end += 1;
                }
                s[..end].parse::<i64>().map(|v| v != 0).unwrap_or(false)
            } else {
                false
            }
        }
    }
}

fn attr_str<'a>(attrs: &'a BTreeMap<String, String>, key: &str) -> Option<&'a str> {
    attrs.get(key).map(String::as_str)
}

/// `sscanf(p, "%lf,%lf", &x, &y)` — returns the conversion count plus the
/// (possibly partially initialized) values.
fn sscanf_2lf(s: &str) -> (i32, f64, f64) {
    let b = s.as_bytes();
    let Some((v1, n1)) = c_strtod_at(b, 0) else {
        return (0, 0.0, 0.0); // EOF/0 both mean "no conversion"
    };
    let mut i = n1;
    if i >= b.len() || b[i] != b',' {
        return (1, v1, 0.0);
    }
    i += 1;
    let Some((v2, _)) = c_strtod_at(b, i) else {
        return (1, v1, 0.0);
    };
    (2, v1, v2)
}

/// `PAD(d)` (macros.h:27-29): `x += 4*GAP` (16 pt), `y += 2*GAP` (8 pt).
fn pad(d: &mut PointF) {
    d.x += 4.0 * GAP;
    d.y += 2.0 * GAP;
}

/// `quant` (shapes.c:367-370): `ceil(val/q)*q`.
fn quant(val: f64, q: f64) -> f64 {
    (val / q).ceil() * q
}

/// `RADIANS(deg)` (arith.h:49).
fn radians(deg: f64) -> f64 {
    deg / 180.0 * PI
}

/// `INCH2PS` on f64.
fn inch2ps(inches: f64) -> f64 {
    inches * POINTS_PER_INCH
}

/// Effective `ND_width` after `common_init_node` (utils.c:417-443):
/// `late_double(N_width, 0.75, 0.01)` in inches.
fn eff_width(attrs: &BTreeMap<String, String>) -> f64 {
    attr_f64(attrs, "width", DEFAULT_NODEWIDTH, MIN_NODEWIDTH)
}

/// Effective `ND_height`: `late_double(N_height, 0.5, 0.02)` in inches.
fn eff_height(attrs: &BTreeMap<String, String>) -> f64 {
    attr_f64(attrs, "height", DEFAULT_NODEHEIGHT, MIN_NODEHEIGHT)
}

/// `userSize` (shapes.c:1900-1906): max user-supplied width/height in
/// points, 0 if neither attribute is set.
fn user_size(attrs: &BTreeMap<String, String>) -> f64 {
    let w = attr_f64(attrs, "width", 0.0, MIN_NODEWIDTH);
    let h = attr_f64(attrs, "height", 0.0, MIN_NODEHEIGHT);
    inch2ps(w.max(h))
}

// ===========================================================================
// poly_init — sizing (shapes.c:1933-2383), point_init (3103-3192)
// ===========================================================================

/// The computed `polygon_t` (`ND_shape_info`) plus the node geometry the
/// init functions write back.
#[derive(Debug, Clone, Default)]
pub struct PolyShape {
    pub regular: bool,
    pub peripheries: i32,
    /// Vertex count per ring: 2 for ellipses, `sides` for polygons.
    pub sides: i32,
    pub orientation: f64,
    pub skew: f64,
    pub distortion: f64,
    /// `option.fixedshape` (`fixedsize=shape`).
    pub fixedshape: bool,
    /// `outp` rings of `sides` vertices, node frame, center origin.
    /// Rings `0..peripheries` are GAP-spaced peripheries; when
    /// `peripheries >= 1 && penwidth > 0` the final ring is the
    /// penwidth/2 outline used for spline clipping (shapes.c:2157-2165).
    pub vertices: Vec<PointF>,
}

/// Everything `poly_init`/`point_init` produce.
#[derive(Debug, Clone, Default)]
pub struct PolyInit {
    /// `ND_width` in inches (gv_nodesize derives lw/rw from this).
    pub width_in: f64,
    /// `ND_height` in inches.
    pub height_in: f64,
    /// `ND_outline_width` in inches (penwidth-inclusive).
    pub outline_width_in: f64,
    /// `ND_outline_height` in inches.
    pub outline_height_in: f64,
    /// `ND_label->valign` ('t' | 'b' | 'c'), shapes.c:2066-2070.
    pub label_valign: char,
    /// `ND_label->space` (justification area, points).
    pub label_space: PointF,
    /// False when `fixedshape` — C leaves `space.y` untouched then.
    pub label_space_y_set: bool,
    pub poly: PolyShape,
}

/// Full `poly_init` (shapes.c:1933-2383), dispatching to `point_init`
/// (shapes.c:3103-3192) for `shape=point` and returning attribute-only
/// defaults for record/epsf (whose real sizing needs the label text — see
/// [`record_layout`]).
///
/// * `label_dimen` — `ND_label->dimen` in points.
/// * `attrs` — the node's attribute map.
/// * `penwidth` — `late_double(N_penwidth, 1.0, 0.0)`.
/// * `quantum_in` — the graph `quantum` attribute in inches (0 = unset).
pub fn poly_init_info(
    desc: &ShapeDesc,
    label_dimen: PointF,
    attrs: &BTreeMap<String, String>,
    penwidth: f64,
    quantum_in: f64,
) -> PolyInit {
    let penwidth = attr_f64_wrapper(attrs, penwidth);
    match desc.fns {
        ShapeFns::Point => return point_init_info(desc, attrs, penwidth),
        ShapeFns::Record | ShapeFns::Epsf => {
            // record_init/epsf_init sizes need the label text / image file;
            // the caller must use record_layout for records. Keep the
            // attribute size as the placeholder (records add the +1 pt
            // height kluge in record_node_size).
            let (w, h) = (eff_width(attrs), eff_height(attrs));
            return PolyInit {
                width_in: w,
                height_in: h,
                outline_width_in: w,
                outline_height_in: h,
                label_valign: 'c',
                label_space: label_dimen,
                label_space_y_set: true,
                poly: PolyShape::default(),
            };
        }
        ShapeFns::Poly | ShapeFns::Star | ShapeFns::Cylinder => {}
    }
    poly_init_poly(desc, label_dimen, attrs, penwidth, quantum_in)
}

/// The node's penwidth as `late_double(N_penwidth, DEFAULT, MIN)` sees it.
fn attr_f64_wrapper(_attrs: &BTreeMap<String, String>, penwidth: f64) -> f64 {
    penwidth.max(MIN_NODEPENWIDTH)
}

/// `point_init` (shapes.c:3103-3192).
fn point_init_info(desc: &ShapeDesc, attrs: &BTreeMap<String, String>, penwidth: f64) -> PolyInit {
    let dbl_max = f64::MAX;
    let w = attr_f64(attrs, "width", dbl_max, MIN_NODEWIDTH);
    let h = attr_f64(attrs, "height", dbl_max, MIN_NODEHEIGHT);
    let mut w = w.min(h);
    if w == dbl_max && h == dbl_max {
        // neither defined (shapes.c:3120-3122)
        w = DEF_POINT;
    } else {
        w = w.min(h);
        if w > 0.0 {
            w = w.max(MIN_POINT);
        }
    }
    let mut sz = w * POINTS_PER_INCH;
    let peripheries = attr_i32(attrs, "peripheries", desc.poly.peripheries, 0);
    let mut outp = if peripheries < 1 { 1 } else { peripheries };
    if peripheries >= 1 && penwidth > 0.0 {
        outp += 1;
    }
    let mut vertices: Vec<PointF> = Vec::with_capacity((outp * 2) as usize);
    let mut p = PointF::new(sz / 2.0, sz / 2.0);
    vertices.push(PointF::new(-p.x, -p.y));
    vertices.push(p);
    if peripheries > 1 {
        for _ in 1..peripheries {
            p.x += GAP;
            p.y += GAP;
            vertices.push(PointF::new(-p.x, -p.y));
            vertices.push(p);
        }
        sz = 2.0 * p.x;
    }
    if peripheries >= 1 && penwidth > 0.0 && outp > peripheries {
        // outline at half the penwidth outside the outermost periphery
        p.x += penwidth / 2.0;
        p.y += penwidth / 2.0;
        vertices.push(PointF::new(-p.x, -p.y));
        vertices.push(p);
    }
    let sz_outline = 2.0 * p.x;
    PolyInit {
        width_in: sz / POINTS_PER_INCH,
        height_in: sz / POINTS_PER_INCH,
        outline_width_in: sz_outline / POINTS_PER_INCH,
        outline_height_in: sz_outline / POINTS_PER_INCH,
        label_valign: 'c',
        label_space: PointF::ZERO,
        label_space_y_set: false,
        poly: PolyShape {
            regular: true,
            peripheries,
            sides: 2,
            orientation: 0.0,
            skew: 0.0,
            distortion: 0.0,
            fixedshape: false,
            vertices,
        },
    }
}

/// `poly_init` proper (shapes.c:1933-2383).
fn poly_init_poly(
    desc: &ShapeDesc,
    label_dimen: PointF,
    attrs: &BTreeMap<String, String>,
    penwidth: f64,
    quantum_in: f64,
) -> PolyInit {
    let base = &desc.poly;
    let is_plain = base.plain; // IS_PLAIN (shapes.c:209-211) — see module docs
    let regular = base.regular || mapbool(attr_str(attrs, "regular"));
    let mut peripheries = base.peripheries;
    let mut sides = base.sides;
    let mut orientation = base.orientation;
    let mut skew = base.skew;
    let mut distortion = base.distortion;

    // all calculations in floating point POINTS (shapes.c:1955-1977)
    let (mut width, mut height);
    if is_plain {
        width = 0.0;
        height = 0.0;
    } else if regular {
        let sz = user_size(attrs);
        if sz > 0.0 {
            width = sz;
            height = sz;
        } else {
            width = inch2ps(eff_width(attrs).min(eff_height(attrs)));
            height = width;
        }
    } else {
        width = inch2ps(eff_width(attrs));
        height = inch2ps(eff_height(attrs));
    }

    peripheries = attr_i32(attrs, "peripheries", peripheries, 0);
    orientation += attr_f64(attrs, "orientation", 0.0, -360.0);
    if sides == 0 {
        // not for builtins (shapes.c:1981-1985)
        skew = attr_f64(attrs, "skew", 0.0, -100.0);
        sides = attr_i32(attrs, "sides", 4, 0);
        distortion = attr_f64(attrs, "distortion", 0.0, -100.0);
    }

    // get label dimensions + minimal whitespace around label
    let mut dimen = label_dimen;
    if (dimen.x > 0.0 || dimen.y > 0.0)
        && !is_plain
    {
        match attr_str(attrs, "margin") {
            Some(m) => {
                let (i, marginx, marginy) = sscanf_2lf(m);
                let (marginx, marginy) = (marginx.max(0.0), marginy.max(0.0));
                if i > 0 {
                    dimen.x += 2.0 * inch2ps(marginx);
                    if i > 1 {
                        dimen.y += 2.0 * inch2ps(marginy);
                    } else {
                        dimen.y += 2.0 * inch2ps(marginx);
                    }
                } else {
                    pad(&mut dimen);
                }
            }
            None => pad(&mut dimen),
        }
    }
    let spacex = dimen.x - label_dimen.x;

    // quantization (shapes.c:2013-2018)
    if quantum_in > 0.0 {
        let q = inch2ps(quantum_in);
        dimen.x = quant(dimen.x, q);
        dimen.y = quant(dimen.y, q);
    }
    // imagesize (shapes.c:2020-2052) — not ported; stays (0,0).
    let mut bb = PointF::new(dimen.x.max(0.0), dimen.y.max(0.0));

    // "I don't know how to distort or skew ellipses in postscript"
    if sides <= 2 && (distortion != 0.0 || skew != 0.0) {
        sides = 120; // shapes.c:2060-2063
    }

    // labelloc (shapes.c:2065-2070)
    let label_valign = match attr_str(attrs, "labelloc") {
        Some(s) if s.starts_with('t') || s.starts_with('b') => s.as_bytes()[0] as char,
        _ => 'c',
    };

    let is_box =
        sides == 4 && orientation.rem_euclid(90.0).abs() < 0.5 && distortion == 0.0 && skew == 0.0;
    if is_box {
        // for regular boxes the fit should be exact (shapes.c:2074-2075)
    } else if let Some(vg) = base.vertex_gen {
        bb = match vg {
            VertexGen::Star => star_size(bb),
            VertexGen::Cylinder => cylinder_size(bb),
        };
    } else {
        // smallest ellipse containing bb, then pad (shapes.c:2079-2103)
        let temp = bb.y * SQRT2;
        if height > temp && label_valign == 'c' {
            bb.x *= (1.0 / (1.0 - (bb.y / height) * (bb.y / height))).sqrt();
        } else {
            bb.x *= SQRT2;
            bb.y = temp;
        }
        if sides > 2 {
            let temp = (PI / sides as f64).cos();
            bb.x /= temp;
            bb.y /= temp;
        }
    }
    let min_bb = bb;

    // honor width/height / fixed (shapes.c:2108-2123)
    let mut fixedshape = false;
    let fxd = attr_str(attrs, "fixed").unwrap_or("false");
    if fxd.as_bytes().first() == Some(&b's') && fxd == "shape" {
        bb = PointF::new(width, height);
        fixedshape = true;
    } else if mapbool(Some(fxd)) {
        // size-too-small warning skipped
        bb = PointF::new(width, height);
    } else {
        bb.x = width.max(bb.x);
        width = bb.x;
        bb.y = height.max(bb.y);
        height = bb.y;
    }

    // regular: final size must be square (shapes.c:2128-2130)
    if regular {
        let m = bb.x.max(bb.y);
        width = m;
        height = m;
        bb.x = m;
        bb.y = m;
    }

    // label justification space (shapes.c:2132-2152)
    let mut label_space = PointF::ZERO;
    if !mapbool(attr_str(attrs, "nojustify")) {
        if is_box {
            label_space.x = dimen.x.max(bb.x) - spacex;
        } else if dimen.y < bb.y {
            let temp = bb.x * (1.0 - (dimen.y * dimen.y) / (bb.y * bb.y)).sqrt();
            label_space.x = dimen.x.max(temp) - spacex;
        } else {
            label_space.x = dimen.x - spacex;
        }
    } else {
        label_space.x = dimen.x - spacex;
    }
    let mut label_space_y_set = false;
    if !fixedshape {
        let mut temp = bb.y - min_bb.y;
        if dimen.y < 0.0 {
            // imagesize.y is 0 — branch kept for citation parity
            temp += 0.0 - dimen.y;
        }
        label_space.y = dimen.y + temp;
        label_space_y_set = true;
    }

    // rings: peripheries plus the penwidth outline (shapes.c:2154-2165)
    let mut outp = peripheries;
    if peripheries < 1 {
        outp = 1;
    }
    if peripheries >= 1 && penwidth > 0.0 {
        // allocate extra vertices representing the outline, i.e. the
        // outermost periphery with penwidth taken into account
        outp += 1;
    }

    let mut vertices: Vec<PointF>;
    let mut outline_bb;
    if sides < 3 {
        // ellipses (shapes.c:2167-2197)
        sides = 2;
        vertices = Vec::with_capacity((outp * 2) as usize);
        let mut p = PointF::new(bb.x / 2.0, bb.y / 2.0);
        vertices.push(PointF::new(-p.x, -p.y));
        vertices.push(p);
        if peripheries > 1 {
            for _ in 1..peripheries {
                p.x += GAP;
                p.y += GAP;
                vertices.push(PointF::new(-p.x, -p.y));
                vertices.push(p);
            }
            bb.x = 2.0 * p.x;
            bb.y = 2.0 * p.y;
        }
        outline_bb = bb;
        if outp > peripheries {
            p.x += penwidth / 2.0;
            p.y += penwidth / 2.0;
            vertices.push(PointF::new(-p.x, -p.y));
            vertices.push(p);
            outline_bb.x = 2.0 * p.x;
            outline_bb.y = 2.0 * p.y;
        }
    } else {
        // polygons (shapes.c:2198-2360)
        vertices = vec![PointF::ZERO; (outp * sides) as usize];
        let mut ring0: Vec<PointF>;
        if let Some(vg) = base.vertex_gen {
            // vertex_gen mutates bb to the generated size (shapes.c:2215-2219)
            let mut sz = bb;
            match vg {
                VertexGen::Star => {
                    ring0 = Vec::with_capacity(sides as usize);
                    star_vertices_into(&mut ring0, &mut sz);
                }
                VertexGen::Cylinder => {
                    ring0 = cylinder_vertices(sz);
                    // cylinder_vertices does not modify bb
                }
            }
            let xmax = sz.x;
            let ymax = sz.y;
            bb = PointF::new(width.max(xmax), height.max(ymax));
            outline_bb = bb;
            let scalex = bb.x / xmax;
            let scaley = bb.y / ymax;
            for v in ring0.iter_mut() {
                v.x *= scalex;
                v.y *= scaley;
            }
        } else {
            let unit = gen_poly_vertices_inner(sides, skew, distortion, orientation);
            let xmax_unit = unit.iter().map(|v| v.x.abs()).fold(0.0f64, f64::max);
            let ymax_unit = unit.iter().map(|v| v.y.abs()).fold(0.0f64, f64::max);
            let xmax = 2.0 * xmax_unit * bb.x;
            let ymax = 2.0 * ymax_unit * bb.y;
            bb = PointF::new(width.max(xmax), height.max(ymax));
            outline_bb = bb;
            // vertices[i] = unit[i] * bb_old * (bb_new / xmax) — algebraically
            // identical to the C scale loop (shapes.c:2280-2289).
            ring0 = unit
                .into_iter()
                .map(|v| {
                    PointF::new(
                        v.x * bb.x / (2.0 * xmax_unit),
                        v.y * bb.y / (2.0 * ymax_unit),
                    )
                })
                .collect();
        }
        vertices[..ring0.len()].copy_from_slice(&ring0);

        if outp > 1 {
            add_periphery_rings(&mut vertices, sides as usize, peripheries, penwidth);
            // final bbs from the outermost rings (shapes.c:2352-2359)
            let per = peripheries.max(1) as usize;
            for i in 0..sides as usize {
                let p = vertices[i + (per - 1) * sides as usize];
                bb.x = (2.0 * p.x.abs()).max(bb.x);
                bb.y = (2.0 * p.y.abs()).max(bb.y);
                let q = vertices[i + (outp as usize - 1) * sides as usize];
                outline_bb.x = (2.0 * q.x.abs()).max(outline_bb.x);
                outline_bb.y = (2.0 * q.y.abs()).max(outline_bb.y);
            }
        } else {
            outline_bb = bb;
        }
    }

    let poly = PolyShape {
        regular,
        peripheries,
        sides,
        orientation,
        skew,
        distortion,
        fixedshape,
        vertices,
    };
    let (w_in, h_in, ow_in, oh_in) = if fixedshape {
        (
            label_dimen.x.max(bb.x) / POINTS_PER_INCH,
            label_dimen.y.max(bb.y) / POINTS_PER_INCH,
            label_dimen.x.max(outline_bb.x) / POINTS_PER_INCH,
            label_dimen.y.max(outline_bb.y) / POINTS_PER_INCH,
        )
    } else {
        (
            bb.x / POINTS_PER_INCH,
            bb.y / POINTS_PER_INCH,
            outline_bb.x / POINTS_PER_INCH,
            outline_bb.y / POINTS_PER_INCH,
        )
    };
    PolyInit {
        width_in: w_in,
        height_in: h_in,
        outline_width_in: ow_in,
        outline_height_in: oh_in,
        label_valign,
        label_space,
        label_space_y_set,
        poly,
    }
}

/// `gv_nodesize` (utils.c:1525-1536) — `(ND_lw, ND_rw, ND_ht)` in points.
/// Pass `flip = GD_flip(g)` (LR/BT). Unflipped sizing is `flip = false`.
pub fn gv_nodesize(width_in: f64, height_in: f64, flip: bool) -> (f64, f64, f64) {
    if flip {
        let w = inch2ps(height_in);
        (w / 2.0, w / 2.0, inch2ps(width_in))
    } else {
        let w = inch2ps(width_in);
        (w / 2.0, w / 2.0, inch2ps(height_in))
    }
}

/// `shape_size` — the node size in **points** `(w, h)` after
/// `poly_init`/`point_init`. `regular_quantum` is the graph `quantum`
/// attribute in inches (0 = unset). For `record`/`Mrecord` this returns
/// the attribute size (height + the 1 pt kluge of record_init,
/// shapes.c:3704); the authoritative record size comes from
/// [`record_layout`] + [`record_node_size`].
pub fn shape_size(
    desc: &ShapeDesc,
    label_dimen: PointF,
    attrs: &BTreeMap<String, String>,
    regular_quantum: f64,
) -> (f64, f64) {
    let info = poly_init_info(
        desc,
        label_dimen,
        attrs,
        DEFAULT_NODEPENWIDTH,
        regular_quantum,
    );
    match desc.fns {
        ShapeFns::Record => (
            info.width_in * POINTS_PER_INCH,
            info.height_in * POINTS_PER_INCH + 1.0,
        ),
        _ => (
            info.width_in * POINTS_PER_INCH,
            info.height_in * POINTS_PER_INCH,
        ),
    }
}

// ===========================================================================
// Vertex generation (shapes.c:2220-2271, star 4039-4087, cylinder 4153-4197)
// ===========================================================================

/// Unit vertex generation for a polygon: circumradius 0.5 in the node
/// frame, with `skewdist`/`gdistortion`/`gskew` and `orientation` applied
/// (shapes.c:2221-2271). The `regular` parameter mirrors the C signature
/// but does not affect generation (regularity is a sizing property).
/// When the result is an axis-aligned 4-gon the exact-symmetry break of
/// shapes.c:2265-2270 applies.
pub fn gen_poly_vertices(
    sides: i32,
    skew: f64,
    distortion: f64,
    regular: bool,
    orientation: f64,
) -> Vec<PointF> {
    let _ = regular;
    gen_poly_vertices_inner(sides, skew, distortion, orientation)
}

/// The inner generator (also used by `poly_init`).
fn gen_poly_vertices_inner(
    sides: i32,
    skew: f64,
    distortion: f64,
    orientation: f64,
) -> Vec<PointF> {
    let sides = sides as usize;
    let sectorangle = 2.0 * PI / sides as f64;
    let sidelength = (sectorangle / 2.0).sin();
    let skewdist = (distortion.abs() + skew.abs()).hypot(1.0);
    let gdistortion = distortion * SQRT2 / (sectorangle / 2.0).cos();
    let gskew = skew / 2.0;
    let mut angle = (sectorangle - PI) / 2.0;
    let mut sinx = angle.sin();
    let mut cosx = angle.cos();
    let mut r = PointF::new(0.5 * cosx, 0.5 * sinx);
    let mut vertices: Vec<PointF> = Vec::with_capacity(sides);
    angle += (PI - sectorangle) / 2.0;
    let is_box =
        sides == 4 && orientation.rem_euclid(90.0).abs() < 0.5 && distortion == 0.0 && skew == 0.0;
    for _ in 0..sides {
        // next regular vertex
        angle += sectorangle;
        sinx = angle.sin();
        cosx = angle.cos();
        r.x += sidelength * cosx;
        r.y += sidelength * sinx;

        // distort and skew
        let mut p = PointF::new(r.x * (skewdist + r.y * gdistortion) + r.y * gskew, r.y);

        // orient
        let alpha = radians(orientation) + p.y.atan2(p.x);
        sinx = alpha.sin();
        cosx = alpha.cos();
        let len = p.x.hypot(p.y);
        p.x = len * cosx;
        p.y = len * sinx;

        vertices.push(p);
        if is_box {
            // enforce exact symmetry of box (shapes.c:2265-2270)
            vertices.push(PointF::new(-p.x, p.y));
            vertices.push(PointF::new(-p.x, -p.y));
            vertices.push(PointF::new(p.x, -p.y));
            break;
        }
    }
    vertices
}

// star constants (shapes.c:4034-4037)
const STAR_ALPHA: f64 = PI / 10.0;
const STAR_ALPHA2: f64 = 2.0 * STAR_ALPHA;
const STAR_ALPHA3: f64 = 3.0 * STAR_ALPHA;
const STAR_ALPHA4: f64 = 2.0 * STAR_ALPHA2;

/// `star_size` (shapes.c:4039-4052) — fit a bounding box to the star.
fn star_size(sz0: PointF) -> PointF {
    let rx = sz0.x / (2.0 * STAR_ALPHA.cos());
    let ry = sz0.y / (STAR_ALPHA.sin() + STAR_ALPHA3.sin());
    let r0 = rx.max(ry);
    let r = r0 * STAR_ALPHA4.sin() * STAR_ALPHA2.cos() / (STAR_ALPHA.cos() * STAR_ALPHA4.cos());
    PointF::new(2.0 * r * STAR_ALPHA.cos(), r * (1.0 + STAR_ALPHA3.sin()))
}

/// `star_vertices` (shapes.c:4054-4087) — 10 points; updates `bb` to the
/// aspect-fixed size.
fn star_vertices_into(vertices: &mut Vec<PointF>, bb: &mut PointF) {
    let mut sz = *bb;
    let aspect = (1.0 + STAR_ALPHA3.sin()) / (2.0 * STAR_ALPHA.cos());
    let a = if sz.x == 0.0 {
        f64::INFINITY
    } else {
        sz.y / sz.x
    };
    if a > aspect {
        sz.x = sz.y / aspect;
    } else if a < aspect {
        sz.y = sz.x * aspect;
    }
    let r = sz.x / (2.0 * STAR_ALPHA.cos());
    let r0 = r * STAR_ALPHA.cos() * STAR_ALPHA4.cos() / (STAR_ALPHA4.sin() * STAR_ALPHA2.cos());
    let offset = (r * (1.0 - STAR_ALPHA3.sin())) / 2.0;
    let mut theta = STAR_ALPHA;
    let mut i = 0;
    while i < 10 {
        vertices.push(PointF::new(r * theta.cos(), r * theta.sin() - offset));
        theta += STAR_ALPHA2;
        vertices.push(PointF::new(r0 * theta.cos(), r0 * theta.sin() - offset));
        theta += STAR_ALPHA2;
        i += 2;
    }
    *bb = sz;
}

/// `cylinder_size` (shapes.c:4153-4157) — extra height for the caps.
fn cylinder_size(sz: PointF) -> PointF {
    PointF::new(sz.x, sz.y * 1.375)
}

/// `cylinder_vertices` (shapes.c:4159-4197) — 19 points filling the box.
fn cylinder_vertices(bb: PointF) -> Vec<PointF> {
    let x = bb.x / 2.0;
    let y = bb.y / 2.0;
    let yr = bb.y / 11.0;
    let k = 0.551784; // ellipse κ ≈ 1 − cos 22.5°
    let mut v = vec![PointF::ZERO; 19];
    v[0] = PointF::new(x, y - yr);
    v[1] = PointF::new(x, y - (1.0 - k) * yr);
    v[2] = PointF::new(k * x, y);
    v[3] = PointF::new(0.0, y);
    v[4] = PointF::new(-k * x, y);
    v[5] = PointF::new(-x, v[1].y);
    v[6] = PointF::new(-x, y - yr);
    v[7] = v[6];
    v[8] = PointF::new(-x, yr - y);
    v[9] = v[8];
    v[10] = PointF::new(-x, -v[1].y);
    v[11] = PointF::new(v[4].x, -v[4].y);
    v[12] = PointF::new(v[3].x, -v[3].y);
    v[13] = PointF::new(v[2].x, -v[2].y);
    v[14] = PointF::new(v[1].x, -v[1].y);
    v[15] = PointF::new(v[0].x, -v[0].y);
    v[16] = v[15];
    v[17] = v[0];
    v[18] = v[0];
    v
}

/// Periphery rings: offset each ring-0 vertex along its angle bisector by
/// `GAP` per ring, plus a `penwidth/2` outline ring (shapes.c:2291-2350).
/// `vertices` must be `outp*sides` long with ring 0 filled.
fn add_periphery_rings(vertices: &mut [PointF], sides: usize, peripheries: i32, penwidth: f64) {
    let outp = vertices.len() / sides;
    let i_scan = sides; // C leaves its loop variable at `sides` here
    let r = vertices[0];
    let mut q = vertices[0];
    for j in 1..sides {
        q = vertices[(i_scan - j) % sides];
        if q.x != r.x || q.y != r.y {
            break;
        }
    }
    let mut beta = (r.y - q.y).atan2(r.x - q.x);
    let mut qprev = q;
    let (mut sinx, mut cosx) = (0.0f64, 0.0f64);
    for i in 0..sides {
        // for each vertex find the bisector
        let mut qq = vertices[i];
        if qq.x != qprev.x || qq.y != qprev.y {
            let mut rr = vertices[i];
            for j in 1..sides {
                rr = vertices[(i + j) % sides];
                if rr.x != qq.x || rr.y != qq.y {
                    break;
                }
            }
            let alpha = beta;
            beta = (rr.y - qq.y).atan2(rr.x - qq.x);
            let gamma = (alpha + PI - beta) / 2.0;
            // distance along the bisector to the intersection of the next
            // periphery
            let temp = GAP / gamma.sin();
            sinx = (alpha - gamma).sin() * temp;
            cosx = (alpha - gamma).cos() * temp;
        }
        qprev = qq;
        // save the vertices of all the peripheries at this base vertex
        for j in 1..peripheries.max(0) as usize {
            qq.x += cosx;
            qq.y += sinx;
            vertices[i + j * sides] = qq;
        }
        if outp > peripheries.max(0) as usize {
            // outline at half the penwidth outside the outermost periphery
            qq.x += cosx * penwidth / 2.0 / GAP;
            qq.y += sinx * penwidth / 2.0 / GAP;
            vertices[i + peripheries.max(0) as usize * sides] = qq;
        }
    }
}

/// `polyBB` (utils.c:592-608) — bbox of the outermost periphery ring.
pub fn poly_bb(sides: i32, peripheries: i32, vertices: &[PointF]) -> Option<BoxF> {
    let sides = sides as usize;
    if sides == 0 || vertices.len() < sides {
        return None;
    }
    let peris = peripheries.max(1) as usize;
    let base = (peris - 1) * sides;
    if base + sides > vertices.len() {
        return None;
    }
    let mut bb = BoxF::new(vertices[base], vertices[base]);
    for v in &vertices[base..base + sides] {
        bb.ll.x = bb.ll.x.min(v.x);
        bb.ll.y = bb.ll.y.min(v.y);
        bb.ur.x = bb.ur.x.max(v.x);
        bb.ur.y = bb.ur.y.max(v.y);
    }
    Some(bb)
}

// ===========================================================================
// Inside tests (shapes.c:373-386, 2395-2535, 3194-3233, 3762-3787,
// 4089-4147)
// ===========================================================================

/// `same_side` (shapes.c:373-386) — are `p0` and `p1` on the same side of
/// the line through `l0`,`l1`?
pub fn same_side(p0: PointF, p1: PointF, l0: PointF, l1: PointF) -> bool {
    // a x + b y = c
    let a = -(l1.y - l0.y);
    let b = l1.x - l0.x;
    let c = a * l0.x + b * l0.y;
    let s0 = a * p0.x + b * p0.y - c >= 0.0;
    let s1 = a * p1.x + b * p1.y - c >= 0.0;
    s0 == s1
}

/// The convex walk of `poly_inside` (shapes.c:2507-2534) over one ring.
/// `base` is the ring's first index in `vertex`. `last` (the cached
/// segment) starts at 0, as in a freshly zeroed `inside_t`.
fn convex_inside(sides: usize, vertex: &[PointF], base: usize, p: PointF) -> bool {
    const O: PointF = PointF::ZERO;
    // fast test in case we are converging on a segment
    let mut i = 0usize; // ictxt.last % sides
    let mut i1 = (i + 1) % sides;
    let q = vertex[base + i];
    let r = vertex[base + i1];
    if !same_side(p, O, q, r) {
        return false; // outside the segment's face
    }
    let s = same_side(p, q, r, O) && same_side(p, r, O, q);
    if s {
        return true; // between the segment's sides
    }
    for _ in 1..sides {
        if s {
            i = i1;
            i1 = (i + 1) % sides;
        } else {
            i1 = i;
            i = (i + sides - 1) % sides;
        }
        if !same_side(p, O, vertex[base + i], vertex[base + i1]) {
            return false;
        }
    }
    true
}

/// `star_inside` (shapes.c:4089-4147) — non-convex test over rays.
fn star_inside(sides: usize, vertex: &[PointF], base: usize, p: PointF) -> bool {
    const O: PointF = PointF::ZERO;
    let mut outcnt = 0;
    let mut i = 0usize;
    while i < sides {
        let q = vertex[base + i];
        let r = vertex[base + (i + 4) % sides];
        if !same_side(p, O, q, r) {
            outcnt += 1;
            if outcnt == 2 {
                return false;
            }
        }
        i += 2;
    }
    true
}

/// The shared vertex-based test: bbox rejection (outline ring), ellipse
/// test, star rays, convex walk. `ring_count*sides == vertex.len()`.
fn vertex_inside(
    sides: usize,
    ring_count: usize,
    vertex: &[PointF],
    star: bool,
    p: PointF,
) -> bool {
    let base = (ring_count - 1) * sides; // outp = outermost (outline) ring
    let mut box_urx = 0.0f64;
    let mut box_ury = 0.0f64;
    for v in &vertex[base..base + sides] {
        box_urx = box_urx.max(v.x.abs());
        box_ury = box_ury.max(v.y.abs());
    }
    // scale factors: shapes.c:2465-2470 evaluate to exactly 1 for
    // gv_nodesize-derived lw/rw/ht, so no scaling is applied here.
    if p.x.abs() > box_urx || p.y.abs() > box_ury {
        return false;
    }
    if sides <= 2 {
        return (p.x / box_urx).hypot(p.y / box_ury) < 1.0;
    }
    if star {
        star_inside(sides, vertex, base, p)
    } else {
        convex_inside(sides, vertex, base, p)
    }
}

/// Rebuild the periphery rings from the final node box — used when no
/// cached `ShapeInfo::vertices` is available (see [`build_rings`]).
fn build_rings_flat(desc: &ShapeDesc, w: f64, h: f64, penwidth: f64) -> (usize, Vec<PointF>) {
    let rings = build_rings(desc, w, h, penwidth);
    let sides = rings.first().map_or(0, Vec::len);
    (sides, rings.into_iter().flatten().collect())
}

/// `poly_inside_test` — `poly_inside` (shapes.c:2395-2535), dispatching to
/// `point_inside`/`star_inside`/`record_inside` semantics by shape kind.
///
/// * `p` is relative to the node center in the **node frame** (+y up);
///   callers must apply `ccwrotatepf(p, 90*rankdir)` first for flipped
///   rankdirs (the C inside functions do it internally).
/// * `lw`, `rw`, `ht` are the stored `ND_lw`/`ND_rw`/`ND_ht` (points) —
///   used only for the fallbacks when `info.vertices` is empty.
/// * `bp` (port rectangle) short-circuits to `INSIDE(P, *bp)`
///   (shapes.c:2420-2424).
/// * `info` should come from [`poly_init_info`] (via `ShapeInfo::vertices`)
///   so the outline ring is available.
pub fn poly_inside_test(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: &ShapeInfo,
    bp: Option<BoxF>,
    p: PointF,
) -> bool {
    if let Some(b) = bp {
        return b.contains(p);
    }
    match desc.fns {
        // record_inside fallback without the field tree: the root field
        // box is the node box expanded by penwidth/2 (shapes.c:3762-3787,
        // default penwidth 1.0).
        ShapeFns::Record => {
            let ext = DEFAULT_NODEPENWIDTH / 2.0;
            return p.x >= -lw - ext
                && p.x <= rw + ext
                && p.y >= -ht / 2.0 - ext
                && p.y <= ht / 2.0 + ext;
        }
        // epsf_inside (shapes.c:3992-4001)
        ShapeFns::Epsf => {
            let x2 = ht / 2.0;
            return p.y >= -x2 && p.y <= x2 && p.x >= -lw && p.x <= rw;
        }
        ShapeFns::Poly | ShapeFns::Point | ShapeFns::Star | ShapeFns::Cylinder => {}
    }
    // Ellipse family (`sides` 1-2, and `circle`): `poly_inside`'s hypot test.
    // The stored rings hold only 2 "vertices" for these shapes, so the
    // vertex-walk below cannot test them.
    if desc.poly.sides >= 1 && desc.poly.sides <= 2 {
        let rx = (lw + rw) / 2.0;
        let ry = ht / 2.0;
        if rx <= 0.0 || ry <= 0.0 {
            return false;
        }
        let (nx, ny) = (p.x / rx, p.y / ry);
        return nx * nx + ny * ny <= 1.0;
    }
    let star = desc.fns == ShapeFns::Star;
    let cached = info
        .vertices
        .as_deref()
        .filter(|v| info.sides > 0 && v.len() % info.sides as usize == 0);
    if let Some(verts) = cached {
        let sides = info.sides as usize;
        let ring_count = verts.len() / sides;
        vertex_inside(sides, ring_count, verts, star, p)
    } else {
        let (sides, flat) = build_rings_flat(desc, lw + rw, ht, DEFAULT_NODEPENWIDTH);
        if sides == 0 || flat.is_empty() {
            // last resort: the node box
            return p.x >= -lw && p.x <= rw && p.y >= -ht / 2.0 && p.y <= ht / 2.0;
        }
        let ring_count = flat.len() / sides;
        vertex_inside(sides, ring_count, &flat, star, p)
    }
}

/// `record_inside` (shapes.c:3762-3787): `INSIDE(p, bbox)` with the bbox
/// (port box or root field box) expanded by `penwidth/2` on all sides.
pub fn record_inside_test(
    record: &RecordLayout,
    bp: Option<BoxF>,
    p: PointF,
    penwidth: f64,
) -> bool {
    let mut bbox = bp.unwrap_or(record.root.b());
    let ext = PointF::new(penwidth / 2.0, penwidth / 2.0);
    bbox.ll = PointF::new(bbox.ll.x - ext.x, bbox.ll.y - ext.y);
    bbox.ur = PointF::new(bbox.ur.x + ext.x, bbox.ur.y + ext.y);
    bbox.contains(p)
}

// ===========================================================================
// Records — record/Mrecord (shapes.c:3321-3756)
// ===========================================================================

// parse_reclbl state bits (shapes.c:3323-3327)
const HASTEXT: u32 = 1;
const HASPORT: u32 = 2;
const HASTABLE: u32 = 4;
const INTEXT: u32 = 8;
const INPORT: u32 = 16;

/// `ISCTRL` (shapes.c:3329-3331).
fn isctrl(c: u8) -> bool {
    matches!(c, b'{' | b'}' | b'|' | b'<' | b'>')
}

/// One record field — `field_t` (types.h:235-244) with the rect as
/// `x, y, w, h` (node frame, y up, origin = node center): `b.LL = (x,
/// y-h)`, `b.UR = (x+w, y)`, mirroring `pos_reclbl` (shapes.c:3594-3595).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RecordField {
    /// Upper-left corner x (node frame).
    pub x: f64,
    /// Upper-left corner y (node frame).
    pub y: f64,
    /// Width (points) — `field_t.size.x`.
    pub w: f64,
    /// Height (points) — `field_t.size.y`.
    pub h: f64,
    /// Exposed-sides bitmask (`field_t.sides`).
    pub sides: u8,
    /// Port identifier from `<...>`.
    pub id: Option<String>,
    /// Children laid left-to-right? (`field_t.LR`).
    pub lr: bool,
    /// Leaf text (escapes processed; None for containers).
    pub text: Option<String>,
    /// Measured dimen of `text` (points), before padding.
    pub text_dimen: PointF,
    /// Label space after resizing (`lp->space`).
    pub text_space: PointF,
    pub children: Vec<RecordField>,
}

impl RecordField {
    /// `field_t.b` — the field rect as a `boxf`.
    pub fn b(&self) -> BoxF {
        BoxF::new(
            PointF::new(self.x, self.y - self.h),
            PointF::new(self.x + self.w, self.y),
        )
    }

    /// Rect center (node frame).
    pub fn center(&self) -> PointF {
        PointF::new(self.x + self.w / 2.0, self.y - self.h / 2.0)
    }

    /// `field_t.size`.
    pub fn size(&self) -> PointF {
        PointF::new(self.w, self.h)
    }

    /// Depth-first search for a field with `id == str`
    /// (`map_rec_port`, shapes.c:3716-3730).
    pub fn find_port(&self, s: &str) -> Option<&RecordField> {
        if self.id.as_deref() == Some(s) {
            return Some(self);
        }
        for child in &self.children {
            if let Some(f) = child.find_port(s) {
                return Some(f);
            }
        }
        None
    }
}

/// The parsed + laid-out record — `ND_shape_info` for `shape=record`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RecordLayout {
    /// Root field (its box is the node box minus the +1 pt height kluge).
    pub root: RecordField,
    /// `info->size` after `resize_reclbl` (points, pre-kluge).
    pub size: PointF,
}

/// Node size for a record: `(info.size.x, info.size.y + 1)` in points —
/// the "+1 to fix rounding diff between layout and rendering" kluge at
/// shapes.c:3703-3705.
pub fn record_node_size(record: &RecordLayout) -> (f64, f64) {
    (record.size.x, record.size.y + 1.0)
}

/// Shared mutable parse state for `parse_reclbl`.
struct RecState {
    mode: u32,
    tmpport: Option<String>,
    text: Vec<u8>, // tsp buffer
    pbuf: Vec<u8>, // psp buffer
    hstsp: usize,
    hspsp: usize,
    ishardspace: bool,
}

/// `parse_reclbl` (shapes.c:3358-3500). `lr` = lay this field's children
/// out left-to-right; `flag` = top-level call (end-of-string allowed).
/// `measure` sizes each leaf's label (`make_label` + `textspan_size`).
fn parse_reclbl(
    c: &mut RecCursor,
    lr: bool,
    flag: bool,
    measure: &dyn Fn(&str) -> PointF,
) -> Option<RecordField> {
    let mut rv = RecordField {
        lr,
        ..RecordField::default()
    };
    let mut st = RecState {
        mode: 0,
        tmpport: None,
        text: Vec::new(),
        pbuf: Vec::new(),
        hstsp: 0,
        hspsp: 0,
        ishardspace: false,
    };
    let mut wflag = true;
    while wflag {
        let uc = c.cur();
        if uc != 0 && uc < b' ' {
            // ignore non-0 control characters (shapes.c:3391-3394)
            c.pos += 1;
            continue;
        }
        match uc {
            b'<' if !c.html => {
                if st.mode & (HASTABLE | HASPORT) != 0 {
                    return None;
                }
                st.mode |= HASPORT | INPORT;
                c.pos += 1;
                st.pbuf.clear();
                st.hspsp = 0;
            }
            b'>' if !c.html => {
                if st.mode & INPORT == 0 {
                    return None;
                }
                // trim one trailing space (shapes.c:3410-3411)
                if st.pbuf.len() > 1
                    && st.pbuf.len() - 1 != st.hspsp
                    && st.pbuf[st.pbuf.len() - 1] == b' '
                {
                    st.pbuf.pop();
                }
                st.tmpport = Some(String::from_utf8_lossy(&st.pbuf).into_owned());
                st.mode &= !INPORT;
                c.pos += 1;
            }
            b'{' => {
                c.pos += 1;
                if st.mode != 0 || c.cur() == 0 {
                    return None;
                }
                st.mode = HASTABLE;
                let child = parse_reclbl(c, !lr, false, measure)?;
                rv.children.push(child);
            }
            b'}' | b'|' | 0 => {
                if (uc == 0 && !flag) || st.mode & INPORT != 0 {
                    return None;
                }
                let mut fp: Option<usize> = None;
                if st.mode & HASTABLE == 0 {
                    fp = Some(rv.children.len());
                    rv.children.push(RecordField {
                        lr: false,
                        ..RecordField::default()
                    });
                }
                if let Some(id) = st.tmpport.take()
                    && let Some(idx) = fp {
                        rv.children[idx].id = Some(id);
                    }
                if st.mode & (HASTEXT | HASTABLE) == 0 {
                    // empty field ⇒ " " (shapes.c:3436-3439)
                    st.mode |= HASTEXT;
                    st.text.push(b' ');
                }
                if st.mode & HASTEXT != 0 {
                    // trim one trailing space (shapes.c:3441-3442)
                    if st.text.len() > 1
                        && st.text.len() - 1 != st.hstsp
                        && st.text[st.text.len() - 1] == b' '
                    {
                        st.text.pop();
                    }
                    let s = String::from_utf8_lossy(&st.text).into_owned();
                    let dimen = measure(&s);
                    let idx = fp.expect("HASTEXT implies a leaf field");
                    let fld = &mut rv.children[idx];
                    fld.text = Some(s);
                    fld.text_dimen = dimen;
                    // make_simple_label: lp.space starts at lp.dimen
                    fld.text_space = dimen;
                    fld.lr = true;
                    st.text.clear();
                    st.hstsp = 0;
                }
                if uc != 0 {
                    if uc == b'}' {
                        c.pos += 1;
                        return Some(rv); // end of nested table
                    }
                    st.mode = 0;
                    c.pos += 1;
                } else {
                    wflag = false;
                }
            }
            b'\\' => {
                if c.peek(1) != 0 {
                    let next = c.peek(1);
                    if isctrl(next) {
                        // nothing — the escaped control char is copied by dotext
                    } else if next == b' ' && !c.html {
                        st.ishardspace = true;
                    } else {
                        st.text.push(b'\\');
                        st.mode |= INTEXT | HASTEXT;
                    }
                    c.pos += 1;
                }
                // fall through to dotext (shapes.c:3472-3473)
                dotext(&mut st, c)?;
            }
            _ => {
                dotext(&mut st, c)?;
            }
        }
    }
    Some(rv)
}

/// The `dotext` body (shapes.c:3474-3495). Returns `None` on parse error.
fn dotext(st: &mut RecState, c: &mut RecCursor) -> Option<()> {
    let ch = c.cur();
    if st.mode & HASTABLE != 0 && ch != b' ' {
        return None;
    }
    if st.mode & (INTEXT | INPORT) == 0 && ch != b' ' {
        st.mode |= INTEXT | HASTEXT;
    }
    if st.mode & INTEXT != 0 {
        let last_is_space = st.text.last() == Some(&b' ');
        if !(ch == b' ' && !st.ishardspace && last_is_space && !c.html) {
            st.text.push(ch);
        }
        if st.ishardspace {
            st.hstsp = st.text.len().saturating_sub(1);
        }
    } else if st.mode & INPORT != 0 {
        let prev_is_space = st.pbuf.last() == Some(&b' ');
        if !(ch == b' ' && !st.ishardspace && (st.pbuf.is_empty() || prev_is_space)) {
            st.pbuf.push(ch);
        }
        if st.ishardspace {
            st.hspsp = st.pbuf.len().saturating_sub(1);
        }
    }
    c.pos += 1;
    // copy UTF-8 continuation bytes (shapes.c:3493-3494) — always into tsp
    while c.pos < c.src.len() && (c.src[c.pos] & 0xc0) == 0x80 {
        st.text.push(c.src[c.pos]);
        c.pos += 1;
    }
    Some(())
}

/// Byte cursor standing in for the C `reclblp` global.
struct RecCursor<'a> {
    src: &'a [u8],
    pos: usize,
    html: bool,
}

impl<'a> RecCursor<'a> {
    fn cur(&self) -> u8 {
        if self.pos < self.src.len() {
            self.src[self.pos]
        } else {
            0
        }
    }

    fn peek(&self, k: usize) -> u8 {
        if self.pos + k < self.src.len() {
            self.src[self.pos + k]
        } else {
            0
        }
    }
}

/// `size_reclbl` (shapes.c:3502-3545) — leaf dimen + margin (note: the
/// record margin parse does **not** clamp negatives, unlike `poly_init`).
fn size_reclbl(f: &mut RecordField, attrs: &BTreeMap<String, String>) {
    if f.text.is_some() {
        let mut dimen = f.text_dimen;
        if dimen.x > 0.0 || dimen.y > 0.0 {
            match attr_str(attrs, "margin") {
                Some(m) => {
                    let (i, marginx, marginy) = sscanf_2lf(m);
                    if i > 0 {
                        dimen.x += 2.0 * inch2ps(marginx);
                        if i > 1 {
                            dimen.y += 2.0 * inch2ps(marginy);
                        } else {
                            dimen.y += 2.0 * inch2ps(marginx);
                        }
                    } else {
                        pad(&mut dimen);
                    }
                }
                None => pad(&mut dimen),
            }
        }
        f.w = dimen.x;
        f.h = dimen.y;
    } else {
        let mut d = PointF::ZERO;
        for child in f.children.iter_mut() {
            size_reclbl(child, attrs);
            let d0 = child.size();
            if f.lr {
                d.x += d0.x;
                d.y = d.y.max(d0.y);
            } else {
                d.y += d0.y;
                d.x = d.x.max(d0.x);
            }
        }
        f.w = d.x;
        f.h = d.y;
    }
}

/// `resize_reclbl` (shapes.c:3547-3582) — with the `(int)` truncation
/// distribution of the extra space.
fn resize_reclbl(f: &mut RecordField, sz: PointF, nojustify: bool) {
    let d = PointF::new(sz.x - f.w, sz.y - f.h);
    f.w = sz.x;
    f.h = sz.y;
    if f.text.is_some() && !nojustify {
        f.text_space.x += d.x;
        f.text_space.y += d.y;
    }
    let n = f.children.len();
    if n > 0 {
        let inc = if f.lr { d.x / n as f64 } else { d.y / n as f64 };
        for i in 0..n {
            let amt = (((i + 1) as f64 * inc) as i32 - (i as f64 * inc) as i32) as f64;
            let newsz = if f.lr {
                PointF::new(f.children[i].w + amt, sz.y)
            } else {
                PointF::new(sz.x, f.children[i].h + amt)
            };
            resize_reclbl(&mut f.children[i], newsz, nojustify);
        }
    }
}

/// `pos_reclbl` (shapes.c:3589-3628) — place fields and assign side masks.
fn pos_reclbl(f: &mut RecordField, ul: PointF, sides: u8) {
    f.sides = sides;
    f.x = ul.x;
    f.y = ul.y;
    let last = f.children.len() as i32 - 1;
    let mut ul = ul;
    for i in 0..f.children.len() {
        let mask: u8 = if sides != 0 {
            if f.lr {
                if i == 0 {
                    if i as i32 == last {
                        ALL_SIDES
                    } else {
                        TOP | BOTTOM | LEFT
                    }
                } else if i as i32 == last {
                    TOP | BOTTOM | RIGHT
                } else {
                    TOP | BOTTOM
                }
            } else if i == 0 {
                if i as i32 == last {
                    ALL_SIDES
                } else {
                    TOP | RIGHT | LEFT
                }
            } else if i as i32 == last {
                LEFT | BOTTOM | RIGHT
            } else {
                LEFT | RIGHT
            }
        } else {
            0
        };
        let child_size = f.children[i].size();
        pos_reclbl(&mut f.children[i], ul, sides & mask);
        if f.lr {
            ul.x += child_size.x;
        } else {
            ul.y -= child_size.y;
        }
    }
}

/// `record_init` (shapes.c:3663-3707) as a pure function: parse the record
/// label, size it, distribute extra space and place the field rects.
///
/// * `text` — the raw label text (`ND_label->text`).
/// * `attrs` — node attributes (`margin`/`fixed`/`nojustify`/`width`/
///   `height`).
/// * `flip` — `!GD_realflip(g)` (records lay out horizontally unless the
///   rankdir flips axes).
/// * `measure` — text extent function `(text) -> (w, h)` in points for
///   each field's label (should apply `\N` etc. substitution and
///   `textspan_size` semantics).
///
/// Parse errors fall back to the `"\\N"` label exactly like
/// `record_init` (shapes.c:3681-3685).
pub fn record_layout(
    text: &str,
    attrs: &BTreeMap<String, String>,
    flip: bool,
    measure: &dyn Fn(&str) -> PointF,
) -> RecordLayout {
    let parse = |src: &str| {
        let mut c = RecCursor {
            src: src.as_bytes(),
            pos: 0,
            html: false,
        };
        parse_reclbl(&mut c, flip, true, measure)
    };
    let mut root = parse(text).or_else(|| parse("\\N")).unwrap_or_else(|| {
        // unreachable for "\\N", kept for totality
        RecordField {
            lr: flip,
            text: Some(String::new()),
            ..RecordField::default()
        }
    });

    size_reclbl(&mut root, attrs);
    let mut sz = PointF::new(inch2ps(eff_width(attrs)), inch2ps(eff_height(attrs)));
    let fixed = mapbool(attr_str(attrs, "fixed").or(Some("false")));
    if !fixed {
        sz.x = sz.x.max(root.w);
        sz.y = sz.y.max(root.h);
    }
    let nojustify = mapbool(attr_str(attrs, "nojustify").or(Some("false")));
    resize_reclbl(&mut root, sz, nojustify);
    let ul = PointF::new(-sz.x / 2.0, sz.y / 2.0); // shapes.c:3701
    pos_reclbl(&mut root, ul, ALL_SIDES);
    RecordLayout { root, size: sz }
}

// ===========================================================================
// Ports — compassPort (shapes.c:2548-2915), record_port (3732-3756)
// ===========================================================================

/// The resolved `port` (types.h:40-57 plus `dyna`/`name`). Convert with
/// [`ResolvedPort::into_model_port`] / `From<ResolvedPort> for Port`.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPort {
    /// Aiming point relative to the node center, **layout frame** (the C
    /// applies `cwrotatepf(p, 90*rankdir)` at shapes.c:2856).
    pub p: PointF,
    /// Port rectangle (record field / html cell), node frame.
    pub bp: Option<BoxF>,
    /// Slope constraint in radians; meaningful iff `constrained`.
    pub theta: f64,
    pub defined: bool,
    pub constrained: bool,
    pub clip: bool,
    /// Compass `"_"` — choose side dynamically.
    pub dyna: bool,
    /// `MC_SCALE`-ordered mincross sort key (0..=256).
    pub order: u8,
    /// Side bitmask (BOTTOM/RIGHT/TOP/LEFT).
    pub side: u8,
    /// `chkPort` name: the post-colon compass text, or the whole port
    /// string when no colon was present (utils.c:478-495). `None` for
    /// poly shapes (`poly_port` NULLs it, shapes.c:2913).
    pub name: Option<String>,
    /// True when the compass/port string was not recognized — C returns 1
    /// and keeps the center fallback (shapes.c:2678-2679).
    pub unrecognized: bool,
}

impl Default for ResolvedPort {
    /// `static port Center = {.theta = -1, .clip = true}` (shapes.c:38).
    fn default() -> Self {
        ResolvedPort {
            p: PointF::ZERO,
            bp: None,
            theta: -1.0,
            defined: false,
            constrained: false,
            clip: true,
            dyna: false,
            order: 0,
            side: 0,
            name: None,
            unrecognized: false,
        }
    }
}

impl ResolvedPort {
    fn center() -> Self {
        Self::default()
    }

    /// Fill the `dotgen::model::Port` subset (the model has no `dyna` or
    /// `name` fields).
    pub fn into_model_port(self) -> Port {
        Port {
            p: self.p,
            bp: self.bp.unwrap_or_default(),
            defined: self.defined,
            clip: self.clip,
            order: self.order,
            theta: self.theta,
            side: self.side,
            constrained: self.constrained,
            dyna: self.dyna,
        }
    }
}

/// `closestSide` (shapes.c:4221-4316) — the compass point of `n` whose port
/// rectangle side lies closest to `other`, among the sides `side` allows.
/// Returns `None` for "use the center" (no side, or every side allowed).
#[allow(clippy::too_many_arguments)]
fn closest_side(
    lw: f64,
    rw: f64,
    ht: f64,
    bp: Option<BoxF>,
    side: u8,
    flip: bool,
    n_coord: PointF,
    other_coord: PointF,
) -> Option<&'static str> {
    const SIDE_PORT: [&str; 4] = ["s", "e", "n", "w"];
    let all = BOTTOM | RIGHT | TOP | LEFT;
    if side == 0 || side == all {
        return None; // use center
    }
    let b = match bp {
        Some(b) => b,
        None => {
            let (bx, by) = if flip { (ht / 2.0, lw) } else { (lw, ht / 2.0) };
            BoxF::new(PointF::new(-bx, -by), PointF::new(bx, by))
        }
    };
    let cvt = |p: PointF| match flip {
        // cvtPt (shapes.c:4223-4243) — `flip` is GD_flip (LR/RL/BT).
        true => PointF::new(-p.y, p.x), // RANKDIR_LR
        false => p,                     // RANKDIR_TB
    };
    let pt = cvt(n_coord);
    let opt = cvt(other_coord);
    let mut best: Option<&'static str> = None;
    let mut mind = 0.0f64;
    for i in 0..4usize {
        if side & (1 << i) == 0 {
            continue;
        }
        let mut p = match i {
            0 => PointF::new((b.ll.x + b.ur.x) / 2.0, b.ll.y), // BOTTOM_IX
            1 => PointF::new(b.ur.x, (b.ll.y + b.ur.y) / 2.0), // RIGHT_IX
            2 => PointF::new((b.ll.x + b.ur.x) / 2.0, b.ur.y), // TOP_IX
            _ => PointF::new(b.ll.x, (b.ll.y + b.ur.y) / 2.0), // LEFT_IX
        };
        p = p.add(pt);
        let d = (p.x - opt.x).powi(2) + (p.y - opt.y).powi(2);
        if best.is_none() || d < mind {
            mind = d;
            best = Some(SIDE_PORT[i]);
        }
    }
    let _ = rw;
    best
}

/// `resolvePort` (shapes.c:4322-4333) — re-resolve a `dyna` (`_`) port now
/// that the other endpoint's position is known.
#[allow(clippy::too_many_arguments)]
pub fn resolve_port(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: Option<&ShapeInfo>,
    old: &Port,
    flip: bool,
    n_coord: PointF,
    other_coord: PointF,
    rd: RankDir,
) -> Port {
    let compass = closest_side(
        lw,
        rw,
        ht,
        if old.bp == BoxF::default() {
            None
        } else {
            Some(old.bp)
        },
        old.side,
        flip,
        n_coord,
        other_coord,
    );
    // compassPort(n, oldport->bp, &rv, compass, oldport->side, NULL); a NULL
    // compass leaves the port at the rectangle centre.
    let mut rp = match compass {
        Some(c) => compass_port_core(
            desc,
            lw,
            rw,
            ht,
            info,
            if old.bp == BoxF::default() {
                None
            } else {
                Some(old.bp)
            },
            c,
            old.side,
            rd,
            DEFAULT_NODEPENWIDTH,
        ),
        None => {
            let mut rp = ResolvedPort::default();
            if old.bp != BoxF::default() {
                rp.bp = Some(old.bp);
                rp.p = PointF::new(
                    (old.bp.ll.x + old.bp.ur.x) / 2.0,
                    (old.bp.ll.y + old.bp.ur.y) / 2.0,
                );
                rp.defined = true;
            }
            rp
        }
    };
    rp.dyna = false;
    rp.into_model_port()
}

impl From<ResolvedPort> for Port {
    fn from(rp: ResolvedPort) -> Port {
        rp.into_model_port()
    }
}

/// `invflip_side` (shapes.c:2548-2604).
pub fn invflip_side(side: u8, rankdir: RankDir) -> u8 {
    match rankdir {
        RankDir::Tb => side,
        RankDir::Bt => match side {
            TOP => BOTTOM,
            BOTTOM => TOP,
            other => other,
        },
        RankDir::Lr => match side {
            TOP => RIGHT,
            BOTTOM => LEFT,
            LEFT => TOP,
            RIGHT => BOTTOM,
            other => other,
        },
        RankDir::Rl => match side {
            TOP => RIGHT,
            BOTTOM => LEFT,
            LEFT => BOTTOM,
            RIGHT => TOP,
            other => other,
        },
    }
}

/// `invflip_angle` (shapes.c:2606-2635).
pub fn invflip_angle(angle: f64, rankdir: RankDir) -> f64 {
    match rankdir {
        RankDir::Tb => angle,
        RankDir::Bt => -angle,
        RankDir::Lr => angle - PI * 0.5,
        RankDir::Rl => {
            if angle == PI {
                -0.5 * PI
            } else if angle == PI * 0.75 {
                -0.25 * PI
            } else if angle == PI * 0.5 {
                0.0
            } else if angle == 0.0 {
                PI * 0.5
            } else if angle == PI * -0.25 {
                PI * 0.75
            } else if angle == PI * -0.5 {
                PI
            } else {
                angle
            }
        }
    }
}

/// `cwrotatepf` (geom.c:130-147) — exact per-angle transforms.
pub fn cwrotatepf(p: PointF, cwrot: i32) -> PointF {
    match cwrot {
        90 => PointF::new(p.y, -p.x),
        180 => PointF::new(p.x, -p.y),
        270 => PointF::new(p.y, p.x), // exch_xyf
        _ => p,
    }
}

/// `ccwrotatepf` (geom.c:149-163); 90° = `perp` = (−y, x)
/// (geomprocs.h:140-145).
pub fn ccwrotatepf(p: PointF, ccwrot: i32) -> PointF {
    match ccwrot {
        90 => PointF::new(-p.y, p.x),
        180 => PointF::new(p.x, -p.y),
        270 => PointF::new(p.y, p.x), // exch_xyf
        _ => p,
    }
}

/// One painted record field: its box (layout frame, node-relative) and text.
#[derive(Debug, Clone)]
pub struct RecordPart {
    pub ll: PointF,
    pub ur: PointF,
    pub text: String,
}

/// The painted form of a record: leaf boxes with their text and the internal
/// divider segments (`record_gencode`, shapes.c:3800-3830).
///
/// Everything is in the **record frame**, which is also the frame the node is
/// painted in: `record_init` sizes the record *before* `gv_nodesize` swaps the
/// axes for a rotated (LR/BT) layout, and `translate_drawing` swaps them back
/// for rendering (`gv_nodesize(v, false)`), so painted box and record fields
/// share one frame. Only the splines (and hence the `record_inside` query
/// point) live in the rotated layout frame.
pub fn record_parts(record: &RecordLayout) -> (Vec<RecordPart>, Vec<(PointF, PointF)>) {
    let to_layout = |p: PointF| p;
    let mut fields = Vec::new();
    let mut dividers = Vec::new();

    // Dividers between siblings: the shared edge of consecutive children,
    // spanning the parent's extent across it.
    fn walk(
        parent: &RecordField,
        fields: &mut Vec<RecordPart>,
        dividers: &mut Vec<(PointF, PointF)>,
        to_layout: &dyn Fn(PointF) -> PointF,
    ) {
        if parent.children.is_empty() {
            let b = parent.b();
            fields.push(RecordPart {
                ll: to_layout(b.ll),
                ur: to_layout(b.ur),
                text: parent.text.clone().unwrap_or_default(),
            });
            return;
        }
        for (i, child) in parent.children.iter().enumerate() {
            if i > 0 {
                let prev = &parent.children[i - 1];
                let pb = prev.b();
                let cb = child.b();
                if parent.lr {
                    // children run left to right: a vertical divider at the
                    // shared x, spanning the parent's height
                    let x = cb.ll.x.max(pb.ur.x);
                    dividers.push((
                        to_layout(PointF::new(x, parent.b().ll.y)),
                        to_layout(PointF::new(x, parent.b().ur.y)),
                    ));
                } else {
                    // children stack: a horizontal divider at the shared y
                    let y = cb.ur.y.min(pb.ll.y);
                    dividers.push((
                        to_layout(PointF::new(parent.b().ll.x, y)),
                        to_layout(PointF::new(parent.b().ur.x, y)),
                    ));
                }
            }
            walk(child, fields, dividers, to_layout);
        }
    }
    walk(&record.root, &mut fields, &mut dividers, &to_layout);
    (fields, dividers)
}

/// Inside test used by the compass ray search — the node's `insidefn`
/// with the outline ring. Prefers cached `info.vertices`; otherwise
/// rebuilds the rings from the node box.
fn ray_inside(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: Option<&ShapeInfo>,
    penwidth: f64,
    p: PointF,
) -> bool {
    if let Some(info) = info
        && let Some(verts) = info
            .vertices
            .as_deref()
            .filter(|v| info.sides > 0 && v.len() % info.sides as usize == 0)
        {
            let sides = info.sides as usize;
            let ring_count = verts.len() / sides;
            return vertex_inside(sides, ring_count, verts, desc.fns == ShapeFns::Star, p);
        }
    if desc.fns == ShapeFns::Record {
        // records use compassPort with bp set — no ray search happens.
        let ext = penwidth / 2.0;
        return p.x >= -lw - ext
            && p.x <= rw + ext
            && p.y >= -ht / 2.0 - ext
            && p.y <= ht / 2.0 + ext;
    }
    let (sides, flat) = build_rings_flat(desc, lw + rw, ht, penwidth);
    if sides == 0 || flat.is_empty() {
        return p.x.abs() <= (lw + rw) / 2.0 && p.y.abs() <= ht / 2.0;
    }
    let ring_count = flat.len() / sides;
    vertex_inside(sides, ring_count, &flat, desc.fns == ShapeFns::Star, p)
}

/// `compassPoint` (shapes.c:2637-2672) — ray-cast from the node center to
/// the shape boundary along `(x, y)` (node frame), with `bezier_clip`'s
/// binary search (splines.c:107-151) on the degenerate straight curve
/// `[(0,0),(0,0),p,p]`, whose Bézier is `p * t²(3-2t)`.
#[allow(clippy::too_many_arguments)]
fn compass_point(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: Option<&ShapeInfo>,
    penwidth: f64,
    y: f64,
    x: f64,
) -> PointF {
    // C rotates the ray into the layout frame and lets the inside fn
    // rotate back; that rigid round-trip is a no-op for the straight ray,
    // so we search the node-frame ray directly. The result is converted
    // to the layout frame by the caller like curve[0] is (ccwrotate).
    let dir = PointF::new(x, y);
    let mut low = 0.0f64;
    let mut high = 1.0f64;
    let mut found = false;
    let mut pt = PointF::ZERO; // sp[0]
    let mut best = pt;
    loop {
        let opt = pt;
        let t = (high + low) / 2.0;
        let b = t * t * (3.0 - 2.0 * t);
        pt = PointF::new(dir.x * b, dir.y * b);
        if ray_inside(desc, lw, rw, ht, info, penwidth, pt) {
            low = t;
            best = pt;
            found = true;
        } else {
            high = t;
        }
        if (opt.x - pt.x).abs() <= 0.5 && (opt.y - pt.y).abs() <= 0.5 {
            break;
        }
    }
    if found { best } else { pt }
}

/// `compassPort` (shapes.c:2698-2878).
#[allow(clippy::too_many_arguments)]
fn compass_port_core(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: Option<&ShapeInfo>,
    bp: Option<BoxF>,
    compass: &str,
    sides: u8,
    rd: RankDir,
    penwidth: f64,
) -> ResolvedPort {
    let flip = rd.flip(); // GD_flip: LR | BT
    let (b, mut p, mut defined) = match bp {
        Some(b) => (
            b,
            PointF::new((b.ll.x + b.ur.x) / 2.0, (b.ll.y + b.ur.y) / 2.0),
            true,
        ),
        None => {
            let b = if flip {
                BoxF::new(PointF::new(-ht / 2.0, -lw), PointF::new(ht / 2.0, lw))
            } else {
                BoxF::new(PointF::new(-lw, -ht / 2.0), PointF::new(lw, ht / 2.0))
            };
            (b, PointF::ZERO, false)
        }
    };
    let maxv = b.ur.x.max(b.ur.y) * 4.0;
    let ctr = p;
    let mut theta = 0.0f64;
    let mut constrain = false;
    let mut dyna = false;
    let mut side = 0u8;
    let mut clip = true;
    let mut unrecognized = false;
    let use_ictxt = bp.is_none() && !is_box(desc); // shapes.c:2902-2908

    let mut chars = compass.chars();
    if let Some(first) = chars.next() {
        let rest = chars.as_str();
        match first {
            'e' => {
                if !rest.is_empty() {
                    unrecognized = true;
                } else {
                    if use_ictxt {
                        p = compass_point(desc, lw, rw, ht, info, penwidth, ctr.y, maxv);
                    } else {
                        p.x = b.ur.x;
                    }
                    theta = 0.0;
                    constrain = true;
                    defined = true;
                    clip = false;
                    side = sides & RIGHT;
                }
            }
            'w' => {
                if !rest.is_empty() {
                    unrecognized = true;
                } else {
                    if use_ictxt {
                        p = compass_point(desc, lw, rw, ht, info, penwidth, ctr.y, -maxv);
                    } else {
                        p.x = b.ll.x;
                    }
                    theta = PI;
                    constrain = true;
                    defined = true;
                    clip = false;
                    side = sides & LEFT;
                }
            }
            's' => {
                p.y = b.ll.y;
                constrain = true;
                clip = false;
                match rest {
                    "" => {
                        theta = -PI * 0.5;
                        defined = true;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, -maxv, ctr.x);
                        } else {
                            p.x = ctr.x;
                        }
                        side = sides & BOTTOM;
                    }
                    "e" => {
                        theta = -PI * 0.25;
                        defined = true;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, -maxv, maxv);
                        } else {
                            p.x = b.ur.x;
                        }
                        side = sides & (BOTTOM | RIGHT);
                    }
                    "w" => {
                        theta = -PI * 0.75;
                        defined = true;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, -maxv, -maxv);
                        } else {
                            p.x = b.ll.x;
                        }
                        side = sides & (BOTTOM | LEFT);
                    }
                    _ => {
                        p.y = ctr.y;
                        constrain = false;
                        clip = true;
                        unrecognized = true;
                    }
                }
            }
            'n' => {
                p.y = b.ur.y;
                constrain = true;
                clip = false;
                match rest {
                    "" => {
                        defined = true;
                        theta = PI * 0.5;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, maxv, ctr.x);
                        } else {
                            p.x = ctr.x;
                        }
                        side = sides & TOP;
                    }
                    "e" => {
                        defined = true;
                        theta = PI * 0.25;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, maxv, maxv);
                        } else {
                            p.x = b.ur.x;
                        }
                        side = sides & (TOP | RIGHT);
                    }
                    "w" => {
                        defined = true;
                        theta = PI * 0.75;
                        if use_ictxt {
                            p = compass_point(desc, lw, rw, ht, info, penwidth, maxv, -maxv);
                        } else {
                            p.x = b.ll.x;
                        }
                        side = sides & (TOP | LEFT);
                    }
                    _ => {
                        p.y = ctr.y;
                        constrain = false;
                        clip = true;
                        unrecognized = true;
                    }
                }
            }
            '_' => {
                dyna = true;
                side = sides;
            }
            'c' => {}
            _ => unrecognized = true,
        }
    }

    p = cwrotatepf(
        p,
        90 * match rd {
            RankDir::Tb => 0,
            RankDir::Lr => 1,
            RankDir::Bt => 2,
            RankDir::Rl => 3,
        },
    );
    let final_side = if dyna { side } else { invflip_side(side, rd) };
    let order = if p.x == 0.0 && p.y == 0.0 {
        (MC_SCALE / 2.0) as u8 // shapes.c:2864-2865 — center = 128
    } else {
        // angle with 0 at the north pole, increasing CCW (shapes.c:2867-2871)
        let mut angle = p.y.atan2(p.x) + 1.5 * PI;
        if angle >= 2.0 * PI {
            angle -= 2.0 * PI;
        }
        let o = (MC_SCALE * angle / (2.0 * PI)) as i32;
        o.clamp(0, 255) as u8
    };
    ResolvedPort {
        p,
        bp,
        theta: invflip_angle(theta, rd),
        defined,
        constrained: constrain,
        clip,
        dyna,
        order,
        side: final_side,
        name: None,
        unrecognized,
    }
}

/// `compass_port` — the public port resolver combining `chkPort`
/// (utils.c:478-495), `poly_port` (shapes.c:2880-2915) and `record_port`
/// (shapes.c:3732-3756).
///
/// * `port` — the raw port attribute text; split at the **first** `':'`
///   into `(name, compass)`. A leading colon (`":e"`) makes the name
///   empty → the C `Center` port.
/// * `flip` — `GD_flip(g)`; mapped to `RankDir::Lr` internally. For
///   BT/RL exactness use [`compass_port_rankdir`].
/// * `lw`, `rw`, `ht` — the stored node half-widths/height (points).
/// * `record` — the parsed record layout for record shapes.
///
/// Note the C quirk kept here: for non-record nodes the *name* part is
/// interpreted as the compass string and the post-colon text is ignored
/// (shapes.c:2909 passes `portname`, not `compass`).
pub fn compass_port(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    record: Option<&RecordLayout>,
    port: &str,
    flip: bool,
) -> ResolvedPort {
    compass_port_rankdir(
        desc,
        lw,
        rw,
        ht,
        None,
        record,
        port,
        if flip { RankDir::Lr } else { RankDir::Tb },
    )
}

/// [`compass_port`] with full rankdir control and optional cached shape
/// info (used by the compass ray search for non-box shapes).
#[allow(clippy::too_many_arguments)] // mirrors compass_port's parameters
pub fn compass_port_rankdir(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    info: Option<&ShapeInfo>,
    record: Option<&RecordLayout>,
    port: &str,
    rd: RankDir,
) -> ResolvedPort {
    // chkPort (utils.c:478-495): split at the first ':'
    let (name, compass_part) = match port.find(':') {
        Some(i) => (&port[..i], Some(&port[i + 1..])),
        None => (port, None),
    };
    if name.is_empty() {
        // poly_port/record_port: portname[0] == '\0' → Center
        let mut rp = ResolvedPort::center();
        rp.name = Some(
            compass_part
                .map(str::to_string)
                .unwrap_or_else(|| port.to_string()),
        );
        return rp;
    }
    let penwidth = DEFAULT_NODEPENWIDTH;
    if desc.fns == ShapeFns::Record {
        if let Some(field) = record.and_then(|r| r.root.find_port(name)) {
            // compassPort(n, &subf->b, &rv, compass, subf->sides, NULL)
            let mut rp = compass_port_core(
                desc,
                lw,
                rw,
                ht,
                info,
                Some(field.b()),
                compass_part.unwrap_or("_"),
                field.sides,
                rd,
                penwidth,
            );
            rp.name = Some(
                compass_part
                    .map(str::to_string)
                    .unwrap_or_else(|| name.to_string()),
            );
            rp
        } else {
            // compassPort(n, &f->b, &rv, portname, sides, NULL)
            let mut rp = compass_port_core(
                desc,
                lw,
                rw,
                ht,
                info,
                record.map(|r| r.root.b()),
                name,
                ALL_SIDES,
                rd,
                penwidth,
            );
            rp.name = Some(
                compass_part
                    .map(str::to_string)
                    .unwrap_or_else(|| name.to_string()),
            );
            rp
        }
    } else {
        // poly_port (shapes.c:2880-2915): the *portname* is the compass.
        let mut rp = compass_port_core(desc, lw, rw, ht, info, None, name, ALL_SIDES, rd, penwidth);
        rp.name = None; // rv.name = NULL (shapes.c:2913)
        rp
    }
}

// ===========================================================================
// shape_vertices — outline polygons for painting (poly_gencode
// shapes.c:2917-3094, round_corners shapes.c:566-906)
// ===========================================================================

/// `interpolate_pointf` (geomprocs.h:112-119).
fn interpolate(t: f64, p: PointF, q: PointF) -> PointF {
    PointF::new(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y))
}

/// `alloc_interpolation_points` (shapes.c:566-617): `B[3s..3s+3]` per edge
/// (corner, t-entry, 1−t-exit), or `B[4s..4s+4]` when `rounded`, plus the
/// 3 wraparound entries.
fn alloc_interpolation_points(af: &[PointF], shape_code: u8, rounded: bool) -> Vec<PointF> {
    let sides = af.len();
    let mut rbconst = RBCONST;
    for seg in 0..sides {
        let p0 = af[seg];
        let p1 = af[(seg + 1) % sides];
        let d = (p1.x - p0.x).hypot(p1.y - p0.y);
        rbconst = rbconst.min(d / 3.0);
    }
    let mut b: Vec<PointF> = Vec::with_capacity(4 * sides + 4);
    for seg in 0..sides {
        let p0 = af[seg];
        let p1 = af[(seg + 1) % sides];
        let d = (p1.x - p0.x).hypot(p1.y - p0.y);
        let mut t = rbconst / d;
        if shape_code == BOX3D || shape_code == COMPONENT {
            t /= 3.0;
        } else if shape_code == DOGEAR {
            t /= 2.0;
        }
        if !rounded {
            b.push(p0);
        } else {
            b.push(interpolate(RBCURVE * t, p0, p1));
        }
        b.push(interpolate(t, p0, p1));
        b.push(interpolate(1.0 - t, p0, p1));
        if rounded {
            b.push(interpolate(1.0 - RBCURVE * t, p0, p1));
        }
    }
    b.push(b[0]);
    b.push(b[1]);
    b.push(b[2]);
    b
}

/// `rounded_draw` (shapes.c:643-663) point list: `6*sides + 1` points —
/// the outline start followed by 3 control points per cubic
/// (`2*sides` cubics), exactly what `gvrender_beziercurve(job, pts+1)`
/// receives.
pub fn rounded_outline_points(af: &[PointF]) -> Vec<PointF> {
    let sides = af.len();
    let b = alloc_interpolation_points(af, 0, true);
    let mut pts: Vec<PointF> = Vec::with_capacity(6 * sides + 2);
    for seg in 0..sides {
        pts.push(b[4 * seg]);
        pts.push(b[4 * seg + 1]);
        pts.push(b[4 * seg + 1]);
        pts.push(b[4 * seg + 2]);
        pts.push(b[4 * seg + 2]);
        pts.push(b[4 * seg + 3]);
    }
    pts.push(pts[0]);
    pts.push(pts[1]);
    pts[1..].to_vec()
}

/// `diagonals_draw` (shapes.c:624-635) corner chords: per corner, a
/// segment from `B[3s+2]` (1−t on edge s) to `B[3s+4]` (t on edge s+1).
/// Draw the polygon rings from [`shape_vertices`] plus these chords.
pub fn corner_diagonals(af: &[PointF]) -> Vec<[PointF; 2]> {
    let sides = af.len();
    let b = alloc_interpolation_points(af, 0, false);
    (0..sides)
        .map(|seg| [b[3 * seg + 2], b[3 * seg + 4]])
        .collect()
}

/// `Mcircle_hack` (shapes.c:547-564) — the two horizontal chords of the
/// `Mcircle` cross, relative to the node center (node frame): `y = .75`,
/// `x = .6614` (`x² + y² = 1`). Two segments: `(±p.x, p.y)` and
/// `(±p.x, −p.y)` where `p = (ND_rw * x, ND_ht/2 * y)`.
pub fn mcircle_hack_lines(lw: f64, rw: f64, ht: f64) -> [PointF; 4] {
    let y = 0.7500;
    let x = 0.6614;
    let py = y * ht / 2.0;
    let px = rw * x; // assume the node is symmetric
    let _ = lw;
    [
        PointF::new(px, py),
        PointF::new(-px, py),
        PointF::new(px, -py),
        PointF::new(-px, -py),
    ]
}

/// `cylinder_draw` (shapes.c:4199-4219) bottom cap: `AF[0..6]` mirrored
/// about the horizontal line through `AF[0]`.
pub fn cylinder_bottom_cap(af: &[PointF]) -> [PointF; 7] {
    let y0 = af[0].y;
    let y02 = y0 + y0;
    let mut v = [PointF::ZERO; 7];
    for (i, item) in v.iter_mut().enumerate().take(6) {
        *item = PointF::new(af[i].x, y02 - af[i].y);
    }
    v[6] = af[6];
    v
}

/// Periphery ring half-extents for ellipse-family shapes (`sides <= 2`):
/// one `(hx, hy)` pair per ring, innermost first, from the final node box
/// (ring `j` sits `GAP` outside ring `j-1`; shapes.c:2170-2184).
pub fn ellipse_rings(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    penwidth: f64,
) -> Vec<(f64, f64)> {
    let peripheries = desc.poly.peripheries.max(0);
    let mut rings = Vec::new();
    if peripheries < 1 {
        rings.push(((lw + rw) / 2.0, ht / 2.0));
        return rings;
    }
    for j in 0..peripheries {
        let inset = (peripheries - 1 - j) as f64 * GAP;
        rings.push(((lw + rw) / 2.0 - inset, ht / 2.0 - inset));
    }
    if peripheries >= 1 && penwidth > 0.0 {
        rings.push(((lw + rw) / 2.0 + penwidth / 2.0, ht / 2.0 + penwidth / 2.0));
    }
    rings
}

/// Generate ring 0 for a shape at the final node box `(w, h)`
/// (`w = lw + rw`, `h = ht`), mirroring poly_init's vertex output.
fn ring0_for(desc: &ShapeDesc, w: f64, h: f64) -> Vec<PointF> {
    let base = &desc.poly;
    if let Some(vg) = base.vertex_gen {
        return match vg {
            VertexGen::Star => {
                let mut v = Vec::with_capacity(10);
                let mut sz = PointF::new(w, h);
                star_vertices_into(&mut v, &mut sz);
                v
            }
            VertexGen::Cylinder => cylinder_vertices(PointF::new(w, h)),
        };
    }
    if base.sides <= 2 {
        // 2-point bbox ring (shapes.c:2169-2172)
        return vec![
            PointF::new(-w / 2.0, -h / 2.0),
            PointF::new(w / 2.0, h / 2.0),
        ];
    }
    let unit = gen_poly_vertices_inner(base.sides, base.skew, base.distortion, base.orientation);
    let xmax = unit.iter().map(|v| v.x.abs()).fold(0.0f64, f64::max);
    let ymax = unit.iter().map(|v| v.y.abs()).fold(0.0f64, f64::max);
    unit.into_iter()
        .map(|v| PointF::new(v.x / xmax * w / 2.0, v.y / ymax * h / 2.0))
        .collect()
}

/// Build all periphery rings (plus the penwidth/2 clip outline when
/// applicable) for a shape at the final node box.
fn build_rings(desc: &ShapeDesc, w: f64, h: f64, penwidth: f64) -> Vec<Vec<PointF>> {
    let base = &desc.poly;
    let peripheries = base.peripheries.max(0);
    let outp = if peripheries < 1 {
        1
    } else if peripheries >= 1 && penwidth > 0.0 {
        peripheries + 1
    } else {
        peripheries
    };
    let ring0 = ring0_for(desc, w, h);
    let sides = ring0.len();
    if sides == 0 {
        return Vec::new();
    }
    let mut flat = vec![PointF::ZERO; outp as usize * sides];
    flat[..sides].copy_from_slice(&ring0);
    if outp > 1 {
        add_periphery_rings(&mut flat, sides, peripheries, penwidth);
    }
    flat.chunks(sides).map(<[PointF]>::to_vec).collect()
}

/// `shape_vertices` — the outline polygons for painting
/// (`poly_gencode`, shapes.c:2999-3060, evaluated at the final node box
/// `w = lw+rw`, `h = ht`).
///
/// * Ellipse family (`sides <= 2`: ellipse/circle/egg/point/doublecircle/
///   Mcircle/…) returns an **empty vec** — the renderer draws an ellipse
///   (use [`ellipse_rings`] for per-ring half-extents, e.g. doublecircle).
/// * Polygonal shapes return one ring per periphery plus the
///   `penwidth/2` clip-outline ring when `penwidth > 0` — the C draw loop
///   renders only the first `peripheries` rings
///   (`poly.vertices` ring `j` at `vertices[i + j*sides]`); the final
///   ring exists for spline clipping and may be skipped by the painter.
/// * `style_rounded` (node `style=rounded`) rewrites each ring into the
///   `rounded_draw` Bézier control list (`6*sides+1` points,
///   shapes.c:643-663) — only when the descriptor has no diagonal /
///   special-shape mode of its own, mirroring the `round_corners`
///   dispatch order (shapes.c:722-732).
/// * `Mdiamond`/`Msquare` (diagonals) keep plain rings; add the corner
///   chords from [`corner_diagonals`].
/// * Cylinder: rings plus [`cylinder_bottom_cap`] for the lower ellipse;
///   `Mcircle`: [`mcircle_hack_lines`]; DOGEAR/TAB/FOLDER/BOX3D/COMPONENT
///   outlines: [`special_shape_outline`].
/// * Record/epsf return an empty vec — render from [`RecordLayout`] /
///   the node box.
///
/// `desc` with the computed `poly_init` geometry from `info` layered on —
/// the descriptor a renderer or inside-test must consult whenever a node
/// carries resolved shape info: `shape=polygon` has table `sides` 0, so
/// without the computed `sides`/`skew`/`distortion`/… a user 9-gon is
/// indistinguishable from an ellipse. A default (`kind` empty) `info`
/// leaves `desc` untouched.
pub fn resolved_desc(desc: &ShapeDesc, info: &ShapeInfo) -> ShapeDesc {
    let mut resolved = desc.clone();
    if info.kind.is_empty() {
        return resolved;
    }
    resolved.poly.sides = info.sides;
    resolved.poly.skew = info.skew;
    resolved.poly.distortion = info.distortion;
    resolved.poly.regular = info.regular;
    resolved.poly.peripheries = info.peripheries;
    resolved.poly.orientation = info.orientation;
    resolved
}

pub fn shape_vertices(
    desc: &ShapeDesc,
    lw: f64,
    rw: f64,
    ht: f64,
    penwidth: f64,
    style_rounded: bool,
) -> Vec<Vec<PointF>> {
    match desc.fns {
        ShapeFns::Record | ShapeFns::Epsf => return Vec::new(),
        ShapeFns::Poly | ShapeFns::Point | ShapeFns::Star | ShapeFns::Cylinder => {}
    }
    if desc.fns != ShapeFns::Cylinder && desc.poly.vertex_gen.is_none() && desc.poly.sides <= 2 {
        return Vec::new(); // ellipse — renderer draws an ellipse
    }
    let rings = build_rings(desc, lw + rw, ht, penwidth);
    if style_rounded && !desc.poly.diagonals && desc.poly.shape_code == 0 {
        rings.iter().map(|r| rounded_outline_points(r)).collect()
    } else {
        rings
    }
}

/// Special-shape outlines (`round_corners` cases DOGEAR/TAB/FOLDER/BOX3D/
/// COMPONENT, shapes.c:740-906): returns `(polygon, inner polylines)` for
/// the given ring (the node-frame ring 0 from [`shape_vertices`]).
/// `None` for other shapes. SBOLv glyphs are not ported.
pub fn special_shape_outline(
    desc: &ShapeDesc,
    af: &[PointF],
) -> Option<(Vec<PointF>, Vec<[PointF; 2]>)> {
    if desc.fns != ShapeFns::Poly || af.len() != 4 {
        return None;
    }
    let shape_code = desc.poly.shape_code;
    let sides = af.len();
    let mut lines: Vec<[PointF; 2]> = Vec::new();
    let polygon: Vec<PointF> = match shape_code {
        DOGEAR => {
            let b = alloc_interpolation_points(af, shape_code, false);
            let sseg = sides - 1;
            let mut d = Vec::with_capacity(sides + 1);
            d.push(b[3 * sseg + 4]); // wrap B[1]: t-point on edge 0
            for seg in 1..sides {
                d.push(af[seg]);
            }
            d.push(b[3 * sseg + 2]); // 1−t on the last edge
            // inner fold lines (shapes.c:751-758)
            let c0 = b[3 * sseg + 2];
            let c1 = b[3 * sseg + 4];
            let c2 = PointF::new(
                c1.x + (c0.x - b[3 * sseg + 3].x),
                c1.y + (c0.y - b[3 * sseg + 3].y),
            );
            lines.push([c1, c2]);
            lines.push([c0, c2]);
            d
        }
        TAB => {
            let b = alloc_interpolation_points(af, shape_code, false);
            let mut d = Vec::with_capacity(sides + 2);
            d.push(af[0]);
            d.push(b[2]);
            d.push(PointF::new(
                b[2].x + (b[3].x - b[4].x) / 3.0,
                b[2].y + (b[3].y - b[4].y) / 3.0,
            ));
            d.push(PointF::new(
                b[3].x + (b[3].x - b[4].x) / 3.0,
                b[3].y + (b[3].y - b[4].y) / 3.0,
            ));
            for seg in 4..sides + 2 {
                d.push(af[seg - 2]);
            }
            lines.push([b[3], b[2]]);
            d
        }
        FOLDER => {
            let b = alloc_interpolation_points(af, shape_code, false);
            let mut d = Vec::with_capacity(sides + 3);
            d.push(af[0]);
            d.push(PointF::new(
                af[0].x - (af[0].x - b[1].x) / 4.0,
                af[0].y + (b[3].y - b[4].y) / 3.0,
            ));
            d.push(PointF::new(af[0].x - 2.0 * (af[0].x - b[1].x), d[1].y));
            d.push(PointF::new(af[0].x - 2.25 * (af[0].x - b[1].x), b[3].y));
            d.push(b[3]);
            for seg in 4..sides + 3 {
                d.push(af[seg - 3]);
            }
            d
        }
        BOX3D => {
            let b = alloc_interpolation_points(af, shape_code, false);
            let d = vec![af[0], b[2], b[4], af[2], b[8], b[10]];
            let c0 = PointF::new(b[1].x + (b[11].x - b[0].x), b[1].y + (b[11].y - b[0].y));
            lines.push([c0, b[4]]);
            lines.push([c0, b[8]]);
            lines.push([c0, b[0]]);
            d
        }
        COMPONENT => {
            let b = alloc_interpolation_points(af, shape_code, false);
            let mut d = vec![PointF::ZERO; sides + 8];
            d[0] = af[0];
            d[1] = af[1];
            d[2] = PointF::new(b[3].x + (b[4].x - b[3].x), b[3].y + (b[4].y - b[3].y));
            d[3] = PointF::new(d[2].x + (b[3].x - b[2].x), d[2].y + (b[3].y - b[2].y));
            d[4] = PointF::new(d[3].x + (b[4].x - b[3].x), d[3].y + (b[4].y - b[3].y));
            d[5] = PointF::new(d[4].x + (d[2].x - d[3].x), d[4].y + (d[2].y - d[3].y));
            d[9] = PointF::new(b[6].x + (b[5].x - b[6].x), b[6].y + (b[5].y - b[6].y));
            d[8] = PointF::new(d[9].x + (b[6].x - b[7].x), d[9].y + (b[6].y - b[7].y));
            d[7] = PointF::new(d[8].x + (b[5].x - b[6].x), d[8].y + (b[5].y - b[6].y));
            d[6] = PointF::new(d[7].x + (d[9].x - d[8].x), d[7].y + (d[9].y - d[8].y));
            d[10] = af[2];
            d[11] = af[3];
            let c1 = PointF::new(d[2].x - (d[3].x - d[2].x), d[2].y - (d[3].y - d[2].y));
            let c2 = PointF::new(c1.x + (d[4].x - d[3].x), c1.y + (d[4].y - d[3].y));
            lines.push([d[2], c1]);
            lines.push([c1, c2]);
            lines.push([c2, d[5]]);
            let c1 = PointF::new(d[6].x - (d[7].x - d[6].x), d[6].y - (d[7].y - d[6].y));
            let c2 = PointF::new(c1.x + (d[8].x - d[7].x), c1.y + (d[8].y - d[7].y));
            lines.push([d[6], c1]);
            lines.push([c1, c2]);
            lines.push([c2, d[9]]);
            d
        }
        _ => return None,
    };
    Some((polygon, lines))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn attrs(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    /// (a) box + label "hello" (est. dimen 40x14): PAD adds +16/+8 and the
    /// 0.75x0.5 in defaults only raise the result: (max(54, 56), max(36, 22)).
    #[test]
    fn box_size_pads_label_and_honors_defaults() {
        let desc = shape_of("box");
        let (w, h) = shape_size(&desc, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.0);
        assert_eq!(w, 56.0, "max(0.75in=54, 40+16)");
        assert_eq!(h, 36.0, "max(0.5in=36, 14+8)");
    }

    /// (b) the plaintext family adds no padding and no default size.
    #[test]
    fn plaintext_adds_no_padding() {
        for name in ["plaintext", "plain", "none"] {
            let desc = shape_of(name);
            let (w, h) = shape_size(&desc, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.0);
            assert_eq!((w, h), (40.0, 14.0), "{name}");
        }
    }

    /// (c) ellipse inside test: center accepted, box corners rejected.
    #[test]
    fn ellipse_inside_rejects_corners_accepts_center() {
        let desc = shape_of("ellipse");
        let info = poly_init_info(
            &desc,
            PointF::ZERO,
            &attrs(&[("width", "1"), ("height", "1")]),
            0.0, // penwidth 0 → outline ring == drawn ellipse
            0.0,
        );
        let si = ShapeInfo {
            kind: "ellipse".into(),
            sides: info.poly.sides,
            peripheries: info.poly.peripheries,
            vertices: Some(info.poly.vertices.clone()),
            ..ShapeInfo::default()
        };
        let inside = |p| poly_inside_test(&desc, 36.0, 36.0, 72.0, &si, None, p);
        assert!(inside(PointF::ZERO));
        assert!(inside(PointF::new(25.0, 25.0)));
        assert!(inside(PointF::new(30.0, 0.0)));
        // bbox corner of the ellipse
        assert!(!inside(PointF::new(36.0, 36.0)));
        assert!(!inside(PointF::new(37.0, 0.0)));
        assert!(!inside(PointF::new(0.0, -40.0)));
    }

    /// (d) diamond: 4 vertices per ring at the expected positions.
    #[test]
    fn diamond_vertices() {
        let desc = shape_of("diamond");
        let rings = shape_vertices(&desc, 27.0, 27.0, 36.0, 0.0, false);
        assert_eq!(rings.len(), 1, "peripheries=1, penwidth=0");
        let ring = &rings[0];
        assert_eq!(ring.len(), 4);
        // (fp noise ~1e-15 mirrors the C trig; compare with tolerance)
        let close = |a: PointF, b: PointF| (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9;
        assert!(close(ring[0], PointF::new(0.0, 18.0)), "{:?}", ring[0]); // top
        assert!(close(ring[1], PointF::new(-27.0, 0.0)), "{:?}", ring[1]); // left
        assert!(close(ring[2], PointF::new(0.0, -18.0)), "{:?}", ring[2]); // bottom
        assert!(close(ring[3], PointF::new(27.0, 0.0)), "{:?}", ring[3]); // right
        // unit generation has the same count
        assert_eq!(gen_poly_vertices(4, 0.0, 0.0, false, 45.0).len(), 4);
        // the inside test rejects outside the diamond edges
        let info = poly_init_info(
            &desc,
            PointF::ZERO,
            &attrs(&[("width", "0.75"), ("height", "0.5")]),
            0.0,
            0.0,
        );
        let si = ShapeInfo {
            kind: "diamond".into(),
            sides: info.poly.sides,
            vertices: Some(info.poly.vertices.clone()),
            ..ShapeInfo::default()
        };
        assert!(poly_inside_test(
            &desc,
            27.0,
            27.0,
            36.0,
            &si,
            None,
            PointF::ZERO
        ));
        assert!(!poly_inside_test(
            &desc,
            27.0,
            27.0,
            36.0,
            &si,
            None,
            PointF::new(26.0, 17.0)
        ));
        assert!(poly_inside_test(
            &desc,
            27.0,
            27.0,
            36.0,
            &si,
            None,
            PointF::new(13.0, 8.0)
        ));
    }

    /// (e) record "{a|b}" parses into two leaf fields with rects.
    #[test]
    fn record_parses_two_fields() {
        let measure = |t: &str| PointF::new(t.chars().count() as f64 * 7.0, 14.0);
        let layout = record_layout("{a|b}", &BTreeMap::new(), true, &measure);
        // root is the "{...}" table wrapper holding one nested table
        assert_eq!(layout.root.children.len(), 1);
        let table = &layout.root.children[0];
        assert!(!table.lr, "nested table flips orientation");
        assert_eq!(table.children.len(), 2);
        let a = &table.children[0];
        let b = &table.children[1];
        assert_eq!(a.text.as_deref(), Some("a"));
        assert_eq!(b.text.as_deref(), Some("b"));
        // root/table boxes span the node; leaves are two stacked rows
        assert_eq!(layout.root.b().ll, PointF::new(-27.0, -22.0));
        assert_eq!(layout.root.b().ur, PointF::new(27.0, 22.0));
        assert_eq!(a.b().ll, PointF::new(-27.0, 0.0));
        assert_eq!(a.b().ur, PointF::new(27.0, 22.0));
        assert_eq!(b.b().ll, PointF::new(-27.0, -22.0));
        assert_eq!(b.b().ur, PointF::new(27.0, 0.0));
        // sides masks: first row lacks RIGHT, last lacks LEFT (TB parent)
        assert_eq!(a.sides, TOP | RIGHT | LEFT);
        assert_eq!(b.sides, LEFT | BOTTOM | RIGHT);
        // node size: attr box grown to content, +1 pt height kluge
        let (w, h) = record_node_size(&layout);
        assert_eq!(w, 54.0);
        assert_eq!(h, 45.0);
        // record_inside: inside the root box (+penwidth/2), outside beyond
        let desc = shape_of("record");
        assert!(record_inside_test(&layout, None, PointF::ZERO, 1.0));
        assert!(!record_inside_test(
            &layout,
            None,
            PointF::new(28.0, 0.0),
            1.0
        ));
        // the shape-sized fallback also treats it as a box
        assert!(poly_inside_test(
            &desc,
            27.0,
            27.0,
            44.0,
            &ShapeInfo::default(),
            None,
            PointF::ZERO
        ));
    }

    /// (f) compass port "e" on a box → right border midpoint.
    #[test]
    fn compass_east_on_box() {
        let desc = shape_of("box");
        let rp = compass_port(&desc, 27.0, 27.0, 36.0, None, "e", false);
        assert_eq!(rp.p, PointF::new(27.0, 0.0));
        assert!(rp.defined);
        assert!(!rp.unrecognized);
        assert!(rp.constrained);
        assert!(!rp.clip);
        assert_eq!(rp.theta, 0.0);
        assert_eq!(rp.side, RIGHT);
        assert_eq!(rp.order, 192); // MC_SCALE * (1.5π) / 2π
        // model port conversion keeps the dotgen-relevant fields
        let mp: Port = rp.clone().into();
        assert_eq!(mp.p, rp.p);
        assert!(mp.defined && !mp.clip && mp.constrained);
        // other compass points
        let n = compass_port(&desc, 27.0, 27.0, 36.0, None, "n", false);
        assert_eq!(n.p, PointF::new(0.0, 18.0));
        assert_eq!(n.side, TOP);
        assert_eq!(n.order, 0); // north pole = 0 (π/2 + 1.5π wraps to 0)
        let se = compass_port(&desc, 27.0, 27.0, 36.0, None, "se", false);
        assert_eq!(se.p, PointF::new(27.0, -18.0));
        assert_eq!(se.side, BOTTOM | RIGHT);
        let c = compass_port(&desc, 27.0, 27.0, 36.0, None, "c", false);
        assert_eq!(c.p, PointF::ZERO);
        assert!(!c.defined);
        assert_eq!(c.order, 128);
        let unk = compass_port(&desc, 27.0, 27.0, 36.0, None, "zz", false);
        assert!(unk.unrecognized);
        assert!(!unk.defined);
        assert_eq!(unk.order, 128);
        // chkPort: empty name → Center; "foo:e" on a poly node interprets
        // "foo" as the compass → unrecognized center (shapes.c:2909)
        let lead = compass_port(&desc, 27.0, 27.0, 36.0, None, ":e", false);
        assert_eq!(lead.p, PointF::ZERO);
        assert_eq!(lead.theta, -1.0);
        assert_eq!(lead.name.as_deref(), Some("e"));
        let foo = compass_port(&desc, 27.0, 27.0, 36.0, None, "foo:e", false);
        assert!(foo.unrecognized);
    }

    /// gv_nodesize (utils.c:1525-1536) — unflipped and flipped.
    #[test]
    fn gv_nodesize_matches_utils() {
        assert_eq!(gv_nodesize(0.75, 0.5, false), (27.0, 27.0, 36.0));
        assert_eq!(gv_nodesize(0.75, 0.5, true), (18.0, 18.0, 54.0));
    }

    /// point shape: DEF_POINT diameter, peripheries rings, MIN_POINT.
    #[test]
    fn point_shape_sizing() {
        let desc = shape_of("point");
        let (w, h) = shape_size(&desc, PointF::ZERO, &BTreeMap::new(), 0.0);
        assert_eq!((w, h), (3.6, 3.6)); // DEF_POINT = 0.05 in
        let info = poly_init_info(&desc, PointF::ZERO, &BTreeMap::new(), 0.0, 0.0);
        assert_eq!(info.poly.sides, 2);
        assert_eq!(info.width_in, 0.05);
        // width=0.1in wins over the default
        let (w2, h2) = shape_size(&desc, PointF::ZERO, &attrs(&[("width", "0.1")]), 0.0);
        assert_eq!((w2, h2), (7.2, 7.2));
        // peripheries=2 adds a GAP ring: 3.6 + 2*4 = 11.6? no: width attr
        // unset → 3.6 + 8 = 11.6 pt
        let info = poly_init_info(
            &desc,
            PointF::ZERO,
            &attrs(&[("peripheries", "2")]),
            0.0,
            0.0,
        );
        assert!((info.width_in * POINTS_PER_INCH - (3.6 + 2.0 * GAP)).abs() < 1e-9);
    }

    /// doublecircle: two ellipse rings 4pt apart, regular sizing.
    #[test]
    fn doublecircle_rings() {
        let desc = shape_of("doublecircle");
        let (w, h) = shape_size(&desc, PointF::new(20.0, 20.0), &BTreeMap::new(), 0.0);
        // regular: dimen padded to (36,28), ellipse-fit, regular max → then
        // ring expansion +GAP each side
        let info = poly_init_info(&desc, PointF::new(20.0, 20.0), &BTreeMap::new(), 0.0, 0.0);
        assert_eq!(info.poly.peripheries, 2);
        assert_eq!(info.poly.sides, 2);
        assert_eq!(info.poly.vertices.len(), 4); // 2 rings x 2 points
        let rings = ellipse_rings(&desc, w / 2.0, w / 2.0, h, 0.0);
        assert_eq!(rings.len(), 2);
        assert!((rings[1].0 - rings[0].0 - GAP).abs() < 1e-9);
        let _ = (w, h);
    }

    /// margin attribute parsing: explicit margin replaces PAD; invalid
    /// text falls back to PAD.
    #[test]
    fn margin_attr() {
        let desc = shape_of("box");
        // margin=0.1in → dimen (40+14.4, 14+14.4); defaults still raise y
        let (w, h) = shape_size(
            &desc,
            PointF::new(40.0, 14.0),
            &attrs(&[("margin", "0.1")]),
            0.0,
        );
        assert_eq!((w, h), (54.4, 36.0));
        // margin=0,0 disables padding entirely
        let (w, h) = shape_size(
            &desc,
            PointF::new(40.0, 14.0),
            &attrs(&[("margin", "0,0")]),
            0.0,
        );
        assert_eq!((w, h), (54.0, 36.0));
        let (w, _) = shape_size(
            &desc,
            PointF::new(40.0, 14.0),
            &attrs(&[("margin", "junk")]),
            0.0,
        );
        assert_eq!(w, 56.0); // PAD fallback
        // two-component margin
        let (w, h) = shape_size(
            &desc,
            PointF::new(40.0, 14.0),
            &attrs(&[("margin", "0.1,0.2")]),
            0.0,
        );
        assert_eq!((w, h), (54.4, 42.8));
    }

    /// star and cylinder size generators.
    #[test]
    fn star_and_cylinder_sizing() {
        let star = shape_of("star");
        let (w, h) = shape_size(&star, PointF::new(20.0, 20.0), &BTreeMap::new(), 0.0);
        let info = poly_init_info(&star, PointF::new(20.0, 20.0), &BTreeMap::new(), 0.0, 0.0);
        assert_eq!(info.poly.sides, 10);
        assert_eq!(info.poly.vertices.len(), 10);
        // star aspect ratio: height/width = (1+sin54°)/(2cos18°)
        let aspect = (1.0 + (3.0 * PI / 10.0).sin()) / (2.0 * (PI / 10.0).cos());
        assert!(((h / w - aspect).abs()) < 1e-9);
        let _ = (w, h);

        let cyl = shape_of("cylinder");
        let (w, h) = shape_size(&cyl, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.0);
        assert_eq!((w, h), (56.0, 36.0)); // 1.375 y-fit then defaults
        let rings = shape_vertices(&cyl, 28.0, 28.0, 36.0, 0.0, false);
        assert_eq!(rings.len(), 1);
        assert_eq!(rings[0].len(), 19);
        // v[0] = (x, y − yr) with x = 28, y = 18, yr = 36/11 (shapes.c:4165)
        assert_eq!(rings[0][0], PointF::new(28.0, 18.0 - 36.0 / 11.0));
        let cap = cylinder_bottom_cap(&rings[0]);
        assert_eq!(cap[0], rings[0][0]);
        assert!((cap[1].y - (2.0 * rings[0][0].y - rings[0][1].y)).abs() < 1e-9);
    }

    /// record ports: field lookup + compass, and the fallback compass.
    #[test]
    fn record_ports() {
        let measure = |t: &str| PointF::new(t.chars().count() as f64 * 7.0, 14.0);
        let layout = record_layout("<p>hello|{x|<q>y}", &BTreeMap::new(), true, &measure);
        let desc = shape_of("record");
        // port "q" found: east of the q field
        let rp = compass_port(&desc, 27.0, 27.0, 45.0, Some(&layout), "q:e", false);
        assert!(rp.defined);
        assert!(!rp.unrecognized);
        assert_eq!(rp.name.as_deref(), Some("e"));
        let q = layout.root.find_port("q").unwrap();
        assert_eq!(rp.p, PointF::new(q.b().ur.x, q.center().y));
        assert_eq!(rp.bp, Some(q.b()));
        // unknown field id: compass fallback against the root box
        let rp = compass_port(&desc, 27.0, 27.0, 45.0, Some(&layout), "nope", false);
        assert!(rp.unrecognized);
        assert!(rp.defined);
        assert_eq!(rp.p, layout.root.center());
        // dyna port on a found field
        let rp = compass_port(&desc, 27.0, 27.0, 45.0, Some(&layout), "p", false);
        assert!(rp.dyna);
        assert!(rp.defined);
        assert_eq!(rp.p, layout.root.find_port("p").unwrap().center());
    }

    /// Rounded corners: RBCONST/RBCURVE interpolation list shape.
    #[test]
    fn rounded_corners_layout() {
        let desc = shape_of("box");
        let rings = shape_vertices(&desc, 27.0, 27.0, 36.0, 0.0, true);
        assert_eq!(rings.len(), 1);
        // 6*sides + 1 points (start + 3 controls per cubic, 2*sides cubics)
        assert_eq!(rings[0].len(), 6 * 4 + 1);
        // corner radius capped at RBCONST / 3 per edge... rbconst = min(12, len/3)
        // box: 72 wide, 36 tall → rbconst = min(12, 72/3, 36/3) = 12 → t = 12/36
        let b = alloc_interpolation_points(
            &[
                PointF::new(27.0, 18.0),
                PointF::new(-27.0, 18.0),
                PointF::new(-27.0, -18.0),
                PointF::new(27.0, -18.0),
            ],
            0,
            false,
        );
        // B[1] = t-point on edge 0 from corner 0
        assert!((b[1].x - (27.0 - 12.0)).abs() < 1e-9);
        // Mdiamond keeps plain rings; chords via corner_diagonals
        let md = shape_of("Mdiamond");
        let rings = shape_vertices(&md, 27.0, 27.0, 36.0, 0.0, false);
        assert_eq!(rings[0].len(), 4);
        assert_eq!(corner_diagonals(&rings[0]).len(), 4);
    }

    /// inside_t-style port box fast path (shapes.c:2420-2424).
    #[test]
    fn inside_port_box_fast_path() {
        let desc = shape_of("box");
        let b = BoxF::new(PointF::new(-10.0, -5.0), PointF::new(10.0, 5.0));
        assert!(poly_inside_test(
            &desc,
            27.0,
            27.0,
            36.0,
            &ShapeInfo::default(),
            Some(b),
            PointF::new(9.0, 4.0)
        ));
        assert!(!poly_inside_test(
            &desc,
            27.0,
            27.0,
            36.0,
            &ShapeInfo::default(),
            Some(b),
            PointF::new(11.0, 0.0)
        ));
    }

    /// Unknown shapes bind to box (Shapes[0] fallback, shapes.c:3958).
    #[test]
    fn unknown_shape_falls_back_to_box() {
        let desc = shape_of("castle");
        assert_eq!(desc.name, "box");
        assert_eq!(desc.poly.sides, 4);
        let (w, _) = shape_size(&desc, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.0);
        assert_eq!(w, 56.0);
        // lookup is case-sensitive like streq
        assert_eq!(shape_of("BOX").name, "box");
        assert_eq!(shape_of("ellipse").name, "ellipse");
    }

    /// fixedsize / regular handling.
    #[test]
    fn fixedsize_and_regular() {
        let desc = shape_of("ellipse");
        // fixed=true: node size is the attribute box even if small
        let (w, h) = shape_size(
            &desc,
            PointF::new(40.0, 14.0),
            &attrs(&[("width", "0.5"), ("height", "0.5"), ("fixed", "true")]),
            0.0,
        );
        assert_eq!((w, h), (36.0, 36.0));
        // regular square: max dimension wins
        let sq = shape_of("square");
        let (w, h) = shape_size(&sq, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.0);
        assert_eq!(w, h);
        assert!(w >= 56.0);
        // quantum quantizes the padded dimen
        let desc = shape_of("box");
        let (w, _) = shape_size(&desc, PointF::new(40.0, 14.0), &BTreeMap::new(), 0.1);
        assert_eq!(w, quant(56.0, 7.2).max(54.0));
    }

    /// invflip_side / invflip_angle tables (shapes.c:2548-2635).
    #[test]
    fn invflip_tables() {
        assert_eq!(invflip_side(TOP, RankDir::Lr), RIGHT);
        assert_eq!(invflip_side(LEFT, RankDir::Lr), TOP);
        assert_eq!(invflip_side(RIGHT, RankDir::Rl), TOP);
        assert_eq!(invflip_side(LEFT, RankDir::Rl), BOTTOM);
        assert_eq!(invflip_side(TOP, RankDir::Bt), BOTTOM);
        assert_eq!(invflip_side(BOTTOM, RankDir::Tb), BOTTOM);
        assert_eq!(invflip_angle(PI, RankDir::Bt), -PI);
        assert_eq!(invflip_angle(PI * 0.5, RankDir::Lr), 0.0);
        assert_eq!(invflip_angle(0.0, RankDir::Lr), -PI * 0.5);
        assert_eq!(invflip_angle(PI * 0.5, RankDir::Rl), 0.0);
        assert_eq!(invflip_angle(0.0, RankDir::Rl), PI * 0.5);
        assert_eq!(invflip_angle(-PI * 0.5, RankDir::Rl), PI);
    }
}
