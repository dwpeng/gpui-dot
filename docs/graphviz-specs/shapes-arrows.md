# Graphviz Node Shapes & Edge Arrows — Implementation Spec for a Faithful Rust Port

Sources (read in full, current `main` of graphviz):

* `lib/common/arrows.c` — 1361 lines (all cited)
* `lib/common/shapes.c` — 4342 lines (all relevant parts cited)
* `lib/common/splines.c` — arrow/spline-end clipping entry points (`arrow_clip`, `bezier_clip`, `clip_and_install`, `shape_clip`)
* `lib/common/utils.c` — `gv_nodesize`, `common_init_node`, `common_init_edge`, `polyBB`, `Bezier`, `late_*` helpers
* `lib/common/labels.c` — `make_label`, `make_simple_label`, `storeline`, `emit_label` (label dimension model)
* `lib/common/textspan.c`, `lib/common/const.h`, `lib/common/types.h`, `lib/common/macros.h`, `lib/common/geom.h`, `lib/common/geom.c`, `lib/common/arith.h`, `lib/dotgen/dotinit.c`, `lib/dotgen/position.c`, `lib/common/htmltable.c` (html ports), `lib/common/psusershape.c` (epsf), `lib/common/emit.c` (arrow emission sites)

Everything below is in **points** (1 inch = 72 points, `POINTS_PER_INCH = 72`, geom.h:58; `INCH2PS(a) = a*72`, `PS2INCH(p) = p/72`, geom.h:63–64) unless a name says *inches*. Graphviz node coordinates are **math orientation in the node's own frame** (+y up, origin at node center); the *layout/render* frame has +y down and is related to the node frame by the `rankdir` rotations (`cwrotatepf`/`ccwrotatepf`, geom.c:130–163).

---

## 0. Conventions and helper macros used throughout

| Macro | Definition | Source |
|---|---|---|
| `POINTS_PER_INCH` | `72` | geom.h:58 |
| `INCH2PS(x)` / `PS2INCH(x)` | `x*72` / `x/72` | geom.h:63–64 |
| `INSIDE(p,b)` | `b.LL.x <= p.x <= b.UR.x && b.LL.y <= p.y <= b.UR.y` (`BETWEEN`) | geom.h:45 |
| `DIST(p,q)` / `DIST2` | euclidean distance / squared | geom.h:55–56 |
| `APPROXEQPT(p,q,tol)` | `DIST2(p,q) < tol²` | geom.h:71 |
| `MILLIPOINT` | `0.001` | geom.h:74 |
| `GAP` | `4` points — whitespace between peripheries and around labels | const.h:251 |
| `XPAD(d)/YPAD(d)/PAD(d)` | `d.x += 4*GAP (=16)`; `d.y += 2*GAP (=8)`; `PAD` = both. **This is the modern replacement for the old `LMARGIN/RMARGIN/BMARGIN/TIMARGIN` (0.11/0.055 in).** | macros.h:27–29 |
| `SQRT2` | `1.41421356237309504880` | arith.h:44–45 |
| `RADIANS(deg)` | `deg/180*π` | arith.h:49 |
| `MC_SCALE` | `256` (mincross ordering granularity) | const.h:99 |
| `quant(val,q)` | `ceil(val/q)*q` | shapes.c:367–370 |

Point rotations (geom.c:130–163), used pervasively for `rankdir`:

```
cwrotatepf(p, 90k):  k=90  → ( p.y, -p.x)      # clockwise in graph frame
                     k=180 → ( p.x, -p.y)
                     k=270 → ( p.y,  p.x)      # exch_xyf
ccwrotatepf(p, 90k): k=90  → ( p.y,  p.x)      # perp
                     k=180 → ( p.x, -p.y)
                     k=270 → ( p.y,  p.x)      # exch_xyf
```

`flip_rec_boxf(b, p)` (geom.c:166–173): swap x/y of both corners (`exch_xyf`), then add point `p`.

Note on historical constants the porting brief asked about: this source tree has **no** `LMARGIN/RMARGIN/BMARGIN/TIMARGIN`, **no** `NV(...)` macro, and **no** `REC_*` macros. The current equivalents are `PAD` (16 pt horizontal, 8 pt vertical total padding), `late_double`-based defaults, and `field_t.b` rects; see the record section. The "0.375 in" figure is just half of `DEFAULT_NODEWIDTH = 0.75` in and appears as `ND_lw = ND_rw = INCH2PS(0.75)/2` in `gv_nodesize`.

---

## 1. Core data model (types.h)

### 1.1 `port` (types.h:40–57)

```rust
struct Port {
    p: PointF,          // aiming point, relative to node center, node frame
    theta: f64,         // slope constraint in radians (0 = east; -1 == unconstrained sentinel)
    bp: Option<BoxF>,   // if set, rectangle target (record field / html cell); port box
    defined: bool,      // edge had a port spec at this end
    constrained: bool,  // theta is meaningful
    clip: bool,         // clip spline end to node/port shape
    dyna: bool,         // compass "_" — choose side dynamically
    order: u8,          // mincross ordering 0..=MC_SCALE
    side: u8,           // bitmask BOTTOM|RIGHT|TOP|LEFT of exposed sides
    name: String,       // port name if explicitly given
}
```

Side bits (const.h:111–120): `BOTTOM = 1<<0, RIGHT = 1<<1, TOP = 1<<2, LEFT = 1<<3`, with `BOTTOM_IX..LEFT_IX = 0..3`.

The default port is `static port Center = {.theta = -1, .clip = true}` (shapes.c:38) — i.e. `p = (0,0)`, `theta = -1` (unused because `constrained == false`), `clip = true`, everything else false/0.

### 1.2 `polygon_t` (types.h:143–152) — per-node mutable shape data

```rust
struct Polygon {
    regular: bool,
    peripheries: usize,
    sides: usize,
    orientation: f64,   // degrees, +CCW
    distortion: f64,
    skew: f64,
    option: PolyStyle,  // ROUNDED/DIAGONALS/etc, see below
    vertices: Vec<PointF>, // peripheries(+outline) rings of `sides` points, node frame, center origin
}
```

### 1.3 `graphviz_polygon_style_t` (types.h:135–141) and shape codes (const.h:192–219)

```rust
struct PolyStyle {           // all bool except shape:u8
    filled, radial, rounded, diagonals, auxlabels, invisible,
    striped, dotted, dashed, wedged, underline, fixedshape: bool,
    shape: u8,               // 0 = none, else one of:
}                            // DOGEAR=1, TAB=2, FOLDER=3, BOX3D=4, COMPONENT=5,
                             // PROMOTER=6, CDS=7, TERMINATOR=8, UTR=9, PRIMERSITE=10,
                             // RESTRICTIONSITE=11, FIVEPOVERHANG=12, THREEPOVERHANG=13,
                             // NOVERHANG=14, ASSEMBLY=15, SIGNATURE=16, INSULATOR=17,
                             // RIBOSITE=18, RNASTAB=19, PROTEASESITE=20, PROTEINSTAB=21,
                             // RPROMOTER=22, RARROW=23, LARROW=24, LPROMOTER=25, CYLINDER=26
```

### 1.4 `shape_functions` (types.h:178–185) / `shape_desc` (types.h:189–194)

```rust
struct ShapeFns { init, free, port, inside, pbox, code } // see tables in §4.2
struct ShapeDesc { name: &str, fns: ShapeFns, polygon: Option<&'static PolygonDesc>, usershape: bool }
```

`insidefn(inside_t, pointf p)` — p is **relative to node center in the *rotated* (layout) frame**; each insidefn first converts with `ccwrotatepf(p, 90*rankdir)`. `pboxfn` returns a box path from a port to the border (only `record_path` implements it; `poly_path` is a 0-box stub, shapes.c:2537–2546).

### 1.5 `inside_t` (types.h:154–171)

A union used two ways:
* by `bezier_clip` callers in arrows.c: `.a = { p: *PointF, r: *f64 }` — circle center + radius;
* by shape inside functions: `.s = { n, bp, lastn, radius, last_poly, last, outp, scalex, scaley, box_URx, box_URy }` — a per-node cache.

### 1.6 `field_t` (types.h:235–244) — record shape node

```rust
struct Field {
    size: PointF,        // dimension in points
    b: BoxF,             // placement in node coords (LL/UR, origin = node center, y up)
    n_flds: usize,
    lp: Option<TextLabel>,  // present iff n_flds == 0 (leaf with text)
    fld: Vec<Field>,        // children iff n_flds > 0
    id: Option<String>,     // port identifier from <...>
    lr: bool,               // children laid left-to-right
    sides: u8,              // exposed-sides bitmask
}
```

### 1.7 `textlabel_t` (types.h:112–133)

```rust
struct TextLabel {
    text, fontname, fontcolor: String,
    charset: i32, fontsize: f64,
    dimen: PointF,  // estimated rendered size of label, points
    space: PointF,  // size of the space the label is aligned within (excludes pad/margin)
    pos: PointF,    // center of `space` (absolute coords at emit time)
    spans: Vec<TextSpan>,  // or html tree
    valign: char,   // 't' | 'c' | 'b'
    set: bool, html: bool,
}
```

### 1.8 Node/edge layout accessors (types.h:506–537, 560–590)

`ND_width/ND_height` are **inches** (the attribute values after init); `ND_lw`, `ND_rw` are left/right half-widths in **points**; `ND_ht` is full height in points; `ND_outline_width/ND_outline_height` are inches including penwidth; `ND_xsize = lw+rw`, `ND_ysize = ht`. Edges carry `ED_tail_port/ED_head_port` (type `port`), `ED_label` etc.

---

## 2. Master table of numeric constants

| Constant | Value | Meaning | Source |
|---|---|---|---|
| `EPSILON` | `0.0001` | stabilizes arrow vector normalization | arrows.c:26 |
| `ARROW_LENGTH` | `10.0` | standard arrow length in points | arrows.c:29 |
| `NUMB_OF_ARROW_HEADS` | `4` | max arrows packed per edge end | arrows.c:31 |
| `BITS_PER_ARROW` | `8` | bits per arrow in packed flag | arrows.c:34 |
| `BITS_PER_ARROW_TYPE` | `4` | low bits of an arrow = type | arrows.c:36 |
| `ARR_NONE` | `0` | no arrow / terminated flag list | const.h:108, arrows.c:38 |
| `ARR_TYPE_NORM..GAP` | 1..8 | normal, crow, tee, box, diamond, dot, curve, gap | arrows.c:39–46 |
| `ARR_MOD_OPEN/INV/LEFT/RIGHT` | `1<<4..1<<7` | `o`, `inv` special, `l`, `r` | arrows.c:50–53 |
| `lenfact` per type | norm 1.0, crow 1.0, tee 0.5, box 1.0, diamond 1.2, dot 0.8, curve 1.0, gap 0.5 | relative arrow lengths | arrows.c:146–155 |
| `stroke_miterlimit` | `4.0` | SVG miter limit in `miter_shape` | arrows.c:484 |
| `arrowwidth` (normal) | `0.35` (× `penwidth/4` if penwidth>4) | half-width factor of u | arrows.c:521–523 |
| crow `arrowwidth`/`shaftwidth` | `0.45` / `0.05*(penwidth-1)/arrowsize` | crow geometry | arrows.c:637–643 |
| tee proportions | `0.2`, `0.6` of u | crossbar positions | arrows.c:818–821 |
| box proportions | `0.4`, `0.8` of u | half-width, box length | arrows.c:875–878 |
| diamond | `u/3`, `u/2` | half-width, waist | arrows.c:930–933 |
| dot | `r = |u|/2` | radius | arrows.c:995 |
| curve `arrowwidth` | `0.5` (`0.5*penwidth/4` if penwidth>4) | semicircle factor | arrows.c:1035 |
| curve Bézier magic | `0.95`, `4.0/3.0` | semicircle approximation | arrows.c:1067–1078 |
| `RBCONST` | `12` | max corner radius (points) for rounded/diagonals | shapes.c:30 |
| `RBCURVE` | `0.5` | fraction of `t` used for Bézier corner approach | shapes.c:31 |
| `DEF_POINT` | `0.05` in (3.6 pt) | default `shape=point` diameter | shapes.c:42 |
| `MIN_POINT` | `0.0003` in (0.02 pt) | min point diameter | shapes.c:47 |
| `GAP` | `4` | periphery/ring spacing, label pad unit | const.h:251 |
| `DEFAULT_NODEWIDTH` / `DEFAULT_NODEHEIGHT` | `0.75` / `0.5` inches | node attribute defaults | const.h:74,72 |
| `MIN_NODEWIDTH` / `MIN_NODEHEIGHT` | `0.01` / `0.02` inches | clamps | const.h:75,73 |
| `DEFAULT_NODESHAPE` | `"ellipse"` | | const.h:76 |
| `DEFAULT_NODEPENWIDTH` / `MIN_NODEPENWIDTH` | `1.0` / `0.0` | | const.h:77–78 |
| `DEFAULT_FONTSIZE` / `MIN_FONTSIZE` | `14.0` / `1.0` | node & edge label font size | const.h:61,63 |
| `DEFAULT_LABEL_FONTSIZE` | `11.0` | head/tail labels (attribute default, not used in utils.c paths below) | const.h:62 |
| `DEFAULT_FONTNAME` / `DEFAULT_COLOR` / `DEFAULT_FILL` | `"Times-Roman"` / `"black"` / `"lightgrey"` | | const.h:67,48,69 |
| `LINESPACING` | `1.20` | line height factor | const.h:70 |
| `MC_SCALE` | `256` | port order granularity | const.h:99 |
| cylinder `size` factor / rim | `1.375` / `1/11`, `0.551784` | ellipse κ ≈ 1−cos22.5° | shapes.c:4153–4197 |
| star angles | `α = π/10` (18°), `α2=36°`, `α3=54°`, `α4=72°` | | shapes.c:4034–4037 |
| `W_DEGREE` | `5` | Bézier degree in `Bezier()` | utils.c:167 |
| FILL / GRADIENT / RGRADIENT | 1 / 2 / 3 | fill enum passed to renderers | const.h:222–224 |

---

# Part A — `arrows.c`

## A.1 Packed arrow flag format (arrows.c:26–55)

A `uint32_t` carries up to 4 arrowheads, 8 bits each: arrow `i` occupies bits `[8i, 8i+8)`. Within one byte, the low 4 bits are the **type** (`ARR_TYPE_*`, values 1–8; 0 = none) and bits 4–7 are **modifiers**: `ARR_MOD_OPEN` (bit 4, `o`/`e`), `ARR_MOD_INV` (bit 5, only used by names `inv` and `vee`/`pen`/`icurve`), `ARR_MOD_LEFT` (bit 6, `l`/`half`), `ARR_MOD_RIGHT` (bit 7, `r`).

Parsing produces, for the **head end** (`eflag`) and **tail end** (`sflag`) independently. In the packed word, arrow index 0 is **closest to the node** at that end (drawn first, then the vector advances outward — arrows.c:1171–1177).

## A.2 Name tables (arrows.c:62–115)

```text
Arrowdirs (dir attribute): "forward" → (s=NONE, e=NORM); "back" → (NORM, NONE);
                           "both"  → (NORM, NORM);  "none" → (NONE, NONE)
Arrowsynonyms: "invempty" → NORM|INV|OPEN           (deprecated "oinv")
Arrowmods:     "o"→OPEN, "r"→RIGHT, "l"→LEFT,
               "e"→OPEN (compat), "half"→LEFT (compat)
Arrownames:    "normal"→NORM, "crow"→CROW, "tee"→TEE, "box"→BOX,
               "diamond"→DIAMOND, "dot"→DOT, "none"→GAP,
               "inv"→NORM|INV, "vee"→CROW|INV, "pen"→CROW|INV (see below),
               "mpty"→NORM (compat: makes "empty" parse as o+mpty = OPEN|NORM),
               "curve"→CURVE, "icurve"→CURVE|INV
```

### Name resolution algorithm

```
arrow_match_name_frag(name, table, &flag)          # arrows.c:160-175
  for each entry: if name.starts_with(entry.name) { flag |= entry.type; return name + len }
  return name (unmodified)

arrow_match_shape(name, &flag) -> rest             # arrows.c:177-193
  f = 0
  rest = match_frag(name, Arrowsynonyms)
  if rest == name:                       # no synonym; try any number of mods then the base name
     loop: next = rest; rest = match_frag(next, Arrowmods) until no progress
     rest = match_frag(rest, Arrownames)
  if f != 0 and (f & 0xF) == 0: f |= ARR_TYPE_NORM   # mods without a name => normal
  return rest

arrow_match_name(name, &flag)                      # arrows.c:195-216
  flag = 0; i = 0
  while *rest and i < 4:
     f = 0; next = rest; rest = arrow_match_shape(next, &f)
     if f == 0 { warn "Arrow type \"%s\" unknown - ignoring"; return }   # whole spec dropped
     if f == GAP and i == 3: f = 0                        # trailing "none" dropped
     if f == GAP and i == 0 and *rest == '\0': f = 0      # bare "none" dropped
     if f != 0 { flag |= f << (8*i); i += 1 }
```

Consequences for the documented arrow names (all resolved exactly by the above):

| Attribute name | Packed value | Rendered as |
|---|---|---|
| `normal` | `NORM` | filled triangle |
| `onormal` | `NORM\|OPEN` | open (stroked) triangle |
| `inv` | `NORM\|INV` | triangle pointing *back* into the edge |
| `empty` | `OPEN\|NORM` | = onormal |
| `invempty` | `NORM\|INV\|OPEN` | open back triangle |
| `diamond` | `DIAMOND` | filled diamond |
| `odiamond` / `ediamond` | `DIAMOND\|OPEN` | open diamond |
| `dot` | `DOT` | filled disc |
| `odot` | `DOT\|OPEN` | open disc |
| `crow` | `CROW` | crow's foot |
| `vee` / `open` / `pen` | `CROW\|INV` | V (open at node side) |
| `halfopen` | `CROW\|INV\|OPEN\|LEFT` | half V (LEFT has no effect on crow draw; OPEN ignored for CROW) |
| `tee` | `TEE` | T bar |
| `box` / `obox` | `BOX` / `BOX\|OPEN` | filled/open rectangle |
| `curve` / `icurve` | `CURVE` / `CURVE\|INV` | concave/convex semicircle |
| `none` | nothing (or `GAP` as a *spacer* inside a list, e.g. `arrowhead="nonenormal"`) | gap segment |

Compound specs like `arrowhead="dotodot"` or `"normalnone"` parse into 2–4 stacked arrowheads via the loop above; each is drawn in sequence outward from the node.

## A.3 `arrow_flags` — which arrows an edge gets (arrows.c:218–251)

```
arrow_flags(e) -> (sflag, eflag):
  sflag = ARR_NONE
  eflag = graph_is_directed ? ARR_TYPE_NORM : ARR_NONE
  if E_dir attr nonempty:                       # "forward"/"back"/"both"/"none"
      sflag, eflag = Arrowdirs[value]
  if eflag == ARR_TYPE_NORM:                    # arrowhead attr only applies if an arrow exists
      if "arrowhead" attr exists and nonempty: arrow_match_name(attr, &eflag)
  if sflag == ARR_TYPE_NORM:
      if "arrowtail" attr exists and nonempty: arrow_match_name(attr, &sflag)
  if ED_conc_opp_flag(e):                       # flat edge with opposing edge (same pair reversed)
      f = edge(aghead(e), agtail(e)); (s0, e0) = arrow_flags(f)
      eflag |= s0 ; sflag |= e0
```

Note `dir=both` gives arrows on both ends (each end independently `NORM` unless overridden by `arrowhead`/`arrowtail`).

## A.4 Length model — `arrow_length` and per-type length functions

`arrow_length(e, flag)` (arrows.c:253–277):

```
penwidth  = late_double(e, E_penwidth, 1.0, 0.0)
arrowsize = late_double(e, E_arrowsz,  1.0, 0.0)
if arrowsize == 0: return 0
length = 0
for i in 0..4:                                  # over packed arrows
    f = (flag >> 8i) & 0xF                      # type only
    for t in Arrowtypes: if t.type == f:
        arrow_flag = (flag >> 8i) & 0xFF        # type + mods
        length += t.len(t.lenfact, arrowsize, penwidth, arrow_flag)
```

All length functions return the distance the spline end must be pulled back from the arrow tip. `ARROW_LENGTH = 10.0` is the base; `u = (lenfact * arrowsize * ARROW_LENGTH, 0)` is the nominal arrow vector used to re-run the geometry at the origin:

* `arrow_length_generic` (gap; also used for none/tee fallback): `lenfact * arrowsize * ARROW_LENGTH` (arrows.c:1182–1188).
* `arrow_length_normal` (arrows.c:1190–1227): re-runs `arrow_type_normal0(p=(0,0), u, penwidth, flag, a)` with `u = (lenfact*arrowsize*ARROW_LENGTH, 0)`; then
  `full_length = q.x` (tip-to-base distance along +x, arrow ends at origin),
  `nominal_length = |a[1].x - a[2].x|` (base1→tip), `nominal_base_width = a[3].y - a[1].y`,
  `full_base_width = nominal_base_width * full_length / nominal_length`,
  `overlap_at_base = penwidth/2`,
  `overlap_at_tip = full_length * penwidth / full_base_width` (distance from tip where arrow width == penwidth),
  `overlap = INV ? overlap_at_tip : overlap_at_base`,
  **return `full_length - overlap`**.
* `arrow_length_tee` (arrows.c:1229–1254): `L = lenfact*arrowsize*ARROW_LENGTH`; if `penwidth/2 - 0.4*L > 0` add `penwidth/2 - 0.4*L`; and (bug-compatible) `if (penwidth/2 - 0.4*L) > 0` again add `penwidth/2 - 0.2*L` — note the second `if` tests the *start* extension but adds the *end* amount (arrows.c:1243–1251, replicate literally).
* `arrow_length_box` (1256–1265): `lenfact*arrowsize*ARROW_LENGTH + penwidth/2`.
* `arrow_length_diamond` (1267–1306): re-runs `arrow_type_diamond0` at origin; `full_length = q.x/2` (half-depth = waist), `nominal_*` from `a[3],a[1],a[2]`, `overlap = full_length*penwidth/full_base_width`, **return `2*full_length - overlap`**.
* `arrow_length_dot` (1308–1313): `lenfact*arrowsize*ARROW_LENGTH + penwidth`.
* `arrow_length_curve` (1315–1320): `lenfact*arrowsize*ARROW_LENGTH + penwidth/2`.
* `arrow_length_crow` (1322–1361): re-runs `arrow_type_crow0`; `full_length = q.x`; `full_length_without_shaft = full_length - (a[1].x - a[3].x)`; `nominal_length = |a[1].x - a[0].x|`, `nominal_base_width = a[7].y - a[1].y`, `full_base_width = nominal_base_width * full_length_without_shaft / nominal_length`; `overlap = INV ? penwidth/2 : full_length_without_shaft*penwidth/full_base_width`; **return `full_length - overlap`**.

These lengths are what `arrowStartClip`/`arrowEndClip`/`arrowOrthoClip` use to shorten the spline.

## A.5 Clipping spline ends to arrows

### A.5.1 `bezier_clip` (splines.c:107–151) — binary search on a cubic

```
bezier_clip(inside_context, insidefn, sp[4], left_inside):
  if left_inside:  pt = sp[0]; idir = low;  odir = high;  left=NULL, right=seg
  else:            pt = sp[3]; idir = high; odir = low;   left=seg,  right=NULL
  low = 0; high = 1; found = false
  loop {
      opt = pt
      t = (high + low)/2
      pt = Bezier(sp, t, left, right)        # de Casteljau; fills left/right subdivision ctrl pts
      if insidefn(context, pt) { *idir = t; best = seg; found = true }
      else                      { *odir = t }
  } while (|opt.x-pt.x| > 0.5 or |opt.y-pt.y| > 0.5)      # half-point tolerance
  sp = found ? best : seg
```

`Bezier(V,t,Left,Right)` (utils.c:175–203): standard degree-5 triangle for a cubic; `Left[j] = Vt[j][0]`, `Right[j] = Vt[3-j][j]`.

### A.5.2 `inside` for arrows (arrows.c:280–283)

`DIST(p, ctx.a.p[0]) <= ctx.a.r[0]` — inside a circle of radius = arrow length centered on the (original) spline endpoint.

### A.5.3 `arrowEndClip` (arrows.c:285–311) — head end

```
elen = arrow_length(e, eflag); spl.eflag = eflag; spl.ep = ps[endp+3]   # remember true tip
if endp > startp and DIST(ps[endp], ps[endp+3]) < elen: endp -= 3       # last Bézier too short: use previous
sp = [spl.ep, ps[endp+2], ps[endp+1], ps[endp]]      # REVERSED copy; sp[0] = tip ("starts inside")
if elen > 0: bezier_clip(ctx(sp[0]=tip, r=elen), inside, sp, left_inside = true)
ps[endp]   = sp[3];  ps[endp+1] = sp[2];  ps[endp+2] = sp[1];  ps[endp+3] = sp[0]
return endp
```

Effect: the last control point `ps[endp+3]` (the spline tip) is **replaced by the point at distance `elen` from the tip** (the arrow base); `ps[endp+1]` and `ps[endp+2]` are replaced by the de-Casteljau right-subdivision controls of the reversed curve (note `ps[endp]` itself is unchanged — it is `best[3]`). The arrowhead itself is later drawn at `spl.ep` with direction `ep − list[last]`.

### A.5.4 `arrowStartClip` (arrows.c:313–339) — tail end

```
slen = arrow_length(e, sflag); spl.sflag = sflag; spl.sp = ps[startp]    # true tip
if endp > startp and DIST(ps[startp], ps[startp+3]) < slen: startp += 3
sp = [ps[startp+3], ps[startp+2], ps[startp+1], spl.sp]   # reversed; sp[3] = tip
if slen > 0: bezier_clip(ctx(sp[3]=tip, r=slen), inside, sp, left_inside = false)
ps[startp]   = sp[3];  ps[startp+1] = sp[2];  ps[startp+2] = sp[1];  ps[startp+3] = sp[0]
return startp
```

The first control point `ps[startp]` (the tail tip) is replaced by the point `slen` away from the tip; `ps[startp+1]`/`ps[startp+2]` come from the left-subdivision controls; `ps[startp+3]` is unchanged (`best[0]`).

### A.5.5 `arrowOrthoClip` (arrows.c:350–440) — orthogonal routing variant

Each Bézier is an H/V segment. Two cases:

* **Both arrows on one segment** (`sflag && eflag && endp == startp`, lines 355–391): `p = ps[endp]` (tail tip), `q = ps[endp+3]` (head tip), `tlen/hlen = arrow_length(e, sflag/eflag)`, `d = DIST(p,q)`; if `hlen + tlen >= d` then `hlen = tlen = d/3`. Then on the segment (horizontal: fixed y; vertical: fixed x) mark two points `s` (tlen in from p) and `t` (hlen back from q) and set `ps[endp]=ps[endp+1]=s`, `ps[endp+2]=ps[endp+3]=t`; `spl.sp = p`, `spl.ep = q`.
* **Single arrow** (lines 392–439): for the head: `hlen = min(arrow_length, 0.9*d)`; move `ps[endp+1] = p` (the old end) and `ps[endp+2] = ps[endp+3] = r` where `r` is `hlen` back along the segment from `q = ps[endp+3]`; `spl.ep = q`. Symmetric for the tail: `tlen = min(len, 0.9*d)`, `ps[startp] = ps[startp+1] = r` (tlen in from `p = ps[startp]`), `ps[startp+2] = q`; `spl.sp = p`.

### A.5.6 `arrow_clip` (splines.c:64–98) — dispatcher

```
arrow_clip(fe, hn, ps, &startp, &endp, spl, info):
  e = fe; while ED_to_orig(e): e = ED_to_orig(e)          # original edge
  j = info.ignoreSwap ? false : info.swapEnds(e)          # dot swaps ends for reversed/flat edges
  (sflag, eflag) = arrow_flags(e)
  if info.splineMerge(hn):        eflag = ARR_NONE       # arrow at a virtual/merge node is dropped
  if info.splineMerge(agtail(fe)): sflag = ARR_NONE
  if j: swap(sflag, eflag)
  if info.isOrtho: if (eflag||sflag) arrowOrthoClip(e, ps, startp, endp, spl, sflag, eflag)
  else:
      if sflag: startp = arrowStartClip(e, ps, startp, endp, spl, sflag)
      if eflag: endp   = arrowEndClip  (e, ps, startp, endp, spl, eflag)
```

**Interaction with `ED_tail_port` / `ED_head_port` and arrowsize.** `arrow_clip` itself does not move the port aiming points; the ports enter earlier and shape the raw spline:

* `common_init_edge` stores `ED_tail_port/ED_head_port` (aiming point `p` relative to node center, optional port rect `bp`, `clip` flag, compass-derived `theta`) — see §C.3. When the port has a rect (`bp`), `clip_and_install` (§A.5.7) walks the spline while its endpoint is inside the port box and then clips to the shape with `bp` as the target box, so a ported edge already stops at the port boundary before arrows are applied.
* `ED_tail_port.clip == false` (via `tailclip=false`/`headclip=false`, utils.c:546–555, 462–475) skips shape clipping entirely for that end (`clip_and_install` sets `start = 0` / `end = pn−4`); arrow clipping still runs.
* "Arrowsize adjustments" live in the length model: `arrow_length(e, flag)` reads `E_arrowsz` (default 1.0) and `E_penwidth` (default 1.0) and returns the pull-back distance per stacked arrow (§A.4); `arrow_gen` additionally rescales the unit arrow vector by `lenfact·arrowsize` per stacked component (§A.7.9).
* The flags that survive are stored on the first Bézier (`spl->sflag/spl->eflag/sp/ep`) and consumed by `arrow_gen` at emit time (emit.c:2039–2044 etc.), with `arrowsize = late_double(e, E_arrowsz, 1.0, 0.0)` (emit.c:2367).

### A.5.7 Surrounding context — `clip_and_install` (splines.c:233–313) and `shape_clip` (193–209)

For completeness, the order of end processing before arrows:

1. If `clipTail` (`ED_tail_port.clip`) and the tail node shape has an insidefn: scan forward over Béziers while `insidefn(ctx{n=tail, bp=tailportbox}, ps[start+3] − ND_coord(tn))` is true, advancing `start += 3`; then `shape_clip0(..., &ps[start], left_inside=true)`.
2. Symmetric for the head (`end` scans down from `pn-4`), `left_inside=false`.
3. Skip degenerate Béziers (`APPROXEQPT(ps[i], ps[i+3], MILLIPOINT)`) outward from both ends.
4. `arrow_clip(fe, hn, ps, &start, &end, newspl, info)`.
5. Copy `ps[start..end+4]` into `newspl`, updating the graph bbox with each cubic; `newspl.size = end - start + 4`.

`shape_clip(n, curve)` (193–209) translates the 4 control points into node coordinates (`c[i] = curve[i] − ND_coord(n)`), decides `left_inside = insidefn(c[0])`, calls `bezier_clip` with the shape's insidefn, and translates back. `shape_clip0` also saves/restores `ND_rw` (inside functions temporarily perturb it).

## A.6 `miter_shape` — SVG line-join model for penwidth-aware arrow tips (arrows.c:442–514)

Given a "corner" `P` and the two adjacent base points `base_left`, `base_right`, compute the 3-point stroke triangle `{P3 (or Pbevel), P1, P2}` for the miter/bevel join at `P`, given `penwidth`:

```
if base_left == P or base_right == P: return triangle {P,P,P}
A = base_left→P:   hypotA, cosAlpha = dxA/hypotA, sinAlpha = dyA/hypotA,
                   alpha = dyA > 0 ? acos(cosAlpha) : -acos(cosAlpha)
P1 = P − (penwidth/2)·(sinAlpha, −cosAlpha)        # = (P.x − pw/2·sinAlpha, P.y + pw/2·cosAlpha)
B = P→base_right: hypotB, cosBeta = dxB/hypotB, beta = dyB > 0 ? acos(cosBeta) : −acos(cosBeta)
beta_rev = beta − π
theta = beta_rev − alpha + (beta_rev − alpha <= −π ? 2π : 0)     # interior angle ∈ [0, π]
normalized_miter_length = 1 / sin(theta/2)                        # SVG stroke-miterlimit math
P2 = P + (penwidth/2)·(−sinBeta, cosBeta)                         # sinBeta = dyB/hypotB
if normalized_miter_length > 4.0:                                  # bevel fallback
    Pbevel = midpoint(P1, P2); return {Pbevel, P1, P2}
l = (penwidth/2) / tan(theta/2)
P3 = P1 + l·(cosAlpha, sinAlpha)
return {P3, P1, P2}
```

The triangle points are indexed: `.points[0]` = miter tip (P3), `.points[1]` = P1 (left offset), `.points[2]` = P2 (right offset).

## A.7 Geometry generators

All generators receive `p` (current attachment point on the spline, working **outward**), `u` (the arrow vector, already scaled by `lenfact*arrowsize`), `penwidth`, `flag`, and return the new `p` for the next stacked arrow (`q`). "Move the arrow backwards" deltas exist so the stroked shape does not visually overlap the node.

### A.7.1 `arrow_type_normal0` / `arrow_type_normal` (arrows.c:516–630)

```
arrowwidth = 0.35; if penwidth > 4: arrowwidth *= penwidth/4
v = perp(u) * arrowwidth          # v = (−u.y, u.x)·arrowwidth  (half-width)
q = p + u                          # nominal base point
origin = (0,0)
normal_left  = RIGHT ? origin : −v
normal_right = LEFT  ? origin : v
base_left  = INV ? normal_right : normal_left
base_right = INV ? normal_left  : normal_right
P = INV ? u : (−u)                 # tip point relative to p (INV points backwards)

delta_tip = (0,0)
if u ≠ 0:
    phi = angle of P: cosPhi = P.x/|P|, sinPhi = P.y/|P|, phi = P.y>0 ? acos(cosPhi) : −acos(cosPhi)
    if LEFT:   (L=P1 from miter_shape)  delta_tip = projection of P→P1 on direction P
    elif RIGHT:(P2 from miter_shape)    delta_tip = projection of P→P2 on direction P
    else:      (P3 = points[0])         delta_tip = P3 − P
    delta_base = (penwidth/2)·(cosPhi, sinPhi)

if INV:                             # arrow points back into the edge; base sits on spline
    p += delta_base; q += delta_base
    a[0]=a[4]=p; a[1]=p−v; a[2]=q; a[3]=p+v
    q += delta_tip                  # return value is beyond the base by tip extension
else:                               # normal: tip sits at p
    p −= delta_tip; q −= delta_tip  # pull arrow back so stroke doesn't cross node boundary
    a[0]=a[4]=q; a[1]=q−v; a[2]=p (tip); a[3]=q+v
    q −= delta_base
return q
```

Emission (`arrow_type_normal`, 613–630): polygon `a[1..3]` (3 points: the triangle), or `a[0..2]` for LEFT, `a[2..4]` for RIGHT; filled unless `ARR_MOD_OPEN`.

### A.7.2 `arrow_type_crow0` / `arrow_type_crow` (arrows.c:632–789)

```
arrowwidth = 0.45; if penwidth > 4*arrowsize and INV: arrowwidth *= penwidth/(4*arrowsize)
shaftwidth = 0;     if penwidth > 1 and INV: shaftwidth = 0.05*(penwidth−1)/arrowsize
v = perp(u)·arrowwidth ; w = perp(u)·shaftwidth
q = p + u ; m = p + u/2            # m = waist point
left/right base points use: normal_left  = RIGHT ? origin : v ; normal_right = LEFT ? origin : −v
     (note: opposite sign convention vs normal!)
P = INV ? −u : u                   # crow tip points outward
delta_tip: mirrored selection vs normal —
   (LEFT&&INV) || (RIGHT&&!INV) → use P2 projection
   (LEFT&&!INV) || (RIGHT&&INV) → use P1 projection
   else                          → P3 − P
if INV (vee):   delta_base = (penwidth/2)·(cosPhi,sinPhi)
else (crow):    toe-based delta: toe_base_left = (m−q)+w, toe_base_right = 0, toe_P = v−u;
                miter triangle → P1; delta_base = −(projection of toe_P→P1)  (negated!)
if INV:  p,q −= delta_tip
     a[0]=a[8]=p; a[1]=q−v; a[2]=m−w; a[3]=q−w; a[4]=q; a[5]=q+w; a[6]=m+w; a[7]=q+v
     q −= delta_base
else:    p,q += delta_base
     a[0]=a[8]=q; a[1]=p−v; a[2]=m−w; a[3]=p+delta_base; a[4]=p+delta_base; a[5]=p+delta_base;
     a[6]=m+w; a[7]=p+v
     q += delta_tip
```

Emission (774–789): LEFT → polygon `a[0..4]` (5 pts, filled=1 always); RIGHT → `a[4..8]`; else polygon `a[0..7]` (8 pts). Crow feet are always stroked, never "open".

### A.7.3 `arrow_type_gap` (`none` as spacer) (arrows.c:791–806)

`q = p+u`; polyline `p→q`; return `q`. Produces a blank segment of length `lenfact(0.5)·arrowsize·10`.

### A.7.4 `arrow_type_tee` (arrows.c:808–866)

```
v = perp(u)                       # full crossbar half-length = |u| (i.e. 10·arrowsize·0.5 after scaling)
q = p+u ; m = p+0.2u ; n = p+0.6u  # crossbar spans m..n
length = |u|
extend = penwidth/2 − 0.2·length
if length>0 and extend>0: shift p,m,n,q backwards by extend·(unit vector of −u)
a = [m+v, m−v, n−v, n+v]           # crossbar rectangle
if LEFT:  a[0]=m, a[3]=n           # left half only
if RIGHT: a[1]=m, a[2]=n
polygon(a, filled=1); polyline(p, q)   # shaft line
return q                            # polylines don't extend beyond endpoints; no extra delta
```

### A.7.5 `arrow_type_box` (arrows.c:868–924)

```
v = perp(u)·0.4 ; m = p+0.8u ; q = p+u
delta = (penwidth/2)·unit(−u)  if u≠0 else 0
p,m,q −= delta                       # shift back by half penwidth
a = [p+v, p−v, m−v, m+v]             # box rectangle from p to m
if LEFT:  a[0]=p, a[3]=m
if RIGHT: a[1]=p, a[2]=m
polygon(a, filled = !OPEN); polyline(m, q)   # shaft from box to nominal tip
return q
```

### A.7.6 `arrow_type_diamond0` / `arrow_type_diamond` (arrows.c:926–985)

```
v = perp(u)/3 ; r = p+u/2 ; q = p+u
unmod_left  = −u/2 − v ; unmod_right = −u/2 + v
base_left  = RIGHT ? 0 : unmod_left ; base_right = LEFT ? 0 : unmod_right
P = −u (tip at p)
delta = miter_shape(base_left, P, base_right, penwidth).points[0] − P
p,r,q −= delta                          # shift back by miter extension
a[0]=a[4]=q (base); a[1]=r+v; a[2]=p (tip); a[3]=r−v
return q − delta  (visual start)
```

Emission (968–985): LEFT → triangle `a[2..4]`; RIGHT → triangle `a[0..2]`; else quadrilateral `a[0..3]`; filled unless `OPEN`.

### A.7.7 `arrow_type_dot` (arrows.c:987–1025)

```
r = |u|/2
if u≠0: p −= (penwidth/2)·unit(−u)
corners = [p+u/2−r, p+u/2+r]            # opposite corners of the circle's bbox
ellipse(corners, filled = !OPEN)
return (p+u) − delta                     # visual start shifted back
```

### A.7.8 `arrow_type_curve` (arrows.c:1028–1089)

Concave semicircle drawn as one cubic; `arrowwidth = 0.5` (or `0.5·penwidth/4` if `penwidth > 4`).

```
a = [p, q=p+u]                            # shaft polyline
if !INV and u≠0: p −= (penwidth/2)·unit(−u)     # shift back (only for the non-inverted form)
q = p+u ; v = perp(u)·arrowwidth ; w = (v.y, −v.x)   # w points along u with |v|
AF[0] = p + v + w ; AF[3] = p − v + w
if INV:  AF[1] = (p.x + 0.95v.x + w.x + (4/3)w.x, AF[0].y + (4/3)w.y)
         AF[2] = (p.x − 0.95v.x + w.x + (4/3)w.x, AF[3].y + (4/3)w.y)
else:    AF[1] = (p.x + 0.95v.x + w.x − (4/3)w.x, AF[0].y − (4/3)w.y)
         AF[2] = (p.x − 0.95v.x + w.x − (4/3)w.x, AF[3].y − (4/3)w.y)
polyline(a)                              # shaft
if LEFT:  AF = Bezier(AF, 0.5, NULL, AF)  # keep right half (first half discarded)
if RIGHT: AF = Bezier(AF, 0.5, AF, NULL)  # keep left half
beziercurve(AF, 4)
return q
```

### A.7.9 Dispatch — `arrow_gen_type` (1092–1105) and `arrow_gen` (1145–1180)

```
arrow_gen_type(p, u, arrowsize, penwidth, flag):
  f = flag & 0xF
  for t in Arrowtypes: if t.type == f:
      u = u * (t.lenfact * arrowsize)      # scale BOTH components
      p = t.gen(p, u, arrowsize, penwidth, flag); break
  return p

arrow_gen(job, emit_state, p, u, arrowsize, penwidth, flag):
  obj.emit_state = emit_state
  gvrender_set_style(job, defaultlinestyle)     # dotted/dashed styles must not leak into the head
  gvrender_set_penwidth(job, penwidth)
  u = u − p
  s = ARROW_LENGTH / (hypot(u) + EPSILON)       # normalize to exactly 10 points
  u += ((u >= 0) ? EPSILON : −EPSILON) per component (before scaling; keeps zero-length stable)
  u *= s
  for i in 0..4:
      f = (flag >> 8i) & 0xFF
      if f == ARR_NONE: break                   # stop at first empty slot
      p = arrow_gen_type(p, u, arrowsize, penwidth, f)
  restore emit_state
```

Callers (emit.c:2039–2044, 2436–2439, 2516–2527, 2669–2673):

```
if bz.sflag: arrow_gen(job, EMIT_TDRAW, bz.sp,   bz.list[0],            arrowsize, penwidth, bz.sflag)
if bz.eflag: arrow_gen(job, EMIT_HDRAW, bz.ep,   bz.list[bz.size−1],    arrowsize, penwidth, bz.eflag)
```
with `arrowsize = late_double(e, E_arrowsz, 1.0, 0.0)` (emit.c:2367) and `penwidth = job->obj->penwidth`.

### A.7.10 `arrow_bb` (arrows.c:1107–1143) — conservative arrow bounding box

Used only for overlap tests (utils.c:1319–1323, emit.c:4116):

```
u = u − p ; s = ARROW_LENGTH*arrowsize / (hypot(u) + EPSILON)
u += ±EPSILON per component; u *= s
half = u/2
corners: a = p−(half.y, half.x)? — precisely:
  ax = p.x − u.y/2 ; ay = p.y − u.x/2
  bx = p.x + u.y/2 ; by = p.y + u.x/2
  c = a+u ; d = b+u
bb = bbox(a,b,c,d)
```

---

# Part B — `shapes.c`

## B.1 Base polygon descriptors (shapes.c:96–203)

`polygon_t` fields: `regular, peripheries, sides, orientation (deg), distortion, skew, option, vertices-desc`.

| Shape | regular | periph | sides | orient | distortion | skew | option |
|---|---|---|---|---|---|---|---|
| `polygon` (p_polygon) | – | 1 | 0 (user-set) | 0 | 0 | 0 | – |
| `ellipse`, `oval` | – | 1 | 1 | – | – | – | – |
| `circle` | ✓ | 1 | 1 | – | – | – | – |
| `point` | uses `p_circle` but `point_fns` | | | | | | |
| `egg` | – | 1 | 1 | – | **−0.3** | – | – |
| `triangle` | – | 1 | 3 | – | – | – | – |
| `box`/`rect`/`rectangle` | – | 1 | 4 | – | – | – | – |
| `square` | ✓ | 1 | 4 | – | – | – | – |
| `plaintext` / `none` (p_plaintext) | – | **0** | 4 | – | – | – | – |
| `plain` (p_plain) | – | **0** | 4 | – | – | – | – |
| `diamond` | – | 1 | 4 | **45.0** | – | – | – |
| `trapezium` | – | 1 | 4 | – | **−0.4** | – | – |
| `parallelogram` | – | 1 | 4 | – | – | **0.6** | – |
| `house` | – | 1 | 5 | – | **−0.64** | – | – |
| `pentagon` | – | 1 | 5 | | | | |
| `hexagon` | – | 1 | 6 | | | | |
| `septagon` | – | 1 | 7 | | | | |
| `octagon` | – | 1 | 8 | | | | |
| `note` | – | 1 | 4 | | | | `shape=DOGEAR` |
| `tab` | – | 1 | 4 | | | | `shape=TAB` |
| `folder` | – | 1 | 4 | | | | `shape=FOLDER` |
| `box3d` | – | 1 | 4 | | | | `shape=BOX3D` |
| `component` | – | 1 | 4 | | | | `shape=COMPONENT` |
| `underline` | – | 1 | 4 | | | | `underline=true` |
| `cylinder` | – | 1 | **19** | | | | `shape=CYLINDER`, `vertices=cylinder_gen` |
| `doublecircle` | ✓ | **2** | 1 | | | | |
| `invtriangle` | – | 1 | 3 | **180.0** | | | |
| `invtrapezium` | – | 1 | 4 | 180.0 | −0.4 | | |
| `invhouse` | – | 1 | 5 | 180.0 | −0.64 | | |
| `doubleoctagon` | – | **2** | 8 | | | | |
| `tripleoctagon` | – | **3** | 8 | | | | |
| `Mdiamond` | – | 1 | 4 | 45.0 | | | `diagonals=true, auxlabels=true` |
| `Msquare` | ✓ | 1 | 4 | | | | `diagonals=true` |
| `Mcircle` | ✓ | 1 | 1 | | | | `diagonals=true, auxlabels=true` |
| `star` | – | 1 | **10** | | | | `vertices=star_gen` |
| SBOLv: `promoter, cds, terminator, utr, insulator, ribosite, rnastab, proteasesite, proteinstab, primersite, restrictionsite, fivepoverhang, threepoverhang, noverhang, assembly, signature, rpromoter, larrow, rarrow, lpromoter` | | 1 | 4 | | | | `shape = PROMOTER..LPROMOTER` respectively |

`p_*` structs with unlisted fields zero/false (shapes.c:96–203). `sides = 1` means "ellipse" (≤2); `sides = 0` means fully user-controlled (`shape=polygon` with `sides/distortion/skew` attributes).

## B.2 Function tables and the shape registry (shapes.c:243–360)

```
poly_fns     = { poly_init,    poly_free,    poly_port,    poly_inside,  poly_path,    poly_gencode }
point_fns    = { point_init,   poly_free,    poly_port,    point_inside, NULL,         point_gencode }
record_fns   = { record_init,  record_free,  record_port,  record_inside,record_path,  record_gencode }
epsf_fns     = { epsf_init,    epsf_free,    poly_port,    epsf_inside,  NULL,         epsf_gencode }
star_fns     = { poly_init,    poly_free,    poly_port,    star_inside,  poly_path,    poly_gencode }
cylinder_fns = { poly_init,    poly_free,    poly_port,    poly_inside,  poly_path,    poly_gencode }
```

`Shapes[]` (shapes.c:292–360), **first entry is the fallback for unknown names**:

```
box, polygon, ellipse, oval, circle, point, egg, triangle, none, plaintext, plain,
diamond, trapezium, parallelogram, house, pentagon, hexagon, septagon, octagon,
note, tab, folder, box3d, component, cylinder, rect, rectangle, square,
doublecircle, doubleoctagon, tripleoctagon, invtriangle, invtrapezium, invhouse,
underline, Mdiamond, Msquare, Mcircle,
promoter, cds, terminator, utr, insulator, ribosite, rnastab, proteasesite, proteinstab,
primersite, restrictionsite, fivepoverhang, threepoverhang, noverhang, assembly,
signature, rpromoter, larrow, rarrow, lpromoter,
record, Mrecord, epsf, star, (terminator)
```

`bind_shape(name, n)` (3970–3990): if `shapefile` attr set and name ≠ "epsf" → name = "custom"; look up in `Shapes`; otherwise create/return a user shape (`user_shape`, 3949–3968: copy of `Shapes[0]`, `usershape = true` unless no lib and name ≠ "custom", with warning "using box for unknown shape"). `shapeOf` (1908–1926) maps initfn → `SH_POLY | SH_RECORD | SH_POINT | SH_EPSF | SH_UNSET`. `isPolygon` (1928–1931) = initfn == poly_init. `polyBB(poly)` (utils.c:592–608) = bbox of the **outermost periphery** ring `vertices[(max(peripheries,1)−1)*sides .. +sides)`.

## B.3 Style handling (shapes.c:422–545)

* `isBox(n)` (422–430): base polygon has `sides == 4 && |fmod(orientation,90)| < 0.5 && distortion == 0 && skew == 0`.
* `isEllipse(n)` (432–439): `sides <= 2`.
* `checkStyle(n, &istyle)` (465–529): parse `style` attribute tokens (`parse_style`): `filled`, `rounded`, `diagonals`, `invis`, `radial` (implies filled), `striped` (only if `isBox`), `wedged` (only if `isEllipse`); `rounded`, `diagonals`, `radial`, `striped`, `wedged` are **removed** from the string list passed on to the renderer. Then `istyle = style_or(istyle, poly->option)` (442–463; bitwise OR of each flag; `shape` ORed, asserted to have at most one side non-zero).
* `stylenode(job, n)` (531–545): `checkStyle`, apply the remaining style list to the renderer, and if `penwidth` attr set, `gvrender_set_penwidth(late_double(n, N_penwidth, 1.0, 0.0))`.
* `SPECIAL_CORNERS(style)` (214–216): `style.rounded || style.diagonals || style.shape != 0` — such shapes must go through `round_corners`.
* `penColor` (389–398): `color` attr or `DEFAULT_COLOR` ("black").
* `findFill(n)` (401–420): `fillcolor` attr, else `color` attr (backward compat), else `DEFAULT_FILL` ("lightgrey").

## B.4 `poly_init` — full sizing algorithm (shapes.c:1933–2383)

Inputs: node attributes, `ND_label->dimen` (already computed from the label), base polygon descriptor. Outputs: `polygon_t` in `ND_shape_info`, `ND_width/height` (inches), `ND_outline_width/height` (inches, penwidth included), `ND_label->space`.

```
poly = alloc; base = ND_shape->polygon
isPlain  = base == p_plain (or p_plaintext)      # IS_PLAIN, shapes.c:209-211
regular  = base.regular | mapbool(attr "regular")
peripheries = base.peripheries; sides = base.sides
orientation = base.orientation; skew = base.skew; distortion = base.distortion

# --- width/height in points ---
if isPlain: width = height = 0
elif regular:
    sz = INCH2PS(max(attr width, attr height))            # userSize, 1900-1906; 0 if neither set
    if sz > 0: width = height = sz
    else:      width = height = INCH2PS(min(ND_width, ND_height))
else: width = INCH2PS(ND_width); height = INCH2PS(ND_height)

peripheries = late_int(N_peripheries, peripheries, min 0)
orientation += late_double(N_orientation, 0.0, min −360)
if sides == 0:                       # generic "polygon" shape: take attrs
    skew       = late_double(N_skew,       0.0, min −100)
    sides      = late_int  (N_sides,       4,   min 0)
    distortion = late_double(N_distortion, 0.0, min −100)

dimen = ND_label->dimen              # points

# --- margin / padding ---
if (dimen.x > 0 or dimen.y > 0) and !isPlain:
    if attr "margin" parses "%lf,%lf" (1 or 2 values, clamped ≥ 0):
        dimen.x += 2*INCH2PS(marginx)
        dimen.y += 2*INCH2PS(i > 1 ? marginy : marginx)
    else: PAD(dimen)                  # +16 x, +8 y
spacex = dimen.x − ND_label->dimen.x  # total margin width added

# --- quantum ---
if graph quantum > 0: dimen = (quant(dimen.x, q), quant(dimen.y, q)) with q = INCH2PS(quantum)

# --- images ---
imagesize = (0,0)
if shape is "custom" (usershape) with "shapefile", or attr "image" nonempty:
    imagesize = gvusershape_size(...) ; on error → 0 + warning
    else: imagesize += (2,2) ; GD_has_images = true

bb = (max(dimen.x, imagesize.x), max(dimen.y, imagesize.y))

if sides <= 2 and (distortion ≠ 0 or skew ≠ 0): sides = 120     # approximate distorted ellipse

# labelloc
ND_label->valign = attr labelloc in {'t','b'} ? that : 'c'

isBox_local = sides==4 and |fmod(orientation,90)| < 0.5 and distortion==0 and skew==0

if isBox_local: pass                          # exact fit
elif base.vertices:                           # custom vertex generator (star, cylinder)
    bb = size_gen(bb)
else:                                         # smallest ellipse containing bb, then pad
    temp = bb.y * SQRT2
    if height > temp and valign == 'c':
        bb.x *= sqrt(1 / (1 − (bb.y/height)²))
    else:
        bb.x *= SQRT2 ; bb.y = temp
    if sides > 2:
        temp = cos(π/sides) ; bb.x /= temp ; bb.y /= temp
min_bb = bb

# --- honor width/height / fixed ---
fxd = late_string(N_fixed, "false")
if fxd == "shape":  bb = (width,height); poly.option.fixedshape = true
elif mapbool(fxd):  if width < label.dimen.x or height < label.dimen.y: warn
                    bb = (width,height)
else:               bb.x = width  = max(width,  bb.x)
                    bb.y = height = max(height, bb.y)
if regular: width = height = bb.x = bb.y = max(bb.x, bb.y)

# --- label justification space ---
if !mapbool(attr nojustify):
    if isBox_local: ND_label->space.x = max(dimen.x, bb.x) − spacex
    elif dimen.y < bb.y:
        temp = bb.x * sqrt(1 − (dimen.y/bb.y)²)
        ND_label->space.x = max(dimen.x, temp) − spacex
    else: ND_label->space.x = dimen.x − spacex
else: ND_label->space.x = dimen.x − spacex
if !fixedshape:
    temp = bb.y − min_bb.y
    if dimen.y < imagesize.y: temp += imagesize.y − dimen.y
    ND_label->space.y = dimen.y + temp

penwidth = late_double(N_penwidth, 1.0, 0.0)
outp = max(peripheries, 1)
if peripheries >= 1 and penwidth > 0: outp += 1        # extra ring = outer stroke outline

# ================= ellipse branch (sides < 3) =================
if sides < 3:
    sides = 2
    vertices = alloc(outp*2)
    P = (bb.x/2, bb.y/2)
    vertices[0] = (−P.x, −P.y) ; vertices[1] = P
    if peripheries > 1:
        for j in 1..peripheries:                       # GAP = 4 pt per ring
            P += (GAP, GAP)
            vertices[2j] = −P ; vertices[2j+1] = P
        bb = 2P
    outline_bb = bb
    if outp > peripheries:                             # outline ring with penwidth
        P += (penwidth/2, penwidth/2)
        vertices[2*peripheries] = −P ; vertices[2*peripheries+1] = P
        outline_bb = 2P

# ================= polygon branch (sides >= 3) =================
else:
    vertices = alloc(outp*sides)
    if base.vertices:                                  # star/cylinder generators (§B.10/§B.11)
        vertex_gen(vertices, &bb); xmax = bb.x/2; ymax = bb.y/2
    else:
        sectorangle = 2π/sides
        sidelength  = sin(sectorangle/2)
        skewdist    = hypot(|distortion| + |skew|, 1)
        gdistortion = distortion * SQRT2 / cos(sectorangle/2)
        gskew       = skew / 2
        angle = (sectorangle − π)/2
        R = (0.5·cos(angle), 0.5·sin(angle))
        angle += (π − sectorangle)/2
        for i in 0..sides:
            angle += sectorangle
            R += sidelength·(cos(angle), sin(angle))        # next regular-polygon vertex (circumradius .5)
            P.x = R.x*(skewdist + R.y·gdistortion) + R.y·gskew   # distort + skew
            P.y = R.y
            alpha = RADIANS(orientation) + atan2(P.y, P.x)       # orient
            P.x = P.y = hypot(P.x, P.y)                          # then
            P.x *= cos(alpha) ; P.y *= sin(alpha)                # rotate
            P.x *= bb.x ; P.y *= bb.y                            # scale to label box
            xmax = max(|P.x|, xmax) ; ymax = max(|P.y|, ymax)
            vertices[i] = P
            if isBox_local:                        # force exact box symmetry; stop after first vertex
                vertices[1] = (−P.x,  P.y)
                vertices[2] = (−P.x, −P.y)
                vertices[3] = ( P.x, −P.y)
                break
    xmax *= 2 ; ymax *= 2
    bb = (max(width, xmax), max(height, ymax))
    outline_bb = bb
    scalex = bb.x/xmax ; scaley = bb.y/ymax        # stretch to exactly bb
    for i in 0..sides: vertices[i] *= (scalex, scaley)

    if outp > 1:
        # offset each vertex along its angle bisector by GAP (and penwidth/2 for outline)
        R = vertices[0]
        Q = first vertex scanning j = 1.. backwards from index sides−1 that differs from R
        beta = atan2(R.y−Q.y, R.x−Q.x)
        for i in 0..sides:
            Q = vertices[i]
            if Q == Qprev:                    # degenerate (zero-length) side — e.g. cylinder Bézier doubles;
                                              # reuse previous (cosx,sinx) offset
                pass
            else:
                R = first vertex forward from i that differs from Q
                alpha = beta ; beta = atan2(R.y−Q.y, R.x−Q.x)
                gamma = (alpha + π − beta)/2
                temp  = GAP / sin(gamma)
                sinx  = sin(alpha − gamma)·temp
                cosx  = cos(alpha − gamma)·temp
            Qprev = Q
            for j in 1..peripheries:
                Q += (cosx, sinx) ; vertices[i + j*sides] = Q
            if outp > peripheries:
                Q += (cosx, sinx)·(penwidth/2/GAP)      # penwidth/2 outward along bisector
                vertices[i + peripheries*sides] = Q
        # final bbs from outermost rings
        for i in 0..sides:
            P = vertices[i + (peripheries−1)*sides]
            bb = (max(2|P.x|, bb.x), max(2|P.y|, bb.y))
            Q = vertices[i + (outp−1)*sides]
            outline_bb = (max(2|Q.x|, outline_bb.x), max(2|Q.y|, outline_bb.y))

# --- store ---
poly.{regular,peripheries,sides,orientation,skew,distortion,vertices} = ...
if fixedshape:
    ND_width  = PS2INCH(max(dimen.x, bb.x)) ; ND_height = PS2INCH(max(dimen.y, bb.y))
    ND_outline_width  = PS2INCH(max(dimen.x, outline_bb.x)) ; ND_outline_height = PS2INCH(max(dimen.y, outline_bb.y))
else:
    ND_width = PS2INCH(bb.x) ; ND_height = PS2INCH(bb.y)
    ND_outline_width = PS2INCH(outline_bb.x) ; ND_outline_height = PS2INCH(outline_bb.y)
ND_shape_info = poly
```

Vertex order for an axis-aligned box (orientation 0): `vertices[0] = (+x,+y)` (top-right in math coords), `1 = (−x,+y)`, `2 = (−x,−y)`, `3 = (+x,−y)` — i.e. CCW in math orientation (clockwise on screen).

## B.5 `poly_free` (shapes.c:2385–2393)

Free `polygon_t.vertices` then the struct. (Also used as the freefn of `point_fns`.)

## B.6 `poly_inside` — hit testing (shapes.c:2395–2535)

```
poly_inside(ictxt, p):
  if !ictxt: return false
  n = ictxt.s.n ; bp = ictxt.s.bp
  P = ccwrotatepf(p, 90·rankdir)                # to node (unrotated) frame
  if bp: return INSIDE(P, *bp)                  # port-box fast path

  if n != ictxt.s.lastn:                        # (re)build cache
      poly = ictxt.s.last_poly = ND_shape_info(n)
      sides = poly.sides
      if poly.option.fixedshape:
          bb = polyBB(poly); n_width = bb.UR.x−bb.LL.x; n_height = bb.UR.y−bb.LL.y
          n_outline_w = n_width; n_outline_h = n_height
          (xsize,ysize) = flip ? (n_height,n_width) : (n_width,n_height)
      else:
          xsize = flip ? ND_lw+ND_rw (same) ; ysize = ND_ht    # flip ⇒ swap roles:
                 # flip: ysize = lw+rw, xsize = ht ; else xsize = lw+rw, ysize = ht
          n_width  = INCH2PS(ND_width);  n_height = INCH2PS(ND_height)
          n_outline_w = INCH2PS(ND_outline_width); n_outline_h = INCH2PS(ND_outline_height)
      ictxt.scalex = n_width  / (xsize ≠ 0 ? xsize : 1)
      ictxt.scaley = n_height / (ysize ≠ 0 ? ysize : 1)
      ictxt.box_URx = n_outline_w/2 ; ictxt.box_URy = n_outline_h/2
      penwidth = late_double(N_penwidth, 1.0, 0.0)
      if poly.peripheries >= 1 and penwidth > 0: outp = peripheries*sides   # penwidth outline ring
      elif poly.peripheries < 1:                 outp = 0                                  # innermost ring
      else:                                      outp = (peripheries−1)*sides              # outer ring
      ictxt.s.outp = outp ; ictxt.s.lastn = n

  P.x *= scalex ; P.y *= scaley
  if |P.x| > box_URx or |P.y| > box_URy: return false         # outline bbox rejection

  if sides <= 2:                                               # ellipse test (normalized)
      return hypot(P.x/box_URx, P.y/box_URy) < 1

  # polygon test with cached segment ("last") fast path
  i  = ictxt.s.last % sides ; i1 = (i+1) % sides
  Q = vertex[outp + i] ; R = vertex[outp + i1]
  if !same_side(P, O=(0,0), Q, R): return false                # outside this face ⇒ out
  s = same_side(P, Q, R, O) and same_side(P, R, O, Q)          # between the side rays?
  if s: return true
  for j in 1..sides:                                           # walk remaining faces, direction by s
      if s: i = i1 ; i1 = (i+1)%sides                          # clockwise
      else: i1 = i  ; i  = (i+sides−1)%sides                   # counter-clockwise
      if !same_side(P, O, vertex[outp+i], vertex[outp+i1]):
          ictxt.s.last = i ; return false
  ictxt.s.last = i
  return true
```

`same_side(p0, p1, L0, L1)` (shapes.c:373–386): line `a·x + b·y = c` with `a = −(L1.y−L0.y)`, `b = L1.x−L0.x`, `c = a·L0.x + b·L0.y`; true iff both `p0`, `p1` give `a·x+b·y−c >= 0` or both `< 0` (test is `>= 0` boolean equality).

## B.7 Ports

### B.7.1 `poly_port` (shapes.c:2880–2915)

```
poly_port(n, portname, compass):
  if portname == "": return Center
  compass = compass ?: "_"
  sides = BOTTOM|RIGHT|TOP|LEFT
  if ND_label->html and (bp = html_port(n, portname, &sides)):     # htmltable.c:918-930
      if compassPort(n, bp, &rv, compass, sides, NULL): warn unrecognized compass
  else:
      ictxtp = IS_BOX(n) ? NULL : {s.n = n, s.bp = NULL}    # boxes: compass points are trivial
      if compassPort(n, NULL, &rv, portname, sides, ictxtp): unrecognized(n, portname)
      # NOTE: for non-html ports the *portname* itself is interpreted as the compass string
  rv.name = NULL
```

### B.7.2 `compassPort` (shapes.c:2674–2878) — the core port finalizer

```
compassPort(n, bp, pp, compass, sides, ictxt) -> rv (1 = unrecognized, use center):
  if bp:  b = *bp ; p = center(b) ; defined = true
  else:
      p = (0,0) ; defined = false
      if GD_flip: b = [−ht/2, ht/2] × [−lw, lw]  (x = ±ht/2, y = ±lw)
      else:       b = x ∈ [−lw, lw], y ∈ [−ht/2, ht/2]
  maxv = 4·max(b.UR.x, b.UR.y)          # "sufficiently far outside"
  ctr = p ; theta = 0 ; constrain = false ; dyna = false ; side = 0 ; clip = true

  switch compass[0] (with compass++ consumed):
   'e': if more chars → rv=1 else:
        p = ictxt ? compassPoint(ictxt, ctr.y, maxv) : (b.UR.x, p.y)
        theta = 0 ; constrain = true ; clip = false ; defined = true ; side = sides & RIGHT
   'w': same with −maxv → theta = π ; side = sides & LEFT
   's': p.y = b.LL.y ; constrain = true ; clip = false
        sub: '' → theta=−π/2 ; p = compassPoint(ictxt,−maxv, ctr.x) or (ctr.x, b.LL.y); side = BOTTOM
             'e'→ theta=−π/4 ; p = compassPoint(−maxv, +maxv) or (b.UR.x, b.LL.y); side = BOTTOM|RIGHT
             'w'→ theta=−3π/4 ; p = compassPoint(−maxv, −maxv) or (b.LL.x, b.LL.y); side = BOTTOM|LEFT
             else → p.y = ctr.y ; constrain=false ; clip=true ; rv=1
   'n': p.y = b.UR.y ; constrain = true ; clip = false
        sub: '' → theta=π/2 ; side = TOP
             'e'→ theta=π/4 ; side = TOP|RIGHT
             'w'→ theta=3π/4 ; side = TOP|LEFT
             else → rv = 1 (as 's')
   '_': dyna = true ; side = sides
   'c': nothing (center)
   default: rv = 1

  p = cwrotatepf(p, 90·rankdir)             # back to layout frame
  pp.side = dyna ? side : invflip_side(side, rankdir)
  pp.bp = bp ; pp.p = p ; pp.theta = invflip_angle(theta, rankdir)
  if p == (0,0): pp.order = MC_SCALE/2
  else: angle = atan2(p.y, p.x) + 1.5π ; if angle ≥ 2π: angle −= 2π
        pp.order = MC_SCALE·angle/(2π)
  pp.constrained / defined / clip / dyna = ...
```

`invflip_side(side, rankdir)` (2548–2604): TB: identity; BT: swap TOP/BOTTOM; LR: TOP→RIGHT, BOTTOM→LEFT, LEFT→TOP, RIGHT→BOTTOM; RL: TOP→RIGHT, BOTTOM→LEFT, LEFT→BOTTOM, RIGHT→TOP.

`invflip_angle(a, rankdir)` (2606–2635): TB: a; BT: −a; LR: a − π/2; RL: exact remap of the 8 canonical angles: π→−π/2, 3π/4→−π/4, π/2→0, 0→π/2, −π/4→3π/4, −π/2→π.

`compassPoint(ictxt, y, x)` (2637–2672): ray-cast from node center to the shape boundary:

```
p = (x, y); if rankdir: p = cwrotatepf(p, 90·rankdir)
curve = [ (0,0), (0,0), p, p ]                 # degenerate straight "Bézier"
bezier_clip(ictxt, shape.insidefn, curve, left_inside = true)
if rankdir: curve[0] = ccwrotatepf(curve[0], 90·rankdir)
return curve[0]
```

### B.7.3 Dynamic ports — `resolvePorts`, `resolvePort`, `closestSide`, `cvtPt` (shapes.c:4221–4342)

```
resolvePorts(e): if ED_tail_port.dyna: ED_tail_port = resolvePort(tail, head, &ED_tail_port)
                 if ED_head_port.dyna: ED_head_port = resolvePort(head, tail, &ED_head_port)

cvtPt(p, rankdir):  TB → p ; BT → (p.x, −p.y) ; LR → (−p.y, p.x) ; RL → (p.y, p.x)

closestSide(n, other, oldport) -> "s"|"e"|"n"|"w"|NULL:
  sides = oldport.side
  if sides == 0 or sides == ALL: return NULL (use center)
  b = oldport.bp ?: node bbox (flip-adjusted as in compassPort)
  for each side i present in sides (BOTTOM_IX..LEFT_IX):
      p = midpoint of that side of b (+ node coord in rankdir-converted space, via cvtPt)
      d = DIST2(p, cvtPt(ND_coord(other)))
      keep min → side_port[i] = {"s","e","n","w"}[i]

resolvePort(n, other, oldport):
  compass = closestSide(n, other, oldport)
  rv.name = oldport.name
  compassPort(n, oldport.bp, &rv, compass, oldport.side, NULL)
```

### B.7.4 `record_port` (shapes.c:3732–3756)

```
if portname == "": return Center
sides = ALL ; compass = compass ?: "_"
f = ND_shape_info
if subf = map_rec_port(f, portname)          # depth-first search for field with id == portname
     compassPort(n, &subf.b, &rv, compass, subf.sides, NULL)  (warn on bad compass)
else compassPort(n, &f.b, &rv, portname, sides, NULL)         # portname treated as compass
```

## B.8 `poly_gencode` — emitting the shape (shapes.c:2917–3094)

```
poly = ND_shape_info ; vertices = poly.vertices ; sides = poly.sides ; peripheries = poly.peripheries
AF = alloc(sides+5)
ND_label->pos = ND_coord(n)                            # nominal label position = center
xsize = (ND_lw+ND_rw)/INCH2PS(ND_width)                # layout growth scale factors
ysize = ND_ht(n)/INCH2PS(ND_height)
style = stylenode(job, n)
# GUI states (ACTIVE/SELECTED/DELETED/VISITED) override colors, filled = FILL
# else: style.filled → findStopColor(fillcolor) may make GRADIENT/RGRADIENT with gradientangle;
#       striped/wedged → filled = 1 ; else filled = 0 ; pencolor = penColor(n)
pfilled = !shape.usershape or name == "custom"
if peripheries == 0 and filled ≠ 0 and pfilled:
    peripheries = 1 ; pencolor = "transparent"          # fill-only shape: stroke invisible

for j in 0..peripheries:                                # rings, innermost first
    for i in 0..sides:
        P = vertices[i + j*sides]
        AF[i] = (P.x*xsize + coord.x, P.y*ysize + coord.y)
    if sides <= 2:
        if style.wedged and j == 0 and fillcolor has ':': wedgedEllipse(job, AF, fillcolor); filled = 0
        ellipse(AF, filled)                             # AF[0..1] = opposite bbox corners
        if style.diagonals: Mcircle_hack(job, n)
    elif style.striped:
        if j == 0: stripedBox(job, AF, fillcolor, rotate=1)   # emit.c:595
        polygon(AF, sides, filled=0)
    elif style.underline:
        pencolor = "transparent"; polygon(AF, sides, filled); pencolor = saved
        polyline(AF+2, 2)                               # bottom edge (BL→BR)
    elif SPECIAL_CORNERS(style): round_corners(job, AF, sides, style, filled)
    else: polygon(AF, sides, filled)
    filled = 0                                          # fill innermost ring only

# user shapes: redraw innermost ring coords via gvrender_usershape(name, AF, sides, ...)
emit_label(job, EMIT_NLABEL, ND_label(n))
# anchor (URL/tooltip) begin/end around the above per EMIT_CLUSTERS_LAST flag
```

`Mcircle_hack` (547–564) — the `Mcircle` cross: `y = 0.75, x = 0.6614` (x²+y² = 1); `p = (ND_rw·x, ND_ht/2·y)`; two horizontal polylines at `±p.y` spanning `±p.x` around the center.

## B.9 `round_corners` machinery (shapes.c:566–1892)

### B.9.1 `alloc_interpolation_points` (566–617)

```
RBCONST = 12 ; RBCURVE = 0.5
rbconst = RBCONST
for each edge (AF[seg] → AF[seg+1 mod sides]): rbconst = min(rbconst, edge_length/3)
B = alloc(4·sides + 4)
for seg in 0..sides:
    d = edge length ; t = rbconst/d
    if style.shape in {BOX3D, COMPONENT}: t /= 3
    elif style.shape == DOGEAR:           t /= 2
    if !rounded: B[i++] = p0                              # corner itself
    else:        B[i++] = interpolate(RBCURVE·t, p0, p1)  # 0.5t (corner approach)
    B[i++] = interpolate(t, p0, p1)                       # edge entry
    B[i++] = interpolate(1−t, p0, p1)                     # edge exit
    if rounded:  B[i++] = interpolate(1−RBCURVE·t, p0, p1)# 0.5t before next corner
B[4·sides] = B[0] ; B[+1] = B[1] ; B[+2] = B[2]           # wraparound
```

So per edge, non-rounded layout is `B[3s] = corner s`, `B[3s+1] = t`, `B[3s+2] = 1−t` (plus the three wrap entries); rounded layout is `B[4s..4s+3]` as above.

### B.9.2 `diagonals_draw` (624–635) — Mdiamond/Msquare style

```
B = alloc_interpolation_points(AF, sides, style, rounded=false)
polygon(AF, sides, filled)
for seg in 0..sides: polyline(B[3·seg+2], B[3·seg+4])   # 1−t on edge seg → t on edge seg+1
                                                        # (i.e. a chord cutting corner seg+1)
```

### B.9.3 `rounded_draw` (643–663)

```
B = alloc_interpolation_points(..., rounded=true)
pts = alloc(6·sides+2)
for seg in 0..sides:
    pts += [B[4s], B[4s+1], B[4s+1], B[4s+2], B[4s+2], B[4s+3]]
pts += [pts[0], pts[1]]
beziercurve(pts+1, 6·sides+1, filled)     # renderer consumes (n−1)/3 = 2·sides cubics:
                                          # corner arc = cubic with both controls at the 0.5t points;
                                          # straight run = degenerate cubic between edge points
```

### B.9.4 `round_corners` dispatch (709–737) and special shapes

```
round_corners(job, AF, sides, style, filled):
  if style.diagonals: diagonals_draw; return
  if style.shape != 0: mode.shape = style.shape
  elif style.rounded:  rounded_draw; return
  else: UNREACHABLE
  if mode.shape == CYLINDER: cylinder_draw(job, AF, sides, filled); return
  B = alloc_interpolation_points(AF, sides, style, rounded=false)
  switch (mode.shape): ... (below)
```

Non-SBOLv special shapes (DOGEAR/TAB/FOLDER/BOX3D/COMPONENT), verbatim index semantics (with non-rounded `B`):

* **DOGEAR** (`note`, 740–759): polygon `D` of `sides+1` pts: `D[0] = B[3(sides−1)+4]` (= wrap `B[1]`, i.e. point `t` on edge 0 measured from corner 0), `D[1..sides−1] = AF[1..]`, `D[sides] = B[3(sides−1)+2]` (point `1−t` on the last edge) — corner 0 chamfered. Inner fold lines: `C[0] = B[3(sides−1)+2]`, `C[1] = B[3(sides−1)+4]`, `C[2] = C[1] + (C[0] − B[3(sides−1)+3])` (with `B[3(sides−1)+3]` = wrap `B[0]` = corner 0); polyline `C[1]→C[2]` then `C[0]→C[2]`.
* **TAB** (`tab`, 760–792): polygon `D` of `sides+2`: `D[0]=AF[0]`, `D[1]=B[2]` (1−t on edge 0), `D[2] = B[2] + (B[3]−B[4])/3`, `D[3] = B[3] + (B[3]−B[4])/3` (with `B[3]` = corner 1 = AF[1], `B[4]` = t-point on edge 1), then `D[4..] = AF[2..]`. Inner line `B[3]→B[2]`.
* **FOLDER** (793–822): polygon `D` of `sides+3`: `D[0]=AF[0]`; `D[1] = (AF[0].x − (AF[0].x−B[1].x)/4, AF[0].y + (B[3].y−B[4].y)/3)`; `D[2] = (AF[0].x − 2(AF[0].x−B[1].x), D[1].y)`; `D[3] = (AF[0].x − 2.25(AF[0].x−B[1].x), B[3].y)`; `D[4] = B[3]`; `D[5..] = AF[2..]`.
* **BOX3D** (823–845; asserts sides==4): polygon `D` of 6: `D[0]=AF[0], D[1]=B[2], D[2]=B[4], D[3]=AF[2], D[4]=B[8], D[5]=B[10]`. Inner lines: `C[0] = B[1] + (B[11]−B[0])`, then polylines `C[0]→B[4]`, `C[0]→B[8]`, `C[0]→B[0]`.
* **COMPONENT** (846–906; asserts sides==4): polygon `D` of 12 built by chained offsets from `B[2..7]` (see code: `D[2] = B[3] + (B[4]−B[3])`, `D[3] = D[2] + (B[3]−B[2])`, `D[4] = D[3] + (B[4]−B[3])`, `D[5] = D[4] + (D[2]−D[3])`, and mirrored `D[6..9]` from `B[5..8]`), `D[10]=AF[2]`, `D[11]=AF[3]`; two internal 4-point polylines.

The SBOLv shapes (PROMOTER … LPROMOTER, 908–1889) are all built from the same `B` table with `mid_x(AF)`, `mid_y(&AF[1])`, and width units `(B[2].x−B[3].x)` / `(B[3].y−B[4].y)`; each constructs one polygon (plus extra polylines for the dsDNA line and details) from 5–16 computed points. They are render-only (they do not affect sizing or hit-testing; hit-testing uses the 4-sided convex approximation via `poly_inside`). For a faithful port, transcribe each `case` arithmetic literally; the constants are all small fractions (1/8, 1/4, 1/2, 3/4, 1/3, 5/8, 9/8, 2.25, …) applied to those width units. Emission summary:

| shape | polygon pts | extra geometry |
|---|---|---|
| PROMOTER | 9 (`sides+5`) | dsDNA polyline left→right at mid height |
| CDS | 5 | — |
| TERMINATOR | 8 | dsDNA line |
| UTR | 6 | dsDNA line |
| PRIMERSITE | 5 | dsDNA line |
| RESTRICTIONSITE | 8 | two half dsDNA lines |
| FIVEPOVERHANG | 4 + 4 (two polygons) | right half dsDNA |
| THREEPOVERHANG | 4 + 4 | left half dsDNA |
| NOVERHANG | 4×4 rectangles | left+right dsDNA |
| ASSEMBLY | 4 + 4 | left+right dsDNA |
| SIGNATURE | 4 | "\" , "/" and bottom line polylines |
| INSULATOR | 4 | outer square polyline (5 pts) + two dsDNA halves |
| RIBOSITE | 16 ("X") | 2-part dashed line + dsDNA |
| RNASTAB | 8 (octagon) | 2-part dashed line + dsDNA |
| PROTEASESITE | 16 | solid line + dsDNA |
| PROTEINSTAB | 8 | solid line + dsDNA |
| RPROMOTER | 9 | — |
| RARROW | 7 | — |
| LARROW | 7 | — |
| LPROMOTER | 9 | — |

## B.10 `point` shape (shapes.c:3098–3319)

### `point_init` (3103–3192)

```
w = late_double(N_width,  DBL_MAX, MIN_NODEWIDTH)     # DBL_MAX sentinel = "not set"
h = late_double(N_height, DBL_MAX, MIN_NODEHEIGHT)
if both DBL_MAX: ND_width = ND_height = DEF_POINT (0.05 in)
else: w = min(w,h); if w > 0: w = max(w, MIN_POINT (0.0003)); ND_width = ND_height = w
sz = ND_width·72
peripheries = late_int(N_peripheries, base(=1), min 0)
outp = max(peripheries,1); if peripheries>=1 and penwidth>0: outp += 1
sides = 2 ; vertices = alloc(outp·2)
P = (sz/2, sz/2); vertices[0] = −P ; vertices[1] = P
if peripheries > 1: for j in 1..peripheries: P += GAP; append ±P ; sz = 2·P.x
if peripheries>=1 and penwidth>0 and outp>peripheries: P += penwidth/2 ; append ±P
sz_outline = 2·P.x
poly = {regular=true, peripheries, sides=2, orientation=0, skew=0, distortion=0, vertices}
ND_width = ND_height = PS2INCH(sz) ; ND_outline_* = PS2INCH(sz_outline)
```

### `point_inside` (3194–3233)

Same `outp` index math; `radius = poly->vertices[outp+1].x` (the corner of the outline ring); reject if `|P.x| > radius or |P.y| > radius`; else `hypot(P.x, P.y) <= radius`. (`P = ccwrotatepf(p, 90·rankdir)` first.)

### `point_gencode` (3235–3319)

`checkStyle` only for `invisible`; renderer style = `["invis", "filled"]` if invisible else `["filled"]`; penwidth applied if set; GUI states override colors; else `fillcolor = findFillDflt(n, "black")`, pen = `color` or black; `filled = true`; if `peripheries == 0` set to 1 with pen = fill color; per ring: `AF[i] = vertices[i+j*sides] + ND_coord` and `gvrender_ellipse(AF, filled)`, then `filled = false`.

## B.11 `star` (shapes.c:4034–4147)

`α = π/10` (18°), `α2 = 2α`, `α3 = 3α`, `α4 = 4α`.

```
star_size(sz0):                                  # fits a bb to the star
  rx = sz0.x / (2·cos α) ; ry = sz0.y / (sin α + sin α3)
  r0 = max(rx, ry)
  r  = r0 · sin α4 · cos α2 / (cos α · cos α4)
  return (2·r·cos α, r·(1 + sin α3))

star_vertices(vertices, bb):
  aspect = (1 + sin α3) / (2·cos α)
  if sz.y/sz.x > aspect: sz.x = sz.y/aspect elif < : sz.y = sz.x·aspect
  r  = sz.x/(2·cos α)
  r0 = r·cos α·cos α4/(sin α4·cos α2)
  offset = r·(1 − sin α3)/2                     # circle-center y shift from bb center
  theta = α
  for i in 0,2,4,..,8:
     vertices[i]   = r ·(cos θ, sin θ) − (0, offset) ; θ += α2
     vertices[i+1] = r0·(cos θ, sin θ) − (0, offset) ; θ += α2
  bb = sz

star_inside(ictxt, p):                            # non-convex test
  P = ccwrotatepf(...); if bp: INSIDE(P, *bp)
  outp index as poly_inside; sides = 10
  outcnt = 0
  for i in 0,2,4,..,8:
      Q = vertex[outp+i] ; R = vertex[outp + (i+4)%10]
      if !same_side(P, O, Q, R): outcnt += 1 ; if outcnt == 2: return false
  return true
```

(`star_fns` uses `poly_init` for sizing; the vertices generator hooks in through `p_star.vertices`.)

## B.12 `cylinder` (shapes.c:4149–4219)

```
cylinder_size(sz): sz.y *= 1.375 ; return sz       # extra height for the elliptical caps

cylinder_vertices(vertices, bb):                    # 19 points; Bézier-ish cap approximation
  x = bb.x/2 ; y = bb.y/2 ; yr = bb.y/11
  v[0] = ( x, y−yr)
  v[1] = ( x, y−(1−0.551784)·yr)
  v[2] = (0.551784·x, y)   v[3] = (0, y)   v[4] = (−0.551784·x, y)
  v[5] = (−x, v[1].y)      v[6] = (−x, y−yr)   v[7] = v[6]
  v[8] = (−x, yr−y)        v[9] = v[8]
  v[10] = (−x, −v[1].y)
  v[11..15] = mirrors of v[4..0] across origin (negate x,y)
  v[16] = v[15] ; v[17] = v[18] = v[0]

cylinder_draw(job, AF, sides, filled):              # AF = the 19 points mapped to canvas
  bottom = AF[0..6] mirrored about the horizontal center line: vertices[i].y = 2·AF[0].y − AF[i].y (x kept)
  beziercurve(AF, sides, filled)                    # outline incl. top cap
  beziercurve(bottom, 7, 0)                         # bottom cap
```

(`cylinder_fns` uses `poly_inside` — the 19-gon is treated as its convex hull for hit tests; `poly_init`'s periphery loop handles the duplicated vertices explicitly, shapes.c:2307–2315.)

## B.13 `record` / `Mrecord` (shapes.c:3321–3933)

Parse-state bits (3323–3327): `HASTEXT=1, HASPORT=2, HASTABLE=4, INTEXT=8, INPORT=16`. `ISCTRL(c)` = one of `{ } | < >`.

### B.13.1 `parse_reclbl(n, LR, flag, text)` (3358–3500)

Global cursor `reclblp` walks the label text; `text` is a scratch buffer. Returns a `field_t*` tree or NULL (after freeing) on parse error.

```
# pre-count max fields: scan for '|' at brace depth 0 ('\\' escapes next char; depth from '{'/'}')
rv.fld = alloc(maxf) ; rv.LR = LR ; mode = 0 ; fi = 0
hstsp = tsp = text ; wflag = true ; ishardspace = false
while wflag:
  skip bytes < ' ' (control chars)
  switch *reclblp:
   '<': if mode & (HASTABLE|HASPORT) → error ; (html labels: treat as text)
        mode |= HASPORT|INPORT ; advance ; hspsp = psp = text
   '>': if !(mode & INPORT) → error
        trim one trailing space (if psp > text+1 and last char is ' ' and not the first char)
        tmpport = strdup(text) ; mode &= ~INPORT ; advance
   '{': advance ; if mode != 0 or end → error ; mode = HASTABLE ;
        rv.fld[fi++] = parse_reclbl(n, !LR, false, text) or error     # nested table flips orientation
   '}' | '|' | '\0':
        if (!*reclblp && !flag) or (mode & INPORT) → error
        if !(mode & HASTABLE): fp = rv.fld[fi++] = new field
        if tmpport: fp.id = tmpport ; tmpport = NULL
        if !(mode & (HASTEXT|HASTABLE)): mode |= HASTEXT ; *tsp++ = ' '   # empty field ⇒ " "
        if mode & HASTEXT:
            trim one trailing space (same rule) ; *tsp = 0 ;
            fp.lp = make_label(n, text, lbl->html, is_record=false,
                               lbl->fontsize, lbl->fontname, lbl->fontcolor)
            fp.LR = true ; reset tsp/hstsp
        if *reclblp:
            if '}': advance ; rv.n_flds = fi ; return rv      # end of nested table
            mode = 0 ; advance                                # next '|' section
        else wflag = false
   '\\': if next char: if ISCTRL(next) → nothing (escaped below)
                      elif next == ' ' and !html → ishardspace = true
                      else { *tsp++ = '\\' ; mode |= INTEXT|HASTEXT } ; advance
         # falls through to default
   default (dotext):
        if (mode & HASTABLE) and *reclblp != ' ' → error
        if !(mode & (INTEXT|INPORT)) and *reclblp != ' ': mode |= INTEXT|HASTEXT
        if mode & INTEXT:  copy char unless (space collapsing rule: c==' ' and !ishardspace
                            and previous copied char is ' ' and !html); if ishardspace hstsp = tsp−1
        elif mode & INPORT: same for port buffer psp
        advance ; copy UTF-8 continuation bytes (0x80 ≤ b < 0xC0) into tsp
rv.n_flds = fi ; return rv
```

Record init (record_init, 3663–3707):

```
flip = !GD_realflip(g)                 # top-level layout is horizontal (LR=true) unless rankdir flips axes
reclblp = ND_label->text
textbuf = calloc(max(max(len,1), len("\\N")) + 1)
info = parse_reclbl(n, flip, true, textbuf)
if !info: agerrorf("bad label format %s\n", ...) ; reclblp = "\\N" ; info = parse_reclbl(...)  # fallback label
size_reclbl(n, info)
sz = (INCH2PS(ND_width), INCH2PS(ND_height))
if mapbool(fixed): (no resize; size check only)
else: sz = max(sz, info.size) per axis
resize_reclbl(info, sz, mapbool(nojustify))
pos_reclbl(info, ul = (−sz.x/2, +sz.y/2), sides = BOTTOM|RIGHT|TOP|LEFT)
ND_width  = PS2INCH(info.size.x)
ND_height = PS2INCH(info.size.y + 1)          # "+1 point" rounding kluge (comment at 3704)
ND_shape_info = info
```

### B.13.2 Sizing and layout

`size_reclbl(n, f)` (3502–3545): leaf with label → `d = f.lp->dimen` plus margin: `margin` attr (`"%lf,%lf"`, both clamped as parsed, `i>0` gate) `d.x += 2·INCH2PS(marginx)`, `d.y += 2·INCH2PS(i>1 ? marginy : marginx)`, else `PAD(d)`; container → LR: `d.x = Σ child.x`, `d.y = max(child.y)`; TB: `d.y = Σ child.y`, `d.x = max(child.x)`. Stores `f.size = d`.

`resize_reclbl(f, sz, nojustify)` (3547–3582): `d = sz − f.size` ; `f.size = sz` ; if leaf and !nojustify: `f.lp->space += d` (component-wise); if container: `inc = (LR ? d.x : d.y)/n_flds`, and for child `i` the amount is `(int)((i+1)·inc) − (int)(i·inc)` (integer truncation distributes remainders to earlier children); child newsz = LR ? `(child.size.x + amt, sz.y)` : `(sz.x, child.size.y + amt)`; recurse.

`pos_reclbl(f, ul, sides)` (3589–3628): `f.b.LL = (ul.x, ul.y − f.size.y)`, `f.b.UR = (ul.x + f.size.x, ul.y)`; `f.sides = sides`. Children masks (when `sides != 0`):

```
LR container: i==0==last → TOP|BOTTOM|RIGHT|LEFT ; i==0 → TOP|BOTTOM|LEFT ;
              i==last     → TOP|BOTTOM|RIGHT   ; else TOP|BOTTOM
TB container: i==0==last → TOP|BOTTOM|RIGHT|LEFT ; i==0 → TOP|RIGHT|LEFT ;
              i==last     → LEFT|BOTTOM|RIGHT  ; else LEFT|RIGHT
```
child `i` gets `sides & mask` and ul advances: LR → `ul.x += child.size.x`; TB → `ul.y -= child.size.y`.

Note the field rect geometry: record fields are **y-up rects in node coordinates**, rooted at `ul = (−width/2, +height/2)` and grown right/down; `b` does **not** include `ND_coord` (added at use time).

### B.13.3 `record_inside` (3762–3787)

```
p = ccwrotatepf(p, 90·rankdir)
bbox = bp ?: (ND_shape_info as Field).b
extend bbox by penwidth/2 on all sides (outline-aware; DEFAULT_NODEPENWIDTH/MIN from attrs)
return INSIDE(p, bbox)
```

### B.13.4 `record_path` (3793–3829)

```
if !prt->defined: return 0
p = prt->p ; info = ND_shape_info
for each top-level field i:
    ls,rs = flip ? (b.LL.y, b.UR.y) : (b.LL.x, b.UR.x)
    if BETWEEN(ls, p.x, rs):
        rv[0] = flip ? flip_rec_boxf(field.b, ND_coord)
                     : { LL = (coord.x+ls, coord.y − ht/2), UR = (coord.x+rs, ·) }
        rv[0].UR.y = coord.y + ht/2
        *kptr = 1 ; break
return side
```

### B.13.5 `gen_fields` / `record_gencode` (3831–3933)

```
gen_fields(job, n, f):
  if f.lp: f.lp->pos = midpoint(f.b.LL, f.b.UR) + ND_coord ; emit_label(...) ; penColor(n)
  for i in 0..n_flds:
      if i > 0:
          LR: separator = vertical line at child.b.LL.x from child.b.LL.y to child.b.UR.y
          TB: separator = horizontal line at child.b.UR.y from child.b.LL.x to child.b.UR.x
          (+ ND_coord) ; polyline
      gen_fields(child)

record_gencode(job, n):
  BF = root field.b + ND_coord
  style = stylenode(job, n) ; penColor(n)
  fill handling as poly_gencode (findStopColor → GRADIENT/RGRADIENT)
  if shape name == "Mrecord": style.rounded = true
  if SPECIAL_CORNERS(style):
      AF = [(LL), (UR.x, LL.y), (UR), (LL.x, UR.y)]     # 4 corners in edge order
      round_corners(job, AF, 4, style, filled)
  else: gvrender_box(BF, filled)
  gen_fields(job, n, f)
```

`record_free` frees the field tree recursively (`free_field`, 3335–3347).

## B.14 `epsf` (shapes.c:3992–4032, psusershape.c:99–124)

`epsf_init`: resolve `shapefile` via `safefile`, `user_init` reads the EPS bounding box; `ND_width/height = PS2INCH(dx/dy)`; `ND_shape_info = {macro_id, offset = (−us.x − dx/2, −us.y − dy/2)}`. Missing file → warning (shape falls back to default size). `epsf_inside`: `P = ccwrotatepf(p, 90·rankdir); x2 = ND_ht/2; return −x2 ≤ P.y ≤ x2 && −ND_lw ≤ P.x ≤ ND_rw`. `epsf_gencode` emits `translate + user_shape_<id>` (PostScript-only path) and the label at `ND_coord`.

## B.15 `shape=point` emission notes and `point_style` (shapes.c:51)

`static char *point_style[3] = { "invis\0", "filled\0", 0 }` — the leading `"invis"` (with embedded NUL) is only used when `style=invis`; otherwise the renderer gets the sublist starting at `"filled"`.

---

# Part C — `utils.c` and `labels.c`

## C.1 `gv_nodesize` (utils.c:1525–1536)

```
gv_nodesize(n, flip):
  if flip:   w = INCH2PS(ND_height); ND_lw = ND_rw = w/2 ; ND_ht = INCH2PS(ND_width)
  else:      w = INCH2PS(ND_width);  ND_lw = ND_rw = w/2 ; ND_ht = INCH2PS(ND_height)
```

Called by dot after `common_init_node` (dotinit.c:44–56: `common_init_node(n); gv_nodesize(n, GD_flip(g));`). So `ND_lw == ND_rw` always here (asymmetric widths would come from HTML labels via other paths). `ND_xsize = lw+rw`, `ND_ysize = ht` (types.h:536–537). `postproc.c:163` re-runs it with `flip=false` for virtual nodes.

## C.2 `common_init_node` (utils.c:416–443)

```
ND_width  = late_double(n, N_width,  0.75, min 0.01)       # inches
ND_height = late_double(n, N_height, 0.5,  min 0.02)
ND_shape  = bind_shape(late_nnstring(n, N_shape, "ellipse"), n)
fi.fontsize = late_double(n, N_fontsize, 14.0, min 1.0)
fi.fontname = late_nnstring(n, N_fontname, "Times-Roman")
fi.fontcolor= late_nnstring(n, N_fontcolor, "black")
str = agxget(n, N_label)
ND_label(n) = make_label(n, str, aghtmlstr(str), shapeOf(n) == SH_RECORD,
                         fi.fontsize, fi.fontname, fi.fontcolor)
if N_xlabel and xlabel nonempty:
    ND_xlabel = make_label(..., is_record=false, ...) ; GD_has_labels |= NODE_XLABEL
ND_showboxes = min(late_int(n, N_showboxes, 0, 0), UCHAR_MAX) as u8
ND_shape->fns->initfn(n)              # poly_init / point_init / record_init / epsf_init
```

Record labels are created with `is_record = true`, so `make_label` stores the raw text and does **not** compute `dimen` (labels.c:139–144); each record field's label is sized independently in `parse_reclbl` via `make_label(..., is_record=false, ...)`.

## C.3 `common_init_edge` (utils.c:498–556)

```
if E_label nonempty:  ED_label = make_label(e, str, html, false,
                          late_double(e, E_fontsize, 14.0, 1.0),
                          late_nnstring(e, E_fontname, "Times-Roman"),
                          late_nnstring(e, E_fontcolor, "black"))
                      GD_has_labels |= EDGE_LABEL
                      ED_label_ontop = mapbool(late_string(e, E_label_float, "false"))
if E_xlabel nonempty: (fonts from edge attrs) ED_xlabel ; |= EDGE_XLABEL
if E_headlabel nonempty: fonts fall back — labelfontsize ← E_labelfontsize else E_fontsize;
    labelfontname ← E_labelfontname else E_fontname; labelfontcolor ← E_labelfontcolor else E_fontcolor
    (initFontLabelEdgeAttr, 452–460) ; ED_head_label ; |= HEAD_LABEL
if E_taillabel nonempty: same → ED_tail_label ; |= TAIL_LABEL

# ports (TAIL_ID="tailport", HEAD_ID="headport")
str = agget(e, "tailport") ?: ""
if str nonempty: ND_has_port(tail) = true
ED_tail_port = chkPort(shape(tail).portfn, tail, str)
if noClip(e, E_tailclip): ED_tail_port.clip = false       # tailclip=false disables clipping
same for head with "headport"/E_headclip/ED_head_port
```

`chkPort(pf, n, s)` (477–495): splits `s` at the **first** `':'` — `"name:compass"` → `pf(n, "name", "compass")` with `pt.name = "compass"`; a leading colon `":abc"` is accepted (deprecated). No colon → `pf(n, s, NULL)` with `pt.name = s`. Empty string → `pf(n, "", NULL)` → `poly_port`/`record_port` return `Center` (`defined=false`, `clip=true`, `theta=-1`), with `pt.name = ""`.

`noClip(e, sym)` (462–475): attr nonempty and `!mapbool(value)` → true (i.e. `tailclip=false` ⇒ no clipping).

`ports_eq(e, f)` (dotgen/position.c:1030–1039): two edges share ports iff `head.defined` equal and (both undefined or head `p` equal) and (tail `p` equal or tail undefined). Used by dot to merge multi-edges.

## C.4 Label sizing (`make_label`, `make_simple_label`, `storeline`, `emit_label`) — labels.c

```
make_label(obj, str, is_html, is_record, fontsize, fontname, fontcolor):
  is_html &= str != ""
  charset = GD_charset(g)
  if is_record:  rv.text = strdup(str); rv.html = is_html        # NO dimension computed here
  elif is_html:  rv.text = strdup(str); rv.html = true; make_html_label(obj, rv)  # computes dimen
  else:
     rv.text = strdup_and_subst_obj0(str, obj, escBackslash=0)    # \G \N \E \T \H \L substitution
     latin1→UTF8 or htmlEntity→UTF8 per charset
     make_simple_label(gvc, rv)                                   # computes dimen
  return rv

make_simple_label(gvc, lp):                       # labels.c:57-105
  lp.dimen = (0,0)
  scan text: BIG5 lead bytes consumed pairwise; '\\' + one of n/l/r → storeline(current, just)
             '\\' + other → emit next char literally; real '\n' → storeline(just='n')
  trailing buffer → storeline(just='n')
  lp.space = lp.dimen

storeline(gvc, lp, line, terminator):             # labels.c:26-54
  span.str = line ; span.just = terminator        # 'n' center | 'l' left | 'r' right
  if line nonempty: span.font = {lp.fontname, lp.fontsize}; size = textspan_size(gvc, span)
  else: size.x = 0 ; size.y = (int)(lp.fontsize·LINESPACING)     # 1.2
  lp.dimen.x = max(dimen.x, size.x) ; lp.dimen.y += size.y
```

`textspan_size` (textspan.c:51–76): asks the text-layout plugin; fallback `estimate_textspan_size` (34–48): `size.y = fontsize·1.2`, `yoffset_layout = fontsize`, `yoffset_centerline = 0.1·fontsize`, `size.x = fontsize·estimate_text_width_1pt(font, str, bold, italic)`. **For the Rust port this is where you plug GPUI font metrics** (width of the string at `fontsize`, line height `1.2·fontsize`).

`emit_label` (labels.c:217–275) — baseline/just positioning given `space`/`dimen`/`pos`:

```
first span y: valign 't' → pos.y + space.y/2 − fontsize
              'b'       → pos.y − space.y/2 + dimen.y − fontsize
              'c'/def   → pos.y + dimen.y/2 − fontsize
if labeledgealigned: p.y −= pos.y
per span x:  just 'l' → pos.x − space.x/2
             'r' → pos.x + space.x/2
             'n'/def → pos.x
emit textspan at p ; p.y −= span.size.y        # spans stack downward
```

(Recall graphviz render coordinates are y-down; `pos` is the center of `space`.)

---

# Part D — Integration checklist for the Rust port

1. **Unit discipline.** Keep `width/height` (node attrs, `ND_width/height`, `ND_outline_*`) in inches; everything geometric in points. `ND_lw/ND_rw/ND_ht` in points, set once by `gv_nodesize` (flip-aware).
2. **Two frames.** Shape geometry (vertices, field rects, port aiming points) lives in the node frame (+y up, centered). Layout coordinates are y-down; convert at the boundary exactly like `cwrotatepf(p, 90·rankdir)`/`ccwrotatepf` pairs do. For `rankdir=TB` the two frames differ only by the y-axis sign flip applied at render time.
3. **Label pipeline order.** `common_init_node` → `make_label` (dimen) → `shape initfn` (consumes `dimen`, produces `space`, vertices, `ND_width/height`, `ND_outline_*`) → `gv_nodesize` (produces `lw/rw/ht`).
4. **Peripheries & penwidth.** The vertex array always has `outp·sides` points: rings 0..peripheries−1 are the GAP-spaced peripheries; when `peripheries ≥ 1 && penwidth > 0` there is a final "outline" ring at `penwidth/2` outside the outer ring. Hit tests (`poly_inside`, `point_inside`, `star_inside`, `record_inside`) use the outline ring so that endpoints land just outside the stroke; rendering draws the periphery rings only.
5. **Inside-function cache.** `inside_t.s` caches per-node scale factors, outline bbox half-sizes, `outp` ring index and the last-tested segment index (`last`) — the fast path `same_side(P, O, Q, R)` short-circuits spline clipping. Replicate it for identical clip behavior/performance.
6. **Arrow clip invariants.** After `arrowStartClip`/`arrowEndClip`, the stored `bezier` keeps `sflag/eflag/sp/ep`; rendering draws arrows from `sp`/`ep` toward `list[0]`/`list[last]`. The spline's extreme control point is moved to the arrow base; the tip coordinate is preserved in `sp`/`ep` even though the curve no longer reaches it.
7. **Bug-compatibility.** Replicate literally: the duplicated condition in `arrow_length_tee` (arrows.c:1243–1251), the `+1` point height kluge in `record_init` (shapes.c:3704), the `arrowsize` cancellation comment for crow `shaftwidth` (arrows.c:643), `xsize`/`ysize` swap on `GD_flip` in `poly_inside`, and `EMPTY`-field space insertion in `parse_reclbl`.
8. **Rounding.** `poly_init` uses `fmax` everywhere (no integer snapping); the only quantization is the `quantum` attribute (`ceil(v/q)·q`). `resize_reclbl` uses `(int)` truncation for equal distribution. `storeline` casts line height to `int` for empty lines only.
9. **Constants that look arbitrary but matter** for pixel-identical output: `ARROW_LENGTH=10`, lenfacts `{1.0, 1.0, 0.5, 1.0, 1.2, 0.8, 1.0, 0.5}`, arrow width factors `{0.35, 0.45, 0.4, 1/3, 0.5}` with the `penwidth > 4` scaling rules, `GAP=4`, `RBCONST=12`, `RBCURVE=0.5`, `0.551784` (cylinder cap κ), `1.375`/`11` (cylinder), star angles `π/10`, `miterlimit=4.0`, `DEF_POINT=0.05`, `MIN_POINT=0.0003`, bezier clip tolerance `0.5` pt, `MILLIPOINT=0.001`.
