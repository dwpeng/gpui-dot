# Graphviz spline fitting & routing — code-level implementation spec

**Target:** faithful Rust port of Graphviz's edge-spline machinery:
`clip_and_install` / `routesplines_` (lib/common) and the whole `lib/pathplan`
library (`Proutespline`, `Pshortestpath`, the `Pobs*` visibility stack).

**Source tree examined:** `/tmp/graphviz-src` (Graphviz master, post-8.x —
`lib/util/list.h`-era code; `ccw`/`triang.c`/`solvers.c`/`routespl.c` in their
current form). All `file:line` citations below refer to that tree.

**Sources read in full:**

| File | Purpose |
|---|---|
| `lib/common/splines.c` (1373 lines) | `clip_and_install`, self edges, labels, midpoints |
| `lib/common/routespl.c` (1036 lines) | box-polygon builder, `checkpath`, `routesplines_`, straight edges |
| `lib/pathplan/route.c` (495 lines) | `Proutespline` — Hermite fit + recursive split |
| `lib/pathplan/shortest.c` (448 lines) | `Pshortestpath` — triangulation + funnel |
| `lib/pathplan/triang.c` (150 lines) | `ccw`, `Ptriangulate`, `isdiagonal` |
| `lib/pathplan/solvers.c` (105 lines) | cubic/quadratic/linear root solver |
| `lib/pathplan/util.c` (70 lines) | `make_polyline`, `Ppolybarriers` |
| `lib/pathplan/visibility.c` (355 lines) | visibility graph (`compVis`, `ptVis`, `directVis`) |
| `lib/pathplan/cvt.c` (194 lines) | `Pobsopen`/`Pobsclose`/`Pobspath` |
| `lib/pathplan/shortestpth.c` (109 lines) | Dijkstra `shortestPath`, `makePath` |
| `lib/pathplan/inpoly.c` (35 lines) | `in_poly` |
| headers: `pathgeom.h`, `pathplan.h`, `pathutil.h`, `vis.h`, `vispath.h`, `tri.h`, `solvers.h` | types & API |
| consulted: `lib/common/arrows.c`, `lib/common/utils.c` (`Bezier`), `lib/common/emit.c` (`update_bb_bz`), `lib/common/geom.h`, `lib/common/types.h`, `lib/dotgen/dotsplines.c`, `lib/neatogen/neatosplines.c`, `lib/util/gv_math.h` | callers & helpers |

---

## 0. Corrections to commonly repeated descriptions (read first)

The folklore description of these files (and older papers) drifts from the
current code. A porter MUST work from the following facts:

1. **There is no `lib/pathplan/vispath.c`.** The visibility-graph code lives in
   `visibility.c` (`compVis`, `ptVis`, `directVis`) plus `cvt.c`
   (`Pobsopen`/`Pobsclose`/`Pobspath`) and `shortestpth.c`
   (`shortestPath`/`makePath`). `vispath.h`/`vis.h` are headers only.
2. **There is no `SLOP`, `GROWU`, `CHI`, `VMULD`, `HMUL`, `NCH`, `EPSILON3`,
   `THETA` anywhere in this tree.** Those constants belonged to 1990s-era
   Graphviz. The real constants are enumerated in §4.
3. `_routesplines` is now the static `routesplines_`
   (`lib/common/routespl.c:290`) with two public one-line wrappers
   `routesplines` / `routepolylines` (`routespl.c:595-601`).
4. There is **no `make_poly_from_boxes`** function; the box→polygon
   construction is inline in `routesplines_` (`routespl.c:329-436`).
5. There is **no `straight_simple`**; the corresponding helpers are
   `makeStraightEdge` / `makeStraightEdges` (`routespl.c:954-1036`).
6. There is **no function named `splines()`**. Neato's obstacle-based router is
   `spline_edges_` + `makeSpline` (`lib/neatogen/neatosplines.c:586, 538`).
   **dot does not use it** — dot uses `routesplines_` via `dotsplines.c`.
7. **There is no "midpoint rule" for polyline→Bézier conversion.** The two
   conversions that actually exist are:
   * `make_polyline` (`pathplan/util.c:44-62`): duplicates interior polyline
     points so each input *edge* becomes one *degenerate* cubic
     (`P0=P1=p_i`, `P2=P3=p_{i+1}`) — used for `splines=polyline`, not for
     smooth splines.
   * `Proutespline` (`route.c`): Hermite fit; shortest-path polyline *vertices
     lie exactly on the output spline as segment boundaries*; handles are
     solved by least squares (`mkspline`) then tuned by halving in
     `splinefits`; the path is recursively split at the worst-fitting vertex.
   See §8.
8. `Pobsbarriers` does not exist; the function is `Ppolybarriers`
   (`util.c:24-42`), and neato has its own `make_barriers`
   (`neatosplines.c:48-79`) that excludes endpoint polygons.
9. **`ccw()` sign convention is inverted relative to its enum names** — see
   §5.1. This is the single easiest thing to get wrong in a port.

---

## 1. Architecture: who calls what

```
dot (ranked layouts)
  dotsplines.c dot_splines_            (dotsplines.c:228-486; wrapper 486)
    routesplinesinit()                 routespl.c:211
    per edge/segment:
      beginpath()/endpath()  (splines.c:375, 573)  -> pathend_t.boxes
      completeregularpath()  -> path *P: P->start, P->end, P->boxes[]
      routesplines(P,&pn)  [splines] or routepolylines(P,&pn) [pline]
        routespl.c:1412,1486,1610,1807,1857 call sites
        = routesplines_(P, &pn, polyline)        routespl.c:290
          checkpath(boxn, boxes, P)              routespl.c:631
          build channel polygon (8*boxn pts)     routespl.c:342-436
          Pshortestpath(&poly, eps, &pl)         shortest.c:83
            triangulate() (ear clipping)         shortest.c:317 / triang.c
            funnel algorithm (deque)             shortest.c:230-288
          polyline ? make_polyline(pl) : Proutespline(edges, pn, pl, evs, &spl)
            route.c:70 — Hermite fit against barrier edges
          limitBoxes() loop — reclaim box x-extent routespl.c:231,543
      clip_and_install(fe, hn, ps, pn, sinfo)  splines.c:233
        new_spline, shape clipping via insidefn, arrow_clip, ED_spl attach
      addEdgeLabels(e)                         splines.c:1305
    routesplinesterm()                 routespl.c:224

neato/fdp/sfdp (unpositioned layouts)  neatosplines.c:586 spline_edges_
    makeObstacle() per node -> obs[]
    Pobsopen(obs, npoly)               cvt.c:28  (builds vconfig_t + vis matrix)
    getPath(e, vconfig, true) -> Pobspath()  cvt.c:102 (ptVis + makePath/Dijkstra)
    per edge:
      splines: makeSpline(e, obs,...)  neatosplines.c:538
               = make_barriers + Proutespline(barriers, ED_path(e), {0,0})
      polyline: makePolyline(e)        neatosplines.c:519 = make_polyline + clip_and_install
      fallback: makeStraightEdge        routespl.c:954
    Pobsclose(vconfig)                 cvt.c:89
```

**dot never uses `Pobsopen`/`Pobspath`/Dijkstra.** For dot, the only pathplan
entry points are `Pshortestpath` (funnel over the channel polygon) and
`Proutespline` (spline fit against the polygon edges as barriers).
`simpleSplineRoute` (`routespl.c:168-206`) is a third, smaller entry point used
only for labeled flat edges in dot (`dotsplines.c:1025, 1069`).

---

## 2. Core data types

### 2.1 pathplan geometry (`lib/pathplan/pathgeom.h:33-59`)

```c
typedef struct Pxy_t { double x, y; } Pxy_t;
typedef struct Pxy_t Ppoint_t;      // a 2D point
typedef struct Pxy_t Pvector_t;     // same layout, interpreted as vector
typedef struct Ppoly_t      { Ppoint_t *ps; size_t pn; } Ppoly_t;
typedef Ppoly_t Ppolyline_t;        // same struct; open path
typedef struct Pedge_t       { Ppoint_t a, b; } Pedge_t;   // barrier segment
typedef struct vconfig_s vconfig_t; // opaque; see vis.h:29-39
typedef double COORD;               // pathutil.h:31
```

`vconfig_t` (`vis.h:29-39`):

```c
struct vconfig_s {
  int Npoly;     // number of obstacle polygons
  int N;         // total vertices in the walk of all barriers
  Ppoint_t *P;   // barrier points, concatenated per polygon
  int *start;    // Npoly+1 offsets; polygon i occupies P[start[i]..start[i+1])
  int *next;     // cyclic next index within polygon
  int *prev;     // cyclic prev index within polygon
  COORD **vis;   // N x N (plus 2 spare rows) symmetric weight matrix
};
```

### 2.2 Rendering-side structs (`lib/common/types.h:64-102`)

```c
typedef struct { bool swapEnds(edge_t*); bool splineMerge(node_t*);
                 bool ignoreSwap; bool isOrtho; } splineInfo;   // types.h:64-71

typedef struct {            // types.h:73-79
  boxf nb;                  // node box
  pointf np;                // node port point
  int sidemask;             // TOP/BOTTOM/LEFT/RIGHT bit
  int boxn; boxf boxes[20]; // end boxes built by beginpath/endpath
} pathend_t;

typedef struct {            // types.h:81-87
  port start, end;          // .p (pointf), .theta (double), .constrained (bool)
  size_t nbox; boxf *boxes; // channel boxes, tail end first
  void *data;               // the edge (edge_t*)
} path;

typedef struct {            // types.h:89-96
  pointf *list; size_t size;// 3n+1 control points; segments = (size-1)/3
  uint32_t sflag, eflag;    // arrow types (0 = none)
  pointf sp, ep;            // arrow tip anchor points
} bezier;

typedef struct { bezier *list; size_t size; boxf bb; } splines; // types.h:98-102
```

`ED_spl(e)` is `splines*`; `ED_spl(e)->list[k]` is one Bézier. `inside_t`
(`types.h:160-170`) is a union; shape clipping uses the `.s` variant
(`{node_t *n; boxf *bp; ...}`), arrow clipping uses the `.a` variant
(`{pointf *p; double *r;}`).

### 2.3 Pathplan API (`pathplan.h:46-60`)

```c
int  Pshortestpath(Ppoly_t *boundary, Ppoint_t endpoints[2], Ppolyline_t *out);
int  Proutespline(Pedge_t *barriers, size_t n_barriers, Ppolyline_t input_route,
                  Pvector_t endpoint_slopes[2], Ppolyline_t *output_route);
int  Ppolybarriers(Ppoly_t **polys, int npolys, Pedge_t **barriers, int *n_barriers);
void make_polyline(Ppolyline_t line, Ppolyline_t *sline);
// vispath.h:35-51
vconfig_t *Pobsopen(Ppoly_t **obstacles, int n_obstacles);
void       Pobsclose(vconfig_t *config);
void       Pobspath(vconfig_t*, Ppoint_t p0, int poly0, Ppoint_t p1, int poly1,
                    Ppolyline_t *output_route);
#define POLYID_NONE    -1111   // vispath.h:50
#define POLYID_UNKNOWN -2222   // vispath.h:51
```

---

## 3. All constants, verbatim

| Constant | Value | Defined at | Meaning |
|---|---|---|---|
| `EPSILON1` | `1E-3` | route.c:21 | (a) spline-vs-input-path length slack in the "shortcut" rejection of `splinefits` (compared against a *distance*, route.c:241); (b) squared-distance slack for allowing a spline to touch a barrier endpoint (route.c:305-306) |
| `EPSILON2` | `1E-6` | route.c:22 | spline parameter margin: roots in `[EPSILON2, 1-EPSILON2]` count as real intersections (route.c:294) |
| det threshold (mkspline) | `1e-6` | route.c:184,188 | `fabs(det01) < 1e-6` ⇒ least-squares system considered singular ⇒ fallback `d01 = dist/3` |
| `normv` threshold | `1e-6` | route.c:414 | normalize vector only if `x²+y² > 1e-6` |
| initial handle factor `a` | `4` | route.c:222 | `sps[1] = pa + a*va/3`, `sps[2] = pb - a*vb/3` |
| `a` shrink | `a /= 2` while `a > .01`, then `a = 0` | route.c:272-275 | halving loop of `splinefits` |
| `a` termination | `a < 0.005` | route.c:258 | give up (or force if `inpn == 2`) |
| `solve3/2/1` `EPS` | `1E-7` | solvers.c:23 | leading-coefficient zero test |
| `AEQ0(x)` | `(x) < EPS && (x) > -EPS` | solvers.c:24 | "absolutely equal zero" macro |
| `FUDGE` (limitBoxes) | `.0001` | routespl.c:259 | (a) y-in-box slack when bounding boxes to the spline; (b) horizontal/vertical detection slack (routespl.c:507,521) |
| `INIT_DELTA` | `10` | routespl.c:270 | initial sampling density for `limitBoxes`: `num_div = delta * boxn` samples per cubic segment |
| `LOOP_TRIES` | `15` | routespl.c:271-273 | max `limitBoxes` retries with `delta *= 2` |
| degenerate-box threshold | `.01` | routespl.c:638,640 | `checkpath` drops boxes thinner than this |
| don't-touch second fix | `(a+b)/2.0 + 0.5` | routespl.c:692-701 | midpoint-plus-half repair of remaining axis violations |
| `INITIAL_LLX/URX` | `DBL_MAX / -DBL_MAX` | routespl.c:438-439 | sentinels for "box not yet x-bounded" (tested with bitwise fp equality, routespl.c:557-558) |
| `FUDGE` (beginpath) | `2` | splines.c:372 | gap kept from node side when building end boxes |
| `HT2(n)` | `ND_ht(n)/2` | splines.c:373 | half node height |
| `MILLIPOINT` | `.001` | geom.h:74 | tolerance for `APPROXEQPT` (dist² < tol²) in `clip_and_install` (splines.c:290,293) and `edgeMidpoint` |
| bezier_clip convergence | `.5` (points) | splines.c:144 | binary search stops when consecutive split points are within 0.5 in x **and** y |
| `W_DEGREE` | `5` | utils.c:171 | de Casteljau table size in `Bezier()` (degree 3) |
| `HW` | `2.0` | emit.c:739 | control-point flatness tolerance in `update_bb_bz` |
| `unseen` | `INT_MAX` | shortestpth.c:16 | Dijkstra "infinity" (as COORD, stored negated) |
| `DQ_FRONT / DQ_BACK` | `1 / 2` | shortest.c:23-24 | funnel deque side selectors |
| `POLYID_NONE` | `-1111` | vispath.h:50 | endpoint not inside any obstacle |
| `POLYID_UNKNOWN` | `-2222` | vispath.h:51 | endpoint containment not determined |
| `wind` deadband | `±.0001` | visibility.c:61 | orientation deadband (gcc -ffast-math tolerance) |
| `ISCCW / ISCW / ISON` | `1 / 2 / 3` | tri.h:41-44 | return values of `ccw()` — **names are misleading**, see §5.1 |
| `SELF_EDGE_SIZE` | (referenced) | splines.c:1147 | self-edge extra space; equals default `nodesep` |

Constants from the folklore list that **do not exist here**: `SLOP`, `GROWU`,
`CHI`, `VMULD`, `HMUL`, `NCH`, `THETA`, `EPSILON3`, `CI`, `LAG`, `SIDE`,
`PORT`*.

---

## 4. Geometric conventions and the `ccw` sign trap

### 4.1 `ccw` (triang.c:25-28) — port the *behavior*, not the names

```c
int ccw(Ppoint_t p1, Ppoint_t p2, Ppoint_t p3) {
    double d = (p1.y - p2.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p1.x - p2.x);
    return d > 0 ? ISCW : (d < 0 ? ISCCW : ISON);
}
```

`d` equals the standard cross product `(p2-p1) × (p3-p2)`. So **`d > 0` (a left
/ counter-clockwise turn in y-up coordinates) returns `ISCW` (=2)** and
`d < 0` returns `ISCCW` (=1), i.e. the enum names in `tri.h:41-44` are
swapped relative to mathematical convention. Every call site
(`Pshortestpath`, funnel, `isdiagonal`, `pointintri`) compares against these
constants, so a Rust port should either (a) reproduce this exact function and
constant mapping verbatim, or (b) replace `ccw(...) == ISCCW` with
`sign(cross) < 0` everywhere *systematically*. Do not mix conventions.

`ccw` uses an exact sign test (no deadband). Contrast with `wind`
(visibility.c:55-62), the same formula with a `±1e-4` deadband:

```c
int wind(Ppoint_t a, Ppoint_t b, Ppoint_t c) {
    COORD w = (a.y - b.y) * (c.x - b.x) - (c.y - b.y) * (a.x - b.x);
    return w > .0001 ? 1 : (w < -.0001 ? -1 : 0);
}
```

`area2(a,b,c)` (visibility.c:46-49) returns `w` unnormalized (twice signed
triangle area, same formula). `dist2` (visibility.c:122-128) is `dx²+dy²`.

### 4.2 Misc helpers

* `APPROXEQPT(p,q,tol)` = `DIST2(p,q) < SQR(tol)` (geom.h:71);
  `DIST2(p,q) = (p.x-q.x)² + (p.y-q.y)²` (geom.h:66).
* `is_exactly_equal(a,b)` is `memcmp(&a,&b,8)==0` (gv_math.h:51-53) — so
  `+0.0 != -0.0` and it distinguishes `DBL_MAX` from anything else bitwise.
  Used at routespl.c:557-558 and routespl.c:944.
* `Bezier(V, t, Left, Right)` (utils.c:175-203): standard de Casteljau on 4
  control points; returns the point at `t`; optionally fills the left
  (`Vtemp[j][0]`, j=0..3) and right (`Vtemp[3-j][j]`, j=0..3) sub-curve
  control polygons. Used by `bezier_clip`, `place_portlabel`,
  `update_bb_bz`, and the label midpoint code.
* `update_bb_bz(bb, cp)` (emit.c:754-796): if any control point of segment
  `cp` lies outside `bb`, and the segment is "sufficiently flat"
  (`ptToLine2(cp0,cp3,cp1) < HW² && ...cp2...`, HW=2.0, emit.c:746-751),
  expand `bb` by all four control points; otherwise split at t=0.5 and
  recurse on both halves. Called from `clip_and_install` (splines.c:310).

---

## 5. `lib/common/splines.c`

### 5.1 `new_spline(e, sz)` — splines.c:212-226

```text
walk e = ED_to_orig(e) while e is not NORMAL            (splines.c:214-215)
if ED_spl(e) == NULL: allocate splines (list=NULL, size=0, bb unset)
realloc list to (size+1) beziers                        (218-219)
rv = &list[size]; size += 1
rv.list = calloc(sz pointf); rv.size = sz
rv.sflag = rv.eflag = 0; rv.sp = rv.ep = (0,0)          (223-224)
```

So a fresh Bézier has **no arrow flags and zero `sp`/`ep`**; they are only set
if `arrow_clip` clips an arrow (§5.3).

### 5.2 `clip_and_install(fe, hn, ps, pn, info)` — splines.c:233-313

This is the single function every router funnels through. Inputs: raw control
point array `ps[0..pn-1]` in 3-stride Bézier form (pn = 3k+1), the edge `fe`,
the head node `hn` (which end of the raw path is being finished), and
`splineInfo`.

```text
tn = agtail(fe); g = graph of tn
newspl = new_spline(fe, pn)                        (242-244)   // appends a bezier
orig = fe, walk ED_to_orig while not NORMAL        (246-247)

// reversed flat edge? swap which node is "head" for clipping  (249-252)
if !info->ignoreSwap && ND_rank(tn)==ND_rank(hn) && ND_order(tn)>ND_order(hn):
    SWAP(hn, tn)

if tn == agtail(orig):                             (253-258)
    clipTail = ED_tail_port(orig).clip; clipHead = ED_head_port(orig).clip
    tbox = ED_tail_port(orig).bp;      hbox  = ED_head_port(orig).bp
else:  // fe and orig are reversed                 (259-264)
    clipTail = ED_head_port(orig).clip; clipHead = ED_tail_port(orig).clip
    hbox = ED_tail_port(orig).bp;       tbox = ED_head_port(orig).bp

// ---- tail-end shape clipping --------------------------------- (266-277)
if clipTail && ND_shape(tn) && ND_shape(tn)->fns->insidefn:
    ctx = inside_t{ .s = { .n = tn, .bp = tbox } }
    start = 0
    while start < pn-4:                            (269-274, stride 3)
        p2 = ps[start+3] - ND_coord(tn)            // node-relative coords
        if !insidefn(ctx, p2): break
        start += 3
    shape_clip0(ctx, tn, &ps[start], left_inside = true)
else start = 0

// ---- head-end shape clipping (mirror) ------------------------- (278-288)
if clipHead && ...:
    end = pn-4
    while end > 0:
        p2 = ps[end] - ND_coord(hn)
        if !insidefn(ctx_h, p2): break
        end -= 3
    shape_clip0(ctx_h, hn, &ps[end], left_inside = false)
else end = pn-4

// ---- skip degenerate (zero-length) end segments --------------- (289-294)
while start < pn-4 and APPROXEQPT(ps[start], ps[start+3], MILLIPOINT): start += 3
while end > 0     and APPROXEQPT(ps[end],   ps[end+3],   MILLIPOINT): end   -= 3

arrow_clip(fe, hn, ps, &start, &end, newspl, info)              (295)

// ---- copy surviving points into the new bezier ---------------- (296-311)
i = start
loop:
    newspl->list[i-start] = ps[i]; cp[0] = ps[i]; i++
    if i >= end+4: break
    newspl->list[i-start] = ps[i]; cp[1] = ps[i]; i++
    newspl->list[i-start] = ps[i]; cp[2] = ps[i]; i++
    cp[3] = ps[i]                       // copied at top of next iteration
    update_bb_bz(&GD_bb(g), cp)         // grow graph bounding box
newspl->size = end - start + 4                       (312)
```

Key facts for a porter:

* **Number of cubic segments** in the final Bézier is
  `(newspl->size - 1) / 3 = (end - start)/3 + 1`. `ED_spl(e)->size` (number of
  Béziers in the list) is 1 after a normal routing; multi-Bézier `ED_spl`
  lists only arise from multi-spline routing paths (e.g. `makeMultiSpline`,
  orthogonal router) or `new_spline` being called repeatedly on the same edge.
* The allocated `list` array keeps the original `pn` capacity
  (`new_spline(fe, pn)`, splines.c:244) but `size` is shrunk to
  `end-start+4`; entries `> size` are stale.
* **Arrow adjustments are done *inside* `clip_and_install`** (via
  `arrow_clip`, §5.3), not by the caller. The caller passes the *unclipped*
  router output; the tail/head shape clipping and arrow trimming both mutate
  `ps` in place and adjust `start`/`end`.
* The whole `ps` array (length pn) is mutated in place by shape/arrow
  clipping before the copy; the copy loop is the only thing that reads
  `start..end+3` afterwards.
* The point at `end+3` (the final endpoint) is written into
  `newspl->list[end-start+3]` at the top of the last loop iteration (the
  `break` happens right after copying it) — make sure the port copies it.
* `update_bb_bz` is called once per cubic segment, with `cp` = the segment's
  four control points (shared endpoints included).

### 5.3 `arrow_clip(fe, hn, ps, &start, &end, spl, info)` — splines.c:64-98

```text
e = fe; while ED_to_orig(e): e = ED_to_orig(e)              (72)
j = info->ignoreSwap ? false : info->swapEnds(e)            (74-77)
arrow_flags(e, &sflag, &eflag)                              (79)  // arrows.c:218
if info->splineMerge(hn):        eflag = ARR_NONE           (80-81)
if info->splineMerge(agtail(fe)): sflag = ARR_NONE          (82-83)
if j: SWAP(sflag, eflag)                                    (85-87)
if info->isOrtho:
    if eflag || sflag: arrowOrthoClip(e, ps, start, end, spl, sflag, eflag) (88-91)
else:
    if sflag: start = arrowStartClip(e, ps, start, end, spl, sflag)         (93-94)
    if eflag: end   = arrowEndClip(e, ps, start, end, spl, eflag)           (95-96)
```

dot's `sinfo` (`dotsplines.c:108-127`): `swapEnds = swap_ends_p` (true when,
on the original edge, head rank < tail rank, or same rank and head order <
tail order), `splineMerge = n => ND_node_type(n)==VIRTUAL && (in>1||out>1)`,
`ignoreSwap=false`, `isOrtho=false`. So dot always takes the
`arrowStartClip`/`arrowEndClip` branch.

`arrowStartClip` / `arrowEndClip` (arrows.c:313-339 / 285-311) — this is where
`ED_spl->list[0].sp/ep` and `sflag/eflag` come from:

```text
arrowStartClip(e, ps, startp, endp, spl, sflag):
    spl->sflag = sflag; spl->sp = ps[startp]            // pre-clip tip anchor (318-319)
    if endp > startp && DIST(ps[startp], ps[startp+3]) < slen: startp += 3
    sp[0..3] = (ps[startp+3], ps[startp+2], ps[startp+1], spl->sp)  // reversed
    if slen > 0: bezier_clip(ctx, inside=circle(sp[3], r=slen), sp, left_inside=false)
    ps[startp..startp+3] = sp[0..3]
    return startp
```

i.e. the arrow clip shrinks the first (resp. last) cubic so it does not
penetrate a disc of radius `arrow_length(e, flag)` centered on the original
endpoint, using `bezier_clip` (§5.4) with the `.a` inside context (circle
test, arrows.c:280-283). `arrowOrthoClip` (arrows.c:350+) instead shortens
the single horizontal/vertical end segment(s), possibly shrinking arrow
lengths to `d/3` when both arrows share one segment.

### 5.4 `bezier_clip(inside_context, inside, sp, left_inside)` — splines.c:107-151

Binary search clipping of one cubic to a region.

```text
if left_inside:  pt = sp[0]; left=NULL; right=seg; idir=&low;  odir=&high
else:            pt = sp[3]; left=seg;  right=NULL; idir=&high; odir=&low
low = 0.0; high = 1.0; found = false
do {
    opt = pt
    t = (high + low) / 2.0
    pt = Bezier(sp, t, left, right)        // seg receives the kept half
    if inside(ctx, pt): idir = t; best = seg; found = true
    else:               odir = t
} while (|opt.x - pt.x| > .5 or |opt.y - pt.y| > .5)
sp = best if found else seg
```

Note the loop condition is checked *after* the body, and the convergence
criterion is an absolute 0.5-point move in **both** coordinates between
consecutive iterations. If no sampled point was ever inside, the final `seg`
(the last split at t=0.5... actually at the last t) is used as-is.

### 5.5 `shape_clip0` / `shape_clip` — splines.c:159-209

`shape_clip0` translates the 4-point curve into node-relative coordinates
(minus `ND_coord(n)`), runs `bezier_clip` with
`ND_shape(n)->fns->insidefn(ctx, p)`, translates back, and restores
`ND_rw(n)` (the insidefn is allowed to mutate it; splines.c:167,179).

`shape_clip(n, curve)` (public, used by `makeSpline`-less callers and shapes)
first tests `insidefn(curve[0] - coord)` to decide `left_inside` (204-207) —
with the documented feedback caveat in the comment at splines.c:182-192.

The `insidefn` contract: `p` is in node-local coordinates (y up);
`inside_t.s.n` is the node, `.s.bp` is the edge's port box (`ED_*_port.bp`) or
NULL. Implementations live per shape in `lib/common/shapes.c`
(`poly_inside`, `record_inside`, `point_inside`, ...). For a minimal Rust port
rendering only box/ellipse-ish nodes, a rectangle/capsule inside test that
matches `poly_inside` for rectangles suffices — but consult shapes.c before
committing to any specific behavior.

### 5.6 `add_box(P, b)` — splines.c:336-340

Appends `b` to `P->boxes[P->nbox++]` **only if** `b.LL.x < b.UR.x &&
b.LL.y < b.UR.y` (strictly non-degenerate). All dot box assembly goes through
this.

### 5.7 `beginpath` / `endpath` — splines.c:375-571 / 573-769

Build `pathend_t` (node box + up to 20 end boxes) and set `P->start` / `P->end`.
Summarized contract (details matter only if you port dot's box construction):

* `P->start.p = ND_coord(tail) + ED_tail_port(e).p` (390); `P->end.p`
  analogously (587).
* `merge` (edge concentrated into a virtual merge node):
  `P->start.theta = conc_slope(agtail(e))` (393);
  `P->end.theta = conc_slope(aghead(e)) + M_PI` (590); `constrained = true`.
* Otherwise `theta`/`constrained` come from the port (`ED_*_port`), 396-401 /
  594-599.
* If the port has an explicit side (`ED_tail_port(e).side != 0`) and the edge
  is REGULAREDGE between NORMAL nodes (405-471) or FLATEDGE (472-538), hand
  built 1- or 2-box configurations are emitted per side (TOP/BOTTOM/LEFT/
  RIGHT), with exact coordinates involving `ND_lw/ND_rw`, `HT2(n)`,
  `GD_ranksep/2`, and `FUDGE-2` gaps; `P->start.p`/`P->end.p` are nudged by
  ±1 in x or y so the endpoint lies strictly inside the polygon (reason
  stated in the comment at 350-356: `Proutespline` gets confused when the
  start point lies exactly on the polygon). The corresponding port on `orig`
  gets `clip = false` (464-469 etc.).
* Default path (no side): `endp->boxes[0] = endp->nb; endp->boxn = 1` (546-547)
  then per edge type: REGULAREDGE ⇒ `boxes[0].UR.y = P->start.p.y`,
  `sidemask = BOTTOM`, `P->start.p.y -= 1` (564-568); FLATEDGE ⇒ adjust LL.y or
  UR.y per sidemask (558-563); SELFEDGE ⇒ `assert(0)` (554) — begin/endpath are
  never used for self edges.
* If a `pboxfn` exists it replaces the default boxes (542-544) and returns the
  sidemask.

`FUDGE = 2` (372) is only used in the side-specific branches.

### 5.8 Self edges — splines.c:772-1200

`makeSelfEdge(edges, cnt, sizex, sizey, sinfo)` (1162-1200) dispatches to
`selfRight` / `selfLeft` / `selfTop` / `selfBottom` based on the port sides of
the first edge (right-biased; see comment 1157-1161). Each generator builds a
7-point control array (three cubic segments: leave tail, run around the node,
enter head) with per-iteration offsets `dx/dy += step*`, label placement
(`ED_label(e)->pos`, `height/width > stepy` slack), and calls
`clip_and_install(e, aghead(e), points, 7, sinfo)` (869, 976, 1047, 1122).
`convert_sides_to_points(tail_side, head_side)` (772-805) maps side-bit
combos to a "point pair" code (vertices table `{12,4,6,2,3,1,9,8}` at 774)
used for special-case tweaks (e.g. `case 67: sgn = -sgn` at 834).
`selfRightSpace(e)` (1137-1155) computes the horizontal space to reserve.

### 5.9 Labels and endpoints — splines.c:1203-1372

* `addEdgeLabels(e)` = `makePortLabels(e)` (1305-1307); only active if
  `labelangle`/`labeldistance` exist (1208).
* `place_portlabel(e, head_p)` (1314-1359): anchor `pe` = `sp`/`ep` if the
  corresponding flag is set else the raw endpoint; second reference point
  `pf` = the other endpoint, or `Bezier(first 4 pts, 0.1)` /
  `Bezier(last 4 pts, 0.9)`; position = `pe + PORT_LABEL_DISTANCE *
  late_double(labeldistance,1) * (cos,sin)(atan2(pf-pe) +
  RADIANS(late_double(labelangle, PORT_LABEL_ANGLE, -180)))`.
* `endPoints(spl,&p,&q)` (1221-1239): `p` = `list[0].sp` if `sflag` else
  `list[0].list[0]`; `q` = `list[size-1].ep` if `eflag` else
  `list[size-1].list[bz.size-1]`.
* `polylineMidpoint(spl,&pp,&pq)` (1244-1278): total length = Σ over Béziers
  and segments of `DIST(list[j], list[j+3])` (chord length! j,k stride 3);
  walk again accumulating `d`, and when `d >= dist/2` return the linear
  interpolation on the chord `pf..qf` at distance `dist` (remaining half):
  `mf = (qf*dist + pf*(d-dist)) / d`. Never returns (UNREACHABLE) because
  `dist` was halved.
* `edgeMidpoint(g, e)` (1280-1299): degenerate (endpoints equal within
  MILLIPOINT) ⇒ endpoint; `EDGETYPE_SPLINE/CURVED` ⇒
  `dotneato_closest(ED_spl(e), midpoint(p,q))`; PLINE/ORTHO/LINE ⇒
  `polylineMidpoint`.
* `getsplinepoints(e)` (1361-1372): walks `ED_to_orig` until a spline is
  found; errors if none.

---

## 6. `lib/common/routespl.c`

### 6.1 Module state — routespl.c:32-35

```c
static int nedges;      // total edges routed (since routesplinesinit)
static size_t nboxes;   // total boxes routed
static int routeinit;   // recursion guard for init/term
```

`routesplinesinit()` (211-222): `if (++routeinit > 1) return 0;` then reset
counters, start timer if `Verbose`. Returns 0 always.
`routesplinesterm()` (224-229): `if (--routeinit > 0) return;` then
`GV_DEBUG("routesplines: %d edges, %zu boxes %.2f sec", ...)`.

### 6.2 `checkpath(boxn, boxes, thepath)` — routespl.c:631-760

Validates and **repairs in place** dot's channel boxes; returns 1 on fatal
error (after `agerrorf` + `printpath`), 0 on success.

```text
// 1. drop degenerate boxes (compact in place)                (636-645)
i = 0
for bi in 0..boxn:
    if |LL.y - UR.y| < .01: continue
    if |LL.x - UR.x| < .01: continue
    boxes[i++] = boxes[bi]
boxn = i
if boxn == 0: error "all bounding boxes are below threshold"; return 1

// 2. sanity                                                  (653-666)
if boxes[0].LL > boxes[0].UR in x or y: error "box 0 has LL coord > UR coord"; return 1
for bi in 0..boxn-2:
    if boxes[bi+1] LL>UR: error "box bi+1 has LL coord > UR coord"; return 1

// 3. adjacency repair                                        (659-704)
for bi in 0..boxn-2:
    ba, bb = boxes[bi], boxes[bi+1]
    l = (ba.UR.x < bb.LL.x)        // ba entirely left of bb  ("don't touch")
    r = (ba.LL.x > bb.UR.x)        // ba entirely right of bb
    d = (ba.UR.y < bb.LL.y)        // ba below bb
    u = (ba.LL.y > bb.UR.y)        // ba above bb
    errs = l + r + d + u
    if errs > 0:   // (Verbose prints "boxes i and i+1 don't touch" first, 672-678)
        // first violating axis: swap the offending coordinates exactly (682-689)
        if l: swap(ba.UR.x, bb.LL.x)
        elif r: swap(ba.LL.x, bb.UR.x)
        elif d: swap(ba.UR.y, bb.LL.y)
        elif u: swap(ba.LL.y, bb.UR.y)
        // remaining errs-1 violations: meet at midpoint + 0.5 (690-703)
        repeat errs-1 times:
            if l:   ba.UR.x = bb.LL.x = (ba.UR.x + bb.LL.x)/2.0 + 0.5
            elif r: ba.LL.x = bb.UR.x = (ba.LL.x + bb.UR.x)/2.0 + 0.5
            elif d: ba.UR.y = bb.LL.y = (ba.UR.y + bb.LL.y)/2.0 + 0.5
            elif u: ba.LL.y = bb.UR.y = (ba.LL.y + bb.UR.y)/2.0 + 0.5

    // 4. overlap resolution                                    (706-738)
    xoverlap = overlap(ba.LL.x, ba.UR.x, bb.LL.x, bb.UR.x)
    yoverlap = overlap(ba.LL.y, ba.UR.y, bb.LL.y, bb.UR.y)
    if xoverlap > 0 && yoverlap > 0:
        if xoverlap < yoverlap:   // resolve on x
            if width(ba) > width(bb):   // take space from wider box
                if ba.UR.x < bb.UR.x: ba.UR.x = bb.LL.x else ba.LL.x = bb.UR.x
            else:
                if ba.UR.x < bb.UR.x: bb.LL.x = ba.UR.x else bb.UR.x = ba.LL.x
        else:                     // symmetric on y  (723-737)

// 5. clamp endpoints into first/last box                      (741-758)
if P->start.p outside boxes[0]:  clamp each coordinate with fmin/fmax
if P->end.p   outside boxes[boxn-1]: clamp likewise
return 0
```

`overlap(i0,i1,j0,j1)` (routespl.c:603-620): 0 if disjoint; otherwise the
length of the intersection, computed via subsumption cases in this exact
order: (1) `i0<=j0 && i1>=j1` → `i1-i0`; (2) `j0<=i0 && j1>=i1` → `j1-j0`;
(3) `j0<=i0<=j1` → `j1-i0`; (4) assert + `i1-j0`.

Note: `checkpath` mutates `pp->boxes`, `pp->start.p`, `pp->end.p` — these
mutations persist and are how dot shrinks/repairs channels for later edges.

### 6.3 `limitBoxes(boxes, boxn, pps, pn, delta)` — routespl.c:231-268

Bound each box's x-extent by sampling the Bézier polyline `pps`
(3-stride control points).

```text
num_div = delta * boxn                     (235)
for splinepi in 0,3,6,... while splinepi+3 < pn:       // each cubic segment (237)
    for si in 0..=num_div:                             // integer-valued double
        t = si / num_div
        sp = (pps[splinepi..splinepi+3])               // copy 4 ctrl points
        // one de Casteljau level at t, three times:   (244-255)
        sp0 += t*(sp1-sp0); sp1 += t*(sp2-sp1); sp2 += t*(sp3-sp2)
        sp0 += t*(sp1-sp0); sp1 += t*(sp2-sp1)
        sp0 += t*(sp1-sp0)                             // sp[0] = curve point at t
        for bi in 0..boxn:
            if sp0.y <= boxes[bi].UR.y + FUDGE && sp0.y >= boxes[bi].LL.y - FUDGE:
                boxes[bi].LL.x = min(LL.x, sp0.x)
                boxes[bi].UR.x = max(UR.x, sp0.x)
```

`FUDGE = .0001` (259). Only x is bounded; y selects the box. Sampling includes
both endpoints of every segment (`si` from 0 to `num_div` inclusive ⇒
`num_div+1` samples).

### 6.4 `routesplines_(pp, npoints, polyline)` — routespl.c:290-593

The core. `routesplines(pp,&n)` ≡ `routesplines_(pp,&n,0)` (595-597);
`routepolylines(pp,&n)` ≡ `routesplines_(pp,&n,1)` (599-601).
Returns a malloc'd array of `*npoints` control points (3k+1 of them), or NULL
with `*npoints = 0` on any failure.

```text
*npoints = 0; nedges++; nboxes += pp->nbox                          (301-303)

realedge = pp->data, walk ED_to_orig until NORMAL                   (305-307)
if !realedge: error "cannot find NORMAL edge"; return NULL          (308-311)

boxes = pp->boxes; boxn = pp->nbox                                  (313-314)
if checkpath(boxn, boxes, pp): return NULL                          (316-317)

polypoints = alloc(8 * boxn)                                        (329)

// ---- y-flip if the channel runs upward -------------------------- (331-339)
flip = false
if boxn > 1 && boxes[0].LL.y > boxes[1].LL.y:
    flip = true
    for bi in 0..boxn:   // mirror about y=0: (LL,UR).y -> (-UR, -LL)
        v = boxes[bi].UR.y; boxes[bi].UR.y = -boxes[bi].LL.y; boxes[bi].LL.y = -v

// ---- build the channel polygon ---------------------------------- (341-426)
if agtail(realedge) == aghead(realedge):
    error "edge is a loop at %s"; return NULL                        (421-426)
pi = 0

// forward pass: left boundary, bi = 0 .. boxn-1                     (346-378)
for bi in 0..boxn-1:
    prev = (bi > 0)        ? (boxes[bi].LL.y > boxes[bi-1].LL.y ? -1 : +1) : 0
    next = (bi+1 < boxn)   ? (boxes[bi+1].LL.y > boxes[bi].LL.y ? +1 : -1) : 0
    if prev != next:
        if next == -1 || prev == 1:
            emit (LL.x, UR.y); emit (LL.x, LL.y)          // down the left side
        else:
            emit (UR.x, LL.y); emit (UR.x, UR.y)          // up the right side
    elif prev == 0:      // single box
        emit (LL.x, UR.y); emit (LL.x, LL.y)
    else:                // prev == next != 0 : stacked boxes, must go down
        assert prev == -1 && next == -1, else
            error "illegal values of prev %d and next %d, line %d"; return NULL
        // emit nothing in forward pass

// backward pass: right boundary, bi = boxn-1 .. 0                    (379-420)
for bi in boxn-1 .. 0:
    prev = (bi+1 < boxn) ? (boxes[bi].LL.y > boxes[bi+1].LL.y ? -1 : +1) : 0
    next = (bi > 0)      ? (boxes[bi-1].LL.y > boxes[bi].LL.y ? +1 : -1) : 0
    if prev != next:
        if next == -1 || prev == 1: emit (LL.x, UR.y); emit (LL.x, LL.y)
        else:                       emit (UR.x, LL.y); emit (UR.x, UR.y)
    elif prev == 0:     // single box
        emit (UR.x, LL.y); emit (UR.x, UR.y)
    else:
        assert prev == -1 && next == -1 else error(...)  // return NULL
        // degenerate/stacked box: traverse it fully (411-418)
        emit (UR.x, LL.y); emit (UR.x, UR.y); emit (LL.x, UR.y); emit (LL.x, LL.y)

// ---- un-flip ------------------------------------------------------ (428-436)
if flip:
    restore boxes[bi].y exactly as before (UR.y = -LL.y; LL.y = -old UR.y)
    for i in 0..pi: polypoints[i].y *= -1
// Net effect: boxes and endpoints are in original coordinates; the polygon
// was built in mirrored space and mirrored back. Vertex ORDER is preserved.

// ---- reset box x extents and run shortest path -------------------- (438-451)
for bi: boxes[bi].LL.x = DBL_MAX; boxes[bi].UR.x = -DBL_MAX
poly = { polypoints, pi }
eps = (pp->start.p, pp->end.p)
if Pshortestpath(&poly, eps, &pl) < 0:
    error "Pshortestpath failed"; return NULL

// ---- convert polyline to Bézier control points -------------------- (459-490)
if polyline:  make_polyline(pl, &spl)                     // degenerate cubics
else:
    edges[i] = (polypoints[i], polypoints[(i+1) % pi])    // polygon edges as barriers
    evs[0] = pp->start.constrained ? ( cos(start.theta),  sin(start.theta)) : (0,0)
    evs[1] = pp->end.constrained   ? (-cos(end.theta),   -sin(end.theta))   : (0,0)
    if Proutespline(edges, pi, pl, evs, &spl) < 0:
        error "Proutespline failed"; return NULL
ps = copy of spl.ps (spl.pn entries)                      (491-501)

// ---- reclaim box x-space ------------------------------------------ (503-579)
is_horizontal = (spl.pn > 0) && all i: |ps[0].y - ps[i].y| <= FUDGE  (505-511)
is_vertical   = (spl.pn > 0) && all i: |ps[0].x - ps[i].x| <= FUDGE  (519-525)
unbounded = true
if is_horizontal || is_vertical:                       (534-540)
    for bi: boxes[bi].LL.x = boxes[bi].UR.x = ps[0].x
    unbounded = false
delta = INIT_DELTA
for loopcnt in 0..LOOP_TRIES while unbounded:          (545-565)
    limitBoxes(boxes, boxn, ps, spl.pn, delta)
    for bi in 0..boxn:
        if is_exactly_equal(boxes[bi].LL.x, DBL_MAX) ||
           is_exactly_equal(boxes[bi].UR.x, -DBL_MAX):
            delta *= 2; break                          // retry, finer samples
    if all boxes bounded: unbounded = false
if unbounded:                                          (566-579)
    warn "Unable to reclaim box space in spline routing for edge ..."
    make_polyline(pl, &polyspl)                        // fall back to shortest path
    limitBoxes(boxes, boxn, polyspl.ps, polyspl.pn, INIT_DELTA)

*npoints = spl.pn                                      (581)
free(polypoints); return ps                            (591-592)
```

Notes that matter for exact reproduction:

* The polygon builder's `prev`/`next` semantics: `+1` = "the box in that
  direction has strictly greater LL.y" (higher on screen), `-1` = lower,
  `0` = nonexistent. The forward pass walks the *left* boundary top→bottom,
  the backward pass the *right* boundary bottom→top (with the flip making
  "down" canonical). Comparison is strict `>`: equal LL.y ⇒ `-1`.
* The polygon is emitted with **duplicated corner points** in the
  single-box/degenerate cases (a closed polygon with no explicit closing
  edge; `Proutespline`'s barrier list closes it with the wrap-around edge
  `(polypoints[pi-1], polypoints[0])`).
* `pp->boxes` are permanently **shrunk in x** by this routine (that is the
  documented space-reclaiming mechanism, routespl.c:283-286); y values are
  restored exactly after the flip.
* The horizontal/vertical detection uses `FUDGE=.0001` on the *control
  points* (not sampled curve points).
* `is_exactly_equal` is bitwise (gv_math.h:51), so a box touched by
  `limitBoxes` can never compare equal to the sentinels again even if the
  value coincides numerically.
* On every failure path the router returns NULL and `*npoints=0`; the dot
  callers then simply skip the edge (no spline attached): see
  dotsplines.c:1415-1418, 1489-1492, 1816-1822, 1869-1874
  (`free(ps); return;`). Additionally, for `EDGETYPE_LINE` a polyline result
  longer than one segment is straightened in place:
  `ps[1]=ps[0]; ps[3]=ps[2]=ps[pn-1]; pn=4` (dotsplines.c:1810-1814, 1860-1868).
* The returned buffer is freshly allocated (`calloc`, routespl.c:491) and
  owned by the caller; the dot callers `free(ps)` after
  `clip_and_install` (e.g. dotsplines.c:1420-1422, 1493-1494, 1878).

### 6.5 `simpleSplineRoute(tp, hp, poly, n_spl_pts, polyline)` — routespl.c:168-206

Standalone helper used by dot for labeled flat edges (dotsplines.c:1025,
1069) where an ad-hoc 8-point polygon (`poly`) is built around the labels:

```text
eps = (tp, hp)
if Pshortestpath(&poly, eps, &pl) < 0: return NULL
if polyline: make_polyline(pl, &spl)
else:
    edges[i] = (poly.ps[i], poly.ps[(i+1) % poly.pn])    // polygon edges
    if Proutespline(edges, poly.pn, pl, (Pvector_t[2]){0}, &spl) < 0:
        return NULL                                      // zero endpoint slopes!
ps = copy of spl.ps; *n_spl_pts = spl.pn; return ps
```

Difference vs `routesplines_`: no boxes/checkpath/flip/limitBoxes, and
endpoint slope vectors are **zero** (unconstrained).

### 6.6 `makeStraightEdge` / `makeStraightEdges` — routespl.c:954-1036

Fallback when no routing is wanted/possible (`makeStraightEdge` walks
`ED_to_virt` to collect the multi-edge chain, 957-967).

```text
dumb[0]=dumb[1] = tail coord + tail port
dumb[2]=dumb[3] = head coord + head port                       (982-983)
if e_cnt == 1 || Concentrate:                                   (984-990)
    if curved (EDGETYPE_CURVED): bend(dumb, get_cycle_centroid(g, e))  (985-986)
    clip_and_install(e, head, dumb, 4, sinfo); addEdgeLabels(e); return
if APPROXEQPT(dumb[0], dumb[3], MILLIPOINT):                    // degenerate (992-997)
    dumb[1] = dumb[0]; dumb[2] = dumb[3]; del = (0,0)
else:
    perp = (dumb[0].y - dumb[3].y, dumb[3].x - dumb[0].x)       // perpendicular (999)
    xstep = GD_nodesep(g->root)
    dx = xstep * (e_cnt-1) / 2
    dumb[1] = dumb[0] + dx * perp/|perp|; dumb[2] = dumb[3] + dx * perp/|perp|
    del = -xstep * perp/|perp|
for i in 0..e_cnt:
    e0 = edge_list[i]
    dumber = dumb (reversed if aghead(e0) != head)              (1015-1023)
    if et == EDGETYPE_PLINE:
        make_polyline({dumber,4}, &spl); clip_and_install(e0, ..., spl.ps, spl.pn, ...)
    else: clip_and_install(e0, ..., dumber, 4, ...)
    addEdgeLabels(e0)
    dumb[1] += del; dumb[2] += del                              (1033-1034)
```

`bend(spl[4], centroid)` (936-951): moves both middle control points to the
point at distance `dist(p0,p3)/5` from the midpoint of `p0..p3`, in the
direction *away* from `centroid` (`a = mid - v/|v| * r`, v = mid-centroid);
no-op if `magV == 0` (bitwise zero test). The cycle machinery
(`find_all_cycles`, `dfs`, `cycle_contains_edge`, `is_cycle_unique`,
`find_shortest_cycle_with_edge`, `get_cycle_centroid`, routespl.c:784-934)
finds the centroid of the shortest cycle (length ≥ 3) containing the edge, or
the graph bounding-box center if none.

### 6.7 Misc

* `printpath(pp)` (762-774) — debug dump of boxes and both ports including
  the literal words "constrained" / "not constrained".
* Debug PostScript emitters (40-165) are `#ifdef DEBUG` only.

---

## 7. `lib/pathplan/shortest.c` — `Pshortestpath` (funnel)

### 7.1 Local structures — shortest.c:31-53

```c
typedef struct pointnlink_t { Ppoint_t *pp; struct pointnlink_t *link; } pointnlink_t;
typedef struct { pointnlink_t *pnl0p, *pnl1p; size_t right_index; } tedge_t;
typedef struct { int mark; tedge_t e[3]; } triangle_t;
typedef struct { pointnlink_t **pnlps; size_t pnlpn, fpnlpi, lpnlpi, apex; } deque_t;
static LIST(triangle_t) tris;      // file-static triangle list (cleared per call)
static Ppoint_t *ops; static size_t opn;   // file-static output buffer
```

`right_index` indexes `tris` for the triangle sharing that edge
(`SIZE_MAX` = none). **`mark` values: 0 unvisited, 1 = on the strip path
(DFS result), 2 = consumed by the funnel walk** (set at shortest.c:237, 383).

### 7.2 `Pshortestpath(polyp, eps, output)` — shortest.c:83-314

Returns 0 on success, −1 bad input (endpoint in no triangle), −2 alloc
failure. Emits to the file-static `ops` buffer (reused every call).

```text
// allocation; deque capacity 2*pn, front starts at mid        (93-116)
pnls   = calloc(pn); pnlps = calloc(pn)
LIST_CLEAR(&tris)
dq.pnlps = calloc(2*pn); dq.fpnlpi = pn; dq.lpnlpi = pn - 1

// orientation: make point order CCW-per-this-code, drop dups   (119-148)
find minpi = argmin polyp->ps[i].x  (strict <; first index wins ties)
p2 = ps[minpi]; p1 = ps[minpi-1 or pn-1]; p3 = ps[(minpi+1)%pn]
reverse = (p1.x==p2.x==p3.x && p3.y>p2.y) || ccw(p1,p2,p3) != ISCCW
if reverse: for pi = pn-1 downto 0:  skip if ps[pi]==ps[pi+1] (dup) else load
else:       for pi = 0 .. pn-1:      skip if ps[pi]==ps[pi-1] (dup) else load
load = append pointnlink_t{pp=&ps[pi], link=&pnls[pnll % pn]} to pnls/pnlps

// triangulation (ear clipping)                                 (157-162)
if triangulate(pnlps, pnll): return -2

// connect shared edges: O(T^2) pairwise scan                   (173-175)
for trii < T, trij in trii+1..T: connecttris(trii, trij)

// locate endpoints                                             (178-199)
ftrii = first trii with pointintri(trii, eps[0])   else prerror("source point not
        in any triangle"); return -1
ltrii = first trii with pointintri(trii, eps[1])   else ... "destination point
        ..."; return -1

// mark strip of triangles from ftrii to ltrii (DFS)            (202-214)
if !marktripath(ftrii, ltrii):
    // "a straight line is better than failing"
    growops(2); output = {pn:2, ps:[eps[0], eps[1]]}; return 0

// same triangle => straight line                               (217-227)
if ftrii == ltrii: growops(2); output = {2, [eps[0],eps[1]]}; return 0

// funnel                                                       (230-288)
epnls[0] = {&eps[0], NULL}; epnls[1] = {&eps[1], NULL}
add2dq(&dq, DQ_FRONT, &epnls[0])          // deque = [eps0]
dq.apex = dq.fpnlpi
trii = ftrii
while trii != SIZE_MAX:
    trip = &tris[trii]; trip->mark = 2
    // find exiting edge: one whose right neighbor is still marked 1  (240-242)
    for ei in 0..2:
        if trip->e[ei].right_index != SIZE_MAX && tris[right].mark == 1: break
    if ei == 3:   // last triangle: pair deque end with eps[1]       (243-248)
        if ccw(eps[1], *front, *back) == ISCCW:
            lpnlp = back; rpnlp = &epnls[1]
        else:
            lpnlp = &epnls[1]; rpnlp = back           // NOTE: back, not front!
        // front = dq.pnlps[dq.fpnlpi], back = dq.pnlps[dq.lpnlpi]
    else:         // third vertex of the triangle decides sides     (249-256)
        pnlp = trip->e[(ei+1)%3].pnl1p
        if ccw(*e[ei].pnl0p->pp, *pnlp->pp, *e[ei].pnl1p->pp) == ISCCW:
            lpnlp = e[ei].pnl1p; rpnlp = e[ei].pnl0p
        else:
            lpnlp = e[ei].pnl0p; rpnlp = e[ei].pnl1p
    // deque update                                                  (259-281)
    if trii == ftrii:
        add2dq(DQ_BACK, lpnlp); add2dq(DQ_FRONT, rpnlp)
    elif front != rpnlp && back != rpnlp:       // right point is new
        splitindex = finddqsplit(&dq, rpnlp)
        splitdq(&dq, DQ_BACK, splitindex)       // fpnlpi = splitindex (cut front)
        add2dq(DQ_FRONT, rpnlp)
        if splitindex > dq.apex: dq.apex = splitindex
    else:                                       // left point is new
        splitindex = finddqsplit(&dq, lpnlp)
        splitdq(&dq, DQ_FRONT, splitindex)      // lpnlpi = splitindex (cut back)
        add2dq(DQ_BACK, lpnlp)
        if splitindex < dq.apex: dq.apex = splitindex
    // advance to next strip triangle                               (282-287)
    trii = SIZE_MAX
    for ei in 0..2:
        if e[ei].right_index != SIZE_MAX && tris[right].mark == 1:
            trii = e[ei].right_index; break

// output: walk link chain from epnls[1], fill ops in reverse      (297-309)
i = count of chain(epnls[1])           // includes eps[1] .. eps[0]
growops(i); output->pn = i
for (i--; pnlp = &epnls[1]; pnlp = pnlp->link): ops[i--] = *pnlp->pp
return 0
```

Helpers:

* `add2dq(dq, side, pnlp)` (395-407): FRONT ⇒ if deque non-empty
  (`lpnlpi >= fpnlpi`) set `pnlp->link = pnlps[fpnlpi]` (current front);
  `fpnlpi--`; `pnlps[fpnlpi] = pnlp`. BACK ⇒ mirrored (`link` to current
  back, `lpnlpi++`). **The link chain is the shortest-path parent structure;
  the same `pointnlink_t` object may re-enter the deque many times and its
  `link` is overwritten each time — a Rust port must model this aliasing**
  (e.g. `Vec<Cell<usize>>` or an arena of `Rc<RefCell<...>>`).
* `splitdq(dq, side, index)` (409-414): FRONT ⇒ `lpnlpi = index` (truncate
  back); BACK ⇒ `fpnlpi = index` (truncate front). (The naming is
  counterintuitive: `DQ_FRONT` here means "keep the front part".)
* `finddqsplit(dq, pnlp)` (416-424):
  ```text
  for index = fpnlpi ..< apex:
      if ccw(*pnlps[index+1]->pp, *pnlps[index]->pp, *pnlp->pp) == ISCCW:
          return index
  for index = lpnlpi ..> apex (descending):
      if ccw(*pnlps[index-1]->pp, *pnlps[index]->pp, *pnlp->pp) == ISCW:
          return index
  return apex
  ```
* `pointintri(trii, pp)` (426-434): counts edges with
  `ccw(e0, e1, p) != ISCW` (i.e. right turn or collinear); inside iff
  `sum == 3 || sum == 0`. Boundary points (ports on polygon edges) qualify
  via the collinear case; the *first* triangle in `tris` order wins.
* `marktripath(trii, trij)` (378-392): recursive DFS; returns false if
  `mark != 0` already; sets `mark=1`; true when `trii == trij`; otherwise
  tries the three `right_index` neighbors in order 0,1,2; resets `mark=0` on
  failure.
* `connecttris(tri1, tri2)` (360-375): for all 9 edge pairs, if the endpoint
  **pointers** match (either orientation) set mutual `right_index`. Pointer
  equality = same polygon vertex (each `pnls[k].pp` is `&polyp->ps[pi]` for a
  unique pi; duplicates were dropped at load). Port as vertex-index equality.
* `triangulate(points, n)` (317-341): recursive ear clipping:
  ```text
  if n > 3:
      for pnli in 0..n-1:
          pnlip1 = (pnli+1)%n; pnlip2 = (pnli+2)%n
          if isdiagonal(pnli, pnlip2, points, n, point_indexer):
              loadtriangle(points[pnli], points[pnlip1], points[pnlip2])
              remove element pnlip1 from the array (shift left, n-1 remain)
              return triangulate(points, n-1)
      prerror("triangulation failed")     // falls through, returns 0!
  else:
      loadtriangle(points[0], points[1], points[2])
  return 0
  ```
  Note: the failure to find any diagonal only prints an error and returns 0
  (degenerate polygons yield a partial triangulation). The ear removed is at
  vertex `pnli+1`; the recorded triangle is `(i, i+1, i+2)`.
* `loadtriangle(a,b,c)` (343-357): appends a triangle with edges
  (a,b), (b,c), (c,a), all `right_index = SIZE_MAX`.
* `growops(n)` (436-448): realloc the static `ops` to ≥ n points.
* `point_indexer` (73-76): `*b[index]->pp` — lets triang.c's `isdiagonal`
  work on `pointnlink_t**`.

**Output contract:** `output->ps` is the static `ops` buffer — invalidated by
the next `Pshortestpath` call. It contains `eps[0]`, the turning polygon
vertices, `eps[1]` in order. For dot's use this polyline is then fed to
`Proutespline` (or `make_polyline`).

### 7.3 `triang.c` internals used by both triangulators

* `Ptriangulate(polygon, fn, vc)` (triang.c:38-57): public; builds
  `pointp[i] = &ps[i]`, asserts `pn >= 3`, calls the static `triangulate`,
  returns 1 on failure else 0. Each emitted triangle is reported via
  `fn(vc, A)` with `A = {p_i, p_{i+1}, p_{i+2}}`.
* static `triangulate` (63-91): same ear-clipping, but the compaction loop is
  `for (i=0,j=0; i<pointn; i++) if (i != ip1) pointp[j++] = pointp[i];`
  (removes `ip1 = (i+2)%n`... careful: here the ear tip removed is `ip1` where
  `ip1 = (i+1)%n`; the diagonal tested is `(i, ip2=(i+2)%n)`). Returns −1 if
  no diagonal exists (differs from shortest.c's behavior!).
* `between(pa,pb,pc)` (94-101): collinear (`ccw==ISON`) and `pca·pba >= 0`
  and `|pca|² <= |pba|²`.
* `intersects(pa,pb,pc,pd)` (104-120): if any of the 4 `ccw` tests is `ISON`
  ⇒ true iff any `between(...)` holds; else proper-crossing test
  `(ccw1 ^ ccw2) && (ccw3 ^ ccw4)` where `ccwK = (ccw(...) == ISCCW)`.
* `isdiagonal(i, ip2, pointp, pointn, indexer)` (122-150):
  ```text
  ip1 = (i+1)%n; im1 = (i+n-1)%n
  if ccw(p[im1], p[i], p[ip1]) == ISCCW:      // "reflex" per this convention
      res = ccw(p[i], p[ip2], p[im1]) == ISCCW && ccw(p[ip2], p[i], p[ip1]) == ISCCW
  else:                                        // "convex"; assume no collinear nbhd
      res = ccw(p[i], p[ip2], p[ip1]) == ISCW
  if !res: return false
  for j in 0..n-1:                             // vs all other edges
      jp1 = (j+1)%n
      if !(j==i || jp1==i || j==ip2 || jp1==ip2):
          if intersects(p[i], p[ip2], p[j], p[jp1]): return false
  return true
  ```
  This is the classic O(n²)-per-ear test. Both triangulators share it via the
  `indexer_t` function pointer (tri.h:49-53).

---

## 8. `lib/pathplan/route.c` — `Proutespline`

### 8.1 Constants and helpers

`EPSILON1 = 1e-3`, `EPSILON2 = 1e-6` (route.c:21-22). `tna_t { double t;
Ppoint_t a[2]; }` (24-27). `DISTSQ(a,b)` (29-31). Static output state
`Ppoint_t *ops; size_t opn, opl;` (35-36) — **the returned spline lives in
this static buffer and is clobbered by the next `Proutespline` call** (also
shared with nothing else; `Pshortestpath` has its own `ops`).

Bernstein helpers (route.c:462-495):

```text
B0(t) = (1-t)³            B1(t) = 3t(1-t)²
B2(t) = 3t²(1-t)          B3(t) = t³
B01(t) = B0+B1 = (1-t)²(1+2t)   = (1-t)²((1-t)+3t)
B23(t) = B2+B3 = t²(3-2t)       = t²(3(1-t)+t)
```

`normv(v)` (409-419): if `x²+y² > 1e-6`, divide by `sqrt` — else return
unchanged (zero vector stays zero). `add/sub/scale/dot/dist` (431-460) are
the obvious 2D ops; `dist` = `hypot(dx,dy)`.

### 8.2 `Proutespline(barriers, n_barriers, input_route, endpoint_slopes, output_route)` — route.c:70-95

```text
inps = input_route.ps; inpn = (int)input_route.pn      (assert ≤ INT_MAX)
endpoint_slopes[0] = normv(endpoint_slopes[0])         (81-82)
endpoint_slopes[1] = normv(endpoint_slopes[1])
opl = 0
if growops(4) < 0: return -1                           (84-86)
ops[opl++] = inps[0]                                   (87)  // first point
if reallyroutespline(barriers, n_barriers, inps, inpn, slopes[0], slopes[1]) == -1:
    return -1
output_route->pn = opl; output_route->ps = ops         (91-92)
return 0
```

The input polyline is the **shortest path** (not the `make_polyline`-expanded
version). The output is a 3k+1 control-point array: `ops[0] = inps[0]`, then
each fitted cubic appends its last 3 control points, so consecutive segments
share endpoints. Barrier edges are **not modified** ("without touching
barrier segments", pathplan.h:49).

### 8.3 `reallyroutespline(edges, edgen, inps, inpn, ev0, ev1)` — route.c:97-157

Recursive divide-and-conquer Hermite fit.

```text
assert inpn > 0
tnas = calloc(inpn)                                    (104)
tnas[0].t = 0
for i in 1..inpn-1: tnas[i].t = tnas[i-1].t + dist(inps[i], inps[i-1])   (109-110)
for i in 1..inpn-1: tnas[i].t /= tnas[inpn-1].t        (111-112)  // normalize to [0,1]
for i in 0..inpn-1:                                    (113-116)
    tnas[i].a[0] = ev0 * B1(tnas[i].t)     // design column for the start handle
    tnas[i].a[1] = ev1 * B2(tnas[i].t)     // design column for the end handle
if mkspline(inps, inpn, tnas, ev0, ev1, &p1, &v1, &p2, &v2) == -1: return -1  (117)
fit = splinefits(edges, edgen, p1, v1, p2, v2, inps, inpn)                   (121)
if fit > 0: return 0            // appended; done                       (122-125)
if fit < 0: return -1           // alloc failure                        (126-129)

// fit == 0: reject; split at the input vertex farthest from the LSQ curve
cp1 = p1 + v1/3; cp2 = p2 - v2/3                                       (130-131)
maxi = -1; maxd = -1
for i in 1..inpn-2:                                                    (134-143)
    t = tnas[i].t
    p = ( B0(t)*p1 + B1(t)*cp1 + B2(t)*cp2 + B3(t)*p2 )   // evaluate LSQ curve
    if (d = dist(p, inps[i])) > maxd: maxd = d; maxi = i
spliti = maxi                                                          (145)
splitv1 = normv(inps[spliti]   - inps[spliti-1])                       (146)
splitv2 = normv(inps[spliti+1] - inps[spliti])                         (147)
splitv  = normv(splitv1 + splitv2)                                     (148)
if reallyroutespline(edges, edgen, inps, spliti+1, ev0, splitv) < 0: return -1   (149)
if reallyroutespline(edges, edgen, inps+spliti, inpn-spliti, splitv, ev1) < 0:
    return -1                                                          (152)
return 0
```

* The recursion splits the *closed* sub-polylines: the first gets
  `inps[0..spliti]` (`spliti+1` points) with tangents `(ev0, splitv)`; the
  second `inps[spliti..inpn-1]` with `(splitv, ev1)`. The shared vertex
  `inps[spliti]` lies exactly on both cubics ⇒ the composed curve is
  G1 (tangent-continuous) at every polyline vertex it passes through, but not
  C1 in magnitude.
* `maxi` is only used when `inpn >= 3` (the loop `1..inpn-2` is empty for
  `inpn == 2`, but `splinefits` always succeeds/forces then — see §8.5 — so
  the `spliti = -1` path is unreachable).
* Recursion depth ≤ inpn; each call allocates its own `tnas`.

### 8.4 `mkspline(inps, inpn, tnas, ev0, ev1, &sp0, &sv0, &sp1, &sv1)` — route.c:159-198

Least-squares magnitudes for the two end handles. Model: the cubic
`P(t) = B01(t)·sp0 + B23(t)·sp1 + B1(t)·(sv0/3)·3 - B2(t)·(sv1/3)·3`
with unknown scalars `scale0, scale3` multiplying the *unit* directions —
the code absorbs the 1/3 into the unknowns by using design columns
`a[0] = ev0·B1(t)`, `a[1] = ev1·B2(t)` (so the solved `scale` is
`handle/3`-free; `sv0 = ev0·scale0` is the actual handle *vector* used
directly as the Bézier derivative, i.e. `ctrl1 = sp0 + sv0/3`).

```text
c00 = c01 = c11 = x0 = x1 = 0
for i in 0..inpn-1:                                    (171-180)
    c00 += dot(a0, a0); c01 += dot(a0, a1); c11 += dot(a1, a1)
    tmp = inps[i] - ( inps[0]*B01(t_i) + inps[inpn-1]*B23(t_i) )
    x0 += dot(a0, tmp); x1 += dot(a1, tmp)
c10 = c01
det01 = c00*c11 - c10*c01                              (181)
det0X = c00*x1 - c01*x0                                (182)
detX1 = x0*c11 - x1*c01                                (183)
if fabs(det01) >= 1e-6:
    scale0 = detX1 / det01                             // Cramer's rule (185-186)
    scale3 = det0X / det01
if fabs(det01) < 1e-6 || scale0 <= 0 || scale3 <= 0:   (188-192)
    d01 = dist(inps[0], inps[inpn-1]) / 3.0
    scale0 = scale3 = d01
sp0 = inps[0];        sv0 = ev0 * scale0               (193-196)
sp1 = inps[inpn-1];   sv1 = ev1 * scale3
return 0
```

The fallback (`d01 = chord/3`) fires for degenerate systems (including
`inpn == 2`, where both design columns vanish) and for non-positive scales
(handle pointing "backwards"), and whenever the LSQ solution is not
simultaneously positive.

### 8.5 `splinefits(edges, edgen, pa, va, pb, vb, inps, inpn)` — route.c:212-281

Tries candidate cubics with progressively shorter handles; appends on
success. **Return semantics: 1 = fitted (points appended), 0 = no fit
(caller splits), −1 = alloc failure.**

```text
forceflag = (inpn == 2)                                (220)
a = 4                                                  (222)
first = true
loop:                                                  (223-276)
    sps[0] = pa
    sps[1] = pa + a*va/3
    sps[2] = pb - a*vb/3
    sps[3] = pb
    // "shortcuts not allowed": a candidate whose arc length is shorter than
    // the input path (minus EPSILON1) must be cutting the polygon.  (241-243)
    if first && dist_n(sps,4) < dist_n(inps,inpn) - EPSILON1: return 0
    first = false
    if splineisinside(edges, edgen, sps):              (245-255)
        growops(opl + 4)
        ops[opl..opl+2] = sps[1..3]; opl += 3          // append last 3 points
        return 1
    if a < 0.005:                                      (258-271)
        if forceflag:
            growops(opl + 4); append sps[1..3]; return 1   // forced straight
        break
    if a > .01: a /= 2 else: a = 0                     (272-275)
return 0                                               (280)
```

* Sequence of `a`: 4, 2, 1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625,
  0.0078125, then 0 (since `a ≤ .01` sets `a = 0`), then the `a < 0.005`
  check fires. So at most ~11 candidate cubics per invocation; the final one
  is the straight chord `pa→pb` (`a = 0`).
* `dist_n(p, n)` (200-210) = polyline length `Σ hypot(p[i]-p[i-1])`.
* Note that `splinefits` **ignores the magnitudes of `va`/`vb` computed by
  mkspline except through this `a`-scaling**: effective handles are
  `(a/3)·sv0` and `(a/3)·sv1` with `a` starting at 4. mkspline's scale still
  matters (it sets the direction... no — `sv0 = ev0·scale0` keeps ev0's
  direction; the scale multiplies the handle length. So mkspline's scale0/3
  scale the initial handle lengths, and `splinefits` shrinks from
  `4/3·scale0·ev0`).

### 8.6 `splineisinside(edges, edgen, sps)` — route.c:283-312

```text
for each barrier edge ei (lps = (a,b)):
    rootn = splineintersectsline(sps, lps, roots)
    if rootn == 4: continue            // coincident — ignore
    for each root t of roots[0..rootn):
        if t < EPSILON2 || t > 1 - EPSILON2: continue
        // Bernstein evaluate the spline at t (ta..td as in the code):
        ta=(1-t)³, tb=3t(1-t)², tc=3t²(1-t), td=t³
        ip = ta*sps[0] + tb*sps[1] + tc*sps[2] + td*sps[3]
        if DISTSQ(ip, lps[0]) < EPSILON1 || DISTSQ(ip, lps[1]) < EPSILON1:
            continue                   // touching a barrier endpoint is OK
        return 0                       // genuine crossing → not inside
return 1
```

**Unit caveat:** `EPSILON1 = 1e-3` is compared against a *squared* distance,
so the effective clearance around barrier endpoints is `√1e-3 ≈ 0.0316`
points. Port verbatim.

### 8.7 `splineintersectsline(sps, lps, roots)` — route.c:314-392

Intersect cubic `sps` with segment `lps[0]..lps[1]` (line parametrized
`L(s) = lps[0] + s·(lps[1]-lps[0])`, `s ∈ [0,1]`). Returns the number of
`spline-parameter` roots in `[0,1]` (via `addroot`), or **4** to signal
"degenerate / infinitely many" (caller skips the edge).

Let `xcoeff = (lps[0].x, lps[1].x - lps[0].x)`, `ycoeff` likewise.

* **Both deltas zero** (barrier is a point, 326-348): build the x cubic
  (`points2coeff` of the four x's, minus `xcoeff[0]`) and the y cubic; solve
  both (`solve3`).
  * xrootn==4 && yrootn==4 ⇒ return 4.
  * xrootn==4 ⇒ add all yroots; yrootn==4 ⇒ add all xroots;
  * else add `xroots[i]` for which `xroots[i] == yroots[j]` for some j
    (exact fp equality).
* **Vertical** (`xcoeff[1] == 0`, 349-368): solve the x cubic = x0; if 4
  return 4; for each root `tv ∈ [0,1]`: evaluate the spline's y at `tv`
  (`sv = c0 + tv(c1 + tv(c2 + tv·c3))`), `s = (sv - y0)/ycoeff[1]`; if
  `0 ≤ s ≤ 1` add root `tv`.
* **General** (369-391): `rat = ycoeff[1]/xcoeff[1]`; build cubic
  `y - rat·x` from the four spline points, then `scoeff[0] +=
  rat·xcoeff[0] - ycoeff[0]`; `solve3`; for each root `tv ∈ [0,1]`:
  `sv = (spline_x(tv) - x0)/xcoeff[1]`; keep if `0 ≤ sv ≤ 1`.

`points2coeff(v0,v1,v2,v3, coeff)` (394-401): converts Bézier ordinates to
power-basis `f(t) = c0 + c1·t + c2·t² + c3·t³`:

```text
coeff[3] = v3 + 3*v1 - (v0 + 3*v2)
coeff[2] = 3*v0 + 3*v2 - 6*v1
coeff[1] = 3*(v1 - v0)
coeff[0] = v0
```

`addroot(root, roots, &n)` (403-407): appends only if `0 ≤ root ≤ 1`.

### 8.8 `growops` — route.c:421-429

`if newopn <= opn return 0; ops = realloc(ops, 16*newopn); opn = newopn;`
(POINTSIZE = 16). Returns −1 on failure. `opl` (logical length) is module
state reset by `Proutespline`.

### 8.9 Error returns summary (route.c)

| Condition | Result |
|---|---|
| `growops` failure (initial or during fit) | `Proutespline` returns −1 |
| `reallyroutespline` recursion failure | −1 |
| `tnas` calloc failure | −1 |
| no fit even at `a=0` for some sub-path (and not forced) | the recursion has no other option ⇒ it still splits; since `spliti` always exists for `inpn ≥ 3` the recursion bottoms out at `inpn == 2` which is forced ⇒ `Proutespline` effectively cannot fail "no fit"; failures are only allocation errors |

---

## 9. `lib/pathplan/solvers.c`

Power-basis polynomial root solver, coefficients ascending
(`coeff[0] + coeff[1]x + ...`). `EPS = 1e-7` (23), `AEQ0(x) = -EPS < x < EPS`
(24).

`solve3(coeff, roots)` (26-67): cubic `a t³ + b t² + c t + d` with
`a=coeff[3], b=coeff[2], c=coeff[1], d=coeff[0]`.

```text
if AEQ0(a): return solve2(coeff, roots)
b3 = b/(3a); ca = c/a; da = d/a
p = b3² ; q = 2*b3*p - b3*ca + da ; p = ca/3 - p
disc = q² + 4p³
if disc < 0:                                   // three real roots (trig form)
    r     = 0.5 * sqrt(-disc + q²)
    theta = atan2(sqrt(-disc), -q)
    temp  = 2 * cbrt(r)
    roots[0] = temp * cos( theta/3 )
    roots[1] = temp * cos( (theta + 2π)/3 )
    roots[2] = temp * cos( (theta - 2π)/3 )
    rootn = 3
else:
    alpha = 0.5*(sqrt(disc) - q)
    beta  = -q - alpha
    roots[0] = cbrt(alpha) + cbrt(beta)
    rootn = 1 if disc > 0 else (roots[1]=roots[2]=-0.5*roots[0], 3)
for i in 0..rootn: roots[i] -= b3              // undo depressed shift
return rootn
```

`solve2` (69-90): `a=coeff[2], b=coeff[1], c=coeff[0]`; `AEQ0(a)` ⇒
`solve1`; `disc = (b/2a)² - c/a`; `<0` ⇒ 0; `>0` ⇒
`roots[0] = -b/2a + sqrt(disc)`, `roots[1] = -2*(b/2a) - roots[0]`
(computed exactly this way), return 2; `==0` ⇒ `roots[0] = -b/2a`, return 1.

`solve1` (92-105): `a=coeff[1], b=coeff[0]`; `AEQ0(a)` ⇒ return 4 if
`AEQ0(b)` else 0; else `roots[0] = -b/a`, return 1.

**Return value 4 means "identically zero — infinitely many roots"**; the
route.c callers treat it as "skip".

---

## 10. `lib/pathplan/util.c`

* `freePath(p)` (19-22): frees `p->ps` then `p`.
* `Ppolybarriers(polys, npolys, barriers, n_barriers)` (24-42): appends every
  polygon edge `(ps[j], ps[(j+1)%pn])` for all polygons, in polygon order;
  `LIST_DETACH` gives the array; `*n_barriers = total`; returns 1 (always).
* `make_polyline(line, sline)` (44-62): **the polyline→Bézier expansion**.
  Static list, cleared per call:
  ```text
  out = [p0, p0]
  for i in 1 .. pn-2:  out += [p_i, p_i, p_i]
  out += [p_{pn-1}, p_{pn-1}]
  ```
  Total `3·pn - 2` points (pn ≥ 2). Read as 3-stride cubics: segment k is
  `(out[3k], out[3k+1], out[3k+2], out[3k+3])` = `(p_k, p_k, p_{k+1}, p_{k+1})`
  — a degenerate cubic tracing input edge k with zero end derivatives
  (ease-in/ease-out parametrization of a straight segment). This is the exact
  representation `routepolylines` produces for `splines=polyline` and the
  one neato's `makePolyline` feeds to `clip_and_install`
  (neatosplines.c:519-527).

---

## 11. `lib/pathplan` visibility stack (`Pobs*` — neato only)

*(Included for completeness; dot does not use it.)*

### 11.1 `visibility.c`

* `allocArray(V, extra)` (26-41): V row pointers into one V×V zeroed block,
  plus `extra` NULL rows (for the two endpoint rows appended in `makePath`).
* `wind`, `area2`, `dist2` — see §4.1. `dist` = `sqrt(dist2)` (133-136).
* `inBetween(a,b,c)` (67-73): strict containment of c in open segment (a,b)
  on the non-degenerate axis.
* `intersect(a,b,c,d)` (80-102): blocks visibility iff `wind(a,b,c)==0 &&
  inBetween(a,b,c)` or `wind(a,b,d)==0 && inBetween(a,b,d)` or
  `w1·w2 < 0 && w3·w4 < 0` where `w1=wind(a,b,c), w2=wind(a,b,d),
  w3=wind(c,d,a), w4=wind(c,d,b)` (proper crossing).
* `in_cone(a0,a1,a2,b)` (108-117): `m = wind(b,a0,a1); p = wind(b,a1,a2);`
  convex at a1 (`wind(a0,a1,a2) > 0`) ⇒ `m ≥ 0 && p ≥ 0`; reflex ⇒
  `m ≥ 0 || p ≥ 0`. Cone is closed.
* `inCone(i,j,pts,nextPt,prevPt)` (138-141): `in_cone(pts[prev[i]], pts[i],
  pts[next[i]], pts[j])`.
* `clear(pti,ptj,start,end,V,pts,nextPt)` (147-162): no polygon edge
  `k ∉ [start,end)` (i.e. `k ∈ [0,start) ∪ [end,V)`) intersects segment
  (pti,ptj) per `intersect`.
* `compVis(conf)` (171-206):
  ```text
  for i in 0..V-1:
      previ = prev[i]; d = dist(P[i], P[previ]); vis[i][previ] = vis[previ][i] = d
      j = (previ == i-1) ? i-2 : i-1
      for j downto 0:
          if inCone(i,j) && inCone(j,i) && clear(P[i], P[j], V, V, V, ...):
              vis[i][j] = vis[j][i] = dist(P[i], P[j])
  ```
  (boundary edges are always visible; other pairs need mutual cone tests and
  a clear segment against **all** edges).
* `visibility(conf)` (213-217): `conf->vis = allocArray(N, 2); compVis(conf);`
* `polyhit(conf, p)` (224-236): first polygon i whose slice
  `P[start[i]..start[i+1])` contains p per `in_poly`.
* `ptVis(conf, pp, p)` (247-299): returns a `V+2` COORD vector; for vertices
  outside the containing polygon's range (`[start,end)` if `pp ≥ 0`,
  resolved via `polyhit` when `pp == POLYID_UNKNOWN`), `vadj[k] = dist(p, P[k])`
  if `in_cone(...)` around vertex k and `clear(p, P[k], start, end, V, ...)`
  else 0; vertices of the containing polygon get 0; `vadj[V] = vadj[V+1] = 0`.
* `directVis(p, pp, q, qp, conf)` (306-355): segment p–q is checked against
  all edges except indices in `[s1,e1) ∪ [s2,e2)` (the two endpoint
  polygons' ranges; a negative id contributes an empty range; ranges are
  ordered so `s1 ≤ s2`). Returns false iff some `intersect(p,q, P[k],
  P[next[k]])` holds.

### 11.2 `cvt.c`

* `Pobsopen(obs, n_obs)` (28-87): allocate `vconfig_t`; `N = Σ pn` (fail if
  > INT_MAX); concat points into `P`; per polygon fill cyclic `next`/`prev`;
  `start[i]` offsets with `start[Npoly] = N`; then `visibility(rv)`. Points
  of each obstacle must be in **clockwise** order (vispath.h:33 comment).
* `Pobsclose(config)` (89-100): frees everything including `vis[0]`/`vis`.
* `Pobspath(config, p0, poly0, p1, poly1, out)` (102-140):
  ```text
  ptvis0 = ptVis(config, poly0, p0); ptvis1 = ptVis(config, poly1, p1)
  dad = makePath(p0, poly0, ptvis0, p1, poly1, ptvis1, config)
  opn = 2 + count of chain from dad[N] to N+1
  ops[opn-1] = p1; walk dad[N], dad[dad[N]], ... writing config->P[i] backwards;
  ops[0] = p0
  output = {opn, ops}
  ```
  (the dad chain encodes the path from the target back to the root; indices
  `V` and `V+1` are the synthetic q and p nodes).

### 11.3 `shortestpth.c`

* `shortestPath(root, target, V, wadj)` (30-81): Dijkstra (Sedgewick 2nd,
  p.466) using **negated values** in `val` so the unsettled set is
  `val[t] < 0`, with sentinel `val[-1] = -(unseen+1)`:
  ```text
  dad[k] = -1; val[k] = -unseen (unseen = INT_MAX)
  min = root
  while min != target:
      k = min; val[k] *= -1; min = -1
      if val[k] == unseen: val[k] = 0          // root start
      for t in 0..V-1 where val[t] < 0:
          wkt = (k >= t) ? wadj[k][t] : wadj[t][k]   // lower triangle only
          newpri = -(val[k] + wkt)
          if wkt != 0 && val[t] < newpri: val[t] = newpri; dad[t] = k
          if val[t] > val[min]: min = t          // min-dist unsettled
  return dad
  ```
  `wkt != 0` (exact fp compare) encodes "no edge". Zero-weight edges are
  invisible to the search.
* `makePath(p, pp, pvis, q, qp, conf)` (93-108): if `directVis(p,pp,q,qp)`
  ⇒ `dad[V] = V+1; dad[V+1] = -1` (direct two-point path); else temporarily
  mount `wadj[V] = qvis; wadj[V+1] = pvis` (the two spare rows from
  `allocArray(N,2)`) and run `shortestPath(V+1, V, V+2, wadj)`.

### 11.4 `inpoly.c`

`in_poly(poly, q)` (26-35): false iff `wind(P[i-1], P[i], q) == 1` for some i
(cyclic). For the documented CW vertex order, inside points never trigger a
`+1` winding. Used by `polyhit` and neato's `makeSpline` endpoint check.

---

## 12. dot-side plumbing (context for reproduction)

* `sinfo` (dotsplines.c:108-127): `swapEnds` per §5.3, `splineMerge` =
  virtual node with in-degree or out-degree > 1; `ignoreSwap=false`,
  `isOrtho=false`.
* Regular edges: `beginpath`/`makeregularend`/`completeregularpath` assemble
  `P->boxes` (tail boxes, rank boxes, per-virtual-node `maximal_bbox` boxes,
  head boxes; added via `add_box` so degenerate boxes are dropped), then
  `routesplines` (splines) or `routepolylines` (pline/line/ortho-fallback)
  at dotsplines.c:1806-1815 and 1854-1859, `clip_and_install` at
  dotsplines.c:1885-1889 (single edge) or per-copy loops for multi-edges.
* Flat edges: `makeFlatEnd` (dotsplines.c:1290), `make_flat_edge` /
  `make_flat_bottom_edges` (1425) call sites dotsplines.c:1411-1420,
  1485-1493; labeled flat edges (`make_flat_labeled_edge`,
  dotsplines.c:1321) use `simpleSplineRoute` with hand-built 8-point
  polygons (dotsplines.c:975-1074).
* Failure handling: every `pn == 0` result simply abandons that edge's spline
  (dotsplines.c:1415-1418, 1489-1492, 1816-1822, 1869-1874); `ED_spl` stays
  NULL and nothing is emitted for the edge.

---

## 13. Reproducing `Proutespline`'s output for dot channel boxes — exact recipe

Given: channel boxes `B[0..n)` (LL/UR, y-up, `B[0]` = tail end), start point
`s` with optional direction `θs`, end point `e` with optional direction `θe`.

1. **checkpath** (§6.2): drop degenerate boxes (< 0.01 thickness), repair
   don't-touch neighbors (swap first, then midpoint+0.5), resolve overlaps
   (smaller-overlap axis, shrink wider box), clamp `s` into `B[0]`, `e` into
   `B[n-1]`. Abort on LL>UR or zero boxes remaining.
2. **Optional y-mirror**: if `n > 1 && B[0].LL.y > B[1].LL.y`, mirror all box
   y's (`y → -y`, with LL/UR swapped by the negation) — build the polygon in
   mirrored space, then mirror polygon vertices back. Endpoints are never
   mirrored (they're restored implicitly because the polygon is).
3. **Polygon** (§6.4): forward pass over boxes emits the left boundary
   (2 points per box; stacked boxes emit nothing in the forward pass when
   `prev == next == -1`), backward pass emits the right boundary (2 points
   per box; stacked boxes emit 4 points there). Error out if
   `prev == next != 0` with anything other than `-1/-1`.
4. **Pshortestpath** (§7): orientation fix (see §7.2), duplicate-point
   removal, ear-clip triangulation in scan order, O(T²) adjacency, DFS strip
   mark, funnel with the exact deque/index/link mechanics. Output polyline
   `pl = [s, vertices..., e]`. Straight-line fallbacks: strip-mark failure ⇒
   `[s,e]`; both endpoints in one triangle ⇒ `[s,e]`.
5. **Barriers**: `E[i] = (poly[i], poly[(i+1) mod m])` for all m polygon
   vertices (including the wrap edge).
6. **Endpoint slopes**: `evs[0] = s.constrained ? (cos θs, sin θs) : (0,0)`;
   `evs[1] = e.constrained ? (−cos θe, −sin θe) : (0,0)`. Then `Proutespline`
   normalizes each with `normv` (zero stays zero).
7. **Proutespline** (§8): `ops[0] = pl[0]`; recursive Hermite fit:
   * arc-length parameter `t` over the input polyline, normalized to [0,1];
   * `mkspline` least-squares handle magnitudes (fallback chord/3);
   * `splinefits` candidate loop `a = 4, 2, 1, ..., 0` with shortcut
     rejection and barrier-intersection test (skip roots within EPSILON2 of
     0/1 and hits within DISTSQ < 1e-3 of barrier endpoints; forced success
     when only 2 input points remain and `a < 0.005`);
   * on rejection, evaluate the LSQ cubic at each input vertex, split at the
     farthest, recurse with the averaged adjacent-edge direction.
8. **Output**: `ops[0..opl)`, `opl = 3k+1`. Feed to `clip_and_install` (§5.2)
   with the raw router points; tail/head insidefn clipping, degenerate
   segment skipping, arrow trimming, and `update_bb_bz` finalize
   `ED_spl->list[0]` (`size = end-start+4`, `sflag/eflag/sp/ep` per §5.3).

**Bit-exactness checklist** (things that silently change output):

* `ccw` sign convention (§4.1) and the strict `.0001` deadband in `wind`.
* `hypot` vs `sqrt(dx²+dy²)`: route.c `dist` uses `hypot`; `dist_n` uses
  `hypot`; shortest.c/visibility.c use `sqrt(dist2)`. Keep each as-is.
* `mkspline` Cramer's rule expression order (`detX1/det01`, `det0X/det01`)
  and the `1e-6` thresholds.
* `splinefits` handle arithmetic `pa + a*va/3.0` (multiply by `a`, then
  divide by 3).
* `limitBoxes` de Casteljau update order (three passes written out at
  routespl.c:244-255 — not a general loop).
* Bitwise-equality sentinels (`DBL_MAX`, `is_exactly_equal` memcmp semantics
  where `+0.0 ≠ -0.0`).
* `solve3`'s exact trigonometric/cbrt formulas and the `roots[i] -= b_over_3a`
  shift; `solve2`'s `roots[1] = -2*b_over_2a - roots[0]`.
* Recursion order: triangulation ear scan (lowest i first), strip DFS edge
  order (0,1,2), funnel deque operations, `finddqsplit` scan order.
* `Proutespline`'s `ops` buffer is static: callers copy or use immediately.

---

## 14. Rust porting notes

1. **Static buffers**: `route.c`'s `ops/opn/opl`, `shortest.c`'s
   `ops/opn/tris`, and `util.c`'s `ispline` are all module-static and
   reused/clobbered. In Rust, return `Vec<Ppoint>` (or document the same
   reuse if you keep a `static mut`-free pool). `Pshortestpath` writing its
   output into a static buffer is observable behavior only through reuse;
   a fresh `Vec` per call is a safe, faithful replacement.
2. **Aliased `link` fields**: funnel `pointnlink_t` nodes are mutated in
   place while reachable from the deque (`Vec<Cell<usize>>` for links, or an
   arena of node indices).
3. **Pointer-identity comparisons** (`connecttris`, `pnls[k].link = &pnls[k
   % pn]`): replace with vertex indices; equality means "same original
   polygon vertex".
4. **Signed/unsigned mixing**: `inpn` is `int` (route.c), `pn` is `size_t`;
   `bi != SIZE_MAX` loops count down; deque indices are `size_t` with
   sentinel `SIZE_MAX`. Prefer `isize`/`usize` carefully around
   `dq.fpnlpi - 1` style arithmetic and the `val[-1]` sentinel (use an
   offset array).
5. **Recursion**: `triangulate` (both variants), `marktripath`, and
   `reallyroutespline` are recursive; depth is bounded by polygon size /
   input length but convert to explicit stacks if you worry about pathological
   inputs.
6. **NaN/inf behavior**: the code assumes finite coordinates; `solve3` with
   extreme coefficients can produce NaNs that then fail `0 ≤ root ≤ 1` tests
   (harmless). Don't add validations that change control flow.
7. **Integer sampling in `limitBoxes`**: `si` is a `double` loop variable;
   `num_div = delta * boxn` may be fractional if `delta` were fractional
   (it never is: 10, 20, 40...). `t = si / num_div` with `si == num_div` ⇒
   `t == 1.0` exactly (inclusive endpoint).
8. **`assert(0)` paths** (beginpath SELFEDGE, splines.c:554) indicate
   unreachable code — keep them as `unreachable!()`/`debug_assert!`.
9. **f32 vs f64**: everything is `double` (f64). Do not downgrade.
10. Testing: build a differential harness feeding hand-made box channels to
    both implementations and comparing (a) `Pshortestpath` polylines, (b)
    `Proutespline` 3k+1 control points, (c) `clip_and_install`'s final
    `bezier.size`/list. Graphviz's own debug output (`showboxes=3`) emits the
    polygon/line/spline as PostScript via `psprintpoly`/`psprintline`/
    `psprintspline` (routespl.c:85-129) — replicate those formats for easy
    diffing.
