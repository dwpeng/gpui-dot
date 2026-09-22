# Graphviz Postprocessing, Attribute Defaults & Label Sizing — Faithful-Port Spec (dot pipeline)

Source snapshot: **graphviz `main`, commit `2e92c7f2776f6611dd78fa83c213ecc65cc47bfa`** (2026-09-20), checkout at `/tmp/graphviz-src`. All `file:line` citations refer to that tree.

> **Relocation notes vs. older graphviz / vs. the task statement.** Function granularity is preserved but several functions do *not* live where older releases (or the request) put them:
>
> | Requested (old location) | Actual location in current `main` |
> |---|---|
> | `lib/common/utils.c`: `dotneato_postprocess`, `gv_postprocess`, `translate_drawing`, `addClusterObj`, `place_graph_label` | **`lib/common/postproc.c`** (`gv_postprocess` 599, `dotneato_postprocess` 691, `translate_drawing` 153, `addClusterObj` 374, `place_graph_label` 734) |
> | `size_label` / `sizeOneLabel` | **Removed.** Equivalent logic: `storeline`/`make_simple_label` (`lib/common/labels.c:26-105`) builds `dimen` from textsize; the "+border*2" step is now `PAD()` applied at each call site (`do_graph_label` `lib/common/input.c:877-878`, `gv_postprocess` `lib/common/postproc.c:621`, node shapes `lib/common/shapes.c:2006-2008`) |
> | `place_vnlabel` in utils.c | **`lib/dotgen/dotsplines.c:491-502`** |
> | `ports_eq` in utils.c | **`lib/dotgen/position.c:1030-1039`** |
> | `setsize` in `lib/common/input.c` | **Removed** for dot; `size` is parsed by `getdoubles2ptf` (`input.c:476-501`, applied at `input.c:686`) and applied (a) inside layout only for `ratio`, and (b) at render time in `lib/common/emit.c:3376-3385` |
> | `lib/pack/pack_graph.c` | **`lib/pack/pack.c`** (`cccomps` is in **`lib/pack/ccomps.c:437`**); `pack_graph()` API is `pack.c:1131-1142` |
> | `sizeArrayRects` | Removed; nearest survivor is `arrayRects` (`lib/pack/pack.c:605-715`) |
> | `GD_pt2` | **Does not exist** in current `main` (grep over `lib/` finds no such field). Cluster/root label reservation uses `GD_border[4]` and `GD_ht1/GD_ht2` only |

---

## 1. Postprocessing pipeline (`lib/common/postproc.c`, `lib/common/utils.c`)

### 1.1 Entry points and call order

```
dot_layout (lib/dotgen/dotinit.c:473-481)
  if agnnodes(g): doDot(g)                    // dotinit.c:476
  dotneato_postprocess(g)                     // dotinit.c:480
      └─ gv_postprocess(g, 1)                 // postproc.c:691-694
```

`gv_postprocess(Agraph_t *g, int allowTranslation)` — `postproc.c:599-687`. Exact order of operations:

```pseudocode
gv_postprocess(g, allowTranslation):
  Rankdir = GD_rankdir(g)            # postproc.c:605  (0..3, TB=0 LR=1 BT=2 RL=3)
  Flip    = GD_flip(g)               # postproc.c:606  (GD_flip = rankdir & 1, types.h)
  dimen   = {0,0}

  # 1. cluster labels (positions only; boxes assumed computed)
  if Flip: place_flip_graph_label(g) # postproc.c:608-609 (recursively, postproc.c:699-726)
  else:    place_graph_label(g)      # postproc.c:611    (recursively, postproc.c:734-761)

  # 2. exterior (xlabel / unset edge / head / tail) labels, map placement
  addXLabels(g)                      # postproc.c:616  (details §1.6)

  # 3. reserve space for root graph label, if any
  if GD_label(g) && !GD_label(g)->set:          # postproc.c:619
      dimen = GD_label(g)->dimen                # postproc.c:620
      PAD(dimen)                                # postproc.c:621  (macros.h:29)
      if Flip:                                  # postproc.c:622-634
          if label_pos & LABEL_AT_TOP: GD_bb.UR.x += dimen.y
          else:                        GD_bb.LL.x -= dimen.y
          if dimen.x > (GD_bb.UR.y - GD_bb.LL.y):      # widen to fit label length
              diff = (dimen.x - (UR.y-LL.y)) / 2
              LL.y -= diff; UR.y += diff
      else:                                     # postproc.c:635-654
          if label_pos & LABEL_AT_TOP:
              if Rankdir == RANKDIR_TB: GD_bb.UR.y += dimen.y
              else:                     GD_bb.LL.y -= dimen.y
          else:
              if Rankdir == RANKDIR_TB: GD_bb.LL.y -= dimen.y
              else:                     GD_bb.UR.y += dimen.y
          if dimen.x > (GD_bb.UR.x - GD_bb.LL.x):
              diff = (dimen.x - (UR.x-LL.x)) / 2
              LL.x -= diff; UR.x += diff

  # 4. translation to origin (only if allowed)
  if allowTranslation:                          # postproc.c:656
      switch Rankdir:                           # postproc.c:657-672
        RANKDIR_TB: Offset = GD_bb(g).LL
        RANKDIR_LR: Offset = {-GD_bb(g).UR.y, GD_bb(g).LL.x}
        RANKDIR_BT: Offset = {GD_bb(g).LL.x, -GD_bb(g).UR.y}
        RANKDIR_RL: Offset = {GD_bb(g).LL.y,  GD_bb(g).LL.x}
      translate_drawing(g)                      # postproc.c:673

  # 5. place root label (AFTER translation; coordinates are final)
  if GD_label(g) && !GD_label(g)->set:
      place_root_label(g, dimen)                # postproc.c:675-676 → 180-200

  # 6. Show_boxes debug prelude (PostScript helper), only if set
  if !LIST_IS_EMPTY(&Show_boxes): ...           # postproc.c:678-686
```

`PAD(d)` is defined at `lib/common/macros.h:27-29`:
```c
#define XPAD(d) ((d).x += 4*GAP)
#define YPAD(d) ((d).y += 2*GAP)
#define PAD(d)  {XPAD(d); YPAD(d);}
```
with `GAP 4` (`lib/common/const.h:251`) ⇒ **PAD adds 16 pt to x and 8 pt to y**.

Label-position flags (`lib/common/const.h:174-178`): `LABEL_AT_BOTTOM 0, LABEL_AT_TOP 1, LABEL_AT_LEFT 2, LABEL_AT_RIGHT 4`.

`dotneato_postprocess(g)` is exactly `gv_postprocess(g, 1)` (`postproc.c:691-694`) — translation always enabled for dot. (neato/fdp pass `allowTranslation=0` in some paths, e.g. `lib/fdpgen/layout.c:1076`, `lib/neatogen/neatoinit.c:1360`.)

### 1.2 `translate_drawing` — rotation + translation to origin (`postproc.c:148-172`)

Module statics (`postproc.c:26-28`): `Rankdir` (int), `Flip` (bool), `Offset` (pointf).

```pseudocode
map_point(p):                          # postproc.c:90-96
  p = ccwrotatepf(p, Rankdir * 90)     # rotate by rankdir first
  p.x -= Offset.x;  p.y -= Offset.y    # then subtract offset
  return p

translate_drawing(g):
  shift = Offset.x != 0 || Offset.y != 0
  if !shift && Rankdir == 0: return                 # postproc.c:157-160
  for v in agfstnode(g) .. agnxtnode(g, v):         # postproc.c:161-170
      if Rankdir != 0:
          gv_nodesize(v, false)        # RESET lw/rw/ht to unflipped values §1.4
      ND_coord(v) = map_point(ND_coord(v))
      if ND_xlabel(v): ND_xlabel(v)->pos = map_point(...)
      if State == GVSPLINES:           # only after splines were computed
          for e in out-edges of v: map_edge(e)      # postproc.c:98-125
  translate_bb(g, GD_rankdir(g))                    # postproc.c:171
```

`map_edge` (`postproc.c:98-125`): transforms every Bézier control point of every spline segment, `sp`/`ep` arrow endpoints when flagged, and the positions of `ED_label`, `ED_xlabel`, `ED_head_label`, `ED_tail_label`. Errors `"lost %s %s edge\n"` if `ED_spl==NULL` and not `Concentrate` and not `IGNORED` (`postproc.c:102-107`).

`translate_bb(g, rankdir)` (`postproc.c:127-146`):
```pseudocode
bb = GD_bb(g)
if rankdir == RANKDIR_LR || rankdir == RANKDIR_BT:
    new_bb.LL = map_point({bb.LL.x, bb.UR.y})
    new_bb.UR = map_point({bb.UR.x, bb.LL.y})
else:
    new_bb.LL = map_point({bb.LL.x, bb.LL.y})
    new_bb.UR = map_point({bb.UR.x, bb.UR.y})
GD_bb(g) = new_bb
if GD_label(g): GD_label(g)->pos = map_point(GD_label(g)->pos)
for c in 1..GD_n_cluster(g): translate_bb(GD_clust(g)[c], rankdir)   # recursive
```

**Net effect:** after `gv_postprocess` with `allowTranslation=1`, `GD_bb(g).LL == (0,0)` in the *final* (post-rotation) coordinate system; every stored point (node coords, splines, labels, cluster boxes) is integral-rounded not required — dot positions are already integral here.

### 1.3 Cluster label placement — `place_graph_label` / `place_flip_graph_label`

`place_graph_label(g)` (`postproc.c:728-761`, non-flip case; recursive over `GD_clust[1..n]`):

```pseudocode
if g != agroot(g) && GD_label(g) && !GD_label(g)->set:
    if GD_label_pos(g) & LABEL_AT_TOP:                        # postproc.c:740-742
        d = GD_border(g)[TOP_IX]                              # TOP_IX = 2 (const.h:113)
        p.y = GD_bb(g).UR.y - d.y/2
    else:                                                     # postproc.c:743-745
        d = GD_border(g)[BOTTOM_IX]                           # BOTTOM_IX = 0 (const.h:111)
        p.y = GD_bb(g).LL.y + d.y/2
    if GD_label_pos(g) & LABEL_AT_RIGHT:   p.x = GD_bb.UR.x - d.x/2   # :748-749
    elif GD_label_pos(g) & LABEL_AT_LEFT:  p.x = GD_bb.LL.x + d.x/2   # :750-751
    else:                                  p.x = (LL.x + UR.x)/2         # :752-753
    GD_label(g)->pos = p;  GD_label(g)->set = true            # :755-756
for c in 1..n_cluster: place_graph_label(GD_clust(g)[c])
```

`place_flip_graph_label(g)` (`postproc.c:696-726`, flip case; runs *before* `translate_drawing`, i.e. in the rotated frame where x↔y are swapped):
```pseudocode
if g != agroot(g) && GD_label(g) && !set:
    if label_pos & LABEL_AT_TOP: d = GD_border(g)[RIGHT_IX]; p.x = UR.x - d.x/2   # :705-707
    else:                        d = GD_border(g)[LEFT_IX];  p.x = LL.x + d.x/2   # :708-710
    if label_pos & LABEL_AT_RIGHT:  p.y = LL.y + d.y/2        # :713-714
    elif label_pos & LABEL_AT_LEFT: p.y = UR.y - d.y/2        # :715-716
    else:                           p.y = (LL.y + UR.y)/2     # :717-718
    set pos; set = true
```

`GD_border[4]` (indices `BOTTOM_IX=0, RIGHT_IX=1, TOP_IX=2, LEFT_IX=3`, `const.h:111-114`) is populated by `do_graph_label` (§3.6) — that is the only writer.

`place_root_label(g, d)` (`postproc.c:174-200`) — root label only, after translation:
```pseudocode
if label_pos & LABEL_AT_RIGHT: p.x = UR.x - d.x/2
elif label_pos & LABEL_AT_LEFT: p.x = LL.x + d.x/2
else: p.x = (LL.x + UR.x)/2
if label_pos & LABEL_AT_TOP: p.y = UR.y - d.y/2
else: p.y = LL.y + d.y/2
GD_label(g)->pos = p; set = true
```

### 1.4 `gv_nodesize` (`lib/common/utils.c:1525-1536`)

```c
void gv_nodesize(node_t *n, bool flip) {
    if (flip) {
        double w = INCH2PS(ND_height(n));
        ND_lw(n) = ND_rw(n) = w / 2;
        ND_ht(n) = INCH2PS(ND_width(n));
    } else {
        double w = INCH2PS(ND_width(n));
        ND_lw(n) = ND_rw(n) = w / 2;
        ND_ht(n) = INCH2PS(ND_height(n));
    }
}
```
`INCH2PS(a) = a*72.0` (`lib/common/geom.h:63`). Called from `dot_init_node` with `GD_flip(agraphof(n))` (`dotinit.c:49`) — i.e. during layout lw/rw/ht are in *layout* orientation — and from `translate_drawing` with `flip=false` (`postproc.c:163`) to restore unflipped sizes before the final rotation. `ND_xsize = ND_lw+ND_rw`, `ND_ysize = ND_ht` (`types.h`).

### 1.5 `addClusterObj` (`postproc.c:368-387`)

```pseudocode
typedef struct { boxf bb; object_t* objp; } cinfo_t;     # postproc.c:368-371

addClusterObj(g, info):
  for c in 1..GD_n_cluster(g):                            # deepest first (children before self)
      info = addClusterObj(GD_clust(g)[c], info)
  if g != agroot(g) && GD_label(g) && GD_label(g)->set:   # postproc.c:380
      info.bb = addLabelObj(GD_label(g), info.objp, info.bb)   # obstacle at label pos
      info.objp++
  return info
```
`addLabelObj` (`postproc.c:327-343`): object size = label `dimen` (swapped if `Flip`); object position = `lp->pos - sz/2` (labels store centers); extends `bb` via `adjustBB` (`postproc.c:282-296`, MIN/MAX of LL and `pos+sz`). Used only by `addXLabels` (count via `countClusterLabels`, `postproc.c:389-396`).

### 1.6 `addXLabels` — exterior label placement (`postproc.c:398-590`)

Skips entirely unless `GD_has_labels` has `NODE_XLABEL | EDGE_XLABEL | TAIL_LABEL | HEAD_LABEL`, or `EDGE_LABEL` with `EdgeLabelsDone == 0` (`postproc.c:420-425`).

- Labels to place (`n_lbls`): unset node xlabels + unset edge `xlabel`/`label`/`headlabel`/`taillabel` for edges **that have geometry** (`HAVE_EDGE(ep) = (et != EDGETYPE_NONE) && ED_spl(ep) != NULL`, `postproc.c:399`).
- Obstacles (`n_objs`): every node (`addNodeObj`, size = `INCH2PS(width/height)`, swapped if `Flip`, `postproc.c:350-366`), every *set* external label (`addLabelObj`), cluster labels (`addClusterObj`), plus one object per unset edge label.
- Anchor points: edge `label`/`xlabel` → `edgeMidpoint(gp, ep)` (`lib/common/splines.c:1281-1302`: closest point on spline to endpoint-midpoint for `SPLINE/CURVED`, polyline midpoint otherwise); `taillabel` → first spline point (`edgeTailpoint`, `postproc.c:242-259`); `headlabel` → last spline point (`edgeHeadpoint`, `postproc.c:261-278`). Warns `"no position for edge with label %s\n"` etc. when geometry missing (`postproc.c:502,517,532,547`).
- Solver: `placeLabels(objs, n_objs, &params)` with `params.bb` = accumulated bbox and `params.force = late_bool(gp, agfindgraphattr(gp,"forcelabels"), true)` (`postproc.c:563-566`) — **`forcelabels` defaults to true**.
- After solving: for each placed label `lp->set = 1; lp->pos = centerPt(xlp)` (lower-left + `sz/2`, `postproc.c:206-215,570-581`) and `updateBB(gp, lp)` grows the root bbox to include the label half-extents (`lib/common/utils.c:613-616` → `addLabelBB` `utils.c:558-587`). Warns `"%zu out of %zu exterior labels positioned.\n"` if some remain unset (`postproc.c:585-587`).

---

## 2. `lib/common/utils.c` helpers (attribute readers, node/edge init)

### 2.1 `late_*` family — exact semantics

```c
int late_int(void *obj, attrsym_t *attr, int defaultValue, int minimum)   // utils.c:40-53
```
- `attr == NULL` → `defaultValue`.
- `agxget` result empty (`p[0]=='\0'`) → `defaultValue`.
- `strtol(p,&endp,10)`; if no digits consumed (`p==endp`) **or** value `> INT_MAX` → `defaultValue` (note: *no* clamping on parse failure — the default, not the minimum, wins; negative values are accepted if ≥ `minimum`).
- If `rv < minimum` → `minimum`; else `(int)rv`.

```c
double late_double(void *obj, attrsym_t *attr, double defaultValue, double minimum)  // utils.c:55-69
```
- `!attr || !obj` → default; empty string → default; `strtod` no-consumption → default; `rv < minimum` → `minimum`. No upper clamp.

```c
char *late_string(void *obj, attrsym_t *attr, char *defaultValue)   // utils.c:85-89
   // attr NULL or obj NULL → default; else raw agxget (may be "")
char *late_nnstring(...)                                            // utils.c:91-96
   // late_string, but "" or NULL → default ("non-null/ non-empty string")
bool late_bool(void *obj, attrsym_t *attr, bool defaultValue)       // utils.c:98-103
   // attr NULL → defaultValue; else mapbool(agxget(...)) — the default is NOT
   // used for unrecognized strings; mapbool's own false is (see below)
```

`maptoken` / `mapBool` / `mapbool` (`utils.c:315-344`):
```c
int maptoken(char *p, char **name, int *val) {
    int i = 0;
    for (; (q = name[i]) != 0; i++)
        if (p && streq(p, q)) break;      // exact, case-SENSITIVE match
    return val[i];                        // p==NULL or no match → val at the NULL terminator
}
bool mapBool(const char *p, bool defaultValue) {
    if (!p || *p=='\0') return defaultValue;
    if (!strcasecmp(p,"false")) return false;
    if (!strcasecmp(p,"no"))    return false;
    if (!strcasecmp(p,"true"))  return true;
    if (!strcasecmp(p,"yes"))   return true;
    if (gv_isdigit(*p))         return atoi(p) != 0;
    return defaultValue;
}
bool mapbool(const char *p) { return mapBool(p, false); }   // default false
```
⇒ **`mapbool("garbage") == false`**, `mapbool("2")==true`, `mapbool("0")==false`, `mapbool("")` depends on the caller-passed default in `mapBool`.

### 2.2 `common_init_node` (`utils.c:416-443`)

```pseudocode
common_init_node(n):
  ND_width(n)  = late_double(n, N_width,  DEFAULT_NODEWIDTH 0.75, MIN_NODEWIDTH 0.01)   # :420-421
  ND_height(n) = late_double(n, N_height, DEFAULT_NODEHEIGHT 0.5, MIN_NODEHEIGHT 0.02)  # :422-423
  ND_shape(n)  = bind_shape(late_nnstring(n, N_shape, DEFAULT_NODESHAPE "ellipse"), n)  # :424-425
  str = agxget(n, N_label)
  fi.fontsize  = late_double(n, N_fontsize, DEFAULT_FONTSIZE 14.0, MIN_FONTSIZE 1.0)    # :427
  fi.fontname  = late_nnstring(n, N_fontname, DEFAULT_FONTNAME "Times-Roman")           # :428
  fi.fontcolor = late_nnstring(n, N_fontcolor, DEFAULT_COLOR "black")                   # :429
  # record detection: 4th arg is_html_handling = (shapeOf(n) == SH_RECORD)
  ND_label(n) = make_label(n, str, aghtmlstr(str), shapeOf(n)==SH_RECORD,
                           fi.fontsize, fi.fontname, fi.fontcolor)                      # :430-431
  if N_xlabel && agxget(n,N_xlabel) nonempty:
      ND_xlabel(n) = make_label(...same fonts..., false /*not record*/)                 # :432-434
      GD_has_labels(agraphof(n)) |= NODE_XLABEL                                         # :435
  ND_showboxes(n) = (unsigned char)imin(late_int(n, N_showboxes, 0, 0), UCHAR_MAX)      # :438-441
  ND_shape(n)->fns->initfn(n)   # shape-specific init (poly_init computes node size, §3.3)
```

Record detection: `shapeOf(n)` (`lib/common/shapes.c:1908-1919`) dispatches on the shape's `initfn` pointer — `record_init` ⇒ `SH_RECORD`. Only then is the label built with `is_record=true` (raw text preserved for `field_t` splitting; `make_label` `lib/common/labels.c:139-144` keeps `rv->text = strdup(str)` and, if the string is HTML-like, also sets `rv->html`).

`gv_nodesize` is **not** called here; callers do it (dot does, `dotinit.c:49`) because flip orientation is graph-dependent.

### 2.3 `common_init_edge` (`utils.c:497-556`)

```pseudocode
common_init_edge(e):    # returns void; doc comment says "return true if edge has label" (stale)
  sg = agraphof(agtail(e))
  fi.fontname = NULL; lfi.fontname = NULL

  if E_label && agxget(e,E_label) nonempty:                       # :506-512
      initFontEdgeAttr(e,&fi)      # fontsize=late_double(E_fontsize,14,1); fontname/fontcolor
      ED_label(e) = make_label(e, str, aghtmlstr(str), false, fi.*)
      GD_has_labels(sg) |= EDGE_LABEL
      ED_label_ontop(e) = mapbool(late_string(e, E_label_float, "false"))   # :511

  if E_xlabel && agxget(e,E_xlabel) nonempty:                     # :514-520
      if !fi.fontname: initFontEdgeAttr(e,&fi)                    # fonts fall back to edge fonts
      ED_xlabel(e) = make_label(...); GD_has_labels(sg) |= EDGE_XLABEL

  if E_headlabel && agxget nonempty:                              # :522-527
      initFontLabelEdgeAttr(e,&fi,&lfi)                           # :452-460
      # lfi.fontsize  = late_double(E_labelfontsize, fi.fontsize, 1.0)
      # lfi.fontname  = late_nnstring(E_labelfontname, fi.fontname)
      # lfi.fontcolor = late_nnstring(E_labelfontcolor, fi.fontcolor)
      ED_head_label(e) = make_label(... lfi.*); GD_has_labels(sg) |= HEAD_LABEL

  if E_taillabel && agxget nonempty:                              # :528-534
      if !lfi.fontname: initFontLabelEdgeAttr(e,&fi,&lfi)         # lazy: only when tail label exists
      ED_tail_label(e) = make_label(...); GD_has_labels(sg) |= TAIL_LABEL

  # ports (deprecated leading-colon form still accepted)          # :536-555
  str = agget(e, TAIL_ID /* "tailport" */); if !str: str = ""
  if str[0]: ND_has_port(agtail(e)) = true
  ED_tail_port(e) = chkPort(ND_shape(agtail(e))->fns->portfn, agtail(e), str)
  if noClip(e, E_tailclip): ED_tail_port(e).clip = false
  str = agget(e, HEAD_ID /* "headport" */); if !str: str = ""
  if str[0]: ND_has_port(aghead(e)) = true
  ED_head_port(e) = chkPort(ND_shape(aghead(e))->fns->portfn, aghead(e), str)
  if noClip(e, E_headclip): ED_head_port(e).clip = false
```

**`headlabel`/`taillabel` font defaults**: fall back to the *edge's* `fontsize/fontname/fontcolor` (14 pt / Times-Roman / black), **not** to `DEFAULT_LABEL_FONTSIZE 11.0` (`const.h:62`) — that constant is currently unreferenced in `lib/`.

### 2.4 `chkPort` and compass parsing

`chkPort` (`utils.c:477-495`) — **the first-colon rule**:
```pseudocode
chkPort(pf, n, s):
  cp = s ? strchr(s, ':') : NULL       # FIRST colon only
  if cp:
     *cp = '\0'; pt = pf(n, s, cp+1); *cp = ':'    # name = before colon, compass = after
     pt.name = cp+1
  else:
     pt = pf(n, s, NULL); pt.name = s
  return pt
```
So `"n:e"` → port name `"n"`, compass `"e"`; `"n"` → name `"n"`, compass NULL; `":e"` → empty name, compass `"e"` (deprecated form). A second colon stays inside the compass string and will make it unrecognized.

The shape `portfn` is `poly_port` (`lib/common/shapes.c:2887-2912`) for all polygon/record-ish shapes:
- `portname == ""` → return `Center` (`static port Center = {.theta=-1, .clip=true};`, `shapes.c:38` — `defined=false`).
- HTML labels with a matching cell: `html_port(n, portname, &sides)` gives a port box; else `compassPort(n, NULL, &rv, portname, sides, ictxt)` with `bp==NULL` (box derived from `ND_lw/rw/ht`, swapped if `GD_flip`), `defined=false` unless the compass sets it; unknown compass → `unrecognized(n, portname)` (warning).

`compassPort` (`lib/common/shapes.c:2698-2884`) — full compass table (for non-ictxt, i.e. box shapes, the coordinates below; with `ictxt` the point is the ray hit on the shape boundary via `compassPoint`/`bezier_clip`, `shapes.c:2650-2670`):

| compass | point p | theta (unrotated) | constrained | clip | defined | side |
|---|---|---|---|---|---|---|
| `e` | `p.x = b.UR.x` | 0.0 | true | false | true | `sides & RIGHT` |
| `s`  | `p.y = b.LL.y; p.x = ctr.x` | `-M_PI*0.5` | true | false | true | `sides & BOTTOM` |
| `se` | `p.y = b.LL.y; p.x = b.UR.x` | `-M_PI*0.25` | true | false | true | `BOTTOM|RIGHT` |
| `sw` | `p.y = b.LL.y; p.x = b.LL.x` | `-M_PI*0.75` | true | false | true | `BOTTOM|LEFT` |
| `w`  | `p.x = b.LL.x` | `M_PI` | true | false | true | `sides & LEFT` |
| `n`  | `p.y = b.UR.y; p.x = ctr.x` | `M_PI*0.5` | true | false | true | `sides & TOP` |
| `ne` | `p.y = b.UR.y; p.x = b.UR.x` | `M_PI*0.25` | true | false | true | `TOP|RIGHT` |
| `nw` | `p.y = b.UR.y; p.x = b.LL.x` | `M_PI*0.75` | true | false | true | `TOP|LEFT` |
| `_`  | unchanged | 0 | false | true | (as base) | `side = sides` (dynamic port, `pp->side=side`, `pp->dyna=true`) |
| `c`  | unchanged (center) | 0 | false | true | as base | 0 |
| anything else (`rv=1`) | reset `p.y=ctr.y` for bad `s*`/`n*`; else center | | false | true | false | warning path |

Epilogue (`shapes.c:2852-2876`):
```pseudocode
p = cwrotatepf(p, 90 * GD_rankdir(agraphof(n)))       # apply rankdir rotation
pp->side   = dyna ? side : invflip_side(side, rankdir)
pp->bp     = bp
pp->p      = p
pp->theta  = invflip_angle(theta, rankdir)
if p == (0,0): pp->order = MC_SCALE/2                 # MC_SCALE=256 → 128
else:
    angle = atan2(p.y, p.x) + 1.5*M_PI                # 0 at north, CCW
    if angle >= 2*M_PI: angle -= 2*M_PI
    pp->order = (int)(MC_SCALE * angle / (2*M_PI))
pp->constrained = constrain; pp->defined = defined; pp->clip = clip; pp->dyna = dyna
```

### 2.5 `noClip` (`utils.c:462-475`)

```c
static bool noClip(edge_t *e, attrsym_t* sym) {
    bool rv = false;
    if (sym) {   /* mapbool isn't a good fit, because we want "" to mean true */
        str = agxget(e, sym);
        if (str && str[0]) rv = !mapbool(str);
        else rv = false;
    }
    return rv;
}
```
I.e. **empty string ⇒ clipping stays on** (rv=false); `tailclip=false`/`no`/`0` ⇒ `port.clip=false`. Since `E_tailclip = agfindedgeattr(g,"tailclip")` (`input.c:779-780`) creates the attribute with default `""` when absent, an undeclared `tailclip` behaves as `true` (clip on).

### 2.6 `ports_eq` (`lib/dotgen/position.c:1030-1039`)

```c
int ports_eq(edge_t * e, edge_t * f) {
    return ED_head_port(e).defined == ED_head_port(f).defined
        && ((ED_head_port(e).p.x == ED_head_port(f).p.x &&
             ED_head_port(e).p.y == ED_head_port(f).p.y) || !ED_head_port(e).defined)
        && ((ED_tail_port(e).p.x == ED_tail_port(f).p.x &&
             ED_tail_port(e).p.y == ED_tail_port(f).p.y) || !ED_tail_port(e).defined);
}
```
Used by `expand_leaves` (`position.c:1041-1063`) to decide whether leaf edges keep fast-edge status.

### 2.7 `maptoken` and `ND_ranktype`

`ND_ranktype(n)` is `((Agnodeinfo_t*)AGDATA(n))->ranktype`, a `char` (`lib/common/types.h:524`; field `char ranktype`). Values (`const.h:33-40`): `NOCMD 0, SAMERANK 1, MINRANK 2, SOURCERANK 3, MAXRANK 4, SINKRANK 5, LEAFSET 6, CLUSTER 7`.

- Parsing: `rank_set_class` (`lib/dotgen/rank.c:224-237`) — see §4.6.
- `collapse_rankset` (`rank.c:185-222`) assigns the kind to all merged nodes and merges MIN/SOURCE into `GD_minset`, MAX/SINK into `GD_maxset`; SOURCE/SINK re-stamp the merged leader.
- `UF_singleton` resets `ND_ranktype(u) = NORMAL` (`utils.c:143-148`).
- Cluster membership sets `ND_ranktype(n) = CLUSTER` (`rank.c:322`, `cluster.c:326,354`).

### 2.8 Other utils.c functions relevant to postprocessing

- `compute_bb(g)` (`utils.c:622-682`): bbox over nodes (`coord(n) = ND_pos*72` for pack — note dot uses `ND_coord` and this fn is used by the pack library path via `polyGraphs`/`putGraphs`), node xlabels, spline control points, set edge/`head`/`tail`/`x` labels, cluster bboxes (`B2BF(GD_bb(clust))`), and set graph label. Empty graph ⇒ `{0,0}-{0,0}` (`utils.c:630-634`).
- `updateBB(g, lp)` (`utils.c:610-616`): `GD_bb(g) = addLabelBB(GD_bb(g), lp, GD_flip(g))`; `addLabelBB` extends by `pos ± dimen/2` (swapped when flip) (`utils.c:558-587`).
- `is_a_cluster(g)` (`utils.c:684-688`): `g == g->root || strncasecmp(name,"cluster",7)==0 || mapbool(agget(g,"cluster"))` — the graph attribute **`cluster`** (bool) is read here.
- `setEdgeType` / `edgeType` (`utils.c:1362-1425`) — see §4.4.
- `get_inputscale(g)` (`utils.c:71-83`): `-s` flag `PSinputscale>0` wins; else graph `inputscale` (default −1, min 0); `0` ⇒ `POINTS_PER_INCH` (72).

---

## 3. Label text sizing (`lib/common/labels.c`, `lib/common/textspan.c`, `lib/common/textspan_lut.c`)

### 3.1 From text to `textlabel_t` (`make_label`, `labels.c:110-183`)

```pseudocode
make_label(obj, str, is_html, is_record, fontsize, fontname, fontcolor):
  rv = new textlabel_t{fontname, fontcolor, fontsize, charset = GD_charset(g)}
  is_html &= str != ""                       # empty string never HTML (labels.c:119)
  switch obj kind:
    GRAPH: g = sg->root; NODE: g = agroot(agraphof(n)); EDGE: g = agroot(agraphof(aghead(e)))
  if is_record:            rv->text = strdup(str); if is_html: rv->html = true   # no sizing here
  elif is_html:            rv->text = strdup(str); rv->html = true;
                           make_html_label(obj, rv)          # htmltable.c computes rv->dimen
  else:
      rv->text = strdup_and_subst_obj0(str, obj, 0)   # \\G \\N \\E \\T \\H \\L substitution,
                                                      # backslashes preserved (labels.c:282-384)
      rv->text = charset==CHAR_LATIN1 ? latin1ToUTF8(text) : htmlEntityUTF8(text, g)
      make_simple_label(GD_gvc(g), rv)                # line splitting + sizing (below)
```

### 3.2 `make_simple_label` / `storeline` — the `size_label` equivalent (`labels.c:26-105`)

```pseudocode
make_simple_label(gvc, lp):
  lp->dimen = (0,0)
  if lp->text == "": return
  scan text char by char (with BIG5 two-byte handling, charset==CHAR_BIG5, 0xA1..0xFE):
    '\\' followed by 'n' | 'l' | 'r'  → storeline(gvc, lp, line, that char)   # hard line break + justification
    '\\' + other                      → copy next char literally ("\\x"→"x", "\\"→"\")
    real '\n'                         → storeline(gvc, lp, line, 'n')
    else                              → append byte
  if leftover line nonempty: storeline(gvc, lp, line, 'n')
  lp->space = lp->dimen                       # labels.c:104

storeline(gvc, lp, line, terminator):
  span = &lp->u.txt.span[nspans++]            # append textspan_t
  span->str = line; span->just = terminator   # 'n' | 'l' | 'r'
  if line nonempty:
      span->font = dict{lp->fontname, lp->fontsize}    # gvc->textfont_dt
      size = textspan_size(gvc, span)                  # §3.3
  else:
      size.x = 0.0
      span->size.y = size.y = (int)(lp->fontsize * LINESPACING)    # empty line: truncated to int
  lp->dimen.x = MAX(lp->dimen.x, size.x)      # width = max over spans
  lp->dimen.y += size.y                       # height = sum of spans
```

`LINESPACING 1.20` (`const.h:70`). **No border/padding is added here** — `dimen` is pure textsize; the `+ border*2` (old `sizeOneLabel`) now happens per use site (see §3.3/§3.6/§1.1).

Emission-time justification/valign (`emit_label`, `labels.c:217-275`) for reference: first span baseline `p.y = pos.y + space.y/2 - fontsize` (valign 't'), `pos.y - space.y/2 + dimen.y - fontsize` ('b'), `pos.y + dimen.y/2 - fontsize` ('c'/default); per span `p.x = pos.x - space.x/2` (just 'l'), `pos.x + space.x/2` ('r'), `pos.x` ('n'); then `p.y -= span[i].size.y`.

### 3.3 Per-span metrics — `textspan_size` / `estimate_textspan_size` (`textspan.c`)

```c
// textspan.c:73-103
pointf textspan_size(GVC_t *gvc, textspan_t *span) {
    font = span->font;                        // must exist
    if (!font->postscript_alias) font->postscript_alias = translate_postscript_fontname(font->name);
    if (!gvtextlayout(gvc, span, fpp))        // plugin (pango/cairo) if available
        estimate_textspan_size(span, fpp);    // fallback estimator
    return span->size;
}
```

`estimate_textspan_size` (`textspan.c:31-52`) — the portable formulas a Rust port should reproduce when no font plugin exists:
```pseudocode
bold   = font.flags & HTML_BF
italic = font.flags & HTML_IF
span.size.y  = fontsize * LINESPACING                     # 1.20
span.yoffset_layout      = fontsize                       # ascent ≈ 1×size from top of logical rect
span.yoffset_centerline  = 0.1 * fontsize
span.size.x  = fontsize * estimate_text_width_1pt(font.name, span.str, bold, italic)
fontpath = "[internal hard-coded]"
```

`estimate_text_width_1pt` (`lib/common/textspan_lut.c:833-847`):
- Looks up a case-insensitive font-family entry in `all_font_metrics` (`textspan_lut.c:33-831`); families: times/timesroman/timesnewroman/freeserif/liberationserif/nimbusroman/texgyretermes/tinos/thorndale (units_per_em 2048), helvetica/arial/arialmt/freesans/liberationsans/arimo/albany/nimbussans/texgyreheros (2048), cour/courier/couriernew/nimbusmono/texgyrecursor/freemono/liberationmono/cousine/cumberland (2048), Nunito (1000), DejaVu Sans (2048), Consolas (2048), Trebuchet MS (2048), Verdana (2048), Open Sans (2048), Georgia (2048). No match ⇒ Times metrics (`textspan_lut.c:768-784`).
- Per style variant (regular/bold/italic/bold-italic) there is a 128-entry `short widths[...]` table in units of `units_per_em` per 1 pt; `-1` = unknown ⇒ warning `"Warning: no value for width of ASCII character %u. Falling back to 0\n"` and width 0 (non-ASCII bytes therefore contribute 0).
- `width_1pt(text) = Σ widths[byte] / units_per_em`.

### 3.4 Node labels — where padding comes from

`common_init_node` leaves `ND_label->dimen` = textsize. Sizing to node dimensions happens in the shape's init:
- `poly_init` (`shapes.c:1932-2160`): `dimen = ND_label->dimen`; if nonempty and the shape is not "plain", padding is added — either from the **node `margin` attribute** (`shapes.c:1994-2005`: `"x,y"` in inches ⇒ `dimen.x += 2*INCH2PS(x)`, `dimen.y += 2*INCH2PS(y)`; single value duplicates) or `PAD(dimen)` (i.e. +16 pt x, +8 pt y, `shapes.c:2006-2008`). Then `quantum` quantization (`quant()`), ellipse fitting, `space.x/space.y` justification borders (`shapes.c:2132-2151`), and finally `ND_width/ND_height = PS2INCH(max(dimen, bb))` for `fixedsize=shape/none` cases (`shapes.c:2372-2375`).
- `record_init`/`point_init`/`epsf_init` have their own rules (records: per-field margins; points: fixed).
- `gv_nodesize` then derives `ND_lw/rw/ht` from `ND_width/height` (§1.4).

### 3.5 Edge labels

- **Regular (between ranks)**: `dimen` = textsize (no padding). Space is reserved by doubling edge lengths in ranking and halving `GD_ranksep` with integer division (`edgelabel_ranks`, `lib/dotgen/rank.c:170-182`):
  ```c
  if (GD_has_labels(g) & EDGE_LABEL) {
      for all edges: ED_minlen(e) *= 2;
      GD_ranksep(g) = (GD_ranksep(g) + 1) / 2;    // int division
  }
  ```
  A label virtual node is created at `label_rank = (ND_rank(from)+ND_rank(to))/2` (`make_chain`, `lib/dotgen/class2.c:69-96`) by `label_vnode` (`class2.c:22-39`):
  ```c
  ND_label(v) = ED_label(orig);
  ND_lw(v) = GD_nodesep(agroot(v));
  if (!ED_label_ontop(orig)) {
      if (GD_flip(agroot(g))) { ND_ht(v) = dimen.x; ND_rw(v) = dimen.y; }
      else                    { ND_ht(v) = dimen.y; ND_rw(v) = dimen.x; }
  }
  ```
  (`plain_vnode` adds `GD_nodesep/2` to both lw and rw, `class2.c:41-53`.) Positioning is `place_vnlabel` (`lib/dotgen/dotsplines.c:491-502`):
  ```pseudocode
  place_vnlabel(n):                     # regular edge labels only
    if ND_in(n).size == 0: return       # skip flat edge labels
    walk ND_out(n).list[0] via ED_to_orig until ED_edge_type == NORMAL → e
    dimen  = ED_label(e)->dimen
    width  = GD_flip(agraphof(n)) ? dimen.y : dimen.x
    ED_label(e)->pos.x = ND_coord(n).x + width / 2.0
    ED_label(e)->pos.y = ND_coord(n).y
    ED_label(e)->set = true
  ```
  Called at `dotsplines.c:212` (line-splines mode), `dotsplines.c:345` (ortho/other early placement) and after routing at `dotsplines.c:428-435` where each label also does `updateBB(g, l)`. For `splines=curved`, edge labels are unsupported: warning `"edge labels with splines=curved not supported in dot - use xlabels\n"` (`dotsplines.c:240-244`).
- **Flat edge labels** (`dotsplines.c:944-1040`): stacked around the flat chain with `#define LBL_SPACE 6` (`dotsplines.c:944`); first label `pos.y = tp.y + (dimen.y + LBL_SPACE)/2`, subsequent labels alternate below/above with 6 pt gaps (`dotsplines.c:984-1035`).
- **`labelfloat=true`** ⇒ `ED_label_ontop` ⇒ the vnode gets no label height (`class2.c:29-37`) and the label may overlap edges.
- **`xlabel` on an edge**: never positioned inline; treated as an exterior label by `addXLabels` anchored at `edgeMidpoint` (§1.6).

### 3.6 head/tail labels (`headlabel`, `taillabel`)

Created in `common_init_edge` (§2.3). Positioned in one of two ways:
1. If the edge defines **`labelangle` or `labeldistance`**: `place_portlabel` (`lib/common/splines.c:1314-1352`), invoked from `makePortLabels` (`splines.c:1203-1210`, called via `addEdgeLabels`, `splines.c:1305-1307`) and from `dot_splines_` (`dotsplines.c:443-462`, which also `updateBB`s the graph):
   ```pseudocode
   angle = atan2(pf.y-pe.y, pf.x-pe.x) + RADIANS(late_double(e, E_labelangle, PORT_LABEL_ANGLE (-25), -180.0))
   dist  = PORT_LABEL_DISTANCE (10) * late_double(e, E_labeldistance, 1.0, 0.0)
   l->pos = pe + dist * {cos(angle), sin(angle)};  l->set = true
   ```
   with `PORT_LABEL_DISTANCE 10`, `PORT_LABEL_ANGLE -25` (`const.h:101-102`); `pe` = spline endpoint (arrow tip when flagged), `pf` = point at t=0.1 (tail) / 0.9 (head) on the adjacent Bézier.
2. Otherwise: left unset and placed by `addXLabels` at `edgeTailpoint`/`edgeHeadpoint` (§1.6), participating in the exterior-label overlap map.

### 3.7 Graph & cluster labels — `do_graph_label` (`lib/common/input.c:829-895`)

```pseudocode
do_graph_label(sg):
  if !(str = agget(sg,"label")) or *str=='\0': return
  GD_has_labels(sg->root) |= GRAPH_LABEL                                    # :840
  GD_label(sg) = make_label(sg, str, aghtmlstr(str), false,
      late_double(sg, agfindgraphattr(sg,"fontsize"),  14.0, 1.0),
      late_nnstring(sg, agfindgraphattr(sg,"fontname"), "Times-Roman"),
      late_nnstring(sg, agfindgraphattr(sg,"fontcolor"), "black"))          # :842-848

  # labelloc → LABEL_AT_*                                                   # :850-862
  pos = agget(sg, "labelloc")
  if sg != agroot(sg):   # clusters default to TOP
      pos_flag = (pos && pos[0]=='b') ? LABEL_AT_BOTTOM : LABEL_AT_TOP
  else:                  # root graphs default to BOTTOM
      pos_flag = (pos && pos[0]=='t') ? LABEL_AT_TOP : LABEL_AT_BOTTOM
  just = agget(sg, "labeljust")                                             # :863-869
  if just: if just[0]=='l': pos_flag |= LABEL_AT_LEFT
           elif just[0]=='r': pos_flag |= LABEL_AT_RIGHT
  GD_label_pos(sg) = pos_flag

  if sg == agroot(sg): return                                               # :872-873 (no border)

  # cluster label border reservation                                        # :875-893
  dimen = GD_label(sg)->dimen
  PAD(dimen)                                # +16 x, +8 y (this is the old border*2)
  if !GD_flip(agroot(sg)):
      pos_ix = (label_pos & LABEL_AT_TOP) ? TOP_IX : BOTTOM_IX
      GD_border(sg)[pos_ix] = dimen
  else:   # rotated: labels will be restored to TOP/BOTTOM after translate
      pos_ix = (label_pos & LABEL_AT_TOP) ? RIGHT_IX : LEFT_IX
      GD_border(sg)[pos_ix].x = dimen.y;  GD_border(sg)[pos_ix].y = dimen.x
```

Call sites of `do_graph_label`: root graph in `graph_init` (`input.c:711`); every cluster as it is registered, `make_new_cluster` (`lib/dotgen/rank.c:239-249`); also neato/fdp/osage cluster init.

**How `GD_border` is consumed during dot layout** (this is the "GD_border[4] reservations" mechanism):
- Vertical (non-flip): `clust_ht` (`lib/dotgen/position.c:708-753`) adds `GD_border(g)[BOTTOM_IX].y` to `ht1` (space below) and `GD_border(g)[TOP_IX].y` to `ht2` (space above) for every non-root cluster with a label (`position.c:736-742`); comment at `position.c:735`: *"room for root graph label is handled in dotneato_postprocess"*.
- Horizontal: `make_lrvn` (`position.c:1078-1096`) forces the cluster's width ≥ `fmax(GD_border[BOTTOM_IX].x, GD_border[TOP_IX].x)` via an aux edge `ln→rn` (`position.c:1089-1092`); `contain_nodes` (`position.c:1101-1125`) and `contain_subclust` (`position.c:444-466`) add `GD_border[LEFT_IX].x` / `[RIGHT_IX].x` plus `margin` to the containment edges.
- Flip (rankdir=LR/RL): `adjustRanks` (`position.c:656-701`) grows the cluster vertically to fit `lht = MAX(GD_border[LEFT_IX].y, GD_border[RIGHT_IX].y)` (`position.c:685-694`).
- Cluster box margins everywhere: `margin = late_int(g, G_margin /* "margin" */, CL_OFFSET (8), 0)` at `position.c:415, 454, 478, 668, 719, 1106` and per-rank `yoff` at `position.c:788`; `CL_OFFSET 8` is defined at `const.h:142` ("margin of cluster box in PS points"). Also `set_ycoords` adds `CL_OFFSET` to the cluster rank separation `d1 = ht2+ht1+CL_OFFSET` (`position.c:806`).

---

## 4. Attribute defaults & registration for dot

### 4.1 Registration mechanics

cgraph attributes are created lazily: `agfindgraphattr(g,name)` is `agattr_text(g,AGRAPH,name,NULL)` (`types.h` bottom) which *creates* the attribute with default `""` when missing. So all "defaults" below come from the constants passed at the read sites, not from an attribute table. Two exceptions:
- node `label` default `"\N"` (`NODENAME_ESC`, `const.h:80`) is registered globally if absent: `input.c:466-469` (`dotneato_args_initialize`) and again in `graph_init` (`input.c:729-731`).
- dot phase output attrs `rank` / `order` are (re)created as node attrs with default `""` by `attach_phase_attrs` (`dotinit.c:241-256`) when `phase` stops early.
- `-G/-N/-E/-A` command-line definitions set `fixed` defaults via `global_def` (`input.c:178-193`).

### 4.2 Graph-level defaults read in `graph_init` (`lib/common/input.c:600-789`)

| code (input.c) | attribute | default | min | stored as |
|---|---|---|---|---|
| :633-634 | `quantum` | `0.0` | `0.0` | `GD_drawing->quantum` (double) |
| :643-655 | `rankdir` | `TB` (`RANKDIR_TB 0`) | — | `SET_RANKDIR(g,(rankdir<<2)\|rankdir)`; `GD_rankdir = rankdir&3`, `GD_flip = rankdir&1`, `GD_realrankdir = >>2` (types.h) |
| :657-659 | `nodesep` | `DEFAULT_NODESEP 0.25` | `MIN_NODESEP 0.02` | **`GD_nodesep(g) = POINTS(xf)`** |
| :661-673 | `ranksep` | `DEFAULT_RANKSEP 0.5` | `MIN_RANKSEP 0.02` | parse `%lf`; parse-failure ⇒ default; `<min` ⇒ min; substring `"equally"` ⇒ `GD_exact_ranksep(g)=true`; **`GD_ranksep(g) = POINTS(xf)`** |
| :675-681 | `showboxes` | `0` | `0` | `GD_showboxes` (clamped to `UCHAR_MAX 255`) |
| :682-683 | `fontnames` | — | — | `maptoken(p, {"gd","ps","svg"}, {NATIVEFONTS,PSFONTS,SVGFONTS,-1})`; absent string matches the trailing `NULL` entry ⇒ `LOCAL`? — code is `GD_fontnames(g) = maptoken(...)`, the terminator value is `-1` for an unknown string but the array ends `{..., -1}` after `SVGFONTS`; with `p==NULL` the loop returns `fontnamecodes[3] = -1` which is cast into the `fontname_kind` enum |
| :685 | `ratio` | — | — | `setRatio` (§6.2) |
| :686 | `size` | — | — | `GD_drawing->filled = getdoubles2ptf(g,"size",&GD_drawing->size)`; `filled` ⇔ trailing `!` |
| :687 | `page` | — | — | `getdoubles2ptf(g,"page",&GD_drawing->page)` |
| :689 | `center` | `false` | — | `GD_drawing->centered = mapbool(...)` (render-time only, `emit.c:1267-1272`) |
| :691-696 | `rotate` / `orientation` / `landscape` | `false` | — | `landscape = (rotate==90) \| (orientation[0] ∈ {l,L}) \| mapbool(landscape)`, first present wins |
| :698-699 | `clusterrank` | — | — | `CL_type = maptoken(p, {"local","global","none"}, {LOCAL 100, GLOBAL 101, NOCLUST 102, LOCAL})`; `p==NULL`/unknown ⇒ `LOCAL` via terminator |
| :700-701 | `concentrate` | `false` | — | global `Concentrate = mapbool(p)` |
| :705-709 | `dpi` / `resolution` | `0` | `0` | `GD_drawing->dpi` (double) |
| :711 | (label block) | — | — | `do_graph_label(g)` §3.7 |
| :715-717 | `ordering`, `gradientangle`, `margin` | — | — | only *cached into globals* `G_ordering`, `G_gradientangle`, `G_margin` for later `late_*` reads |

**Unit conversion / rounding** (`lib/common/geom.h:58-64`, `lib/common/arith.h:48`):
```c
#define POINTS_PER_INCH 72
#define POINTS(a_inches)   (ROUND((a_inches)*POINTS_PER_INCH))
#define INCH2PS(a_inches)  ((a_inches)*(double)POINTS_PER_INCH)
#define PS2INCH(a_points)  ((a_points)/(double)POINTS_PER_INCH)
#define ROUND(f)  ((f>=0) ? (int)(f + .5) : (int)(f - .5))
```
`GD_nodesep` and `GD_ranksep` are both `int` fields of `Agraphinfo_t` (`types.h:334-335`). So `nodesep=0.25` ⇒ `POINTS(0.25)=ROUND(18.0)=18`; the "int truncation" is really **round-half-away-from-zero after the ×72 multiply** (`arith.h:48`); `ranksep=0.5` ⇒ 36. `getdoubles2ptf` (`input.c:476-501`) parses `"%lf,%lf%c"` (both must be > 0) or `"%lf%c"` (single value → both x and y), multiplies by `POINTS()`, and returns `true` iff a trailing `!` was consumed.

`GD_nodesep`/`GD_ranksep` assignment sites: `input.c:659` and `input.c:673` (root graph); subgraphs inherit copies in `initSubg` (`dotinit.c:316-317`) and pack aux graphs; `edgelabel_ranks` rewrites `GD_ranksep` in place (`rank.c:180`).

### 4.3 `splines` — `setEdgeType` and `EDGETYPE_*` (`lib/common/utils.c:1362-1425`)

```c
void setEdgeType(graph_t *g, int defaultValue) {          // utils.c:1412-1425
    char* s = agget(g, "splines");
    int et;
    if (!s)                et = defaultValue;             // attribute not present
    else if (*s == '\0')   et = EDGETYPE_NONE;            // empty string ⇒ NO edges drawn
    else                   et = edgeType(s, defaultValue);
    GD_flags(g) |= et;
}
```
`edgeType(s, defaultValue)` (`utils.c:1363-1398`), case-insensitive (`strcasecmp`), leading char shortcuts first:

| value | result |
|---|---|
| `""` (handled by caller) | `EDGETYPE_NONE` |
| `"0"` | `EDGETYPE_LINE` |
| `'1'`–`'9'` (any number ≥1) | `EDGETYPE_SPLINE` |
| `"curved"` | `EDGETYPE_CURVED` |
| `"compound"` | `EDGETYPE_COMPOUND` |
| `"false"`, `"line"`, `"no"` | `EDGETYPE_LINE` |
| `"none"` | `EDGETYPE_NONE` |
| `"ortho"` | `EDGETYPE_ORTHO` |
| `"polyline"` | `EDGETYPE_PLINE` |
| `"spline"`, `"true"`, `"yes"` | `EDGETYPE_SPLINE` |
| anything else | warning `"Unknown \"splines\" value: \"%s\" - ignored\n"`, `defaultValue` |

Bit values (`const.h:233-243`), stored OR-ed into `GD_flags` and masked by `EDGE_TYPE(g) = GD_flags(g) & (7 << 1)` (`lib/common/macros.h:25`):
```c
#define EDGETYPE_NONE       (0 << 1)   // 0
#define EDGETYPE_LINE       (1 << 1)   // 2
#define EDGETYPE_CURVED     (2 << 1)   // 4
#define EDGETYPE_PLINE      (3 << 1)   // 6
#define EDGETYPE_ORTHO      (4 << 1)   // 8
#define EDGETYPE_SPLINE     (5 << 1)   // 10
#define EDGETYPE_COMPOUND   (6 << 1)   // 12
#define NEW_RANK            (1 << 4)   // 16
```
dot's call: `setEdgeType(g, EDGETYPE_SPLINE)` in `dotLayout` (`dotinit.c:262`) ⇒ **the dot default is true splines; an *empty* `splines=""` disables edge routing entirely** (`dot_splines_` returns early on `EDGETYPE_NONE`, `dotsplines.c:237`; see also comment `dotsplines.c:482-486`).

### 4.4 `rank`, `rank_set_class` (`lib/dotgen/rank.c:224-237`)

```c
static int rank_set_class(graph_t * g) {
    static char *name[] = { "same", "min", "source", "max", "sink", NULL };
    static int class[]  = { SAMERANK, MINRANK, SOURCERANK, MAXRANK, SINKRANK, 0 };
    if (is_a_cluster(g)) return CLUSTER;
    val = maptoken(agget(g, "rank"), name, class);   // exact match; NULL/unknown ⇒ 0 (NOCMD)
    GD_set_type(g) = val;
    return val;
}
```
Consumed by `collapse_rankset` (§2.7) and by the new-rank path: `rankset_kind` (`rank.c:573-590`, case-sensitive `strcmp`) and the warning at `rank.c:690-696`. `clusterrank`: §4.2 (`LOCAL`/`GLOBAL`/`NOCLUST`; consumed at `expand_ranksets` `rank.c:496-503` — `CL_type == LOCAL` ⇒ per-cluster `set_minmax`, else `find_clusters`).

### 4.5 `newrank`, `compound`, `concentrate`

- `newrank` (`lib/dotgen/rank.c:528-533`): `if (mapbool(agget(g,"newrank"))) { GD_flags(g) |= NEW_RANK; dot2_rank(g); } else dot1_rank(g);` — default false ⇒ old ranker.
- `compound` (`dotinit.c:301-302`): after splines, `if (mapbool(agget(g,"compound"))) dot_compoundEdges(g);` (lhead/ltail reading at `lib/dotgen/compound.c:274-382`). Default false.
- `concentrate`: global bool from `graph_init` (`input.c:700-701`); used in edge classification to merge 2-edge chains (`class2.c:214,269`), in `dot_concentrate` (`position.c:126-131`), and to suppress the "lost edge" error in `map_edge` (`postproc.c:103`); splines for concentrated edges merged at `lib/common/routespl.c:984`.

### 4.6 Iteration-limit knobs

| attribute | where read | semantics |
|---|---|---|
| `nslimit` | `lib/dotgen/position.c:159-162` (`nsiter2`) | `maxiter = INT_MAX` default; if set: `scale_clamp(agnnodes(g), atof(s))` — bounds network-simplex iterations of the *x*-coordinate ranking pass |
| `nslimit1` | `lib/dotgen/rank.c:457-460` (`rank1`) and `rank.c:1089-1093` (`dot2_rank`) | same formula; bounds the ranking network-simplex (old and new rankers) |
| `searchsize` | `lib/dotgen/rank.c:1102-1106` (new ranker only) | `ssize = atoi(s)` else `-1`; passed to `rank2(Xg, 1, maxiter, ssize)` |
| `mclimit` | `lib/dotgen/mincross.c:1753-1763` (`mincross_options`) | `MinQuit = 8; MaxIter = 24;` if `mclimit` parses to `f > 0`: `MinQuit = MAX(1, scale_clamp(MinQuit,f)); MaxIter = MAX(1, scale_clamp(MaxIter,f));` |
| `maxiter` | **not read by dot** (neato-only); dot's internal `MaxIter` constants are as above |
| `phase` | `dotinit.c:260`: `late_int(g, agfindgraphattr(g,"phase"), -1, 1)` — stop after rank/mincross/position |
| `remincross` | `mincross.c:385-389`: re-run mincross when clusters exist unless `mapbool` false; default true |

`scale_clamp` (`lib/util/gv_math.h:79-89`):
```c
static inline int scale_clamp(int original, double scale) {
    assert(original >= 0);
    if (scale < 0) return 0;
    if (scale > 1 && original > INT_MAX / scale) return INT_MAX;
    return (int)(original * scale);
}
```
⇒ `nslimit=1.0` ⇒ maxiter = #nodes; `nslimit=0` ⇒ 0 iterations; negative ⇒ 0.

### 4.7 Checklist — every attribute read in the dot pipeline

Legend: **G** graph, **N** node, **E** edge. "Default" = value used when attribute absent/empty. Sites marked (cache) only bind the `Agsym_t` for later `late_*`/`agxget` reads.

**Graph attributes**

| attribute | default | read at | effect |
|---|---|---|---|
| `Damping`, `K`, … | — | (neato only — out of scope) | |
| `aspect` | disabled | `lib/dotgen/aspect.c:27-37` (`setAspect`, called `dotinit.c:263`) | parse `"%lf,%d"`; if parses, warns *"the aspect attribute has been disabled due to implementation flaws - attribute ignored."* |
| `center` | false | `input.c:689` | `GD_drawing->centered`; render-time page centering (`emit.c:1267`) |
| `charset` | `"utf-8"` | `input.c:553-573` (`findCharset`) | `CHAR_UTF8/LATIN1/BIG5`; unknown ⇒ warn + UTF-8 |
| `cluster` | false | `utils.c:684-688` (`is_a_cluster`) | treat subgraph as cluster even if name lacks `cluster` prefix |
| `clusterrank` | `local` | `input.c:698-699` | `LOCAL/GLOBAL/NOCLUST` cluster ranking mode |
| `compound` | false | `dotinit.c:301` | contract edges to cluster boundaries post-splines |
| `concentrate` | false | `input.c:700-701` | merge multi-edges, `dot_concentrate` |
| `dpi`, `resolution` | 0 | `input.c:705-709` | `GD_drawing->dpi` (render scaling) |
| `fixed` | — | `dotsplines.c:768` (node attr of the aux graph only) | copied into aux graph for label-aware splines |
| `fontcolor` | `"black"` | `input.c:847-848` | graph/cluster label color |
| `fontname` | `"Times-Roman"` | `input.c:845-846` | graph/cluster label font |
| `fontnames` | — | `input.c:682-683` | svg/ps/gd font-name handling |
| `fontpath` | `$DOTFONTPATH` | `input.c:612-622` | exported to `GDFONTPATH` env |
| `fontsize` | `14.0` | `input.c:843-844` | graph/cluster label size (min 1) |
| `forcelabels` | **true** | `postproc.c:563-565` | place exterior labels even if they overlap |
| `gradientangle` | — | `input.c:716` (cache) | graph fill gradient |
| `id` | — | `input.c:787-788` | output id, escape-substituted |
| `imagepath` | `Gvfilepath` | `input.c:627-631` | image lookup path |
| `inputscale` | −1 | `utils.c:78-83` | legacy input scale (with `-s`) |
| `label` | — | `input.c:836` | graph/cluster label (root ⇒ bottom, cluster ⇒ top) |
| `labeljust` | center | `input.c:863-869` | `'l'`/`'r'` → `LABEL_AT_LEFT/RIGHT` |
| `labelloc` | root: `b`; cluster: `t` | `input.c:851-862` | vertical label placement + `GD_border` side |
| `landscape` | false | `input.c:695-696` | render rotation 90° |
| `margin` | `CL_OFFSET 8` (clusters) | `input.c:717` (cache); `position.c:415,454,478,668,719,788,1106` | cluster box margin, `late_int` min 0; render margin `emit.c:3231-3240` |
| `newrank` | false | `rank.c:529` | use new ranker (`rank2`) |
| `nodesep` | `0.25` (min `0.02`) | `input.c:657-659` | `GD_nodesep = POINTS()` int |
| `nslimit` | none (INT_MAX) | `position.c:160-161` | x-iteration bound `scale_clamp(agnnodes, f)` |
| `nslimit1` | none (INT_MAX) | `rank.c:458-459`, `rank.c:1090-1091` | rank NS iteration bound |
| `ordering` | — | `input.c:715` (cache), `mincross.c:494,509` (node/graph) | keep out-edge order in mincross |
| `orientation`, `rotate` | portrait | `input.c:691-694` | landscape detection |
| `outputorder` | breadth-first | `emit.c:1078-1090` (`chkOrder`) | `nodesfirst`/`edgesfirst` emit order |
| `pack` | −1 (off) | `lib/pack/pack.c:1270-1283` (`getPack`, used `dotinit.c:404`) | `true`/`t`/`T` ⇒ margin `CL_OFFSET 8`; number ⇒ margin; enables packing |
| `packmode` | (context) | `pack.c:1212-1259` | `array[_c/_i/_u/_t/_b/_l/_r][N]`, `aspect[_f]`, `cluster`, `graph`, `node` |
| `page` | — | `input.c:687` | page size for paged output & `ratio=auto` |
| `pagedir` | `"BL"` | `emit.c:3259-3262` | page traversal order |
| `pad` | `DEFAULT_GRAPH_PAD 4` | `emit.c:3242-3251`, default `emit.c:3303` | output padding (points×72) |
| `phase` | −1 (all) | `dotinit.c:260` | stop after phase 1/2/3, write `rank`/`order` attrs |
| `quantum` | 0 | `input.c:633-634` | snap node dims to multiples |
| `rank` (subgraph) | none | `rank.c:234` (`rank_set_class`); new ranker: `rankset_kind` `rank.c:573-590` + `rank.c:695` | `same/min/source/max/sink` (`rankset_kind` uses case-**sensitive** `strcmp`, unknown ⇒ `NORANK 6`) |
| `rankdir` | `TB` | `input.c:643-655` | `GD_rankdir`, `GD_flip` |
| `ranksep` | `0.5` (min `0.02`) | `input.c:661-673` | `GD_ranksep = POINTS()`; `equally` ⇒ `GD_exact_ranksep` |
| `ratio` | none | `input.c:576-598` (`setRatio`) | `auto/compress/expand/fill/<number>` (§6.2) |
| `remincross` | true | `mincross.c:385-389` | re-run mincross with clusters |
| `resolution` | 0 | `input.c:705-709` | alias for `dpi` |
| `rotate` | 0 | `input.c:691-692` | `==90` ⇒ landscape |
| `samehead`/`sametail` | — | `dotsplines.c:733-734` (aux graph) | heads/tails sharing a port for label routing |
| `searchsize` | −1 | `rank.c:1103-1106` | new-ranker search bound |
| `showboxes` | 0 | `input.c:675-681` | debug boxes (clamped 255) |
| `size` | none | `input.c:686` (`getdoubles2ptf`) | `W,H` or `W,H!`; `!` ⇒ `filled` (§6.2) |
| `sortv` | 0 | `pack.c:916-919` | user order for `packmode=array` with `_u` flag |
| `splines` | `EDGETYPE_SPLINE` for dot | `utils.c:1412-1425`, `dotinit.c:262` | edge type (§4.3); `""` ⇒ none |
| `viewport` | — | `emit.c:3391-3414` | render viewport override |

**Node attributes** (bound `input.c:719-751`; consumed where noted)

| attribute | default | consumed at | effect |
|---|---|---|---|
| `comment` | — | `input.c:748` (cache) | emitted in comments |
| `distortion` | 0 | `input.c:741` (cache) | polygon shape param |
| `fillcolor` | `color` | `input.c:724` (cache) | node fill |
| `fixedsize` | false | `input.c:742` (cache); `shapes.c:2100-2126` | label does not grow node |
| `fontcolor`/`fontname`/`fontsize` | black/Times-Roman/14 | `utils.c:427-429` | label metrics |
| `gradientangle` | — | `input.c:751` (cache) | node gradient |
| `group` | "" | `dotinit.c:66-72` | cross-rank alignment; edges inside a group get `xpenalty=CL_CROSS`, `weight*=100` |
| `height` | `0.5` (min `0.02`) | `utils.c:422-423` | inches |
| `imagepos` | — | `input.c:744` (cache) | image anchor |
| `imagescale` | — | `input.c:743` (cache) | image scaling |
| `label` | `"\N"` | `input.c:729-731`, `utils.c:426` | node label |
| `layer` | — | `input.c:746` (cache) | layer selection |
| `margin` | shape default (`PAD`) | `shapes.c:1994-2008` (polygon), `shapes.c:3516` (record) | label padding in inches |
| `nojustify` | false | `input.c:745` (cache); `emit.c` | justify to bbox not shape |
| `ordering` | — | `input.c:735` (cache); `mincross.c:494` | "out"/"in" ordering |
| `orientation` | 0 | `input.c:740` (cache) | polygon rotation |
| `penwidth` | 1 | `input.c:734` (cache) | outline width |
| `peripheries` | shape default | `input.c:738` (cache) | polygon rings |
| `regular` | — | `shapes.c:1957ff` (via `late_string`) | equal w/h |
| `shape` | `"ellipse"` | `utils.c:424-425` | shape binding |
| `showboxes` | 0 | `utils.c:438-441` | debug boxes |
| `sides` | shape default | `input.c:737` (cache) | polygon sides |
| `skew` | 0 | `input.c:739` (cache) | polygon skew |
| `style` | "" | `input.c:725` (cache) | draw style words |
| `vertices` | — | `input.c:749` (cache) | custom polygon |
| `width` | `0.75` (min `0.01`) | `utils.c:420-421` | inches |
| `xlabel` | — | `utils.c:432-436` | exterior label (`NODE_XLABEL`) |
| `z` | 0 | `input.c:750` (cache) | 3-D (neato) |
| `group` | — | see above | |

**Edge attributes** (bound `input.c:753-781`; consumed where noted)

| attribute | default | consumed at | effect |
|---|---|---|---|
| `arrowsize` | 1.0 | `input.c:775` (cache) | arrow scale |
| `color` | black | `input.c:755` (cache) | pen color |
| `comment` | — | `input.c:778` (cache) | output comment |
| `constraint` | true | `class1.c:22-29` (`nonconstraint_edge`, used `dotinit.c:73-76`) | false ⇒ `xpenalty=0, weight=0` |
| `decorate` | false | `input.c:774` (cache) | line under labels |
| `dir` | `forward` | `input.c:763` (cache) | arrow ends |
| `fillcolor` | `color` | `input.c:756` (cache) | arrowhead fill |
| `fontcolor/fontname/fontsize` | black/Times-Roman/14 | `utils.c:445-450` (`initFontEdgeAttr`) | label fonts (base for label fonts) |
| `headclip`/`tailclip` | true (empty ⇒ true) | `utils.c:546-555` (`noClip`) | disable endpoint clipping |
| `headlabel`/`taillabel` | — | `utils.c:522-534` | port labels (`HEAD_LABEL`/`TAIL_LABEL`) |
| `headport`/`tailport` | `""` | `utils.c:540-553` (`agget(e, TAIL_ID/HEAD_ID)`) | `name[:compass]` |
| `label` | — | `utils.c:506-512` | `EDGE_LABEL`, label vnode |
| `label_float` (`labelfloat`) | `"false"` | `utils.c:511` | label may overlap edges |
| `labelangle` | `PORT_LABEL_ANGLE −25` | `splines.c:1345` | port label angle (min −180) |
| `labeldistance` | 1.0 | `splines.c:1346` | ×10 pt distance (min 0) |
| `labelfontcolor/fontname/fontsize` | edge's | `utils.c:452-460` (`initFontLabelEdgeAttr`) | head/tail label fonts |
| `layer` | — | `input.c:777` (cache) | layers |
| `minlen` | 1 (min 0) | `dotinit.c:85` | rank separation units |
| `penwidth` | 1 | `input.c:781` (cache) | spline width |
| `samehead`/`sametail` | — | `dotsplines.c:733-734` (aux) | shared arrow endpoints |
| `showboxes` | 0 | `dotinit.c:78-84` | debug boxes (clamped 255) |
| `style` | "" | `input.c:773` (cache) | line style |
| `weight` | 1 (min 0) | `dotinit.c:65` | ranking weight |
| `xlabel` | — | `utils.c:514-520` | exterior label |
| `lhead`/`ltail` | — | `compound.c:274-382` | `compound=true` cluster endpoints |

---

## 5. Packing pipeline (`lib/pack/pack.c`, `lib/pack/ccomps.c`, `dotinit.c:doDot`)

### 5.1 `doDot` (`lib/dotgen/dotinit.c:396-471`)

```pseudocode
doDot(g):
  Pack = getPack(g, -1, CL_OFFSET)                 # :404  pack attr; absent ⇒ -1
  mode = getPackModeInfo(g, l_undef, &pinfo)       # :405  packmode attr parsed (defaults l_undef)
  getPackInfo(g, l_node, CL_OFFSET, &pinfo)        # :406  pinfo.margin = getPack(g,8,8);
                                                   #        doSplines=false; fixed=NULL; re-parse packmode
  if mode == l_undef && Pack < 0:
      return dotLayout(g)                          # :408-415 classic path, components during layout
  # packing requested
  if mode == l_undef: pinfo.mode = l_graph         # :418-419 (pack=true, no packmode)
  elif Pack < 0:      Pack = CL_OFFSET               # :420-421 (packmode set, no pack ⇒ margin 8)
  pinfo.margin = (unsigned)Pack                    # :423
  pinfo.fixed   = NULL                             # :424
  ccs = cccomps(g, &ncc, 0)                        # :428  cluster-aware components
  if ncc == 1:                return dotLayout(g)  # :429-435
  elif ratio_kind == R_NONE:                       # :435-450
      pinfo.doSplines = true
      for sg in ccs: initSubg(sg, g); dotLayout(sg)
      attachPos(g)                                 # ND_pos = PS2INCH(ND_coord)   :324-334
      packSubgraphs(ncc, ccs, g, &pinfo)           # :448
      resetCoord(g)                                # ND_coord = INCH2PS(ND_pos)   :339-348
      copyClusterInfo(ncc, ccs, g)                 # rebuild cluster tree on root :375-394
  else: return dotLayout(g)                        # :451-462 nontrivial ratio ⇒ no split
  cleanup ccs                                      # :464-468
```

Defaults recap:
- `pinfo.margin` when `pack=true` ⇒ `CL_OFFSET = 8` points; `pack=17` ⇒ 17; `packmode` set without `pack` ⇒ 8; neither ⇒ no packing at all.
- `packmode` default in `doDot` (when `pack` set, `packmode` absent) ⇒ **`l_graph`** (bounding-box packing), *not* `l_node` — `getPackInfo`'s `l_node` default (`pack.c:1285-1298`) is only visible to other callers (neato uses `l_undef`, circogen `l_node`, osage `l_array` with `DFLT_MARGIN`).
- `packmode=array...` ⇒ `l_array` with flags `PK_COL_MAJOR/PK_INPUT_ORDER/PK_USER_VALS/PK_TOP_ALIGN/PK_BOT_ALIGN/PK_LEFT_ALIGN/PK_RIGHT_ALIGN` (`pack.h:57-63`, parser `pack.c:1144-1187`) and `sz` = trailing integer > 0; `packmode=aspect_f` ⇒ `l_aspect`, `aspect=f` (default 1) — but note dot never implements `l_aspect` packing (putGraphs handles `<= l_graph` and `l_array` only, `pack.c:892-929`).
- `doSplines` true only in the multi-component, ratio-less dot path (`dotinit.c:436`), or via `pack_graph()` API (`pack.c:1136`).

### 5.2 `getPack` / `getPackModeInfo` / `getPackInfo` (`pack.c:1257-1298`)

```c
int getPack(Agraph_t *g, int not_def, int dflt) {          // pack.c:1270-1283
    int v = not_def;
    if ((p = agget(g, "pack"))) {
        if (sscanf(p, "%d", &i) == 1 && i >= 0) v = i;     // numeric ≥ 0
        else if (*p == 't' || *p == 'T')        v = dflt;  // "true"/"t" ⇒ dflt
    }
    return v;
}   // note: "false"/"no"/"" ⇒ not_def (no packing) because sscanf fails
```
`parsePackModeInfo(p, dflt, pinfo)` (`pack.c:1212-1252`): resets `flags=0, mode=dflt, sz=0, vals=NULL`; `p` starts-with `array` ⇒ `l_array` + flags (`chkFlags` consumes `_c_i_u_t_b_l_r` letters) + optional integer size; starts-with `aspect` ⇒ `l_aspect`, `aspect = parsed f>0 else 1`; `"cluster"`/`"graph"`/`"node"` ⇒ those modes; anything else (incl. NULL) keeps `dflt`.
`getPackInfo(g, dflt, dfltMargin, pinfo)` (`pack.c:1285-1298`): `pinfo->margin = getPack(g, dfltMargin, dfltMargin); doSplines=false; fixed=NULL; return getPackModeInfo(g, dflt, pinfo);` (the re-parse also wipes flags/sz/vals set by an earlier parse).

### 5.3 `cccomps` (`lib/pack/ccomps.c:437ff`)

Decomposes `g` into "connected" components where connectivity is by edge **or** shared cluster membership: binds `ccgraphinfo/ccgnodeinfo` records, builds `deriveGraph(g)` (clone where cluster nodes are merged), then DFS-marks nodes and emits one subgraph per component (`agsubg(dg, "<pfx><i>", 1)`); `*ncc` = count. Components containing clusters carry cloned cluster subgraphs (`subgInduce`, `ccomps.c:411-426`) and `mapClust` links clones to originals (`pack.h:111`).

### 5.4 Placement engines (`pack.c`)

`putGraphs` (`pack.c:892-929`): mode `<= l_graph` ⇒ `polyGraphs`; `l_array` ⇒ `compute_bb` each graph then `arrayRects`.

`polyGraphs` (`pack.c:786-890`) essentials (for `l_node`/`l_clust`/`l_graph`):
1. `compute_bb(g)` per graph; fixed graphs contribute `fixed_bb` if `pinfo->fixed`.
2. `stepSize = computeStep(ng, bbs, margin)` — abort if ≤ 0.
3. Per graph: `l_graph` ⇒ `genBox(GD_bb, ...)` (box polyomino, `pack.c:228-262`); else `genPoly(root, gs[i], ...)` (node/cluster polyomino from `ND_pos` in inches + optional splines when `doSplines`, `fillEdge` `pack.c:171-223`).
4. Sort by decreasing perimeter (`cmpf`, `pack.c:104-115`); first graph centered on origin (`placeFixed`/`placeGraph`, `pack.c:480ff`); scan-line point-set placement (`newPS/addPS`).
5. Result: array of translation points.

`computeStep` (`pack.c:62-99`) — verbatim math:
```c
#define C 100                      // pack.c:33  "Max. avg. polyomino size"
a = C*(double)ng - 1;  b = 0;  c = 0;
for each bb:  W = bb.UR.x-bb.LL.x + 2*margin;  H = bb.UR.y-bb.LL.y + 2*margin;
              b -= W + H;      c -= W*H;
d = b*b - 4*a*c;  r = sqrt(d);
l1 = (-b + r)/(2*a);  l2 = (-b - r)/(2*a);
root = (int)l1;  if (root == 0) root = 1;
return root;                       // grid cell size in points
```

`arrayRects` (`pack.c:605-715`) essentials:
- Rows/cols: `sz = pinfo->sz`; column-major (`PK_COL_MAJOR`): `nr = sz>0 ? sz : ceil(sqrt(ng))`, `nc = ceil(ng/nr)`; else row-major symmetric.
- Per graph: `width = bb width + pinfo->margin`, `height = bb height + pinfo->margin` (margin added **once**, not doubled).
- Order: `pinfo->vals` (sortv, `PK_USER_VALS`) asc; else descending `height+width` (`acmpf`) unless `PK_INPUT_ORDER`.
- Column widths/row heights = max of members; prefix-summed; position = left/top-aligned, right/bottom-aligned, or centered (`round((widths[c]+widths[c+1]-bb.UR.x-bb.LL.x)/2)` etc.).
- Returns integer-rounded offsets.

`putRects`/`packRects` (`pack.c:931-971`): rectangle-level API; `l_node/l_clust` unsupported (NULL), `l_graph` ⇒ `polyRects` (same polyomino machinery on boxes), `l_array` ⇒ `arrayRects`.

### 5.5 Translation and bbox fixup

`shiftGraphs(ng, gs, pp, root, doSplines)` (`pack.c:1041-1080`): for each graph, `dx,dy = pp[i]` and `fx,fy = PS2INCH(dx),PS2INCH(dy)`; per node: `ND_pos[0..1] += f*` **and** `ND_coord += {dx,dy}`; `ND_xlabel->pos` too; if `doSplines`, `shiftEdge` translates all Bézier points, `sp/ep`, and the 4 label positions (`pack.c:974-997`); then `shiftGraph` translates `GD_bb` and the (set) cluster labels recursively over clusters (`pack.c:999-1017`).

`packSubgraphs` (`pack.c:1108-1128`): `packGraphs` (= `putGraphs` + `shiftGraphs`, `pack.c:1093-1102`), then recomputes the root bbox with `compute_bb(root)` and expands it by every component's cluster bboxes, storing into `GD_bb(root)`.

`pack_graph()` public API (`pack.c:1131-1142`): `getPackInfo(root, l_graph, CL_OFFSET, &info); info.doSplines = true; info.fixed = fixed; packSubgraphs(...); dotneato_postprocess(root);`

---

## 6. Bounding box, size/ratio, margin, centering

### 6.1 How the final `GD_bb` of the root is produced in dot

1. **Rank heights & cluster boxes** — `set_ycoords` (`position.c:756-860`) computes per-rank `ht1/ht2` (including self-edge labels: `ht2 = fmax(ht2, ED_label->dimen.y/2)`, `position.c:772-778`), cluster `ht1/ht2` with margins (`position.c:787-793`), `clust_ht` recursion adding `GD_border` reservations (§3.7), rank separation `delta = fmax(pht2+pht1+GD_ranksep, ht2+ht1+CL_OFFSET)` (`position.c:805-807`), and (flip case) `adjustRanks`.
2. **x-coordinates** — LP/network-simplex with cluster containment via `ln/rn` virtual nodes (`contain_nodes` `position.c:1101-1125`, margins `GD_border[LEFT/RIGHT].x`).
3. **Cluster/root bbox** — `set_aspect(g)` first calls `rec_bb(g,g)` ⇒ `dot_compute_bb` per cluster bottom-up (`position.c:891-910`): `bb = union(node half-sizes) ∪ GD_bb(subclusters)`, then `GD_bb(g) = {LL,UR}` (`position.c:900-901`).
4. **ratio scaling** — `set_aspect` (`position.c:930-998`, §6.2) rescales `ND_coord` (rounded) and all bboxes (`scale_bb`, `position.c:915-924`).
5. **Splines** — `dot_splines_` (`dotsplines.c:228`) starts from that bbox and grows it with `update_bb_bz(&GD_bb(g), cp)` per Bézier segment (`dotsplines.c:1277` in the label-aware aux-graph path; the regular path grows via `clip_and_install`/`updateBB` calls, e.g. `dotsplines.c:428-462` which `updateBB`s edge labels and port labels). `update_bb_bz` (`lib/common/emit.c:754-794`) recursively bisects (`Bezier(cp,0.5,…)` until `check_control_points` says near-linear, tolerance `HW`² distance-to-line) then unions the 4 control points.
6. **Exterior labels** — `addXLabels` `updateBB`s (§1.6).
7. **Root label space + translation** — `gv_postprocess` §1.1; final LL = origin.

Note: for `splines=""` (`EDGETYPE_NONE`), `dot_splines_` returns before drawing (comment `dotsplines.c:482-486`: *"If the splines attribute is defined but equal to "", skip edge routing"*), so no spline growth happens.

### 6.2 `size` / `ratio` semantics

Parsing (`input.c:476-501, 576-598, 685-687`):
- `size="W,H"` or `"W"` (inches; each > 0), trailing `!` ⇒ `GD_drawing->filled = true`.
- `ratio`: `"auto"` ⇒ `R_AUTO`; `"compress"` ⇒ `R_COMPRESS`; `"expand"` ⇒ `R_EXPAND`; `"fill"` ⇒ `R_FILL`; numeric `> 0` ⇒ `R_VALUE` with `GD_drawing->ratio = atof(p)`; unrecognized/`≤0` ⇒ `R_NONE` (no change).

Application for dot (`lib/dotgen/position.c`):
- `R_FILL` / `R_AUTO`: `set_aspect` computes `xf = size.x/sz.x, yf = size.y/sz.y` (sz = bb size, swapped if `GD_flip`); if either < 1, normalize the smaller to 1 (`if xf<yf {yf/=xf; xf=1} else {xf/=yf; yf=1}`); `R_AUTO` first synthesizes `size` from `page` via `idealsize(g, 0.5)` (`position.c:1130-1159`, subtracts `GD_drawing->margin` — always `{0,0}` for dot since nothing assigns it — and rounds up to whole pages).
- `R_EXPAND`: scale up uniformly by `fmin(xf,yf)` only if **both** > 1.
- `R_VALUE`: stretch one axis: `actual = sz.y/sz.x`; `actual < desired` ⇒ `yf = desired/actual, xf = 1` else `xf = actual/desired, yf = 1`.
- Scaling: if `GD_flip`, swap xf/yf; `ND_coord = round(coord * f)` for all nodes; `scale_bb` multiplies every cluster/root bbox coordinate (`position.c:987-996`). Guard: only applied when `GD_maxrank(g) > 0` (`position.c:936`).
- `R_COMPRESS`: not a scale — `compress_graph` (`position.c:505-528`) adds an aux edge `GD_ln→GD_rn` of length `min(size.x (or .y if flip), USHRT_MAX)` with weight 1000, forcing the layout solver to squeeze the drawing into `size`.
- **Plain `size` (no `ratio`)**: *not* applied during layout at all; at render time `init_job_viewport` (`emit.c:3363-3422`) computes `Z = fmin(size.x/sz.x, size.y/sz.y)` when the drawing is too big, or when `filled` (`!`) and the drawing is too small in both axes (`emit.c:3376-3385`); `job->zoom = Z` scales the output. `gv_postprocess` never rescales.

### 6.3 `margin` and `pad` — three distinct mechanisms

1. **Cluster box margin** (layout): graph attr `margin` (inches) → `G_margin` (`input.c:717`) → `late_int(g, G_margin, CL_OFFSET=8, 0)` at the seven `position.c` sites (§3.7). Int value = points.
2. **Node label margin** (layout): node attr `margin` in inches (§3.4, `shapes.c:1994-2008, 3516`).
3. **Output margin/pad** (rendering only): `init_gvc` (`emit.c:3200-3260`) parses graph `margin` (`"%lf,%lf"` inches → `gvc->margin`, default format-dependent: `DEFAULT_PRINT_MARGIN 36` pt paged, `DEFAULT_EMBED_MARGIN 0` embedded) and `pad` (`gvc->pad`, default `DEFAULT_GRAPH_PAD 4` pt, `emit.c:3303`). The job bbox is `GD_bb ± pad` (`emit.c:3370-3371`).
   **There is no path from the graph `margin` attribute into `GD_border[4]`** — `GD_border` is written only by `do_graph_label` (cluster labels, §3.7). If an older port map claims "graph margin → GD_border", it refers to this cluster-margin vs. cluster-label-border coupling, which are separate.

### 6.4 Centering

`center=true` sets `GD_drawing->centered` (`input.c:689`); consumed only at render time to center the image within the page (`emit.c:1267-1272`). Layout always translates the drawing so `GD_bb.LL == (0,0)` (§1.1); there is no layout-time centering for dot.

---

## 7. Verbatim constant appendix

All from `lib/common/const.h` unless noted:

```c
/* defaults (const.h:47-98) */
DEFAULT_COLOR            "black"
DEFAULT_FONTSIZE         14.0
DEFAULT_LABEL_FONTSIZE   11.0        /* for head/taillabel — currently UNREFERENCED */
MIN_FONTSIZE             1.0
DEFAULT_FONTNAME         "Times-Roman"   /* iPhone: "TimesNewRomanPSMT" */
DEFAULT_FILL             "lightgrey"
LINESPACING              1.20
DEFAULT_NODEHEIGHT       0.5
MIN_NODEHEIGHT           0.02
DEFAULT_NODEWIDTH        0.75
MIN_NODEWIDTH            0.01
DEFAULT_NODESHAPE        "ellipse"
DEFAULT_NODEPENWIDTH     1.0
MIN_NODEPENWIDTH         0.0
NODENAME_ESC             "\\N"
DEFAULT_NODESEP          0.25
MIN_NODESEP              0.02
DEFAULT_RANKSEP          0.5
MIN_RANKSEP              0.02
DEFAULT_PRINT_MARGIN     36          /* points, paged formats */
DEFAULT_EMBED_MARGIN     0           /* points, embedded formats */
DEFAULT_GRAPH_PAD        4           /* points */
SELF_EDGE_SIZE           18
MC_SCALE                 256         /* port order quantum, mincross */
PORT_LABEL_DISTANCE      10          /* points */
PORT_LABEL_ANGLE         -25         /* degrees, CCW pos / CW neg */
DFLT_SAMPLE              20
GAP                      4           /* const.h:251 — points; PAD = +4*GAP x, +2*GAP y */
CL_OFFSET                8           /* const.h:142 — cluster box margin, points */
CL_BACK                  10
CL_CROSS                 1000        /* 100 on _WIN32 */

/* sides (const.h:111-114) */
BOTTOM_IX 0, RIGHT_IX 1, TOP_IX 2, LEFT_IX 3
BOTTOM 1<<0, RIGHT 1<<1, TOP 1<<2, LEFT 1<<3

/* label flags (const.h:167-178) */
EDGE_LABEL (1<<0), HEAD_LABEL (1<<1), TAIL_LABEL (1<<2), GRAPH_LABEL (1<<3),
NODE_XLABEL (1<<4), EDGE_XLABEL (1<<5)
LABEL_AT_BOTTOM 0, LABEL_AT_TOP 1, LABEL_AT_LEFT 2, LABEL_AT_RIGHT 4

/* rankdir (const.h:181-184) */
RANKDIR_TB 0, RANKDIR_LR 1, RANKDIR_BT 2, RANKDIR_RL 3

/* edge types (const.h:233-243) */
EDGETYPE_NONE 0, EDGETYPE_LINE 2, EDGETYPE_CURVED 4, EDGETYPE_PLINE 6,
EDGETYPE_ORTHO 8, EDGETYPE_SPLINE 10, EDGETYPE_COMPOUND 12, NEW_RANK (1<<4)
EDGE_TYPE(g) = GD_flags(g) & (7 << 1)          /* macros.h:25 */

/* node/edge classes (const.h:24-45) */
NORMAL 0, VIRTUAL 1, SLACKNODE 2, REVERSED 3, FLATORDER 4, CLUSTER_EDGE 5, IGNORED 6
NOCMD 0, SAMERANK 1, MINRANK 2, SOURCERANK 3, MAXRANK 4, SINKRANK 5, LEAFSET 6, CLUSTER 7
LOCAL 100, GLOBAL 101, NOCLUST 102

/* drawing phases (const.h:162-164) */
GVBEGIN 0, GVSPLINES 1

/* units (geom.h:58-64, arith.h:48) */
POINTS_PER_INCH 72
POINTS(a_inches)  = ROUND((a_inches)*72)
INCH2PS(a) = a*72.0 ;  PS2INCH(p) = p/72.0
ROUND(f) = (f>=0) ? (int)(f+.5) : (int)(f-.5)

/* pack (pack.c:33, pack.h:55-76) */
C 100
pack_mode { l_undef, l_clust, l_node, l_graph, l_array, l_aspect }
PK_COL_MAJOR (1<<0), PK_USER_VALS (1<<1), PK_LEFT_ALIGN (1<<2), PK_RIGHT_ALIGN (1<<3),
PK_TOP_ALIGN (1<<4), PK_BOT_ALIGN (1<<5), PK_INPUT_ORDER (1<<6)

/* dotsplines.c */
LBL_SPACE 6                    /* dotsplines.c:944 — gap between stacked flat-edge labels */
NSUB ~= 10 (cell subdivision in spline routing)

/* mincross (mincross.c:1754-1755) */
MinQuit 8, MaxIter 24  (scaled by mclimit via scale_clamp, min 1)

/* rank.c new ranker (rank.c:539-545) */
BACKWARD_PENALTY 1000, STRONG_CLUSTER_WEIGHT 1000, NORANK 6
```

---

## 8. Key pseudocode recap (port checklist)

```pseudocode
# --- translation to origin, complete ---
postprocess(g):
    placeClusterLabels(g)                      # §1.3 (uses GD_border, label_pos)
    placeXLabels(g)                            # §1.6 (map solver, updates GD_bb)
    if rootLabel && !set:
        d = labelDimen + (16,8)
        growBbForLabel(g, d, rankdir, flip)    # §1.1 step 3
    Offset = perRankdirOffset(GD_bb(g))        # §1.1 table
    translateDrawing(g):                       # §1.2
        for n: if rankdir: nodesize(n, unflipped); coord = rot90(rankdir)*coord − Offset
               xlabel likewise; edges if splines computed
        bb(g) = rotatedBb(bb(g), rankdir) − Offset   (LL/UR corner mapping per §1.2)
        recurse into clusters
    if rootLabel && !set: placeRootLabel(g, d) # §1.3
```

```pseudocode
# --- label dimension (text) ---
labelDimen(text, fontsize, font, charset):
    lines = split on \n \l \r (justification kept), \\ → \
    for each line:
        h = line=="" ? (int)(fontsize*1.2) : spanHeight(line)
        w = max(w, spanWidth(line))
    dimen = (w, Σh);  space = dimen
spanWidth(line)  = fontsize * Σ(charWidth_1pt(byte)/units_per_em)   # LUT §3.3
spanHeight(line) = fontsize * 1.20                                  # LINESPACING
```

```pseudocode
# --- defaults pipeline for dot ---
graph_init:  nodesep=POINTS(max(0.02, parse_or(0.25))) → int
             ranksep=POINTS(clamp(parse_or(0.5), ≥0.02)) → int; "equally" → exact_ranksep
             rankdir→(rd<<2)|rd; clusterrank maptoken; concentrate mapbool
             setRatio; size/page getdoubles2ptf; do_graph_label
dotLayout:   setEdgeType(g, SPLINE); setAspect (disabled warn); init subg/node/edge
dot_rank:    newrank? rank2 : rank1(+edgelabel_ranks)
dot_position:set_ycoords(+clust_ht+adjustRanks) → aux edges(+margin+GD_border) →
             ns with nslimit → set_xcoords → set_aspect(ratio) → rec_bb
dot_splines: per EDGETYPE route; place_vnlabel ×3; update_bb_bz + updateBB(labels);
             State=GVSPLINES; EdgeLabelsDone=1
compound?    dot_compoundEdges
postprocess: §above
```

### Known divergences to be careful about in a Rust port

1. `late_int` returns the *default* (not the minimum) on unparseable input, but the *minimum* on values below minimum; `late_double` behaves identically. Negative `nslimit*` values become `0` via `scale_clamp`.
2. `maptoken` returns the value at the **terminator slot** for absent/unknown strings — tables rely on this (e.g. `clusterrank` defaults to `LOCAL`, `fontnames` to `-1`).
3. `mapbool` treats unknown strings as **false**, while `late_bool`'s default is only used when the attribute symbol itself is missing.
4. `noClip` intentionally deviates: empty string means "keep clipping" (i.e. `tailclip=""` ≡ `true`).
5. `POINTS()` rounds half away from zero; `place_vnlabel`/`set_aspect` use `round()` (half away from zero too); `emit` uses `ROUND` consistently.
6. `GD_ranksep` is an int in points and `edgelabel_ranks` halves it with C integer division (`(x+1)/2` — round-up for positives).
7. The estimator font LUT covers only ASCII bytes 32–126 (+ NUL guard); every other byte contributes width 0 after a one-time warning.
8. `addXLabels` runs **only** when the layout set `State == GVSPLINES` for edge geometry presence; edge labels without geometry warn and are skipped.
9. `forcelabels` default is `true` (not false).
10. `aspect` is parsed only to emit a "disabled" warning in current main.
