# `dotsplines.c` — dot's edge spline router: exhaustive implementation spec

Source audited: `/tmp/graphviz-src/lib/dotgen/dotsplines.c` (2316 lines), current
Graphviz `main` (commit `2e92c7f2776f6611dd78fa83c213ecc65cc47bfa`). All line
numbers below refer to that file unless prefixed with another file name. This
document is written so that a Rust port can reproduce dot's output *bit-for-bit*
(in `f64` arithmetic, same iteration order).

---

## 0. Scope and a critical version note

`dotsplines.c` implements phase 3.5 of `dot`: after ranking, ordering, and
position (`ND_coord`, `ND_lw/rw`, `ND_ht`, `rank_t::ht1/ht2` are final), it
converts every edge of the layout into Bézier control-point lists stored in
`ED_spl(e)`.

**Names used in the task brief vs. this source.** Several names in the brief
(`make_edge_aux`, `make_big_box`, `compsplines`, `MA_*`, `MULTIP`, `REGULAR`,
`LINES_*`, `HT/DIST`, `REPEATED_EDGE`, `SDMULT`, `onearc`, `make_lrvn`,
`makeBox`, `boxesEqual`, `P2PF`, `label_vnode`, `GD_pt2`) are from the
**pre-2.40 (≤2014) dotsplines.c**. They do not exist in this tree. The modern
file has the same responsibilities with a different decomposition. Mapping:

| Legacy name (old file) | Meaning | Where it lives now |
|---|---|---|
| `make_edge_aux`, aux edge chain, `ED_minlen` tweaks | aux-graph ranking machinery | gone from splines; ranking-time aux edges are `create_aux_edges` (`dotgen/position.c`), `ED_minlen` is used only during ranking, never here |
| `REPEATED_EDGE`, `SDMULT`, `MULTIP`, `onearc` flags | grouping/spacing multi-edges with shared endpoints | per-group fan-out in `make_regular_edge` (`dotsplines.c:1892-1914`, `Multisep` spacing) and `makeSimpleFlat` / `make_flat_edge` steps; no aux edges |
| `REGULAR` / `HT` / `DIST` constants | multiplier knobs for aux ranking | gone; replaced by plain arithmetic on `GD_nodesep` (`Splinesep = nodesep/4`, `Multisep = nodesep`, `dotsplines.c:274-275`) |
| `MA_*` (`MA_2PI`, `MA_3PI/2`, …) trig tables, `MAKEMULTIEDGE` | spline re-aiming/box math | gone; box→spline conversion is `routesplines_` in `common/routespl.c` (pathplan); `NSUB = 9` here is only an allocation fudge (`dotsplines.c:36`) |
| `make_big_box`, `compsplines` | one big box covering inter-rank space | gone; inter-rank space is one `rank_box` per rank (`rank_box`, `dotsplines.c:2016-2028`) + per-edge `maximal_bbox`es |
| `make_lrvn` (left/right virtual nodes) | label vnodes | label virtual nodes are created in `class2.c`/`position.c` with `ND_label(vn)` set; here they are only consumed (`place_vnlabel` `dotsplines.c:491-502`, `maximal_bbox:2203-2206, 2223-2227`, `recover_slack:2073-2078`) |
| `makeBox`, `boxesEqual` | box helpers | boxes are plain `boxf` literals built inline; equality never tested |
| `P2PF` | point/pointf conversion | gone; everything is `pointf` (f64) |
| `LINES_*` | polyline output flags | `EDGETYPE_LINE` (`common/const.h:235`) handled via `makeLineEdge` (`dotsplines.c:1643-1705`) and straightening at `dotsplines.c:1810-1814, 1860-1868` |
| `label_vnode` pos | label placement | `place_vnlabel` (`dotsplines.c:491-502`), `ND_alg` flat-label copy (`dotsplines.c:293-298`, `setEdgeLabelPos:199-218`) |
| `GD_pt2`, `compound.cpp`, `lheads` | compound (cluster) edge clipping | `dot_compoundEdges` → `makeCompoundEdge` in `dotgen/compound.c:434` (details §11.3) |
| `makeStraightEdge` | straight-line routing | exists in `common/routespl.c:937` (`makeStraightEdges` wrapper); `EDGETYPE_LINE`/`EDGETYPE_CURVED` dispatch in `dotsplines.c:388-394` |

Everything else in the brief (boxes between ranks, channel algorithm, multi-edge
grouping, labels, self loops, flat edges, ports, arrows, `clip_and_install`,
`ED_spl` flags, `updateBB`) is present and is covered exactly below.

---

## 1. Data structures

### 1.1 Module-local (`dotsplines.c:64-72`)

```c
typedef struct {
  double LeftBound, RightBound;  // graph x-extent, grown by MINW each rank (279-287)
  double Splinesep;              // = GD_nodesep(g) / 4        (274)
  double Multisep;               // = GD_nodesep(g)            (275)
  boxf *Rank_box;                // cache, one entry per rank (allocated with
                                 //  maxrank+1 entries; lazy init in rank_box)
} spline_info_t;

typedef LIST(pointf) points_t;   // growable pointf array (util/list.h)
```

`path` (`common/types.h:81-88`): `{ port start; port end; size_t nbox; boxf *boxes; }`.
`pathend_t` (`types.h:73-79`): `{ boxf nb; pointf np; int sidemask; int boxn; boxf boxes[20]; }`.
`port` (`types.h:48-64`): `p` (offset from node center), `theta`, `bp` (port bbox),
`defined`, `constrained`, `clip`, `dyna`, `order`, `side` (bitmask TOP/BOTTOM/LEFT/RIGHT), `name`.
`bezier` (`types.h:89-96`): `{ pointf *list; size_t size; uint32_t sflag, eflag; pointf sp, ep; }`
— control points are concatenated cubic Béziers: 4 points per segment,
consecutive segments share exactly one anchor point (`size = 3*segments + 1`).
`sflag/sp` = explicit arrow start point, `eflag/ep` = explicit arrow end point.
`splines` (`types.h:98-102`): `{ bezier *list; size_t size; boxf bb; }`.

`splineInfo` (`types.h:66-71`), initialized at `dotsplines.c:125-126`:
`swapEnds = swap_ends_p`, `splineMerge = spline_merge`, `ignoreSwap = false`,
`isOrtho = false`.

### 1.2 Constants (verbatim)

| Constant | Value | Site | Meaning |
|---|---|---|---|
| `NSUB` | `9` | dotsplines.c:36 | "number of subdivisions, re-aiming splines"; used only in the work-array size `n_nodes + 20*2*NSUB` (line 338) |
| `MINW` | `16` | :38 | minimum box width in the edge path |
| `HALFMINW` | `8` | :39 | `MINW/2` |
| `FWDEDGE` | `16` | :41 | tree_index flag: edge stored in forward direction |
| `BWDEDGE` | `32` | :42 | tree_index flag: edge is reversed (needs `makefwdedge`) |
| `MAINGRAPH` | `64` | :44 | edge belongs to main graph (set for regular edges) — used to break `-C` (concentrate) groups |
| `AUXGRAPH` | `128` | :45 | edge belongs to aux graph (flat/self/other edges) |
| `GRAPHTYPEMASK` | `192` | :46 | `MAINGRAPH|AUXGRAPH` |
| `REGULAREDGE` | `1` | common/const.h:150 | tree_index edge-type bits |
| `FLATEDGE` | `2` | const.h:151 | 〃 |
| `SELFWPEDGE` | `4` | const.h:152 | self edge with ports |
| `SELFNPEDGE` | `8` | const.h:153 | self edge, no ports (`SELFEDGE == 8`) |
| `EDGETYPEMASK` | `15` | const.h:155 | OR of the four above |
| `EDGETYPE_NONE/LINE/CURVED/PLINE/ORTHO/SPLINE/COMPOUND` | `0,2,4,6,8,10,12` (`k<<1`) | const.h:234-240 | `EDGE_TYPE(g) = GD_flags(g) & (7<<1)` (macros.h:25); selected from `splines` graph attr |
| `NORMAL/VIRTUAL/SLACKNODE/REVERSED/FLATORDER/CLUSTER_EDGE/IGNORED` | `0..6` | const.h:24-30 | `ND_node_type` / `ED_edge_type` values |
| `EDGE_LABEL` | `1<<0` | const.h:167 | `GD_has_labels` bit: graph has edge labels |
| `LBL_SPACE` | `6` | dotsplines.c:944 | vertical gap between stacked flat-edge labels (points) |
| `FUDGE` | `4` | dotsplines.c:2173 | extra x-space in `maximal_bbox` so `beginpath/endpath` can keep boxes ≥2 wide |
| `FUDGE` | `2` | common/splines.c:376 area (`#define FUDGE 2`) | offset used inside beginpath/endpath box tweaks (`FUDGE-2 == 0` there) |
| `MILLIPOINT` | 0.001 (approx-equal epsilon) | common/globals? used by `APPROXEQPT` in `clip_and_install` | coincidence tolerance |
| `INIT_DELTA` / `LOOP_TRIES` | `10` / `15` | common/routespl.c:270-273 | `limitBoxes` sampling refinement |
| `FUDGE` (routespl) | `0.0001` | routespl.c:259 | y-inclusion tolerance in `limitBoxes` |
| `PI/2` constraints | `M_PI/2`, `-M_PI/2` | dotsplines.c:1802, 1840 | forced vertical tangents at virtual-node merges |
| self-edge sizing | `sizey/2`, `max(...,2.0)` | dotsplines.c:411, splines.c selfRight | `makeSelfEdge(edges, cnt, Multisep, sizey/2, …)` |

Also relevant input state produced by earlier phases: `ND_coord`, `ND_lw/rw`
(+`ND_mval` swap trick, §3.4), `ND_ht`, `ND_node_type`, `ND_alg`, `ND_label`,
`ND_out/in/flat_out/flat_in/other`, `ND_order`, `ND_rank`, `GD_rank(r) = {n, v[], ht1, ht2, pht1, pht2}`,
`GD_minrank/maxrank`, `GD_nodesep`, `GD_ranksep`, `GD_flip` (rankdir=LR/RL),
`GD_has_labels(root)`, `GD_bb`, `Concentrate` (global), `E_headlabel`,
`E_taillabel`, `E_labelangle`, `E_labeldistance` (globals from common/input).

---

## 2. Entry point and call site

Call chain (`dotgen/dotinit.c:293-303`):

```
dot_sameports(g);                 // samehead/sametail port merging (dotgen/sameport.c:41)
dot_splines(g);                   // dotsplines.c:486
if (mapbool(agget(g,"compound"))) dot_compoundEdges(g);   // compound.c:434
```

`dot_splines(g)` (`:486`) is exactly `dot_splines_(g, 1)`. The `normalize`
parameter exists because `make_flat_adj_edges` recursively calls
`dot_splines_(auxg, 0)` on a rotated clone (`:1236`) — the clone's splines are
already copied in the desired direction; only the top-level call normalizes
(`:439-440`).

## 3. `dot_splines_` — the driver (`:228-480`)

Pseudocode (every step in order):

```
fn dot_splines_(g, normalize) -> i32:
  et = EDGE_TYPE(g)                                        # :235
  if et == EDGETYPE_NONE: return 0                          # :239-240  ("splines=" empty ⇒ no routing at all)
  if et == EDGETYPE_CURVED:                                 # :241-247
      resetRW(g)                                            # restore self-loop-inflated rw (§3.4)
      if GD_has_labels(g.root) & EDGE_LABEL:
          warn "edge labels with splines=curved not supported in dot - use xlabels"
  # ORTHO branch (compiled only with #ifdef ORTHO):         # :250-266
  #   resetRW(g); if EDGE_LABEL: setEdgeLabelPos(g) then orthoEdges(g,true)
  #              else orthoEdges(g,false); goto finish      # lib/ortho/ortho.c — separate algorithm, out of scope
  mark_lowclusters(g)                                       # :271 dotgen/cluster.c — mark bottom clusters
  routesplinesinit()   # :272 pathplan init; return 0 on failure (silently skips routing)
  sd = spline_info_t { Splinesep: GD_nodesep(g)/4, Multisep: GD_nodesep(g) }   # :274-275
  edges: LIST(edge_t*) = []                                 # :249
  LeftBound = RightBound = 0.0                              # (zero-init of sd, :248)

  # ---- pass 1: classify and collect every routable edge ----   # :277-327
  n_nodes = 0
  for i in GD_minrank(g) ..= GD_maxrank(g):                 # rank order, ascending
      n_nodes += GD_rank(g)[i].n
      n = GD_rank(g)[i].v[0]
      if n != null: LeftBound  = min(LeftBound,  n.x - n.lw)
      n = GD_rank(g)[i].v[ GD_rank(g)[i].n - 1 ]            # only if rank non-empty
      if n != null: RightBound = max(RightBound, n.x + n.rw)
      LeftBound  -= MINW      # NOTE: applied every rank iteration (cumulative!)  # :285-286
      RightBound += MINW
      for j in 0 ..< GD_rank(g)[i].n:                       # left-to-right order
          n = GD_rank(g)[i].v[j]
          if ND_alg(n) != null:                             # n is the label vnode of a flat edge
              fe = ND_alg(n); ED_label(fe).pos = ND_coord(n); ED_label(fe).set = true   # :293-298
          if ND_node_type(n) != NORMAL and not spline_merge(n): continue        # :299-300
          for e in ND_out(n):                               # k=0.. while list[k]!=NULL
              if ED_edge_type(e) in {FLATORDER, IGNORED}: continue              # :302-303
              setflags(e, REGULAREDGE, FWDEDGE, MAINGRAPH)  # :304
              edges.append(e)
          for e in ND_flat_out(n):  setflags(e, FLATEDGE, 0, AUXGRAPH); edges.append(e)  # :307-311
          if ND_other(n) nonempty:                          # :312-325
              if ND_node_type(n) == NORMAL: swap(ND_rw(n), ND_mval(n))          # undo position.c loop inflation
              for e in ND_other(n): setflags(e, 0, 0, AUXGRAPH); edges.append(e)

  LIST_SORT(edges, edgecmp)                                 # :335 — qsort, see §4.3

  P.boxes    = calloc(n_nodes + 20*2*NSUB, sizeof(boxf))    # :338
  sd.Rank_box= calloc(i, sizeof(boxf))                      # :339 — i == maxrank+1 here
  if et == EDGETYPE_LINE:                                   # :341-348
      for n in GD_nlist(g):                                 # ND_next chain, layout order
          if ND_node_type(n)==VIRTUAL and ND_label(n): place_vnlabel(n)

  # ---- pass 2: route one equivalence group at a time ----   # :350-427
  l = 0
  while l < len(edges):
      ind = l
      e0 = edges[l]; l += 1
      le0 = getmainedge(e0)
      ea = e0 if (ED_tail_port(e0).defined or ED_head_port(e0).defined) else le0   # :353-357
      if ED_tree_index(ea) & BWDEDGE: ea = makefwdedge(fwdedgea, ea)               # :358-361
      cnt = 1
      while l < len(edges):                                # :363-386 (C `for(cnt=1; …; cnt++, l++)`)
          e1 = edges[l]; le1 = getmainedge(e1)
          if le1 is not le0: break                         # different endpoint pair ⇒ new group
          if ED_adjacent(e0): { cnt += 1; l += 1; continue }   # :367 — ALL flat adjacent edges join the
              # group at once, bypassing the port/label/MAINGRAPH tests below
          eb = e1 if ports(e1) else le1
          if ED_tree_index(eb) & BWDEDGE: eb = makefwdedge(fwdedgeb, eb)
          if portcmp(ED_tail_port(ea), ED_tail_port(eb)) != 0: break           # :377
          if portcmp(ED_head_port(ea), ED_head_port(eb)) != 0: break           # :379
          if (ED_tree_index(e0)&EDGETYPEMASK)==FLATEDGE and ED_label(e0) is not ED_label(e1): break  # :381
          if ED_tree_index(edges[l]) & MAINGRAPH: break    # "Aha! -C is on"        # :384
          cnt += 1; l += 1
      # dispatch                                          # :388-426
      if et == EDGETYPE_CURVED:
          edgelist = [getmainedge(edges[ind])] + edges[ind+1 .. ind+cnt-1]
          makeStraightEdges(g, edgelist, cnt, et, &sinfo)  # common/routespl.c:973 — even self/flat edges!
      elif agtail(e0) == aghead(e0):                       # self loop
          n = agtail(e0); r = ND_rank(n)
          if r == GD_maxrank(g):
              sizey = (ND_coord(GD_rank(g)[r-1].v[0]).y - ND_coord(n).y) if r > 0 else ND_ht(n)
          elif r == GD_minrank(g):
              sizey = ND_coord(n).y - ND_coord(GD_rank(g)[r+1].v[0]).y
          else:
              upy  = ND_coord(GD_rank(g)[r-1].v[0]).y - ND_coord(n).y
              dwny = ND_coord(n).y - ND_coord(GD_rank(g)[r+1].v[0]).y
              sizey = min(upy, dwny)                                            # :396-410
          makeSelfEdge(&edges[ind], cnt, sd.Multisep, sizey/2, &sinfo)          # :411
          for b in 0..cnt-1: if ED_label(edges[ind+b]): updateBB(g, ED_label(edges[ind+b]))  # :412-416
      elif ND_rank(agtail(e0)) == ND_rank(aghead(e0)):     # flat edge
          rc = make_flat_edge(g, sd, &P, &edges[ind], cnt, et)                  # :417-424
          if rc != 0: free Rank_box, edges, P.boxes; return rc
      else:
          make_regular_edge(g, &sd, &P, &edges[ind], cnt, et)                   # :425-426

  # ---- epilogue ----                                      # :429-479
  for n in GD_nlist(g):                                     # :430-435
      if ND_node_type(n)==VIRTUAL and ND_label(n):
          place_vnlabel(n); updateBB(g, ND_label(n))
  if normalize: edge_normalize(g)                           # :439-440
  finish:                                                   # :443 (ORTHO lands here)
  if (E_headlabel or E_taillabel) and (E_labelangle or E_labeldistance):        # :447-465
      for n in agfstnode..:                                 # head labels: agfstin loop, place_portlabel(e,true)
          ... updateBB(head_label) ; tail labels: agfstout loop, if place_portlabel(e,false) updateBB
  if et not in {ORTHO, CURVED}: routesplinesterm()          # :467-473
  free sd.Rank_box; free edges; free P.boxes
  State = GVSPLINES; EdgeLabelsDone = 1                     # :477-478
  return 0
```

Notes on exact semantics the port must keep:

* **Grouping key** is effectively "same main edge" = same
  `{(tail node,tail port),(head node,head port)}` after `getmainedge` plus the
  `portcmp`/label/`MAINGRAPH` tie-breakers. All flat edges between adjacent
  nodes (`ED_adjacent`) are forced into one group (`:367`) regardless of ports.
* `cnt` counts group members including the first; `edges[ind..ind+cnt-1]` is the
  group, in `edgecmp` order.
* The `LeftBound -= MINW; RightBound += MINW` at `:285-286` is **inside** the
  rank loop (not per-rank geometry): after `R+1` ranks the bounds have been
  padded by `MINW*(#ranks processed)` — reproduce verbatim for identical
  `rank_box` x-extents.
* `spline_merge(n)` (`:108-111`): `ND_node_type(n)==VIRTUAL && (|ND_in(n)|>1 || |ND_out(n)|>1)`
  — a virtual node where several chain segments merge (only via `concentrate`).
* `swap_ends_p(e)` (`:113-123`): walks `ED_to_orig` to the original edge; true
  iff `ND_rank(head) < ND_rank(tail)` (head drawn above tail), or equal ranks
  and `ND_order(head) < ND_order(tail)`. Used by `edge_normalize`.
* `getmainedge(e)` (`:99-106`): follow `ED_to_virt` to the innermost virtual
  edge, then `ED_to_orig` to the original — the *canonical* user edge.
* `makefwdedge(new, old)` (`:48-62`): bitwise-copies the whole `Agedgeinfo_t`
  and the `Agedge_t` header, then `AGTAIL(new)=AGHEAD(old)`,
  `AGHEAD(new)=AGTAIL(old)`, swaps tail/head ports, `ED_edge_type=VIRTUAL`,
  `ED_to_orig=old`. In Rust: construct a temporary reversed edge view carrying
  cloned port info; do **not** mutate the real edge.

## 3.4 The `rw`/`mval` swap trick

During `position`, nodes touched by self loops have `ND_rw` inflated to make
room, with the original saved in `ND_mval` (dotgen/position.c:126 uses
`Concentrate` etc.). `resetRW` (`:187-193`) swaps them back for every node with
`ND_other.list`. It is called at the top for CURVED and ORTHO; for the default
(SPLINE/LINE/PLINE) path the swap is done lazily inside the collection loop
(`:318-320`) exactly when the first `ND_other` edge of that node is collected.
`resetRW`-equivalents must be applied before reading `ND_rw` for bounds.

## 3.5 `setflags` (`:504-530`)

```
fn setflags(e, hint1, hint2, f3):
  f1 = hint1 != 0 ? hint1 :
       (tail == head) ? (tail_port.defined||head_port.defined ? SELFWPEDGE : SELFNPEDGE)
     : (rank(tail)==rank(head)) ? FLATEDGE : REGULAREDGE
  f2 = hint2 != 0 ? hint2 :
       f1==REGULAREDGE ? (rank(tail)<rank(head) ? FWDEDGE : BWDEDGE)
     : f1==FLATEDGE    ? (order(tail)<order(head) ? FWDEDGE : BWDEDGE)
     :                   FWDEDGE                     # self edges
  ED_tree_index(e) = f1 | f2 | f3
```

`ED_tree_index` is a scratch int on the edge record (also reused by mincross;
here it holds `EDGETYPEMASK` bits | direction bit | graph bit).

## 3.6 `portcmp` (`:128-142`, exported via dotprocs.h:64)

Three-way compare: undefined handling first (`p1 undefined`: `p0 defined → 1`
else `0`; `p0 undefined → -1`), then compare `.p.x`, then `.p.y`. **Does not**
compare `theta`, `side`, `bp`, or `order` — two ports with identical offsets
group together even if declared differently.

## 3.7 `edgecmp` (`:542-641`) — total order of the edge list

Given `e0`, `e1` (pointers to edges):

1. `et0 = ED_tree_index(e0)&EDGETYPEMASK` vs `et1`; **if `et0 < et1` return +1,
   if `et0 > et1` return −1** (i.e. *descending* type: SELFNPEDGE(8) group
   first, then SELFWPEDGE(4), FLATEDGE(2), REGULAREDGE(1)).
2. `le0=getmainedge(e0)`, `le1=getmainedge(e1)`; compare
   `abs(ND_rank(tail(le)) - ND_rank(head(le)))` ascending (`:565-576`).
3. Compare `fabs(ND_coord(tail(le)).x - ND_coord(head(le)).x)` ascending
   (`:578-589`).
4. Compare `AGSEQ(le0)` vs `AGSEQ(le1)` ascending (`:592-597`) — the sequence
   number of the *main* edge, giving a stable per-endpoint-pair witness id.
5. Port comparison: for each side pick `e_i` if either port is `defined` else
   `le_i`; if its index has `BWDEDGE`, build the reversed `makefwdedge` copy
   first (`:599-610`); then `portcmp(tail ports)` then `portcmp(head ports)`
   (`:611-614`).
6. Compare `ED_tree_index & GRAPHTYPEMASK` **ascending** (`:616-623`):
   MAINGRAPH(64) before AUXGRAPH(128).
7. If both are FLATEDGE, compare the raw `ED_label` *pointers* (`:625-632`).
8. Finally compare `AGSEQ(e0)` vs `AGSEQ(e1)` ascending (`:634-639`); else 0.

## 3.8 `edge_normalize` / `swap_spline` / `swap_bezier` (`:144-180`)

After all groups are routed: for every edge `e` in `agfstnode`/`agfstout` order,
if `swap_ends_p(e)` and `ED_spl(e)` exists, `swap_spline`:
reverse the order of the Bézier segments in `splines.list`, and within each
`bezier` reverse the point list and swap `sflag↔eflag`, `sp↔ep`. Result: all
control points run tail→head (comment `:168-172`; `place_portlabel` relies on
this, splines.c:1309).

## 3.9 Label positions before/after routing

* `place_vnlabel(n)` (`:491-502`): only for regular-edge label vnodes
  (`ND_in(n).size != 0`; flat-edge label vnodes are skipped). Find the original
  edge by following `ED_to_orig` from `ND_out(n).list[0]` until
  `ED_edge_type == NORMAL`. Then
  `width = GD_flip ? label.dimen.y : label.dimen.x`;
  `label.pos = (ND_coord(n).x + width/2, ND_coord(n).y)`; `label.set = true`.
  (The `+width/2` accounts for the label vnode's *center* being the left edge of
  the text: the vnode was widened leftwards during position.)
* `setEdgeLabelPos` (`:199-218`): for CURVED/ORTHO only — copies `ND_coord` of
  flat-edge label vnodes (`ND_alg`) into the label, runs `place_vnlabel` for
  regular-edge labels, and `updateBB` for each.
* Flat-edge labels between *non-adjacent* nodes and self-loop labels are placed
  inside their respective routines (§6, §7); the final GD_nlist sweep
  (`:430-435`) then re-places regular-edge labels and expands `GD_bb`.

---

## 4. Routing a regular (inter-rank) edge — `make_regular_edge` (`:1707-1917`)

This is the core "box channel" algorithm. Preconditions: `ND_rank(tail) !=
ND_rank(head)`, group already normalized so member 0 is a FWDEDGE-oriented
edge. Local state: `pointfs`, `pointfs2` (points_t), `P` (path), `tend`, `hend`
(pathend_t).

### 4.1 Step 1 — cross-rank "hack" edge (`:1723-1760`)

```
e = edges[0]; hackflag = false
if abs(ND_rank(tail(e)) - ND_rank(head(e))) > 1:          # spans >1 rank
    fwdedgea = {out: copy(e), in: copy(AGOUT2IN(e))}      # both directions of chain start
    if BWDEDGE: fwdedgeb = makefwdedge(e); fwdedgea.out.tail = head(e);
                fwdedgea.out.tail_port = head_port(e)
    else:       fwdedgeb = {out: copy(e)}; fwdedgeb.in = copy(AGOUT2IN(e))
    le = getmainedge(e); while ED_to_virt(le): le = ED_to_virt(le)   # innermost virtual edge
    fwdedgea.out.head = head(le)                          # the LAST vnode/real head of chain
    fwdedgea.out.head_port.defined = false
    fwdedgea.out.head_port.p = (0,0)
    fwdedgea.out.edge_type = VIRTUAL; fwdedgea.out.to_orig = e
    e = &fwdedgea.out; hackflag = true
elif BWDEDGE: e = makefwdedge(fwdedgea, e)
fe = e                                     # fe = edge whose tail anchor is used for clipping
```

Effect: for long edges the router routes a *first segment* from the real tail to
the last virtual node of the chain with no head port, then handles the remainder
(via `straight_path`) as a straight vertical run. `fwdedgeb` keeps the final
head-end info (`endpath` at `:1845` uses `&fwdedgeb.out` when `hackflag`).

### 4.2 Step 2 — build box path (`:1762-1881`)

```
if et == EDGETYPE_LINE and (pn = makeLineEdge(g, fe, &pointfs, &hn)) != 0:
    goto fanout                                     # straight-line case, §4.6
else:                                               # box-based routing
    is_spline = (et == EDGETYPE_SPLINE)
    boxes: LIST(boxf) = []
    segfirst = e; tn = tail(e); hn = head(e)
    b = tend.nb = maximal_bbox(g, sd, tn, NULL, e)   # :1771
    beginpath(P, e, REGULAREDGE, &tend, spline_merge(tn))        # common/splines.c:376
    b.UR.y = tend.boxes[tend.boxn-1].UR.y; b.LL.y = tend.boxes[tend.boxn-1].LL.y
    b = makeregularend(b, BOTTOM, ND_coord(tn).y - GD_rank(g)[ND_rank(tn)].ht1)  # :1775
    if b nonempty: tend.boxes[tend.boxn++] = b       # extend down to rank bottom
    smode = si = false
    while ND_node_type(hn) == VIRTUAL and not spline_merge(hn):   # :1780-1842
        boxes.append(rank_box(sd, g, ND_rank(tn)))   # inter-rank box, §5.2
        if not smode and (sl = straight_len(hn)) >= ((GD_has_labels(root)&EDGE_LABEL) ? 4+1 : 2+1):
            smode = true; si = true; sl -= 2          # straight-run shortcut trigger
        if not smode or si:
            si = false
            boxes.append(maximal_bbox(g, sd, hn, e, ND_out(hn).list[0]))  # rank-local box of vnode
            e = ND_out(hn).list[0]; tn = tail(e); hn = head(e)
            continue
        # smode: terminate segment at hn, route, then skip sl straight hops
        hend.nb = maximal_bbox(g, sd, hn, e, ND_out(hn).list[0])
        endpath(P, e, REGULAREDGE, &hend, spline_merge(head(e)))   # splines.c:573
        b = makeregularend(hend.boxes[hend.boxn-1], TOP, ND_coord(hn).y + GD_rank(g)[ND_rank(hn)].ht2)
        if b nonempty: hend.boxes[hend.boxn++] = b
        P.end.theta = M_PI/2; P.end.constrained = true             # :1802 vertical arrival
        completeregularpath(P, segfirst, e, &tend, &hend, &boxes)  # :1921
        ps = is_spline ? routesplines(P,&pn) : routepolylines(P,&pn)
        if et==EDGETYPE_LINE and pn>4: ps[1]=ps[0]; ps[3]=ps[2]=ps[pn-1]; pn=4   # :1810-1814
        if pn==0: free ps; free boxes/pointfs/pointfs2; return
        pointfs.extend(ps[0..pn]); free ps
        e = straight_path(ND_out(hn).list[0], sl, &pointfs)        # :2049 append last point twice
        recover_slack(segfirst, P)                                 # :2061 widen label vnodes
        segfirst = e; tn = tail(e); hn = head(e)
        boxes.clear()
        tend.nb = maximal_bbox(g, sd, tn, ND_in(tn).list[0], e)
        beginpath(P, e, REGULAREDGE, &tend, spline_merge(tn))
        b = makeregularend(tend.boxes[tend.boxn-1], BOTTOM, ND_coord(tn).y - GD_rank(g)[ND_rank(tn)].ht1)
        if b nonempty: tend.boxes[tend.boxn++] = b
        P.start.theta = -M_PI/2; P.start.constrained = true        # :1840 vertical departure
        smode = false
    # final segment: from tn (real tail after last straight run, or original tail)
    boxes.append(rank_box(sd, g, ND_rank(tn)))                     # :1843
    b = hend.nb = maximal_bbox(g, sd, hn, e, NULL)                 # :1844
    endpath(P, hackflag ? &fwdedgeb.out : e, REGULAREDGE, &hend, spline_merge(head(e)))  # :1845
    b.UR.y = hend.boxes[hend.boxn-1].UR.y; b.LL.y = hend.boxes[hend.boxn-1].LL.y
    b = makeregularend(b, TOP, ND_coord(hn).y + GD_rank(g)[ND_rank(hn)].ht2)     # :1849
    if b nonempty: hend.boxes[hend.boxn++] = b
    completeregularpath(P, segfirst, e, &tend, &hend, &boxes)
    boxes.clear()
    ps = is_spline ? routesplines(P,&pn) : routepolylines(P,&pn)
    if et==EDGETYPE_LINE and pn>4: ps[1]=ps[0]; ps[3]=ps[2]=ps[pn-1]; pn=4       # :1860-1868
    if pn==0: return (after freeing)
    pointfs.extend(ps[0..pn]); free ps
    recover_slack(segfirst, P)
    hn = hackflag ? head(fwdedgeb.out) : head(e)
```

`straight_len(n)` (`:2031-2047`): starting from vnode `n`, count consecutive
virtual successors reached via `ND_out(v).list[0]` while
`ND_node_type == VIRTUAL && ND_out(v).size==1 && ND_in(v).size==1 &&
ND_coord(v).x == ND_coord(n).x` (x compared against the **original** `n`).
Threshold: 5 hops without edge labels, 3 hops with labels (`4+1`/`2+1` at
`:1782-1787`) — beyond that, route one box-path segment, then emit the rest as
a straight polyline run (`straight_path` appends the last point twice so
`clip_and_install` sees two 4-point Béziers with a zero-length first segment).

### 4.3 `completeregularpath` (`:1921-1953`)

```
uleft  = top_bound(first, -1); uright = top_bound(first, +1)
lleft  = bot_bound(last,  -1); lright = bot_bound(last,  +1)
if any of them is non-null and getsplinepoints(x) == NULL: return   # neighbor not routed yet
for i in 0..tendp->boxn-1: add_box(P, tendp->boxes[i])
fb = P.nbox + 1                       # index PAST the first interrank box
lb = fb + LIST_SIZE(boxes) - 3
for i in 0..len(boxes)-1: add_box(P, boxes[i])
for i in hendp->boxn-1..0 (descending): add_box(P, hendp->boxes[i])
adjustregularpath(P, fb, lb)
```

`add_box` (`common/splines.c`) silently drops empty boxes
(`LL.x<UR.x && LL.y<UR.y` required). The `fb`/`lb` arithmetic encodes: box
layout is `[tail boxes…][rank box][vnode box][rank box][vnode box]…[head boxes]`;
`fb-1` is the first interrank box; `lb` the last; boxes alternate "rank-wide"
(even offset) and "vnode-local" (odd offset) — which `adjustregularpath` uses.

`top_bound(e, side)` (`:2088-2102`): among all out-edges `f` of `tail(e)`, find
the one with `side*(ND_order(head(f)) - ND_order(head(e))) > 0` that already has
a spline (directly or on its original), minimizing `side*(order(ans)-order(f))`
— i.e. the nearest routed neighbor on that side. `bot_bound(e, side)`
(`:2104-2118`) is the same on `ND_in(head(e))` comparing tail orders. If a
bound exists but has no spline yet (`getsplinepoints == NULL`,
common/splines.c:1361), `completeregularpath` returns without adding any boxes —
the edge falls back to a later group (this is why order matters).

### 4.4 `makeregularend` (`:1959-1965`)

```
BOTTOM: boxf{{b.LL.x, y}, {b.UR.x, b.LL.y}}    # extend the box down to y
TOP:    boxf{{b.LL.x, b.UR.y}, {b.UR.x, y}}    # extend the box up to y
```

### 4.5 `adjustregularpath` (`:1981-2014`)

```
for i in fb-1 ..= lb:                       # interrank boxes only
    if (i - fb) % 2 == 0:                   # rank-wide boxes: only widen if degenerate
        if bp1.LL.x >= bp1.UR.x:
            x = (LL.x+UR.x)/2; LL.x = x-8 (HALFMINW); UR.x = x+8
    else:                                   # vnode-local boxes: stretch to MINW
        if bp1.LL.x + 16 (MINW) > bp1.UR.x:
            x = (LL.x+UR.x)/2; LL.x = x-8; UR.x = x+8
for i in 0 ..< P.nbox-1:                    # all adjacent box pairs; guarantee ≥MINW overlap
    if i in [fb,lb] and (i-fb)%2==0:
        if bp1.LL.x + MINW > bp2.UR.x: bp2.UR.x = bp1.LL.x + MINW
        if bp1.UR.x - MINW < bp2.LL.x: bp2.LL.x = bp1.UR.x - MINW
    elif i+1 in [fb,lb] and (i+1-fb)%2==0:
        if bp1.LL.x + MINW > bp2.UR.x: bp1.LL.x = bp2.UR.x - MINW
        if bp1.UR.x - MINW < bp2.LL.x: bp1.UR.x = bp2.LL.x + MINW
```

(The comment at `:1967-1980` admits the second loop "doesn't do" what it was
meant to; reproduce as-is.)

### 4.6 Multi-edge fan-out (`:1883-1917`)

```
if cnt == 1:
    clip_and_install(fe, hn, pointfs, len(pointfs), &sinfo); return
dx = sd.Multisep * (cnt - 1) / 2                      # :1892
for k in 1 .. len(pointfs)-2:  pointfs[k].x -= dx      # shift interior points left
pointfs2 = copy(pointfs); clip_and_install(fe, hn, &pointfs2)
for j in 1 .. cnt-1:
    e = edges[j];  if BWDEDGE: e = makefwdedge(e)
    for k in 1 .. len(pointfs)-2: pointfs[k].x += sd.Multisep   # step right, one Multisep each
    pointfs2 = copy(pointfs); clip_and_install(e, head(e), &pointfs2)
```

Endpoints (`pointfs[0]` and last) are **not** shifted — all parallel edges share
the exact node boundary points; interior control points are spaced
`Multisep = GD_nodesep` apart, centered on the group. This is the modern
replacement for `REPEATED_EDGE`/`SDMULT` handling.

### 4.7 `makeLineEdge` (`:1643-1705`) — `splines=line`, long edges

Applies when `et == EDGETYPE_LINE`. Returns 0 (fall through to box routing) if
`abs(ND_rank(head)-ND_rank(tail)) == 1`, or `== 2` while the graph has edge
labels (adjacent-rank edges use the normal machinery "because the usual code
handles the interaction of multiple edges better", comment `:1639-1642`).
Otherwise:

* Normalize direction: if `tail(fe) == tail(orig)` go tail→head else head→tail
  (`*hp` receives the far node; start/end points include ports:
  `startp = ND_coord(tn)+ED_tail_port(e).p` etc.).
* Without a label: append `[startp, startp, endp, endp]` (pn=4) — one Bézier
  with collinear controls, i.e. a straight segment.
* With a label: `width/height = label.dimen` (swapped if `GD_flip`);
  `lp = ED_label(e)->pos`; if `leftOf(endp, startp, lp)` then
  `lp += (width/2, -height/2)` else `lp -= (width/2, -height/2)`;
  append `[startp, startp, lp, lp, lp, endp, endp]` (pn=7) — two Béziers with
  the bend at the label.
  `leftOf(p1,p2,p3)` (`:1625-1627`): `(p1.y-p2.y)*(p3.x-p2.x) - (p3.y-p2.y)*(p1.x-p2.x) > 0`.
* The caller ignores the return value except as a boolean (`:1764`); points land
  in `pointfs` and flow into the fan-out code of §4.6.

For adjacent-rank `EDGETYPE_LINE` edges the box path runs with
`routepolylines` and is then straightened (`:1860-1868`):
`ps[1] = ps[0]; ps[3] = ps[2] = ps[pn-1]; pn = 4` — the polyline collapses to a
single straight Bézier (only when `pn > 4`).

### 4.8 `recover_slack` / `resize_vn` (`:2061-2085`)

Walk the chain `vn = head(segfirst)`, while `VN && !spline_merge`:
advance `b` (box index) while `P.boxes[b].LL.y > ND_coord(vn).y`, skipping the
first box (`b starts at 0` — comment says "skip first rank box"; boxes beyond
`nbox` stop the loop). If `P.boxes[b].UR.y < ND_coord(vn).y` (vnode not inside
this box) continue to the next vnode. Then:

* labeled vnode: `resize_vn(vn, LL.x, UR.x, UR.x + ND_rw(vn))`
* otherwise: `resize_vn(vn, LL.x, (LL.x+UR.x)/2, UR.x)`

where `resize_vn(vn,lx,cx,rx)` sets `ND_coord.x = cx; ND_lw = cx-lx;
ND_rw = rx-cx` (`:2082-2085`) — the vnode is re-centered/slid inside the slack
the routed spline left unused. This mutates layout, so it must run in exactly
this order relative to later groups' `maximal_bbox`es.

---

## 5. Box geometry helpers

### 5.1 `maximal_bbox` (`:2175-2232`)

Returns the maximal axis-aligned box around `vn` that the path may occupy on
`vn`'s rank:

```
LeftB = ND_coord(vn).x - ND_lw(vn) - 4 (FUDGE)
left = neighbor(g, vn, ie, oe, -1)
if left:
    if cl = cl_bound(g, vn, left): nb = GD_bb(cl).UR.x + sd.Splinesep
    else:
        nb = ND_coord(left).x + ND_mval(left)               # mval == real rw (see §3.4)
        nb += (ND_node_type(left)==NORMAL) ? GD_nodesep(g)/2 : sd.Splinesep
    if nb < LeftB: LeftB = nb
    rv.LL.x = round(LeftB)                                   # C round(): half-away-from-zero
else: rv.LL.x = min(round(LeftB), sd.LeftBound)              # fmin, verbatim
RightB = if VIRTUAL with ND_label(vn): ND_coord(vn).x + 10    # room for own label  (:2203-2204)
         else: ND_coord(vn).x + ND_rw(vn) + 4
right = neighbor(...,+1)
if right:
    if cl = cl_bound(...): nb = GD_bb(cl).LL.x - sd.Splinesep
    else: nb = ND_coord(right).x - ND_lw(right) ± (NORMAL ? nodesep/2 : Splinesep)  # subtract
    if nb > RightB: RightB = nb
    rv.UR.x = round(RightB)
else: rv.UR.x = max(round(RightB), sd.RightBound)
if VIRTUAL with ND_label(vn):                                 # :2223-2227
    rv.UR.x -= ND_rw(vn); if rv.UR.x < rv.LL.x: rv.UR.x = ND_coord(vn).x
rv.LL.y = ND_coord(vn).y - GD_rank(g)[ND_rank(vn)].ht1
rv.UR.y = ND_coord(vn).y + GD_rank(g)[ND_rank(vn)].ht2
```

Key behaviors: virtual nodes "give" only `Splinesep = nodesep/4` to the channel,
real nodes `nodesep/2`; clusters truncate via their bbox ± `Splinesep`; the
node's own label vnode reserves `+10` then retreats by its (loop-inflated) `rw`.

### 5.2 `rank_box` (`:2016-2028`)

```
b = sd.Rank_box[r]
if b.LL.x == b.UR.x:            # uninitialized (zeroed) cache slot
    left0 = GD_rank(g)[r].v[0]; left1 = GD_rank(g)[r+1].v[0]
    b.LL.x = sd.LeftBound;  b.LL.y = ND_coord(left1).y + GD_rank(g)[r+1].ht2
    b.UR.x = sd.RightBound; b.UR.y = ND_coord(left0).y - GD_rank(g)[r].ht1
    sd.Rank_box[r] = b
return b
```

One full-width box spanning the inter-rank band between rank `r` and `r+1`
(this is the modern `make_big_box`). Because every edge shares these
cache entries, mutating an edge's rank box is impossible; the box is only read.

### 5.3 `neighbor` (`:2234-2256`)

Scan `GD_rank(g)[rank(vn)].v[order(vn) ± i]`, `i = 1,2,…` in direction `dir`,
first match wins: (a) virtual node carrying `ND_label`, (b) `NORMAL` node,
(c) any virtual node whose paths do not cross ours (`pathscross == false`).
Off-rank ends return NULL.

### 5.4 `pathscross` (`:2258-2299`)

`order = ND_order(n0) > ND_order(n1)`; if neither node has exactly one out-edge,
false. Walk up to 2 hops along out-edges (`ie/oe` as the counterpart chain):
if heads coincide, stop; if `order != (ND_order(na) > ND_order(nb))` the two
paths swap sides → true. Then the same walk on in-edges comparing tails.

### 5.5 Cluster interference: `cl_vninside` (`:2122-2125`), `REAL_CLUSTER`
(`:2132`), `cl_bound` (`:2136-2164`)

`cl_vninside(cl,n)` = `BETWEEN(GD_bb(cl).LL.x, n.x, GD_bb(cl).UR.x) &&
BETWEEN(GD_bb(cl).LL.y, n.y, GD_bb(cl).UR.y)`. `cl_bound` returns the cluster of
the adjacent node that interferes: for NORMAL adjacents, `REAL_CLUSTER(adj)`
(`ND_clust(adj) == g ? NULL : ND_clust(adj)`) if it is neither the tail's nor the
head's cluster of the current edge; for virtual adjacents, the tail-side then
head-side cluster of `ED_to_orig(ND_out(adj).list[0])`, additionally requiring
`cl_vninside(cl, adj)`.

---

## 6. Flat edges (`make_flat_edge`, `:1509-1622` and helpers)

Dispatch order (`:1519-1552`):

1. Sample edge `e = edges[0]`; if `BWDEDGE`, replace with `makefwdedge` copy
   (`:1522-1525`). `isAdjacent = ED_adjacent(e) or any ED_adjacent(edges[i])`
   (`:1521-1533`).
2. If adjacent (`nodes are next to each other in the rank`) →
   `make_flat_adj_edges` (`:1129-1288`), §6.4.
3. Else if `ED_label(e)` (labels force single-edge groups) →
   `make_flat_labeled_edge` (`:1321-1423`), §6.2.
4. Else if `et == EDGETYPE_LINE` → `makeSimpleFlat(tail,head,edges,cnt,et)`
   (`:1082-1117`), §6.1.
5. Else if `(tail_port.side==BOTTOM && head_port.side!=TOP) ||
   (head_port.side==BOTTOM && tail_port.side!=TOP)` →
   `make_flat_bottom_edges` (`:1425-1497`), §6.3.
6. Else the "top" route (default), `:1554-1621`.

### 6.1 `makeSimpleFlat` (`:1082-1117`) — multi-edge spindle

```
tp = coord(tn)+tail_port.p; hp = coord(hn)+head_port.p
stepy = cnt>1 ? ND_ht(tn)/(cnt-1) : 0;  dy = tp.y - (cnt>1 ? ND_ht(tn)/2 : 0)
for i in 0..cnt-1:
  if et in {SPLINE, LINE}:  points = [tp, {(2*tp.x+hp.x)/3, dy}, {(2*hp.x+tp.x)/3, dy}, hp]   # 4 pts
  else /* PLINE */:         points = [tp,tp, {(2*tp.x+hp.x)/3,dy}×3, {(2*hp.x+tp.x)/3,dy}×3, hp,hp]  # 10 pts
  dy += stepy
  clip_and_install(e, head(e), points, n)
```

The `(2a+b)/3` control points are the classic "tangent = 1/3 of the chord"
rule (equivalently, control at 1/3 and 2/3 of the chord horizontally, at the
offset `dy` vertically). No ports/labels by construction (checked by caller).

### 6.2 `make_flat_labeled_edge` (`:1321-1423`)

```
f = deepest ED_to_virt(e);  ln = tail(f)          # the label vnode between tn and hn
ED_label(e).pos = ND_coord(ln); set = true        # :1338-1339
if et == EDGETYPE_LINE:
    startp = coord(tn)+tail_port.p; endp = coord(hn)+head_port.p
    lp = label.pos; lp.y -= label.dimen.y/2
    points = [startp, startp, lp, lp, lp, endp, endp] (pn=7)
else:
    lb.LL.x = coord(ln).x - ND_lw(ln); lb.UR.x = coord(ln).x + ND_rw(ln)
    lb.UR.y = coord(ln).y + ND_ht(ln)/2
    ydelta = coord(ln).y - GD_rank(g)[rank(tn)].ht1 - coord(tn).y + GD_rank(g)[rank(tn)].ht2
    ydelta /= 6
    lb.LL.y = lb.UR.y - max(5, ydelta)            # :1359-1362  (label box height)
    makeFlatEnd(g, sp, P, tn, e, &tend, true);  makeFlatEnd(g, sp, P, hn, e, &hend, false)
    boxes[0] = { LL:(tend.boxes[-1].LL.x, tend.boxes[-1].UR.y), UR: lb.LL }
    boxes[1] = { LL:(tend.boxes[-1].LL.x, lb.LL.y), UR:(hend.boxes[-1].UR.x, lb.UR.y) }
    boxes[2] = { LL:(lb.UR.x, hend.boxes[-1].UR.y), UR:(hend.boxes[-1].UR.x, lb.LL.y) }
    P: tend.boxes (ascending) + boxes[0..2] + hend.boxes (descending)
    ps = routesplines (SPLINE) or routepolylines (PLINE)
clip_and_install(e, head(e), ps, pn)
```

`makeFlatEnd` (`:1290-1303`): `endp->nb = maximal_bbox(g,sp,n,NULL,e)`;
`sidemask = TOP`; `beginpath/endpath(P, e, FLATEDGE, endp, false)`; then take
the last box's y-extent and `makeregularend(b, TOP, ND_coord(n).y +
GD_rank(g)[ND_rank(n)].ht2)`, appending it if non-empty — i.e. extend the port
box up to the top of the node's rank band. `makeBottomFlatEnd` (`:1305-1319`) is
the mirror with `BOTTOM` and `coord(n).y - ht2`.

### 6.3 `make_flat_bottom_edges` (`:1425-1497`) and the default top route
(`:1554-1621`)

Both stack `cnt` parallel edges with fan steps; they differ only in direction
and the vertical space source:

* bottom route: `vspace = coord(tn).y - GD_rank(g)[r].pht1 -
  (coord(next_rank.v[0]).y + next_rank.pht2)` when `r < maxrank`, else
  `GD_ranksep(g)` (`:1437-1443`); uses `makeBottomFlatEnd`.
* top route: if `r > 0`, `prevr = GD_rank(g)+(r-2)` when the graph has edge
  labels (label vnodes occupy a rank!) else `r-1`;
  `vspace = coord(prevr.v[0]).y - prevr.ht1 - coord(tn).y - GD_rank(g)[r].ht2`;
  if `r == 0`, `vspace = GD_ranksep(g)` (`:1556-1567`); uses `makeFlatEnd`.

Shared body (top version shown; `:1574-1620`):

```
stepx = sp.Multisep/(cnt+1); stepy = vspace/(cnt+1)
makeFlatEnd(tn, true); makeFlatEnd(hn, false)
for i in 0..cnt-1:
  b  = tend.boxes[-1]
  boxes[0] = { LL:(b.LL.x, b.UR.y), UR:(b.UR.x + (i+1)*stepx, b.UR.y + (i+1)*stepy) }
  boxes[1] = { LL:(tend.boxes[-1].LL.x, boxes[0].UR.y),
               UR:(hend.boxes[-1].UR.x, boxes[0].UR.y + stepy) }
  b  = hend.boxes[-1]
  boxes[2] = { LL:(b.LL.x - (i+1)*stepx, b.UR.y), UR:(b.UR.x, boxes[1].UR.y?) }
      # verbatim: boxes[2].UR.x = b.UR.x; boxes[2].LL.y = b.UR.y;
      #           boxes[2].LL.x = b.LL.x - (i+1)*stepx; boxes[2].UR.y = boxes[1].LL.y  — note the
      #           assignment order in C (UR.y from boxes[1].LL.y AFTER LL.y set from b.UR.y)
  P: tend.boxes ascending + boxes + hend.boxes descending
  ps = routesplines (SPLINE) else routepolylines; clip_and_install(e, head(e), ps, pn); P.nbox = 0
```

The bottom route mirrors it with `UR.y/LL.y` negated steps (`:1450-1496`) and
resets `P->nbox = 0` after each edge (`:1495`).

### 6.4 `make_flat_adj_edges` (`:1129-1288`) — recursive dot on a rotated clone

For flat edges between rank-adjacent nodes (any ports/labels). Steps:

1. If either endpoint has shape `SH_RECORD`, warn once (`static atomic_flag`)
   and return without routing (`:1144-1152`).
2. Count `labels` and detect any `ports`.
3. **No ports:** `labels == 0` → `makeSimpleFlat`; else
   `makeSimpleFlatLabels` (§6.5). Return.
4. **With ports:** clone the graph (`cloneGraph`, `:780-825`): fresh `agopen`
   with same quantum/dpi/charset; `rankdir` forced to `LR` if the original is
   `TB`, else `TB` (`:794-797`) — i.e. the flat problem becomes a normal
   top-to-bottom problem; all node/edge attributes are re-declared
   (`agattr_text/html` copying defaults), `headport`/`tailport` attrs ensured,
   and `setState` (`:688-772`) swaps the module-global `E_*/N_*/G_*` attribute
   symbols to the clone's (saved in `attr_state_t`, restored by
   `cleanupCloneGraph` `:827-872`; several symbols are nulled — `E_constr`,
   `E_minlen`, `E_headlabel`, `E_taillabel`, `E_xlabel`, `N_showboxes`,
   `N_style`, `N_nojustify`, `N_group` — so the clone ignores them).
5. Subgraph `xxx` with `rank=source` holds `auxt` (`:1177-1179`);
   `cloneNode` (`:877-889`) copies attributes, wrapping record labels in
   `{...}` to survive rankdir change; `cloneEdge` (`:891-897`) copies attrs.
   Original edges get `ED_alg(e) = auxe`; the first port-less clone is `hvye`
   with `ED_alg(hvye) = e` and is given `weight = 10000` (`:1204`) to pin the
   layout; if none exists, a synthetic `hvye` edge is created (`:1201-1203`).
6. Run the pipeline on the clone: `dot_init_node_edge`, `dot_rank`,
   `dot_mincross`, `dot_position`, then `dot_sameports(auxg)` and
   `dot_splines_(auxg, 0)` (`:1208-1239`), then `dotneato_postprocess(auxg)`.
   Any nonzero code aborts the whole routing (propagated out of
   `dot_splines_` by `make_flat_edge`'s caller at `:419-424`).
7. **Reposition** (`:1222-1234`): `midx = (coord(tn).x - rw(tn) + coord(hn).x + lw(hn))/2`;
   `midy = (coord(auxt).x + coord(auxh).x)/2`; then for every clone node:
   `auxt: (x,y) ← (midy, rightx)`; `auxh: (x,y) ← (midy, leftx)`; others
   `y ← midx` where `rightx = coord(hn).x`, `leftx = coord(tn).x` captured
   before any `GD_flip` swap (`:1180-1181`) — the rotated coordinates are mapped
   back onto the real node axis.
8. **Copy splines** (`:1242-1284`): translation `del`:
   * `GD_flip(g)`: `del.x = coord(tn).x - coord(auxt).y`,
     `del.y = coord(tn).y + coord(auxt).x`
   * else: `del.x = coord(tn).x - coord(auxt).x`,
     `del.y = coord(tn).y - coord(auxt).y`
   `transformf(p, del, flip)` (`:900-907`): if flip, `(x,y) ← (y, -x)`; then
   `p += del`. For each edge (skipping the pseudo-edge `hvye` when
   `ED_alg(auxe)==NULL`): `bz = new_spline(e, auxbz->size)`, copy `sflag/sp`,
   `eflag/ep` (transformed), and every control point via `transformf`;
   `update_bb_bz(&GD_bb(g), cp)` per segment; if `ED_label(e)`, transform
   `ED_label(auxe)->pos`, `set = true`, `updateBB(g, label)`.
9. `cleanupCloneGraph` restores the global attribute symbols and destroys the
   clone (`:1286`).

### 6.5 `makeSimpleFlatLabels` (`:951-1080`) — adjacent flat edges with labels

```
earray = edges sorted by edgelblcmpfn          # :914-942: labeled first (−1);
                                               #  wider dimen.x first, then wider dimen.y
tp = coord(tn)+tail_port(e0).p; hp = coord(hn)+head_port(e0).p   # of the FIRST edge
leftend = tp.x + ND_rw(tn); rightend = hp.x - ND_lw(hn)
ctrx = (leftend + rightend)/2
e = earray[0]: straight [tp,tp,hp,hp]; label.pos = (ctrx, tp.y + (dimen.y + 6)/2)
miny = tp.y + 6/2; maxy = miny + dimen.y
uminx = ctrx − dimen.x/2; umaxx = ctrx + dimen.x/2
for i in 1 ..< n_lbls:                          # :993-1036
    if i odd  ("down"):  if i==1: lminx = ctrx−dimen.x/2; lmaxx = ctrx+dimen.x/2
                         miny −= LBL_SPACE + dimen.y
                         octagon: [tp, (tp.x, miny−6), (hp.x, miny−6), hp,
                                   (lmaxx, hp.y), (lmaxx, miny), (lminx, miny), (lminx, tp.y)]
                         ctry = miny + dimen.y/2
    else     ("up"):     octagon: [tp, (uminx, tp.y), (uminx, maxy), (umaxx, maxy),
                                   (umaxx, hp.y), hp, (hp.x, maxy+6), (tp.x, maxy+6)]
                         ctry = maxy + dimen.y/2 + 6;  maxy += dimen.y + 6
    ps = simpleSplineRoute(tp, hp, octagon, &pn, et==EDGETYPE_PLINE)   # pathplan
    if failed: return (edge silently unrouted)
    label.pos = (ctrx, ctry); set = true; clip_and_install(e, head(e), ps, pn)
for i in n_lbls ..< cnt:                        # unlabeled edges, :1039-1077
    if i odd ("down"): if i==1: lminx = (2*leftend + rightend)/3; lmaxx = (leftend + 2*rightend)/3
                       miny −= 6; same "down" octagon as above
    else:              same "up" octagon; maxy += 6      # note: plain `+= LBL_SPACE` at :1064
    simpleSplineRoute + clip_and_install
```

`simpleSplineRoute` (`common/render.h:142`, pathplan) routes the two endpoints
through the 8-gon with splines (or polyline when `et==EDGETYPE_PLINE`).

---

## 7. Self loops

Self edges never reach `make_flat_edge`/`make_regular_edge`; they are handled
entirely in `dot_splines_` (`:395-416`) by `makeSelfEdge(edges+ind, cnt,
sd.Multisep, sizey/2, &sinfo)` (`common/splines.c:1162`):

* sizey (vertical room available): computed from the neighboring rank centers —
  at maxrank: distance to rank above (`r>0`) else the node's own height;
  at minrank: distance to rank below; else `min(up,down)` (`:396-410`).
* `makeSelfEdge` chooses among `selfRight`, `selfLeft`, `selfTop`, `selfBottom`
  based on port sides (`splines.c:1162-1205`):
  no ports, or all ports inside/right with at most 1 top and 1 bottom →
  `selfRight`; any LEFT port → `selfLeft` (or `selfTop` if the other side is
  RIGHT); both on TOP → `selfTop`; both on BOTTOM → `selfBottom`.
* `selfRight` (`splines.c`, cited above) arithmetic — reproduced because it is
  what most loops look like:

```
stepy = max(sizey/2/cnt, 2.0);  np = coord(n)
tp = np + tail_port.p;  hp = np + head_port.p
sgn = (tp.y >= hp.y) ? +1 : -1
dx = ND_rw(n); dy = 0
point_pair = convert_sides_to_points(tail.side, head.side)   # splines.c:742ff, vertex-pair code
if point_pair in {32, 65} and tp.y == hp.y: sgn = -sgn
tx = min(dx, 3*(np.x+dx − tp.x)); hx = min(dx, 3*(np.x+dx − hp.x))
for i in 0..cnt-1:
    dx += stepx; tx += stepx; hx += stepx; dy += sgn*stepy
    points = [ tp,
               (tp.x + tx/3, tp.y + dy),
               (np.x + dx,   tp.y + dy),
               (np.x + dx,   (tp.y+hp.y)/2),
               (np.x + dx,   hp.y − dy),
               (hp.x + hx/3, hp.y − dy),
               hp ]                                       # 7 points = 2 Béziers
    if ED_label(e): label.pos = (np.x + dx + labelwidth/2, np.y), set=true
                    if width > stepx: dx += width − stepx
    clip_and_install(e, head(e), points, 7)
```

`selfLeft` mirrors with `ND_lw` and minus signs (pair codes `{12,67}`); the
control points again sit at 1/3 of the horizontal run (`tx/3`), i.e. the same
1/3-tangent family as the rest of dot. After the group, each labeled edge gets
`updateBB(g, ED_label(e))` (`:412-416`).

`EDGETYPE_CURVED` **does not** use this path — curved mode routes self edges
through `makeStraightEdges` too (§9), bending them toward the nearest cycle's
centroid.

---

## 8. Where the Bézier points come from: `routesplines` / `routepolylines`

`dotsplines.c` never computes curve geometry itself; it only builds
`path {start port, end port, boxes[]}` and calls
`routesplines(P,&pn)` / `routepolylines(P,&pn)` (`common/routespl.c:595-600`,
impl `routesplines_` at `:316`). Contract, in order:

1. Resolve `P.data` to the NORMAL edge (`realedge`).
2. `checkpath` sanity (boxes non-degenerate).
3. Build the containing polygon from the box chain (`boxn*8` vertex budget):
   for each box emit 2 vertices on the correct side depending on whether the
   chain steps up or down (`prev`/`next` from comparing `LL.y` of neighbors);
   error "illegal values of prev %d and next %d" on contradictions; "edge is a
   loop" error if start/end on same box side.
4. If `GD_flip` (rankdir LR/RL), y coordinates are pre-negated on input and
   negated back on output (`flip` block, routespl.c:429-439).
5. `Pshortestpath(poly, eps=[start,end])` → shortest polyline `pl` inside the
   polygon (pathplan library).
6. `polyline ? make_polyline(pl,&spl) : Proutespline(edges_of_polygon, pl,
   end_vectors, &spl)` — the end vectors are `(cos θ, sin θ)` at start and
   `(-cos θ, -sin θ)` at end when `constrained`.
7. **`limitBoxes`**: sample each Bézier segment at
   `num_div = INIT_DELTA(10) * boxn` uniformly-spaced De Casteljau points
   (`t = si/num_div`, two intermediate bisection levels as written at
   routespl.c:233-256) and shrink every box's `[LL.x, UR.x]` to the sampled
   x-extent where the sample's y is within `FUDGE=0.0001` of the box's y-range.
   Repeat up to `LOOP_TRIES=15` times doubling `delta` until all boxes got
   values; horizontal/vertical splines are detected and take the trivial bound;
   total failure falls back to limiting by the shortest polyline with a
   warning ("Unable to reclaim box space…"). The shrunk boxes are stored back
   into `P->boxes` — this is how later edges reclaim channel space, and why
   edge order affects output.
   **Port trap (fixed 2025)**: the six lerps at routespl.c:244-255 must end
   with `sp[0]` holding the curve point — they are three De Casteljau levels
   `(sp[0],sp[1])`, `(sp[1],sp[2])`, `(sp[2],sp[3])` applied in C's order.
   An implementation that only updates `sp[3]`, `sp[2]`, `sp[1]` leaves `sp[0]`
   at `pps[splinepi]`, so a single box is sampled, the "all boxes bounded"
   test never succeeds, every edge pays all 15 doubling retries, and the
   channel-reuse state written back into `P->boxes` is wrong (1.dot: 17.5s of
   routing instead of 0.014s, with visibly wrong edge geometry).
8. Return the raw control-point array (`spl.pn` points = `3*segments+1`).
   Return NULL + `npoints=0` on any error (callers in dotsplines.c treat that
   as "edge silently unrouted", freeing state and returning).

Thus **the Bézier shape is entirely determined by the box chain, the two
endpoints, the two constrained tangent directions, and the pathplan library**.
A faithful port must reimplement `Pshortestpath` and `Proutespline`
(`lib/pathplan/`) — they are the "1/2,1/2 tangent/curvature" geometry engine;
dotsplines.c's own control points (the `(2a+b)/3` and `tx/3` formulas above)
appear only in the flat/self/straight special cases.

---

## 8b. Ports and records (port notes)

1. **`chkPort` runs once, at edge init** (common/utils.c:534-560): the port
   text comes from the `:port` syntax or the `tailport`/`headport` attribute,
   is split at the first `:` and dispatched to the shape's `portfn`
   (`poly_port` for polygons, `record_port` for records). `ND_has_port(n)` is
   set when the text is non-empty (it feeds `rcross`'s port corrections and
   the spline group ordering).
2. **The default port is not "no port"**: `static port Center = {.theta = -1,
   .clip = true}` (shapes.c:38) — an edge with no port aims at the node centre
   and *is* clipped to the outline. Only an explicit port turns `clip` off,
   and `clip_and_install` must honour that flag (splines.c:253-288); clipping
   unconditionally pulls a ported endpoint back to wherever the spline happens
   to cross the outline.
3. **`dyna` ports (`_`) resolve at routing time** (`resolvePort`,
   shapes.c:4322): `closestSide` picks the allowed side nearest the other
   endpoint, then `compassPort` re-resolves. `beginpath`/`endpath` do this
   before reading the port (splines.c:382-383, 584-585).
4. **`dot_sameports`** (sameport.c) runs *after* position and *before*
   splines: `samehead`/`sametail` groups aim the whole group at the average
   direction of their far endpoints, clipped to the outline; the port is
   installed on every edge of the group and every virtual edge of its chain.
5. **Records are sized before the axes are swapped** — `record_init` runs
   inside `common_init_node`, `gv_nodesize(flip)` afterwards — so the *painted*
   record frame equals the record tree's frame, while the splines work in the
   rotated layout frame (`record_inside` rotates its query point by
   `90*rankdir`, shapes.c:3762-3785). `record_gencode` paints the field boxes
   and their texts.

## 9. `EDGETYPE_CURVED` and `EDGETYPE_LINE` — `makeStraightEdges`
(`common/routespl.c:937-1042`, wrapper `makeStraightEdge` at `:919`)

Called from `dot_splines_` once per group (`:388-394`) with the group's edges
(the first replaced by its main edge).

```
e = edge_list[0]; start = coord(tail)+tail_port.p; end = coord(head)+head_port.p
dumb = [start, start, end, end]
if e_cnt == 1 or Concentrate:
    if et == EDGETYPE_CURVED: bend(dumb, get_cycle_centroid(g, edge_list[0]))
    clip_and_install(e, head(e), dumb, 4); addEdgeLabels(e); return
if APPROXEQPT(start, end, MILLIPOINT):      # degenerate
    dumb[1]=dumb[0]; dumb[2]=dumb[3]; del=(0,0)
else:
    perp = (start.y − end.y, end.x − start.x);  l = hypot(perp)
    xstep = GD_nodesep(g.root)
    dx = xstep*(e_cnt−1)/2
    dumb[1] = start + dx*perp/l ; dumb[2] = end + dx*perp/l
    del = −xstep*perp/l
for i in 0..e_cnt-1:
    e0 = edge_list[i]
    dumber = dumb, or reversed if head(e0) != head(edge_list[0])   # orient each edge
    if et == EDGETYPE_PLINE: make_polyline(dumber) then clip_and_install
    else clip_and_install(e0, head(e0), dumber, 4)
    addEdgeLabels(e0)                       # == makePortLabels (head/tail labels)
    dumb[1] += del; dumb[2] += del          # march one nodesep toward the line
```

`bend` (`routespl.c:913-927`): `midpt = (dumb[0]+dumb[3])/2`,
`r = DIST(dumb[3],dumb[0])/5`, direction = unit vector from `midpt` to
`centroid` (which is the centroid of the shortest directed cycle of length ≥3
containing the edge, else the graph centroid; see `get_cycle_centroid`
`routespl.c:889-906`); set both middle control points to
`midpt − v/mag*r`. This is the entire "curved" mode: a single quadratic-looking
cubic whose waist is pulled `dist/5` toward the cycle centroid.

`EDGETYPE_LINE` uses the same routine with no bend, plus the box-based
straightening in `make_regular_edge` for adjacent ranks (§4.7), plus
`makeLineEdge` for long edges, plus the flat `makeSimpleFlat` (§6.1), plus the
labeled-flat 7-point polyline (§6.2), plus `place_vnlabel` pre-pass
(`:341-348`).

---

## 10. Endpoint clipping and arrows: `clip_and_install`
(`common/splines.c:234-318`) — the final output step

Called as `clip_and_install(fe, hn, ps, pn, &sinfo)` where `fe` is the
(port-bearing) edge view used for routing and `hn` the head node the points run
toward. Sequence:

1. `tn = tail(fe)`; `newspl = new_spline(fe, pn)` — walk `ED_to_orig` until
   `ED_edge_type==NORMAL`, allocate/append a `bezier` with `pn` slots on
   `ED_spl(orig)` (`splines.c:211-225`).
2. Swapped-flat-edge guard: if `!info->ignoreSwap && rank(tn)==rank(hn) &&
   order(tn)>order(hn)`, swap `tn/hn` (`splines.c:248-250`).
3. Decide which of `orig`'s ports corresponds to `tn` (tail vs head, i.e.
   whether `fe` is reversed relative to `orig`); read
   `clipTail/clipHead` (`port.clip` flags) and `tbox/hbox` (`port.bp`).
4. Shape clipping: if `clip` and the node shape has an `insidefn`, binary-search
   the first/last control-point index whose 4th point is outside the shape
   (`start` from 0 stepping 3 while inside; `end` from `pn-4` stepping −3) and
   call `shape_clip0` to intersect the endpoint segment with the shape
   perimeter. `port.bp` (the port target box) is passed inside `inside_t`.
5. Skip degenerate zero-length segments: advance `start`/`end` while
   `APPROXEQPT(ps[i], ps[i+3], MILLIPOINT)`.
6. `arrow_clip(fe, hn, ps, &start, &end, newspl, info)` (`splines.c:60-88`):
   resolve original edge, `j = info->swapEnds(e)`, `arrow_flags(e,&sflag,&eflag)`
   (arrowhead styles from `dir`/`arrowhead`/`arrowtail`, honoring
   `dir=back` by swapping; see `common/arrows.c`), suppress the head arrow if
   `info->splineMerge(hn)` and the tail arrow if `splineMerge(tail(fe))`
   (concentrate merge points), swap flags if `j`; then
   `arrowStartClip`/`arrowEndClip` (non-ortho) trim `start`/`end` and set
   `newspl->sp` (with `sflag`) / `newspl->ep` (`eflag`) — the pulled-back
   arrow-tip base points; `arrowOrthoClip` for ortho. The amount pulled back is
   the arrow size computed from the penwidth/style (arrows.c `arrow_gen`);
   `port.clip == false` (set by `beginpath/endpath` when a compass port put the
   endpoint on the node boundary) prevents double-clipping.
7. Emit: for `i` in `start .. end+4` (step 1, with the exact interleaved copy
   shown at splines.c:300-315) copy points into `newspl->list[i-start]` and feed
   every 4-tuple to `update_bb_bz(&GD_bb(g), cp)`. Finally
   `newspl->size = end - start + 4`.

So the fields the user asked about are: **`ED_spl(orig)->list[k]`** = the Bézier
segments; **`bezier.sflag/sp`** = explicit arrow start (only when an arrow was
clipped); **`bezier.eflag/ep`** = arrow end; graph bbox grown via
`update_bb_bz`. `headclip=false`/`tailclip=false` map to
`port.clip = false` (skips shape clipping; arrows still apply).

`tailclip/headclip`/`dir=back` details live in `common/arrows.c`
(`arrow_flags`, `arrow_start_clip`, `arrow_end_clip`) and in the port resolution
`resolvePort` (compass points, `dyna` ports) in `common/splines.c`; `dotsplines.c`
itself only propagates ports it reads (`ED_tail_port(e).defined/.side/.p`).

---

## 11. Cluster/compound interactions

### 11.1 In this file
* `mark_lowclusters(g)` (`:271`, dotgen/cluster.c) — before routing, marks the
  lowest cluster each node/edge belongs to; needed by `beginpath/endpath`'s
  cluster-aware box code and `ND_clust` used by `cl_bound`.
* `cl_bound`/`cl_vninside` (§5.5) shrink `maximal_bbox`es against neighboring
  cluster bounding boxes with `Splinesep` clearance.
* Inter-cluster edges are routed as ordinary regular edges: their virtual
  chains include the cluster-boundary vnodes created in `class2.c`, so the
  boxes naturally stop at cluster borders.

### 11.2 `concentrate=true`
Creates merged virtual nodes (`spline_merge` true) and `IGNORED` edges — both
are skipped in collection (`:299-303`). Where edges do merge at a vnode,
`beginpath/endpath(…, merge=true)` sets `P->start/end.theta` from `conc_slope`
(splines.c:331-349: average of the incoming/outgoing slope angles) and arrows
are suppressed (§10.6). The `MAINGRAPH` bit at `:384` breaks multi-edge groups
when `-C` is on.

### 11.3 `compound=true` — `dot_compoundEdges` (dotgen/compound.c:434-443)
Runs **after** `dot_splines`. For each original edge whose `lhead`/`ltail` names
a cluster (`mkClustMap` resolves names → cluster graphs), `makeCompoundEdge`
(compound.c:~250-431) truncates the already-computed spline at the cluster
bounding box: finds the first Bézier crossing `GD_bb(lhead)` via
`splineIntersectf`, handles the degenerate "first control point inside the
box" case by rebuilding 4 points between the arrow head and the box
intersection using `mid_pointf` chains (`compound.c:323-331`), re-runs
`arrowEndClip`/`arrowStartClip` on the truncated segment, and replaces
`ED_spl(e)->list[0]` with the shortened Bézier (`compound.c:410-431`). There is
no `GD_pt2` in current sources.

---

## 12. Output state (what a port must produce)

Per original NORMAL edge `e` (reached through `ED_to_orig` chains):

* `ED_spl(e)`: `splines { list[k] = bezier{ list: 3n+1 pointf control points,
  size: 3n+1, sflag: uint32 arrow style or 0, sp: pointf, eflag, ep } }`.
  Multiple Béziers occur only for self loops (7-point two-segment) and for
  multi-segment box paths (routesplines output).
  **Orientation**: after `edge_normalize`, control points always run
  tail→head of the *original* edge (`swap_ends_p` test at `:113-123`).
* `ED_label(e)->pos` / `->set`:
  * regular edges: from `place_vnlabel` (§3.9) — `pos = (coord(vn).x +
    width/2, coord(vn).y)` with `width` swapped under `GD_flip`;
  * flat adjacent: `makeSimpleFlatLabels`/`make_flat_adj_edges` copy from the
    aux graph via `transformf`;
  * flat labeled non-adjacent: `ND_coord(ln)` of the label vnode
    (`:1338-1339`);
  * self loops: `selfRight`-family formula (§7).
* `GD_bb(g)`: grown by `update_bb_bz` for every routed Bézier segment and by
  `updateBB(g, label)` (`common/utils.c:613` = `addLabelBB(GD_bb(g), lp,
  GD_flip(g))`) for every placed label.
* Globals: `State = GVSPLINES` (const.h: GVBEGIN=0, GVSPLINES=1),
  `EdgeLabelsDone = 1` (`:477-478`).

## 13. Rust port notes (exactness checklist)

1. **Ordering**: reproduce `edgecmp` exactly (descending type bits!, ascending
   rank-diff, ascending |Δx|, `AGSEQ` of main edge, ports, graph bits ascending,
   label pointer → use a stable insertion index instead, `AGSEQ` of the edge).
   `qsort` is not stable — the final `AGSEQ(e0)` tie-break makes it total, so a
   stable Rust `sort_by` gives identical results.
2. **Float ops**: all coordinates are `f64`. Keep expression shapes: `(2*tp.x +
   hp.x)/3`, `dx = Multisep*(cnt-1)/2`, `sizey/2`, `ydelta/6`,
   `max(5.0, ydelta)`, `fmin/fmax` in `maximal_bbox` (note the `fmin(round(b),
   LeftBound)` direction), C `round()` (half away from zero — use
   `f64::round`-equivalent semantics, not `rint` banker's rounding).
3. **`beginpath`/`endpath`** mutate `P.start.p`/`P.end.p` by ±1 in several
   branches (the "keep the endpoint strictly inside the box" hack) and set
   `endp->sidemask`; port `dyna` triggers `resolvePort` (compass
   `n/e/ne/...`), `pboxfn` gives shapes (records, cylinders) the first chance
   to build end boxes. Port these before dotsplines works at all.
4. **`pathend_t.boxes` is a fixed 20-slot array**; `beginpath` writes
   `boxes[0..1]` directly — index arithmetic must match (`tend.boxes[tend.boxn-1]`
   before `makeregularend`).
5. **`makefwdedge`** copies the entire edge info struct; in Rust model the
   temporary reversed edge as an owned `EdgeView { tail, head, tail_port,
   head_port, edge_type: VIRTUAL, to_orig: Rc<...>, ..info.clone() }`.
6. **Scratch buffers**: `P.boxes` sized `n_nodes + 20*2*NSUB` (= `n_nodes+360`)
   with `NSUB=9`; `add_box` only appends non-degenerate boxes; `P.nbox = 0`
   resets between fan-out iterations.
7. **`limitBoxes` writes back into `P.boxes`** — a port that rebuilds boxes
   fresh per edge will not reproduce dot's channel reuse.
8. **Failure semantics**: `routesplines` returning NULL ⇒ the edge gets **no**
   `ED_spl` and routing continues; `make_flat_adj_edges` returning nonzero ⇒
   whole `dot_splines` aborts with that code.
9. **Recursive clone graph** (§6.4) needs the full dot pipeline on the clone;
   ports must not depend on graph-level `E_constr`/`E_minlen` etc. being
   present (they are nulled in `setState`).
10. **Warnings** to preserve verbatim: "flat edge between adjacent nodes one of
    which has a record shape…" (once), "edge labels with splines=curved not
    supported in dot - use xlabels", and the routespl.c messages.

## 14. Function index (file:lines)

| Function | Lines | Role |
|---|---|---|
| `makefwdedge` | 48-62 | reversed temp edge view |
| `spline_info_t` / `points_t` | 64-72 | routing state |
| forward decls | 74-97 | — |
| `getmainedge` | 99-106 | canonical original edge |
| `spline_merge` | 108-111 | vnode merge test |
| `swap_ends_p` | 113-123 | tail→head normalization test |
| `sinfo` | 125-126 | splineInfo with the two callbacks |
| `portcmp` | 128-142 | port ordering |
| `swap_bezier` / `swap_spline` / `edge_normalize` | 144-180 | direction fixup |
| `resetRW` | 187-193 | undo self-loop rw inflation |
| `setEdgeLabelPos` | 199-218 | label positions for curved/ortho |
| `dot_splines_` | 228-480 | driver (§3) |
| `dot_splines` | 486 | `dot_splines_(g,1)` |
| `place_vnlabel` | 491-502 | regular-edge label placement |
| `setflags` | 504-530 | tree_index bits |
| `edgecmp` | 542-641 | group sort order |
| `attr_state_t`/`setState`/`cleanupCloneGraph` | 643-872 | aux-graph attr swap |
| `cloneGraph` | 780-825 | rotated clone graph |
| `cloneNode`/`cloneEdge` | 877-897 | clone helpers |
| `transformf` | 900-907 | rotate+translate |
| `edgelblcmpfn` | 914-942 | label sort |
| `makeSimpleFlatLabels` | 951-1080 | adjacent flat edges w/ labels |
| `makeSimpleFlat` | 1082-1117 | adjacent flat spindle |
| `make_flat_adj_edges` | 1129-1288 | recursive dot routing |
| `makeFlatEnd`/`makeBottomFlatEnd` | 1290-1319 | flat endpoint boxes |
| `make_flat_labeled_edge` | 1321-1423 | one labeled flat edge |
| `make_flat_bottom_edges` | 1425-1497 | bottom-side flat routing |
| `make_flat_edge` | 1509-1622 | flat dispatcher |
| `leftOf` | 1625-1627 | orientation predicate |
| `makeLineEdge` | 1643-1705 | splines=line long edges |
| `make_regular_edge` | 1707-1917 | inter-rank router (§4) |
| `completeregularpath` | 1921-1953 | assemble+adjust box path |
| `makeregularend` | 1959-1965 | rank-band extension box |
| `adjustregularpath` | 1981-2014 | MINW enforcement |
| `rank_box` | 2016-2028 | inter-rank band box |
| `straight_len`/`straight_path` | 2031-2059 | straight run detection |
| `recover_slack`/`resize_vn` | 2061-2085 | vnode recentering |
| `top_bound`/`bot_bound` | 2088-2118 | routed neighbors |
| `cl_vninside`/`REAL_CLUSTER`/`cl_bound` | 2122-2164 | cluster obstacles |
| `maximal_bbox` | 2175-2232 | per-rank channel box |
| `neighbor`/`pathscross` | 2234-2299 | obstruction search |
| `showpath` (DEBUG) | 2302-2316 | PostScript dump |

Boundary functions to port alongside (not in this file):
`add_box`, `beginpath`, `endpath`, `conc_slope`, `resolvePort` (common/splines.c),
`clip_and_install`, `new_spline`, `shape_clip0`, `bezier_clip`, `arrow_clip`,
`arrow_flags`, `arrowStartClip`, `arrowEndClip`, `arrowOrthoClip`,
`place_portlabel`, `makePortLabels`, `makeSelfEdge`/`self{Right,Left,Top,Bottom}`,
`convert_sides_to_points` (common/splines.c, common/arrows.c),
`routesplinesinit/term`, `routesplines`, `routepolylines`, `limitBoxes`,
`makeStraightEdges`, `get_cycle_centroid`, `bend`, `addEdgeLabels`
(common/routespl.c), `Pshortestpath`, `Proutespline`, `make_polyline`,
`simpleSplineRoute` (pathplan), `dot_sameports` (dotgen/sameport.c),
`dot_compoundEdges`/`makeCompoundEdge` (dotgen/compound.c),
`updateBB`/`addLabelBB`/`update_bb_bz` (common/utils.c).
