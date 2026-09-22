# Graphviz `dot` auxiliary passes — implementation spec

**Files covered:** `flat.c`, `conc.c`, `sameport.c`, `aspect.c`, `compound.c` (plus the flat-edge
machinery that now lives in `mincross.c`/`fastgr.c`/`position.c`, because the versions of those
routines this checkout ships are no longer in `flat.c`).

**Source checkout:** `/tmp/graphviz-src` (Graphviz main branch; `lib/dotgen/`).
Citations of the form `flat.c:104-129` are `lib/dotgen/<file>` line ranges unless another
directory is named (e.g. `lib/common/const.h:142`).

> **Faithfulness warning — read this first.** The task brief for this document references the
> *historical* (≈ graphviz 2.28–2.39) API of some of these files. In this checkout several of
> those symbols no longer exist where the brief expects them:
>
> | Brief expects | In this checkout |
> |---|---|
> | `flat_lock`, `all_flat` (flat.c) | **Gone.** Replaced by the single flag `GD_has_flat_edges`, set only in `flat_edge()` at `fastgr.c:215-220`, queried in `mincross.c:1333`. |
> | `flat_search` / `flat_breakcycles` (flat.c) | **Moved to `mincross.c:1059-1090` / `mincross.c:1092-1117`.** Still DFS with `ND_mark`/`ND_onstack`; documented in §4.2. |
> | `flat_search` "labeling constraints solver" with `LT_LT/LT_RT/RT_LT/RT_RT`, `delta`, `lpos/rpos` arrays | **Gone entirely.** The modern label placement is `flat_node()` + `flat_limits()` (§3) with no case table; no such constants exist anywhere in this checkout. |
> | `ND_lw/rw/ht` "boxes around flat edges" | Label nodes only get `ND_ht = dimen.y`, `ND_lw = ND_rw = dimen.x/2` (`flat.c:163-165`); no port "p boxes" are built; `ED_head_port`/`ED_tail_port` of the two `FLATORDER` edges carry plain `p.x` offsets (`flat.c:169-174`). |
> | `aspect.c` with `R_NONE/R_VALUE/R_FILL/R_COMPRESS`, `A_ASPECT`, `badGraph`, `ASPECT_MATRIX` | **Gutted upstream.** `aspect.c` is now a 35-line stub that warns "the aspect attribute has been disabled due to implementation flaws - attribute ignored" (`aspect.c:27-35`). Documented verbatim in §7; the still-alive `ratio`/sizing machinery lives in `position.c:set_aspect` (§7.3). |
> | `makeClustEdge`, `markclusters` (compound.c) | **Gone.** Current compound clipping is `dot_compoundEdges` → `makeCompoundEdge` with `mkClustMap`/`findCluster` from `lib/common/utils.c`. Documented in §8. |
>
> A faithful Rust port must port the code **as it is in this checkout**; the historical
> algorithms are noted only where the difference matters.

---

## 0. Conventions

- "rank `r`" means `GD_rank(g)[r]` (a `rank_t`, `lib/common/types.h:200-213`).
- `order(n)` = `ND_order(n)`, `rank(n)` = `ND_rank(n)`, `coord(n)` = `ND_coord(n)`.
- All elists (`elist`, `lib/common/types.h:251-254`) are NULL-terminated arrays of `edge_t *`
  with a `size` field; iteration idiom `for (i = 0; (e = L.list[i]); i++)` relies on the
  terminator. `elist_append` reallocs to `size+2` and writes a NULL sentinel
  (`lib/common/types.h:261-266`).
- `SWAP(&a,&b)` swaps via a temp of the argument type.
- `MIN`/`MAX`/`ROUND`: `lib/common/arith.h:28-48`; `ROUND(f)` = `(f>=0)?(int)(f+.5):(int)(f-.5)`.
- `fcmp(a,b)`: sign of `a-b` (`lib/util/gv_math.h:15-23`).
- `INSIDE(p,b)`: `BETWEEN(b.LL.x,p.x,b.UR.x) && BETWEEN(b.LL.y,p.y,b.UR.y)`
  (`lib/common/geom.h:45`).
- `hypot`, `round`, `fabs` are C library semantics (round = half away from zero).
- Node types (`lib/common/const.h:24-30`): `NORMAL 0`, `VIRTUAL 1`, `SLACKNODE 2`,
  `REVERSED 3`, `FLATORDER 4`, `CLUSTER_EDGE 5`, `IGNORED 6`.
- `MC_SCALE 256` (`lib/common/const.h:99`); `CL_OFFSET 8` (`lib/common/const.h:142`).
- `GD_flip(g)` = `GD_rankdir(g) & 1` (`lib/common/types.h:378`).

### 0.1 Key struct fields (verbatim, `lib/common/types.h`)

```c
typedef struct port {            // types.h:48-64
    pointf p;                    // aiming point relative to node center
    double theta;                // slope in radians
    boxf  *bp;                   // if not null, bbox of rectangular port target
    bool defined;                // edge has port info at this end
    bool constrained;            // theta constraints set
    bool clip;                   // clip end to node/port shape
    bool dyna;                   // assign compass point dynamically
    unsigned char order;         // for mincross
    unsigned char side;          // bitwise OR of sides port is on
    char *name;                  // explicit port name or NULL
} port;

typedef struct bezier {          // types.h:89-96
    pointf *list; size_t size; uint32_t sflag, eflag; pointf sp, ep;
} bezier;

typedef struct splines {         // types.h:98-102
    bezier *list; size_t size; boxf bb;
} splines;

typedef struct textlabel_t {     // types.h:104-125
    char *text, *fontname, *fontcolor; int charset; double fontsize;
    pointf dimen;   // estimated diagonal size of the label
    pointf space;   // size of the space for the label
    pointf pos;     // center of the space for the label
    union { struct { textspan_t *span; size_t nspans; } txt; htmllabel_t *html; } u;
    char valign; bool set, html;
} textlabel_t;

typedef struct rank_t {          // types.h:200-213
    int n; node_t **v; int an; node_t **av;
    double ht1, ht2;             // height below/above centerline
    double pht1, pht2;           // same, primitive nodes only
    bool candidate, valid; int64_t cache_nc; adjmatrix_t *flat;
} rank_t;
```

Node fields used here (`types.h:410-481`, accessors 483-538): `ND_coord` (`pointf`),
`ND_ht`, `ND_lw`, `ND_rw` (doubles), `ND_label` (`textlabel_t*`), `ND_alg` (`void*` —
for a label node, points to the flat edge it labels), `ND_node_type`, `ND_rank`,
`ND_order`, `ND_mval` (`double`), `ND_has_port` (`bool`), `ND_mark` (`size_t`),
`ND_onstack` (`char`), `ND_low` (int; aliased to `ND_hops` storage in `rank.c:555`,
used as `flatindex` in mincross), `ND_in/ND_out/ND_flat_in/ND_flat_out/ND_other`
(`elist`), `ND_next/ND_prev` (fast node list), `ND_clust`, `ND_save_in/ND_save_out`.

Edge fields (`types.h:543-603`): `ED_tail_port`, `ED_head_port` (`port`),
`ED_label` (`textlabel_t*`), `ED_edge_type` (`char`, values from `const.h:24-30`),
`ED_adjacent` (`char`, "true for flat edge with adjacent nodes"), `ED_dist` (`double`,
largest label width of adjacent flat edges), `ED_to_orig`, `ED_to_virt`, `ED_alg`,
`ED_spl` (`splines*`), `ED_minlen` (`int`), `ED_weight` (`int`), `ED_count`,
`ED_xpenalty` (`short`), `ED_conc_opp_flag` (`bool`), `ED_compound`.

Graph fields (`types.h:278-349`): `GD_rank` (`rank_t*`), `GD_nlist`, `GD_minrank`,
`GD_maxrank`, `GD_n_cluster`, `GD_clust` ("clusters are in clust[1..n_cluster]!!!",
`types.h:310-311`), `GD_bb` (`boxf`), `GD_rankleader` (`node_t**`), `GD_has_flat_edges`
(`bool`), `GD_has_labels` (`unsigned char`, bit `EDGE_LABEL = 1<<0`,
`lib/common/const.h:167`), `GD_flip`, `GD_nodesep`, `GD_ranksep` (`int`),
`GD_drawing` (`layout_t*` with `ratio_kind`, `ratio`, `size` — `types.h:218-228`).

---

## 1. Pipeline context — where each pass is invoked

`dotLayout` (`lib/dotgen/dotinit.c:259-304`), in order:

```
setEdgeType(g, EDGETYPE_SPLINE)        dotinit.c:262
setAspect(g)                           dotinit.c:263     <- aspect.c (stub, §7)
dot_init_subg / dot_init_node_edge     dotinit.c:265-266
dot_rank(g)                            dotinit.c:269     (phase 1)
dot_mincross(g)                        dotinit.c:275     (phase 2)
    └─ flat_breakcycles / flat_reorder / checkLabelOrder happen here (§4)
dot_position(g)                        dotinit.c:285     (phase 3)
    └─ dot_concentrate, flat_edges happen here (§3.9, §5.9)
if (NEW_RANK) removeFill(g)            dotinit.c:294-295
dot_sameports(g)                       dotinit.c:296     <- sameport.c (§6)
dot_splines(g)                         dotinit.c:297
if (mapbool(agget(g, "compound"))) dot_compoundEdges(g)  dotinit.c:301-302  <- compound.c (§8)
```

`dot_position` (`lib/dotgen/position.c:121-153`), in order:

```
mark_lowclusters(g)                        position.c:124
set_ycoords(g)                             position.c:125
if (Concentrate) dot_concentrate(g)        position.c:126-131   <- conc.c; error aborts
expand_leaves(g)                           position.c:132
if (flat_edges(g)) set_ycoords(g)          position.c:133-134   <- flat.c; re-run y if labels added
create_aux_edges(g)                        position.c:136-140
rank(g, 2, nsiter2(g)) [network simplex]   position.c:141-146   (LR balance == 2; retry after connectGraph)
set_xcoords(g)                             position.c:147
set_aspect(g)                              position.c:148       (ratio scaling / scale_bb, §7.3)
remove_aux_edges(g)                        position.c:149-151   ("must come after set_aspect since we now
                                                                 use GD_ln and GD_rn for bbox width")
```

- `Concentrate` is a global `bool` (`lib/common/globals.h:62`) set from the graph attribute
  `"concentrate"` via `mapbool` in `graph_init` (`lib/common/input.c:700-701`).
- `flat_edges` is called **only** from `position.c:133`. It is *not* called from mincross;
  mincross uses flat edges through `flat_breakcycles`/`flat_reorder`/`left2right` (§4).
- `dot_sameports` is additionally called **inside spline routing** on an auxiliary graph, from
  `make_flat_adj_edges` (`dotsplines.c:1235`), see §6.5.
- `dot_compoundEdges` runs only when `compound=true`, **after** splines were computed, and
  post-processes `ED_spl` in place (§8).

---

## 2. Shared helper functions used by these files

All in `lib/dotgen/fastgr.c` unless noted.

| Helper | Lines | Semantics |
|---|---|---|
| `find_fast_edge(u,v)` | fastgr.c:43-46 | scan the shorter of `ND_out(u)`/`ND_in(v)` for a fast edge `u→v`. |
| `find_flat_edge(u,v)` | fastgr.c:56-59 | same, over `ND_flat_out(u)`/`ND_flat_in(v)`. |
| `fast_edge(e)` | fastgr.c:71-93 | append `e` to `ND_out(tail)` and `ND_in(head)`. |
| `zapinlist(L,e)` | fastgr.c:96-106 | remove `e` from `L` by replacing it with the **last** element (order not preserved!) and decrementing `size`, then clearing the vacated slot. |
| `delete_fast_edge(e)` | fastgr.c:109-114 | zap from `ND_out(tail)` and `ND_in(head)`. |
| `other_edge(e)` | fastgr.c:116-119 | append `e` to `ND_other(agtail(e))`. |
| `new_virtual_edge(u,v,orig)` | fastgr.c:131-168 | allocate an `Agedgepair_t`; `ED_edge_type = VIRTUAL`; if `orig`: copy `AGSEQ`, `ED_count/xpenalty/weight/minlen`; copy `ED_tail_port` if `tail==tail(orig)` else if `tail==head(orig)` copy `ED_head_port(orig)` (and symmetric for head port); if `ED_to_virt(orig)==NULL` set `ED_to_virt(orig)=e`; `ED_to_orig(e)=orig`. Else (no orig): weight=xpenalty=count=minlen=1. |
| `virtual_edge(u,v,orig)` | fastgr.c:170-173 | `fast_edge(new_virtual_edge(u,v,orig))`. |
| `fast_node(g,n)` | fastgr.c:175-187 | **prepend** `n` to `GD_nlist(g)` (so GD_nlist order is reverse of insertion). |
| `delete_fast_node(g,n)` | fastgr.c:189-198 | unlink from `GD_nlist` (asserts membership via `find_fast_node`). |
| `virtual_node(g)` | fastgr.c:200-213 | fresh node: `ND_node_type = VIRTUAL`, `ND_lw = ND_rw = ND_ht = ND_UF_size = 1`, `alloc_elist(4)` for in/out, prepended to nlist. |
| `flat_edge(g,e)` | fastgr.c:215-220 | append `e` to `ND_flat_out(agtail(e))` and `ND_flat_in(aghead(e))`; `GD_has_flat_edges(dot_root(g)) = GD_has_flat_edges(g) = true`. |
| `delete_flat_edge(e)` | fastgr.c:222-229 | if `ED_to_orig(e) && ED_to_virt(ED_to_orig(e))==e` clear `ED_to_virt(orig)`; zap from `ND_flat_out(tail)`, `ND_flat_in(head)`. |
| `merge_oneway(e,rep)` | fastgr.c:244-254 | warn+return if `rep==ED_to_virt(e) || e==ED_to_virt(rep)`; **assert `ED_to_virt(e)==NULL`**; `ED_to_virt(e)=rep`; `basic_merge(e,rep)`. |
| `basic_merge(e,rep)` | fastgr.c:231-242 | `if (ED_minlen(rep) < ED_minlen(e)) ED_minlen(rep) = ED_minlen(e);` then walk `rep` chain (`rep = ED_to_virt(rep)`) adding `ED_count += ED_count(e)`, `ED_xpenalty += ED_xpenalty(e)`, `ED_weight += ED_weight(e)` to every edge on the chain. |
| `portcmp(p0,p1)` | dotsplines.c:128-142 | `!p1.defined` → `p0.defined ? 1 : 0`; `!p0.defined` → `-1`; else compare `.p.x` then `.p.y` with `<`/`>` returning −1/1/0. |
| `ports_eq(e,f)` | position.c:1030-1039 | `ED_head_port(e).defined == ED_head_port(f).defined` **and** ((`.p.x` and `.p.y` equal) or both ends undefined) **and** the same for tail ports. Not `portcmp`-based; used by `mergeable` (`class2.c:150-153`). |
| `dot_scan_ranks(g)` | rank.c:282-300 | `GD_minrank = INT_MAX; GD_maxrank = -1;` over `agfstnode/agnxtnode`: update max/min rank; `leader` = first node, then any node of strictly smaller rank (so leader = minimum-rank node, first in cgraph node order on ties); `GD_leader(g) = leader`. |
| `save_vlist(g)` | mincross.c:913-920 | if `GD_rankleader` allocated: `GD_rankleader(g)[r] = GD_rank(g)[r].v[0]` for `minrank..maxrank`. |
| `rec_save_vlists(g)` | mincross.c:922-928 | `save_vlist(g)` then recurse over `GD_clust(g)[1..n]`. |
| `rec_reset_vlists(g)` | mincross.c:930-953 | recurse into subclusters first; then for each rank, `u = furthestnode(g, v, -1)`, `w = furthestnode(g, v, 1)` (v = saved rankleader; furthestnode scans left/right neighbors on the **Root** rank keeping the furthest node belonging to cluster `g` — `is_a_normal_node_of` or `is_a_vnode_of_an_edge_of`, mincross.c:883-911); set `GD_rankleader[g][r] = u`; `GD_rank(g)[r].v = GD_rank(dot_root(g))[r].v + ND_order(u)` (aliasing slice!); `GD_rank(g)[r].n = ND_order(w) - ND_order(u) + 1`. |
| `shape_clip(n, curve[4])` | lib/common/splines.c:193-209 | if node has no shape/insidefn, no-op. Determine `left_inside` by testing `curve[0]` (relative to node center) with the shape's inside function; then `shape_clip0` binary-searches (bisection on t over the Bézier `curve`, until consecutive `pt` differs ≤ .5 in x and y) for the boundary crossing; replaces the 4 control points with the segment inside→boundary (`splines.c:110-151`). |
| `Bezier(V,t,Left,Right)` | lib/common/utils.c:175-203 | de Casteljau for degree 3 (`W_DEGREE`); `Left[j] = Vtemp[j][0]`, `Right[j] = Vtemp[3-j][j]`; returns point at `t`. |
| `arrowEndClip(e,ps,startp,endp,spl,eflag)` | lib/common/arrows.c:285-311 | `elen = arrow_length(e,eflag)`; `spl->eflag=eflag; spl->ep = ps[endp+3]`; if `endp>startp && DIST(ps[endp],ps[endp+3]) < elen` then `endp -= 3`; build reversed 4-pt segment ending at `spl->ep`; if `elen>0` `bezier_clip` inward; write clipped points back into `ps[endp..endp+3]`; return (possibly decremented) `endp`. |
| `arrowStartClip(...)` | lib/common/arrows.c:313-339 | mirror image at the start; if segment shorter than `slen`, `startp += 3`; returns (possibly incremented) `startp`. |
| `mkClustMap(g)` / `findCluster(map,name)` | lib/common/utils.c:1560-1597 | `dtopen(&strDisc, Dtoset)` (ordered string dict). `fillMap` recursively: for `c = 1..GD_n_cluster(g)`, key = `agnameof(cluster)`; **duplicate name ⇒ warning "Two clusters named %s - the second will be ignored" and the second is dropped**; then recurse into the cluster. `findCluster` = `dtmatch`, NULL if absent. |

---

## 3. `flat.c` — flat edges, label nodes, adjacent markers

Full file: `flat.c:1-336`. Public entry point: `flat_edges(g)` (`dotprocs.h:48`).

### 3.1 `make_vn_slot(g, r, pos)` — `flat.c:20-39`

Insert a new virtual node into rank `r` at index `pos`, shifting the tail of the rank right.

```
assert(GD_rank(g)[r].av == GD_rank(g)[r].v)
GD_rank(g)[r].av = recalloc(av, old_n_elems = n+1, new_n_elems = n+2, sizeof(node_t*))
GD_rank(g)[r].v  = av                          // av and v are re-unified
for i = n down to pos+1:
    v[i] = v[i-1]
    ND_order(v[i]) += 1                        // shifted nodes bump order
n = v[pos] = virtual_node(g)
ND_order(n) = pos
ND_rank(n)  = r
GD_rank(g)[r].n += 1
v[GD_rank(g)[r].n] = NULL                      // re-terminate
return v[pos]
```

Notes:
- The recalloc keeps old contents (first `n+1` pointers) and adds one zero slot; the loop
  moves `v[pos..n-1]` to `v[pos+1..n]`.
- After return, the node has the `virtual_node` defaults (`lw=rw=ht=1`, type `VIRTUAL`) and is
  already in `GD_nlist` (prepended).
- Callers: `flat_node` only (with `r = ND_rank(tail of e) - 1`).

### 3.2 Bound constants and `findlr` — `flat.c:41-56`

```c
#define HLB 0   /* hard left bound  */
#define HRB 1   /* hard right bound */
#define SLB 2   /* soft left bound  */
#define SRB 3   /* soft right bound */
```

`findlr(u,v,&l,&r)`: `l = order(u); r = order(v); if (l > r) SWAP(&l,&r);` — i.e. (l, r) is the
ordered pair of the two nodes' orders, `l ≤ r`.

### 3.3 `setbounds(v, bounds[4], lpos, rpos)` — `flat.c:58-102`

Given node `v` on the *previous rank* (rank `r-1`) and the target interval `[lpos, rpos]`
(orders of the flat edge's endpoints, left first), tighten `bounds`:

- Only `VIRTUAL` nodes do anything (`flat.c:63`).
- **Case "flat"** (`ND_in(v).size == 0`, `flat.c:65-82`): these are label nodes of other flat
  edges; `assert(ND_out(v).size == 2)`. Their two out-edges are the `FLATORDER` edges to the
  flat edge's endpoints. Let `l, r` = `findlr(aghead(ND_out(v).list[0]), aghead(ND_out(v).list[1]))`
  (orders of that other flat edge's two endpoints).
  - `r <= lpos` → the other flat edge lies wholly left of us: `bounds[SLB] = bounds[HLB] = ord`.
  - `l >= rpos` → wholly right: `bounds[SRB] = bounds[HRB] = ord`.
  - `l < lpos && r > rpos` → the other edge spans our interval: ignore (no assignment).
  - otherwise (intersecting ranges): `if (l < lpos || (l == lpos && r < rpos)) bounds[SLB] = ord;`
    and `if (r > rpos || (r == rpos && l > lpos)) bounds[SRB] = ord;`
    (Note: the equality tie-breakers make the *equal-left* case a soft left bound only, and
    *equal-right* a soft right bound only.)
- **Case "forward"** (`ND_in(v).size != 0`, `flat.c:83-100`): `v` is a virtual node in a long
  edge chain. Scan `ND_out(v)`: `onleft` if some `order(aghead(f)) <= lpos`; `onright` if some
  `order(aghead(f)) >= rpos`.
  - `onleft && !onright` → `bounds[HLB] = ord + 1`
  - `onright && !onleft` → `bounds[HRB] = ord - 1`
  (both, or neither → no change).
- `NORMAL` nodes: ignored entirely.

### 3.4 `flat_limits(g, e)` — `flat.c:104-129`

Compute the insertion index (order) for the label node of flat edge `e`, looking at rank
`r-1` where `r = ND_rank(agtail(e))`:

```
r = rank(agtail(e)) - 1
rank = GD_rank(g)[r].v
lnode = 0; rnode = GD_rank(g)[r].n - 1
bounds[HLB] = bounds[SLB] = lnode - 1        // -1
bounds[HRB] = bounds[SRB] = rnode + 1        // n
findlr(agtail(e), aghead(e), &lpos, &rpos)   // orders of e's endpoints, lpos ≤ rpos
while (lnode <= rnode):
    setbounds(rank[lnode], bounds, lpos, rpos)
    if lnode != rnode: setbounds(rank[rnode], bounds, lpos, rpos)
    lnode++; rnode--
    if (bounds[HRB] - bounds[HLB] <= 1) break     // hard interval already minimal
if (bounds[HLB] <= bounds[HRB]):
    pos = (bounds[HLB] + bounds[HRB] + 1) / 2     // integer division
else:
    pos = (bounds[SLB] + bounds[SRB] + 1) / 2
return pos
```

- Scan order: **inward from both ends simultaneously** (left node then right node per
  iteration), so bounds from the extremes are seen first; the loop short-circuits as soon as
  the hard bounds are adjacent.
- There are no `lpos/rpos` arrays, no `ND_mval`, no medians in this version — a single
  midpoint of the first feasible hard (else soft) interval.

### 3.5 `flat_node(e)` — `flat.c:136-182`

Create the virtual label node for a **labeled, non-adjacent** flat edge `e`.

```
if (ED_label(e) == NULL) return
g = dot_root(agtail(e)); r = ND_rank(agtail(e))
place = flat_limits(g, e)
// ypos = LL.y (bottom) of the label box, grabbed BEFORE make_vn_slot:
if ((n = GD_rank(g)[r-1].v[0]))                    // rank r-1 nonempty
    ypos = ND_coord(n).y - GD_rank(g)[r-1].ht1     // bottom edge of rank r-1
else
    n = GD_rank(g)[r].v[0];
    ypos = ND_coord(n).y + GD_rank(g)[r].ht2 + GD_ranksep(g)
vn = make_vn_slot(g, r-1, place)                   // label node lives on rank r-1
dimen = ED_label(e)->dimen
if (GD_flip(g)) SWAP(&dimen.x, &dimen.y)
ND_ht(vn)     = dimen.y
h2            = ND_ht(vn) / 2
ND_lw(vn) = ND_rw(vn) = dimen.x / 2
ND_label(vn) = ED_label(e)                         // SHARED pointer with the edge
ND_coord(vn).y = ypos + h2                         // vertical center; .x set later by x-coordination
ve = virtual_edge(vn, agtail(e), e)                // FLATORDER edge to tail
ED_tail_port(ve).p.x = -ND_lw(vn)                  // left edge of label box
ED_head_port(ve).p.x =  ND_rw(agtail(e))           // right side of tail node
ED_edge_type(ve) = FLATORDER
ve = virtual_edge(vn, aghead(e), e)                // FLATORDER edge to head
ED_tail_port(ve).p.x =  ND_rw(vn)                  // right edge of label box
ED_head_port(ve).p.x =  ND_lw(aghead(e))           // left side of head node
ED_edge_type(ve) = FLATORDER
/* another assumed symmetry of ht1/ht2 of a label node */
if (GD_rank(g)[r-1].ht1 < h2) GD_rank(g)[r-1].ht1 = h2
if (GD_rank(g)[r-1].ht2 < h2) GD_rank(g)[r-1].ht2 = h2
ND_alg(vn) = e                                     // marks vn as a flat-edge label node
```

Identifying invariant (comment `flat.c:131-135`): a label node is *virtual* **and** has
non-NULL `ND_alg`. Consumers: `mincross.c:304` (checkLabelOrder), `position.c:277`
(make_LR_constraints), `dotsplines.c:205,293` (label positioning).

Note the rank `r-1` height bump (`flat.c:177-180`) is unconditional on which ranks the
endpoints sit — the label node claims `h2` above and below the rank-`r-1` centerline.

### 3.6 `abomination(g)` — `flat.c:184-202`

Insert one new **empty** rank below rank 0 (used when a labeled non-adjacent flat edge sits on
rank 0, so its label node can go on a new rank −1).

```
assert(GD_minrank(g) == 0)
// 3 = one for new rank, one for sentinel, one for off-by-one   (flat.c:189)
r = GD_maxrank(g) + 3
rptr = gv_recalloc(GD_rank(g), GD_maxrank(g) + 1, r, sizeof(rank_t))
GD_rank(g) = rptr + 1                    // rebased: valid indices now -1 .. maxrank+1
for (r = GD_maxrank(g); r >= 0; r--)
    GD_rank(g)[r] = GD_rank(g)[r - 1]    // struct copy: shift all ranks up one index
// loop exits with r == -1: initialize the new bottom rank in place
GD_rank(g)[-1].n = GD_rank(g)[-1].an = 0
GD_rank(g)[-1].v = GD_rank(g)[-1].av = gv_calloc(2, sizeof(node_t *))
GD_rank(g)[-1].flat = NULL
GD_rank(g)[-1].ht1 = GD_rank(g)[-1].ht2 = 1
GD_rank(g)[-1].pht1 = GD_rank(g)[-1].pht2 = 1
GD_minrank(g)--
```

The old rank-0 `rank_t` (now at `rptr[1]` after rebase) has already been copied into index 0
by the shift; its original storage `rptr[0]` is then re-initialized as the empty new rank.
Net effect: ranks shift up by one index, new rank at index −1, `GD_minrank` becomes −1.

### 3.7 `checkFlatAdjacent(e)` — `flat.c:208-238`

Mark `ED_adjacent` on `e` and its whole `ED_to_virt` chain if `e`'s endpoints have no node
between them on the rank other than flat-edge label nodes:

```
tn = agtail(e); hn = aghead(e)
lo = min(order(tn), order(hn)); hi = max(order(tn), order(hn))
rank = &GD_rank(dot_root(tn))[ND_rank(tn)]
for i = lo+1; i < hi; i++:
    n = rank->v[i]
    if ((ND_node_type(n) == VIRTUAL && ND_label(n)) || ND_node_type(n) == NORMAL)
        break
if (i == hi):                              // nothing between ⇒ adjacent endpoints
    do { ED_adjacent(e) = 1; e = ED_to_virt(e); } while (e)
```

- Nodes that stop the scan: any real node, or any virtual node carrying a label (i.e. the
  label node of another flat edge). Plain virtual chain nodes and plain label-free nodes do
  **not** break adjacency (they shouldn't occur on a rank interior in dot, but the code only
  excludes labeled virtuals).
- The `do/while` marks `e`, then the representative, then everything `ED_to_virt` reaches
  (usually just `e` → rep).

### 3.8 `flat_edges(g)` — `flat.c:256-336` (returns `int`, 0/1)

Three phases. Iteration over `GD_nlist(g)` is the fast-node list order = **reverse of
`fast_node` insertion order**.

**Phase 1 — adjacency marking** (`flat.c:261-272`)

```
for n in GD_nlist(g):
    if (ND_flat_out(n).list):
        for j = 0; (e = ND_flat_out(n).list[j]); j++:
            checkFlatAdjacent(e)
    for j = 0; j < ND_other(n).size; j++:
        e = ND_other(n).list[j]
        if (ND_rank(aghead(e)) == ND_rank(agtail(e)))
            checkFlatAdjacent(e)
```
(Reminder: for flat edges, one of the pair is in `flat_out` of the tail; `ND_other` holds
equivalent/parallel flat edges and self-edges, keyed on the tail node.)

**Phase 2 — possible rank insertion (`abomination`)** (`flat.c:274-292`)

```
if (GD_rank(g)[0].flat || GD_n_cluster(g) > 0):        // rank-0 flat matrix exists, or clusters
    found = false
    for i = 0; !found && (n = GD_rank(g)[0].v[i]); i++:
        for j = 0; !found && (e = ND_flat_in(n).list[j]); j++:
            if (ED_label(e) && !ED_adjacent(e)): abomination(g); found = true
        for j = 0; !found && j < ND_other(n).size; j++:
            e = ND_other(n).list[j]
            if (ED_label(e) && !ED_adjacent(e)): abomination(g); found = true
```
- Gate: `GD_rank(g)[0].flat` is the per-rank flat-edge adjacency matrix created by
  `flat_breakcycles` (§4.2) — non-NULL iff rank 0 had flat edges during mincross.
- Only rank 0 is examined; the scan stops at the first hit. `GD_rank(g)[0].v` is walked with
  the NULL-terminator idiom.
- If triggered, the new rank −1 gives labeled non-adjacent flat edges on rank 0 somewhere to
  put their label nodes (see `flat_node`, which always inserts into rank `r-1`).

**Save/restore of rank vlists** (`flat.c:294`): `rec_save_vlists(g)` before mutating ranks.

**Phase 3 — create label nodes / set `ED_dist`** (`flat.c:296-330`)

```
reset = false
for n in GD_nlist(g):
    // if n is the tail of any flat edge, one will be in flat_out
    if (ND_flat_out(n).list):
        for i = 0; (e = ND_flat_out(n).list[i]); i++:
            if (ED_label(e)):
                if (ED_adjacent(e)):
                    ED_dist(e) = GD_flip(g) ? ED_label(e)->dimen.y : ED_label(e)->dimen.x
                else:
                    reset = true
                    flat_node(e)
        // look for other flat edges with labels (equivalent edges in ND_other)
        for j = 0; j < ND_other(n).size; j++:
            e = ND_other(n).list[j]
            if (ND_rank(agtail(e)) != ND_rank(aghead(e))) continue     // only truly flat
            if (agtail(e) == aghead(e)) continue                        // skip self loops
            le = e; while (ED_to_virt(le)) le = ED_to_virt(le)          // representative
            ED_adjacent(e) = ED_adjacent(le)
            if (ED_label(e)):
                if (ED_adjacent(e)):
                    lw = GD_flip(g) ? ED_label(e)->dimen.y : ED_label(e)->dimen.x
                    ED_dist(le) = MAX(lw, ED_dist(le))                  // widest label wins
                else:
                    reset = true
                    flat_node(e)                                        // one label node per edge
if (reset):
    checkLabelOrder(g)          // mincross.c:297-326, fix ordering of label nodes (§4.4)
    rec_reset_vlists(g)         // mincross.c:930-953, re-anchor cluster rank vlists
return reset                    // 1 ⇒ caller must redo set_ycoords (position.c:133-134)
```

Downstream of `ED_dist` (adjacent flat edges with labels): `make_LR_constraints`
`m0 = MAX(m0, width + GD_nodesep(g) + ROUND(ED_dist(e)))` (`position.c:316`).

### 3.9 Pipeline roles of flat edges (cross-reference)

1. **Creation.** `class2` (`class2.c:242-247`): flat input edges → `flat_edge(g,e)`.
   Parallel flat edges with identical endpoints are merged into the first
   (`merge_oneway(e,prev)`) and the duplicates are appended to `ND_other(agtail(e))`
   (`class2.c:206-211`); self edges also go to `ND_other` (`class2.c:226-230`).
   `GD_has_flat_edges` is set by `flat_edge` (`fastgr.c:219`).
2. **mincross, cycle breaking** — `flat_breakcycles` (§4.2), called at `mincross.c:544`
   (once per cluster expansion in `mincross_clust`) and `mincross.c:710` (pass 0 of
   `mincross`).
3. **mincross, ordering** — `flat_reorder` (§4.3) at `mincross.c:545,711`; the resulting
   per-rank left/right relation matrix is consulted by `left2right` (`mincross.c:563-585`)
   from `transpose_step` (`mincross.c:645`) and `reorder` (`mincross.c:1423`).
   `constraining_flat_edge` (`mincross.c:1297-1305`): a flat edge constrains iff
   `ED_weight(e) != 0` and both endpoints are "inside" the cluster being laid out
   (`inside_cluster` = normal node of the cluster, or vnode of an edge of the cluster,
   `mincross.c:883-900`).
   `do_ordering_node` (`mincross.c:440-477`, invoked from `ordered_edges`, `mincross.c:512+`,
   for nodes with the `ordering` attribute) also creates additional
   `FLATORDER`-type virtual edges between nodes forced by `ordering=out/in` attributes:
   after qsorting the node's incident (non-between-cluster) edges by `edgeidcmpf` (`AGSEQ`
   order, `mincross.c:461`), for each consecutive pair `u, v`: if `find_flat_edge(u, v)`
   is NULL, `fe = new_virtual_edge(u, v, NULL); ED_edge_type(fe) = FLATORDER; flat_edge(g, fe);`
4. **label ordering** — `checkLabelOrder` + `fixLabelOrder` (§4.4), invoked from
   `flat_edges` only when a label node was created (`flat.c:332`).
5. **x-coordination** (`dot_position`): after `flat_edges` (which may add label nodes and
   rerun `set_ycoords`), `create_aux_edges` → `make_LR_constraints` (`position.c:224-336`)
   adds the network-simplex constraints:
   - *chain spacing*: for consecutive nodes `u, v` on a rank,
     `make_aux_edge(u, v, width = ND_rw(u) + ND_lw(v) + nodesep, 0)` (`position.c:266-274`).
     Before that, each node stashes `ND_mval(u) = ND_rw(u)` ("keep it somewhere safe",
     `position.c:248`) and grows `ND_rw` by self-edge space (`position.c:249-265`).
   - *flat label constraint* (`position.c:276-296`): if `ND_alg(u)` (u is a label node of a
     labeled non-adjacent flat edge `e`): take `e0, e1` = the two `FLATORDER` edges in
     `ND_save_out(u)` (saved before aux-edge allocation, `position.c:212-214`); swap so
     `order(aghead(e0)) <= order(aghead(e1))`; `m0 = ED_minlen(e) * GD_nodesep(g) / 2`
     (integer arithmetic on ints); `m1 = m0 + ND_rw(aghead(e0)) + ND_lw(agtail(e0))`;
     **unless** `canreach(agtail(e0), aghead(e0))` (guards "the flat edges work very poorly
     with cluster layout", `position.c:285-286`), add
     `make_aux_edge(aghead(e0), agtail(e0), m1, ED_weight(e))` — i.e. left endpoint node
     forced left of the label node; symmetric for the right side:
     `m1 = m0 + ND_rw(agtail(e1)) + ND_lw(aghead(e1))`,
     `make_aux_edge(agtail(e1), aghead(e1), m1, ED_weight(e))` unless
     `canreach(aghead(e1), agtail(e1))`.
     (`canreach(u,v)` = DFS `go` over `ND_out` chains, `position.c:165-180`.)
   - *flat endpoints* (`position.c:298-332`): for each `e` in `ND_flat_out(u)`, let
     `t0, h0` = (tail, head) ordered left-to-right by `ND_order`;
     `width = ND_rw(t0) + ND_lw(h0)`; `m0 = ED_minlen(e) * GD_nodesep(g) + width`;
     - if `find_fast_edge(t0, h0)` exists (adjacent pair has an actual fast edge):
       `m0 = MAX(m0, width + GD_nodesep(g) + ROUND(ED_dist(e)))`;
       `ED_minlen(e0) = MAX(ED_minlen(e0), m0)`;
       `ED_weight(e0) = MAX(ED_weight(e0), ED_weight(e))` — strengthen the fast edge.
     - else if `!ED_label(e)` (unlabeled non-neighbors):
       `make_aux_edge(t0, h0, m0, ED_weight(e))`.
     - (labeled non-neighbors are already constrained by the label node above.)
   The simplex (`rank(g, 2, nsiter2(g))`) then fills `ND_rank(v)` with **x positions**;
   `set_xcoords` copies `ND_coord(v).x = ND_rank(v)` and restores `ND_rank(v) = i`
   (`position.c:598-610`). So flat edges influence x purely through the aux-edge minlens
   above.
6. **y-coordination**: label nodes on rank `r-1` participate in `set_ycoords` like any node
   (`ND_ht`), and `flat_node` already bumped `GD_rank[r-1].ht1/ht2` to at least `h2`.
   Rank separation uses `d1 = rank[r+1].ht2 + rank[r].ht1 + CL_OFFSET` for cluster spacing
   (`position.c:806`).
7. **splines** (`dotsplines.c`): `FLATORDER` edges are skipped when collecting regular edges
   (`dotsplines.c:302`); label node positions are copied to the edge labels
   (`ED_label(fe)->pos = ND_coord(n); ED_label(fe)->set = true`, `dotsplines.c:293-298`);
   flat edges are grouped per (tail,head) with `ED_adjacent` short-circuiting multi-edge
   grouping ("all flat adjacent edges at once", `dotsplines.c:366-367`) and dispatched to
   `make_flat_edge` (`dotsplines.c:417-424`) which routes adjacent cases through a recursive
   aux-graph layout `make_flat_adj_edges` (`dotsplines.c:1129-1270`).

### 3.10 `flat.c` pseudocode summary (whole file)

```
flat_edges(g):
    mark adjacency (phase 1)
    maybe insert rank -1 (phase 2)
    save vlists
    for every labeled flat edge (representatives and ND_other equivalents, excluding loops):
        adjacent  ⇒ ED_dist = label width (max over equivalents, on representative)
        otherwise ⇒ create label node on rank above (flat_node)
    if anything created: fix label order; reset vlists; return 1 else 0
```

---

## 4. Flat-edge machinery now living in `mincross.c`

(Not in the five files, but required to port `flat.c`'s ecosystem and directly requested by
the brief — `flat_breakcycles`, `flat_search`, tie-breakers.)

### 4.1 Support definitions

- `MARK(v)` = `ND_mark(v)` (`mincross.c:113`); `flatindex(v) = (size_t)ND_low(v)`
  (`mincross.c:115`).
- `ND_low(v)` shares storage with `ND_hops` (`rank.c:555`), initialized to the node's rank
  index `i` in `flat_breakcycles` (below).
- `new_matrix(rows, cols)` — bit matrix (`mincross.c:405+`); `matrix_get/set` grow the backing
  store on out-of-range access (`mincross.c:55-110`).

### 4.2 `flat_search(g, v)` — `mincross.c:1059-1090` and `flat_breakcycles(g)` — `mincross.c:1092-1117`

```
flat_breakcycles(g):
    for r = GD_minrank(g) .. GD_maxrank(g):
        flat = false
        for i = 0 .. GD_rank(g)[r].n - 1:
            v = GD_rank(g)[r].v[i]
            ND_mark(v) = false; ND_onstack(v) = false
            ND_low(v) = i                       // flatindex = position within the rank
            if (ND_flat_out(v).size > 0 && !flat):
                GD_rank(g)[r].flat = new_matrix(GD_rank(g)[r].n, GD_rank(g)[r].n)
                flat = true
        if (flat):
            for i = 0 .. n-1:
                v = GD_rank(g)[r].v[i]
                if (!ND_mark(v)) flat_search(g, v)

flat_search(g, v):                               // DFS; same as reverse topological order
    ND_mark(v) = true; ND_onstack(v) = true
    hascl = GD_n_cluster(dot_root(g)) > 0
    for i = 0; (e = ND_flat_out(v).list[i]); i++:
        if (hascl && !(agcontains(g, agtail(e)) && agcontains(g, aghead(e)))) continue
        if (ED_weight(e) == 0) continue          // non-constraining edge
        if (ND_onstack(aghead(e))):              // back edge ⇒ cycle
            matrix_set(M, flatindex(aghead(e)), flatindex(agtail(e)))   // record REVERSED pair
            delete_flat_edge(e)
            i--                                  // compensate for list compaction by zapinlist
            if (ED_edge_type(e) == FLATORDER) continue
            flat_rev(g, e)
        else:
            matrix_set(M, flatindex(agtail(e)), flatindex(aghead(e)))
            if (!ND_mark(aghead(e))) flat_search(g, aghead(e))
    ND_onstack(v) = false
```

- `M = GD_rank(g)[ND_rank(v)].flat` (`mincross.c:1063`).
- Nodes included: **all** nodes of the rank (normal nodes, virtual chain nodes, label nodes —
  anything in the rank vlist); the DFS traverses `ND_flat_out` only, so only rank-local
  structure matters. Tie-break order: DFS roots are visited in rank vlist order
  (`i = 0..n-1`), out-edges in elist order.
- The matrix records the forward pair `(tail_idx, head_idx)` for kept edges, and the
  **swapped** pair `(head_idx, tail_idx)` for the reversed ones, so `left2right` can later
  answer "must v be left of w" for any pair.
- `flat_rev(g, e)` (`mincross.c:1033-1057`): search `ND_flat_out(aghead(e))` for `rev` with
  `aghead(rev) == agtail(e)`; if found: `merge_oneway(e, rev)`, and if
  `ED_edge_type(rev)==FLATORDER && ED_to_orig(rev)==0` then `ED_to_orig(rev)=e`;
  `elist_append(e, ND_other(agtail(e)))`. Else create
  `rev = new_virtual_edge(aghead(e), agtail(e), e)` with
  `ED_edge_type(rev) = FLATORDER if ED_edge_type(e)==FLATORDER else REVERSED`,
  `ED_label(rev) = ED_label(e)`, and `flat_edge(g, rev)`.

### 4.3 `flat_reorder(g)` — `mincross.c:1327-1402`

```
if (!GD_has_flat_edges(g)) return
for r = minrank .. maxrank:
    if (GD_rank(g)[r].n == 0) continue
    base_order = ND_order(GD_rank(g)[r].v[0])
    clear MARK on all nodes of the rank; temprank = []
    // construct reverse topological sort order in temprank
    valid_rank = true
    for i = 0 .. n-1:
        v = GD_flip(g) ? rank.v[i] : rank.v[n - i - 1]     // scan direction depends on flip
        count local_in/local_out = # of constraining flat in/out edges
        if (both 0) temprank += v
        else if (!MARK(v) && local_in == 0) postorder(g, v, &temprank, r)
        else { valid_rank = false; break }                  // "else do no harm!"
    if (valid_rank && !temprank.empty()):
        if (!GD_flip(g)) reverse(temprank)
        for i = 0 .. n-1:
            rank.v[i] = temprank[i]; ND_order(v) = i + base_order
        // nonconstraint flat edges must be made LR
        for i = 0 .. n-1:
            v = rank.v[i]
            for e in ND_flat_out(v):
                if ((!flip && order(aghead(e)) < order(agtail(e))) ||
                    ( flip && order(aghead(e)) > order(agtail(e)))):
                    assert(!constraining_flat_edge(g, e))
                    delete_flat_edge(e); j--; flat_rev(g, e)
    GD_rank(Root)[r].valid = false
```

`postorder(g,v,list,r)` (`mincross.c:1310-1325`): recursive, appends after visiting
constraining flat-out successors ("the same as doing a topological sort in reverse order");
asserts `ND_rank(v) == r`.

### 4.4 `checkLabelOrder(g)` — `mincross.c:297-326` and `fixLabelOrder` — `mincross.c:244-287`

Called from `flat_edges` after label nodes were created. For **each rank independently**:

```
lg = NULL
for j = 0 .. rank[r].n - 1:
    u = rank[r].v[j]
    if (ND_alg(u)):                              // u is a flat-edge label node
        if (!lg) lg = agopen("lg", Agstrictdirected, 0)
        n = agnode(lg, ITOS(j), 1)               // node named by its rank index
        lo = ND_order(aghead(ND_out(u).list[0])); hi = ND_order(aghead(ND_out(u).list[1]))
        if (lo > hi) SWAP(&lo,&hi)
        ND_lo(n) = lo; ND_hi(n) = hi; ND_np(n) = u
if (lg):
    if (agnnodes(lg) > 1) fixLabelOrder(lg, GD_rank(g) + r)
    agclose(lg)
```

`fixLabelOrder(g, rk)` — for each pair of label nodes `n, v` in `agfstnode` order with outer
loop `n`, inner `v` = successor of `n`:

```
if (ND_hi(v) <= ND_lo(n)): haveBackedge = true; agedge(g, v, n, NULL, 1)   // v must precede n
else if (ND_hi(n) <= ND_lo(v)): agedge(g, n, v, NULL, 1)                   // n precedes v
if (!haveBackedge) return                                                  // already consistent
// otherwise: strongly connect each component, topsort it, and rewrite the rank:
sg = agsubg(g, "comp", 1); indices = calloc(agnnodes_z(g))
for n in nodes:
    if (ND_x(n) || agdegree(g,n,1,1) == 0) continue
    if (getComp(g, n, sg, indices)):            // DFS collecting component + backedge count
        arr = topsort(g, sg)
        qsort(indices, arr.size, sizeof(int), ordercmpf)    // ascending order values
        for i in arr: ND_order(i-th node) = indices[i]; rk->v[indices[i]] = that node
    emptyComp(sg)
```

(`ND_x/lo/hi/np` are fields of a local `info_t` bound to `lg`'s nodes, `mincross.c:175-179`;
`ordercmpf` compares `ND_order` of the two nodes, `mincross.c:1562+`.) Net effect: label
nodes whose `[lo,hi]` intervals force a contradictory order get their orders (and slots in
the rank vlist) permuted to a consistent topological order.

---

## 5. `conc.c` — edge concentration (`concentrate=true`)

File: `conc.c:1-248`. Public entry: `dot_concentrate(g)` (`dotprocs.h:78`, WUR).

```c
#define UP   0
#define DOWN 1
```

### 5.1 `samedir(e, f)` — `conc.c:24-40`

```
e0 = e; while (e0 && ED_edge_type(e0) != NORMAL) e0 = ED_to_orig(e0); if (!e0) return false
f0 = f; ... same ...                                                                   :31-33
if (ED_conc_opp_flag(e0)) return false                                                 :34-35
if (ED_conc_opp_flag(f0)) return false                                                 :36-37
return (rank(agtail(f0)) - rank(aghead(f0))) * (rank(agtail(e0)) - rank(aghead(e0))) > 0
```

i.e. both originals run the same vertical direction (product positive), and neither was
marked as the "opposite" edge of a concentrated backward edge. That flag is set in `class2`
when `Concentrate` is on and a backward edge shadows a forward one:
`ED_edge_type(e) = IGNORED; ED_conc_opp_flag(opp) = true;` (`class2.c:269-271`; the parallel
non-concentrate path is `other_edge(e); merge_chain(...)` `class2.c:272-275`, and multi-edge
IGNORED marking at `class2.c:214-215`).

### 5.2 Candidate predicates — `conc.c:42-76`

```
downcandidate(v):        ND_node_type(v)==VIRTUAL && ND_in(v).size==1 && ND_out(v).size==1 && ND_label(v)==NULL
upcandidate(v):          ND_node_type(v)==VIRTUAL && ND_out(v).size==1 && ND_in(v).size==1 && ND_label(v)==NULL
                         // identical conditions, written in swapped order
bothdowncandidates(u,v): e = ND_in(u).list[0]; f = ND_in(v).list[0]
                         return downcandidate(v) && agtail(e)==agtail(f)
                                && samedir(e,f) && portcmp(ED_tail_port(e), ED_tail_port(f))==0
bothupcandidates(u,v):   e = ND_out(u).list[0]; f = ND_out(v).list[0]
                         return upcandidate(v) && aghead(e)==aghead(f)
                                && samedir(e,f) && portcmp(ED_head_port(e), ED_head_port(f))==0
```

(`u` is assumed to have passed the single-node candidate test already; only `v` is retested.)

### 5.3 `mergevirtual(g, r, lpos, rpos, dir)` — `conc.c:78-126`

Merge virtual nodes `v[lpos+1..rpos]` of rank `r` into the **leftmost** one, then compact.

```
left = GD_rank(g)[r].v[lpos]
for i = lpos+1 .. rpos:
    right = GD_rank(g)[r].v[i]
    if (dir == DOWN):
        while ((e = ND_out(right).list[0])):                    // right's out edges
            for k = 0; (f = ND_out(left).list[k]); k++:
                if (aghead(f) == aghead(e)) break               // existing edge to same head
            if (f == NULL) f = virtual_edge(left, aghead(e), e) // clone from right's edge
            while ((e0 = ND_in(right).list[0])):                // right's in edges
                merge_oneway(e0, f)                             // funnel data into f
                delete_fast_edge(e0)
            delete_fast_edge(e)
    else:  // UP: mirror image
        while ((e = ND_in(right).list[0])):
            for k = 0; (f = ND_in(left).list[k]); k++:
                if (agtail(f) == agtail(e)) break
            if (f == NULL) f = virtual_edge(agtail(e), left, e)
            while ((e0 = ND_out(right).list[0])):
                merge_oneway(e0, f)
                delete_fast_edge(e0)
            delete_fast_edge(e)
    assert(ND_in(right).size + ND_out(right).size == 0)
    delete_fast_node(g, right)
// compact rank array
k = lpos + 1
for i = rpos+1 .. GD_rank(g)[r].n - 1:
    GD_rank(g)[r].v[k] = GD_rank(g)[r].v[i]; ND_order(v[k]) = k; k++
GD_rank(g)[r].n = k
GD_rank(g)[r].v[k] = NULL
```

Mechanics of the DOWN case: for each out edge `right→h`, ensure `left` has an edge to `h`
(created via `virtual_edge` from `e`, inheriting `e`'s head port and `AGSEQ`), then every in
edge `t→right` is merged into that `left→h` edge (`merge_oneway` sets `ED_to_virt(e0)=f`,
adds count/xpenalty/weight, maxes minlen, along `f`'s to_virt chain) and deleted; finally
`right→h` is deleted. After processing, `right` has degree 0 (asserted) and is removed.
**Port and label data survive only via `virtual_edge`'s orig-copy and `merge_oneway`.**

### 5.4 `infuse(g, n)` — `conc.c:128-135`

```
lead = GD_rankleader(g)[ND_rank(n)]
if (lead == NULL || ND_order(lead) > ND_order(n))
    GD_rankleader(g)[ND_rank(n)] = n          // keep leftmost-by-order node per rank
```

### 5.5 `rebuild_vlists(g)` — `conc.c:137-200` (returns 0 / −1)

Re-anchor the rank vlists of cluster `g` (and recursively its subclusters) after
concentration deleted nodes from the root's rank arrays.

```
for r = minrank..maxrank: GD_rankleader(g)[r] = NULL                 :143-144
dot_scan_ranks(g)                                                    :145  (recompute minrank/maxrank/leader)
for n in agfstnode(g) ..:                                            :146-155
    infuse(g, n)
    for e in agfstout(g,n) ..:
        rep = e; while (ED_to_virt(rep)) rep = ED_to_virt(rep)       // walk to chain end
        while (rep && ND_rank(aghead(rep)) < ND_rank(aghead(e))):    // climb down the chain
            infuse(g, aghead(rep))
            rep = ND_out(aghead(rep)).list[0]
for r = minrank..maxrank:                                            :157-191
    lead = GD_rankleader(g)[r]
    if (!lead): agerrorf("rebuild_vlists: lead is null for rank %d\n", r); return -1
    if (GD_rank(dot_root(g))[r].v[ND_order(lead)] != lead):
        agerrorf("rebuild_vlists: rank lead %s not in order %d of rank %d\n",
                 agnameof(lead), ND_order(lead), r); return -1
    GD_rank(g)[r].v = GD_rank(dot_root(g))[r].v + ND_order(lead)     // ALIASING slice of root vlist
    maxi = -1
    for i = 0 .. GD_rank(g)[r].n - 1:                                // n = pre-compaction count
        n = GD_rank(g)[r].v[i]
        if (!n) break                                                // NULL sentinel
        if (ND_node_type(n) == NORMAL):
            if (agcontains(g, n)) maxi = i; else break
        else:
            e = ND_in(n).list[0]; while (e && ED_to_orig(e)) e = ED_to_orig(e)
            if (e && agcontains(g, agtail(e)) && agcontains(g, aghead(e))) maxi = i
    if (maxi == -1) agwarningf("degenerate concentrated rank %s,%d\n", agnameof(g), r)
    GD_rank(g)[r].n = maxi + 1
for c = 1 .. GD_n_cluster(g):                                        :193-198
    ret = rebuild_vlists(GD_clust(g)[c]); if (ret) return ret
return 0
```

### 5.6 `dot_concentrate(g)` — `conc.c:202-248` (returns 0 / −1)

```
if (GD_maxrank(g) - GD_minrank(g) <= 1) return 0                     :206-207
// downward looking pass; r is a candidate rank                        :208
for (r = 1; GD_rank(g)[r + 1].n; r++):                               :209
    for leftpos = 0 .. GD_rank(g)[r].n - 1:
        left = GD_rank(g)[r].v[leftpos]
        if (!downcandidate(left)) continue
        for rightpos = leftpos+1 .. GD_rank(g)[r].n - 1:
            right = GD_rank(g)[r].v[rightpos]
            if (!bothdowncandidates(left, right)) break
        if (rightpos - leftpos > 1)
            mergevirtual(g, r, leftpos, rightpos - 1, DOWN)
// corresponding upward pass                                          :224
while (r > 0):                                                       :225-240
    for leftpos = 0 .. GD_rank(g)[r].n - 1:
        left = GD_rank(g)[r].v[leftpos]
        if (!upcandidate(left)) continue
        for rightpos = leftpos+1 .. GD_rank(g)[r].n - 1:
            right = GD_rank(g)[r].v[rightpos]
            if (!bothupcandidates(left, right)) break
        if (rightpos - leftpos > 1)
            mergevirtual(g, r, leftpos, rightpos - 1, UP)
    r--
for c = 1 .. GD_n_cluster(g):                                        :241-246
    if (rebuild_vlists(GD_clust(g)[c]) != 0):
        agerr(AGPREV, "concentrate=true may not work correctly.\n")
        return -1
return 0
```

Exact behavioral notes a port must reproduce:

- **`r` is shared between the two passes** (declared once, `conc.c:203`). The downward loop
  guard `GD_rank(g)[r + 1].n` reads one rank past `maxrank`; that slot is allocated and zero
  because `GD_rank(g) = gv_calloc(GD_maxrank(g)+2, sizeof(rank_t))` (`mincross.c:1141`).
  For ranks `0..N` nonempty the downward loop processes `r = 1..N-1` and exits with `r = N`;
  the upward loop then processes `r = N..1` — **revisiting ranks 1..N-1** that the downward
  pass already merged (merged single virtual nodes remain candidates).
- The inner `rightpos` scan stops at the first non-candidate; the run merged is
  `[leftpos, rightpos-1]`, i.e. a maximal run of candidates starting at `leftpos` (length
  ≥ 2 ⇒ ≥ 1 node to delete ⇒ merge). `leftpos` continues from the outer loop with the
  post-merge rank array.
- Merge direction: DOWN merges virtual nodes whose single out-edge goes to the same head
  with the same tail and same direction and equal tail ports; UP merges on same head/head
  ports.
- `dot_concentrate` is called **between** `set_ycoords` and `expand_leaves` in
  `dot_position` (`position.c:126-131`), so all later phases operate on the compacted graph.
- Error paths: −1 with `agerrorf` from `rebuild_vlists`, augmented by
  `agerr(AGPREV, "concentrate=true may not work correctly.\n")`.

---

## 6. `sameport.c` — `samehead`/`sametail` port merging

File: `sameport.c:1-189`. Public entry: `dot_sameports(g)` (`dotprocs.h:84`).
Header comment (`sameport.c:12-14`): "merge edges with specified samehead/sametail onto the
same port".

### 6.1 Data structures

```c
typedef LIST(edge_t *) edge_list_t;             // sameport.c:25
typedef struct same_t {                         // sameport.c:27-30
    char *id;          // group id (alias into the attribute dictionary string)
    edge_list_t l;     // edges in the group
} same_t;
typedef LIST(same_t) same_list_t;               // sameport.c:36
static void free_same(same_t s) { LIST_FREE(&s.l); }   // sameport.c:32-34
```

### 6.2 `dot_sameports(g)` — `sameport.c:41-78`

```
samehead = {dtor = free_same}; sametail = {dtor = free_same}
E_samehead  = agattr_text(g, AGEDGE, "samehead",  NULL)     // declare/find attribute
E_sametail  = agattr_text(g, AGEDGE, "sametail", NULL)
if (!(E_samehead || E_sametail)) return                     // neither attribute used anywhere
for n = agfstnode(g) .. agnxtnode(g,n):
    for e = agfstedge(g, n) .. agnxtedge(g, e, n):
        if (aghead(e) == agtail(e)) continue                // no same* for self loops
        if (aghead(e) == n && E_samehead && (id = agxget(e, E_samehead))[0])
            sameedge(&samehead, e, id)
        else if (agtail(e) == n && E_sametail && (id = agxget(e, E_sametail))[0])
            sameedge(&sametail, e, id)
    for i = 0 .. LIST_SIZE(&samehead) - 1:
        if (LIST_SIZE(&LIST_AT(&samehead, i)->l) > 1)
            sameport(n, LIST_GET(&samehead, i).l)
    LIST_CLEAR(&samehead)
    for i = 0 .. LIST_SIZE(&sametail) - 1:
        if (LIST_SIZE(&LIST_AT(&sametail, i)->l) > 1)
            sameport(n, LIST_GET(&sametail, i).l)
    LIST_CLEAR(&sametail)
LIST_FREE(&samehead); LIST_FREE(&sametail)
```

Processing-order facts:

- Node order: cgraph node order (`agfstnode/agnxtnode`).
- Edge order per node: `agfstedge/agnxtedge` — **out edges first (insertion/seq order within
  the dict), then in edges**, with self-loops skipped as in-edges
  (`lib/cgraph/edge.c:89-116`).
- The `if/else if` means: at node `n`, an edge whose head is `n` is considered for
  `samehead` **only**; only if that test fails (head ≠ n, or attribute missing/empty) is the
  tail tested for `sametail`. An edge can therefore join a samehead group at its head node
  and a sametail group at its tail node (two different outer-loop iterations).
- Groups are per-node and transient: cleared after that node's ports are assigned. Group
  creation order = first appearance in the edge scan (linear `streq` lookup in `sameedge`),
  and edges are appended in scan order — this order is exactly the order `sameport` averages
  over and assigns to.
- `sameport` runs only for groups with **more than one** edge.

### 6.3 `sameedge(same, e, id)` — `sameport.c:81-91`

```
for i = 0 .. LIST_SIZE(same) - 1:
    if (streq(LIST_GET(same, i).id, id)):
        LIST_APPEND(&LIST_AT(same, i)->l, e); return
same_t to_append = {.id = id}; LIST_APPEND(&to_append.l, e); LIST_APPEND(same, to_append)
```

### 6.4 `sameport(u, l)` — `sameport.c:93-189`

Places one shared port on `u`'s **boundary** along the average direction of the group's
opposite endpoints, then stamps it on every (virtual) edge of every group member at whichever
end touches `u`.

**Step 1 — average direction vector** (`sameport.c:107-124`). Direction vectors, not angles
(comment: `av(a,b) != av(a,b+2π)`):

```
x = y = 0
for e in l:
    v = (aghead(e) == u) ? agtail(e) : aghead(e)
    x1 = ND_coord(v).x - ND_coord(u).x
    y1 = ND_coord(v).y - ND_coord(u).y
    r  = hypot(x1, y1)
    x += x1 / r;  y += y1 / r
r = hypot(x, y); x /= r; y /= r
```

**Step 2 — segment through the node boundary** (`sameport.c:126-146`):

```
x1 = ND_coord(u).x; y1 = ND_coord(u).y                    // center
r  = fmax(ND_lw(u) + ND_rw(u), ND_ht(u) + GD_ranksep(agraphof(u)))   // "far away"
x2 = x * r + ND_coord(u).x;  y2 = y * r + ND_coord(u).y
// build a straight-line Bézier (4 control points) center → far point:
curve[0] = (x1, y1)
curve[1] = ((2*x1 + x2)/3, (2*y1 + y2)/3)
curve[2] = ((2*x2 + x1)/3, (2*y2 + y1)/3)
curve[3] = (x2, y2)
shape_clip(u, curve)                                      // lib/common/splines.c:193-209
x1 = curve[0].x - ND_coord(u).x;  y1 = curve[0].y - ND_coord(u).y
```

(`shape_clip` keeps whichever endpoint was inside — here `curve[0]`, the center — and moves
it to the shape boundary by binary search on the Bézier.)

**Step 3 — the port** (`sameport.c:148-159`), all fields verbatim:

```
port prt = {.p = {.x = round(x1), .y = round(y1)}}        // C round(): half away from zero
prt.bp = 0
prt.order = (MC_SCALE * (ND_lw(u) + prt.p.x)) / (ND_lw(u) + ND_rw(u))
            // MC_SCALE = 256 (lib/common/const.h:99); expression is double; the assignment to
            // `unsigned char order` TRUNCATES toward zero (C conversion). Values are
            // non-negative here; a port at the right edge yields ≈ 256, so the stored byte
            // can wrap for the extreme right-edge case — faithful ports must mimic the
            // double→unsigned char conversion.
prt.constrained = false
prt.defined = true
prt.clip = false
prt.dyna = false
prt.theta = 0
prt.side = 0
prt.name = NULL
```

**Step 4 — stamp the port onto all virtual edges** (`sameport.c:161-186`). For each edge `e`
in the group, walk `e = e; e = ED_to_virt(e)` (`for (; e; e = ED_to_virt(e))` — covers the
edge and its representative chain), and for each such `e` two more walks:

```
for (f = e; f; f = (ED_edge_type(f)==VIRTUAL && ND_node_type(aghead(f))==VIRTUAL &&
                   ND_out(aghead(f)).size==1) ? ND_out(aghead(f)).list[0] : NULL)
    if (aghead(f) == u) ED_head_port(f) = prt
    if (agtail(f) == u) ED_tail_port(f) = prt

for (f = e; f; f = (ED_edge_type(f)==VIRTUAL && ND_node_type(agtail(f))==VIRTUAL &&
                   ND_in(agtail(f)).size==1) ? ND_in(agtail(f)).list[0] : NULL)
    if (aghead(f) == u) ED_head_port(f) = prt
    if (agtail(f) == u) ED_tail_port(f) = prt
```

i.e. outward walk through single-out virtual heads and inward walk through single-in virtual
tails, assigning the port at whichever end is `u`.

**Step 5** (`sameport.c:188`): `ND_has_port(u) = true;` — comment: "kinda pointless, because
mincross is already done". Its one real consumer is `rcross` in mincross
(`mincross.c:1531-1540`), which adds port-aware `local_cross` terms for nodes with
`ND_has_port` (used when sameports is invoked on the aux graph of §6.5, where mincross runs
afterwards).

Note the doc comment (`sameport.c:93-101`): "The port is placed on the node boundary and the
average angle between the edges. FIXME: this assumes naively that the edges are straight
lines… An arr_port is also computed that's ARR_LEN away from the node boundary. It's used for
edges that don't themselves have an arrow." — the second sentence is stale in this version;
no separate arrow port is computed.

### 6.5 Call sites

- `dotLayout`: after positioning, **before** splines — `dotinit.c:296`.
- `make_flat_adj_edges` (flat edges between adjacent nodes with ports/labels,
  `dotsplines.c:1129`): builds `auxg` (clone of the two endpoints + the edge set, with a
  `rank=source` subgraph), runs `dot_rank`/`dot_mincross`/`dot_position` on it
  (`dotsplines.c:1210-1220`), repositions the two aux endpoints onto the original x
  coordinates (`dotsplines.c:1222-1234`), then `dot_sameports(auxg)` (`dotsplines.c:1235`)
  before `dot_splines_(auxg, 0)`; splines are copied back with a translation
  (`dotsplines.c:1242-1259+`).

---

## 7. `aspect.c` — aspect ratio (gutted upstream)

### 7.1 Current code, verbatim semantics — `aspect.c:27-35`

```c
void setAspect(Agraph_t *g) {
  const char *const p = agget(g, "aspect");

  if (!p || sscanf(p, "%lf,%d", &(double){0}, &(int){0}) <= 0) {
    return;
  }
  agwarningf("the aspect attribute has been disabled due to implementation "
             "flaws - attribute ignored.\n");
}
```

Behavior to port:

- Fetch graph attribute `"aspect"`.
- If absent, or `sscanf(p, "%lf,%d", ...) <= 0` (i.e. the string does not *begin* with a
  parseable `double` — the `,%d` part is optional for the return count; both scanned values
  are discarded into compound literals), silently do nothing.
- Otherwise emit the one-time warning exactly as above and do nothing else.
- Declaration: `lib/dotgen/aspect.h:13` (`extern void setAspect(Agraph_t *g);`), called from
  `dotLayout` (`dotinit.c:263`) before `dot_init_subg`.
- TODO comment block retained at `aspect.c:21-25`: support clusters; support disconnected
  graphs; provide algorithms for aspect ratios < 1. Author note: Mohammad T. Irfan,
  Summer 2008 (`aspect.c:16-19`).

There is **no** `A_ASPECT`, no `info`/`aspect_t` structure, no `badGraph`, no `ASPECT_MATRIX`
in this checkout — the entire aspect-with-mincross algorithm (which used to create a matrix
of rank pairs and bias mincross) was removed. The only surviving `ratio_t`
(`R_NONE 0, R_VALUE, R_FILL, R_COMPRESS, R_AUTO, R_EXPAND`, `lib/common/types.h:215-216`)
machinery serves the plain `ratio` attribute, below.

### 7.2 Where `ratio` is parsed (for contrast) — `lib/common/input.c:575-598`

```
setRatio(g): p = agget(g, "ratio")
    "auto"     ⇒ GD_drawing(g)->ratio_kind = R_AUTO
    "compress" ⇒ R_COMPRESS
    "expand"   ⇒ R_EXPAND
    "fill"     ⇒ R_FILL
    otherwise  ⇒ ratio = atof(p); if (ratio > 0.0) { ratio_kind = R_VALUE; GD_drawing->ratio = ratio }
```

### 7.3 What happens at the end of `dot_position` — `set_aspect` + `scale_bb` — `position.c:912-998`

This is the "scaleBB" of old versions:

```
scale_bb(g, xf, yf):                       // position.c:915-924
    for c = 1..GD_n_cluster(g): scale_bb(GD_clust(g)[c], xf, yf)   // clusters first
    GD_bb(g).LL.x *= xf; GD_bb(g).LL.y *= yf; GD_bb(g).UR.x *= xf; GD_bb(g).UR.y *= yf

set_aspect(g):                             // position.c:930-998
    rec_bb(g, g)                                        // compute all bboxes (position.c:904-910)
    if (GD_maxrank(g) > 0 && GD_drawing(g)->ratio_kind):
        sz = GD_bb(g).UR - GD_bb(g).LL; if (GD_flip(g)) sz = exch_xyf(sz)
        scale_it = true
        if (ratio_kind == R_AUTO) filled = idealsize(g, .5)             // position.c:1130+
        else                      filled = (ratio_kind == R_FILL)
        if (filled):                                    // R_FILL / R_AUTO
            if (size.x <= 0) scale_it = false
            else:
                xf = size.x / sz.x; yf = size.y / sz.y
                if (xf < 1.0 || yf < 1.0):
                    if (xf < yf) { yf /= xf; xf = 1.0; } else { xf /= yf; yf = 1.0; }
        else if (ratio_kind == R_EXPAND):
            if (size.x <= 0) scale_it = false
            else:
                xf = size.x / GD_bb(g).UR.x; yf = size.y / GD_bb(g).UR.y
                if (xf > 1.0 && yf > 1.0) { scale = fmin(xf, yf); xf = yf = scale; }
                else scale_it = false
        else if (ratio_kind == R_VALUE):
            desired = GD_drawing(g)->ratio; actual = sz.y / sz.x
            if (actual < desired) { yf = desired / actual; xf = 1.0; }
            else                  { xf = actual / desired; yf = 1.0; }
        else scale_it = false                            // R_NONE / R_COMPRESS fall through
        if (scale_it):
            if (GD_flip(g)) SWAP(&xf, &yf)
            for n in GD_nlist(g): ND_coord(n).x = round(ND_coord(n).x * xf)
                                  ND_coord(n).y = round(ND_coord(n).y * yf)
            scale_bb(g, xf, yf)
```

`R_COMPRESS` is handled elsewhere: `compress_graph` (`position.c:519-541`) only adds the
width aux edge when `GD_drawing(g)->ratio_kind != R_COMPRESS` (`position.c:524`), and
`idealsize` implements `R_AUTO`. `remove_aux_edges` runs *after* `set_aspect`
(`position.c:149-151`) because `GD_ln`/`GD_rn` feed `dot_compute_bb`.

---

## 8. `compound.c` — `compound=true` cluster-edge clipping

File: `compound.c:1-445`. Public entry: `dot_compoundEdges(g)` (`dotprocs.h:53`).
Module purpose (`compound.c:12-13`): "Module for clipping splines to cluster boxes."

### 8.1 `boxIntersectf(pp, cp, bp)` — `compound.c:29-72`

Point where segment `[pp, cp]` hits box `bp`; **assumes `cp` outside, `pp` on or in the box**.

```
ll = bp->LL; ur = bp->UR
if (cp.x < ll.x):
    ipp.x = ll.x; ipp.y = pp.y + round((ipp.x - ppx) * (ppy - cpy) / (ppx - cpx))
    if (ll.y <= ipp.y <= ur.y) return ipp
if (cp.x > ur.x):
    ipp.x = ur.x; ipp.y = pp.y + round(...same...)                       :45-50
    if (inside y range) return ipp
if (cp.y < ll.y):
    ipp.y = ll.y; ipp.x = pp.x + round((ipp.y - ppy) * (ppx - cpx) / (ppy - cpy))   :51-56
    if (inside x range) return ipp
if (cp.y > ur.y):
    ipp.y = ur.y; ipp.x = pp.x + round(...same...)                       :57-62
    if (inside x range) return ipp
// failure:
agerrorf("segment [(%.5g, %.5g),(%.5g,%.5g)] does not intersect box "
         "ll=(%.5g,%.5g),ur=(%.5g,%.5g)\n", ...)
assert(0)
return ipp     // uninitialized!
```

All four `round(...)` calls round the interpolated coordinate to integral points; the checks
are **ordered left, right, bottom, top** and return the first hit.

### 8.2 `inBoxf(p, bb)` — `compound.c:75-77`

`return INSIDE(p, *bb);` (inclusive of the boundary: `BETWEEN` is `a <= x <= b`).

### 8.3 `getCluster(cluster_name, map)` — `compound.c:83-93`

```
if (!cluster_name || *cluster_name == '\0') return NULL
sg = findCluster(map, cluster_name)             // utils.c:1591-1597; NULL if absent
if (!sg) agwarningf("cluster named %s not found\n", cluster_name)
return sg
```

So `lhead`/`ltail` naming a nonexistent cluster warns once per edge and behaves as if unset.

### 8.4 Crossing counters — `compound.c:104-144`

```
countVertCross(pts[4], xcoord):                 // crossings of vertical line x = xcoord
    sign = fcmp(pts[0].x, xcoord); if (sign == 0) num_crossings++      // endpoint ON the line counts
    for i = 1..3:
        old_sign = sign; sign = fcmp(pts[i].x, xcoord)
        if (sign != old_sign && old_sign != 0) num_crossings++
    return num_crossings

countHorzCross(pts[4], ycoord):                 // mirror with .y            :128-144
```

Both count **control-polygon** crossings (not spline crossings), with the `old_sign != 0`
guard noted in the header comment as the fix for bug 145 (Graphics Gems SGN approach,
`compound.c:95-102`).

### 8.5 `findVertical` / `findHorizontal` — `compound.c:153-185, 194-226`

Binary subdivision of one Bézier (4 control points, parameter range `[tmin,tmax]`) to the
first `t` where the spline crosses the vertical line segment `x = xcoord`,
`ymin ≤ y ≤ ymax` (mirror for horizontal). Recursion:

```
if (tmin == tmax) return tmin
no_cross = countVertCross(pts, xcoord)
if (no_cross == 0) return -1.0
// 1 crossing and endpoint within 0.005 of the line:
if (no_cross == 1 && fabs(pts[3].x - xcoord) <= 0.005):
    return (ymin <= pts[3].y <= ymax) ? tmax : -1.0
Bezier(pts, 0.5, Left, Right)
t = findVertical(Left, tmin, (tmin+tmax)/2.0, ...)
if (t >= 0.0) return t
return findVertical(Right, (tmin+tmax)/2.0, tmax, ...)
```

Tie-break: **left half first**. Constant: `0.005` epsilon (both functions,
`compound.c:170, 211`).

### 8.6 `splineIntersectf(pts, bb)` — `compound.c:235-262`

Shortest clipping of one Bézier `pts[0..3]` to enter/exit box `bb`; on success `pts` holds
the truncated Bézier with `pts[3]` on the box, returns 1; else `pts` unchanged, returns 0.

```
tmin = 2.0; origpts = pts
t = findVertical  (pts, 0, 1, bb->LL.x, LL.y..UR.y)      // left side   :240
if (0 <= t < tmin) { Bezier(origpts, t, pts, NULL); tmin = t }
t = findVertical  (pts, 0, MIN(1.0, tmin), bb->UR.x, LL.y..UR.y)   // right side :245
if (0 <= t < tmin) { Bezier(origpts, t, pts, NULL); tmin = t }
t = findHorizontal(pts, 0, MIN(1.0, tmin), bb->LL.y, LL.x..UR.x)   // bottom     :250
if (0 <= t < tmin) { Bezier(origpts, t, pts, NULL); tmin = t }
t = findHorizontal(pts, 0, MIN(1.0, tmin), bb->UR.y, LL.x..UR.x)   // top        :255
if (0 <= t < tmin) { Bezier(origpts, t, pts, NULL); tmin = t }
return tmin < 2.0 ? 1 : 0
```

Subtleties: each subsequent `find*` runs on the **already-clipped** `pts` with an upper
parameter limit `MIN(1.0, tmin)`; the re-subdivision always splits `origpts` at the accepted
`t` (so `pts` after each accept is the prefix Bézier). Side order: **left, right, bottom,
top**.

### 8.7 `makeCompoundEdge(e, clustMap)` — `compound.c:270-432`

```
lh = getCluster(agget(e, "lhead"), clustMap)     // head cluster
lt = getCluster(agget(e, "ltail"), clustMap)     // tail cluster
if (!lt && !lh) return                            // nothing to do                  :276-277
if (!ED_spl(e)) return                            :278-279
if (ED_spl(e)->size > 1):
    agwarningf("%s -> %s: spline size > 1 not supported\n", agnameof(agtail(e)), agnameof(aghead(e)))
    return                                        // only single-spline edges       :281-285
bez = ED_spl(e)->list; size = bez->size           // original Bézier point count
head = aghead(e); tail = agtail(e)
nbez = {0}; nbez.eflag = bez->eflag; nbez.sflag = bez->sflag
starti = 0; endi = 0                              // indices of first/last control point
```

**Head cluster handling** (`compound.c:306-354`), `fixed = false`:

- `lh` set but `!inBoxf(ND_coord(head), GD_bb(lh))`:
  warn `"%s -> %s: head not inside head cluster %s\n"` (args: tail name, head name,
  `agget(e,"lhead")`) — leave head alone (`:309-312`).
- Else if `inBoxf(bez->list[0], bb)` (**degenerate case**, first control point inside the
  box, `:318-335`):
  - if tail is also inside the head cluster: warn
    `"%s -> %s: tail is inside head cluster %s\n"`; nothing done (fixed stays false).
  - else if `!inBoxf(bez->sp, bb)` (tail arrow tip outside box):
    `assert(bez->sflag)` — "must be arrowhead on tail";
    `p = boxIntersectf(bez->list[0], bez->sp, bb)`; then rebuild the single Bézier as 4
    points between `p` and `bez->sp`:
    ```
    bez->list[3] = p
    bez->list[1] = mid_pointf(p, bez->sp)
    bez->list[0] = mid_pointf(bez->list[1], bez->sp)
    bez->list[2] = mid_pointf(bez->list[1], p)
    ```
    (`mid_pointf(p,q) = ((p.x+q.x)/2, (p.y+q.y)/2)`, `lib/common/geomprocs.h:104-108`)
    if `bez->eflag`: `endi = arrowEndClip(e, bez->list, starti, 0, &nbez, bez->eflag)`;
    `endi += 3`; `fixed = true`.
    (If `bez->sp` is inside the box, nothing happens — `fixed` remains false.)
- Else (first control point outside the box, `:336-352`): scan Bézier segments left→right:
  ```
  for (endi = 0; endi < size - 1; endi += 3)
      if (splineIntersectf(&bez->list[endi], bb)) break
  if (endi == size - 1):                      // no intersection anywhere
      assert(bez->eflag)
      nbez.ep = boxIntersectf(bez->ep, bez->list[endi], bb)
  else:
      if (bez->eflag) endi = arrowEndClip(e, bez->list, starti, endi, &nbez, bez->eflag)
      endi += 3
  fixed = true
  ```
  Note: on a hit at segment `endi`, `splineIntersectf` truncated `bez->list[endi..endi+3]`
  in place with `pts[3]` on the box.
- `if (!fixed) { endi = size - 1; if (bez->eflag) nbez.ep = bez->ep; }` (`:355-359`) —
  original head end preserved.

**Tail cluster handling** (`compound.c:361-417`), `fixed = false` again, symmetric but
scanning **right→left** from `endi`:

- `lt` set but `!inBoxf(ND_coord(tail), GD_bb(lt))`: warn
  `"%s -> %s: tail not inside tail cluster %s\n"` (args tail, head, `agget(e,"ltail")`).
- Else if `inBoxf(bez->list[endi], bb)` (degenerate, `:378-394`):
  - head inside tail cluster ⇒ warn `"%s -> %s: head is inside tail cluster %s\n"`.
  - else if `bez->eflag && !inBoxf(nbez.ep, bb)`:
    `p = boxIntersectf(bez->list[endi], nbez.ep, bb)`; `starti = endi - 3`;
    ```
    bez->list[starti]     = p
    bez->list[starti + 2] = mid_pointf(p, nbez.ep)
    bez->list[starti + 3] = mid_pointf(bez->list[starti + 2], nbez.ep)
    bez->list[starti + 1] = mid_pointf(bez->list[starti + 2], p)
    ```
    if `bez->sflag`: `starti = arrowStartClip(e, bez->list, starti, endi - 3, &nbez, bez->sflag)`;
    `fixed = true`.
- Else scan segments right→left (`:395-405`):
  ```
  for (starti = endi; starti > 0; starti -= 3):
      pts[0..3] = { bez->list[starti], bez->list[starti-1], bez->list[starti-2], bez->list[starti-3] }
        // note: pts[i] = bez->list[starti - i] — reversed segment
      if (splineIntersectf(pts, bb)):
          bez->list[starti - i] = pts[i] for i = 0..3      // write truncated segment back
          break
  if (starti == 0 && bez->sflag):
      nbez.sp = boxIntersectf(bez->sp, bez->list[starti], bb)     // no intersection: aim sp at box
  else if (starti != 0):
      starti -= 3
      if (bez->sflag) starti = arrowStartClip(e, bez->list, starti, endi - 3, &nbez, bez->sflag)
  fixed = true
  ```
- `if (!fixed) { /* Note: starti == 0 */ if (bez->sflag) nbez.sp = bez->sp; }` (`:418-422`).

**Splice and install** (`compound.c:424-432`):

```
nbez.size = endi - starti + 1                    // control-point count, 4 + 3k form preserved
nbez.list = gv_calloc(nbez.size, sizeof(pointf))
for i = 0, j = starti; i < nbez.size; i++, j++: nbez.list[i] = bez->list[j]
free(bez->list)
*ED_spl(e)->list = nbez                          // replaces the single bezier in place;
                                                 // ED_spl(e)->size is NOT changed (still 1)
```

So "arrow at cluster boundary" = `arrowEndClip`/`arrowStartClip` shorten the outermost Bézier
by the arrow length and capture the new `ep`/`sp` into `nbez` (`lib/common/arrows.c:285-339`);
"insertion of points" = the degenerate-case 4-point reconstruction above; the visible edge
now runs node → cluster border (`GD_bb` of the cluster, which includes the `CL_OFFSET = 8`
margin plus label space, computed in `dot_compute_bb`/`rec_bb`, `position.c:857-910`).

### 8.8 `dot_compoundEdges(g)` — `compound.c:434-445`

```
clustMap = mkClustMap(g)                    // name → cluster graph, ordered string dict
for n = agfstnode(g) .. agnxtnode(g, n):    // root graph node order
    for e = agfstout(g, n) .. agnxtout(g, e, n):
        makeCompoundEdge(e, clustMap)
dtclose(clustMap)
```

- Runs **after** `dot_splines` (`dotinit.c:301-302`), so `ED_spl` is populated.
- Only out-edges of nodes of `g` are visited (each edge once, from its tail).
- Edge attribute names: `"lhead"`, `"ltail"` (read via `agget`).
- `compound` truthiness: `mapbool(agget(g, "compound"))` (`dotinit.c:301`).
- Cluster bboxes come from `GD_bb(cluster)` computed during positioning (`rec_bb`), so this
  pass never re-measures clusters.

---

## 9. Rust port notes (undefined / sharp edges to preserve deliberately)

1. **`ED_dist(le) = MAX(lw, ED_dist(le))`** — `ED_dist` of the *representative* accumulates
   the max label width across all equivalent adjacent flat edges; per-edge `ED_dist(e)` in
   `flat_out` is a plain assignment (`flat.c:302` vs `flat.c:321`).
2. **`ND_label(vn) = ED_label(e)` shares one `textlabel_t`** between the flat edge and its
   label node; later `ED_label(fe)->pos` writes go through the node (`dotsplines.c:296`).
3. **`zapinlist` is order-destroying** — after any deletion, flat_out/other list order changes;
   `flat_search` compensates with `i--` (`mincross.c:1078`).
4. **Rank vlist aliasing** — cluster `rank_t.v` is a pointer *into* the root's rank array
   (`mincross.c:950`, `conc.c:168`); in Rust model this as `(offset, len)` views or re-check
   out each time, never as owned `Vec`s.
5. **`abomination` rebase** — `GD_rank(g) = rptr + 1` makes index −1 valid; a Rust port
   should offset all rank indices by 1 after this call (or use `isize` rank keys).
6. **`GD_rank(g)[r+1].n` in `dot_concentrate`** reads the calloc'ed slot past `maxrank`
   (`mincross.c:1141` guarantees a zero) — replicate with an explicit bounds-checked "0 if
   out of range" read.
7. **`order` byte truncation** in `sameport` (`sameport.c:151-152`): compute in `f64`, then
   convert with C semantics (`as u8` in Rust also saturates differently than C's
   wrap-on-overflow for out-of-range values — for exactness use
   `value.rem_euclid(256.0) as u8` only if you truly need bit-exact parity; in practice
   values stay in `0..=256`).
8. **`round()`** appears in `flat_node` context indirectly (`ND_coord` assignments),
   `boxIntersectf` (`compound.c:41,47,53,59`), `set_aspect` (`position.c:992-993`) and port
   rounding (`sameport.c:149`) — all are C `round` (half away from zero), *not*
   `floor(x+0.5)`.
9. **`assert(0)` after `agerrorf`** in `boxIntersectf` (`compound.c:69`) — in release builds
   the function returns an **uninitialized** `pointf`; a Rust port should return a
   `Result` and treat this path as unreachable-but-handled.
10. **Recursion depth** — `flat_search` (per rank), `postorder`, `findVertical/Horizontal`
    (log₂ of subdivision, bounded by parameter precision), `rebuild_vlists` (cluster depth).
11. **Error propagation** — `dot_concentrate`/`flat_edges` interplay with the caller's
    `rc != 0` abort paths (`dotinit.c:275-289`, `position.c:126-140`); `flat_edges`' int
    return is a bool (`reset`), `dot_concentrate`'s is success/failure.
12. **Iteration orders to freeze in tests**: `GD_nlist` = reverse insertion (`fastgr.c:181`);
    rank vlists; `agfstnode/agnxtout` dict order; `agfstedge` out-then-in
    (`cgraph/edge.c:89-116`); `flat_limits` inward two-ended scan; `dot_concentrate`
    r sharing; sameport group creation = first-appearance order.

---

## Appendix A — public entry points declared for these modules (`lib/dotgen/dotprocs.h`)

| Declaration | Line |
|---|---|
| `extern void checkLabelOrder (graph_t* g);` | dotprocs.h:29 |
| `extern void flat_edge(Agraph_t *, Agedge_t *);` | dotprocs.h:47 |
| `extern int flat_edges(Agraph_t *);` | dotprocs.h:48 |
| `extern void dot_compoundEdges(Agraph_t *);` | dotprocs.h:53 |
| `extern int portcmp(port p0, port p1);` | dotprocs.h:64 |
| `extern void rec_reset_vlists(Agraph_t *);` | dotprocs.h:66 |
| `extern void rec_save_vlists(Agraph_t *);` | dotprocs.h:67 |
| `extern WUR int dot_concentrate(Agraph_t *);` | dotprocs.h:78 |
| `extern void dot_sameports(Agraph_t *);` | dotprocs.h:84 |
| `extern void setAspect(Agraph_t *g);` | aspect.h:13 |

## Appendix B — constants index

| Constant | Value | Source |
|---|---|---|
| `HLB / HRB / SLB / SRB` | `0 / 1 / 2 / 3` | flat.c:41-44 |
| `UP / DOWN` | `0 / 1` | conc.c:21-22 |
| `MC_SCALE` | `256` | lib/common/const.h:99 |
| `CL_OFFSET` | `8` | lib/common/const.h:142 |
| node types `NORMAL..IGNORED` | `0..6` | lib/common/const.h:24-30 |
| `EDGE_LABEL` | `1 << 0` | lib/common/const.h:167 |
| `ratio_t` | `R_NONE 0, R_VALUE, R_FILL, R_COMPRESS, R_AUTO, R_EXPAND` | lib/common/types.h:215-216 |
| Bézier-on-line epsilon | `0.005` | compound.c:170, 211 |
| `splineIntersectf` initial `tmin` | `2.0` | compound.c:237 |
| `abomination` rank-array growth | `maxrank + 3` ("one for new rank, one for sentinel, one for off-by-one") | flat.c:189-190 |
| odd-rank node separation with edge labels | `5` | position.c:237 |
| `shape_clip` bisection tolerance | `0.5` (x and y) | lib/common/splines.c:144 |
