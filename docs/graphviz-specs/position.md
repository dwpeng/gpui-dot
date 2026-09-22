# `lib/dotgen/position.c` — dot phase‑3 coordinate assignment

**Implementation spec for a faithful Rust port.**

* Source: `/tmp/graphviz-src/lib/dotgen/position.c` (1159 lines), Graphviz `main` @ `2e92c7f`.
  Line references `Lnnn` refer to that file unless another file is named.
* Role in the pipeline: `dot_position` is phase 3 of `dot` layout. Called from
  `lib/dotgen/dotinit.c:289-295` (`dot2_layout`), after `dot_mincross` (phase 2) and
  before `dot_sameports` (`dotinit.c:296`) and `dot_splines` (`dotinit.c:297`).
  Declared `extern WUR int dot_position(Agraph_t *)` in `lib/dotgen/dotprocs.h:82`.
* Purpose (file header, L12-18): set `ND_coord(n).x/.y` for every node of `g` using
  `GD_rank(g)`. The x coordinates are computed by *building an auxiliary constraint
  graph and running network simplex on it* (`rank(g, 2, nsiter2(g))`, L141), then
  copying the simplex "rank" values into `ND_coord.x`. Y coordinates are computed
  directly rank-by-rank in `set_ycoords`.

---

## 0. `concentrate=true` (`conc.c`)

`dot_concentrate` runs between the first `set_ycoords` and `expand_leaves`
(position.c:126-131). It merges runs of identical virtual nodes in a rank
(`mergevirtual`, both a downward and an upward pass over the ranks) so parallel
chains share one concentrator; `rebuild_vlists` then re-derives each cluster's
rank slice from the root's rows (`GD_rank(g)[r].v = GD_rank(root)[r].v +
ND_order(lead)`), with `GD_rankleader` indexed by **absolute** rank.

Index bases matter here: `DGraph.rank` rows are allocated per absolute rank
(`allocate_ranks`) and `rank_row_index` keeps that for clusters (whose
`minrank > 0`), while `rankleader` is absolute too — mixing in a
`r - minrank` offset parks every cluster member at the same x (the aux graph
degenerates).

## 1. Legacy-name map (constructs the caller asked about that do **not** exist in this file)

This file is the *modernized* `position.c`. When Graphviz moved x-layout onto the
network-simplex ranker (the `nsiter2`/`rank(g,2,…)` scheme), the entire old
"sectors" x-placement engine was deleted. Status of every construct named in the
porting request:

| Requested symbol | Status in this tree | Replacement / location |
|---|---|---|
| `set_ycoords` | **exists** (L756-848) | — |
| `set_xcoords` | **exists but trivial** (L598-610): copies `ND_rank` (holding simplex x) into `ND_coord.x`, then restores `ND_rank` = rank index | x values produced by `rank(g, 2, nsiter2(g))` (L141) on the aux graph |
| `dot_sameports` | **not called by `dot_position`** | lives in `lib/dotgen/sameport.c:41`; called from `dotinit.c:296` *after* `dot_position` returns (§11.3) |
| `do_leaves`, `make_slots`, `scan_and_place`, `place_leaf`, `expand_leaf`, `expand_cluster`, `merge_leaves` | **removed** from the tree (grep confirms no occurrence) | leaf collapsing (`LEAFSET`) is vestigial: `ND_ranktype == LEAFSET` is never assigned anywhere in modern dotgen (only read at L1011 and `rank.c:492`), so `make_leafslots`’ expansion branch is dead-in-practice |
| `scale_clust` | **removed** | `quantum` is applied to *node dimensions* in `poly_init` (`lib/common/shapes.c:2013-2019`), not to coordinates in position (§12) |
| `check css` / `check_css` | **absent** | — |
| `updateBB` | **absent from dotgen** | bounding boxes are computed by `dot_compute_bb`/`rec_bb` (L857-910) invoked from `set_aspect` (L935). `updateBB` exists only in `neatogen/neatosplines.c` and `dotsplines.c:215,415,433` (label bbox growth during spline routing) |
| aux-graph "pos_edge" weights `ED_minlen*ED_xpenalty*ED_weight*…` | **removed** | modern weights are explicit: ordering constraints weight **0** (L269), flat-edge constraints `ED_weight(e)` (L288, L293, L325, L318), cluster compaction **128** (L380-382), compress edge **1000** (L540). `ED_xpenalty` is never read in `position.c` |
| sectors: `pcmp`, `user_spos`/`user_pos`, `sectorsize`, `guess_dist`, `NW_UP`, `NW_FAILED`, `NTWEIGHTS`, `OS` node weights, `run(g2)`, `MaxIter` | **all removed** (grep over `lib/`: none in dotgen; `user_pos`/`updateBB` only exist in neatogen) | replaced by `nsiter2` (L155-163) + `rank(g, 2, nsiter2(g))` (L141) from `lib/common/ns.c` (§9). `MaxIter` survives only in `mincross.c:1756` (median-pass count, unrelated) |
| rank partitioning of flat edges / `GD_rank(...).n` cluster merging | replaced | flat edges are handled via `ND_flat_out`/`ND_other`/`ND_alg` constraints in `make_LR_constraints` (L277-332); clusters via `GD_clust` recursion + `ln`/`rn` vnodes (§7) |
| `CL_CROSS` | not used in `position.c` | defined `const.h:144` as `1000`, used in `dotinit.c:70` (default `ED_xpenalty`) and `cluster.c:358` (cluster-skeleton penalty during phase‑1 ranking) |
| `ED_label_ontop` | not used in `position.c` | set/read in `class2.c:29` and `dotsplines.c`; label height consumption in position happens through `ND_ht` of label vnodes (`flat.c` `flat_node`) and `GD_border` |
| `GD_pt2`, `MAXDIM` | `GD_pt2` absent; `MAXDIM 10` (`const.h:160`) exists but is unused here (spline boxes) | — |
| `R_NONE/R_VALUE/R_FILL/R_COMPRESS` ratio handling | **exists** in `set_aspect` (L930-998); `R_COMPRESS` is *not* scaled there — it is turned into a constraint edge by `compress_graph` (L519-541) | — |
| `aspect.c` `setAspect` | different function | `lib/dotgen/aspect.c:27` now only warns that the `aspect` attribute is disabled; the multi-pass aspect machinery is gone. The only "aspect" code left is the static `set_aspect` in `position.c` |

Everything else requested (`set_ycoords` accumulation, `clust_ht`, `adjustRanks`,
`make_lrvn`, containment edges, `compute_bb`, `idealsize`, `make_leafslots`,
`ports_eq`, `expand_leaves`, simplex post-processing, quantum) is covered
function-by-function below.

---

## 2. Constants (verbatim, with definition sites)

```c
/* lib/common/const.h */
#define NORMAL        0    /* an original input node            */  (const.h:24)
#define VIRTUAL       1    /* virtual nodes in long edge chains */  (const.h:25)
#define SLACKNODE     2    /* encode edges in node position phase */ (const.h:26)
#define REVERSED      3                                            (const.h:27)
#define FLATORDER     4                                            (const.h:28)
#define CLUSTER_EDGE  5                                            (const.h:29)
#define IGNORED       6                                            (const.h:30)
#define SAMERANK 1  MINRANK 2  SOURCERANK 3  MAXRANK 4  SINKRANK 5      (const.h:37-41)
#define LEAFSET       6    /* set of collapsed leaf nodes       */  (const.h:42)
#define CLUSTER       7                                            (const.h:43)
#define SELF_EDGE_SIZE 18                                          (const.h:98)
#define BOTTOM_IX 0  RIGHT_IX 1  TOP_IX 2  LEFT_IX 3               (const.h:111-114)
#define BOTTOM (1<<BOTTOM_IX) RIGHT (1<<RIGHT_IX)                  (const.h:117-120)
#define TOP (1<<TOP_IX) LEFT (1<<LEFT_IX)
#define DEFAULT_NODESEP 0.25   MIN_NODESEP 0.02                    (const.h:85-86)
#define DEFAULT_RANKSEP 0.5    MIN_RANKSEP 0.02                    (const.h:87-88)
#define EDGE_LABEL    (1 << 0)   /* GD_has_labels bit */            (const.h:167)
#define HEAD_LABEL    (1 << 1)
#define TAIL_LABEL    (1 << 2)
#define GRAPH_LABEL   (1 << 3)
#define CL_OFFSET     8        /* margin of cluster box in PS points */ (const.h:142)
#define CL_CROSS      1000     /* cost of cluster skeleton edge crossing */ (const.h:144)
#define GAP           4        /* PAD: x += 4*GAP, y += 2*GAP */   (const.h:251, macros.h:27-29)

/* lib/common/types.h:215-216 */
typedef enum { R_NONE = 0, R_VALUE, R_FILL, R_COMPRESS, R_AUTO, R_EXPAND } ratio_t;

/* lib/common/arith.h:48 */
#define ROUND(f)  ((f>=0) ? (int)(f + .5) : (int)(f - .5))

/* lib/common/ns.c:55 */
enum { SEARCHSIZE = 30 };
```

Derived/related helpers used here:

```c
/* lib/common/geom.h:62 */  #define POINTS(a_inches) (ROUND((a_inches)*POINTS_PER_INCH))  // 72
/* lib/dotgen/dot.h via types.h:378 */
#define GD_flip(g) (GD_rankdir(g) & 1)
```

`GD_nodesep`/`GD_ranksep` are **`int`** fields (`types.h`, Agraphinfo_t), set at
parse time via `POINTS(xf)` (`input.c:659,673`), so defaults are 18 pt and 36 pt
respectively. `GD_exact_ranksep` is `bool`, set when the `ranksep` string contains
`"equally"` (`input.c:669-671`).

---

## 3. Data model contract (exact C types of every field touched)

Node (`Agnodeinfo_t`, `types.h`; accessor macros at L484-532):

| field | type | used for |
|---|---|---|
| `ND_coord` | `pointf` (double) | final x/y |
| `ND_rank` | **`int`** | *rank index* entering the phase; *aux-graph x variable* during `make_LR_constraints`/`rank()`; restored to rank index by `set_xcoords` |
| `ND_order` | `int` | position within `rank[r].v[]` |
| `ND_lw`, `ND_rw` | `double` | half widths (left/right of center) |
| `ND_ht` | `double` | node height |
| `ND_mval` | `double` | scratch: backup of original `ND_rw` (L248), restored later by `resetRW` in `dotsplines.c:186-195,242,252` |
| `ND_node_type` | `char` | `NORMAL` / `VIRTUAL` / `SLACKNODE` |
| `ND_clust` | `graph_t *` | lowest enclosing cluster (`mark_lowclusters`, `cluster.c:400`) |
| `ND_alg` | `void *` (edge) | for label vnodes: the labeled flat edge (`flat.c:181`) |
| `ND_other` | `elist` | self/multi edges kept off the fast graph |
| `ND_flat_out`/`ND_flat_in` | `elist` | flat edges |
| `ND_save_in`/`ND_save_out` | `elist` | snapshot of fast graph in/out lists while aux edges are installed |
| `ND_UF_size` | `int` | 1 normally; leafset member count (dead path) |
| `ND_ranktype` | `char` | `LEAFSET` test (dead) |
| `ND_next`/`ND_prev` | node pointers | `GD_nlist` doubly linked list |

Edge (`Agedgeinfo_t`): `ED_minlen` **`int`**; `ED_weight` **`int`**; `ED_xpenalty`
`short` (unused here); `ED_dist` `double` (largest label width of adjacent flat
edge, `flat.c:302,321`); `ED_label` `textlabel_t*`; `ED_head_port`/`ED_tail_port`
(`port`: `.defined`, `.p` pointf); `ED_to_orig`; `ED_adjacent`.

Graph (`Agraphinfo_t`): `GD_rank` → `rank_t` array (offset by 1; see `abomination`,
`flat.c:225-236`); `GD_minrank`/`GD_maxrank` `int`; `GD_nodesep`/`GD_ranksep`
`int`; `GD_exact_ranksep` `bool`; `GD_has_labels` `unsigned char` (bit test
`GD_has_labels(g->root) & EDGE_LABEL`, L235); `GD_ht1`/`GD_ht2` `double`
(cluster half-heights below/above… i.e. bottom/top padding of the cluster box);
`GD_ln`/`GD_rn` `node_t*` (left/right bbox vnodes); `GD_bb` `boxf`;
`GD_border` `pointf[4]` (label margins, filled in `input.c:884-893`: TB →
`TOP_IX`/`BOTTOM_IX` = label dimen padded by `PAD` (x+16, y+8); LR →
`RIGHT_IX`/`LEFT_IX` with x/y **swapped**); `GD_label`; `GD_clust[1..n_cluster]`;
`GD_nlist`; `GD_drawing` → `layout_t { size, page, margin, ratio, ratio_kind,
quantum, … }` (`types.h:213-222`).

`rank_t` (`types.h`): `n` int (node count; `v[n] == NULL` sentinel, arrays are
allocated with n+1 slots), `v node_t**`, `an`/`av` (allocated), `ht1`/`ht2`
double (height below/above centerline — *global* values, cluster-inflated),
`pht1`/`pht2` double (same but *primitive nodes only*), plus `candidate`,
`valid`, `cache_nc`, `flat` (used by other phases).

`elist` semantics (`types.h:262-272`) — **critical for the port**:

```c
elist_append(item, L): L.list = gv_recalloc(L.list, L.size+1, L.size+2, ptr);
                       L.list[L.size++] = item; L.list[L.size] = NULL;   // NULL-terminated
alloc_elist(n, L):     L.size = 0; L.list = gv_calloc(n+1, sizeof ptr);  // WIPES the list
free_list(L):          free(L.list)
zapinlist(&L, e):      swap-remove, keeps NULL termination
```

In Rust: model as `Vec<*mut Edge>` (or indices) **with an explicit trailing
NULL** if you keep pointer-based iteration; `alloc_elist` = `vec![null; n+1]`
(discard contents!).

---

## 4. `dot_position` — entry point (L121-153)

```c
int dot_position(graph_t *g) {
    if (GD_nlist(g) == NULL) return 0;              // ignore empty graph          L122-123
    mark_lowclusters(g);                            /* cluster.c:400; sets ND_clust
                                                       of nodes & chain vnodes to the
                                                       lowest containing cluster  */ L124
    set_ycoords(g);                                 //                              L125
    if (Concentrate) {                              // global bool, globals.h:62    L126-131
        const int rc = dot_concentrate(g);          // conc.c:202, returns 0
        if (rc != 0) return rc;
    }
    expand_leaves(g);                               // L132, §10.3
    if (flat_edges(g))                              // flat.c:257 → bool "reset"
        set_ycoords(g);                             // redo y if label vnodes added L133-134
    { const int rc = create_aux_edges(g); if (rc != 0) return rc; }   // L135-140, §5
    if (rank(g, 2, nsiter2(g))) {                   /* ns.c:1029; LR balance == 2
                                                       returns 1 iff aux graph not
                                                       connected                  */ L141-146
        connectGraph(g);
        const int rank_result = rank(g, 2, nsiter2(g));
        assert(rank_result == 0); (void)rank_result;
    }
    set_xcoords(g);                                 // L147, §6
    set_aspect(g);                                  // L148, §8
    remove_aux_edges(g);    /* must come after set_aspect since we now
                             * use GD_ln and GD_rn for bbox width. */              L149-151
    return 0;
}
```

Exact call order: `mark_lowclusters` → `set_ycoords` → [`dot_concentrate`] →
`expand_leaves` → [`flat_edges` → `set_ycoords`] → `create_aux_edges`
(`allocate_aux_edges` → `make_LR_constraints` → `make_edge_pairs` →
`pos_clusters` → `compress_graph`) → `rank(g,2,maxiter)` [→ `connectGraph` →
`rank` again] → `set_xcoords` → `set_aspect` (`rec_bb` → `dot_compute_bb` …
→ optional scaling) → `remove_aux_edges`.

Error contract: returns non-zero only from `dot_concentrate` (propagated) or
`create_aux_edges` (`-1` when `make_aux_edge` hits the `INT_MAX` length cap,
L192-199). `rank()` re-run failure is an `assert` (debug) — release builds
proceed with whatever ranks resulted.

Note `dot_position` is called **only on the root graph**; clusters are reached
through `GD_clust` recursion inside the helpers. All clusters share the root’s
`rank_t` array (`GD_rank(dot_root(g))` is what the cluster routines index).

---

## 5. Aux-graph construction

### 5.1 `allocate_aux_edges` (L206-221)

```c
for (n = GD_nlist(g); n; n = ND_next(n)) {
    ND_save_in(n)  = ND_in(n);        // snapshot the fast-graph lists
    ND_save_out(n) = ND_out(n);
    count i = #ND_out(n), j = #ND_in(n) (scan to NULL sentinel);
    alloc_elist(i + j + 3, ND_in(n));   // size=0, fresh calloc — ORIGINALS ARE GONE
    alloc_elist(3,          ND_out(n)); // size=0, fresh calloc
}
```

After this, `ND_in`/`ND_out` contain **only auxiliary edges**; original fast
edges are reachable only through `ND_save_in`/`ND_save_out`. This is what makes
`canreach` (L178) an *aux-graph* reachability test and makes `find_fast_edge`
(L312) able to find the just-created ordering constraint edges.

### 5.2 `make_aux_edge` (L182-204) — the only aux-edge factory

```c
edge_t *make_aux_edge(node_t *u, node_t *v, double len, int wt) {
    allocate Agedgepair_t e2 (in/out pair; only out half gets Agedgeinfo_t data);
    AGTYPE(&e2->in) = AGINEDGE; AGTYPE(&e2->out) = AGOUTEDGE;
    edge_t *e = &e2->out;
    agtail(e) = u; aghead(e) = v;
    if (len > INT_MAX) {                                     // L192-199
        agerrorf("Edge length %f larger than maximum %d allowed.\n"
                 "Check for overwide node(s).\n", len, INT_MAX);
        free(e2->out.base.data); free(e2);
        return NULL;                                         // callers propagate -1
    }
    ED_minlen(e) = ROUND(len);       // int; ROUND = (int)(len ± .5)   L200
    ED_weight(e) = wt;               // int                            L201
    fast_edge(e);                    // append to ND_out(u) and ND_in(v)
    return e;
}
```

Constraint semantics downstream (network simplex): `rank(head) − rank(tail) ≥
ED_minlen`, cost `Σ ED_weight·slack`.

### 5.3 `make_LR_constraints` (L224-336) — left/right ordering + flat edges

```c
int sep[2];
if (GD_has_labels(g->root) & EDGE_LABEL) { sep[0] = GD_nodesep(g); sep[1] = 5; }  // L235-238
else                     { sep[1] = sep[0] = GD_nodesep(g); }                     // L240-241

for (i = GD_minrank(g); i <= GD_maxrank(g); i++) {
    double last = ND_rank(rank[i].v[0]) = 0;        // x of leftmost node = 0; ALSO
                                                    // writes int ND_rank = 0     L244
    int nodesep = sep[i & 1];                       // smaller separation on odd ranks
                                                    // when edge labels exist     L245
    for (j = 0; j < rank[i].n; j++) {
        u = rank[i].v[j];
        ND_mval(u) = ND_rw(u);                      // backup original rw          L248

        /* self-edge width inflation */
        if (ND_other(u).size > 0) {                                           L249-265
            double sw = 0;
            for (k…; (e = ND_other(u).list[k]); k++)
                if (agtail(e) == aghead(e)) sw += selfRightSpace(e);
            ND_rw(u) += sw;      // NOT restored here; dotsplines resetRW restores
        }

        /* hard ordering constraint to the right neighbor */
        v = rank[i].v[j + 1];                       // v[n] == NULL sentinel       L266
        if (v) {
            double width = ND_rw(u) + ND_lw(v) + nodesep;                         L268
            e0 = make_aux_edge(u, v, width, 0);       // weight 0: pure ordering   L269
            if (e0 == NULL) return -1;
            last = (ND_rank(v) = last + width);       // int truncation of last+width,
                                                      // then last = that int      L273
        }

        /* constraints from labels of flat edges on previous rank (label vnodes) */
        if ((e = ND_alg(u))) {                                                L277-296
            e0 = ND_save_out(u).list[0];    // virtual edges u→flat-tail / u→flat-head
            e1 = ND_save_out(u).list[1];    // (created by flat.c:flat_node)
            if (ND_order(aghead(e0)) > ND_order(aghead(e1))) SWAP(&e0, &e1);
            // now head(e0) is the LEFT flat endpoint, head(e1) the RIGHT one;
            // agtail(e0) == agtail(e1) == u (the label vnode)
            int m0 = ED_minlen(e) * GD_nodesep(g) / 2;      // int*int/2, C division
            double m1 = m0 + ND_rw(aghead(e0)) + ND_lw(agtail(e0)); // + lw(u)
            /* guards needed because flat edges work very poorly with clusters: */
            if (!canreach(agtail(e0), aghead(e0)))          // no cycle u→…→head(e0)
                if (make_aux_edge(aghead(e0), agtail(e0), m1, ED_weight(e)) == NULL)
                    return -1;                    // LEFT endpoint → vnode, gap ≥ m0
            m1 = m0 + ND_rw(agtail(e1)) + ND_lw(aghead(e1));  // rw(u) + lw(right end)
            if (!canreach(aghead(e1), agtail(e1)))
                if (make_aux_edge(agtail(e1), aghead(e1), m1, ED_weight(e)) == NULL)
                    return -1;                    // vnode → RIGHT endpoint, gap ≥ m0
        }

        /* position flat edge endpoints */
        for (size_t k = 0; k < ND_flat_out(u).size; k++) {                    L299-332
            e = ND_flat_out(u).list[k];
            if (ND_order(agtail(e)) < ND_order(aghead(e)))
                 { t0 = agtail(e); h0 = aghead(e); }
            else { t0 = aghead(e); h0 = agtail(e); }     // t0 = left endpoint
            width = ND_rw(t0) + ND_lw(h0);
            int m0 = ED_minlen(e) * GD_nodesep(g) + width;   // int <- double (trunc)
            if ((e0 = find_fast_edge(t0, h0))) {
                /* "flat edge between adjacent neighbors" — with the modern
                 * list-wiping allocate_aux_edges this actually matches the
                 * weight-0 ordering constraint edge created above when t0,h0 are
                 * consecutive; reuse/strengthen it. ED_dist holds the largest
                 * label width. */
                m0 = MAX(m0, width + GD_nodesep(g) + ROUND(ED_dist(e))); // int<-dbl
                ED_minlen(e0) = MAX(ED_minlen(e0), m0);
                ED_weight(e0) = MAX(ED_weight(e0), ED_weight(e));
            } else if (!ED_label(e)) {
                /* unlabeled flat edge between non-neighbors */
                if (make_aux_edge(t0, h0, m0, ED_weight(e)) == NULL) return -1;
            }
            /* labeled non-adjacent flat edges: already constrained via ND_alg */
        }
    }
}
return 0;
```

Port-critical arithmetic notes:

* `ND_rank` is `int`: `ND_rank(v) = last + width` **truncates toward zero**, and
  `last` then continues from the truncated value (L273). Reproduce exactly.
* `m0 = ED_minlen(e) * GD_nodesep(g) + width` (L310) and the `MAX` with
  `width + GD_nodesep + ROUND(ED_dist(e))` (L316) assign doubles back into an
  `int m0` — truncating conversions.
* `ED_minlen(e) * GD_nodesep(g) / 2` (L283) is *integer* division.
* The self-edge inflation `ND_rw(u) += sw` persists for the whole phase (all
  constraints see it) and is undone later by `resetRW` (`dotsplines.c:186-195`,
  called at `dotsplines.c:242,252`) which swaps `ND_rw`/`ND_mval` for nodes with
  `ND_other`.
* `selfRightSpace(e)` (`lib/common/splines.c:1137-1157`): for a self edge with
  no ports and not placed on the left, `sw = SELF_EDGE_SIZE (18)` plus
  `GD_flip ? ED_label(e)->dimen.y : ED_label(e)->dimen.x` if labeled; else 0.

### 5.4 `make_edge_pairs` (L341-370) — soft “keep endpoints ordered” pairs

For every node `n` in nlist order, for every original out edge
`e = ND_save_out(n).list[i]`:

```c
sn = virtual_node(g);                 // prepended to GD_nlist; loop uses ND_next of n,
                                      // so fresh slacknodes are not re-visited    L349
ND_node_type(sn) = SLACKNODE;
int m0 = ED_head_port(e).p.x - ED_tail_port(e).p.x;   // double → int (truncate!)  L351
if (m0 > 0) m1 = 0; else { m1 = -m0; m0 = 0; }
make_aux_edge(sn, agtail(e), m0 + 1, ED_weight(e));   // x(tail) ≥ x(sn) + m0 + 1
make_aux_edge(sn, aghead(e), m1 + 1, ED_weight(e));   // x(head) ≥ x(sn) + m1 + 1
ND_rank(sn) = MIN(ND_rank(agtail(e)) - m0 - 1,
                  ND_rank(aghead(e)) - m1 - 1);       // tentative x (int)         L364-366
```

Port pairs let an edge’s ports push its endpoints apart/squeeze them, with the
edge’s own `ED_weight` (x direction only; `ED_xpenalty` unused).

### 5.5 `pos_clusters` (L509-517)

```c
if (GD_n_cluster(g) > 0) {
    contain_clustnodes(g);   // §7.1
    keepout_othernodes(g);   // §7.2
    contain_subclust(g);     // §7.3
    separate_subclust(g);    // §7.4
}
```

### 5.6 `compress_graph` (L519-541) — `ratio=compress`

```c
if (GD_drawing(g)->ratio_kind != R_COMPRESS) return;
pointf p = GD_drawing(g)->size;
if (p.x * p.y <= 1) return;
contain_nodes(g);                       // root's ln/rn + containment edges (§7.5)
double x = GD_flip(g) ? p.y : p.x;
x = MIN(x, USHRT_MAX);                  // 65535 guard vs INT_MAX aux-length cap  L539
make_aux_edge(GD_ln(g), GD_rn(g), x, 1000);     // width pin, weight 1000          L540
```

### 5.7 `create_aux_edges` (L543-560)

```c
allocate_aux_edges(g);                      // §5.1
rc = make_LR_constraints(g); if (rc) return rc;   // §5.3
rc = make_edge_pairs(g);     if (rc) return rc;   // §5.4
pos_clusters(g);                            // §5.5
compress_graph(g);                          // §5.6
return 0;
```

### 5.8 `remove_aux_edges` (L562-595)

Two separate passes (comment L578: “cannot be merged with previous loop” — the
unlink pass needs stable `ND_next` chain):

```c
// pass 1: free aux edges, restore lists
for (n = GD_nlist(g); n; n = ND_next(n)) {
    for (i = 0; (e = ND_out(n).list[i]); i++) { free(e->base.data); free(e); }
    free_list(ND_out(n));  free_list(ND_in(n));
    ND_out(n) = ND_save_out(n);  ND_in(n) = ND_save_in(n);
}
// pass 2: unlink & free every SLACKNODE from GD_nlist
nprev = NULL;
for (n = GD_nlist(g); n; n = nnext) {
    nnext = ND_next(n);
    if (ND_node_type(n) == SLACKNODE) {
        if (nprev) ND_next(nprev) = nnext; else GD_nlist(g) = nnext;
        if (nnext) ND_prev(nnext) = nprev;
        free(n->base.data); free(n);
    } else nprev = n;
}
```

Removed nodes: all `make_edge_pairs` slacknodes, `connectGraph` slacknodes, and
the `make_lrvn` `ln`/`rn` vnodes (they are `SLACKNODE`s too, L1085,1087).
Kept: label vnodes from `flat_node` (`VIRTUAL`), chain vnodes, normal nodes.
In Rust: aux edges are owned by the aux phase; slacknodes removed from the
intrusive nlist in the same order.

---

## 6. `set_xcoords` (L598-610) — trivial transfer

```c
rank_t *rank = GD_rank(g);
for (int i = GD_minrank(g); i <= GD_maxrank(g); i++)
    for (int j = 0; j < rank[i].n; j++) {
        node_t *v = rank[i].v[j];
        ND_coord(v).x = ND_rank(v);   // simplex x (int) → coordinate
        ND_rank(v) = i;               // restore rank index
    }
```

Nodes **not installed in rank arrays** — the `ln`/`rn` bbox vnodes of clusters —
keep `ND_rank` = their simplex x value. `dot_compute_bb` reads
`ND_rank(GD_ln(g))`/`ND_rank(GD_rn(g))` precisely for that reason (L895-896, and
the `remove_aux_edges` ordering comment L149-151).

---

## 7. Cluster machinery

### 7.1 `contain_clustnodes` (L372-386)

```c
if (g != dot_root(g)) {
    contain_nodes(g);                                   // §7.5 (creates ln/rn)
    if ((e = find_fast_edge(GD_ln(g), GD_rn(g))))       // maybe from lrvn() label edge
        ED_weight(e) += 128;                            // reinforce label-width edge
    else
        make_aux_edge(GD_ln(g), GD_rn(g), 1, 128);      // "clust compaction edge"
}
for (c = 1; c <= GD_n_cluster(g); c++)
    contain_clustnodes(GD_clust(g)[c]);                 // recurse children first
```

The 128-weight `ln→rn` edge with `minlen 1` makes the cluster box as narrow as
possible while surviving simplex; `find_fast_edge` here can find the label-width
aux edge created by `make_lrvn` (aux lists only).

### 7.2 `keepout_othernodes` (L410-442)

For each rank `r` in the cluster’s span, push outside nodes (to the left/right of
the cluster’s occupied span) away from the cluster box by `margin`:

```c
int margin = late_int(g, G_margin, CL_OFFSET, 0);       // utils.c:40
for (r = GD_minrank(g); r <= GD_maxrank(g); r++) {
    if (GD_rank(g)[r].n == 0) continue;
    v = GD_rank(g)[r].v[0];            // leftmost node of the cluster on rank r
    if (v == NULL) continue;
    for (i = ND_order(v) - 1; i >= 0; i--) {            // scan left, root's rank row
        u = GD_rank(dot_root(g))[r].v[i];
        /* can't use "is_a_vnode_of" because elists are swapped */
        if (ND_node_type(u) == NORMAL || vnode_not_related_to(g, u)) {
            make_aux_edge(u, GD_ln(g), margin + ND_rw(u), 0);   // stop at first hit
            break;
        }
    }
    for (i = ND_order(v) + GD_rank(g)[r].n;             // scan right of cluster span
         i < GD_rank(dot_root(g))[r].n; i++) {
        u = GD_rank(dot_root(g))[r].v[i];
        if (ND_node_type(u) == NORMAL || vnode_not_related_to(g, u)) {
            make_aux_edge(GD_rn(g), u, margin + ND_lw(u), 0);
            break;
        }
    }
}
for (c = 1; c <= GD_n_cluster(g); c++) keepout_othernodes(GD_clust(g)[c]);
```

When `g` is the root, `v[0]` has `ND_order == 0` and the right scan starts past
the end, so the body is a no-op — only the subcluster recursion matters.
Requires `GD_ln/GD_rn` to exist: guaranteed by ordering in `pos_clusters`
(`contain_clustnodes` → `contain_nodes` → `make_lrvn` runs first).

### 7.3 `vnode_not_related_to` (L388-399)

```c
if (ND_node_type(v) != VIRTUAL) return false;
for (e = ND_save_out(v).list[0]; ED_to_orig(e); e = ED_to_orig(e));  // chase to orig
return !(agcontains(g, agtail(e)) || agcontains(g, aghead(e)));
```

### 7.4 `contain_subclust` (L449-465) and `separate_subclust` (L472-502)

```c
/* contain_subclust: keep subclusters inside g's box */
margin = late_int(g, G_margin, CL_OFFSET, 0);
make_lrvn(g);
for (c = 1; c <= GD_n_cluster(g); c++) {
    subg = GD_clust(g)[c];  make_lrvn(subg);
    make_aux_edge(GD_ln(g),  GD_ln(subg), margin + GD_border(g)[LEFT_IX].x,  0);
    make_aux_edge(GD_rn(subg), GD_rn(g),  margin + GD_border(g)[RIGHT_IX].x, 0);
    contain_subclust(subg);
}

/* separate_subclust: keep sibling clusters apart where their rank spans overlap */
margin = late_int(g, G_margin, CL_OFFSET, 0);
for (i = 1; i <= GD_n_cluster(g); i++) make_lrvn(GD_clust(g)[i]);
for (i = 1; i <= GD_n_cluster(g); i++) {
    for (j = i + 1; j <= GD_n_cluster(g); j++) {
        low = GD_clust(g)[i]; high = GD_clust(g)[j];
        if (GD_minrank(low) > GD_minrank(high)) SWAP(&low, &high);
        if (GD_maxrank(low) < GD_minrank(high)) continue;   // disjoint spans
        if (ND_order(GD_rank(low)[GD_minrank(high)].v[0])
          < ND_order(GD_rank(high)[GD_minrank(high)].v[0]))
             { left = low; right = high; } else { left = high; right = low; }
        make_aux_edge(GD_rn(left), GD_ln(right), margin, 0);
    }
    separate_subclust(GD_clust(g)[i]);                      // note: inside the i loop
}
```

### 7.5 `contain_nodes` (L1101-1125)

```c
margin = late_int(g, G_margin, CL_OFFSET, 0);
make_lrvn(g);
ln = GD_ln(g); rn = GD_rn(g);
for (r = GD_minrank(g); r <= GD_maxrank(g); r++) {
    if (GD_rank(g)[r].n == 0) continue;
    v = GD_rank(g)[r].v[0];
    if (v == NULL) { agerrorf("contain_nodes clust %s rank %d missing node\n",
                             agnameof(g), r); continue; }
    make_aux_edge(ln, v, ND_lw(v) + margin + GD_border(g)[LEFT_IX].x, 0);
    v = GD_rank(g)[r].v[GD_rank(g)[r].n - 1];
    make_aux_edge(v, rn, ND_rw(v) + margin + GD_border(g)[RIGHT_IX].x, 0);
}
```

### 7.6 `make_lrvn` (L1078-1096)

```c
if (GD_ln(g)) return;                       // idempotent
ln = virtual_node(dot_root(g));  ND_node_type(ln) = SLACKNODE;   // L1084-1087
rn = virtual_node(dot_root(g));  ND_node_type(rn) = SLACKNODE;
if (GD_label(g) && g != dot_root(g) && !GD_flip(agroot(g))) {
    const double w = fmax(GD_border(g)[BOTTOM_IX].x, GD_border(g)[TOP_IX].x);
    make_aux_edge(ln, rn, w, 0);            // cluster at least label-wide (TB only)
}
GD_ln(g) = ln;  GD_rn(g) = rn;
```

(The comment block L1065-1076 documents intent: `ln`/`rn` give the x of the
left/right side of the cluster bbox; side labels are *inside*; in TB a labeled
cluster is forced wide enough for the label at the cost of possibly uncentered
nodes.)

---

## 8. Y coordinates

### 8.1 `set_ycoords` (L756-848)

Called with the root graph (twice in `dot_position`, L125 and L133-134).
`ND_rank` still holds **rank indices** during this function.

```c
rank_t *rank = GD_rank(g);

/* Phase A — scan ranks for tallest nodes                            L763-795 */
for (int r = GD_minrank(g); r <= GD_maxrank(g); r++)
  for (int i = 0; i < rank[r].n; i++) {
    node_t *n = rank[r].v[i];
    double ht2 = ND_ht(n) / 2;                    /* assumes symmetry ht1==ht2 */
    /* high self-edge labels can exceed the node half-height */
    if (ND_other(n).list)
      for (int j = 0; (e = ND_other(n).list[j]); j++)
        if (agtail(e) == aghead(e) && ED_label(e))
            ht2 = fmax(ht2, ED_label(e)->dimen.y / 2);
    if (rank[r].pht2 < ht2) rank[r].pht2 = rank[r].ht2 = ht2;
    if (rank[r].pht1 < ht2) rank[r].pht1 = rank[r].ht1 = ht2;
    /* update nearest enclosing cluster half-heights */
    if ((clust = ND_clust(n))) {
        int yoff = (clust == g) ? 0 : late_int(clust, G_margin, CL_OFFSET, 0);
        if (ND_rank(n) == GD_minrank(clust))
            GD_ht2(clust) = fmax(GD_ht2(clust), ht2 + yoff);
        if (ND_rank(n) == GD_maxrank(clust))
            GD_ht1(clust) = fmax(GD_ht1(clust), ht2 + yoff);
    }
  }

/* Phase B — recursive cluster heights; lbl = "some cluster has a label"  L798 */
const int lbl = clust_ht(g);                                  // §8.2

/* Phase C — initial y assignment to leftmost node of each rank, bottom-up
 * (y grows upward; rank maxrank is the bottom)                      L801-816 */
double maxht = 0;
int r = GD_maxrank(g);
ND_coord(rank[r].v[0]).y = rank[r].ht1;
while (--r >= GD_minrank(g)) {
    const double d0 = rank[r+1].pht2 + rank[r].pht1 + GD_ranksep(g); // prim sep
    const double d1 = rank[r+1].ht2  + rank[r].ht1  + CL_OFFSET;     // cluster sep
    const double delta = fmax(d0, d1);
    if (rank[r].n > 0)    /* empty ranks "reflect some problem" */
        ND_coord(rank[r].v[0]).y = ND_coord(rank[r+1].v[0]).y + delta;
    maxht = fmax(maxht, delta);
}

/* Phase D — rotated (LR) cluster labels need vertical extra room     L823-836 */
if (lbl && GD_flip(g)) {
    adjustRanks(g, 0);                                        // §8.3 / §8.4
    if (GD_exact_ranksep(g)) {          /* recompute maxht from actual gaps */
        maxht = 0; r = GD_maxrank(g);
        double d0 = ND_coord(rank[r].v[0]).y;
        while (--r >= GD_minrank(g)) {
            const double d1 = ND_coord(rank[r].v[0]).y;
            maxht = fmax(maxht, d1 - d0);
            d0 = d1;
        }
    }
}

/* Phase E — re-assign if ranks are equally spaced                    L839-843 */
if (GD_exact_ranksep(g))
    for (r = GD_maxrank(g) - 1; r >= GD_minrank(g); r--)
        if (rank[r].n > 0)
            ND_coord(rank[r].v[0]).y = ND_coord(rank[r+1].v[0]).y + maxht;

/* Phase F — copy y from leftmost nodes to all nodes                  L846-847 */
for (node_t *n = GD_nlist(g); n; n = ND_next(n))
    ND_coord(n).y = ND_coord(rank[ND_rank(n)].v[0]).y;   // ND_rank = rank index here
```

Notes:
* `ED_label(e)->dimen.y / 2` for self-loop labels uses the label dimen **as
  stored** (no flip swap) — reproduce verbatim.
* `rank[].ht1/ht2` and `pht1/pht2` start from whatever phase‑1/2 code left; here
  they are re-derived as node half-heights (Phase A writes both `pht*` and `ht*`
  together, so after Phase A `ht1==pht1`, `ht2==pht2` per rank).
* `yoff` uses the *immediate* cluster’s margin; nodes whose lowest cluster is the
  root get `yoff = 0` (root margin/label is handled in `dotneato_postprocess`,
  cf. comment L735).
* Empty ranks (`n == 0`) are skipped in Phases C/E but `v[0]` dereferences assume
  non-empty neighboring ranks (matches the C code’s implicit assumption).

### 8.2 `clust_ht` (L708-753)

Recursively folds subcluster half-heights and (TB) cluster-label heights into
`GD_ht1/GD_ht2` and the **global** rank half-heights. Returns “some cluster has
a label”.

```c
int margin = (g == dot_root(g)) ? CL_OFFSET
                                : late_int(g, G_margin, CL_OFFSET, 0);   // L716-719
double ht1 = GD_ht1(g), ht2 = GD_ht2(g);
int haveClustLabel = 0;

for (c = 1; c <= GD_n_cluster(g); c++) {
    subg = GD_clust(g)[c];
    haveClustLabel |= clust_ht(subg);                          // children first
    if (GD_maxrank(subg) == GD_maxrank(g)) ht1 = MAX(ht1, GD_ht1(subg) + margin);
    if (GD_minrank(subg) == GD_minrank(g)) ht2 = MAX(ht2, GD_ht2(subg) + margin);
}
/* root-graph label room is handled in dotneato_postprocess (comment L735) */
if (g != dot_root(g) && GD_label(g)) {
    haveClustLabel = 1;
    if (!GD_flip(agroot(g))) {                                 // TB only
        ht1 += GD_border(g)[BOTTOM_IX].y;                      // bottom label
        ht2 += GD_border(g)[TOP_IX].y;                         // top label
    }
}
GD_ht1(g) = ht1;  GD_ht2(g) = ht2;
if (g != dot_root(g)) {          /* propagate into the shared global rank array */
    rank[GD_minrank(g)].ht2 = MAX(rank[GD_minrank(g)].ht2, ht2);
    rank[GD_maxrank(g)].ht1 = MAX(rank[GD_maxrank(g)].ht1, ht1);
}
return haveClustLabel;
```

Subtlety: for the **root** the child-padding margin is the constant
`CL_OFFSET` (the root’s own `margin` attribute is ignored here), while every
non-root level uses its own `late_int(g, G_margin, CL_OFFSET, 0)`.

### 8.3 `adjustRanks` (L656-701) — LR cluster labels

Only invoked when `lbl && GD_flip(g)` with `(g, 0)`:

```c
rank_t *rank = GD_rank(dot_root(g));
int margin = (g == dot_root(g)) ? 0 : late_int(g, G_margin, CL_OFFSET, 0);
double ht1 = GD_ht1(g), ht2 = GD_ht2(g);

for (c = 1; c <= GD_n_cluster(g); c++) {
    subg = GD_clust(g)[c];
    adjustRanks(subg, margin + margin_total);           // margin_total accumulates
    if (GD_maxrank(subg) == GD_maxrank(g)) ht1 = fmax(ht1, GD_ht1(subg) + margin);
    if (GD_minrank(subg) == GD_minrank(g)) ht2 = fmax(ht2, GD_ht2(subg) + margin);
}
GD_ht1(g) = ht1;  GD_ht2(g) = ht2;

if (g != dot_root(g) && GD_label(g)) {
    double lht = MAX(GD_border(g)[LEFT_IX].y, GD_border(g)[RIGHT_IX].y);  // LR dims
    int maxr = GD_maxrank(g), minr = GD_minrank(g);
    double rht = ND_coord(rank[minr].v[0]).y - ND_coord(rank[maxr].v[0]).y;
    double delta = lht - (rht + ht1 + ht2);
    if (delta > 0) adjustSimple(g, delta, margin_total);            // §8.4
}
if (g != dot_root(g)) {
    rank[GD_minrank(g)].ht2 = fmax(rank[GD_minrank(g)].ht2, GD_ht2(g));
    rank[GD_maxrank(g)].ht1 = fmax(rank[GD_maxrank(g)].ht1, GD_ht1(g));
}
```

### 8.4 `adjustSimple` (L622-649)

Expand cluster height by `delta`: `bottom = (delta+1)/2` goes below, the rest
above; shift affected rank-leftmost y’s upward (y += positive delta).

```c
root = dot_root(g); rank = GD_rank(root);
maxr = GD_maxrank(g); minr = GD_minrank(g);
const double bottom = (delta + 1) / 2;
const double delbottom = GD_ht1(g) + bottom - (rank[maxr].ht1 - margin_total);
double deltop;
if (delbottom > 0) {
    for (r = maxr; r >= minr; r--)                    // ranks inside the cluster
        if (rank[r].n > 0) ND_coord(rank[r].v[0]).y += delbottom;
    deltop = GD_ht2(g) + (delta - bottom) + delbottom - (rank[minr].ht2 - margin_total);
} else
    deltop = GD_ht2(g) + (delta - bottom) - (rank[minr].ht2 - margin_total);
if (deltop > 0)
    for (r = minr - 1; r >= GD_minrank(root); r--)    // ranks above the cluster
        if (rank[r].n > 0) ND_coord(rank[r].v[0]).y += deltop;
GD_ht2(g) += delta - bottom;
GD_ht1(g) += bottom;
```

`margin_total` (accumulated ancestor margins) converts the *global* rank
`ht1/ht2` — which include ancestor padding — back to this cluster’s frame.

---

## 9. Simplex invocation — `nsiter2` + `rank(g, 2, …)`

### 9.1 `nsiter2` (L155-163)

```c
int maxiter = INT_MAX;
char *s;
if ((s = agget(g, "nslimit")))
    maxiter = scale_clamp(agnnodes(g), atof(s));
return maxiter;
```

* `agnnodes(g)` counts the *original* graph nodes (aux/slacknodes are not
  `aginsert`ed). `nslimit` scales node count: `scale_clamp(n, f)`
  (`lib/util/gv_math.h:79-91`) = `0` if `f<0`; `INT_MAX` if `f>1 && n > INT_MAX/f`;
  else `(int)(n*f)`. `maxiter ≤ 0` makes `rank2` return right after
  `feasible_tree` (no optimization, no balancing) — `ns.c:981-984`.

### 9.2 `rank(g, balance=2, maxiter)` (`lib/common/ns.c:1029-1040` → `rank2` L962+)

Relevant contract for position (balance == 2, “LR balance”):

1. `searchsize` attribute (default `SEARCHSIZE = 30`) bounds leave-edge search.
2. `init_graph` (ns.c:895-925): for all nodes reachable from `GD_nlist`
   (this includes aux vnodes): clear `ND_mark`, count; zero `ND_priority`,
   `ED_cutvalue=0`, `ED_tree_index=-1` for every aux edge; feasibility test
   `ND_rank(head) − ND_rank(tail) ≥ ED_minlen` over `ND_in`. **Feasibility is
   usually true** because `make_LR_constraints`/`make_edge_pairs` seed `ND_rank`
   with a consistent tentative x assignment.
3. If infeasible, `init_rank` recomputes `ND_rank` (longest-path via topo order).
4. `feasible_tree` (ns.c:623-664): tight-subtree merge; error **2** if a node has
   no tight subtree, **1** if `inter_tree_edge` finds nothing → *graph not
   connected* (this is the only way `rank()` returns nonzero from
   `dot_position`).
5. If `maxiter <= 0`: return 0 immediately after the tree is feasible.
6. Improvement loop: `leave_edge` → `enter_edge` → `update`, `iter++`, stop when
   no leaving edge or `iter >= maxiter` (ns.c:986-1000).
7. `balance == 2` → `LR_balance` (ns.c:788-796): for each **tree edge** with
   `ED_cutvalue == 0`, compute `f = enter_edge(e)`; if `f != NULL` and
   `delta = SLACK(f) > 1`, rerank the smaller side by `±delta/2` (integer
   division): `ND_lim(tail) < ND_lim(head) ? rerank(tail, delta/2)
   : rerank(head, -delta/2)`. `rerank` (ns.c:694-704) recursively shifts the
   subtree via `ND_tree_in/out`. This spreads neutral nodes evenly — the
   x-direction “balance” pass.
   (`balance == 1` would be `TB_balance`; unused from position.)
8. Returns 0 / 1 (disconnected) / 2 (internal LCA mismatch). On 1,
   `dot_position` runs `connectGraph` and re-ranks (asserting 0).

### 9.3 `connectGraph` (L72-119)

Fixes the “components remain because of `source`/`sink` subgraphs” case by
linking rank-first nodes with 0-weight/0-minlen slacknode edges. Called **after**
a failed `rank()`, i.e. when `ND_rank` currently holds simplex x values — the
comparisons below literally compare x values against the rank index `r`;
reproduce as written.

```c
for (r = GD_minrank(g); r <= GD_maxrank(g); r++) {
    rp = GD_rank(g) + r;  found = false;  tp = NULL;
    for (i = 0; i < rp->n; i++) {
        tp = rp->v[i];
        if (ND_save_out(tp).list)
            for (j = 0; (e = ND_save_out(tp).list[j]); j++)
                if (ND_rank(aghead(e)) > r || ND_rank(agtail(e)) > r)
                     { found = true; break; }
        if (found) break;
        if (ND_save_in(tp).list)
            for (j = 0; (e = ND_save_in(tp).list[j]); j++)
                if (ND_rank(agtail(e)) > r || ND_rank(aghead(e)) > r)
                     { found = true; break; }
        if (found) break;
    }
    if (found || !tp) continue;
    tp = rp->v[0];
    hp = (r < GD_maxrank(g)) ? (rp+1)->v[0] : (rp-1)->v[0];
    assert(hp);
    sn = virtual_node(g);  ND_node_type(sn) = SLACKNODE;
    make_aux_edge(sn, tp, 0, 0);
    make_aux_edge(sn, hp, 0, 0);
    ND_rank(sn) = MIN(ND_rank(tp), ND_rank(hp));
}
```

---

## 10. Leaf handling & misc

### 10.1 `make_leafslots` (L1001-1028)

```c
for (r = GD_minrank(g); r <= GD_maxrank(g); r++) {
    int j = 0;
    for (i = 0; i < GD_rank(g)[r].n; i++) {
        v = GD_rank(g)[r].v[i];
        ND_order(v) = j;
        if (ND_ranktype(v) == LEAFSET) j += ND_UF_size(v);   // leave a hole
        else j++;
    }
    if (j <= GD_rank(g)[r].n) continue;                     // no expansion needed
    node_t **new_v = gv_calloc(j + 1, sizeof(node_t*));
    for (i = GD_rank(g)[r].n - 1; i >= 0; i--)              // back to front
        new_v[ND_order(GD_rank(g)[r].v[i])] = GD_rank(g)[r].v[i];
    GD_rank(g)[r].n = j;
    new_v[j] = NULL;
    free(GD_rank(g)[r].v);  GD_rank(g)[r].v = new_v;
}
```

The holes (`NULL` slots between a leafset leader and the next node) were filled
by the old `do_leaves`, which no longer exists; and since `ND_ranktype` is never
set to `LEAFSET` in modern dotgen, the expansion branch is effectively dead.
Port it anyway for fidelity.

### 10.2 `ports_eq` (L1030-1039)

```c
int ports_eq(edge_t *e, edge_t *f) {
    return ED_head_port(e).defined == ED_head_port(f).defined
        && ((ED_head_port(e).p.x == ED_head_port(f).p.x &&
             ED_head_port(e).p.y == ED_head_port(f).p.y) || !ED_head_port(e).defined)
        && ((ED_tail_port(e).p.x == ED_tail_port(f).p.x &&
             ED_tail_port(e).p.y == ED_tail_port(f).p.y) || !ED_tail_port(e).defined);
}
```

### 10.3 `expand_leaves` (L1041-1063) — contains a live bug

```c
make_leafslots(g);
for (n = GD_nlist(g); n; n = ND_next(n))
    if (ND_other(n).list)
        for (i = 0; (e = ND_other(n).list[i]); i++) {
            if ((d = ND_rank(aghead(e)) - ND_rank(aghead(e))) == 0)  // L1051: BOTH
                continue;                                            // are aghead!
            f = ED_to_orig(e);
            if (!ports_eq(e, f)) {
                zapinlist(&(ND_other(n)), e);
                if (d == 1) fast_edge(e);
                /* else unitize(e); ### */
                i--;
            }
        }
```

**`d` is identically 0** (`aghead(e) − aghead(e)`), so every iteration hits
`continue` and the whole `ND_other` loop is a no-op in this snapshot. (The
historical code computed `ND_rank(aghead(e)) − ND_rank(agtail(e))`.) A faithful
port must reproduce the no-op (or guard it behind the same expression); do not
“fix” it silently.

### 10.4 `go` / `canreach` (L165-180)

```c
static bool go(node_t *u, node_t *v) {
    if (u == v) return true;
    for (int i = 0; (e = ND_out(u).list[i]); i++)
        if (go(aghead(e), v)) return true;
    return false;
}
static bool canreach(node_t *u, node_t *v) { return go(u, v); }
```

Depth-first reachability over the **current** `ND_out` (aux edges only during
`make_LR_constraints`, §5.1). No visited-set — termination relies on the aux
graph being acyclic at that point (the `canreach` guards themselves exist to
keep it that way).

---

## 11. Bounding boxes, ratio, aspect

### 11.1 `dot_compute_bb` (L857-902)

```c
static void dot_compute_bb(graph_t *g, graph_t *root) {
    if (g == dot_root(g)) {
        LL.x = INT_MAX;  UR.x = -INT_MAX;                       // L865-866
        for (r = GD_minrank(g); r <= GD_maxrank(g); r++) {
            int rnkn = GD_rank(g)[r].n;
            if (rnkn == 0) continue;
            if ((v = GD_rank(g)[r].v[0]) == NULL) continue;
            for (c = 1; ND_node_type(v) != NORMAL && c < rnkn; c++)
                v = GD_rank(g)[r].v[c];                          // first NORMAL from left
            if (ND_node_type(v) == NORMAL) {
                x = ND_coord(v).x - ND_lw(v);
                LL.x = MIN(LL.x, x);
            } else continue;            /* rank has no NORMAL node → skip row */
            v = GD_rank(g)[r].v[rnkn - 1];                       // first NORMAL from right
            for (c = rnkn - 2; ND_node_type(v) != NORMAL; c--)
                v = GD_rank(g)[r].v[c];
            x = ND_coord(v).x + ND_rw(v);
            UR.x = MAX(UR.x, x);
        }
        offset = CL_OFFSET;                                      // L887
        for (c = 1; c <= GD_n_cluster(g); c++) {                 // expand by clusters
            x = (double)(GD_bb(GD_clust(g)[c]).LL.x - offset);  LL.x = MIN(LL.x, x);
            x = (double)(GD_bb(GD_clust(g)[c]).UR.x + offset);  UR.x = MAX(UR.x, x);
        }
    } else {
        LL.x = ND_rank(GD_ln(g));                                // L895 — simplex x
        UR.x = ND_rank(GD_rn(g));                                // L896 — simplex x
    }
    LL.y = ND_coord(GD_rank(root)[GD_maxrank(g)].v[0]).y - GD_ht1(g);   // L898
    UR.y = ND_coord(GD_rank(root)[GD_minrank(g)].v[0]).y + GD_ht2(g);   // L899
    GD_bb(g).LL = LL;  GD_bb(g).UR = UR;
}
```

* Root: x extent from leftmost/rightmost `NORMAL` node per rank, widened by
  every child cluster bbox ± `CL_OFFSET`. `ln`/`rn` of the root are **not**
  consulted.
* Cluster: x extent = the `ln`/`rn` x values held in `ND_rank` (see §6);
  y extent from the global rank array at the cluster’s min/max rank and the
  cluster’s `GD_ht1/GD_ht2`.
* Cluster label side margins live in `GD_border(LEFT_IX/RIGHT_IX).x` and were
  already added as aux-edge minlens (§7.3-7.5); `GD_border(TOP/BOTTOM).y` space
  is inside `GD_ht1/ht2` via `clust_ht`.

### 11.2 `rec_bb` (L904-910) and `scale_bb` (L915-924)

```c
static void rec_bb(graph_t *g, graph_t *root) {           // post-order
    for (c = 1; c <= GD_n_cluster(g); c++) rec_bb(GD_clust(g)[c], root);
    dot_compute_bb(g, root);
}
static void scale_bb(graph_t *g, double xf, double yf) {  // children first
    for (c = 1; c <= GD_n_cluster(g); c++) scale_bb(GD_clust(g)[c], xf, yf);
    GD_bb(g).LL.x *= xf;  GD_bb(g).LL.y *= yf;
    GD_bb(g).UR.x *= xf;  GD_bb(g).UR.y *= yf;
}
```

### 11.3 `set_aspect` (L930-998) — ratio handling (this *is* the old aspect.c logic)

```c
double xf = 0.0, yf = 0.0;  bool filled;
rec_bb(g, g);
if (GD_maxrank(g) > 0 && GD_drawing(g)->ratio_kind) {      // ratio_kind != R_NONE
    pointf sz = sub_pointf(GD_bb(g).UR, GD_bb(g).LL);      // normalize
    if (GD_flip(g)) sz = exch_xyf(sz);
    bool scale_it = true;
    if (GD_drawing(g)->ratio_kind == R_AUTO) filled = idealsize(g, .5);   // §11.4
    else                     filled = GD_drawing(g)->ratio_kind == R_FILL;
    if (filled) {                                          /* R_FILL / R_AUTO-hit */
        if (GD_drawing(g)->size.x <= 0) scale_it = false;
        else {
            xf = GD_drawing(g)->size.x / sz.x;
            yf = GD_drawing(g)->size.y / sz.y;
            if (xf < 1.0 || yf < 1.0) {                    // only grow
                if (xf < yf) { yf /= xf; xf = 1.0; }
                else         { xf /= yf; yf = 1.0; }
            }
        }
    } else if (GD_drawing(g)->ratio_kind == R_EXPAND) {
        if (GD_drawing(g)->size.x <= 0) scale_it = false;
        else {
            xf = GD_drawing(g)->size.x / GD_bb(g).UR.x;    // note: UR, not sz
            yf = GD_drawing(g)->size.y / GD_bb(g).UR.y;
            if (xf > 1.0 && yf > 1.0) { double s = fmin(xf, yf); xf = yf = s; }
            else scale_it = false;
        }
    } else if (GD_drawing(g)->ratio_kind == R_VALUE) {
        double desired = GD_drawing(g)->ratio;
        double actual  = sz.y / sz.x;
        if (actual < desired) { yf = desired / actual; xf = 1.0; }
        else                  { xf = actual / desired; yf = 1.0; }
    } else scale_it = false;          /* R_COMPRESS lands here (handled in
                                         compress_graph §5.6); R_NONE excluded */
    if (scale_it) {
        if (GD_flip(g)) SWAP(&xf, &yf);
        for (n = GD_nlist(g); n; n = ND_next(n)) {
            ND_coord(n).x = round(ND_coord(n).x * xf);     // C99 round: half away
            ND_coord(n).y = round(ND_coord(n).y * yf);     // from zero — NOT ROUND()
        }
        scale_bb(g, xf, yf);
    }
}
```

Key discriminations for the port:
* Guard `GD_maxrank(g) > 0` — single-rank graphs are never scaled.
* `R_EXPAND` divides by `UR.x`/`UR.y` (not `sz = UR − LL`); `LL` may be nonzero
  after cluster expansion but `UR` is what counts.
* Scaling rounds *node coordinates* with `round()` but scales cluster bboxes
  purely multiplicatively (`scale_bb`).
* `xf`/`yf` are swapped **after** factor computation when `GD_flip`.
* `lib/dotgen/aspect.c` (`setAspect`, L27-33) is unrelated now: it only warns
  that the `aspect` attribute is disabled.

### 11.4 `idealsize` (L1130-1159)

```c
static bool idealsize(graph_t *g, double minallowed) {      // minallowed = 0.5
    relpage = GD_drawing(g)->page;
    if (relpage.x < 0.001 || relpage.y < 0.001) return false;   // no page → not filled
    margin  = GD_drawing(g)->margin;
    relpage = sub_pointf(relpage, margin);  relpage = sub_pointf(relpage, margin);
    b.x = GD_bb(g).UR.x;  b.y = GD_bb(g).UR.y;
    xf = relpage.x / b.x;  yf = relpage.y / b.y;
    if (xf >= 1.0 && yf >= 1.0) return false;                   // fits on one page
    f  = MIN(xf, yf);
    xf = yf = MAX(f, minallowed);
    R  = ceil(xf * b.x / relpage.x);  xf = R * relpage.x / b.x;  // whole pages
    R  = ceil(yf * b.y / relpage.y);  yf = R * relpage.y / b.y;
    GD_drawing(g)->size.x = b.x * xf;
    GD_drawing(g)->size.y = b.y * yf;
    return true;                                                // → treat as R_FILL
}
```

---

## 12. Quantum

`GD_drawing(g)->quantum` (double, `types.h:219`) is parsed in
`lib/common/input.c:633-634` and applied **only to node dimensions**, inside
`poly_init` (`lib/common/shapes.c:2013-2019`):

```c
if ((temp = GD_drawing(agraphof(n))->quantum) > 0.0) {
    temp = INCH2PS(temp);                       // ×72
    dimen.x = quant(dimen.x, temp);
    dimen.y = quant(dimen.y, temp);
}
// static double quant(double val, double q) { return ceil(val / q) * q; }  (shapes.c:367)
```

So by the time `dot_position` runs, `ND_lw/ND_rw/ND_ht` already include quantum
rounding. **`position.c` applies no quantum to x/y** (no `QUANTUM` macro, no
`quant` call); the old `scale_clust`-based coordinate quantization is gone.
Propagation into subgraphs/aux graphs: `dotinit.c:311`,
`dotsplines.c:790`.

---

## 13. Cross-file helpers the port needs (one-line contracts)

| helper | file:line | contract |
|---|---|---|
| `mark_lowclusters(g)` | cluster.c:400 | (re)set `ND_clust` for nodes and chain vnodes to lowest containing cluster |
| `dot_concentrate(g)` | conc.c:202 | merge parallel-edge chains when `concentrate=true`; returns 0; no-op if `maxrank−minrank ≤ 1` |
| `flat_edges(g)` | flat.c:257 | mark `ED_adjacent` (via `checkFlatAdjacent`, flat.c:209: adjacent iff no `NORMAL`/labeled-vnode strictly between); for labeled non-adjacent flat edges create label vnode (`flat_node`, flat.c:155-196: `ND_alg(vn)=e`, `ND_ht=dimen.y` (swapped if flip), `ND_lw=ND_rw=dimen.x/2`, two virtual edges vn→endpoints with x-offset ports, raises `rank[r-1].ht1/ht2` to `h2`); for labeled adjacent edges `ED_dist = flip ? dimen.y : dimen.x` (max on representative, flat.c:302,321). **Returns true** iff any `flat_node` was created (`reset`) → caller must redo `set_ycoords` |
| `virtual_node(g)` | fastgr.c:200-214 | zeroed node; `ND_node_type=VIRTUAL`; `lw=rw=1`; `ht=1`; `UF_size=1`; `alloc_elist(4)` in/out; **prepends** to `GD_nlist` via `fast_node` (fastgr.c:177-189) |
| `fast_edge(e)` | fastgr.c:69+ | append `e` to `ND_out(tail)` and `ND_in(head)` |
| `find_fast_edge(u,v)` | fastgr.c:43-45 | search shorter of `ND_out(u)`/`ND_in(v)` for the connecting edge (during position: aux edges only) |
| `new_virtual_edge` | fastgr.c:137+ | copies `ED_count/xpenalty/weight/minlen`, ports (orientation-aware), sets `ED_to_orig/ED_to_virt` |
| `selfRightSpace(e)` | splines.c:1137-1157 | §5.3 note |
| `late_int(obj,attr,def,min)` | utils.c:40-52 | unset/invalid → `def`; else `max(min, strtol)` |
| `ROUND(f)` | arith.h:48 | `(int)(f±.5)` — distinct from C99 `round()` |
| `scale_clamp(n,f)` | util/gv_math.h:79 | §9.1 |
| `rank(g,balance,maxiter)` | ns.c:1029 | §9.2 |
| `dot_sameports(g)` | sameport.c:41-78 | merges `samehead`/`sametail` edge groups to common ports; runs **after** `dot_position` (`dotinit.c:296`) |
| `resetRW(g)` | dotsplines.c:187-195 | swaps back `ND_rw`/`ND_mval` for nodes with `ND_other` (undo of L248/L264), called from `dot_splines` (dotsplines.c:242,252) |
| `GD_border` fill | input.c:884-893 | TB: `TOP_IX`/`BOTTOM_IX` = PADed label dimen; LR: `RIGHT_IX`/`LEFT_IX` with x/y swapped |

---

## 14. Rust porting notes

1. **`ND_rank` is `int`.** Every `ND_rank(v) = <double>` truncates toward zero
   (C cast). In particular `last = (ND_rank(v) = last + width)` (L273) and
   `m0 = … + width` (L310) / `m0 = MAX(m0, width + nodesep + ROUND(ED_dist(e)))`
   (L316) are truncating conversions. Keep `rank: i32` on the node and mirror
   the truncation; use `f64` everywhere else (`coord`, `lw/rw/ht`, `ht1/ht2`,
   `pht1/pht2`, `ED_dist`, `GD_border`).
2. **Two `round()`s are different**: `ROUND(f)` (`arith.h:48`, truncating
   ±0.5 add, used for `ED_minlen` and `ED_dist`) vs C99 `round()` (half away
   from zero, used for scaled coordinates, L992-993). `f64::round` in Rust
   matches C99 `round`.
3. **elist NULL termination + `alloc_elist` wipe** (§3). The port must model the
   wipe in `allocate_aux_edges`: after it, `find_fast_edge`/`canreach` operate on
   the aux subgraph only. If you model lists as `Vec`, keep the
   “append-only-after-wipe” invariant and remember `v[n]` sentinel reads
   (`rank[i].v[j+1]` at L266 — iterate `0..n` and test `Some`).
4. **Iterate `GD_nlist` in list order** (prepend order: aux slacknodes are
   created at the head; `make_edge_pairs` inserts while iterating but only
   follows `ND_next` of the current node). Deterministic order affects simplex
   tie-breaking → affects output coordinates.
5. **Aux edge ownership**: one allocation per aux pair; freeing is done from the
   tail side (`ND_out`) only (§5.8). In Rust, aux edges can be arena-allocated
   and dropped wholesale at `remove_aux_edges`; slacknode unlinking must
   preserve `nlist` order semantics for the second `rank()` call.
6. **`canreach` has no memoization** and is called per labeled flat edge; port
   semantics (aux graph, current edges) exactly, optionally with a visited set
   only if you can prove identical results (the aux graph is acyclic at those
   points).
7. **Error paths**: `make_aux_edge` returning `None` (length > `INT_MAX`)
   propagates `-1` up through `make_LR_constraints`/`make_edge_pairs` and aborts
   positioning with that code from `dot_position` (L127-139). `WUR` marks
   `dot_position`/`create_aux_edges`/`make_LR_constraints` results as
   must-use.
8. **Cluster bboxes need `ln`/`rn` x from `ND_rank`** — do not copy `ND_rank`
   into `ND_coord.x` for nodes absent from rank arrays (§6). Respect the
   `remove_aux_edges` *after* `set_aspect` ordering (L149-151): `rec_bb` →
   `dot_compute_bb` dereferences `GD_ln(g)`/`GD_rn(g)` (both their `ND_rank`
   scalars and the nodes themselves); `remove_aux_edges` frees the `ln`/`rn`
   slacknodes, so running it first would leave `set_aspect` reading freed nodes.
9. **Empty-rank robustness**: Phase C of `set_ycoords`, `connectGraph`,
   `make_LR_constraints` (via `rank[i].v[0]`), and `dot_compute_bb` dereference
   `v[0]` of possibly-empty ranks. C relies on ranks never being empty in valid
   input (abomination/label-rank fixes guarantee rank 0 exists; empty middle
   ranks are “a problem”). Decide: mirror the UB as a panic, or guard like
   `connectGraph` does (`!tp → continue`).
10. **`GD_flip`** is `rankdir & 1` — `LR` and `RL` are “flipped”; `GD_border`
    side indices were already swapped at parse time (§13), so position code
    reads `TOP/BOTTOM` for vertical space and `LEFT/RIGHT` for horizontal
    space *as stored*.
11. **Reproducibility**: all iteration orders above (ranks ascending, nodes left
    to right, nlist order, cluster indices 1..n, sibling pairs `i<j`) are part
    of the observable behavior because simplex tie-breaks and y-shift
    accumulation depend on them.
```
