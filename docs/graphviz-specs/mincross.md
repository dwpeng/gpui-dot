# `lib/dotgen/mincross.c` — Exhaustive Implementation Spec for a Faithful Rust Port

Source: Graphviz `main`, file `lib/dotgen/mincross.c`, 1795 lines, HEAD commit
`2e92c7f2776f6611dd78fa83c213ecc65cc47bfa` ("Merge branch
'smattr/b76224c0-725d-40e4-8dce-b0d8e108f36a'", file last touched 2026-09-20).
Verified byte-identical to upstream GitLab `main` at spec time.

All line numbers below refer to `/tmp/graphviz-src/lib/dotgen/mincross.c`
unless another file is named. Every constant is quoted verbatim. Pseudocode is
exact (condition order, increment placement, and loop bounds matter for output
equivalence). There is **no randomness anywhere in this file** (verified:
no `drand48`/`rand`/`srand` in `lib/dotgen/*.c`), so identical input +
identical iteration order ⇒ identical output ordering is achievable.

---

## 0a. `class2` backward-edge branch (port bug, fixed)

class2.c:262-265 guards the "backward edge shadows a forward edge" search with

```c
if (aghead(opp) != agtail(e) || aghead(opp) == aghead(e) ||
    ED_edge_type(opp) == IGNORED)
    continue;
```

An earlier revision of this port wrote `aghead(opp) == aghead(opp)` for the
middle test — a tautology, so the branch was dead code. Consequences: a
backward edge never folded into the forward edge it shadows, so it got its own
virtual chain and its own spline (an extra curve on every graph with such a
pair) and `concentrate=true` left the backward chain un-merged (`a->b; b->a;
a->b` drew two curves where dot draws one). Fixed: the branch now runs, with
C's `Concentrate` arm (`IGNORED` + `ED_conc_opp_flag(opp) = true`) and the
non-concentrate arm (`other_edge` + `merge_chain`).

## 0. Cluster port notes (this port)

The cluster machinery is implemented; these are the traps the port hit, kept
here so the next reader does not rediscover them:

1. **`build_ranks`'s flip loop must reverse the row *in place*.** mincross.c:1258-1267
   calls `exchange(vlist[j], vlist[last-j])`, and `exchange` writes
   `GD_rank(Root)[r].v[ND_order(·)]`. While the root's own rows are being built
   the two views coincide, but `build_ranks` also runs for an *unexpanded*
   cluster (`expand_cluster`), where node orders are cluster-relative: C then
   writes into the root's row at those small indices. Reversing the cluster's
   own row is the same operation and cannot corrupt the root (the previous
   port copied C literally and lost a root-row node, producing a duplicated
   entry that made `rec_reset_vlists`/`furthestnode` loop forever on
   `rankdir=LR` cluster graphs).
2. **`rankdir` is per graph.** `GD_flip(g)` reads `GD_rankdir(g) &
   1` (types.h:376-378) and `initSubg` copies `GD_rankdir2` from the parent, so
   clusters *are* flipped for `rankdir=LR`. Sites that need the root's
   direction use `GD_flip(agroot(g))` explicitly (class2.c:30, position.c:738,
   position.c:1089) — this port keeps `DGraph.rankdir` per graph for the same
   reason.
3. **Aliasing.** C re-points `GD_rank(subg)[r].v` into the root's row in
   `merge_ranks`; the arena keeps a copy plus `DGraph.rank_offset` and mirrors
   root-row writes into every expanded cluster
   (`cluster::refresh_expanded_clusters`, called from `exchange` and
   `restore_best`).
4. **`rank_t.pht1/pht2` are not `ht1/ht2`.** Cluster nesting and cluster
   labels raise `ht1/ht2` (`clust_ht`), while `set_ycoords` needs the
   primitive heights for `d0`; using `ht` for both inflates every rank gap by
   the label band (2-30pt on the corpus).
5. **`win`/`candidate` are per rank for the current graph**, but
   `allocate_ranks(cluster)` allocates fewer rows than the root has: grow them
   without shrinking, and reset only the cluster's own span.
6. **Cluster label placement** uses the `LABEL_AT_*` bits
   (`const.h:175-178`: BOTTOM=0, TOP=1, LEFT=2, RIGHT=4) and the band width
   from `GD_border`, which `do_graph_label` fills with `dimen + PAD`.

## 0. Legacy symbols the caller asked about that DO NOT exist in this version

Checked by grep over the whole tree:

| Asked-about symbol | Status | Modern equivalent |
|---|---|---|
| `MEDIANB` / `Median` macro (dot.h) | **absent** | inline code in `medians()` (L1635–1662): `case 2: (list[0]+list[1])/2` (C int division, trunc toward 0; operands ≥ 0 so = floor) and the even-`j` weighted-median branch |
| `N_CONCENTRATOR` | **absent** | `concentrate` is handled by `dot_concentrate()` in `conc.c` (L202), invoked from `position.c` L126–127 *after* mincross. mincross.c is not aware of it |
| `GD_pass` / `GD_n_nodes` | **absent** | pass/iteration state is local to `mincross()` (`pass`, `iter`, `trying`); per-rank crossing cache is `rank_t.valid` + `rank_t.cache_nc` |
| `CountPassing` / `N_engine` | **absent** | convergence loop is `MinQuit` + `Convergence` (L160–161, L730, L737) |
| `MaxIter` | exists — **global** `int` (`lib/common/globals.h:64`), default 24 set in `mincross_options()` L1756 |
| `cluster_cross` | **absent** | cluster crossing cost is folded into the global `ncross()` via `ED_xpenalty = CL_CROSS` on skeleton edges (`cluster.c` `build_skeleton`, `dot_init_edge`) |
| `gen_revert` | **absent** | — |
| `valence` | **absent** | — |
| `wmed` | **absent as a name**; the weighted median is inline in `medians()` L1650–1660 |
| `sortlist` | local alias in `do_ordering_node()` L444: `edge_t **sortlist = TE_list;` |
| `CL_CROSS` | not referenced *in this file*; set on edges in `dotinit.c` L68–74 and `cluster.c` `build_skeleton`; read here via `ED_xpenalty()` |

---

## 1. Data model (contract the port must reproduce)

### 1.1 Graph / node / edge fields used

Fast-graph structures built by `class2()` (class2.c) and `flat_edges()`
(flat.c); all `elist`s (`ND_in/ND_out/ND_flat_in/ND_flat_out/ND_other`) are
**NULL-terminated arrays** (`types.h:251-254`; `elist_append` keeps a trailing
`NULL`, `alloc_elist(n, L)` zeroes an `n+1` array). Iteration idiom
`for (i = 0; (e = L.list[i]); i++)` relies on that NULL sentinel.

| Accessor | C type | Meaning |
|---|---|---|
| `ND_rank(n)` | `int` | rank index (fixed before mincross) |
| `ND_order(n)` | `int` | index within `GD_rank(Root)[ND_rank(n)].v` (+ slice offset); the value being minimized |
| `ND_mval(n)` | `double` | median value / sort key; `-1` = "fixed" (not comparable) |
| `ND_low(n)` | `int` | per-rank stable alias set by `flat_breakcycles` (L1102); `flatindex(v) := (size_t)ND_low(v)` (L115) |
| `ND_coord(n).x` | `double` | **aliased as `saveorder(v)`** (L114) — scratch storage for best-so-far order. Coord is not assigned until position phase, so this is safe in C; a Rust port should use a dedicated `saved_order` field |
| `ND_mark`, `ND_onstack` | bool-ish | DFS marks (`size_t` / `char` in `Agnodeinfo_t`) |
| `ND_node_type` | `char` | `NORMAL=0`, `VIRTUAL=1` (`const.h:24-25`) |
| `ND_ranktype` | `char` | `CLUSTER=7` (`const.h:40`) marks cluster representative nodes (real cluster members and their skeleton vnodes) |
| `ND_clust(n)` | `graph_t*` | innermost cluster owning the node (`NULL` at root level) |
| `ND_weight_class(n)` | `char` | # incident edges capped at 3; set in `class2.c:166-171` (`if (<=2) ++` for each edge endpoint while iterating out-edges) |
| `ND_has_port(n)` | `bool` | set by port computation; only used by `rcross` (L1533, L1538) |
| `ED_xpenalty(e)` | `short` | crossing weight; default 1, `CL_CROSS` (1000, or 100 on `_WIN32`) for same-group edges (`dotinit.c:68-74`), 0 for non-constraint edges, merged/accumulated by `merge_chain`/`basic_merge` |
| `ED_weight(e)` | `int` | constraint weight; 0 disables flat-edge constraints |
| `ED_edge_type(e)` | `char` | `REGULAREDGE=1`, `REVERSED=3`, `FLATORDER=4` (`const.h:150,27-28`) |
| `ED_to_orig(e)` | `edge_t*` | chain back-pointer |
| `ED_tail_port/ED_head_port(e).p.x`, `.order` | `double`, `unsigned char` | port position + tie-break order; `.order` is 0 during mincross in practice (`MC_SCALE/2` uses happen later in shapes.c) |
| `AGSEQ(e)` | `size_t` | cgraph creation sequence; the total order for `edgeidcmpf` |

### 1.2 Rank arrays and the aliasing invariants (critical)

`rank_t` (`types.h:198-213`): `int n; node_t **v; int an; node_t **av; double
ht1, ht2, pht1, pht2; bool candidate; bool valid; int64_t cache_nc;
adjmatrix_t *flat;`

* **Root graph**: `GD_rank(g)` is a `GD_maxrank+2` array (L1141). `av`/`an` are
  the allocated capacity (`cn[r]+1`, one slot of slack + NULL sentinel), `v`/`n`
  the currently-filled prefix. During per-component processing `init_mccomp`
  (L422-432) *advances* `GD_rank(g)[r].v` by the previous component's `n` and
  zeroes `n` — i.e. `v` is a moving window into `av`. `exchange()` (L623-633)
  always writes through `GD_rank(Root)[r].v`, which is the *same moving
  pointer*, so indices are component-relative while `ND_order` values are also
  component-relative until `merge2` renumbers them globally (L818-828).
* **Clusters**: `expand_cluster` (`cluster.c:280-295`) builds cluster-local
  ranks (`class2(subg)`, `allocate_ranks(subg)`, `build_ranks(subg,0)`,
  `merge_ranks(subg)`); `merge_ranks` (`cluster.c:218-248`) copies the cluster's
  nodes into the root arrays at the rankleader slot and then re-points
  `GD_rank(subg)[r].v = GD_rank(root)[r].v + ipos` — a **view into the root
  array** of length `n`. So during `mincross_clust`, `GD_rank(g)[r].v` (cluster
  slice) and `GD_rank(Root)[r].v[...]` (exchange) alias the same memory.
  `valid`/`candidate`/`flat`/`cache_nc`, however, are per-graph struct fields:
  `transpose_step` sets `GD_rank(g)[r].candidate` but
  `GD_rank(Root)[r].valid=false` (L660-669); `ncross()` always scans **Root**
  (L1548-1550).
* `ND_order` is a *global* node attribute: any code may compare orders of nodes
  on different ranks / different clusters. This is why the rank structure is
  global (file header comment L12-15).

### 1.3 Module-static state (L159-179)

```c
static int MinQuit;                    // L160  (set in mincross_options)
static const double Convergence = .995;// L161
static graph_t *Root;                  // L163  set in init_mincross = the top-level graph
static int GlobalMinRank, GlobalMaxRank;// L164 snapshot of root rank bounds (L1029-1030); restored by merge_components L802-803
static edge_t **TE_list;               // L165  scratch: edge sort list, size agnedges(root)+1 (L1019-1020)
static int *TI_list;                   // L166  scratch: median value list, same size
static bool ReMincross;                // L167  true only during the remincross re-run (L388)
```

`info_t` (L169-173) `{ Agrec_t h; int x, lo, hi; Agnode_t *np; }` with macros
`ND_x/ND_lo/ND_hi/ND_np/ND_idx` (L175-179); `ND_idx(n) := ND_order(ND_np(n))`.
Used only by the label-ordering section (§8).

Small helpers (L112-115):

```c
#define MARK(v)      (ND_mark(v))
#define saveorder(v) (ND_coord(v)).x     // aliasing! see 1.1
#define flatindex(v) ((size_t)ND_low(v))
#define isBackedge(e) (ND_idx(aghead(e)) > ND_idx(agtail(e)))  // L191
#define VAL(node, port) (MC_SCALE * ND_order(node) + (port).order) // L1612
```

`MC_SCALE = 256` (`lib/common/const.h:99`). `MIN/MAX` are naive C macros
(`arith.h:28,33`). `SWAP(a,b)` is a generic swap (`gv_math.h:137`).
`scale_clamp(original, scale)` (`gv_math.h:79-93`): `assert(original >= 0)`;
if `scale < 0` → 0; if `scale > 1 && original > INT_MAX/scale` → `INT_MAX`;
else `(int)(original * scale)` (C double multiply, trunc toward 0).

### 1.4 `adjmatrix_t` (L39-110) — dynamic bit matrix

* `matrix_get(me,row,col)` (L51-66): out-of-range ⇒ `false`; else bit
  `row*ncols+col`.
* `matrix_set` (L73-110): if out of range, **reallocate** to
  `nrows'=max(nrows,row+1)`, `ncols'=max(ncols,col+1)` (fresh zeroed buffer,
  `bytes = bits/8 + (bits%8?1:0)`), copy old set bits at their
  `r*ncols'+c` positions, replace `*me`. Then set the bit. Bit order:
  little-endian within each byte (`(data[i/8] >> i%8) & 1`).
* `new_matrix(r,c)` (L405-413): zeroed buffer of `r*c` bits (rounded up to
  bytes), `nrows=r, ncols=c`. `free_matrix` (L415-420) frees data+struct.

Semantics of the flat matrix: `M[a][b] == true` means "a must stay left of b"
(freeze current relative order); set by `flat_search` (§6).

---

## 2. Entry point: `dot_mincross` (L332-403) — public, returns `int rc`

```
fn dot_mincross(g) -> i32:
  rc = 0
  # L341-353: drop empty clusters (malformed input guard)
  i = 1
  while i <= GD_n_cluster(g):
      if GD_clust(g)[i] has no nodes:
          agwarning("removing empty cluster\n")
          memmove GD_clust(g)[i..] left by one        # (size_t)(n_cluster - i) entries
          GD_n_cluster(g) -= 1                        # do NOT increment i
      else: i += 1

  init_mincross(g)                                    # L355
  has_set_vlists = false                              # L356

  nc : i64 = 0
  for comp in 0 .. GD_comp(g).size:                   # L359-367
      init_mccomp(g, comp)
      mc = mincross(g, 0)
      if mc < 0 { rc = -1; goto done }
      nc += mc

  merge2(g)                                           # L369

  # L372-383: per-cluster mincross, outermost first, in cluster-index order
  for c in 1 ..= GD_n_cluster(g):
      mc = mincross_clust(GD_clust(g)[c])
      if mc < 0 { rc = -1; goto done }
      nc += mc
  has_set_vlists = true                               # L384

  # L386-399: remincross over the assembled graph
  if GD_n_cluster(g) > 0 and (agget(g,"remincross") is unset or mapbool(s)):
      mark_lowclusters(g)
      ReMincross = true
      mc = mincross(g, 2)
      if mc < 0 { rc = -1; goto done }
      nc = mc                    # NOTE: assigned, not accumulated (L394)

done:                                                # L400
  cleanup2(g, nc, has_set_vlists)                    # L401
  return rc                                          # 0 on success, -1 on failure
```

Notes:
* `mapbool` accepts "true/false/yes/no/on/off/1/0..." — unset attribute string
  is `NULL`/`""` which also yields true (condition `!(s=agget(...)) || mapbool(s)`).
* `goto done` paths skip remaining phases but always run `cleanup2`; note
  `has_set_vlists` stays `false` if a failure happened before L384, so
  `cleanup2` will skip `rec_reset_vlists`.

## 2.1 `init_mincross` (L1010-1031)

```
fn init_mincross(g):
  if Verbose: start_timer()
  ReMincross = false
  Root = g
  size = agnedges(dot_root(g)) + 1          # +1 for NULL terminator used by do_ordering_node (L459-460)
  TE_list = calloc(size, edge_t*)
  TI_list = calloc(size, int)
  mincross_options(g)                       # §12
  if GD_flags(g) & NEW_RANK: fillRanks(g)   # NEW_RANK = 1<<4 = 16 (const.h:243); §7.5
  class2(g)                                 # builds fast graph, virtual chains, flat edges, skeletons
  decompose(g, 1)                           # connected components into GD_comp(g).list; pass=1 ⇒ cluster-aware:
                                            #   for node v: if ND_clust(v) then v = GD_rankleader(subg)[ND_rank(v)] (decomp.c:108+)
  allocate_ranks(g)                         # §5.2
  ordered_edges(g)                          # §8.3 (ordering= attributes)
  GlobalMinRank = GD_minrank(g); GlobalMaxRank = GD_maxrank(g)
```

`class2` is a no-op for cluster subgraphs' internal structures here; per-cluster
class2 runs inside `expand_cluster`.

## 2.2 `mincross_options` (L1750-1763)

```
MinQuit = 8
MaxIter = 24                                  # global from globals.h:64
p = agget(g, "mclimit")
if p != NULL and (f = atof(p)) > 0.0:
    MinQuit = max(1, scale_clamp(MinQuit, f))
    MaxIter = max(1, scale_clamp(MaxIter, f))
```
`mclimit` is the only tuning input; `scale_clamp` truncates
`(int)(value * f)`. No other attribute affects this file except
`ordering` (§8.3) and `remincross` (§2).

---

## 3. Comparators and sort usage (L1562-1694) — determinism analysis

```c
ordercmpf(x,y)   // L1562-1572: compare *(int*)x vs *(int*)y → -1/0/1
nodeposcmpf(x,y) // L1672-1682: compare ND_order(*n0) vs ND_order(*n1)
edgeidcmpf(x,y)  // L1684-1694: compare AGSEQ(*e0) vs AGSEQ(*e1)
```

`qsort` call sites and stability:

| Site | Sorts | Key | Ties possible? | Effect of instability |
|---|---|---|---|---|
| L461 `do_ordering_node` | edges | `AGSEQ` (unique) | no | none |
| L277 `fixLabelOrder` | `int indices[]` | value | yes (equal `ND_idx`) | sorted *values* identical; pairing `indices[i]`↔`arr[i]` uses `arr` order from `topsort` (deterministic) ⇒ deterministic |
| L646 `medians` | `int list[]` of `VAL` | value | yes | sorted multiset of ints is unique ⇒ deterministic |
| L767 `restore_best` | node ptrs per rank | `ND_order` (unique permutation within rank) | no | none |

⇒ A Rust port may use any sort algorithm; use `sort_by` with the same
comparators. (`AGSEQ` order = cgraph creation sequence: edges get increasing
sequence numbers in declaration/expansion order; the port must keep an
equivalent monotone edge id, including for virtual/flatorder edges created
later — they must sort *after* earlier-created ones, in creation order.)

---

## 4. Crossing counting

### 4.1 `in_cross(v, w)` (L587-604) — returns `int64_t`

Counts crossings between w's in-edges and v's in-edges **as if v were placed
left of w** (order(v) < order(w)):

```
cross = 0i64
for e2 in ND_in(w).list (until NULL):        # outer over w
    cnt = ED_xpenalty(e2)
    inv = ND_order(agtail(e2))
    for e1 in ND_in(v).list (until NULL):    # inner over v, list order preserved
        t = ND_order(agtail(e1)) - inv
        if t > 0 or (t == 0 and ED_tail_port(e1).p.x > ED_tail_port(e2).p.x):
            cross += ED_xpenalty(e1) * cnt   # int * short → int, accumulated in i64
return cross
```
Tie-break: equal tail order ⇒ crossing counted iff e1's tail port x is
**strictly greater**.

### 4.2 `out_cross(v, w)` (L606-621) — returns `int` (i32!)

Identical shape over `ND_out`/`aghead`/`ED_head_port`:
```
cross = 0i32                                  # NOTE: 32-bit accumulator, unlike in_cross
for e2 in ND_out(w).list:
    cnt = ED_xpenalty(e2); inv = ND_order(aghead(e2))
    for e1 in ND_out(v).list:
        t = ND_order(aghead(e1)) - inv
        if t > 0 or (t == 0 and ED_head_port(e1).p.x > ED_head_port(e2).p.x):
            cross += ED_xpenalty(e1) * cnt
return cross
```
A faithful port keeps the i32/i64 asymmetry (wrap only in absurd graphs, but
the *types* feed `transpose_step`'s i64 arithmetic — sign behavior of the sum
is what matters).

### 4.3 `exchange(v, w)` (L623-633)

```
r = ND_rank(v); vi = ND_order(v); wi = ND_order(w)
ND_order(v) = wi;  GD_rank(Root)[r].v[wi] = v
ND_order(w) = vi;  GD_rank(Root)[r].v[vi] = w
```
(No bound/type checks; assumes v,w same rank.)

### 4.4 `transpose_step(g, r, reverse)` (L635-674) — returns i64 improvement

```
rv = 0
GD_rank(g)[r].candidate = false                 # clear BEFORE scanning
for i in 0 .. GD_rank(g)[r].n - 2:              # adjacent pairs, left→right
    v = rank_v[r][i]; w = rank_v[r][i+1]
    assert ND_order(v) < ND_order(w)
    if left2right(g, v, w): continue            # frozen pair (§7.6)
    c0 = 0; c1 = 0                              # crossings as-is vs swapped
    if r > 0:                       c0 += in_cross(v,w);  c1 += in_cross(w,v)
    if GD_rank(g)[r+1].n > 0:       c0 += out_cross(v,w); c1 += out_cross(w,v)
    if c1 < c0 or (c0 > 0 and reverse and c1 == c0):    # ← exact swap condition
        exchange(v, w)
        rv += c0 - c1
        GD_rank(Root)[r].valid   = false
        GD_rank(g)[r].candidate  = true
        if r > GD_minrank(g):
            GD_rank(Root)[r-1].valid  = false
            GD_rank(g)[r-1].candidate = true
        if r < GD_maxrank(g):
            GD_rank(Root)[r+1].valid  = false
            GD_rank(g)[r+1].candidate = true
return rv
```
Tie-breaker: swaps of **equal** cost happen only when `reverse` is set and the
crossing count is nonzero. Note `in_cross` is consulted iff `r > 0` (not "rank
r-1 non-empty"), and `out_cross` iff rank `r+1` is non-empty.

### 4.5 `transpose(g, reverse)` (L676-690)

```
for r in minrank..=maxrank: GD_rank(g)[r].candidate = true
loop:
    delta = 0
    for r in minrank..=maxrank:
        if GD_rank(g)[r].candidate: delta += transpose_step(g, r, reverse)
    if delta < 1: break                          # exact: while (delta >= 1)
```
`delta` sums i64 improvements; a pass of pure tie-swaps (reverse mode) yields
`delta == 0` and terminates. Termination is guaranteed because each swap either
strictly decreases crossings (bounded below) or is a tie-swap inside a single
`transpose_step` sweep — note a tie-swap can enable a later improving swap in
the same sweep, but the outer loop still exits on `delta == 0`.

### 4.6 `local_cross(l, dir)` (L1482-1504) — pairwise within one elist

```
cross = 0i32
for i in 0..:
    e = l.list[i]; if NULL break
    for j in i+1..:
        f = l.list[j]; if NULL break
        if dir > 0:  crossed = (ND_order(aghead(f)) - ND_order(aghead(e)))
                             * (ED_tail_port(f).p.x - ED_tail_port(e).p.x)  < 0
        else:        crossed = (ND_order(agtail(f)) - ND_order(agtail(e)))
                             * (ED_head_port(f).p.x - ED_head_port(e).p.x)  < 0
        if crossed: cross += ED_xpenalty(e) * ED_xpenalty(f)
return cross
```
(The product-of-differences `< 0` test is the exact C code — integer × double
promoted to double, compare `< 0`.)

### 4.7 `rcross(g, r)` (L1506-1543) — crossings between rank r and r+1

```
cross = 0i64; max = 0
Count = calloc(GD_rank(Root)[r+1].n + 1, int)     # NOTE: Root's row length, +1 slack
for top in 0 .. GD_rank(g)[r].n - 1:
    if max > 0:
        for e in ND_out(rtop[top]).list:
            for k in ND_order(aghead(e))+1 ..= max:
                cross += Count[k] * ED_xpenalty(e)
    for e in ND_out(rtop[top]).list:
        inv = ND_order(aghead(e))
        if inv > max: max = inv
        Count[inv] += ED_xpenalty(e)
# port-aware corrections (both ranks):
for each v in GD_rank(g)[r].v:     if ND_has_port(v): cross += local_cross(ND_out(v), +1)
for each v in GD_rank(g)[r+1].v:   if ND_has_port(v): cross += local_cross(ND_in(v),  -1)
free Count; return cross
```
This is the classic O(n·E + maxspan) accumulation: `Count[k]` = total xpenalty
of edges seen so far ending at column k; a new edge to column `inv` crosses all
previously accumulated edges ending at columns `> inv`. `Count` is indexed by
head order which may exceed the allocated length only if `Root[r+1].an` was
exceeded — in practice orders of rank r+1 nodes are `< Root[r+1].n ≤ an`.
During per-component processing, `GD_rank(Root)[r+1].n` may be *smaller* than
the actual orders used (component-local orders run 0..comp_n-1 while the
allocated `Count` uses Root's row length) — keep allocation from `Root`'s `.n`
exactly as written to preserve any (historically benign) over/under-allocation
behavior; the safe port should allocate `max(an, max_order+2)`.

### 4.8 `ncross()` (L1545-1560)

```
g = Root; count = 0i64
for r in GD_minrank(g) ..< GD_maxrank(g):        # exclusive upper bound
    if GD_rank(g)[r].valid: count += GD_rank(g)[r].cache_nc
    else:
        nc = rcross(g, r); GD_rank(g)[r].cache_nc = nc
        count += nc; GD_rank(g)[r].valid = true
return count
```
Cache invalidation is done by `valid=false` in: `build_ranks` (L1259),
`transpose_step` (L660-669), `reorder` (L1449-1451 — note **only** r and r-1,
never r+1), `flat_reorder` (L1399), `merge_ranks` (cluster.c).

---

## 5. Rank-array construction and components

### 5.1 `init_mccomp(g, c)` (L422-432)

```
GD_nlist(g) = GD_comp(g).list[c]
if c > 0:
    for r in minrank..=maxrank:
        GD_rank(g)[r].v += GD_rank(g)[r].n      # advance window past previous component
        GD_rank(g)[r].n = 0
```

### 5.2 `allocate_ranks(g)` (L1122-1147) — public

```
cn = calloc(GD_maxrank(g)+2, int)               # 0-based, not minrank-based (comment L1128)
for n in agfstnode..agnxtnode(g):
    cn[ND_rank(n)] += 1
    for e in out-edges of n:
        low,high = ND_rank(agtail(e)), ND_rank(aghead(e)); if low>high swap
        for r in low+1 ..< high: cn[r] += 1     # interior ranks of long edges
GD_rank(g) = calloc(GD_maxrank(g)+2, rank_t)    # zeroed (rows outside [min,max] are zeroed)
for r in minrank..=maxrank:
    GD_rank(g)[r].an = GD_rank(g)[r].n = cn[r] + 1     # +1 slack: sentinel & insert headroom
    GD_rank(g)[r].av = GD_rank(g)[r].v = calloc(cn[r]+1, node_t*)
free cn
```

### 5.3 `install_in_rank(g, n)` (L1150-1193) — public, returns 0 / -1

```
r = ND_rank(n); i = GD_rank(g)[r].n
if GD_rank(g)[r].an <= 0: agerror("install_in_rank, line %d: %s %s rank %d i = %d an = 0\n", ...); return -1
GD_rank(g)[r].v[i] = n; ND_order(n) = i; GD_rank(g)[r].n += 1
assert GD_rank(g)[r].n <= GD_rank(g)[r].an
# DEBUG only: assert n is in GD_nlist chain (L1165-1174)
if ND_order(n) > GD_rank(Root)[r].an: agerror("... ND_order(%s) [%d] > GD_rank(Root)[%d].an [%d]\n"); return -1
if r < GD_minrank(g) or r > GD_maxrank(g): agerror("... rank %d not in rank range [%d,%d]\n"); return -1
if GD_rank(g)[r].v + ND_order(n) > GD_rank(g)[r].av + GD_rank(Root)[r].an: agerror("..."); return -1
return 0
```

### 5.4 `build_ranks(g, pass)` (L1199-1273) — public, returns 0 / -1

Initial ordering — BFS from edgeless nodes; called twice for the root (pass 0
seeds from in-degree-0 nodes, pass 1 from out-degree-0 nodes) and once per
cluster (pass 0, inside `expand_cluster`).

```
for n in GD_nlist(g) chain: MARK(n) = false
for i in minrank..=maxrank: GD_rank(g)[i].n = 0

walkbackwards = (g != agroot(g))     # clusters: traverse GD_nlist via ND_prev to preserve input node order (comment L1222-1224)
ns = last node of GD_nlist chain if walkbackwards else GD_nlist(g)
for n = ns; n != NULL; n = walkbackwards ? ND_prev(n) : ND_next(n):
    otheredges = (pass == 0) ? ND_in(n).list : ND_out(n).list
    if otheredges[0] != NULL: continue          # seed only sources (pass 0) / sinks (pass 1)
    if MARK(n): continue
    MARK(n) = true; push_back(queue, n)
    while queue not empty:
        n0 = pop_front(queue)
        if ND_ranktype(n0) != CLUSTER:
            if install_in_rank(g, n0) != 0: free queue; return -1
            enqueue_neighbors(&queue, n0, pass)
        else:
            rc = install_cluster(g, n0, pass, &queue)   # cluster.c:380-395: installs the cluster's whole
            if rc != 0: free queue; return rc           #   rankleader chain via install_in_rank, then
                                                        #   enqueue_neighbors of each rankleader; guarded by
                                                        #   GD_installed(clust) == pass+1
assert queue empty
for i in minrank..=maxrank:
    GD_rank(Root)[i].valid = false
    if GD_flip(g) and GD_rank(g)[i].n > 0:              # flip ⇒ reverse each filled row in place
        vlist = GD_rank(g)[i].v; last = GD_rank(g)[i].n - 1
        for j in 0 ..= last/2: exchange(vlist[j], vlist[last - j])   # NB: j == last-j swaps a node with itself for odd n

if g == dot_root(g) and ncross() > 0: transpose(g, false)   # L1269-1270
free queue
return 0
```
`enqueue_neighbors(q, n0, pass)` (L1275-1295): pass 0 ⇒ for each out-edge in
list order, if `!MARK(head)` mark+push; else (pass ≠ 0) same over in-edges to
tails. Marks are set at push time (standard BFS dedup).

The pass-0 vs pass-1 seeding produces the two "series-parallel friendly"
initial orders (comment L1195-1198); `mincross` keeps whichever is better via
`save_best`.

### 5.5 `merge_components(g)` (L784-804)

```
if GD_comp(g).size <= 1: (still fall through? NO — early return, but see below)
    return                      # early return keeps comp/nlist as-is
u = NULL
for c in 0 ..< GD_comp(g).size:
    v = GD_comp(g).list[c]
    if u: ND_next(u) = v
    ND_prev(v) = u
    while ND_next(v): v = ND_next(v)     # walk to end of this component's chain
    u = v
GD_comp(g).size = 1
GD_nlist(g) = GD_comp(g).list[0]
GD_minrank(g) = GlobalMinRank; GD_maxrank(g) = GlobalMaxRank
```

### 5.6 `merge2(g)` (L807-830)

```
merge_components(g)
for r in minrank..=maxrank:
    GD_rank(g)[r].n = GD_rank(g)[r].an     # full allocated width (sentinel included in scan)
    GD_rank(g)[r].v = GD_rank(g)[r].av     # reset window to base
    for i in 0 ..< n:
        v = GD_rank(g)[r].v[i]
        if v == NULL:
            if Verbose: eprintf("merge2: graph %s, rank %d has only %d < %d nodes\n", agnameof(g), r, i, n)
            GD_rank(g)[r].n = i; break
        ND_order(v) = i                     # global renumbering
```
The `.n = .an` + NULL-scan repairs the per-component windows into one global
list (components were laid end-to-end in `av` by repeated `build_ranks`).

### 5.7 `cleanup2(g, nc, has_vlists)` (L832-869)

```
free TI_list (set NULL); free TE_list (set NULL)
if has_vlists:
    for c in 1..=GD_n_cluster(g): rec_reset_vlists(GD_clust(g)[c])    # §7.4
for r in minrank..=maxrank:
    for i in 0 ..< GD_rank(g)[r].n:
        v = GD_rank(g)[r].v[i]
        ND_order(v) = i                                        # final global numbering
        if ND_flat_out(v).list != NULL:
            j = 0
            while (e = ND_flat_out(v).list[j]) != NULL:
                if ED_edge_type(e) == FLATORDER:
                    delete_flat_edge(e); free(e->base.data); free(e)   # frees the FLATORDER virtual edges
                    j--                                             # re-examine same index
                j++
    free_matrix(GD_rank(g)[r].flat)
if Verbose: eprintf("mincross %s: %lld crossings, %.2f secs.\n", agnameof(g), nc, elapsed_sec())
```
(Only edges with type `FLATORDER` created by `ordered_edges` are destroyed;
`REVERSED`/label edges created by `flat_rev` are *not* freed here.)

---

## 6. Flat-edge machinery

Call sites: `mincross(g,0)` pass 0 → `flat_breakcycles` then `flat_reorder`;
pass 1 → `flat_reorder` only (matrix and ND_low persist from pass 0 —
`ND_low` is the per-node stable alias into the matrix, see §1.2/§6.1);
`mincross_clust` → both, before `mincross(g,2)`.

### 6.1 `flat_breakcycles(g)` (L1092-1117)

```
for r in minrank..=maxrank:
    flat = false
    for i in 0 ..< GD_rank(g)[r].n:
        v = rank_v[r][i]
        ND_mark(v) = false; ND_onstack(v) = false
        ND_low(v) = i                                  # stable per-rank alias (used by flatindex)
        if ND_flat_out(v).size > 0 and not flat:
            GD_rank(g)[r].flat = new_matrix(n, n)      # once per rank
            flat = true
    if flat:
        for i in 0 ..< n:
            v = rank_v[r][i]
            if not ND_mark(v): flat_search(g, v)       # left→right DFS
```

### 6.2 `flat_search(g, v)` (L1059-1090)

```
M = GD_rank(g)[ND_rank(v)].flat
ND_mark(v) = true; ND_onstack(v) = true
hascl = GD_n_cluster(dot_root(g)) > 0
i = 0
while (e = ND_flat_out(v).list[i]) != NULL:
    if hascl and not (agcontains(g, agtail(e)) and agcontains(g, aghead(e))):
        i++; continue                                  # edge of another cluster: ignore entirely
    if ED_weight(e) == 0: i++; continue                # non-constraint: ignore (kept in lists)
    if ND_onstack(aghead(e)):                          # cycle: break it
        matrix_set(M, flatindex(aghead(e)), flatindex(agtail(e)))   # M[head][tail]: reversed constraint
        delete_flat_edge(e); i--                       # list shrank; re-examine index i
        if ED_edge_type(e) == FLATORDER: i++; continue # no reverse edge created for FLATORDER
        flat_rev(g, e); i++
    else:
        matrix_set(M, flatindex(agtail(e)), flatindex(aghead(e)))   # M[tail][head]: freeze direction
        if not ND_mark(aghead(e)): flat_search(g, aghead(e))        # recurse (i unchanged)
        i++
ND_onstack(v) = false
```
(The `i--`/`i++` bookkeeping above is exactly C's `for (i = 0; (e = ...); i++)`
with `i--` before `continue`/`flat_rev` — a port must reproduce the *result*:
after deleting an edge, re-scan the same slot.)

### 6.3 `flat_rev(g, e)` (L1033-1057)

```
rev = first rev in ND_flat_out(aghead(e)).list with aghead(rev) == agtail(e) (NULL-terminated scan; loop breaks with rev set, or rev==NULL after end)
if rev != NULL:                                   # opposite edge already exists
    merge_oneway(e, rev)                          # folds weight/xpenalty/count into rev (fastgr.c:246+)
    if ED_edge_type(rev) == FLATORDER and ED_to_orig(rev) == NULL: ED_to_orig(rev) = e
    elist_append(e, ND_other(agtail(e)))
else:
    rev = new_virtual_edge(aghead(e), agtail(e), e)   # reversed virtual copy of e
    ED_edge_type(rev) = FLATORDER if ED_edge_type(e) == FLATORDER else REVERSED
    ED_label(rev) = ED_label(e)
    flat_edge(g, rev)                             # appends to ND_flat_out(new tail) & ND_flat_in(new head);
                                                  # sets GD_has_flat_edges(g) = true (fastgr.c:215-220)
```

### 6.4 `constraining_flat_edge(g, e)` (L1297-1305)

```
ED_weight(e) == 0                    → false
not inside_cluster(g, agtail(e))     → false
not inside_cluster(g, aghead(e))     → false
else true
```
`inside_cluster(g,v)` (L898-900) = `is_a_normal_node_of` (`ND_node_type ==
NORMAL && agcontains(g,v)`, L883-885) or `is_a_vnode_of_an_edge_of`
(`VIRTUAL && ND_in.size==1 && ND_out.size==1 && agcontains(g, original edge
found by walking ED_to_orig until type==NORMAL)`, L887-896).

### 6.5 `postorder(g, v, list, r)` (L1310-1325)

```
MARK(v) = true
for e in ND_flat_out(v).list (NULL-terminated):
    if not constraining_flat_edge(g, e): continue
    if not MARK(aghead(e)): postorder(g, aghead(e), list, r)
assert ND_rank(v) == r
LIST_APPEND(list, v)                    # append AFTER recursion (true post-order)
```

### 6.6 `flat_reorder(g)` (L1327-1402) — exact behavior, verified

```
if not GD_has_flat_edges(g): return
for r in minrank..=maxrank:
    if GD_rank(g)[r].n == 0: continue
    base_order = ND_order(GD_rank(g)[r].v[0])
    for i in 0..<n: MARK(rank_v[r][i]) = false
    clear temprank (LIST_CLEAR: size=0, capacity kept)

    valid_rank = true
    for i in 0 ..< GD_rank(g)[r].n:
        v = GD_flip(g) ? rank_v[r][i] : rank_v[r][n-1-i]     # flip: L→R scan; else R→L scan
        local_in_cnt  = count of e in ND_flat_in(v)  with constraining_flat_edge(g,e)
        local_out_cnt = count of e in ND_flat_out(v) with constraining_flat_edge(g,e)
        if local_in_cnt == 0 and local_out_cnt == 0: LIST_APPEND(temprank, v)
        else if not MARK(v) and local_in_cnt == 0:   postorder(g, v, temprank, r)
        else: valid_rank = false; break

    if valid_rank and temprank not empty:
        if not GD_flip(g): LIST_REVERSE(temprank)
        for i in 0 ..< n:
            v = rank_v[r][i] = temprank[i]
            ND_order(v) = i + base_order
        # "nonconstraint flat edges must be made LR"
        for i in 0 ..< n:
            v = rank_v[r][i]
            if ND_flat_out(v).list != NULL:
                j = 0
                while (e = ND_flat_out(v).list[j]) != NULL:
                    if (not GD_flip and ND_order(aghead(e)) < ND_order(agtail(e)))
                       or (GD_flip     and ND_order(aghead(e)) > ND_order(agtail(e))):
                        assert(not constraining_flat_edge(g, e))
                        delete_flat_edge(e); j--
                        flat_rev(g, e)
                    j++
    # else do no harm!
    GD_rank(Root)[r].valid = false          # ALWAYS, for every non-empty rank (L1399)
free temprank
```

**Empirically verified semantics** (standalone C replica of this exact scan +
`postorder`, exercised with: no edges / rightward chain / leftward chain /
star / interleaved leftward+rightward / disjoint edge pairs): because
`ND_flat_in` is symmetric with
`ND_flat_out` (`flat_edge`, fastgr.c:215-220) and neither the scan nor
`postorder` removes edges, *every* accept branch requires
`local_in_cnt == 0`. Hence:

1. If the rank contains **any** constraining flat edge, the head of that edge
   has `local_in_cnt ≥ 1` at its scan visit ⇒ `valid_rank = false` ⇒ the whole
   rank is untouched ("do no harm").
2. Consequently `postorder`'s output is **always discarded**: `postorder` only
   runs for a node with a constraining out-edge, whose head must later be
   scanned with `local_in_cnt ≥ 1` ⇒ bail. (Prove + replica agree.)
3. When there are **no** constraining edges on the rank, `temprank` ends up
   being the original sequence (R→L append + reverse = identity; flip: L→R
   append = identity), so the reorder itself is a no-op rewrite of `ND_order`.
4. The only real mutation is then the LR postpass: leftward-pointing
   *non-constraint* flat edges (`weight==0`, or endpoints not both inside `g` —
   e.g. cross-cluster edges when `g` is a cluster) are deleted and re-added in
   the opposite direction via `flat_rev` (type `REVERSED`, label copied).
5. `GD_rank(Root)[r].valid = false` executes unconditionally per non-empty
   rank — flat_reorder always flushes the crossing cache of every rank it
   visits.

A faithful port reproduces all of the above including the "useless" scan, or —
provably equivalently — only steps 4–5. Keeping the literal scan is
recommended for review parity.

---

## 7. Clusters

### 7.1 `mincross_clust(g)` (L537-561) — returns i64 (mincrossed crossings) or negative

```
if expand_cluster(g) != 0: return -1        # cluster.c:280: class2(subg); comp=1; allocate_ranks; build_ranks(subg,0);
                                            #   merge_ranks(subg); interclexp(subg); remove_rankleaders(subg)
ordered_edges(g)                            # per-cluster ordering= handling
flat_breakcycles(g)
flat_reorder(g)
nc = mincross(g, 2)                         # NOTE: startpass=2 — no build_ranks/flat_* inside; uses MaxIter passes
if nc < 0: return nc
for c in 1 ..= GD_n_cluster(g):             # depth-first over sub-clusters, index order
    mc = mincross_clust(GD_clust(g)[c])
    if mc < 0: return mc
    nc += mc
save_vlist(g)                               # remember first node of each cluster rank slice
return nc
```
Because `mincross(g,2)`'s objective `ncross()` scans **Root's** ranks, each
cluster pass minimizes whole-root crossings while only permuting the cluster's
own rank slices.

### 7.2 `left2right(g, v, w)` (L563-585) — may v,w not be exchanged?

```
if not ReMincross:
    if ND_clust(v) != ND_clust(w) and ND_clust(v) and ND_clust(w):
        if ND_ranktype(v) == CLUSTER and ND_node_type(v) == VIRTUAL: return false  # skeleton vnodes may swap
        if ND_ranktype(w) == CLUSTER and ND_node_type(w) == VIRTUAL: return false
        return true                                # different real clusters: never swap
else:                                              # remincross pass
    if ND_clust(v) != ND_clust(w): return true     # never interleave clusters after expansion
M = GD_rank(g)[ND_rank(v)].flat
if M == NULL: return false
if GD_flip(g): SWAP(v, w)
return matrix_get(M, flatindex(v), flatindex(w))   # frozen flat-edge orientation (§6.2)
```
Call invariant: `v` is at the smaller order (checked by callers'
`assert ND_order(v) < ND_order(w)`); with `GD_flip`, the matrix lookup is
mirrored.

### 7.3 vlist save/reset

* `save_vlist(g)` (L913-920): if `GD_rankleader(g)` allocated:
  `GD_rankleader(g)[r] = GD_rank(g)[r].v[0]` for r in minrank..=maxrank.
* `rec_save_vlists(g)` (L922-928): `save_vlist(g)` then recurse c=1..n_cluster
  (pre-order).
* `rec_reset_vlists(g)` (L930-953): recurse into sub-clusters **first**
  (post-order), then if rankleader allocated, for each rank:
  ```
  v = GD_rankleader(g)[r]; if v == NULL: continue
  u = furthestnode(g, v, -1); w = furthestnode(g, v, +1)
  GD_rankleader(g)[r] = u
  GD_rank(g)[r].v = GD_rank(dot_root(g))[r].v + ND_order(u)   # slice view into root array
  GD_rank(g)[r].n = ND_order(w) - ND_order(u) + 1
  ```
  (DEBUG guards L941-949: `node_in_root_vlist(v)` and the rank-array assert.)

### 7.4 `neighbor` / `furthestnode` (L871-881, L902-911)

```
neighbor(v, dir):                    # Root rank array!
    rv = NULL
    if dir < 0: if ND_order(v) > 0: rv = GD_rank(Root)[ND_rank(v)].v[ND_order(v)-1]
    else:           rv = GD_rank(Root)[ND_rank(v)].v[ND_order(v)+1]     # no upper bound check (L878)
    assert rv == 0 or (ND_order(rv) - ND_order(v)) * dir > 0
    return rv

furthestnode(g, v, dir):
    rv = v
    for u = v; (u = neighbor(u, dir)) != NULL; :
        if is_a_normal_node_of(g, u) or is_a_vnode_of_an_edge_of(g, u): rv = u
    return rv
```
`furthestnode` walks to the end of the rank and keeps the *last* node that
belongs to the cluster (real node or vnode of an edge of the cluster).

### 7.5 Rank filling: `realFillRanks` (L966-1001) / `fillRanks` (L1003-1008)

Guarantees every cluster has ≥1 node per rank by inserting tiny real nodes
(needed by the NEW_RANK flag path, `GD_flags(g) & NEW_RANK`).

```
realFillRanks(g, ranks: &bitarray, sg: subgraph|NULL) -> sg:
    for c in 1..=GD_n_cluster(g): sg = realFillRanks(GD_clust(g)[c], ranks, sg)   # children first
    if dot_root(g) == g: return sg                    # skip root (comment L962-964)
    bitarray_clear(ranks)                             # set all false
    for n in nodes of g:
        bitarray_set(ranks, ND_rank(n), true)
        for e in out-edges of n:
            for i in ND_rank(n)+1 ..= ND_rank(aghead(e)): bitarray_set(ranks, i, true)
    for i in GD_minrank(g) ..= GD_maxrank(g):
        if not bitarray_get(ranks, i):
            if sg == NULL: sg = agsubg(dot_root(g), "_new_rank", 1)   # collected under root for later removal
            n = agnode(sg, NULL, 1); agbindrec(n, "Agnodeinfo_t", sizeof(Agnodeinfo_t), true)
            ND_rank(n) = i; ND_lw(n) = ND_rw(n) = 0.5; ND_ht(n) = 1; ND_UF_size(n) = 1
            alloc_elist(4, ND_in(n)); alloc_elist(4, ND_out(n))
            agsubnode(g, n, 1)                        # member of the cluster
    return sg

fillRanks(g):
    rnks_sz = GD_maxrank(g) + 2
    rnks = bitarray_new(rnks_sz)
    realFillRanks(g, &rnks, NULL)
    bitarray_reset(&rnks)                              # frees
```

---

## 8. Edge ordering attributes & label ordering

### 8.1 `betweenclust(e)` (L434-438)

```
while ED_to_orig(e): e = ED_to_orig(e)
return ND_clust(agtail(e)) != ND_clust(aghead(e))    # 1 iff edge crosses a cluster boundary
```

### 8.2 `do_ordering_node(g, n, outflag)` (L440-477)

```
if ND_clust(n): return                    # cluster members are skipped
sortlist = TE_list                        # shared module scratch (size agnedges(root)+1)
ne = 0
for e in (outflag ? ND_out(n).list : ND_in(n).list) until NULL:
    if not betweenclust(e): sortlist[ne++] = e
if ne <= 1: return
sortlist[ne] = NULL                       # sentinel (requires the +1 allocation)
qsort(sortlist, ne, edgeidcmpf)           # ascending AGSEQ — declaration order
for idx in 1 ..< ne:                      # consecutive pairs
    e = sortlist[idx-1]; f = sortlist[idx]
    u = outflag ? aghead(e) : agtail(e)
    v = outflag ? aghead(f) : agtail(f)
    if find_flat_edge(u, v): return       # NB: RETURN — aborts the whole node on first hit (L471-472)
    fe = new_virtual_edge(u, v, NULL); ED_edge_type(fe) = FLATORDER
    flat_edge(g, fe)
```
`find_flat_edge(u,v)` (fastgr.c:56) searches `ND_flat_out(u)` for an edge whose
head is `v` **and** `ND_flat_in(v)` for an edge whose tail is `u` (directed
check).

### 8.3 `do_ordering` (L479-486), `do_ordering_for_nodes` (L488-504), `ordered_edges` (L512-535)

```
do_ordering(g, outflag): for n in agfstnode..agnxtnode(g): do_ordering_node(g, n, outflag)

do_ordering_for_nodes(g):                 # node "ordering" attribute
    for n in agfstnode..agnxtnode(g):
        ordering = late_string(n, N_ordering, NULL)
        if ordering: "out"→do_ordering_node(n,true); "in"→do_ordering_node(n,false);
                     else if ordering[0] != 0: agerror("ordering '%s' not recognized for node '%s'.\n", ordering, agnameof(n))

ordered_edges(g):                          # graph attribute dominates node attributes (comment L506-511)
    if G_ordering == NULL and N_ordering == NULL: return
    ordering = late_string(g, G_ordering, NULL)
    if ordering: "out"→do_ordering(g,true); "in"→do_ordering(g,false);
                 else if ordering[0]: agerror("ordering '%s' not recognized.\n", ordering)
    else:
        for subg in agfstsubg..agnxtsubg(g):          # depth-first: non-cluster children first
            if not is_a_cluster(subg): ordered_edges(subg)
        if N_ordering: do_ordering_for_nodes(g)
```
Called from `init_mincross` (root) and `mincross_clust` (each cluster).

### 8.4 `checkLabelOrder(g)` (L297-326) — public; called from `flat.c:332` (inside `flat_edges`, only when label nodes were created: `reset`)

```
lg = NULL
for r in minrank..=maxrank:
    rk = GD_rank(g) + r
    for j in 0 ..< rk.n:
        u = rk.v[j]
        if ND_alg(u) != NULL:                          # flat-edge label vnode (set in flat.c:181)
            if lg == NULL: lg = agopen("lg", Agstrictdirected, 0)
            n = agnode(lg, ITOS(j), 1); agbindrec(n, "info", sizeof(info_t), true)
            lo = ND_order(aghead(ND_out(u).list[0]))   # label vnode has exactly 2 flatorder out-edges
            hi = ND_order(aghead(ND_out(u).list[1]))
            if lo > hi: SWAP(lo, hi)
            ND_lo(n) = lo; ND_hi(n) = hi; ND_np(n) = u
    if lg:
        if agnnodes(lg) > 1: fixLabelOrder(lg, rk)
        agclose(lg); lg = NULL                       # fresh graph per rank
```

### 8.5 `fixLabelOrder(g, rk)` (L244-287)

`g` here is the synthetic label graph `lg`; `rk` the real rank row.

```
haveBackedge = false
for n in agfstnode(g), stepping nxtp = agnxtnode(g,n):     # nxtp is BOTH the outer stepper and the inner start
    v = nxtp = agnxtnode(g, n)
    for ; v; v = agnxtnode(g, v):                          # pairs (n, v) with v strictly after n
        if ND_hi(v) <= ND_lo(n): haveBackedge = true; agedge(g, v, n, NULL, 1)
        elif ND_hi(n) <= ND_lo(v):              agedge(g, n, v, NULL, 1)
if not haveBackedge: return

sg = agsubg(g, "comp", 1)
indices = calloc(agnnodes_z(g), int)
for n in agfstnode(g)..:
    if ND_x(n) or agdegree(g, n, 1, 1) == 0: continue       # visited or isolated
    if getComp(g, n, sg, indices) != 0:                     # component contains a "backedge" wrt current idx order
        sz_before = agnnodes_z(sg)                          # sampled BEFORE topsort (comment L272-274)
        arr = topsort(g, sg)
        assert LIST_SIZE(arr) == sz_before                  # containment DAG is acyclic ⇒ fully drained
        qsort(indices, LIST_SIZE(arr), ordercmpf)
        for i in 0 ..< LIST_SIZE(arr):
            ND_order(LIST_GET(arr, i)) = indices[i]
            rk.v[indices[i]] = LIST_GET(arr, i)             # splice back into the real rank row
        LIST_FREE(arr)
    emptyComp(sg)
free indices
```
`getComp(g, n, comp, indices)` (L221-241): recursive DFS over **both** in and
out edges of `g`; sets `ND_x(n)=1`; `indices[agnnodes(comp)] = ND_idx(n)`
(*before* `agsubnode(comp, n, 1)` — index = component size prior to insert);
counts `isBackedge(e) = ND_idx(head) > ND_idx(tail)` over all traversed edges
(both directions, L228-239); returns the count. `ND_idx(x) =
ND_order(ND_np(x))` reads the *live* `ND_order` of the real dummy node.

`topsort(g, sg)` (L204-219): repeatedly `findSource(g, sg)` = first sg-node
(in sg iteration order) with `agdegree(g, n, 1, 0) == 0` (no in-edges in the
**parent** graph); append `ND_np(n)`; `agdelnode(sg, n)`; delete all out-edges
of n **from g** (`agdeledge`); loop. Returns the appended list.

`emptyComp(sg)` (L181-189): delete every node from sg (`agnxtnode` pre-fetched).

Why the assert holds: an added edge `v→n` requires `hi(v) ≤ lo(n)`; the pair
loop adds an edge for *every* qualifying later node, so any cycle would force
all involved intervals to the same degenerate point, in which case the fan
edges make the graph acyclic. Port: replicate behavior; a Rust version may use
a plain Kahn topological sort with the same iteration source order
(`findSource` scans sg nodes in insertion order).

---

## 9. The mincross driver

### 9.1 `mincross(g, startpass)` (L692-753) — returns i64 (best crossings) or -1

```
endpass = 2
if startpass > 1:
    cur_cross = best_cross = ncross()
    save_best(g)
else:
    cur_cross = best_cross = INT64_MAX

for pass in startpass ..= endpass:
    if pass <= 1:
        maxthispass = min(4, MaxIter)
        if g == dot_root(g):
            if build_ranks(g, pass) != 0: return -1
        if pass == 0: flat_breakcycles(g)
        flat_reorder(g)
        cur_cross = ncross()
        if cur_cross <= best_cross: save_best(g); best_cross = cur_cross
    else:                                       # pass == 2
        maxthispass = MaxIter
        if cur_cross > best_cross: restore_best(g)
        cur_cross = best_cross

    trying = 0
    for iter in 0 ..< maxthispass:
        if Verbose: eprintf("mincross: pass %d iter %d trying %d cur_cross %lld best_cross %lld\n",
                            pass, iter, trying, cur_cross, best_cross)
        if trying++ >= MinQuit: break           # post-increment: compare old value (see below)
        if cur_cross == 0: break
        mincross_step(g, iter)
        cur_cross = ncross()
        if cur_cross <= best_cross:
            save_best(g)
            if (double)cur_cross < Convergence * (double)best_cross: trying = 0
            best_cross = cur_cross
    if cur_cross == 0: break

if cur_cross > best_cross: restore_best(g)
if best_cross > 0:
    transpose(g, false)                          # final polish, no tie swaps
    best_cross = ncross()
return best_cross
```

Exact semantics of `if (trying++ >= MinQuit) break;`: the comparison uses the
pre-increment value. With `MinQuit = 8` and no reset, the loop body executes
for `iter = 0..7` (8 body executions: old `trying` values 0..7 pass the check,
the value 8 breaks) — i.e. **at most `MinQuit` steps between "significant"
improvements**, where significant means `cur < 0.995 * best` (strict, double
multiply). Equal-cost results do *not* reset `trying`. Iterations and passes
interleave as above: for the root the pass sequence is 0, 1, 2; for clusters
and remincross it is only 2. `MAX phase iterations`: pass ≤ 1 runs at most
`min(4, MaxIter)` = 4 improvement steps; pass 2 at most `MaxIter` = 24 (scaled
by `mclimit`).

### 9.2 `save_best` (L772-781) / `restore_best` (L755-770)

```
save_best(g):
    for r in minrank..=maxrank:
        for i in 0 ..< GD_rank(g)[r].n:
            n = GD_rank(g)[r].v[i]
            saveorder(n) = ND_order(n)        # stash into ND_coord.x (see §1.1)

restore_best(g):
    for r in minrank..=maxrank:
        for i in 0 ..< GD_rank(g)[r].n:
            n = GD_rank(g)[r].v[i]; ND_order(n) = saveorder(n)
    for r in minrank..=maxrank:
        GD_rank(Root)[r].valid = false
        qsort(GD_rank(g)[r].v, GD_rank(g)[r].n, nodeposcmpf)   # re-sort row by restored order
```
For cluster graphs, the loops cover exactly the cluster's rank slices (views
into root arrays), so only the cluster's nodes are saved/restored.

### 9.3 `mincross_step(g, pass)` (L1455-1480)

```
reverse = (pass % 4) < 2                 # passes 0,1,4,5,…: true; 2,3,6,7,…: false

if pass % 2 == 0:                        # "down" pass — ranks ascending
    first = GD_minrank(g) + 1
    if GD_minrank(g) > GD_minrank(Root): first -= 1      # cluster deeper than root: include own top rank
    last = GD_maxrank(g); dir = +1
else:                                    # "up" pass — ranks descending
    first = GD_maxrank(g) - 1
    last = GD_minrank(g)
    if GD_maxrank(g) < GD_maxrank(Root): first += 1      # cluster shallower than root: include own bottom rank
    dir = -1

for r = first; r != last + dir; r += dir:
    other = r - dir
    hasfixed = medians(g, r, other)      # mval of rank r from rank `other` (the side we came from)
    reorder(g, r, reverse, hasfixed)
transpose(g, !reverse)                   # NOTE: transpose uses the OPPOSITE of reorder's `reverse`
```
So within a step: `reorder` swaps ties when `reverse == true`, and the
trailing `transpose` performs tie-swaps when `reverse == false`. `hasfixed`
comes from nodes whose fast-graph in/out degrees are both 0 (see §10.2).

### 9.4 `reorder(g, r, reverse, hasfixed)` (L1404-1453)

```
vlist = GD_rank(g)[r].v
ep = vlist + GD_rank(g)[r].n              # exclusive end (pointer)
changed = 0

for nelt in (0 .. GD_rank(g)[r].n-1) reversed:      # nelt = n-1 down to 0 — n outer rounds
    lp = vlist
    while lp < ep:
        while lp < ep and ND_mval(*lp) < 0: lp++     # skip fixed/noncomparable nodes ("find leftmost comparable")
        if lp >= ep: break
        sawclust = false; muststay = false
        rp = lp + 1
        while rp < ep:                               # find the next comparable node
            if sawclust and ND_clust(*rp): rp++; continue   # skip interior cluster nodes (### marker in source)
            if left2right(g, *lp, *rp): muststay = true; break
            if ND_mval(*rp) >= 0: break              # found comparable
            if ND_clust(*rp): sawclust = true
            rp++
        if rp >= ep: break                            # ends this outer round
        if not muststay:
            p1 = ND_mval(*lp); p2 = ND_mval(*rp)      # doubles
            if p1 > p2 or (p1 >= p2 and reverse):     # i.e. p1>p2 always swaps; p1==p2 swaps iff reverse
                exchange(*lp, *rp)                    # exchange positions lp and rp (not adjacent-swap semantics!)
                changed++
        lp = rp
    if not hasfixed and not reverse: ep--             # shrink window from the right each outer round

if changed:
    GD_rank(Root)[r].valid = false
    if r > 0: GD_rank(Root)[r-1].valid = false        # NB: r+1 is NOT invalidated here (asymmetry)
```
Exact tie-breaker summary: bubble-like passes that exchange the leftmost
comparable node `lp` with the next comparable node `rp` whenever
`mval(lp) > mval(rp)` (always) or `mval(lp) == mval(rp)` and `reverse`.
`mval == -1` nodes are never moved into comparison (they can be *skipped over*
— cluster nodes with `mval < 0` between two comparables get displaced, see the
`sawclust` handling).

### 9.5 `medians(g, r0, r1)` (L1614-1670) — returns `hasfixed`

```
list = TI_list                       # shared scratch (int), capacity agnedges(root)+1
v = GD_rank(g)[r0].v
hasfixed = false

for i in 0 ..< GD_rank(g)[r0].n:
    n = v[i]
    j = 0
    if r1 > r0:                      # r1 below r0 ⇒ use out-edges (downward median)
        for e in ND_out(n).list until NULL:
            if ED_xpenalty(e) > 0: list[j++] = VAL(aghead(e), ED_head_port(e))
    else:                            # r1 above (or same) ⇒ use in-edges
        for e in ND_in(n).list until NULL:
            if ED_xpenalty(e) > 0: list[j++] = VAL(agtail(e), ED_tail_port(e))
    match j:
      0 => ND_mval(n) = -1
      1 => ND_mval(n) = list[0]
      2 => ND_mval(n) = (list[0] + list[1]) / 2          # C int division (trunc; operands ≥ 0 ⇒ floor)
      _ => qsort(list, j, ordercmpf)                     # ascending
           if j odd:  ND_mval(n) = list[j/2]
           else:                          # weighted median (the old "wmed")
               rm = j/2; lm = rm - 1
               rspan = list[j-1] - list[rm]               # spread to the END of the value list
               lspan = list[lm]  - list[0]                # spread to the START
               if lspan == rspan: ND_mval(n) = (list[lm] + list[rm]) / 2   # int division again
               else:
                   w = list[lm]*(double)rspan + list[rm]*(double)lspan
                   ND_mval(n) = w / (lspan + rspan)        # REAL-valued median (double, no rounding)

for i in 0 ..< GD_rank(g)[r0].n:                            # second pass: isolated nodes
    n = v[i]
    if ND_out(n).size == 0 and ND_in(n).size == 0:          # fast-graph degrees (NOT flat lists)
        hasfixed |= flat_mval(n)
return hasfixed
```
`VAL(node, port) = MC_SCALE * ND_order(node) + port.order` (L1612) —
`MC_SCALE = 256`; values are ints (overflow only above ~8.3M-order ranks).
Note the medians do **not** average the two middle values for even `j ≥ 4`;
they use the span-weighted interpolation above, with the plain average only
when the two spans are equal. (This is the tie/`MEDIANB`-equivalent logic; see
§0.)

### 9.6 `flat_mval(n)` (L1583-1610) — returns "is fixed"

Precondition: fast-graph `ND_out(n).size == 0 && ND_in(n).size == 0` (node
touches no long edges).

```
if ND_flat_in(n).size > 0:
    nn = agtail(fl[0])                                    # fl = ND_flat_in(n).list
    for i = 1; (e = fl[i]); i++: if ND_order(agtail(e)) > ND_order(nn): nn = agtail(e)
    if ND_mval(nn) >= 0: ND_mval(n) = ND_mval(nn) + 1; return false
elif ND_flat_out(n).size > 0:
    nn = aghead(fl[0])
    for i = 1; (e = fl[i]); i++: if ND_order(aghead(e)) < ND_order(nn): nn = aghead(e)
    if ND_mval(nn) > 0: ND_mval(n) = ND_mval(nn) - 1; return false
return true                                                # mval stays -1 ⇒ "fixed" for reorder
```
Asymmetries to preserve: in-edge branch uses `>= 0` and `+1`; out-edge branch
uses `> 0` and `-1`; "largest-order" in-tail vs "smallest-order" out-head.
One-step propagation only (no transitive closure).

---

## 10. Virtual-node edge weights (L1696-1732)

```c
#define ORDINARY 0
#define SINGLETON 1
#define VIRTUALNODE 2
#define NTYPES 3
#define C_EE 1
#define C_VS 2
#define C_SS 2
#define C_VV 4
static const int table[NTYPES][NTYPES] = {
    /* ordinary */  {C_EE, C_EE, C_EE},
    /* singleton */ {C_EE, C_SS, C_VS},
    /* virtual  */  {C_EE, C_VS, C_VV}};
```

```
endpoint_class(n):                       # L1712-1718
    ND_node_type(n) == VIRTUAL      → VIRTUALNODE
    ND_weight_class(n) <= 1         → SINGLETON
    else                            → ORDINARY

virtual_weight(e):                       # L1720-1732 — public (called from rank.c)
    t = table[endpoint_class(agtail(e))][endpoint_class(aghead(e))]
    assert t >= 0
    if INT_MAX / t < ED_weight(e): agerror("overflow when calculating virtual weight of edge\n"); graphviz_exit(EXIT_FAILURE)
    ED_weight(e) *= t
```
(`ND_weight_class` is capped at 3 by class2.c:166-171: `if (<=2) ++` per
incident edge endpoint.)

---

## 11. Debug-only helpers (compiled out unless `DEBUG`)

* `check_order()` (L1735-1747): for every root rank, `assert(rank.v[rank.n] ==
  NULL)` and per-node `ND_rank == r`, `ND_order == i`.
* `check_vlists(g)` (L1766-1784): recursive; per rank asserts
  `GD_rank(Root)[r].v[ND_order(u)] == u` for every row entry and for the
  rankleader.
* `node_in_root_vlist(n)` (L1786-1794): linear scan of the root row; `abort()`
  if absent.

---

## 12. Cross-cutting porting checklist / gotchas

1. **`saveorder` aliases `ND_coord.x`** (L114). In Rust use a separate field;
   it is only live between `save_best` and `restore_best`/`cleanup2`.
2. **Rank array windows**: root `v` is advanced per component (§5.1) and reset
   by `merge2` (§5.6); cluster rows are views into root rows (§7.1, §7.3).
   `exchange`/`neighbor`/`transpose_step`/`reorder` must agree on which view
   they index. In Rust, model `rank.v` as `(row_start_offset, len)` or as
   index ranges rather than raw pointers, but keep `exchange` writing through
   the **Root** row and `ND_order` globally consistent.
3. **`out_cross` returns i32, `in_cross` i64** (L587 vs L606). Keep the
   asymmetry; `transpose_step`'s i64 sums then behave identically.
4. **Invalidation asymmetry**: `reorder` invalidates `Root[r]` and `Root[r-1]`
   only (L1449-1451); `transpose_step` invalidates `r-1, r, r+1` (L660-669);
   `flat_reorder` invalidates every non-empty row (L1399); `build_ranks`
   invalidates all rows (L1259). `ncross()` reads rows `minrank ..< maxrank`.
5. **Iteration directions**: `reorder` scans left→right within `ep`, outer
   rounds `n-1..0`, window shrink only when `!hasfixed && !reverse`;
   `transpose_step` scans left→right; `medians` scans rows left→right;
   `flat_reorder` scans right→left (flip: left→right); `build_ranks` walks the
   nlist **backwards for clusters**; `mincross_step` sweeps ranks down (even
   pass) / up (odd pass) with the documented first/last adjustments.
6. **Tie-breakers** (must match exactly):
   * `transpose_step`: swap iff `c1 < c0 || (c0 > 0 && reverse && c1 == c0)`.
   * `reorder`: exchange iff `p1 > p2 || (p1 >= p2 && reverse)`.
   * `in_cross`/`out_cross` port ties: strict `>` on port `.p.x`.
   * `medians`: j=2 average truncates; even-j uses span-weighted real value;
     sort ascending; odd picks `list[j/2]`.
   * `flat_mval`: `>=0 → +1` vs `>0 → -1`.
   * `do_ordering_node`: edges sorted by `AGSEQ`; first existing flat edge
     aborts the node's chain (`return`, not `continue`).
7. **Determinism**: no RNG; all `qsort` uses are equivalence-class safe (§3).
   Output depends on: cgraph node/edge creation order (AGSEQ, agfstnode order),
   `decompose` component list order, cluster index order, and `mclimit`.
8. **Verbose messages** (byte-exact, to stderr):
   * `"mincross: pass %d iter %d trying %d cur_cross %" PRId64 " best_cross %" PRId64 "\n"` (L726-729)
   * `"merge2: graph %s, rank %d has only %d < %d nodes\n"` (L822-823)
   * `"mincross %s: %" PRId64 " crossings, %.2f secs.\n"` (L867-868)
   * `"removing empty cluster\n"` warning (L345)
   * `"ordering '%s' not recognized.\n"` / `"ordering '%s' not recognized for node '%s'.\n"` (L500-501, L523)
   * `install_in_rank` errors (L1156-1157, L1176-1178, L1182-1183, L1187-1189)
   * `"overflow when calculating virtual weight of edge\n"` + exit(1) (L1727-1728)
9. **Failure propagation**: `mincross` returns -1 only from `build_ranks`
   failures (root only); `mincross_clust` from `expand_cluster` or nested
   mincross; `dot_mincross` maps any negative to `rc=-1`, still running
   `cleanup2` with `has_set_vlists` as of the failure point.
10. **`transpose` calls**: `build_ranks` (root only, when `ncross() > 0`,
    reverse=false), end of every `mincross_step` with `!reverse` of the step,
    and once at the end of `mincross` (reverse=false) iff `best_cross > 0`.
    `transpose(g, true)` therefore happens exactly at the tail of `mincross_step`
    for `pass % 4 ∈ {2, 3}`.
11. **Flat-order edges' lifecycle**: created by `do_ordering_node` (type
    FLATORDER), possibly reversed by `flat_breakcycles`/`flat_reorder`
    (`flat_rev` creates type REVERSED or FLATORDER), deleted+freed in
    `cleanup2` only when type == FLATORDER.
12. `decompose(g, 1)` (decomp.c:108+) fills `GD_comp(g).list` in discovery
    order: outer loop over `agfstnode` order, DFS with an explicit node stack
    (`search_component`); pass=1 redirects cluster members to their rankleader
    before component search. Component order directly affects
    `init_mccomp`/`merge2` placement, hence output order.

---

## 13. Call graph (mincross.c only)

```
dot_mincross
├── init_mincross ── mincross_options, (fillRanks→realFillRanks), class2*, decompose*,
│                    allocate_ranks, ordered_edges
├── init_mccomp → mincross(g,0)
│     ├── build_ranks ── install_in_rank, install_cluster*, enqueue_neighbors, ncross, transpose
│     ├── flat_breakcycles ── flat_search ── flat_rev (merge_oneway*, new_virtual_edge*, flat_edge*)
│     ├── flat_reorder ── constraining_flat_edge, postorder, delete_flat_edge*, flat_rev
│     ├── ncross ── rcross ── local_cross
│     ├── save_best / restore_best
│     └── mincross_step ── medians (flat_mval), reorder (left2right, exchange), transpose
│                                                         └─ transpose_step ── in_cross/out_cross
├── merge2 ── merge_components
├── mincross_clust (per cluster, recursive)
│     ├── expand_cluster*, ordered_edges, flat_breakcycles, flat_reorder
│     ├── mincross(g,2)     (same machinery, pass 2 only)
│     └── save_vlist
├── mincross(g,2)  (remincross; ReMincross=true changes left2right)
└── cleanup2 ── rec_reset_vlists (recursive; furthestnode/neighbor)
(checkLabelOrder — defined here, called from flat.c flat_edges → fixLabelOrder → getComp/topsort/emptyComp)
```
(`*` = defined in other files: class2.c, decomp.c, cluster.c, fastgr.c.)

## 14. Verified behavioral summary (the two surprising ones)

* **`flat_reorder`** never reorders a rank that contains a constraining flat
  edge, and performs an identity permutation otherwise; its observable effects
  are (a) flipping leftward non-constraining flat edges to point LR via
  `flat_rev`, and (b) invalidating every visited rank's crossing cache
  (§6.6). Confirmed with a line-faithful replica of the scan under edge
  configurations: none / rightward chain / leftward chain / star / interleaved.
* **`mincross` pass budget**: root-level runs passes 0,1,2; per-pass improvement
  loops capped at `min(4, MaxIter)` and `MaxIter` respectively; `MinQuit=8`
  (post-increment comparison ⇒ ≤ 8 steps per stretch); improvement threshold
  `cur < 0.995 * best` (double). Clusters and the remincross run execute pass 2
  only, with `MaxIter` steps.
