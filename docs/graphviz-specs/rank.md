# Graphviz `dot` — Rank Assignment (`lib/dotgen/rank.c`) and Decomposition (`lib/dotgen/decomp.c`)

**Exhaustive implementation spec for a faithful Rust port.**

* Source under spec: `/tmp/graphviz-src/lib/dotgen/rank.c` (1113 lines) and `/tmp/graphviz-src/lib/dotgen/decomp.c` (130 lines), read in full.
* Source tree identity: Graphviz git master, commit `2e92c7f2776f6611dd78fa83c213ecc65cc47bfa`.
* All citations `rank.c:L` / `decomp.c:L` refer to those two files; other files are cited as `path:L` relative to `/tmp/graphviz-src/lib/`.
* Companion spec already in this directory: [dotsplines.md](dotsplines.md).

---

## 0. Scope corrections — what is *not* in these files

The rank-assignment request mentions several symbols that **do not exist in this version of the code**. A port must not invent them. Verified by tree-wide grep:

| Requested symbol | Reality in this tree |
|---|---|
| `init_ufigraph` | Does not exist anywhere. No "ufigraph" concept survives. |
| `rank2_create_rank_sets` / `dot2_create_rank_sets` | Does not exist. The dot2 (`NEW_RANK`) path builds rank sets via `compile_samerank` (rank.c:636-706). |
| `tight_tree`, "tight init BFS from virtual source" | Gone. Ranking does **not** do a tight-tree/BFS init. The fast graph is handed directly to the network simplex (`rank()` / `rank2()` in `common/ns.c`). (The simplex internally has `init_rank` BFS — ns.c:970 — but that is ns-internal.) |
| `build_ranks`, `allocate_ranks`, `install_in_rank`, `enqueue_neighbors`, `node_queue_t` | **Moved out of ranking.** They now live in `dotgen/mincross.c:1122-1295` (ordering phase) and `dotgen/dotprocs.h:22-52`. `node_queue_t` is `typedef LIST(Agnode_t *) node_queue_t;` (dotprocs.h:22). Documented in §11 because the port still needs them — but for mincross, not for `dot_rank`. |
| auxiliary graph `g2` built for network simplex | **No such graph in the dot1 path.** The simplex runs directly on the "fast graph": `GD_nlist` + `ND_out`/`ND_in` elists + `ED_minlen`/`ED_weight`. An auxiliary graph *is* built in the dot2 path: `Xg` ("level assignment constraints", rank.c:1084). |
| `CL_BACK` usage in rank.c | `CL_BACK` is defined in `common/const.h:141` (`#define CL_BACK 10`) and used in `dotgen/class1.c:59` inside `interclust1`, not in rank.c itself. |
| `RANKSTEP` | Does not exist anywhere in the tree. |
| global `MaxIter` | `common/globals.h:63` declares it, but it is a **neato** global (`neatogen/neatoinit.c:1281-1287`); dot ranking uses a **local** `maxiter` (`INT_MAX` unless `nslimit1`, rank.c:455-459, rank.c:1079-1093). mincross separately sets `MaxIter = 24` (mincross.c:1756) for its own iteration cap. |
| `SCOMP` | Not in decomp.c (or anywhere in dotgen). The only `scomp` is an unrelated qsort comparator in `neatogen/adjust.c:164`. |
| `ND_weight_class` | Field exists (`common/types.h:535`) but is **not touched** by rank.c/decomp.c; it is maintained in class2/position phases. |
| "final order of nodes within each rank before mincross" | rank.c assigns only `ND_rank` values. Per-rank arrays and `ND_order` are built **inside mincross init** (`allocate_ranks` + `build_ranks`, mincross.c:1026-1030, 1122-1273) and per-cluster in `expand_cluster` (`dotgen/cluster.c:281-297`). Summarized in §11. |

---

## 1. Data-model reference (everything these files touch)

### 1.1 Lists (common/types.h:246-272)

```c
typedef struct nlist_t { node_t **list; size_t size; } nlist_t;   // types.h:246-249
typedef struct elist  { edge_t **list; size_t size; }  elist;     // types.h:251-254
```

Invariants relied on by the port:

* `elist_append(item, L)` (types.h:261-266): `L.list = recalloc(L.list, size+1, size+2, ptr)`; `L.list[L.size++] = item; L.list[L.size] = NULL;` — i.e. **a NULL sentinel always follows the last element**, and lists are grown with zero-fill.
* `alloc_elist(n, L)` (types.h:267-271): `L.size = 0; L.list = calloc(n + 1, ptr)`.
* `free_list(L)` (types.h:272): `free(L.list)` only.
* `GD_comp(g)` is an `nlist_t` (types.h:321-322, 362): `node_t **list; size_t size;` where `list[c]` is the **head node of component c** in the `ND_next`-linked fast node list.
* `util/list.h` LIST container (`LIST(edge_t *)`, `LIST(node_t *)`): dynamic array with `LIST_APPEND` (push_back), `LIST_PUSH_BACK`, `LIST_POP_BACK`, `LIST_POP_FRONT`, `LIST_IS_EMPTY`, `LIST_SIZE`, `LIST_GET(i)`, `LIST_SORT(cmp)`, `LIST_FREE`. FIFO = push_back + pop_front (mincross). LIFO stack = push_back + pop_back (decomp.c:63-75).
* `gv_recalloc(p, old_n, new_n, sz)`, `gv_calloc`, `gv_alloc`: zero-filled (re)allocations.

### 1.2 Node fields used (common/types.h:410-538, macros at 483-538)

| Field | Type | Meaning / use here |
|---|---|---|
| `ND_rank(n)` | `int` | assigned rank (types.h:464, 523) |
| `ND_ranktype(n)` | `char` | collapsed-node classification, one of §1.4 (types.h:448, 524) |
| `ND_node_type(n)` | `char` | `NORMAL`/`VIRTUAL`/`SLACKNODE`/… (types.h:445, 511) |
| `ND_mark(n)` | `size_t` | visited mark; compared against `Cmark` counters (types.h:446, 507) |
| `ND_onstack(n)` | `char` | DFS recursion-stack flag (rank.c dot2 `dfs`) (types.h:447, 512) |
| `ND_next(n)`, `ND_prev(n)` | `node_t*` | fast node list doubly links (types.h:450-451, 510, 521) |
| `ND_in(n)`, `ND_out(n)` | `elist` | fast-graph in/out edge lists (types.h:452-453) |
| `ND_flat_in(n)`, `ND_flat_out(n)` | `elist` | flat-edge lists, scanned by decompose (types.h:454-455) |
| `ND_UF_parent(n)`, `ND_UF_size(n)` | `node_t*`, `int` | classic dot union-find (types.h:460-461) |
| `ND_set(n)` | `node_t*` | *second*, independent union-find used only by the dot2 path (types.h:442, 486) |
| `ND_rep(n)` | `node_t*` | dot2: mapping original node → Xg representative node (types.h:441, 496) |
| `ND_clust(n)` | `graph_t*` | innermost cluster containing n (types.h:457, 489) |
| `ND_hops(n)` | `int` | **overloaded as `ND_comp(n)` in the dot2 path** (rank.c:555) — connected-component id of Xg nodes |
| `ND_alg(n)` | `void*` | freed/NULLed in `readout_levels` (rank.c:1009-1012) |

### 1.3 Graph fields used (common/types.h:278-349, macros at 351-399)

| Field | Type | Use here |
|---|---|---|
| `GD_nlist(g)` | `node_t*` | head of fast node list (`ND_next`/`ND_prev` chain); per-component when iterating `GD_comp` |
| `GD_comp(g)` | `nlist_t` | connected-component heads (built by `decompose`) |
| `GD_minrank(g)`, `GD_maxrank(g)` | `int` | rank bounds; init `INT_MAX` / `-1` |
| `GD_minset(g)`, `GD_maxset(g)` | `node_t*` | UF leaders of min-/max-rank sets (dot1 path) |
| `GD_minrep(g)`, `GD_maxrep(g)` | `node_t*` | dot2 counterparts (per cluster) |
| `GD_leader(g)` | `node_t*` | cluster representative node |
| `GD_n_cluster(g)`, `GD_clust(g)` | `int`, `graph_t**` | cluster array, valid indices **1..n_cluster** |
| `GD_parent(g)` | `graph_t*` | containing cluster graph |
| `GD_level(g)` | `int` | cluster nesting depth (dot2) |
| `GD_set_type(g)` | `char` | result of `rank_set_class` (§4.6) |
| `GD_ranksep(g)` | `int` | vertical separation; halved when edge labels present |
| `GD_has_labels(g)` | `unsigned char` | bitfield; `EDGE_LABEL = 1<<0` (const.h:167) |
| `GD_flags(g)` | `unsigned short` | `NEW_RANK = 1<<4` (const.h:227-243) |
| `GD_rankleader(g)` | `node_t**` | per-rank representative nodes of a collapsed cluster (used by `decompose(...,pass=1)`) |
| `GD_rank(g)` | `rank_t*` | per-rank arrays (mincross; only read by `install_in_rank`) |
| `GD_installed(g)` | `char` | used by `install_cluster` (cluster.c:385) |

### 1.4 Enumerations (common/const.h — all verbatim)

```c
/* node types */                      /* collapsed node classifications (rankset kinds) */
#define NORMAL       0                #define NOCMD      0   /* default */
#define VIRTUAL      1                #define SAMERANK   1   /* place on same rank */
#define SLACKNODE    2                #define MINRANK    2   /* place on "least" rank */
#define REVERSED     3                #define SOURCERANK 3   /* strict version of MINRANK */
#define FLATORDER    4                #define MAXRANK    4   /* place on "greatest" rank */
#define CLUSTER_EDGE 5                #define SINKRANK   5   /* strict version of MAXRANK */
#define IGNORED      6                #define LEAFSET    6   /* set of collapsed leaf nodes */
                                      #define CLUSTER    7   /* set of clustered nodes */
```
(const.h:24-30 and const.h:33-40)

```c
/* type of cluster rank assignment */  /* GD_flags */
#define LOCAL  100                    /* bits 1-3: EDGETYPE_*; bit 4: */
#define GLOBAL 101                    #define NEW_RANK (1 << 4)
#define NOCLUST 102                   /* const.h:227-243 */
```
(const.h:43-45)

`CL_type` is a process global (common/globals.h:61) set from the graph attribute `clusterrank` (common/input.c:604-605, 698-699):

```c
static char *rankname[] = { "local", "global", "none", NULL };
static int   rankcode[] = { LOCAL,  GLOBAL,  NOCLUST, LOCAL };
CL_type = maptoken(p, rankname, rankcode);
```
`maptoken` (common/utils.c:315-323) scans `name[i]` until the NULL terminator; if the value `p` is NULL or unmatched it returns the **last** entry, so the default `CL_type` is `LOCAL`.

### 1.5 Edge fields used

`ED_minlen` (int), `ED_weight` (int), `ED_to_virt(e)` (fast edge standing for original `e`), `ED_to_orig(e)` (original edge a fast edge stands for), `ED_count`, `ED_xpenalty` (common/types.h:543-626).

### 1.6 Union-find primitives (common/utils.c:105-148)

```c
node_t *UF_find(node_t *n) {            // iterative, path-halving
    while (ND_UF_parent(n) && ND_UF_parent(n) != n) {
        if (ND_UF_parent(ND_UF_parent(n))) ND_UF_parent(n) = ND_UF_parent(ND_UF_parent(n));
        n = ND_UF_parent(n);
    }
    return n;
}
node_t *UF_union(node_t *u, node_t *v) {
    if (u == v) return u;
    if (!ND_UF_parent(u)) { ND_UF_parent(u) = u; ND_UF_size(u) = 1; } else u = UF_find(u);
    if (!ND_UF_parent(v)) { ND_UF_parent(v) = v; ND_UF_size(v) = 1; } else v = UF_find(v);
    if (u == v) return u;
    if (ND_id(u) > ND_id(v)) { ND_UF_parent(u) = v; ND_UF_size(v) += ND_UF_size(u); }
    else                     { ND_UF_parent(v) = u; ND_UF_size(u) += ND_UF_size(v); v = u; }
    return v;                            // leader = node with SMALLER ND_id
}
void UF_singleton(node_t *u) { ND_UF_size(u) = 1; ND_UF_parent(u) = NULL; ND_ranktype(u) = NORMAL; }
```
**Tie-breaker:** the union leader is always the node with the smaller `ND_id` (cgraph creation sequence). `UF_setname(u,v)` (utils.c:150-155): asserts `u == UF_find(u)`, sets `ND_UF_parent(u)=v`, adds sizes.

---

## 2. Constants verbatim

| Constant | Value | Defined | Used |
|---|---|---|---|
| `CL_BACK` | `10` — "cost of backward pointing edge" | const.h:141 | class1.c:59 (`interclust1` aux-edge weight multiplier) |
| `CL_CROSS` | `1000` (non-Windows) / `100` (Windows, "avoid 16 bit overflow") | const.h:143-147 | not used by ranking |
| `NORMAL` | `0` | const.h:24 | rank.c:311, 321; decomp via node types |
| `VIRTUAL` | `1` | const.h:25 | fastgraph virtual chain nodes |
| `SLACKNODE` | `2` | const.h:26 | rank.c:98 (cleanup), class1.c:56 (interclust1 slack nodes) |
| `NOCMD`/`SAMERANK`/`MINRANK`/`SOURCERANK`/`MAXRANK`/`SINKRANK`/`LEAFSET`/`CLUSTER` | `0/1/2/3/4/5/6/7` | const.h:33-40 | rank.c throughout |
| `LOCAL`/`GLOBAL`/`NOCLUST` | `100/101/102` | const.h:43-45 | `CL_type` (rank.c:344,361,497) |
| `EDGE_LABEL` | `1 << 0` | const.h:167 | rank.c:176 |
| `NEW_RANK` | `1 << 4` | const.h:243 | rank.c:530 |
| `BACKWARD_PENALTY` | `1000` | rank.c:545 | `weak()` (rank.c:805) |
| `STRONG_CLUSTER_WEIGHT` | `1000` | rank.c:546 | `compile_clusters` (rank.c:871) |
| `NORANK` | `6` | rank.c:547 | `rankset_kind` default (rank.c:590) |
| `ROOT` | `"\177root"` | rank.c:548 | `connect_components` Xg node name |
| `TOPNODE` | `"\177top"` | rank.c:549 | `compile_clusters` |
| `BOTNODE` | `"\177bot"` | rank.c:550 | `compile_clusters` |
| `SEARCHSIZE` | `30` (`enum { SEARCHSIZE = 30 };`) | ns.c:55 | default simplex search size |
| `maxiter` default | `INT_MAX` | rank.c:455, 1079, 1093 | ns iteration cap |
| `alloc_elist(4, …)` | 4 (+1 sentinel) slots | rank.c:736-737 | `makeXnode` in/out lists |
| weak-edge name format | `"_weak_%d"`, `buf[100]`, `id` is a function-static counter starting at 0, incremented per created weak node | rank.c:788, 799 | `weak()` |
| `infosizes[]` | `{sizeof(Agraphinfo_t), sizeof(Agnodeinfo_t), sizeof(Agedgeinfo_t)}` | rank.c:1071-1075 | cgraph rec sizes for Xg |

Attribute strings read by these paths: `newrank` (rank.c:529), `nslimit1` (rank.c:458, 1090), `searchsize` (rank.c:1103; also ns.c:1034), `rank` (rank.c:227-234, 576-589), `compact` (rank.c:570), `constr` (rank.c:597, class1.c:25), `clusterrank` (input.c:698).

---

## 3. `rank.c` — the classic path (`dot1_rank`)

### 3.1 File-level statics and helpers

```c
static void dot1_rank(graph_t *g);      // rank.c:38
static void dot2_rank(graph_t *g);      // rank.c:39
typedef LIST(edge_t *) edge_set_t;      // rank.c:41
```

### 3.2 `renewlist(elist *L, edge_set_t *track)` — rank.c:45-53

```c
for (size_t i = L->size; i != SIZE_MAX; i--) {   // NB: starts at L->size, NOT size-1
    if (track != NULL && L->list[i] != NULL) LIST_APPEND(track, L->list[i]);
    L->list[i] = NULL;
}
L->size = 0;
```

Pseudocode / port notes:

```
fn renewlist(L, track: Option<&mut Vec<EdgePtr>>):
    i = L.size                      # first index processed is L.size — the NULL
    while i != SIZE_MAX:            # sentinel slot (harmless: entry is NULL)
        if track and L[i] != null: track.append(L[i])
        L[i] = null
        i -= 1
    L.size = 0
```

* Entries are appended to `track` in **reverse index order** (size-1 … 0).
* The backing array is *not* freed here; only entries are nulled and `size` reset (arrays of real nodes are reused by later phases; only slack nodes have their arrays explicitly freed, §3.4).

### 3.3 `edge_ptr_cmp(const void *x, const void *y)` — rank.c:66-78

Comparator over `edge_t *` **addresses** (`uintptr_t`), returning -1/0/1. Used only to co-locate duplicates before deduplicating frees in `cleanup1`. Comment (rank.c:55-61) stresses that pointer ordering carries no semantic meaning — a Rust port should instead use an insertion-sequence id or a hash set; behavior (set of freed edges) is identical.

### 3.4 `cleanup1(graph_t *g)` — rank.c:80-164

Tears down the fast graph built for ranking. Pseudocode:

```
to_free = []                                   # edge_set_t
for c in 0 .. GD_comp(g).size - 1:             # rank.c:87
    GD_nlist(g) = GD_comp(g).list[c]           # rank.c:88  (component head)
    n = GD_nlist(g); prev = None
    while n:                                   # rank.c:89-115
        next = ND_next(n)
        renewlist(&ND_in(n),  None)            # in-lists are not owning
        renewlist(&ND_out(n), &to_free)        # out-lists own the fast edges
        ND_mark(n) = false
        if ND_node_type(n) == SLACKNODE:       # slack nodes exist ONLY here
            if prev == None:
                GD_comp(g).list[c] = next      # unlink from component head
                GD_nlist(g) = next
            else:
                ND_next(prev) = next
            if next: ND_prev(next) = prev
            free_list(ND_in(n)); free_list(ND_out(n))   # only place arrays are freed
            free(n.base.data); free(n)
            # prev is NOT advanced (stays pointing before the removed node)
        else:
            prev = n
        n = next

for n in real nodes of g (agfstnode/agnxtnode):        # rank.c:117-128
    for e in out-edges of n (agfstout/agnxtout):
        f = ED_to_virt(e)
        if f && e != ED_to_orig(f):            # parallel multi-edges share a virtual edge;
            ED_to_virt(e) = NULL               # null the aliasing references so each
                                               # virtual edge is freed exactly once
for n in real nodes of g:                              # rank.c:129-137
    for e in out-edges of n:
        f = ED_to_virt(e)
        if f && ED_to_orig(f) == e:            # e owns f
            LIST_APPEND(&to_free, f)
            ED_to_virt(e) = NULL

LIST_SORT(&to_free, edge_ptr_cmp)              # rank.c:143
previous = None
for current in to_free:                        # rank.c:145-154
    if current != previous:
        if previous != None: free(previous.base.data)
        free(previous)                         # free(NULL) is a no-op on the first iter
        previous = current
if previous != None: free(previous.base.data)  # rank.c:155-157
free(previous)                                 # frees the last unique edge
LIST_FREE(&to_free)

free(GD_comp(g).list)                          # rank.c:161-163
GD_comp(g).list = NULL; GD_comp(g).size = 0
```

Semantics/quirks to reproduce:

1. Component lists are walked head-by-head; `GD_nlist(g)` is repointed per component (callers must not rely on `GD_nlist` after this).
2. Slack nodes (`ND_node_type == SLACKNODE`, created by `interclust1`, class1.c:55-56) are deallocated here; all other nodes survive.
3. Real nodes' elist arrays survive (only contents cleared); slack-node arrays are freed with the node.
4. The `to_free` collection can contain duplicates (both loops append; also parallel edges), hence the sort-dedupe. Net effect: every unique collected fast edge gets `free(base.data); free(edge)` exactly once.
5. `GD_comp` list memory is freed and the nlist_t reset (but `decompose` will later recalloc it; `dot_cleanup` also frees `GD_comp` via `free_list(GD_comp(g))`, dotinit.c:166).

### 3.5 `edgelabel_ranks(graph_t *g)` — rank.c:170-182

```
if GD_has_labels(g) & EDGE_LABEL:
    for n in real nodes of g:
        for e in out-edges of n:
            ED_minlen(e) *= 2                  # reserve a rank for the label vnode
    GD_ranksep(g) = (GD_ranksep(g) + 1) / 2    # integer division; compensate separation
```
Comment (rank.c:166-169): "When there are edge labels, extra ranks are reserved here for the virtual nodes of the labels. This is done by doubling the input edge lengths. The input rank separation is adjusted to compensate." Called from both `dot1_rank` (rank.c:512) and `dot2_rank` (rank.c:1088).

### 3.6 `rank_set_class(graph_t *g)` — rank.c:224-237

```c
static char *name[]  = { "same", "min", "source", "max", "sink", NULL };
static int   class[] = { SAMERANK, MINRANK, SOURCERANK, MAXRANK, SINKRANK, 0 };
if (is_a_cluster(g)) return CLUSTER;
val = maptoken(agget(g, "rank"), name, class);
GD_set_type(g) = val;
return val;
```

* `is_a_cluster(g)` (utils.c:684-687): `g == g->root || strncasecmp(agnameof(g), "cluster", 7) == 0 || mapbool(agget(g, "cluster"))`.
* `maptoken` returns `0` (`NOCMD`) for a missing/unknown `rank` value (NULL string skips to the terminator entry).
* Side effect: records the classification in `GD_set_type` even for ranksets.

### 3.7 `make_new_cluster(graph_t *g, graph_t *subg)` → `int` — rank.c:239-249

```
cno = ++GD_n_cluster(g)
GD_clust(g) = recalloc(GD_clust(g), old = cno, new = cno + 1, sizeof(ptr))
GD_clust(g)[cno] = subg
do_graph_label(subg)             # common/input.c:830; sets subg's label, may set GD_has_labels |= GRAPH_LABEL
return cno
```
Cluster arrays are indexed **1..GD_n_cluster**; index 0 is unused.

### 3.8 `node_induce(graph_t *par, graph_t *g)` — rank.c:251-279

```
# pass 1 — enforce "node in at most one cluster at this level"  (rank.c:258-271)
n = agfstnode(g)
while n:
    nn = agnxtnode(g, n)                  # saved BEFORE possible agdelete
    if ND_ranktype(n) != 0:               # already in a rankset (same/min/max/…)
        agdelete(g, n); n = nn; continue
    i = 1
    while i < GD_n_cluster(par):          # NB: excludes index GD_n_cluster(par)
        if agcontains(GD_clust(par)[i], n): break
        i += 1
    if i < GD_n_cluster(par): agdelete(g, n)
    ND_clust(n) = NULL                    # reset for ALL nodes, incl. deleted ones
    n = nn

# pass 2 — induce edges from the root graph                      (rank.c:273-278)
for n in agfstnode(g) …:
    for e in out-edges of n in dot_root(g):       # agfstout(dot_root(g), n)
        if agcontains(g, aghead(e)):
            agsubedge(g, e, 1)            # add corresponding subedge, create if needed
```

Notes:

* The strict loop bound `i < GD_n_cluster(par)` means: in the dot1 path (`collapse_cluster` calls `node_induce` *before* `make_new_cluster`) all previously registered sibling clusters are checked; in the dot2 path (`set_parent` calls `make_new_cluster` **then** `node_induce`, rank.c:557-562) the just-registered cluster `GD_clust(par)[n_cluster]` is exactly the one excluded.
* `dot_root(p)` (dotinit.c:483-486): `GD_dotroot(agroot(p))`.

### 3.9 `dot_scan_ranks(graph_t *g)` — rank.c:281-300 (public)

```
leader = None
GD_minrank(g) = INT_MAX; GD_maxrank(g) = -1
for n in agfstnode(g) …:
    if GD_maxrank(g) < ND_rank(n): GD_maxrank(g) = ND_rank(n)
    if GD_minrank(g) > ND_rank(n): GD_minrank(g) = ND_rank(n)
    if leader == None: leader = n
    else if ND_rank(n) < ND_rank(leader): leader = n
GD_leader(g) = leader
```
**Tie-breaker:** leader is the *first* node (in cgraph node order) achieving the minimum rank (strict `<` comparison keeps earlier nodes). Used when `CL_type != LOCAL` (`collapse_cluster`, rank.c:348) and recomputes bounds after ranking in GLOBAL/NOCLUST mode.

### 3.10 `cluster_leader(graph_t *clust)` — rank.c:302-324

```
leader = None; maxrank = 0
for n in GD_nlist(clust) … ND_next:                 # fast node list of the ranked cluster
    if ND_rank(n) == 0 and ND_node_type(n) == NORMAL:
        leader = n                                  # keeps the LAST such node (no break!)
    if maxrank < ND_rank(n): maxrank = ND_rank(n)   # local maxrank is computed but unused
assert(leader != NULL)
GD_leader(clust) = leader
for n in agfstnode(clust) …:
    assert(ND_UF_size(n) <= 1 || n == leader)
    UF_union(n, leader)
    ND_ranktype(n) = CLUSTER
```
**Tie-breaker:** leader is the **last** rank-0 `NORMAL` node in the fast-list order (contrast with `dot_scan_ranks`). The `maxrank` local is a dead store in this version; keep the scan order identical anyway because `leader` selection depends on it.

### 3.11 `collapse_cluster(graph_t *g, graph_t *subg)` — rank.c:333-349

```
if GD_parent(subg): return          # already collapsed (idempotence guard)
GD_parent(subg) = g
node_induce(g, subg)
if agfstnode(subg) == NULL: return  # empty cluster: not registered at all
make_new_cluster(g, subg)
if CL_type == LOCAL:
    dot1_rank(subg)                 # rank the cluster locally (recursive entry!)
    cluster_leader(subg)
else:
    dot_scan_ranks(subg)            # GLOBAL/NOCLUST: record bounds only
```
Header comment (rank.c:326-332): a cluster is collapsed in three steps — (1) local ranking, (2) collapse to one node on the least rank, (3) `class1()` converts inter-cluster edges using the "virtual node + 2 edges" trick.

### 3.12 `collapse_sets(graph_t *rg, graph_t *g)` — rank.c:352-369

```
for subg in subgraphs of g (agfstsubg/agnxtsubg):
    c = rank_set_class(subg)
    if c:
        if c == CLUSTER and CL_type == LOCAL:
            collapse_cluster(rg, subg)
        else:
            collapse_rankset(rg, subg, c)
    else:
        collapse_sets(rg, subg)         # plain subgraph: recurse, same rg
```
Docstring (rank.c:351): "Execute union commands for 'same rank' subgraphs and clusters." Note `rg` is threaded unchanged: called as `collapse_sets(g, g)` from `dot1_rank`, so ranksets found at any depth under `g` attach to `g`'s minset/maxset, and (LOCAL) clusters register into `g`'s cluster array. In GLOBAL/NOCLUST mode a cluster subgraph is *not* recursed into (`c != 0` branch) — its members are unioned like a rankset and nested subgraphs are never scanned at that level.

### 3.13 `collapse_rankset(graph_t *g, graph_t *subg, int kind)` — rank.c:185-222

"Merge the nodes of a min, max, or same rank set."

```
u = v = agfstnode(subg)
if u:
    ND_ranktype(u) = kind
    while (v = agnxtnode(subg, v)):
        UF_union(u, v)
        ND_ranktype(v) = ND_ranktype(u)      # = kind
    switch kind:
        MINRANK, SOURCERANK:
            if GD_minset(g) == None: GD_minset(g) = u
            else:                    GD_minset(g) = UF_union(GD_minset(g), u)
        MAXRANK, SINKRANK:
            if GD_maxset(g) == None: GD_maxset(g) = u
            else:                    GD_maxset(g) = UF_union(GD_maxset(g), u)
        # CLUSTER / SAMERANK: no set registration
    switch kind:
        SOURCERANK: ND_ranktype(GD_minset(g)) = kind   # promote merged leader to strict
        SINKRANK:   ND_ranktype(GD_maxset(g)) = kind
```

* All `ND_ranktype` writes land on the shared cgraph node records (cgraph subgraphs alias the root's nodes).
* The final minset/maxset representative is determined by `UF_union`'s smaller-`ND_id` rule (§1.6), applied to the whole transitive merge.
* A second rankset of the same polarity unions into the existing set; the second switch then overwrites the *merged leader's* ranktype (e.g. `min` + `source` ⇒ leader is `SOURCERANK`).

### 3.14 `find_clusters(graph_t *g)` — rank.c:371-379

```
for subg in subgraphs of dot_root(g):
    if GD_set_type(subg) == CLUSTER:
        collapse_cluster(g, subg)
```
Called from `expand_ranksets` (rank.c:501) when ranking the root with `CL_type != LOCAL`. In that mode `collapse_sets` unioned cluster members as if they were a rankset (§3.12), and clusters are materialized only now, after the global ranking — via `collapse_cluster` → `node_induce` + `make_new_cluster` + `dot_scan_ranks` (no local re-ranking).

### 3.15 `set_minmax(graph_t *g)` — rank.c:381-390

```
GD_minrank(g) += ND_rank(GD_leader(g))
GD_maxrank(g) += ND_rank(GD_leader(g))
for c in 1 .. GD_n_cluster(g):
    set_minmax(GD_clust(g)[c])
```
Offsets a cluster's **local** rank bounds by the absolute rank its leader obtained in the parent's ranking; recurses depth-first so nested leaders' ranks have already been made absolute by `expand_ranksets`' node loop (§3.19).

### 3.16 `minmax_edges(graph_t *g)` → `point` — rank.c:395-425

Docstring (rank.c:392-394): "To ensure that min and max rank nodes always have the intended rank assignment, reverse any incompatible edges."

```
slen = (x: 0, y: 0)
if GD_maxset(g) == None and GD_minset(g) == None: return slen
if GD_minset(g): GD_minset(g) = UF_find(GD_minset(g))
if GD_maxset(g): GD_maxset(g) = UF_find(GD_maxset(g))
if (n = GD_maxset(g)):
    slen.y = 1 if ND_ranktype(GD_maxset(g)) == SINKRANK else 0
    while (e = ND_out(n).list[0]):              # drain all out-edges
        assert(aghead(e) == UF_find(aghead(e)))
        reverse_edge(e)                          # acyclic.c:22-31
if (n = GD_minset(g)):
    slen.x = 1 if ND_ranktype(GD_minset(g)) == SOURCERANK else 0
    while (e = ND_in(n).list[0]):               # drain all in-edges
        assert(agtail(e) == UF_find(agtail(e)))
        reverse_edge(e)
return slen        # slen.x: min side is strict (source), slen.y: max side is strict (sink)
```
`reverse_edge(e)` (acyclic.c:22-31): `delete_fast_edge(e)`; if a fast edge already exists head→tail, `merge_oneway(e, f)`, else `virtual_edge(aghead(e), agtail(e), e)` (which reuses `e`'s weight/minlen and sets `ED_to_virt(e)`).

### 3.17 `minmax_edges2(graph_t *g, point slen)` → `bool` — rank.c:427-450

```
e = None
if GD_maxset(g) or GD_minset(g):
    for n in agfstnode(g) …:
        if n != UF_find(n): continue                    # only UF representatives
        if ND_out(n).size == 0 and GD_maxset(g) and n != GD_maxset(g):
            e = virtual_edge(n, GD_maxset(g), None)     # source n → maxset
            ED_minlen(e) = slen.y                       # 0 for min, 1 for sink
            ED_weight(e) = 0
        if ND_in(n).size == 0 and GD_minset(g) and n != GD_minset(g):
            e = virtual_edge(GD_minset(g), n, None)     # minset → sink n
            ED_minlen(e) = slen.x
            ED_weight(e) = 0
return e != None        # true iff at least one aux edge was added (e is sticky)
```
These zero-weight edges pull all isolated sources/sinks onto the min/max rank; `slen.x/.y` (from `minmax_edges`) enforce `rank=source`/`rank=sink` strictly (minlen 1) versus `rank=min`/`rank=max` permissively (minlen 0). Note `ED_minlen`/`ED_weight` assignment overwrites (not `merge`) — these edges are fresh.

### 3.18 `rank1(graph_t *g)` — rank.c:453-464 (public)

"Run the network simplex algorithm on each component."

```
maxiter = INT_MAX
if (s = agget(g, "nslimit1")): maxiter = scale_clamp(agnnodes(g), atof(s))
for c in 0 .. GD_comp(g).size - 1:
    GD_nlist(g) = GD_comp(g).list[c]
    rank(g, GD_n_cluster(g) == 0 ? 1 : 0, maxiter)     // TB balance iff no clusters
```

* `scale_clamp(original, scale)` (util/gv_math.h:79-90): `scale < 0` → 0; `scale > 1 && original > INT_MAX/scale` → `INT_MAX`; else `(int)(original * scale)`. So `nslimit1` scales the simplex iteration cap by node count.
* `rank(g, balance, maxiter)` (ns.c:1029-1040) = `rank2(g, balance, maxiter, search_size)` where `search_size` is `atoi(agget(g,"searchsize"))` or `SEARCHSIZE` (30). Interface contract (ns.c:940-950): input graph needs `GD_nlist` (`ND_next` chain), `ND_out`/`ND_in` elists, `ED_minlen` constraints (`rank(head) - rank(tail) >= minlen`), `ED_weight` costs; returns 0 ok / 1 disconnected / 2 serious. `balance=1` runs `TB_balance` (which zeroes minrank), `balance=2` `LR_balance`, else plain normalize (ns.c:1007-1019).
* **There is no separate auxiliary graph here** — the simplex ranks the fast graph itself. `rank()` is invoked once per connected component, with `GD_nlist(g)` swapped to that component's head; ranks accumulate in `ND_rank` across components.

### 3.19 `expand_ranksets(graph_t *g)` — rank.c:472-507

"Assigns ranks of non-leader nodes. Expands same, min, max rank sets. Leaf sets and clusters remain merged. Sets minrank and maxrank appropriately."

```
if (n = agfstnode(g)):
    GD_minrank(g) = INT_MAX; GD_maxrank(g) = -1
    while n:
        leader = UF_find(n)
        # works because ND_rank(n)==0 for non-cluster nodes and ND_rank(n) is
        # the local offset for nodes inside a (locally ranked) cluster — rank.c:481-483
        if leader != n: ND_rank(n) += ND_rank(leader)
        GD_maxrank(g) = max(GD_maxrank(g), ND_rank(n))
        GD_minrank(g) = min(GD_minrank(g), ND_rank(n))
        if ND_ranktype(n) and ND_ranktype(n) != LEAFSET:
            UF_singleton(n)             # dissolves the set; resets ranktype to NORMAL
        n = agnxtnode(g, n)
    if g == dot_root(g):
        if CL_type == LOCAL:
            for c in 1 .. GD_n_cluster(g): set_minmax(GD_clust(g)[c])
        else:
            find_clusters(g)
else:
    GD_minrank(g) = GD_maxrank(g) = 0   # empty graph
```

Consequences to preserve:

* `LEAFSET` members are the only UF sets that stay merged after ranking (nothing in rank.c assigns `LEAFSET`; it is only tested here — rank.c:492 — and in position.c:1011).
* After the loop, all other `UF` sets are dissolved and `ND_ranktype` of their members becomes `NORMAL` again; cluster membership survives via `GD_clust` arrays and `ND_clust`/`GD_leader` (set later than this point in the GLOBAL path; earlier, during `class1`'s `mark_clusters`, in the LOCAL path).
* For a locally ranked cluster `subg`, this ran during `collapse_cluster` → `dot1_rank(subg)` with `g != dot_root(g)`, producing local offsets and then re-merging under the leader via `cluster_leader`.

### 3.20 `dot1_rank(graph_t *g)` — rank.c:509-526 (static)

```
edgelabel_ranks(g)          # rank.c:512
collapse_sets(g, g)         # rank.c:514   UF-merge ranksets; (LOCAL) collapse+rank clusters recursively
class1(g)                   # rank.c:515   build fast graph; inter-cluster aux edges (see §6.1)
p = minmax_edges(g)         # rank.c:516   reverse edges at min/max sets; get strictness flags
decompose(g, 0)             # rank.c:517   connected components → GD_comp (decomp.c)
acyclic(g)                  # rank.c:518   per-component DFS cycle breaking (acyclic.c:58-69)
if minmax_edges2(g, p):     # rank.c:519   add zero-weight source/sink aux edges
    decompose(g, 0)         # rank.c:520   recompute components (those edges connect comps)
rank1(g)                    # rank.c:522   network simplex per component
expand_ranksets(g)          # rank.c:524   propagate leader ranks; set min/max rank
cleanup1(g)                 # rank.c:525   free fast graph + slack nodes + GD_comp
```

Ordering constraints that a port must keep: `collapse_sets` before `class1` (ranksets must be UF-merged so `class1` sees leaders); `minmax_edges` before `decompose` (reversed edges must be in the fast graph before component finding); `acyclic` after `decompose` (it iterates components); `rank1` needs `GD_comp`; `expand_ranksets` after ranking; `cleanup1` last (needs `GD_comp` for the teardown walk).

### 3.21 `dot_rank(graph_t *g)` — rank.c:528-537 (public entry point)

```c
void dot_rank(graph_t *g) {
    if (mapbool(agget(g, "newrank"))) { GD_flags(g) |= NEW_RANK; dot2_rank(g); }
    else                                dot1_rank(g);
    if (Verbose)
        fprintf(stderr, "Maxrank = %d, minrank = %d\n", GD_maxrank(g), GD_minrank(g));
}
```

* `mapbool` (utils.c:341-347 → mapBool utils.c:325-340): ""/NULL → false; "false"/"no" → false; "true"/"yes" → true; leading digit → `atoi != 0`; else default (false).
* **`dot_rank` itself does not loop over `GD_clust`** — recursion into clusters happens inside `collapse_sets` → `collapse_cluster` → `dot1_rank(subg)` (LOCAL), or later inside `expand_ranksets` → `find_clusters` (GLOBAL/NOCLUST). After `dot_rank` returns, `GD_minrank`/`GD_maxrank` of the root and of every registered cluster are set.

---

## 4. `rank.c` — the `newrank` path (`dot2_rank`)

New ranking code, "Copy of level.c in dotgen2" (rank.c:539-544). Builds an auxiliary **constraint graph `Xg`** (strict directed cgraph), ranks it with the network simplex, then copies ranks back.

### 4.1 Local definitions — rank.c:545-555

```c
#define BACKWARD_PENALTY       1000
#define STRONG_CLUSTER_WEIGHT  1000
#define NORANK                 6
#define ROOT     "\177root"
#define TOPNODE  "\177top"
#define BOTNODE  "\177bot"
#define ND_comp(n)  ND_hops(n)     /* hops is unused in dot: overload as component index */
```

### 4.2 `set_parent(graph_t *g, graph_t *p)` — rank.c:557-562

```
GD_parent(g) = p
make_new_cluster(p, g)      # register g as p's next cluster (§3.7)
node_induce(p, g)           # NB: after registration, so node_induce's scan excludes g itself (§3.8)
```

### 4.3 Predicates — rank.c:564-602

* `is_empty(g)` (564-566): `!agfstnode(g)`.
* `is_a_strong_cluster(g)` (568-572): `mapbool(agget(g, "compact"))`.
* `rankset_kind(g)` (574-591): exact string compares of `rank` attr:

```
if str non-empty:
    "min"    -> MINRANK
    "source" -> SOURCERANK
    "max"    -> MAXRANK
    "sink"   -> SINKRANK
    "same"   -> SAMERANK
return NORANK   # 6 — includes empty/missing attr; NO cluster check here
```

* `is_nonconstraint(e)` (593-602): true iff `E_constr` exists, the edge has a non-empty `constr` value, and `mapbool(constr)` is false. (Duplicate of `nonconstraint_edge`, class1.c:22-30.)

### 4.4 Rank-set union-find over `ND_set` — rank.c:604-634

Independent from the classic `ND_UF_parent` union-find (§1.6):

```c
static node_t *find(node_t *n) {                 // recursive with full path compression
    node_t *set;
    if ((set = ND_set(n))) { if (set != n) set = ND_set(n) = find(set); }
    else set = ND_set(n) = n;                    // unset ⇒ self
    return set;
}
static node_t *union_one(node_t *leader, node_t *n) {
    if (n) return (ND_set(find(n)) = find(leader));   // find(n)'s root joins leader's root
    else return leader;
}
static node_t *union_all(graph_t *g) {
    n = agfstnode(g); if (!n) return n;          // NULL for empty graph
    leader = find(n);
    while ((n = agnxtnode(g, n))) union_one(leader, n);
    return leader;
}
```
Leader election: the *first node in cgraph order* of the rankset remains the root (unions always hang other roots beneath `find(leader)`).

### 4.5 `compile_samerank(graph_t *ug, graph_t *parent_clust)` — rank.c:636-706

Recursive pre-order over the subgraph tree; builds `ND_set` unions and per-cluster `minrep`/`maxrep`.

```
if is_empty(ug): return                                   # empty subgraphs skipped entirely
if is_a_cluster(ug):
    clust = ug
    if parent_clust:
        GD_level(ug) = GD_level(parent_clust) + 1
        set_parent(ug, parent_clust)                      # registers cluster + node_induce
    else:
        GD_level(ug) = 0                                  # root case (dot2_rank calls with NULL)
else:
    clust = parent_clust

for s in subgraphs of ug:                                 # first: children
    compile_samerank(s, clust)

if is_a_cluster(ug):                                      # then: claim nodes
    for n in agfstnode(ug) …:
        if ND_clust(n) == 0: ND_clust(n) = ug             # first (outermost) cluster wins

switch rankset_kind(ug):                                  # then: ranksets
    SOURCERANK, MINRANK:
        leader = union_all(ug)
        if clust: GD_minrep(clust) = union_one(leader, GD_minrep(clust))
    SINKRANK, MAXRANK:
        leader = union_all(ug)
        if clust: GD_maxrep(clust) = union_one(leader, GD_maxrep(clust))
    SAMERANK:
        leader = union_all(ug)                            # recorded only via ND_set
    NORANK:
        pass
    default:
        agwarningf("%s has unrecognized rank=%s", agnameof(ug), agget(ug, "rank"))

if is_a_cluster(ug) and GD_minrep(ug):                    # degenerate cluster
    if GD_minrep(ug) == GD_maxrep(ug):
        up = union_all(ug); GD_minrep(ug) = up; GD_maxrep(ug) = up
```

Notes:
* Called as `compile_samerank(g, 0)` from `dot2_rank` (rank.c:1095); the root graph satisfies `is_a_cluster`, gets level 0, and does *not* get a parent.
* Because the root's node loop claims every unclaimed node (`ND_clust(n)` defaults to the root graph), `ND_clust` is never NULL afterwards, which makes `dot_lca` (§4.6) safe.
* `minrep`/`maxrep` are `ND_set`-based roots (not `ND_UF_parent`), shared across a whole cluster's ranksets.

### 4.6 `dot_lca(graph_t *c0, graph_t *c1)` / `is_internal_to_cluster(edge_t *e)` — rank.c:708-730

```
fn dot_lca(c0, c1):
    while c0 != c1:
        if GD_level(c0) >= GD_level(c1): c0 = GD_parent(c0)
        else:                            c1 = GD_parent(c1)
    return c0

fn is_internal_to_cluster(e):
    ct = ND_clust(agtail(e)); ch = ND_clust(aghead(e))
    if ct == ch: return True
    par = dot_lca(ct, ch)
    return par == ct or par == ch        # one endpoint's cluster contains the other
```
"Internal" ⇒ same cluster or ancestor/descendant clusters; such edges become strong constraints (possibly reversed), everything else crosses cluster boundaries.

### 4.7 `Last_node` / `makeXnode(graph_t *G, char *name)` — rank.c:732-749

```
static node_t *Last_node;                    # single file-static cursor (rank.c:732)

fn makeXnode(G, name):
    n = agnode(G, name, 1)                   # create (always)
    alloc_elist(4, ND_in(n)); alloc_elist(4, ND_out(n))
    if Last_node: ND_prev(n) = Last_node; ND_next(Last_node) = n
    else:         ND_prev(n) = NULL;    GD_nlist(G) = n
    Last_node = n
    ND_next(n) = NULL
    return n
```
Appends to `G`'s fast node list in creation order. `Last_node` is reset by `dot2_rank` (rank.c:1083) and `compile_nodes` (rank.c:756).

### 4.8 `compile_nodes(graph_t *g, graph_t *Xg)` — rank.c:751-765

```
Last_node = None
for n in agfstnode(g) …:                     # pass 1: one Xg node per ND_set leader
    if find(n) == n: ND_rep(n) = makeXnode(Xg, agnameof(n))
for n in agfstnode(g) …:                     # pass 2: members point at leader's rep
    if ND_rep(n) == 0: ND_rep(n) = ND_rep(find(n))
```
Xg node names are the original node names (leaders only). Every original node ends up with a non-NULL `ND_rep`.

### 4.9 `merge(edge_t *e, int minlen, int weight)` — rank.c:767-771

```
ED_minlen(e) = max(ED_minlen(e), minlen)     # lengths: max-merge
ED_weight(e) += weight                       # weights: accumulate
```

### 4.10 `strong(graph_t *g, node_t *t, node_t *h, edge_t *orig)` — rank.c:773-782

```
e = agfindedge(g, t, h) or agfindedge(g, h, t) or agedge(g, t, h, NULL, 1)
if e: merge(e, ED_minlen(orig), ED_weight(orig))
else: agerrorf("ranking: failure to create strong constraint edge between nodes %s and %s\n", …)
```
Direction-insensitive: a constraint in either direction is merged (max minlen, summed weight). Xg is strict (`Agstrictdirected`, rank.c:1084) so `agfindedge` deduplicates.

### 4.11 `weak(graph_t *g, node_t *t, node_t *h, edge_t *orig)` — rank.c:784-808

Cross-boundary edge adjacent to a `compact` cluster; modeled by a fresh *weak node* feeding both endpoints.

```
static int id = 0; char buf[100]                       # function-static counter
for e in agfstin(g, t) …:                              # look for an existing weak (v -> t),(v -> h)
    v = agtail(e)
    f = agfstout(g, v)
    if f and aghead(f) == h: return                    # duplicate: drop the new constraint
if true (loop exhausted ⇒ e == NULL):                  # `if (!e)` — always true here
    snprintf(buf, "_weak_%d", id++)                    # buf[100]
    v = makeXnode(g, buf)
    e = agedge(g, v, t, NULL, 1)
    f = agedge(g, v, h, NULL, 1)
ED_minlen(e) = max(ED_minlen(e), 0)                    # "effectively a nop"
ED_weight(e) += ED_weight(orig) * BACKWARD_PENALTY     # ×1000
ED_minlen(f) = max(ED_minlen(f), ED_minlen(orig))
ED_weight(f) += ED_weight(orig)
```
Quirks to reproduce: the duplicate check inspects only the **first** out-edge (`agfstout`) of each candidate tail `v`; the counter never resets across graphs; the `e`/`f` after the loop are always freshly created.

### 4.12 `compile_edges(graph_t *ug, graph_t *Xg)` — rank.c:810-847

```
for n in agfstnode(ug) …:
    Xt = ND_rep(n)
    for e in agfstout(ug, n) …:                 # each original edge once (out-edges)
        if is_nonconstraint(e): continue
        Xh = ND_rep(find(aghead(e)))
        if Xt == Xh: continue                   # intra-rankset (incl. self loops)
        tc = ND_clust(agtail(e)); hc = ND_clust(aghead(e))
        if is_internal_to_cluster(e):
            clust_tail = ND_clust(agtail(e)); clust_head = ND_clust(aghead(e))
            # reverse the constraint if it would drag a cluster extreme outward:
            if (clust_tail and find(agtail(e)) == GD_maxrep(clust_tail))
               or (clust_head and find(aghead(e)) == GD_minrep(clust_head)):
                SWAP(Xt, Xh)
            strong(Xg, Xt, Xh, e)
        else:
            if is_a_strong_cluster(tc) or is_a_strong_cluster(hc): weak(Xg, Xt, Xh, e)
            else:                                                  strong(Xg, Xt, Xh, e)
```
Note `Xt` uses `ND_rep(n)` directly (== `ND_rep(find(n))` after pass 2 of `compile_nodes`) while `Xh` recomputes `find`. `GD_maxrep`/`GD_minrep` may be NULL (cluster without that kind of rankset); the `find(...) == NULL` comparison then safely fails.

### 4.13 `compile_clusters(graph_t *g, graph_t *Xg, node_t *top, node_t *bot)` — rank.c:849-876

```
if is_a_cluster(g) and is_a_strong_cluster(g):
    for n in agfstnode(g) …:
        if agfstin(g, n) == 0:                       # internal source
            rep = ND_rep(find(n))
            if not top: top = makeXnode(Xg, TOPNODE)
            agedge(Xg, top, rep, NULL, 1)
        if agfstout(g, n) == 0:                      # internal sink
            rep = ND_rep(find(n))
            if not bot: bot = makeXnode(Xg, BOTNODE)
            agedge(Xg, rep, bot, NULL, 1)
    if top and bot:
        e = agedge(Xg, top, bot, NULL, 1)
        merge(e, 0, STRONG_CLUSTER_WEIGHT)           # minlen max(cur,0); weight += 1000
for sub in agfstsubg(g) …:
    compile_clusters(sub, Xg, top, bot)
```
Called as `compile_clusters(g, Xg, 0, 0)` (rank.c:1098). Quirk: `top`/`bot` are threaded through recursion **unchanged**, so the *first* strong cluster encountered creates `TOPNODE`/`BOTNODE` and all nested strong clusters reuse the same two nodes (the `if (!top)` guard only creates once). Fresh Xg edges have zeroed `Agedgeinfo_t` (bound via `mydisc`, §4.18), hence `minlen 0` and weight only from `merge` — the 1000-weight top→bot edge is what squeezes a `compact` cluster's internal slack.

### 4.14 `reverse_edge2(graph_t *g, edge_t *e)` — rank.c:878-887

```
rev = agfindedge(g, aghead(e), agtail(e)) or agedge(g, aghead(e), agtail(e), NULL, 1)
merge(rev, ED_minlen(e), ED_weight(e))
agdelete(g, e)
```
(Cycle breaking on the Xg constraint graph — distinct from the fast-graph `reverse_edge` in acyclic.c:22.)

### 4.15 `dfs` / `break_cycles` — rank.c:889-921

```
fn dfs(g, v):
    if ND_mark(v): return
    ND_mark(v) = true; ND_onstack(v) = true
    e = agfstout(g, v)
    while e:
        f = agnxtout(g, e)                # saved BEFORE body: agdelete in reverse_edge2
        w = aghead(e)                     #   invalidates the edge iterator
        if ND_onstack(w): reverse_edge2(g, e)
        elif not ND_mark(w): dfs(g, w)
        e = f
    ND_onstack(v) = false

fn break_cycles(g):
    for n in agfstnode(g) …: ND_mark(n) = false; ND_onstack(n) = false
    for n in agfstnode(g) …: dfs(g, n)
```
Recursive DFS in cgraph node/out-edge order; back edges (w on stack) are reversed and merged. Runs on Xg before component numbering (rank.c:1099).

### 4.16 `setMinMax(graph_t *g, int doRoot)` — rank.c:927-952

"Will only be called with the root graph or a cluster which are guaranteed to contain nodes. Thus, leader will be set."

```
for c in 1 .. GD_n_cluster(g):                 # children first
    setMinMax(GD_clust(g)[c], 0)
if not GD_parent(g) and not doRoot: return     # skip the root unless forced
GD_minrank(g) = INT_MAX; GD_maxrank(g) = -1
for n in agfstnode(g) …:
    v = ND_rank(n)
    if GD_maxrank(g) < v: GD_maxrank(g) = v
    if GD_minrank(g) > v: GD_minrank(g) = v; leader = n
GD_leader(g) = leader
```
Leader = **first** node achieving the minimum (strict `<`; the first node always becomes leader because minrank starts at `INT_MAX`).

### 4.17 `readout_levels(graph_t *g, graph_t *Xg, int ncc)` — rank.c:960-1014

"Store node rank information in original graph. Set rank bounds in graph and clusters. Free added data structures. rank2 is called with balance=1, which ensures that minrank=0."

```
doRoot = 0
GD_minrank(g) = INT_MAX; GD_maxrank(g) = -1
minrk = None
if ncc > 1:
    minrk = calloc(ncc + 1, int); for i in 1..ncc: minrk[i] = INT_MAX
for n in agfstnode(g) …:
    xn = ND_rep(find(n))
    ND_rank(n) = ND_rank(xn)
    update g's minrank/maxrank
    if minrk:
        ND_comp(n) = ND_comp(xn)                              # component id via ND_hops
        minrk[ND_comp(n)] = min(minrk[ND_comp(n)], ND_rank(n))
if minrk:
    for n in agfstnode(g) …: ND_rank(n) -= minrk[ND_comp(n)]  # per-component shift
    doRoot = 1                                                # non-uniform: force root recompute
elif GD_minrank(g) > 0:                                       # "should never happen"
    delta = GD_minrank(g)
    for n …: ND_rank(n) -= delta
    GD_minrank(g) -= delta; GD_maxrank(g) -= delta
setMinMax(g, doRoot)                                          # sets cluster bounds + leaders
for n in agfstnode(Xg) …:                                     # release Xg fastgraph memory
    free_list(ND_in(n)); free_list(ND_out(n))
free(ND_alg(agfstnode(g)))                                    # defensive: normally NULL here
for n in agfstnode(g) …: ND_alg(n) = NULL
free(minrk)
```

### 4.18 `dfscc` / `connect_components` / `add_fast_edges` — rank.c:1016-1061

```
fn dfscc(g, n, cc):                          # recursive, Xg
    if ND_comp(n) == 0:
        ND_comp(n) = cc
        for e in agfstout(g, n) …: dfscc(g, aghead(e), cc)
        for e in agfstin(g, n) …:  dfscc(g, agtail(e), cc)

fn connect_components(g) -> ncc:             # Xg
    cc = 0
    for n: ND_comp(n) = 0
    for n in agfstnode(g) …:
        if ND_comp(n) == 0: dfscc(g, n, ++cc)
    if cc > 1:
        root = makeXnode(g, ROOT)            # "\177root"
        ncc = 1
        for n in agfstnode(g) …:
            if ND_comp(n) == ncc:
                agedge(g, root, n, NULL, 1)  # root → first node of each component
                ncc += 1
    return cc

fn add_fast_edges(g):                        # Xg — build the fastgraph elists ns.c needs
    for n in agfstnode(g) …:
        for e in agfstout(g, n) …:
            elist_append(e, ND_out(n))
            elist_append(e, ND_in(aghead(e)))
```
Component ids are assigned in first-appearance order over `agfstnode` iteration; the ROOT node links to the first-encountered node of each component (ids ascending), making Xg connected so one simplex run covers everything.

### 4.19 cgraph record plumbing — rank.c:1063-1075

```
my_init_graph/my_init_node/my_init_edge: agbindrec(obj, "level {graph|node|edge} rec", sz[i], true)
static Agcbdisc_t mydisc = { {my_init_graph,0,0}, {my_init_node,0,0}, {my_init_edge,0,0} };
int infosizes[] = { sizeof(Agraphinfo_t), sizeof(Agnodeinfo_t), sizeof(Agedgeinfo_t) };
```
Pushed onto Xg (rank.c:1086) so every object created in Xg gets a zeroed dotgen info record. (In a Rust port this corresponds to "Xg nodes/edges carry the same fastgraph structs".)

### 4.20 `dot2_rank(graph_t *g)` — rank.c:1077-1113 (static)

```
Last_node = None
Xg = agopen("level assignment constraints", Agstrictdirected, NULL)
agbindrec(Xg, "level graph rec", sizeof(Agraphinfo_t), true)
agpushdisc(Xg, &mydisc, infosizes)
edgelabel_ranks(g)                                       # same doubling as dot1 path
maxiter = agget(g, "nslimit1") ? scale_clamp(agnnodes(g), atof(s)) : INT_MAX
compile_samerank(g, 0)                                   # ND_set unions; cluster registration
compile_nodes(g, Xg)                                     # rankset representatives in Xg
compile_edges(g, Xg)                                     # strong/weak constraints
compile_clusters(g, Xg, 0, 0)                            # compact-cluster top/bot nodes
break_cycles(Xg)                                         # make Xg acyclic
ncc = connect_components(Xg)                             # ND_comp ids + ROOT joiner
add_fast_edges(Xg)                                       # fastgraph elists for ns
ssize = agget(g, "searchsize") ? atoi(s) : -1            # -1 ⇒ ns uses SEARCHSIZE (30)
rank2(Xg, 1, maxiter, ssize)                             # balance=1 (TB) ⇒ minrank 0
readout_levels(g, Xg, ncc)                               # copy ranks back; set bounds; free
agclose(Xg)
```
(`ssize` is declared at rank.c:1078 and assigned at 1103-1106.) Note `dot2_rank` never touches `GD_nlist`/`GD_comp` of `g`; decomposition of the *original* graph is not needed here.

---

## 5. `decomp.c` — connected components (complete)

File-level statics (decomp.c:28-29):

```c
static node_t *Last_node;    // tail cursor of the component list being built
static size_t  Cmark;        // monotonically increasing visit stamp
```

Header comment (decomp.c:12-18): "Decompose finds the connected components of a graph. It searches the temporary edges and ignores non-root nodes. The roots of the search are the real nodes of the graph, but any virtual nodes discovered are also included in the component."

### 5.1 Component list construction — decomp.c:31-59

```
fn begin_component(g):
    Last_node = GD_nlist(g) = None                 # start a fresh fast node list

fn add_to_component(g, n):
    ND_mark(n) = Cmark                             # stamped as finalized
    if Last_node: ND_prev(n) = Last_node; ND_next(Last_node) = n
    else:         ND_prev(n) = None;    GD_nlist(g) = n
    Last_node = n
    ND_next(n) = None

fn end_component(g):
    i = GD_comp(g).size++                          # append index
    GD_comp(g).list = gv_recalloc(GD_comp(g).list, GD_comp(g).size - 1, GD_comp(g).size, ptr)
    GD_comp(g).list[i] = GD_nlist(g)               # record the component's head node
```
Side effect: each component's nodes are (re)linked into `GD_nlist` in the order they were finalized; `GD_comp(g).list[c]` is that component's head. The `list` array grows by recalloc one slot at a time and is *not* freed between decompose calls (only `size` is reset, decomp.c:116).

### 5.2 Explicit-DFS stack — decomp.c:61-75

```
typedef LIST(node_t *) node_stack_t;

fn push(sp, np):
    ND_mark(np) = Cmark + 1        # "on stack" marker: unprocessed < Cmark, finalized == Cmark
    LIST_PUSH_BACK(sp, np)

fn pop(sp):
    if LIST_IS_EMPTY(sp): return None
    return LIST_POP_BACK(sp)       # LIFO
```
Comment (decomp.c:77-84) explains the three-state mark: `< Cmark` unvisited (from earlier passes), `== Cmark` finalized into a component, `== Cmark + 1` currently on the stack. **This is a LIFO stack, not the FIFO `node_queue_t` of mincross.**

### 5.3 `search_component(node_stack_t *stk, graph_t *g, node_t *n)` — decomp.c:85-106

Iterative DFS; edge vectors are processed **in reverse order within each list** so that the node processing order matches the old recursive implementation (comment decomp.c:77-80):

```
push(stk, n)
while (n = pop(stk)):
    if ND_mark(n) == Cmark: continue        # already finalized (duplicates on stack)
    add_to_component(g, n)
    vec = [ND_flat_in(n), ND_flat_out(n), ND_in(n), ND_out(n)]   # fixed order!
    for c in 0..3:
        if vec[c].list and vec[c].size != 0:
            for i = vec[c].size - 1 downto 0:                    # reverse index order
                e = vec[c].list[i]
                other = aghead(e); if other == n: other = agtail(e)   # undirected step
                if ND_mark(other) != Cmark and other == UF_find(other):
                    push(stk, other)
```

Facts to preserve:

* Traverses **all four** elists (flat in/out and regular in/out) — i.e., temporary/fast edges and flat edges, per the header comment.
* Undirected traversal: from `n` it steps to the *other* endpoint of each edge.
* Only `UF_find` representatives are pushed (`other == UF_find(other)`): non-leader nodes of collapsed ranksets/clusters are excluded, so each set enters a component exactly once through its leader. Virtual nodes and slack nodes are pushed normally (they are their own UF root; `UF_find` on a node with `ND_UF_parent == NULL` returns itself).
* A node already on the stack (`Cmark + 1`) may be pushed multiple times; the `== Cmark` check at pop time makes that harmless.
* Deterministic order: for each popped node, lists are scanned in the order `flat_in, flat_out, in, out` and each list from its **last** to its **first** element.

### 5.4 `decompose(graph_t *g, int pass)` — decomp.c:108-130 (public)

```
++Cmark; if Cmark == 0: Cmark = 1              # wrap guard
GD_comp(g).size = 0                            # keep the list allocation
stk = empty node_stack_t
for n in agfstnode(g) … agnxtnode(g, n):       # real nodes of g, in cgraph order
    v = n
    if pass > 0 and (subg = ND_clust(v)):
        v = GD_rankleader(subg)[ND_rank(v)]    # expand cluster: use its rank representative
    else if v != UF_find(v):
        continue                               # skip non-representatives
    if ND_mark(v) != Cmark:
        begin_component(g)
        search_component(&stk, g, v)
        end_component(g)
LIST_FREE(&stk)
```

* `pass == 0` (ranking, rank.c:517/520): seeds are UF representatives among the real nodes; virtual/slack nodes join via edges.
* `pass == 1` (mincross, mincross.c:1026): a node that belongs to a cluster (`ND_clust`) is replaced by its cluster's `GD_rankleader[ND_rank(v)]` — the collapsed cluster's per-rank representative in the expanded layout — before searching. Unclustered nodes are still UF-filtered.
* Components are appended to `GD_comp` in order of first-encountered unmarked seed, i.e. **cgraph node order of the root graph**.
* `Cmark` is a process-lifetime counter shared by all graphs; marks of *other* graphs/passes never equal the current `Cmark`, so stale marks are safe.
* Complexity: each edge scanned at most twice (once from each finalized endpoint); stack may hold duplicates bounded by degree.

---

## 6. External contracts invoked by rank.c

### 6.1 `class1(graph_t *g)` — dotgen/class1.c:63-100

"Classify edges for rank assignment phase to create temporary edges" (class1.c:12-14).

```
mark_clusters(g)                             # cluster.c:302-345: ND_clust marks; UF_setname(n, GD_leader(clust));
                                             #   ND_ranktype = CLUSTER; extends marks over virtual chains
for n in agfstnode(g) …:
    for e in agfstout(g, n) …:
        if ED_to_virt(e): continue           # already processed
        if nonconstraint_edge(e): continue   # constr=false
        t = UF_find(agtail(e)); h = UF_find(aghead(e))
        if t == h: continue                  # self, flat (intra-set), intra-cluster edges
        if ND_clust(t) or ND_clust(h):
            interclust1(g, agtail(e), aghead(e), e); continue
        if (rep = find_fast_edge(t, h)): merge_oneway(e, rep)     # parallel edges merge
        else: virtual_edge(t, h, e)                                # new fast edge
```

`interclust1` (class1.c:32-62) implements the "virtual node + 2 edges" trick for inter-cluster edges (referenced by rank.c:326-332):

```
t_rank = ND_rank(tail) - ND_rank(GD_leader(ND_clust(tail)))   # 0 if tail not in a cluster
h_rank = ND_rank(head) - ND_rank(GD_leader(ND_clust(head)))   # 0 if head not in a cluster
offset = ED_minlen(e) + t_rank - h_rank
if offset > 0: t_len = 0;      h_len = offset
else:          t_len = -offset; h_len = 0
v = virtual_node(g); ND_node_type(v) = SLACKNODE        # freed by cleanup1
t0 = UF_find(t); h0 = UF_find(h)                        # cluster leaders
rt = make_aux_edge(v, t0, t_len, CL_BACK * ED_weight(e))    # CL_BACK = 10 (const.h:141)
rh = make_aux_edge(v, h0, h_len, ED_weight(e))
ED_to_orig(rt) = ED_to_orig(rh) = e
```
`make_aux_edge(u, v, len, wt)` (position.c:182-211): allocates an edge pair, errors out (`return NULL`) if `len > INT_MAX`, sets `ED_minlen = ROUND(len)`, `ED_weight = wt`, and `fast_edge`s it into the graph.

`merge_oneway(e, rep)` (fastgr.c:244-254) → `basic_merge` (fastgr.c:231-242): `ED_minlen(rep) = max(rep, e)`; walks the `ED_to_virt` chain adding `ED_count`/`ED_xpenalty`/`ED_weight`.

`virtual_edge(u, v, orig)` (fastgr.c:170-173) → `new_virtual_edge` (fastgr.c:131-168): with `orig`, copies `AGSEQ`, `ED_count`, `ED_xpenalty`, `ED_weight`, `ED_minlen`, ports, sets `ED_to_virt(orig) = e` if unset, `ED_to_orig(e) = orig`; without `orig`, defaults all four to 1.

### 6.2 `acyclic(graph_t *g)` — dotgen/acyclic.c:58-69

```
for c in 0 .. GD_comp(g).size - 1:
    GD_nlist(g) = GD_comp(g).list[c]
    for n in GD_nlist …: ND_mark(n) = false
    for n in GD_nlist …: dfs(n)
```
`dfs` (acyclic.c:33-55): recursive; on encountering a head that is `ND_onstack`, `reverse_edge(e)` and `i--` (re-examine the slot after the swap-removal `zapinlist`). Consumes `GD_comp` produced by `decompose` and the `ND_mark`/`ND_onstack` fields.

### 6.3 Network simplex: `rank2` / `rank` — common/ns.c:951-1040

Contract for the port (see §3.18): works on `GD_nlist` + `ND_out`/`ND_in` fastgraph lists; constraints `rank(head) - rank(tail) >= ED_minlen`; cost Σ `ED_weight · (rank(h) - rank(t))`; `balance=1` → `TB_balance` (minrank forced to 0), `balance=0` → `scan_and_normalize`; iteration cap `maxiter` (`maxiter <= 0` ⇒ skip pivoting entirely, ns.c:984-987); `search_size < 0` → `SEARCHSIZE = 30` (ns.c:972-975). Returns 0/1/2 (ok / disconnected / error). Ranks may be negative in general; the dot1 path tolerates that (no normalization before `expand_ranksets`), while the dot2 path forces minrank 0 via `balance=1` (rank.c:958 comment).

### 6.4 Misc helpers

* `maptoken(p, name, val)` — utils.c:315-323 (returns last `val` entry when `p` is NULL or unmatched).
* `mapbool(p)` — utils.c:325-347 (`""`/NULL ⇒ false via `mapBool(p, false)`).
* `do_graph_label(sg)` — input.c:830 (cluster label; may set `GD_has_labels |= GRAPH_LABEL`).
* `fast_node(g, n)` — fastgr.c:175-187 (prepend to `GD_nlist`); `fast_edge(e)` — fastgr.c:71-93 (append to `ND_out(tail)`/`ND_in(head)`); `virtual_node(g)` — fastgr.c:200-213 (`ND_node_type = VIRTUAL`, `ND_lw=ND_rw=ND_ht=1`, `ND_UF_size=1`, `alloc_elist(4, in/out)`, `fast_node`); `zapinlist` — fastgr.c:96-106 (swap-with-last removal); `delete_fast_edge` — fastgr.c:109-114.

---

## 7. End-to-end control flow

### 7.1 Classic path (`newrank` false / unset)

```
dot_rank(g)                                     rank.c:528
└─ dot1_rank(g)                                 rank.c:509
   ├─ edgelabel_ranks(g)                        ×2 minlens, ÷2 ranksep (if EDGE_LABEL)
   ├─ collapse_sets(g, g)                       for each subgraph of g:
   │  ├─ cluster & LOCAL → collapse_cluster(g, subg)
   │  │    ├─ node_induce, make_new_cluster
   │  │    ├─ dot1_rank(subg)                   ← full recursive pipeline, local ranks
   │  │    └─ cluster_leader(subg)              merge under leader; ranktype=CLUSTER
   │  └─ rankset (same/min/source/max/sink) or cluster & GLOBAL/NOCLUST
   │       → collapse_rankset(g, subg, kind)    UF merges; minset/maxset registration
   ├─ class1(g)                                 fastgraph; mark_clusters; interclust1 aux edges
   ├─ p = minmax_edges(g)                       reverse edges at min/max reps; strictness flags
   ├─ decompose(g, 0)                           GD_comp = components of fastgraph
   ├─ acyclic(g)                                per-component cycle break (may add reversed vedges)
   ├─ if minmax_edges2(g, p): decompose(g, 0)   re-split after zero-weight source/sink edges
   ├─ rank1(g)                                  per component: rank(g, 1 if no clusters else 0, maxiter)
   ├─ expand_ranksets(g)                        ND_rank(n) += ND_rank(UF_find(n)); bounds;
   │    ├─ (root, LOCAL)  set_minmax per cluster (offset by leader rank)
   │    └─ (root, GLOBAL/NOCLUST) find_clusters → collapse_cluster (no local ranking)
   └─ cleanup1(g)                               free fastgraph, slack nodes, GD_comp
```

### 7.2 `newrank` path

```
dot_rank(g)
└─ dot2_rank(g)                                 rank.c:1077
   ├─ Xg = strict directed graph with dotgen info recs
   ├─ edgelabel_ranks(g)
   ├─ maxiter = nslimit1 ? scale_clamp(agnnodes(g), atof) : INT_MAX
   ├─ compile_samerank(g, 0)                    ND_set unions; cluster registration via set_parent
   ├─ compile_nodes(g, Xg)                      one Xg node per rankset leader
   ├─ compile_edges(g, Xg)                      strong (merged) / weak (×1000 penalty) constraints
   ├─ compile_clusters(g, Xg, 0, 0)             compact clusters: TOPNODE/BOTNODE + weight-1000 edge
   ├─ break_cycles(Xg)                          reverse back edges
   ├─ ncc = connect_components(Xg)              ND_comp ids; ROOT node joins components
   ├─ add_fast_edges(Xg)                        build fastgraph elists
   ├─ rank2(Xg, 1, maxiter, ssize)              simplex, TB balance ⇒ minrank 0
   ├─ readout_levels(g, Xg, ncc)                copy ranks (per-component shift if ncc>1);
   │                                            setMinMax for root+clusters; free Xg fastgraph
   └─ agclose(Xg)
```

---

## 8. `decompose` call sites and the life of `GD_comp`

| Site | pass | Purpose |
|---|---|---|
| rank.c:517 `decompose(g, 0)` | 0 | component split before `acyclic` + `rank1` |
| rank.c:520 `decompose(g, 0)` | 0 | recompute after `minmax_edges2` connected isolated sources/sinks to the min/max sets |
| mincross.c:1026 `decompose(g, 1)` | 1 | split expanded graph (cluster rankleaders) for mincross |
| class2.c:287-291 | — | *not* decompose: fabricates `GD_comp.size = 1; list[0] = GD_nlist(g)` ("since decompose() is not called on subgraphs") |
| cluster.c:283-284 `expand_cluster` | — | same fabrication for a cluster's own layout |

`GD_comp` consumers: `cleanup1` (rank.c:87-116), `acyclic` (acyclic.c:62-68), `rank1` (rank.c:460-463), mincross `merge2`/`merge` paths (mincross.c:359, 425, 787-801 — merges components then resets `size = 1`), `dot_cleanup` (dotinit.c:166 `free_list(GD_comp(g))`). Ordering guarantee: components are stored in first-seed order (cgraph node order), each component's `ND_next` chain in DFS finalization order.

---

## 9. Determinism inventory (tie-breakers the port must reproduce)

1. **UF leader (classic):** smaller `ND_id` wins (utils.c:132-139); `ND_id` = cgraph creation sequence of the root graph.
2. **UF leader (dot2 `ND_set`):** first node in the rankset's cgraph order stays root (§4.4).
3. **`collapse_rankset`:** nominal leader `u` = `agfstnode(subg)`; minset/maxset registration merges into whichever set exists first.
4. **`dot_scan_ranks` / `setMinMax` leader:** first node in cgraph order with minimal rank.
5. **`cluster_leader`:** *last* rank-0 `NORMAL` node in the fast list (rank.c:310-315 — no break).
6. **`decompose` component order:** cgraph node order of seeds; within a component, LIFO DFS with vector order `[flat_in, flat_out, in, out]`, each list scanned last→first (decomp.c:92-104).
7. **`acyclic`/`break_cycles` DFS:** cgraph node order; out-edge list order; reversal on back edges only.
8. **`class1` merging:** first fast edge between a pair becomes the representative (`find_fast_edge` scan order = insertion order in `ND_out`).
9. **`compile_samerank` `ND_clust`:** outermost cluster wins (`if (ND_clust(n) == 0)`, rank.c:662-663); subgraphs processed in cgraph subgraph order.
10. **`weak` dedup:** only the first out-edge of candidate tails is inspected (rank.c:794).
11. **`connect_components`:** component ids in first-appearance order; ROOT edges to first node of each.
12. **Xg node creation order** (`compile_nodes`, `makeXnode`) determines `ND_id` inside Xg, hence the simplex's internal tie-breaking on Xg.

---

## 10. Queue and stack semantics

* **`node_queue_t`** = `LIST(Agnode_t *)` (dotprocs.h:22). FIFO discipline: `LIST_PUSH_BACK` to enqueue, `LIST_POP_FRONT` to dequeue (mincross.c:1238-1240, 1283, 1291). Used only by `build_ranks`/`enqueue_neighbors`/`install_cluster` (§11) — **not** by rank.c/decomp.c.
* **`node_stack_t`** = `LIST(node_t *)` (decomp.c:61). LIFO: `LIST_PUSH_BACK` + `LIST_POP_BACK`, with the three-valued `ND_mark` protocol (§5.2). Empty pop returns NULL (decomp.c:68-75) and the `while ((n = pop(stk)))` loop relies on NULL termination — nodes are never NULL in the list itself.
* `renewlist`'s `track` and `cleanup1`'s `to_free` are plain append-only `LIST(edge_t *)` (§3.2-3.4).

---

## 11. Per-rank node ordering before mincross (for completeness)

rank.c only produces `ND_rank` values. The insertion order inside each rank array is established later, in mincross init (mincross.c:1026-1030) or `expand_cluster` (cluster.c:281-297):

1. **`allocate_ranks(g)`** (mincross.c:1122-1147): counts, per rank r, every node whose `ND_rank == r` plus one for every rank r strictly between the endpoints of each edge (virtual-node slots); then `GD_rank(g)[r].an = GD_rank(g)[r].n = cn[r] + 1` and `av = v = calloc(cn[r] + 1)` for `r` in `[GD_minrank, GD_maxrank]`. ("Note that no nodes are put into the structure yet.")
2. **`build_ranks(g, pass)`** (mincross.c:1199-1273):
   * clears `MARK(n)` and zeroes every `GD_rank(g)[i].n`;
   * seeds: nodes whose `ND_in` (pass 0) / `ND_out` (pass 1) is empty; for clusters the walk over `GD_nlist` is **backwards** (`ND_prev`) to preserve input node order (`walkbackwards = g != agroot(g)`, mincross.c:1222-1231);
   * FIFO queue `node_queue_t`: pop front; `ND_ranktype != CLUSTER` ⇒ `install_in_rank` + `enqueue_neighbors`; else `install_cluster`;
   * **`install_in_rank`** (mincross.c:1150-1193): `i = GD_rank(g)[r].n; GD_rank(g)[r].v[i] = n; ND_order(n) = i; GD_rank(g)[r].n++` — i.e. append at the current right end of the rank, order = insertion index; error paths check capacity and rank range (mincross.c:1155-1191);
   * **`enqueue_neighbors`** (mincross.c:1275-1295): pass 0 scans `ND_out` (push unmarked heads), pass 1 scans `ND_in` (push unmarked tails); marks before pushing;
   * **`install_cluster`** (cluster.c:380-397): once per cluster per pass (`GD_installed(clust) != pass + 1`), installs `GD_rankleader(clust)[r]` for `r` in `[GD_minrank(clust), GD_maxrank(clust)]` in rank order, enqueueing their neighbors;
   * afterwards: `GD_rank(Root)[i].valid = false` for all ranks; if `GD_flip(g)` each non-empty rank's `v` array is reversed in place; if `g == dot_root(g)` and `ncross() > 0`, `transpose(g, false)`.
   Net effect: a BFS from sources (or sinks) that produces crossing-free orders on series-parallel graphs (comment mincross.c:1195-1198).

---

## 12. Rust port notes

### 12.1 Ownership / lifetime map

| Object | Created | Destroyed |
|---|---|---|
| Fast (virtual) edges | `virtual_edge` / `make_aux_edge` / `reverse_edge` (class1, minmax_edges, acyclic) | `cleanup1` (collected via `ND_out` "owning" lists + `ED_to_virt` alias pass) |
| Slack nodes | `interclust1` (class1.c:55-56) | `cleanup1` (rank.c:98-111) |
| Fast node list links | `decompose` add_to_component / `fast_node` / `makeXnode` | `cleanup1` (comp list freed), `readout_levels` (Xg lists freed), `dot_cleanup` |
| `GD_comp.list` | grown by `end_component` | `cleanup1` (rank.c:161-163), class2.c:289, dotinit.c:166 |
| Xg graph + recs | `dot2_rank` | `agclose(Xg)` (rank.c:1112) |
| `minrk` | `readout_levels` | same function (rank.c:1013) |

### 12.2 Aliasing / UB spots to handle deliberately

1. `renewlist` starts at index `L->size` (the NULL sentinel slot) — harmless in C, but a Rust slice walk must clamp to `size` (§3.2).
2. `cleanup1` frees `previous` (an `&e2->out` interior pointer of an `Agedgepair_t`) — in Rust, model a fast edge as a single heap object with in/out roles; free-once semantics per unique pointer, exactly as the sort-dedupe achieves (§3.4).
3. `acyclic`'s `dfs` decrements the loop index after `reverse_edge` because `zapinlist` swap-removes during iteration (acyclic.c:44-52); dot2's `dfs` pre-computes `agnxtout` before the body for the same reason (rank.c:898-907). In Rust, iterate over snapshot indices or rebuild lists.
4. `weak`'s dedup reads `agfstout(g, v)` while new weak nodes/edges are being created — order-dependent, reproduce faithfully (§4.11).
5. `ND_comp` aliases `ND_hops` (rank.c:555) and `decompose` stamps `ND_mark` with a global counter — in Rust use dedicated fields; keep the "stale marks are never equal to the current stamp" invariant by allocating a fresh stamp per `decompose` call.
6. `compile_clusters`'s shared `top`/`bot` across nested clusters is a semantic quirk, not a bug to fix (§4.13).
7. `readout_levels`'s `free(ND_alg(agfstnode(g)))` is defensive and normally a no-op; a port may assert `ND_alg == None` (§4.17).
8. `decompose`'s pass-1 path indexes `GD_rankleader(subg)[ND_rank(v)]` without bounds checks — keep the invariant that a cluster's rankleaders cover `[GD_minrank, GD_maxrank]` (built by `build_ranks` in `expand_cluster`).

### 12.3 Suggested Rust shape (no behavior change)

* `Graph` keeps `nodes: Vec<Node>` with `id: u32` used for both `ND_id` and UF tie-breaking (§1.6); clusters as `Vec<ClusterId>` indexable 1..=n with a dummy 0.
* `Elist<EdgeId>` = `Vec<Option<EdgeId>>`-equivalent (`Vec<EdgeId>` + `len`), preserving the append/swap-remove/zap operations (`elist_append`, `zapinlist`, `alloc_elist`).
* Two union-finds: `uf_classic` (`ND_UF_parent`/`ND_UF_size`, smaller-id leader) and `uf_set` (dot2 `ND_set`, first-node leader). Both need the exact leader-selection rules of §9.
* `decompose` returns `Vec<Vec<NodeId>>` (component lists in order); `GD_comp.list[c]` maps to the head element; component iteration elsewhere swaps `GD_nlist` — model with an explicit `ComponentView` instead of mutating a head pointer.
* Determinism: iterate node sets in creation order everywhere (cgraph `agfstnode` order = creation order in the root graph); iterate subgraphs in declaration order.
