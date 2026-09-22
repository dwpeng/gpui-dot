# Graphviz `lib/common/ns.c` — Network Simplex Ranking: Exhaustive Implementation Spec

Source: `/tmp/graphviz-src/lib/common/ns.c` (1414 lines, current Graphviz master-line tree; EPL-2.0).
There is **no `lib/common/ns.h`** in this tree. The public API is declared in
`lib/common/render.h:133-134`:

```c
RENDER_API int rank(graph_t * g, int balance, int maxiter);                        // render.h:133
RENDER_API int rank2(graph_t * g, int balance, int maxiter, int search_size);      // render.h:134
```

Everything else in `ns.c` is `static`. The only production caller is
`lib/dotgen/rank.c:1107`: `rank2(Xg, 1, maxiter, ssize);` where `maxiter` comes from
`nslimit1` (default `INT_MAX`) and `ssize` is `atoi(agget(g,"searchsize"))` or `-1`
(`-1` → built-in default, see §3). `rank()` (ns.c:1029) is a thin legacy wrapper.

> **Terminology note for the porting checklist.** The task names some functions that do
> not exist in this revision; they map as follows:
> - `run()` main loop → `rank2()` main loop (ns.c:989-1006).
> - `mincut` / delta-direction logic → split across `update()` (ns.c:707-746),
>   `treeupdate()` (ns.c:675-689), `rerank()` (ns.c:691-702) and `merge_trees()`
>   (ns.c:594-615). There is no function named `mincut` here.
> - `beef_up()` → **absent** in this revision (removed upstream years ago). The initial
>   tight-tree construction is `find_tight_subtree()`/`tight_subtree_search()` +
>   `STheap`/union-find merge (`feasible_tree()`, ns.c:622-672).
> - `MAX_INT` checks → no `MAX_INT` identifier exists; the guards that exist are
>   `assert(LIST_SIZE(&ctx->Tree_edge) <= INT_MAX)` (ns.c:63), `INT_MAX` sentinels in
>   `dfs_enter_outedge`/`dfs_enter_inedge` (ns.c:218, 252), `INT_MAX`/`INT_MIN` in
>   `scan_and_normalize` (ns.c:749-750), and `SIZE_MAX` sentinels in the STheap code
>   (ns.c:313, 319, 441-443, 449, 557, 566, 569).
> - `LOWBIT` / `MAXSHORT` → **do not appear anywhere in ns.c**.
> - `search_edge_init` → absent; search state is the single `ctx->S_i` field initialized
>   to 0 by the ctx zero-init (ns.c:52, 893) and advanced by `leave_edge()`.

All line numbers below refer to `ns.c` unless prefixed (e.g. `types.h:445`).

---

## 1. Data model (the `ND_`/`ED_`/`GD_` fields used here)

All accessors are field macros over `Agnodeinfo_t`/`Agedgeinfo_t`/`Agraphinfo_t`
(`lib/common/types.h:393, 501-534, 581-603`). Exact declarations:

```c
#define GD_nlist(g)          (((Agraphinfo_t*)AGDATA(g))->nlist)          // types.h:393

#define ND_in(n)             (((Agnodeinfo_t*)AGDATA(n))->in)             // types.h:501
#define ND_lim(n)            (((Agnodeinfo_t*)AGDATA(n))->lim)            // types.h:504
#define ND_low(n)            (((Agnodeinfo_t*)AGDATA(n))->low)            // types.h:505
#define ND_mark(n)           (((Agnodeinfo_t*)AGDATA(n))->mark)           // types.h:507
#define ND_next(n)           (((Agnodeinfo_t*)AGDATA(n))->next)           // types.h:510
#define ND_node_type(n)      (((Agnodeinfo_t*)AGDATA(n))->node_type)      // types.h:511
#define ND_onstack(n)        (((Agnodeinfo_t*)AGDATA(n))->onstack)        // types.h:512
#define ND_out(n)            (((Agnodeinfo_t*)AGDATA(n))->out)            // types.h:515
#define ND_par(n)            (((Agnodeinfo_t*)AGDATA(n))->par)            // types.h:518
#define ND_priority(n)       (((Agnodeinfo_t*)AGDATA(n))->priority)       // types.h:522
#define ND_rank(n)           (((Agnodeinfo_t*)AGDATA(n))->rank)           // types.h:523
#define ND_tree_in(n)        (((Agnodeinfo_t*)AGDATA(n))->tree_in)        // types.h:533
#define ND_tree_out(n)       (((Agnodeinfo_t*)AGDATA(n))->tree_out)       // types.h:534

#define ED_cutvalue(e)       (((Agedgeinfo_t*)AGDATA(e))->cutvalue)       // types.h:581
#define ED_minlen(e)         (((Agedgeinfo_t*)AGDATA(e))->minlen)         // types.h:592
#define ED_tree_index(e)     (((Agedgeinfo_t*)AGDATA(e))->tree_index)     // types.h:600
#define ED_weight(e)         (((Agedgeinfo_t*)AGDATA(e))->weight)         // types.h:603
```

Underlying field types and meanings (types.h:445-470 for nodes, 543-576 for edges):

| Field | C type | Meaning in this algorithm | Value on entry to `rank2` |
|---|---|---|---|
| `ND_rank(n)` | `int` | assigned rank (layer) of node | caller-supplied (dot: 0 for all) |
| `ND_lim(n)` | `int` | max DFS pre/post index of n's subtree in tree | uninitialized (irrelevant; fully assigned by `dfs_range_init`) |
| `ND_low(n)` | `int` | min DFS index of n's subtree (≥ 1); **-1 is the "invalidated" marker** used by `invalidate_path` (ns.c:88-95) | uninitialized |
| `ND_par(n)` | `edge_t *` | parent tree edge of n in the spanning tree; NULL for root. **Aliased** as `subtree_t*` during feasible_tree via `ND_subtree` (ns.c:307-308) | uninitialized |
| `ND_tree_in(n)`, `ND_tree_out(n)` | `elist` = `{edge_t **list; size_t size;}` (types.h:270-273) | incident tree edges, stored with head/tail resp.; array is NULL-sentinel-terminated (`list[size] == NULL`) | allocated by `init_graph` (ns.c:914-918), `size = 0` |
| `ND_in(n)`, `ND_out(n)` | `elist` | full (non-tree) in/out edge lists; arrays are NULL-sentinel-terminated; iteration idiom is `for (i=0; (e=list[i]); i++)` (relies on the sentinel, not `.size`) | built by caller (dot) |
| `ND_priority(n)` | `int` | in-degree counter for topological `init_rank`; also used as "unprocessed in-edge count" | reset to 0 in `init_graph` (ns.c:905) then counted |
| `ND_mark(n)` | `size_t` (types.h:446 — note: *not* `bool`) | visited flag (DEBUG cycle check), tree membership while constructing | set `false` (=0) in `init_graph` (ns.c:895) |
| `ND_onstack(n)` | `char` (types.h:447) | DFS recursion stack flag, **DEBUG build only** (ns.c:1378, 1401, 1409) | uninitialized until `check_cycles` |
| `ND_node_type(n)` | `char` (types.h:445) | `NORMAL`==0 original input node, `VIRTUAL`==1 virtual node of long-edge chains, `SLACKNODE`==2, `REVERSED`==3, `FLATORDER`==4, `CLUSTER_EDGE`==5, `IGNORED`==6 (`lib/common/const.h:21-29`). Only `NORMAL` is special-cased here (ns.c:752, 829, 846, 851) | set by dot's compile step |
| `ND_next(n)` | `node_t *` | singly-linked global node list starting at `GD_nlist(g)`; **this is the canonical iteration order of every loop over nodes** | built by caller |
| `ED_minlen(e)` | `int` | minimum rank separation `rank(head) - rank(tail) ≥ minlen` | caller-supplied |
| `ED_weight(e)` | `int` | optimization weight | caller-supplied |
| `ED_cutvalue(e)` | `int` | tree-edge cut value; meaningful only while `ED_tree_index(e) ≥ 0`; after a swap the leaving edge keeps value 0, entering edge `-cutvalue` | zeroed for all edges in `init_graph` (ns.c:909) |
| `ED_tree_index(e)` | `int` | index of e in `ctx->Tree_edge`, **-1 = not a tree edge** (`TREE_EDGE(e) ⇔ index ≥ 0`) | set to -1 for all edges in `init_graph` (ns.c:910) |

`free_list(L)` is `free(L.list)` only (types.h:281-282) — it does **not** reset `.list`
or `.size`; after `freeTreeList`/`TB_balance` the elist fields hold dangling pointers
(harmless because nothing reads them afterwards; a Rust port simply drops the Vecs).

### 1.1 Context struct — `network_simplex_ctx_t` (ns.c:47-53)

```c
typedef struct {
    graph_t *G;
    LIST(edge_t *) Tree_edge;   // growable array of spanning-tree edges, order significant
    size_t S_i;                 /* search index for enter_edge */  // (actually for leave_edge; persists across calls)
    size_t N_edges, N_nodes;
    int Search_size;
} network_simplex_ctx_t;
```

- `Tree_edge`: `LIST(edge_t *)` — managed dynamic array (`lib/util/list.h`). Index of an
  edge in this array == its `ED_tree_index`. Order is append-order, mutated by
  `exchange_tree_edges` (ns.c:118) and read in that order by `leave_edge`/`LR_balance`.
- `S_i`: round-robin cursor for `leave_edge` (ns.c:184-211). Initialized to 0 by the
  aggregate zero-init at ns.c:893 (`*ctx = (network_simplex_ctx_t){.G = g};`) and in
  rank2's `network_simplex_ctx_t ctx = {0};` (ns.c:956). **Never reset** between
  iterations; wraps circularly.
- `N_nodes`, `N_edges`: counts computed in `init_graph` (ns.c:896-898).
- `Search_size`: how many negative-cut edges `leave_edge` may examine before returning
  the best seen (ns.c:192, 206).

### 1.2 Constants (verbatim)

```c
enum { SEARCHSIZE = 30 };                            // ns.c:55
#define LENGTH(e)      (ND_rank(aghead(e)) - ND_rank(agtail(e)))   // ns.c:42
#define SLACK(e)       (LENGTH(e) - ED_minlen(e))                  // ns.c:43
#define SEQ(a,b,c)     ((a) <= (b) && (b) <= (c))                  // ns.c:44
#define TREE_EDGE(e)   (ED_tree_index(e) >= 0)                     // ns.c:45
#define ND_subtree(n)      (subtree_t*)ND_par(n)                   // ns.c:307 (field borrow)
#define ND_subtree_set(n,value) (ND_par(n) = (edge_t*)value)       // ns.c:308
```

Support macros (verbatim from other headers):

```c
#define MIN(a,b)  ((a)<(b)?(a):(b))          // arith.h:28
#define MAX(a,b)  ((a)>(b)?(a):(b))          // arith.h:33  (args may be double-evaluated)
#define elist  { edge_t **list; size_t size; }               // types.h:270-273
#define free_list(L)          free(L.list)                   // types.h:282 (no pointer reset!)
#define PRISIZE_T "zu"                                        // prisize_t.h:25 (Linux)
```

`SWAP(a,b)` (gv_math.h:137) swaps the pointed-to values (memcpy-based; used only by
`STheapify`). `sadd_overflow(a,b,res)` (util/overflow.h:22-29) is
`__builtin_sadd_overflow` — checked signed 32-bit add, returns true on overflow.
`streq(a,b)` is `strcmp(a,b) == 0` (util/streq.h:14-16). `graphviz_exit(EXIT_FAILURE)`
terminates the process (util/exit.h). `Verbose` is a global `unsigned char`
(globals.h:53) — only affects stderr diagnostics, never ranks.

`LIST(...)` semantics used below (util/list.h):
- `LIST_APPEND`/`LIST_PUSH_BACK` — append at tail (may realloc; any outstanding
  `LIST_BACK` pointer into the list must be re-fetched, and the ns.c code is written so).
- `LIST_POP_FRONT` (removes+returns first), `LIST_POP_BACK`/`LIST_DROP_BACK` (removes last).
- `LIST_BACK(list)` = pointer to last element (mutation through it persists).
- `LIST_GET(list,i)` = value at i; `LIST_SET(list,i,v)`; `LIST_SIZE`; `LIST_IS_EMPTY`.
- `LIST_RESERVE(list,cap)` — preallocate; `LIST_FREE(list)` — clear + free.
- `LIST_SORT(list,cmp)` → `qsort` of the raw array (util/list.c:355-367) — **not stable**.

Allocators: `gv_calloc(n,sz)` zero-fills; `gv_alloc(sz)` == `gv_calloc(1,sz)`
(util/alloc.h:26,47).

### 1.3 Tight-tree support struct — `subtree_t` (ns.c:310-315)

```c
typedef struct subtree_s {
        node_t *rep;            /* some node in the tree */
        int    size;            /* total tight tree size */
        size_t    heap_index;   ///< required to find non-min elts when merged
        struct subtree_s *par;  /* union find */
} subtree_t;
```

- `rep`: seed node (first node of the subtree in nlist order).
- `size`: number of nodes in the subtree (accumulated by `tight_subtree_search`,
  updated by `STsetUnion`).
- `heap_index`: position in the min-heap; `SIZE_MAX` means "extracted / not on heap".
  `on_heap(tree) ⇔ tree->heap_index != SIZE_MAX` (ns.c:318-320).
- `par`: union-find parent; root satisfies `par == self` (set in `find_tight_subtree`,
  ns.c:415).

Per-DFS-frame state structs (all zero-initialized via C compound literals):
- `tst_t {Agnode_t *v; int in_i; int out_i; int rv;}` (ns.c:323-328).
- `state_t` inside `inter_tree_edge_search`: `{Agnode_t *v; subtree_t *ts; Agnode_t *from; int out_i; int in_i;}` (ns.c:457-463).
- `state_t` inside `dfs_cutval`: `{node_t *v; edge_t *par; int out_i; int in_i;}` (ns.c:1113-1118).
- `dfs_state_t {node_t *v; edge_t *par; int lim; int tree_out_i; int tree_in_i;}` (ns.c:1162-1168).

---

## 2. Function-by-function specification

### 2.1 `init_graph` — ns.c:890-921

```c
static bool init_graph(network_simplex_ctx_t *ctx, graph_t *g)
```

1. `*ctx = {0}` with `G = g` (893). (N_nodes=N_edges=0, S_i=0, Tree_edge empty,
   Search_size=0 at this point.)
2. Loop 1 — nodes in `GD_nlist` order via `ND_next` (894-899):
   - `ND_mark(n) = false`;
   - `ctx->N_nodes++`;
   - for `i = 0; ND_out(n).list[i] != NULL; i++`: `ctx->N_edges++`
     (counts every entry of the out-list once; a self-loop counts once).
3. `LIST_RESERVE(&ctx->Tree_edge, ctx->N_nodes)` (901).
4. Loop 2 (904-919), per node n:
   - `ND_priority(n) = 0` (905);
   - over `ND_in(n).list` while non-NULL (907-913), per in-edge e:
     `ND_priority(n)++`; `ED_cutvalue(e) = 0`; `ED_tree_index(e) = -1`;
     `if (ND_rank(aghead(e)) - ND_rank(agtail(e)) < ED_minlen(e)) feasible = false;`
     (feasibility test is per-edge: current ranks violate minlen ⇒ infeasible);
   - after the in-loop, `i` == in-degree:
     `ND_tree_in(n).list = gv_calloc(i + 1, sizeof(edge_t*))` (zeroed ⇒ sentinel at [0]),
     `ND_tree_in(n).size = 0` (914-915);
   - count out-degree the same way (916), then
     `ND_tree_out(n).list = gv_calloc(i + 1, sizeof(edge_t*))`, `size = 0` (917-918).
     So each node's tree lists have capacity `degree + 1` (degree of the *real* list).
5. Return `feasible` (920).

Notes: if the caller's rank assignment is already feasible, `init_rank` is skipped and
the caller's ranks (possibly not 0-normalized, possibly negative for virtual nodes) are
used as-is. If infeasible, `init_rank` overwrites all ranks.

### 2.2 `init_rank` — ns.c:145-177 (only called when !feasible)

Kahn topological assignment minimizing nothing — ranks = longest path from sources with
edge lengths `ED_minlen`:

1. `LIST(node_t*) Q`, `LIST_RESERVE(&Q, N_nodes)`; `size_t ctr = 0` (150-152).
2. Seed: scan nodes in nlist order; if `ND_priority(v) == 0` → `LIST_PUSH_BACK(&Q, v)`
   (154-157). (After `init_graph`, priority == in-degree; sources get in first.)
3. While Q non-empty (159-169):
   - `v = LIST_POP_FRONT(&Q)` (FIFO — BFS-ish order);
   - `ND_rank(v) = 0; ctr++;`
   - **in-edges first**: `for (i = 0; (e = ND_in(v).list[i]); i++)
     ND_rank(v) = MAX(ND_rank(v), ND_rank(agtail(e)) + ED_minlen(e));`
   - then **out-edges**: `for (i = 0; (e = ND_out(v).list[i]); i++)
     if (--ND_priority(aghead(e)) <= 0) LIST_PUSH_BACK(&Q, aghead(e));`
     (pre-decrement; `<= 0` is defensive — a node is enqueued exactly when its count
     reaches 0; nodes with in-degree 0 were already seeded and are never decremented).
4. If `ctr != ctx->N_nodes` (170-175) — i.e. the constraint graph has a directed cycle
   (should be impossible; dot breaks cycles first):
   `agerrorf("trouble in init_rank\n");` then for every node with
   `ND_priority(v) != 0`: `agerr(AGPREV, "\t%s %d\n", agnameof(v), ND_priority(v));`.
   Ranks of cycle nodes are left at whatever they got; execution continues.
5. `LIST_FREE(&Q)` (176).

### 2.3 `add_tree_edge` — ns.c:57-83

```c
static int add_tree_edge(network_simplex_ctx_t *ctx, edge_t * e)  // 0 = ok, -1 = error
```

1. `if (TREE_EDGE(e)) { agerrorf("add_tree_edge: missing tree edge\n"); return -1; }`
   (59-62) — i.e. adding an edge that already has `ED_tree_index >= 0` is an error.
2. `assert(LIST_SIZE(&ctx->Tree_edge) <= INT_MAX);`
   `ED_tree_index(e) = (int)LIST_SIZE(&ctx->Tree_edge);`
   `LIST_APPEND(&ctx->Tree_edge, e);` (63-65). Tree edges are numbered 0,1,2,… in
   insertion order.
3. Tail side (66-73):
   - `ND_mark(agtail(e)) = true;`
   - `ND_tree_out(n).list[ND_tree_out(n).size++] = e;` (write at old size, then increment)
   - `ND_tree_out(n).list[ND_tree_out(n).size] = NULL;` (restore sentinel)
   - `if (ND_out(n).list[ND_tree_out(n).size - 1] == 0) {
        agerrorf("add_tree_edge: empty outedge list\n"); return -1; }`
     This is a disguised capacity check: the tree-out array was sized
     `(out-degree + 1)` in init_graph, so this fires iff `tree_out.size > out-degree`,
     i.e. iff more tree edges leave n than there are real out edges — exactly when the
     next write would overflow. On this error path `size` has *already been incremented*
     and the edge is already in `Tree_edge` with a valid index; the state is corrupt, but
     every caller propagates the -1 and aborts the layout, so this is unreachable in
     practice. **Rust equivalent: push to `tree_out`, then
     `if n.tree_out.len() > n.out.len() { error }`.**
4. Head side (74-81): identical with `aghead(e)`, `ND_tree_in`, `ND_in`, message
   `"add_tree_edge: empty inedge list\n"`.
5. `return 0` (82).

Every tree-edge insertion also marks both endpoints (`ND_mark = true`).

### 2.4 `exchange_tree_edges` — ns.c:114-143 (swap leaving e for entering f)

```c
static void exchange_tree_edges(network_simplex_ctx_t *ctx, edge_t * e, edge_t * f)
```

1. `ED_tree_index(f) = ED_tree_index(e);` `assert(ED_tree_index(e) >= 0);`
   `LIST_SET(&ctx->Tree_edge, (size_t)ED_tree_index(e), f);` `ED_tree_index(e) = -1;`
   (116-119). f inherits e's slot; array order preserved.
2. Remove e from tail's tree-out (121-128):
   - `size_t i = --ND_tree_out(n).size;` (n = agtail(e); i = index of last occupied slot)
   - `for (j = 0; j <= i; j++) if (ND_tree_out(n).list[j] == e) break;` (linear search,
     e guaranteed present in `[0..i]`)
   - `list[j] = list[i]; list[i] = NULL;` (swap-with-last + restore sentinel;
     **order of the remaining tree-out list is otherwise arbitrary**).
3. Remove e from head's tree-in (129-135): same code with `ND_tree_in`/`aghead`.
4. Append f to tail's tree-out and head's tree-in (137-142):
   `list[size++] = f; list[size] = NULL;` for each. (No capacity check here; capacity is
   guaranteed by init_graph sizing.)

Note: no ranks or cut values are touched here.

### 2.5 `leave_edge` — ns.c:179-213 (choose leaving tree edge)

```c
static edge_t *leave_edge(network_simplex_ctx_t *ctx)
```

Round-robin scan of `ctx->Tree_edge` starting at the persistent cursor `ctx->S_i`:

1. `rv = NULL; cnt = 0; j = ctx->S_i;` (181-184).
2. **Phase 1** (185-196): `while (ctx->S_i < LIST_SIZE(&ctx->Tree_edge))`:
   - `f = LIST_GET(&ctx->Tree_edge, ctx->S_i);`
   - `if (ED_cutvalue(f) < 0)`:
     - if `rv == NULL` → `rv = f`;
       else if `ED_cutvalue(rv) > ED_cutvalue(f)` → `rv = f`
       (**strict `>` ⇒ the earliest-scanned edge wins ties**);
     - `if (++cnt >= ctx->Search_size) return rv;` — after examining `Search_size`
       negative-cut edges, return immediately (early exit).
   - `ctx->S_i++`.
3. **Phase 2, wrap-around** (197-211): only if `j > 0`: set `ctx->S_i = 0` and scan
   `while (ctx->S_i < j)` with the identical body (same tie rule, same cnt budget).
4. `return rv` (212).

Exact cursor end-states (must be preserved for bit-identical iteration order):
- early exit (budget hit): `S_i` points just past the edge that filled the budget;
- phase 1 exhausted with `j == 0`: `S_i == LIST_SIZE`. On the *next* call, `j` becomes
  `LIST_SIZE`, phase 1's `while` is skipped immediately, and — provided
  `LIST_SIZE > 0` — phase 2 scans the whole list `[0, LIST_SIZE)` and ends with
  `S_i == LIST_SIZE` again (a full scan every call, cursor effectively pinned at
  `LIST_SIZE`). Only when `LIST_SIZE == 0` does `leave_edge` return NULL with no scan;
- wrapped fully (phase 2 ran to completion): `S_i == j` (unchanged for next call).
Selection rule overall: most-negative cut value among negative-cut tree edges, earliest
in scan order on ties, scanning at most `Search_size` negative edges starting at `S_i`.
Returns NULL iff no tree edge has negative cut value (⇔ tree optimal).

### 2.6 `dfs_enter_outedge` / `dfs_enter_inedge` — ns.c:215-246, 248-280

```c
static edge_t *dfs_enter_outedge(node_t *v, int Low, int Lim);
static edge_t *dfs_enter_inedge (node_t *v, int Low, int Lim);
```

Iterative DFS over the *subtree below* `v` (the side of the leaving edge that will
move), looking for the minimum-slack non-tree edge that crosses the cut
`[Low..Lim]` (the DFS interval of v's subtree).

Common shape (outedge version; numbers 215-246):
1. `Enter = NULL; Slack = INT_MAX;` `todo = {0}` stack; `LIST_APPEND(&todo, v)`.
2. While `todo` non-empty:
   - `v = LIST_POP_BACK(&todo)` (**LIFO** ⇒ depth-first; nodes may be pushed/visited
     multiple times — there is no visited set).
   - Out-edge scan, **full pass, in list order** (226-237), per
     `e = ND_out(v).list[i]` while non-NULL:
     - if `!TREE_EDGE(e)`:
       `if (!SEQ(Low, ND_lim(aghead(e)), Lim))` — head is *outside* v's interval ⇒
       crossing candidate: `slack = SLACK(e);`
       `if (slack < Slack || Enter == NULL) { Enter = e; Slack = slack; }`
       (strict `<` plus `Enter == NULL` disjunct ⇒ earliest candidate wins ties; the
       `Enter == NULL` term matters only if `slack == INT_MAX`);
     - else (tree edge): `if (ND_lim(aghead(e)) < ND_lim(v))
       LIST_APPEND(&todo, aghead(e));` — descend only to tree children (smaller lim ⇒
       away from root).
   - Tree-in seeding pass (238-240):
     `for (i = 0; (e = ND_tree_in(v).list[i]) && Slack > 0; i++)
        if (ND_lim(agtail(e)) < ND_lim(v)) LIST_APPEND(&todo, agtail(e));`
     Two subtleties: (a) the condition fetches e first, then requires **`Slack > 0`** —
     once a zero-slack candidate has been found, no further subtree nodes are seeded
     (early stop; note `Slack == 0` stops the *seeding*, not the out-scan of the current
     node); (b) this pushes tree children reachable via in-edges (edges whose head is v
     and whose tail has smaller lim — the subtree side), *regardless of tree-edge
     direction*, so the descent covers the whole subtree.
3. `LIST_FREE(&todo); return Enter;`

`dfs_enter_inedge` (248-280) is the exact mirror:
- scans `ND_in(v).list` (260-271): non-tree e is a candidate iff
  `!SEQ(Low, ND_lim(agtail(e)), Lim)`; tree edges descend via
  `if (ND_lim(agtail(e)) < ND_lim(v)) push agtail(e)`;
- seeding pass over `ND_tree_out(v).list` (272-274):
  `for (i = 0; (e = ND_tree_out(v).list[i]) && Slack > 0; i++)
     if (ND_lim(aghead(e)) < ND_lim(v)) push aghead(e);`.

### 2.7 `enter_edge` — ns.c:282-297

```c
static edge_t *enter_edge(edge_t *e) {
    /* v is the down node */
    if (ND_lim(agtail(e)) < ND_lim(aghead(e))) { v = agtail(e); outsearch = false; }
    else                                       { v = aghead(e); outsearch = true;  }
    if (outsearch) return dfs_enter_outedge(v, ND_low(v), ND_lim(v));
    return           dfs_enter_inedge (v, ND_low(v), ND_lim(v));
}
```

- "Down node" = endpoint with the smaller `ND_lim` (the root of the moving subtree).
- Direction rule (verbatim): if the **tail** of the leaving edge is the down node →
  `outsearch = false` → `dfs_enter_inedge`; otherwise head is down →
  `dfs_enter_outedge`. (`ND_lim` values are unique per node, so the equality case never
  occurs for distinct endpoints.)
- Scan window is always `[ND_low(v), ND_lim(v)]` of the down node.

### 2.8 Tight-tree construction: `tight_subtree_search`, `find_tight_subtree` — ns.c:331-417

`tst_t` frame = `{v, in_i, out_i, rv}` (rv = node count accumulated in the frame's
subtree). `ND_subtree` (== `ND_par`) holds the owning `subtree_t*` during this phase.

`tight_subtree_search(ctx, v, st) → int` (331-404) — iterative post-order DFS growing a
maximal set of zero-slack (tight) edges from `v`:

1. `rv = 1; ND_subtree_set(v, st); push {v, rv: 1}` (in_i = out_i = 0).
2. While stack non-empty (341-399), with `top = LIST_BACK(&todo)` (a live pointer):
   a. **In-edge scan** (345-364): `for (; (e = ND_in(top->v).list[top->in_i]); top->in_i++)`:
      - `if (TREE_EDGE(e)) continue;`
      - `if (ND_subtree(agtail(e)) == 0 && SLACK(e) == 0)` — unclaimed tail + tight:
        - `add_tree_edge(ctx, e) != 0` (error): `LIST_DROP_BACK(&todo);` then
          if stack now empty → `rv = -1` else `--LIST_BACK(&todo)->rv;`
          (**the failing edge's index is NOT consumed** — `in_i` stays);
        - success: `++top->in_i;` (consume e) `ND_subtree_set(agtail(e), st);`
          push `{v: agtail(e), rv: 1}`;
        - `updated = true; break;`
   b. If `updated` → `continue` (re-fetch top).
   c. **Out-edge scan** (369-388): exact mirror over `ND_out`/`aghead` with `out_i`
      (claim condition `ND_subtree(aghead(e)) == 0 && SLACK(e) == 0`).
   d. If `updated` → `continue`.
   e. **Pop** (393-398): `last = LIST_POP_BACK(&todo);` if stack now empty →
      `rv = last.rv;` else `LIST_BACK(&todo)->rv += last.rv;`
3. `LIST_FREE(&todo); return rv;` — total node count of the grown subtree (≥ 1), or -1
   if `add_tree_edge` failed.

`find_tight_subtree(ctx, v)` (406-417): allocate zeroed `subtree_t`; `rep = v`;
`size = tight_subtree_search(ctx, v, self)`; if `size < 0` → free, return NULL;
`par = self` (union-find root); return self.

### 2.9 Union-find + heap: `STsetFind`, `STsetUnion`, STheap — ns.c:424-574

`STsetFind(n0)` (424-432):
```
s0 = ND_subtree(n0);
while (s0->par != NULL && s0->par != s0) {
    if (s0->par->par != NULL) s0->par = s0->par->par;   // one-level path compression
    s0 = s0->par;
}
return s0;
```

`STsetUnion(s0, s1)` (434-451):
1. Find roots r0, r1 by walking `par` chains *without* compression
   (`for (r0 = s0; r0->par && r0->par != r0; r0 = r0->par);`).
2. `if (r0 == r1) return r0;` ("safety code but shouldn't happen").
3. Root selection (verbatim precedence):
   ```
   assert(on_heap(r0) || on_heap(r1));
   if (!on_heap(r1))      r = r0;
   else if (!on_heap(r0)) r = r1;
   else if (r1->size < r0->size) r = r0;
   else                          r = r1;
   ```
4. `r0->par = r1->par = r; r->size = r0->size + r1->size; assert(on_heap(r)); return r;`

STheap (min-heap keyed on `subtree_t.size`, array is the caller's `tree` vector,
mutated in place):
- `STheapsize(heap) = heap->size` (532).
- `STheapify(heap, i)` (534-550):
  ```
  do {
      left = 2*(i+1) - 1; right = 2*(i+1); smallest = i;
      if (left  < heap->size && elt[left]->size  < elt[smallest]->size) smallest = left;
      if (right < heap->size && elt[right]->size < elt[smallest]->size) smallest = right;
      if (smallest != i) {
          SWAP(&elt[i], &elt[smallest]);
          elt[i]->heap_index = i; elt[smallest]->heap_index = smallest;
          i = smallest;
      } else break;
  } while (i < heap->size);
  ```
  (strict `<` ⇒ left bias; ties keep the higher-indexed/parent element).
- `STbuildheap(elt, size)` (552-560): set `elt[i]->heap_index = i` for all i; then
  `for (size_t i = size/2; i != SIZE_MAX; i--) STheapify(heap, i);` (unsigned wrap ⇒
  runs for i = size/2 … 0 inclusive).
- `STextractmin(heap)` (562-574):
  ```
  rv = elt[0]; rv->heap_index = SIZE_MAX;          // mark as removed
  elt[0] = elt[heap->size - 1]; elt[0]->heap_index = 0;
  elt[heap->size - 1] = rv;                        /* needed to free storage later */
  heap->size--; STheapify(heap, 0); return rv;
  ```
  **Quirk (dead in practice):** if called with `size == 1`, the two self-assignments
  overwrite `rv->heap_index` back to `0`, so the removed subtree would report
  `on_heap == true`. `feasible_tree` only extracts while `heap->size > 1`, so this
  branch is unreachable; a faithful port may replicate or ignore it.

### 2.10 `inter_tree_edge_search` / `inter_tree_edge` — ns.c:454-530

Explicit DFS over the *whole current tree structure* (via `ND_out`/`ND_in`, both tree
and non-tree edges) from `tree->rep`, hunting the minimum-slack edge connecting two
different trees of the union-find forest. Frame = `{v, ts, from, out_i, in_i}` where
`ts = STsetFind(v)` at push time and `from` = predecessor node (NULL for the root).

**Port trap (fixed 2025)**: `ts` must be the *live* `STsetFind` root. Comparing the
original `ND_subtree` id captured by `tight_subtree_search` instead makes two
already-merged subtrees look distinct, so the search returns an edge whose endpoints
are already connected; `merge_trees` then adds it and the "spanning tree" acquires a
cycle. `init_cutvalues`' DFS (`dfs_range_init`) never terminates on a cycle — the
frame stack simply grows — and the position phase hangs forever (1.dot with
Graphviz's own box sizes). `feasible_tree` therefore now also verifies, *before*
`init_cutvalues`, that the tree lists reach all `N_nodes` nodes with exactly
`N_nodes-1` tree edges, degrading a broken tree to C's Err(1) →
`connectGraph`-and-retry path instead of looping.

1. Push `{v, ts: STsetFind(v)}`; `best = NULL` (465-468).
2. While stack non-empty, `s = LIST_BACK(&todo)` (470-471):
   a. **Prune** (472-475): `if (s->out_i == 0 && s->in_i == 0 && best != NULL &&
      SLACK(best) == 0) { LIST_DROP_BACK(&todo); continue; }`
      (a fresh frame is abandoned immediately if a zero-slack candidate already exists).
   b. Out scan (479-495): `for (; (e = ND_out(s->v).list[s->out_i]) != NULL; ++s->out_i)`:
      - `TREE_EDGE(e)`: `if (aghead(e) == s->from) continue;` (don't search back);
        else `++s->out_i;` push `{v: aghead(e), ts: STsetFind(aghead(e)), from: s->v};`
        `updated = true; break;`
      - non-tree: `if (STsetFind(aghead(e)) != s->ts)` — edge to a *different* tree:
        `if (best == NULL || SLACK(e) < SLACK(best)) best = e;` (strict `<` ⇒ first
        found wins ties). Same-tree non-tree edges are ignored.
   c. If `updated` → `continue`.
   d. In scan (501-515): exact mirror over `ND_in`/`agtail`
      (back-edge test `agtail(e) == s->from`; push `{v: agtail(e), from: s->v}`).
   e. `LIST_DROP_BACK(&todo)` (520).
3. `LIST_FREE(&todo); return best;`

`inter_tree_edge(tree) = inter_tree_edge_search(tree->rep)` (527-530).
Note: tree edges are *traversed* even between different union-find sets? No — tree edges
always have both ends in the same set by construction (each `add_tree_edge` is followed
by a union), so descending a tree edge stays inside one tree; `STsetFind` is still
recomputed per push.

### 2.11 `tree_adjust` — ns.c:576-591

Recursive (C stack, depth = tree height):
```
ND_rank(v) += delta;
for (i = 0; (e = ND_tree_in(v).list[i]); i++)  { w = agtail(e); if (w != from) tree_adjust(w, v, delta); }
for (i = 0; (e = ND_tree_out(v).list[i]); i++) { w = aghead(e); if (w != from) tree_adjust(w, v, delta); }
```
in-list scanned before out-list; `from == NULL` initially means "visit everything".

### 2.12 `merge_trees` — ns.c:593-615

```c
static subtree_t *merge_trees(network_simplex_ctx_t *ctx, Agedge_t *e) /* entering tree edge */
```
1. `assert(!TREE_EDGE(e));`
2. `t0 = STsetFind(agtail(e)); t1 = STsetFind(aghead(e));`
3. **Delta direction** (which component's ranks move):
   - `if (!on_heap(t0))` (t0 already extracted from the heap ⇒ it is the *old* tree):
     `delta = SLACK(e);` `if (delta != 0) tree_adjust(t0->rep, NULL, delta);`
     (move t0 by +slack so that e becomes tight);
   - `else`: `delta = -SLACK(e);` `if (delta != 0) tree_adjust(t1->rep, NULL, delta);`
4. `if (add_tree_edge(ctx, e) != 0) return NULL;`
5. `return STsetUnion(t0, t1);`

### 2.13 `feasible_tree` — ns.c:622-672

```c
static int feasible_tree(network_simplex_ctx_t *ctx)
/* Return 1 if input graph is not connected; 0 on success; 2 on serious error. */
```
1. For all n in nlist order: `ND_subtree_set(n, 0)` (631-633).
2. `tree = gv_calloc(ctx->N_nodes, sizeof(subtree_t*)); subtree_count = 0;` (626, 635).
3. For all n in nlist order (637-646): if `ND_subtree(n) == 0`:
   `tree[subtree_count] = find_tight_subtree(ctx, n);`
   if NULL → `error = 2; goto end;` else `subtree_count++`.
   (Each node is claimed by exactly one tight subtree; nodes whose search yields only
   themselves form singleton subtrees.)
4. `heap = STbuildheap(tree, subtree_count);` (649).
5. `while (STheapsize(heap) > 1)` (650-662) — always merge the *smallest* current tree:
   - `tree0 = STextractmin(heap);`
   - `ee = inter_tree_edge(tree0);` `if (ee == NULL) { error = 1; break; }`
     (graph not connected with respect to tight trees ⇒ no feasible spanning tree);
   - `tree1 = merge_trees(ctx, ee);` `if (tree1 == NULL) { error = 2; break; }`
   - `STheapify(heap, tree1->heap_index);` (restore heap property below the merged root).
6. `end:` (664-668): `free(heap); for (i = 0; i < subtree_count; i++) free(tree[i]);
   free(tree); if (error) return error;`
7. `assert(LIST_SIZE(&ctx->Tree_edge) == ctx->N_nodes - 1);` (669) — spanning tree has
   n-1 edges. (If `N_nodes == 0`, `N_nodes - 1` underflows `size_t`; the assert fires in
   debug builds and is skipped in release. An empty graph is otherwise handled:
   `STbuildheap(…, 0)` and the `while` are no-ops.)
8. `init_cutvalues(ctx); return 0;` (670-671).

### 2.14 `init_cutvalues`, `dfs_range_init` — ns.c:299-303, 1176-1237

`init_cutvalues(ctx)`:
```
dfs_range_init(GD_nlist(ctx->G));
dfs_cutval(GD_nlist(ctx->G), NULL);
```

`dfs_range_init(v) → int` — iterative DFS assigning `ND_par`, `ND_low`, `ND_lim` over
the spanning tree, rooted at the **first node of the nlist** (`GD_nlist`):

1. `lim = 0; ND_par(v) = NULL; ND_low(v) = 1;`
   push `{v, par: NULL, lim: 1}` (`tree_out_i = tree_in_i = 0`).
2. While stack non-empty (1186-1232), `s = LIST_BACK(&todo)`; `pushed_new = false`:
   a. `while (ND_tree_out(s->v).list[s->tree_out_i])` (1190-1202):
      `e = list[s->tree_out_i]; ++s->tree_out_i;`
      `if (e != s->par) { n = aghead(e); ND_par(n) = e; ND_low(n) = s->lim;
        push {n, par: e, lim: s->lim}; pushed_new = true; break; }`
      (the child's `low` is seeded with the parent's *current* lim).
   b. If `pushed_new` → continue.
   c. Mirror over `ND_tree_in` with `agtail` (1207-1219).
   d. Close the node (1224-1231): `ND_lim(s->v) = s->lim; lim = s->lim;
      LIST_DROP_BACK(&todo); if (!LIST_IS_EMPTY(&todo)) LIST_BACK(&todo)->lim = lim + 1;`
      (each completion bumps the parent's lim so sibling subtrees get disjoint ranges).
3. `LIST_FREE(&todo); return lim + 1;` (1236) — return value ignored by the caller.

Result invariants: `ND_low(n) ≥ 1`; each node's `[low, lim]` interval is the contiguous
DFS index range of its subtree; root `lim` == total number of tree nodes; intervals of
siblings are disjoint; for any node x: `x` inside subtree of n ⇔
`SEQ(ND_low(n), ND_lim(x), ND_lim(n))` (this is exactly the `SEQ` test used everywhere).

There is **no** memoization in `dfs_range_init` (unlike `dfs_range`): it always walks
the entire tree, and it never reads prior `ND_par`/`ND_low` values.

### 2.15 `dfs_cutval`, `x_cutval`, `x_val` — ns.c:1110-1159, 1043-1070, 1072-1108

`dfs_cutval(v, par)` (1110-1159) — iterative post-order over tree children; computes
`ED_cutvalue` of every tree edge bottom-up. Frame `{v, par, out_i, in_i}`:

1. Push `{v, par}`.
2. While stack non-empty, `top = LIST_BACK`:
   a. Tree-out scan (1128-1135): `for (; (e = ND_tree_out(top->v).list[top->out_i]);
      ++top->out_i) if (e != top->par) { ++top->out_i; push {aghead(e), par: e};
      updated = true; break; }`
   b. If updated continue.
   c. Tree-in scan (1140-1147): mirror with `agtail`.
   d. Close (1152-1155): `if (top->par) x_cutval(top->par); LIST_DROP_BACK(&todo);`
   ⇒ a parent edge's cut value is computed only after all deeper edges are done
   ("assuming values of edges on one side were already set", ns.c:1042).
3. `LIST_FREE(&todo)`.

`x_cutval(f)` (1043-1070):
1. Pick the already-searched side (1049-1056):
   `if (ND_par(agtail(f)) == f) { v = agtail(f); dir = 1; }
    else                        { v = aghead(f); dir = -1; }`
2. `sum = 0`; scan **out-edges of v then in-edges of v** (1059-1068), each contributing
   `x_val(e, v, dir)`, added with overflow check:
   `if (sadd_overflow(sum, x_val(e, v, dir), &sum)) {
      agerrorf("overflow when computing edge weight sum\n");
      graphviz_exit(EXIT_FAILURE); }` — **the process exits** on overflow.
3. `ED_cutvalue(f) = sum;` (1069).

`x_val(e, v, dir) → int` (1072-1108):
1. `other = (agtail(e) == v) ? aghead(e) : agtail(e);`
2. Inside test: `if (!(SEQ(ND_low(v), ND_lim(other), ND_lim(v))))`
   — `other` is outside v's subtree:
   `f = 1; rv = ED_weight(e);`
   else (inside): `f = 0; rv = (TREE_EDGE(e) ? ED_cutvalue(e) : 0); rv -= ED_weight(e);`
3. Sign: `if (dir > 0) d = (aghead(e) == v) ? 1 : -1;
   else       d = (agtail(e) == v) ? 1 : -1;`
   `if (f) d = -d;`
4. `if (d < 0) rv = -rv;` `return rv;`

Semantics: with `dir = 1` (tail side already done) an edge leaving v's component to the
outside contributes `+weight` if it points "forward" (v is its tail) else `-weight`;
inside edges contribute their cut value minus weight with opposite orientation sign.
This is the classical cut-value formula
`cut(e) = Σ_{forward} w − Σ_{backward} w` computed relative to the already-processed side.

### 2.16 `treeupdate` — ns.c:675-689

```c
/* walk up from v to LCA(v,w), setting new cutvalues. */
static Agnode_t *treeupdate(Agnode_t *v, Agnode_t *w, int cutvalue, bool dir)
```
```
while (!SEQ(ND_low(v), ND_lim(w), ND_lim(v))) {      // while w is outside v's subtree
    e = ND_par(v);
    d = (v == agtail(e)) ? dir : !dir;               // edge orientation vs. walk direction
    if (d) ED_cutvalue(e) += cutvalue;
    else   ED_cutvalue(e) -= cutvalue;
    v = (ND_lim(agtail(e)) > ND_lim(aghead(e))) ? agtail(e) : aghead(e);  // move to parent node
}
return v;                                            // the LCA
```
(`dir` is `true` when walking from `agtail(f)`; false from `aghead(f)`.) No overflow
guard on the cut-value additions here.

### 2.17 `rerank` — ns.c:691-702

Recursive:
```
ND_rank(v) -= delta;
for (i = 0; (e = ND_tree_out(v).list[i]); i++) if (e != ND_par(v)) rerank(aghead(e), delta);
for (i = 0; (e = ND_tree_in (v).list[i]); i++) if (e != ND_par(v)) rerank(agtail(e), delta);
```
⇒ adds `-delta` to the ranks of exactly the tree component containing v (excluding the
parent side). Tree-out scanned before tree-in; recursion depth = tree height.

### 2.18 `invalidate_path` — ns.c:90-112

```
while (true) {
    if (ND_low(to_node) == -1) break;          // already invalidated
    ND_low(to_node) = -1;                      // mark invalidated
    e = ND_par(to_node);
    if (e == NULL) break;                      // reached root
    if (ND_lim(to_node) >= ND_lim(lca)) {
        if (to_node != lca) agerrorf("invalidate_path: skipped over LCA\n");
        break;                                 // walked past/at the LCA: stop
    }
    to_node = (ND_lim(agtail(e)) > ND_lim(aghead(e))) ? agtail(e) : aghead(e);  // parent node
}
```
"borrow field … Assigns ND_low(n) = -1 for the affected nodes" (comment ns.c:85-88).
`-1` cannot collide with a real `low` (real lows ≥ 1).

### 2.19 `update` — ns.c:707-746 (exchange e ← f, fix ranks and cut values)

```c
static int update(network_simplex_ctx_t *ctx, edge_t * e, edge_t * f)
```
1. `delta = SLACK(f);` (710) — amount by which the moving side must shift to make f
   tight (f was chosen with minimal slack ≥ 0).
2. **Rank shift direction** (711-727) — "for (v = in nodes in tail side of e) do
   ND_rank(v) -= delta;":
   ```
   if (delta > 0) {
       s = ND_tree_in(agtail(e)).size + ND_tree_out(agtail(e)).size;
       if (s == 1) rerank(agtail(e), delta);            // tail is a tree leaf: shift tail side
       else {
           s = ND_tree_in(aghead(e)).size + ND_tree_out(aghead(e)).size;
           if (s == 1) rerank(aghead(e), -delta);       // head is a leaf: shift head side by +
           else if (ND_lim(agtail(e)) < ND_lim(aghead(e)))
                        rerank(agtail(e), delta);       // tail is the down node (smaller lim)
           else        rerank(aghead(e), -delta);       // head is the down node
       }
   }
   ```
   (if `delta == 0` nothing is reranked). `rerank(x, d)` subtracts d from x's component;
   so "tail side moves down by delta" or "head side moves up by delta" — pick a
   single-node side first (leaf test), otherwise rerank the endpoint with the **smaller
   `ND_lim`** (the down node — the side of the tree away from the root; same down-node
   convention as `enter_edge`, ns.c:286-293).
3. Cut-value updates (729-734):
   ```
   cutvalue = ED_cutvalue(e);
   lca = treeupdate(agtail(f), aghead(f), cutvalue, true);
   if (treeupdate(aghead(f), agtail(f), cutvalue, false) != lca) {
       agerrorf("update: mismatched lca in treeupdates\n");
       return 2;                                        // fatal: rank2 aborts with code 2
   }
   ```
4. Prune bookkeeping (736-739): `lca_low = ND_low(lca);
   invalidate_path(lca, aghead(f)); invalidate_path(lca, agtail(f));`
   (order: head first, then tail).
5. Swap (741-743): `ED_cutvalue(f) = -cutvalue; ED_cutvalue(e) = 0;
   exchange_tree_edges(ctx, e, f);`
6. Re-run the interval DFS only over the damaged region (744):
   `dfs_range(lca, ND_par(lca), lca_low);` — return 0 (745).

### 2.20 `dfs_range` — ns.c:1242-1316 (incremental re-DFS)

`dfs_range(v, par, low) → int`:
1. Memoized short-circuit (1246-1248):
   `if (ND_par(v) == par && ND_low(v) == low) return ND_lim(v) + 1;`
2. `lim = 0; ND_par(v) = par; ND_low(v) = low; push {v, par, lim: low}`.
3. Loop (1257-1311), `s = LIST_BACK`, `processed_child = false`:
   a. Tree-out scan (1261-1277): `while (ND_tree_out(s->v).list[s->tree_out_i])`:
      `e = …; ++s->tree_out_i; if (e != s->par) { n = aghead(e);
        if (ND_par(n) == e && ND_low(n) == s->lim) s->lim = ND_lim(n) + 1;   // reuse subtree
        else { ND_par(n) = e; ND_low(n) = s->lim; push {n, e, s->lim}; }
        processed_child = true; break; }`
   b. If processed_child continue.
   c. Mirror over `ND_tree_in`/`agtail` (1282-1298).
   d. Close (1303-1310): `ND_lim(s->v) = s->lim; lim = s->lim; LIST_DROP_BACK(&todo);
      if (!LIST_IS_EMPTY(&todo)) LIST_BACK(&todo)->lim = lim + 1;`
4. `LIST_FREE(&todo); return lim + 1;` (caller `update` ignores it).

The reuse test (`ND_par(n) == e && ND_low(n) == s->lim`) is what makes the update O(α):
subtrees whose parent-edge and low value are unchanged from the previous DFS are skipped,
their intervals consumed in O(1). `invalidate_path` sets `low = -1` precisely to break
that equality and force re-descents where intervals changed.

### 2.21 `scan_and_normalize` — ns.c:748-761

```
Minrank = INT_MAX; Maxrank = INT_MIN;
for n in nlist: if (ND_node_type(n) == NORMAL) { Minrank = MIN(Minrank, ND_rank(n));
                                                 Maxrank = MAX(Maxrank, ND_rank(n)); }
for n in nlist: ND_rank(n) -= Minrank;          // all nodes, incl. virtual
Maxrank -= Minrank;
return Maxrank;                                  // == max rank after normalization
```
Order: min/max pass over NORMAL nodes first, then a second full pass subtracts. If the
graph has no NORMAL nodes, Minrank stays INT_MAX (all ranks shift by INT_MAX — no
crash; Maxrank becomes INT_MIN − INT_MAX). Empty graph ⇒ same no-op shift.
`ND_rank` is `int`; the subtraction is unguarded (wraps in practice).

### 2.22 `freeTreeList` / `reset_lists` — ns.c:763-776

```
static void reset_lists(ctx) { LIST_FREE(&ctx->Tree_edge); }        // 763-765

static void freeTreeList(ctx, g) {                                   // 767-776
    for (n = GD_nlist(g); n; n = ND_next(n)) {
        free_list(ND_tree_in(n));      // free(n.tree_in.list) — pointer/size NOT reset
        free_list(ND_tree_out(n));
        ND_mark(n) = false;
    }
    reset_lists(ctx);
}
```
Note: frees every node's tree lists (NORMAL and VIRTUAL alike) — unlike `TB_balance`,
which frees only NORMAL nodes' lists (see §2.24).

### 2.23 `LR_balance` — ns.c:778-796 (balance == 2)

```
for (i = 0; i < LIST_SIZE(&ctx->Tree_edge); i++) {         // tree edges in index order
    e = LIST_GET(&ctx->Tree_edge, i);
    if (ED_cutvalue(e) == 0) {                             // only degenerate (ties) edges
        f = enter_edge(e);
        if (f == NULL) continue;
        delta = SLACK(f);
        if (delta <= 1) continue;                          // need ≥ 2 slack to gain anything
        if (ND_lim(agtail(e)) < ND_lim(aghead(e)))
             rerank(agtail(e), delta / 2);                 // C truncation; delta > 1 here so ≥ 1
        else rerank(aghead(e), -delta / 2);
    }
}
freeTreeList(ctx, ctx->G);
```
Note: **no rank normalization is applied** in the balance==2 path of `rank2` — ranks may
end up un-normalized (dot's consumer handles it). Total tree cost is unchanged only for
zero-cut edges with slack ≥ 2 entering edges.

### 2.24 `TB_balance` — ns.c:814-888 (balance == 1; also the dot default)

```
adj = 0;
Maxrank = scan_and_normalize(ctx);                    // 820: normalizes ranks first
assert(Maxrank >= 0);                                 // 823
nrank = gv_calloc((size_t)Maxrank + 1, sizeof(int));  // 824: per-rank occupancy

s = agget(ctx->G, "TBbalance");                       // 825 — NULL if attribute absent
if (s) { if (streq(s,"min")) adj = 1; else if (streq(s,"max")) adj = 2; }
if (adj) for n in nlist: if NORMAL:
      if (ND_in(n).size == 0  && adj == 1) ND_rank(n) = 0;        // 830-832
      if (ND_out(n).size == 0 && adj == 2) ND_rank(n) = Maxrank;  // 833-835
```

Sorting pass (838-843): `Tree_node` = all nodes in nlist order, then
`LIST_SORT(&Tree_node, adj > 1 ? decreasingrankcmpf : increasingrankcmpf);`
- `decreasingrankcmpf` (798-808): `rank(n1) < rank(n0) → -1; rank(n1) > rank(n0) → 1; else 0`
  (descending by rank).
- `increasingrankcmpf` (810-812): negation (ascending).
- **`LIST_SORT` is `qsort` — NOT stable**; the relative order of equal-rank nodes is
  unspecified and platform-libc dependent. For bit-identical output the Rust port must
  reproduce the *same* qsort (glibc) behavior or, pragmatically, any total order —
  equal-rank nodes influence each other only through the `nrank` occupancy counters.
- `adj == 0` → ascending (increasingrankcmpf), because `adj > 1` is false.

Counting pass (844-848): `for i in 0..len: n = Tree_node[i]; if NORMAL: nrank[ND_rank(n)]++;`

Main pass (849-885), over `Tree_node` in sorted order, NORMAL nodes only:
```
inweight = Σ ED_weight(e) over ND_in(n).list (in list order);      // 857-860
low     = MAX over in-edges of (ND_rank(agtail(e)) + ED_minlen(e)); // starts 0
outweight= Σ ED_weight(e) over ND_out(n).list (in list order);     // 861-864
high    = MIN over out-edges of (ND_rank(aghead(e)) - ED_minlen(e)); // starts Maxrank
if (low < 0) low = 0;                    // 865-866: "vnodes can have ranks < 0"
if (adj) { if (inweight == outweight) ND_rank(n) = (adj == 1 ? low : high); }   // 867-870
else {
    if (inweight == outweight) {         // 872-880: move to least-populated feasible rank
        choice = low;
        for (i = low + 1; i <= high; i++) if (nrank[i] < nrank[choice]) choice = i;
        nrank[ND_rank(n)]--; nrank[choice]++; ND_rank(n) = choice;
    }
}
free_list(ND_tree_in(n)); free_list(ND_tree_out(n)); ND_mark(n) = false;  // 882-884
```
Tie-break in the `choice` scan: strict `<` ⇒ **lowest** rank index wins ties.
(VIRTUAL nodes are skipped entirely — their tree lists are leaked here, unlike
`freeTreeList`.)

Cleanup (886-887): `LIST_FREE(&Tree_node); free(nrank);`

### 2.25 `rank2` — ns.c:951-1027 (the "run" main loop)

```c
int rank2(graph_t *g, int balance, int maxiter, int search_size)
```
1. `iter = 0; ctx = {0};` `#ifdef DEBUG check_cycles(g); #endif` (953-960).
2. `if (Verbose)`: `graphSize(g,&nn,&ne)` (926-937; counts nodes and out-list entries);
   `fprintf(stderr, "%s %zu nodes %zu edges maxiter=%d balance=%d\n", "network simplex: ", …)`
   (954, 964-965); `start_timer()` (966).
3. `feasible = init_graph(&ctx, g); if (!feasible) init_rank(&ctx);` (968-970).
4. `ctx.Search_size = (search_size >= 0) ? search_size : SEARCHSIZE;` (972-975).
   Note `search_size == 0` is honored (leave_edge then returns after… actually
   `++cnt >= 0` is true on the first negative edge ⇒ returns the first negative-cut
   edge found from `S_i`).
5. `err = feasible_tree(&ctx); if (err != 0) { freeTreeList(&ctx, g); return err; }`
   (977-983) — returns 1 (disconnected) or 2 (add_tree_edge/merge failure) verbatim.
6. `if (maxiter <= 0) { freeTreeList(&ctx, g); return 0; }` (984-987) — a non-positive
   maxiter means "build the feasible tree, then stop" (ranks stay as produced by
   init_rank / caller; no simplex pivots, no balance, no normalization).
7. **Main loop** (989-1006):
   ```
   while ((e = leave_edge(&ctx))) {
       f = enter_edge(e);
       err = update(&ctx, e, f);
       if (err != 0) { freeTreeList(&ctx, g); return err; }    // err == 2 only
       iter++;
       if (Verbose && iter % 100 == 0) {                       // progress spam, stderr only
           if (iter % 1000 == 100) fputs("network simplex: ", stderr);
           fprintf(stderr, "%d ", iter);
           if (iter % 1000 == 0) fputc('\n', stderr);
       }
       if (iter >= maxiter) break;
   }
   ```
   Termination: either the tree is optimal (leave_edge → NULL) or `iter >= maxiter`.
   The iteration cap check happens *after* the update, so exactly `maxiter` pivots may
   occur. There is no other cap and no cycle guard in release builds.
8. Balance switch (1007-1019):
   - `balance == 1`: `TB_balance(&ctx); reset_lists(&ctx);` (Tree_edge freed by
     reset_lists; per-node lists freed inside TB_balance for NORMAL nodes only).
   - `balance == 2`: `LR_balance(&ctx);` (frees everything itself).
   - default (0 or anything else): `scan_and_normalize(&ctx); freeTreeList(&ctx, ctx.G);`
     — normalization IS applied here.
9. `if (Verbose)`: `if (iter >= 100) fputc('\n', stderr);
   fprintf(stderr, "%s%zu nodes %zu edges %d iter %.2f sec\n", ns, ctx.N_nodes,
   ctx.N_edges, iter, elapsed_sec());` (1020-1025). `elapsed_sec()` =
   `(clock() − T) / CLOCKS_PER_SEC` (timing.c:23-25).
10. `return 0` (1026).

### 2.26 `rank` — ns.c:1029-1040

```
s = agget(g, "searchsize");          // NULL if the attribute does not exist
search_size = s ? atoi(s) : SEARCHSIZE;   // atoi("")==0; atoi("garbage")==0; truncates
return rank2(g, balance, maxiter, search_size);
```
Note the contrast with dot's own call path (rank.c:1100-1107) where a missing attribute
yields `ssize = -1` ⇒ built-in SEARCHSIZE. Via `rank()` directly, a *present but empty*
`searchsize` gives 0 (which `rank2` treats as a valid, maximally-greedy search size).

### 2.27 DEBUG-only code — ns.c:1318-1414 (`#ifdef DEBUG`)

- `tchk(ctx)` (1319-1335): counts nodes via agfstnode/agnxtnode, tree-out edges; warns
  `"not a tight tree %p"` for tree edges with `SLACK(e) > 0`; `"something missing"` if
  the count ≠ `LIST_SIZE(Tree_edge)`.
- `dump_node`/`dump_graph` (1337-1369): writes the current digraph to **`ns.gv`** in the
  CWD (virtual nodes printed as `%p`).
- `checkdfs` (1371-1403): white/grey/black DFS with `ND_mark`/`ND_onstack`; on finding a
  back edge dumps the graph, prints `"cycle: last edge %p %s(%p) %s(%p)\n"` and returns
  the cycle entry; unwinding prints `"unwind %p %s(%p)\n"`, and returning to the root
  prints `"unwound to root\n"`, `fflush`, `abort()`.
- `check_cycles(g)` (1405-1413): clears `ND_mark`/`ND_onstack` for all nodes, then runs
  `checkdfs` from every node in nlist order. Called at the top of `rank2` (959).

---

## 3. Numeric constants, sentinels and overflow behavior (complete list)

| Constant | Where | Meaning |
|---|---|---|
| `SEARCHSIZE = 30` | ns.c:55 | default `leave_edge` scan budget |
| `-1` | ns.c:910, 119 (ED_tree_index) | "not a tree edge" |
| `-1` | ns.c:92-95, 737 (ND_low) | "DFS interval invalidated" |
| `-1` | ns.c:57-83 return | `add_tree_edge` failure |
| `-1 / 1 / 2` | ns.c:622-672 return | feasible_tree: 1 = disconnected, 2 = serious error, 0 = ok |
| `INT_MAX` | ns.c:218, 252 | initial `Slack` sentinel in dfs_enter_* |
| `INT_MAX`, `INT_MIN` | ns.c:749-750 | min/max-rank accumulators in scan_and_normalize |
| `INT_MAX` | ns.c:63 (assert) | tree-edge count must fit in the `int` `ED_tree_index` |
| `SIZE_MAX` | ns.c:313 (init? no — set by STextractmin:566), 319, 441-443, 449, 557, 569 | `heap_index` sentinel = "not on heap" |
| `SIZE_MAX` | ns.c:557 | unsigned underflow loop bound in STbuildheap |
| `INT_MAX` | dot caller rank.c:1075, 1080 | default maxiter (`nslimit1` unset) |
| `0` | ns.c:192, 206, 1004 | search budget test; iteration cap test |
| `100/1000` | ns.c:997-1002, 1021 | Verbose progress cadence only |
| `MAXSHORT`, `LOWBIT`, `MAX_INT` | — | **not present anywhere in ns.c** |

Integer arithmetic notes for a Rust port:
- All ranks, weights, minlens, cut values, slacks, deltas, lows/lims are **`i32`**.
- `LENGTH(e) = rank(head) − rank(tail)` and `SLACK = LENGTH − minlen` are unchecked C
  `int` arithmetic (UB on overflow, two's-complement wrap on real targets). Use
  `wrapping_sub` to match observed behavior; inputs are expected to keep values well
  inside i32.
- The only *guarded* arithmetic is the cut-value summation in `x_cutval`
  (`sadd_overflow` → process exit) — ns.c:1060-1068. Everything else
  (`treeupdate`'s `ED_cutvalue(e) ± cutvalue`, `rerank`'s `ND_rank(v) -= delta`,
  `tree_adjust`'s `+=`) is unchecked.
- `delta / 2` in LR_balance is C integer division truncating toward zero; `delta > 1`
  there, so it equals floor division.
- `LIST_SIZE(...) <= INT_MAX` is asserted before casting tree-edge count to `int`.
- `size_t` counters (`N_nodes`, `N_edges`, list sizes, DFS list indices `i/j`) are
  `usize`; the one deliberate wrap is `for (i = size/2; i != SIZE_MAX; i--)`.

C library / helper functions used (Rust equivalents):
- `qsort` via `LIST_SORT` (unstable sort) — only in TB_balance.
- `agget` (cgraph attr lookup; NULL ⇒ attribute absent; else the string, possibly ""),
  `atoi` (C semantics: skip whitespace, optional sign, leading digits; 0 on failure),
  `agnameof`, `agerrorf`/`agerr(AGPREV, …)` (stderr diagnostics via cgraph error
  machinery), `streq` (strcmp), `gv_alloc`/`gv_calloc` (zeroed), `graphviz_exit`,
  `start_timer`/`elapsed_sec` (clock()), `SWAP`, `sadd_overflow`, `MIN`/`MAX`/`SEQ`.
- `Verbose` global gates every stderr print; a Rust port can make these no-ops or
  `eprint!` under a verbose flag — they never influence results.

---

## 4. Exact iteration orders (summary table)

| Loop | Order |
|---|---|
| Every node scan (`init_graph`, `init_rank` seed, `feasible_tree` subtree discovery, `scan_and_normalize`, `freeTreeList`, TB_balance seed, `check_cycles`) | `GD_nlist` head, following `ND_next` — i.e. the order dot created nodes |
| `init_rank` queue | FIFO (`LIST_POP_FRONT`); seeds in nlist order; per node: **all in-edges first** (`ND_in.list` order), then all out-edges (`ND_out.list` order) |
| `leave_edge` | `Tree_edge` indices `S_i, S_i+1, …, size−1,` then `0 … j−1` (j = entry cursor); earliest minimum wins |
| `add_tree_edge` | tail's `tree_out` then head's `tree_in` |
| `exchange_tree_edges` removal | tail `tree_out` (last-slot swap) → head `tree_in` |
| `dfs_enter_outedge/inedge` | LIFO stack; on pop, **entire out (resp. in) list scanned in order**, then the tree-in (resp. tree-out) seeding pass with the `Slack > 0` early stop |
| `tight_subtree_search` | per frame: in-scan, then out-scan; children pushed at the current edge index; post-order accumulation of `rv` |
| `feasible_tree` merges | min-size subtree first (STheap), then `inter_tree_edge` DFS with out-scan before in-scan per frame |
| `tree_adjust` | tree_in children first, then tree_out children (recursive) |
| `dfs_range_init` / `dfs_range` / `dfs_cutval` | per frame: tree_out scan, then tree_in scan; explicit stack (LIFO); `low` seeded from parent's current `lim`; on completion parent's `lim = child_lim + 1` |
| `update` | rerank decision (leaf tests, then lim compare) → two `treeupdate` walks (tail-first) → `invalidate_path(head(f))` then `invalidate_path(tail(f))` → swap → `dfs_range(lca, …)` |
| `LR_balance` | tree edges in `Tree_edge` index order 0…size−1 |
| `TB_balance` | nodes sorted by rank (ascending, or descending when `TBbalance=max`), qsort-unstable; in-edges then out-edges per node; `choice` scan low→high, lowest index wins |
| `x_cutval` | out-edges of v then in-edges of v |

---

## 5. Transcription-ready pseudocode (Rust-shaped)

Types: `NodeIdx`/`EdgeIdx` (or `&mut` refs), `Ranks: [i32]` etc. `elist` ⇒ `Vec<E>` with
iteration `0..len` (the C NULL sentinel is an implementation artifact; only
`add_tree_edge`'s capacity check needs the len comparison shown below). `Tree_edge` ⇒
`Vec<E>`. Every `/*C: nnn*/` cites the ns.c line.

```rust
const SEARCHSIZE: i32 = 30;                                   //C:55

// ctx.rs --------------------------------------------------------------
struct NsCtx {                                                //C:47-53
    tree_edge: Vec<E>,          // ED_tree_index(e) == position | -1
    s_i: usize,                 // persistent leave_edge cursor
    n_edges: usize, n_nodes: usize,
    search_size: i32,
}

// ---- per-edge helpers ----
fn length(g: &G, e: E) -> i32 { g.rank(g.head(e)) - g.rank(g.tail(e)) }        //C:42
fn slack(g: &G, e: E) -> i32 { length(g, e) - g.minlen(e) }                    //C:43
fn seq(a: i32, b: i32, c: i32) -> bool { a <= b && b <= c }                    //C:44
fn is_tree_edge(e: E) -> bool { g.tree_index(e) >= 0 }                         //C:45

fn init_graph(ctx: &mut NsCtx, g: &mut G) -> bool {                            //C:890-921
    *ctx = NsCtx { g, ..zeroed() };
    for n in g.nodes() {                                   // nlist order
        g.mark(n) = false;                                 //C:895
        ctx.n_nodes += 1;                                  //C:896
        ctx.n_edges += g.out(n).len();                     //C:897-898
    }
    ctx.tree_edge.reserve(ctx.n_nodes);                    //C:901
    let mut feasible = true;
    for n in g.nodes() {
        g.priority(n) = 0;                                 //C:905
        let mut din = 0;
        while din < g.in_(n).len() {                       //C:907
            let e = g.in_(n)[din];
            g.priority(n) += 1;                            //C:908
            g.cutvalue(e) = 0;                             //C:909
            g.tree_index(e) = -1;                          //C:910
            if g.rank(g.head(e)) - g.rank(g.tail(e)) < g.minlen(e) { feasible = false; }
            din += 1;
        }
        g.tree_in(n)  = Vec::with_capacity(din + 1);       //C:914-915
        g.tree_out(n) = Vec::with_capacity(g.out(n).len() + 1); //C:916-918
    }
    feasible
}

fn init_rank(ctx: &mut NsCtx, g: &mut G) {                                     //C:145-177
    let mut q: VecDeque<N> = g.nodes().filter(|&v| g.priority(v) == 0).collect();
    let mut ctr = 0usize;
    while let Some(v) = q.pop_front() {                        //C:160 FIFO
        g.rank(v) = 0; ctr += 1;                               //C:161-162
        for &e in g.in_(v)  { g.rank(v) = max(g.rank(v), g.rank(g.tail(e)) + g.minlen(e)); } //C:163-164
        for &e in g.out(v) {                                   //C:165-168
            g.priority(g.head(e)) -= 1;
            if g.priority(g.head(e)) <= 0 { q.push_back(g.head(e)); }
        }
    }
    if ctr != ctx.n_nodes {                                    //C:170-175
        error!("trouble in init_rank\n");
        for v in g.nodes() { if g.priority(v) != 0 { error_prev!("\t{} {}\n", g.name(v), g.priority(v)); } }
    }
}

fn add_tree_edge(ctx: &mut NsCtx, g: &mut G, e: E) -> Result<(), ()> {         //C:57-83
    if is_tree_edge(e) { error!("add_tree_edge: missing tree edge\n"); return Err(()); }
    debug_assert!(ctx.tree_edge.len() <= i32::MAX as usize);   //C:63
    g.tree_index(e) = ctx.tree_edge.len() as i32;              //C:64
    ctx.tree_edge.push(e);                                     //C:65
    let n = g.tail(e);                                         //C:66
    g.mark(n) = true;                                          //C:67
    g.tree_out_mut(n).push(e);                                 //C:68-69 (sentinel implicit)
    if g.tree_out(n).len() > g.out(n).len() {                  //C:70-73 capacity guard
        error!("add_tree_edge: empty outedge list\n"); return Err(());
    }
    let n = g.head(e);                                         //C:74
    g.mark(n) = true;                                          //C:75
    g.tree_in_mut(n).push(e);                                  //C:76-77
    if g.tree_in(n).len() > g.in_(n).len() {                   //C:78-81
        error!("add_tree_edge: empty inedge list\n"); return Err(());
    }
    Ok(())
}

fn exchange_tree_edges(ctx: &mut NsCtx, g: &mut G, e: E, f: E) {               //C:114-143
    g.tree_index(f) = g.tree_index(e);                         //C:116
    debug_assert!(g.tree_index(e) >= 0);                       //C:117
    ctx.tree_edge[g.tree_index(e) as usize] = f;               //C:118
    g.tree_index(e) = -1;                                      //C:119
    // remove e from tail.tree_out by swap-with-last:
    let n = g.tail(e);                                         //C:121
    let i = { let v = g.tree_out_mut(n); v.pop_last_occupied(&e) };  // i = --size; find j; list[j]=list[i]; list[i]=NULL  //C:122-128
    //   ⇒ implemented as: find j with tree_out[j]==e (j ≤ new_len); tree_out.swap_remove(j)
    let n = g.head(e);                                         //C:129
    // same for tree_in                                        //C:130-135
    g.tree_out_mut(g.tail(f)).push(f);                         //C:137-139
    g.tree_in_mut(g.head(f)).push(f);                          //C:140-142
}

fn leave_edge(ctx: &mut NsCtx, g: &G) -> Option<E> {                           //C:179-213
    let mut rv: Option<E> = None;
    let mut cnt = 0i32;
    let j = ctx.s_i;                                           //C:184
    while ctx.s_i < ctx.tree_edge.len() {                      //C:185  phase 1
        let f = ctx.tree_edge[ctx.s_i];                        //C:186
        if g.cutvalue(f) < 0 {
            rv = match rv {
                Some(r) if g.cutvalue(r) <= g.cutvalue(f) => Some(r),  // strict > replaces ⇒ first wins ties //C:187-189
                _ => Some(f),
            };
            cnt += 1;                                          //C:192
            if cnt >= ctx.search_size { return rv; }           //C:193
        }
        ctx.s_i += 1;                                          //C:195
    }
    if j > 0 {                                                 //C:197  phase 2 (wrap)
        ctx.s_i = 0;
        while ctx.s_i < j {                                    //C:199
            let f = ctx.tree_edge[ctx.s_i];
            if g.cutvalue(f) < 0 {
                rv = match rv {
                    Some(r) if g.cutvalue(r) <= g.cutvalue(f) => Some(r),
                    _ => Some(f),
                };
                cnt += 1;
                if cnt >= ctx.search_size { return rv; }
            }
            ctx.s_i += 1;
        }
    }
    rv                                                         //C:212
}

fn dfs_enter_outedge(g: &G, v0: N, low: i32, lim: i32) -> Option<E> {          //C:215-246
    let mut enter: Option<E> = None;
    let mut slack_best = i32::MAX;                             //C:218
    let mut todo = vec![v0];                                   //C:220-221 LIFO
    while let Some(v) = todo.pop() {                           //C:223-224
        for i in 0..g.out(v).len() {                           //C:226 full pass, in order
            let e = g.out(v)[i];
            if !is_tree_edge(e) {
                if !seq(low, g.lim(g.head(e)), lim) {          //C:228 crossing candidate
                    let s = slack(g, e);                       //C:229
                    if s < slack_best || enter.is_none() { enter = Some(e); slack_best = s; } //C:230-233
                }
            } else if g.lim(g.head(e)) < g.lim(v) {            //C:235-236 descend to child
                todo.push(g.head(e));
            }
        }
        let mut i = 0;                                         //C:238-240 seeding pass
        while i < g.tree_in(v).len() && slack_best > 0 {       // fetch e, then test Slack > 0
            let e = g.tree_in(v)[i];
            if g.lim(g.tail(e)) < g.lim(v) { todo.push(g.tail(e)); }
            i += 1;
        }
    }
    enter                                                      //C:245
}
// dfs_enter_inedge: mirror with in_/tail ↔ out_/head;                        //C:248-280
//   candidate test !seq(low, lim(tail(e)), lim); descend lim(tail(e)) < lim(v);
//   seeding pass over tree_out with condition lim(head(e)) < lim(v) && slack_best > 0.

fn enter_edge(g: &G, e: E) -> Option<E> {                                      //C:282-297
    let (v, outsearch) =
        if g.lim(g.tail(e)) < g.lim(g.head(e)) { (g.tail(e), false) }
        else                                   { (g.head(e), true)  };
    if outsearch { dfs_enter_outedge(g, v, g.low(v), g.lim(v)) }
    else         { dfs_enter_inedge (g, v, g.low(v), g.lim(v)) }
}

// ---- tight tree phase ----
// ND_subtree(n) aliases ND_par(n) with subtree_t*; in Rust keep a separate
// `subtree: Vec<Option<&mut Subtree>>` keyed by node — the aliasing exists only
// because C reuses the field; see ns.c:306-308.                               //C:307-308
struct Subtree { rep: N, size: i32, heap_index: usize /* SIZE_MAX = off-heap */, par: Cell<…> } //C:310-315
fn on_heap(t: &Subtree) -> bool { t.heap_index != usize::MAX }                 //C:318-320

fn tight_subtree_search(ctx: &mut NsCtx, g: &mut G, v0: N, st: &Subtree) -> i32 { //C:331-404
    let mut rv_total = 1i32;                                   //C:335
    g.subtree_set(v0, st);                                     //C:336
    let mut todo: Vec<Tst> = vec![Tst { v: v0, in_i: 0, out_i: 0, rv: 1 }];       //C:338-339
    while let Some(top) = todo.last_mut() {                    //C:341-343
        let mut updated = false;
        while top.in_i < g.in_(top.v).len() {                  //C:345-348
            let e = g.in_(top.v)[top.in_i];
            if is_tree_edge(e) { top.in_i += 1; continue; }    //C:346
            if g.subtree(g.tail(e)).is_none() && slack(g, e) == 0 {   //C:347
                if add_tree_edge(ctx, g, e).is_err() {         //C:348-354
                    todo.pop();                                // index NOT consumed
                    if todo.is_empty() { rv_total = -1; } else { todo.last_mut().unwrap().rv -= 1; }
                } else {
                    top.in_i += 1;                             //C:356 consume edge
                    g.subtree_set(g.tail(e), st);              //C:357
                    todo.push(Tst { v: g.tail(e), in_i: 0, out_i: 0, rv: 1 });   //C:358-359
                }
                updated = true; break;                         //C:361-362
            }
            top.in_i += 1;                                     // for-loop increment
        }
        if updated { continue; }
        while top.out_i < g.out(top.v).len() {                 //C:369-388 exact mirror:
            let e = g.out(top.v)[top.out_i];
            if is_tree_edge(e) { top.out_i += 1; continue; }
            if g.subtree(g.head(e)).is_none() && slack(g, e) == 0 {
                if add_tree_edge(ctx, g, e).is_err() {
                    todo.pop();
                    if todo.is_empty() { rv_total = -1; } else { todo.last_mut().unwrap().rv -= 1; }
                } else {
                    top.out_i += 1;
                    g.subtree_set(g.head(e), st);
                    todo.push(Tst { v: g.head(e), in_i: 0, out_i: 0, rv: 1 });
                }
                updated = true; break;
            }
            top.out_i += 1;
        }
        if updated { continue; }
        let last = todo.pop().unwrap();                        //C:393-398
        if todo.is_empty() { rv_total = last.rv; } else { todo.last_mut().unwrap().rv += last.rv; }
    }
    rv_total                                                   //C:403
}

fn find_tight_subtree(ctx: &mut NsCtx, g: &mut G, v: N) -> Option<Box<Subtree>> { //C:406-417
    let mut t = Box::new(Subtree { rep: v, size: 0, heap_index: 0, par: None });
    t.size = tight_subtree_search(ctx, g, v, &t);
    if t.size < 0 { return None; }                             //C:411-414
    t.par = Some(&t);  // self-root
    Some(t)
}

fn st_set_find(g: &G, n0: N) -> SubtreeRoot {                  //C:424-432
    let mut s0 = g.subtree(n0);
    while let Some(p) = s0.par.filter(|p| !Rc::ptr_eq(p, s0)) {
        if p.par.is_some() { s0.par = p.par.clone(); }         // one-level path compression
        s0 = p;
    }
    s0
}

fn st_set_union(s0: SubtreeRoot, s1: SubtreeRoot) -> SubtreeRoot {               //C:434-451
    let mut r0 = s0; while r0.par.is_some() && !self_root(&r0) { r0 = r0.par; }   //C:438
    let mut r1 = s1; while r1.par.is_some() && !self_root(&r1) { r1 = r1.par; }   //C:439
    if Rc::ptr_eq(&r0, &r1) { return r0; }                     //C:440
    debug_assert!(on_heap(&r0) || on_heap(&r1));               //C:441
    let r = if !on_heap(&r1) { r0 }                            //C:442-445 (verbatim precedence)
            else if !on_heap(&r0) { r1 }
            else if r1.size < r0.size { r0 } else { r1 };
    r0.par = r.clone(); r1.par = r.clone();                    //C:447
    r.size = r0.size + r1.size;                                //C:448
    debug_assert!(on_heap(&r));
    r
}

fn inter_tree_edge_search(g: &G, v0: N) -> Option<E> {         //C:454-525
    let mut best: Option<E> = None;                            //C:468
    let mut todo = vec![State { v: v0, ts: st_set_find(g, v0), from: None, out_i: 0, in_i: 0 }]; //C:465-466
    while let Some(s) = todo.last_mut() {                      //C:470-471
        if s.out_i == 0 && s.in_i == 0 && best.map_or(false, |b| slack(g, b) == 0) { //C:472-475
            todo.pop(); continue;
        }
        let mut updated = false;
        while s.out_i < g.out(s.v).len() {                     //C:479-495
            let e = g.out(s.v)[s.out_i];
            if is_tree_edge(e) {
                if g.head(e) == s.from { s.out_i += 1; continue; }  //C:481 do not search back
                s.out_i += 1;                                  //C:482
                let hv = g.head(e);
                todo.push(State { v: hv, ts: st_set_find(g, hv), from: Some(s.v), out_i: 0, in_i: 0 }); //C:483-485
                updated = true; break;
            } else {
                if st_set_find(g, g.head(e)) != s.ts           //C:490 different tree
                   && (best.is_none() || slack(g, e) < slack(g, best.unwrap())) { best = Some(e); } //C:491
            }
            s.out_i += 1;
        }
        if updated { continue; }
        while s.in_i < g.in_(s.v).len() {                      //C:501-515 mirror
            let e = g.in_(s.v)[s.in_i];
            if is_tree_edge(e) {
                if g.tail(e) == s.from { s.in_i += 1; continue; }
                s.in_i += 1;
                let tv = g.tail(e);
                todo.push(State { v: tv, ts: st_set_find(g, tv), from: Some(s.v), out_i: 0, in_i: 0 });
                updated = true; break;
            } else {
                if st_set_find(g, g.tail(e)) != s.ts
                   && (best.is_none() || slack(g, e) < slack(g, best.unwrap())) { best = Some(e); }
            }
            s.in_i += 1;
        }
        if updated { continue; }
        todo.pop();                                            //C:520
    }
    best                                                       //C:524
}
fn inter_tree_edge(t: &Subtree) -> Option<E> { inter_tree_edge_search(t.rep) }   //C:527-530

fn st_heapify(h: &mut [SubtreeRef], i0: usize) {               //C:534-550
    let mut i = i0;
    loop {
        let left = 2*(i+1) - 1; let right = 2*(i+1);           //C:537-538
        let mut smallest = i;
        if left  < h.len() && h[left].size  < h[smallest].size { smallest = left; }
        if right < h.len() && h[right].size < h[smallest].size { smallest = right; }
        if smallest != i { h.swap(i, smallest); h[i].heap_index = i; h[smallest].heap_index = smallest; i = smallest; }
        else { break; }
    }   // do-while(i < size): i < len always holds after a swap, so plain loop is equivalent
}

fn st_extract_min(h: &mut Vec<SubtreeRef>) -> SubtreeRef {     //C:562-574
    let rv = h[0];
    rv.heap_index = usize::MAX;                                //C:566
    let last = h.len() - 1;
    h[0] = h[last]; h[0].heap_index = 0;                       //C:568-569
    h[last] = rv;                                              //C:570 (slot bookkeeping only)
    h.truncate(last);                                          //C:571 size--
    st_heapify(h, 0);                                          //C:572
    rv
}
// (size==1 quirk of the C code — heap_index reset to 0 — is unreachable because
//  feasible_tree only extracts while len > 1.)

fn tree_adjust(g: &mut G, v: N, from: Option<N>, delta: i32) { //C:576-591
    g.rank(v) += delta;                                        //C:580
    for i in 0..g.tree_in(v).len() {                           //C:581-585
        let e = g.tree_in(v)[i]; let w = g.tail(e);
        if Some(w) != from { tree_adjust(g, w, Some(v), delta); }
    }
    for i in 0..g.tree_out(v).len() {                          //C:586-590
        let e = g.tree_out(v)[i]; let w = g.head(e);
        if Some(w) != from { tree_adjust(g, w, Some(v), delta); }
    }
}

fn merge_trees(ctx: &mut NsCtx, g: &mut G, e: E) -> Option<SubtreeRef> {          //C:593-615
    debug_assert!(!is_tree_edge(e));                           //C:596
    let t0 = st_set_find(g, g.tail(e));                        //C:598
    let t1 = st_set_find(g, g.head(e));                        //C:599
    if !on_heap(&t0) {                                         //C:601-605 move t0 down by +slack
        let delta = slack(g, e);
        if delta != 0 { tree_adjust(g, t0.rep, None, delta); }
    } else {                                                   //C:606-610 move t1 up by −slack
        let delta = -slack(g, e);
        if delta != 0 { tree_adjust(g, t1.rep, None, delta); }
    }
    if add_tree_edge(ctx, g, e).is_err() { return None; }      //C:611-613
    Some(st_set_union(t0, t1))                                 //C:614
}

fn feasible_tree(ctx: &mut NsCtx, g: &mut G) -> i32 {          //C:622-672
    for n in g.nodes() { g.subtree_set(n, None); }             //C:631-633
    let mut tree: Vec<Option<Box<Subtree>>> = (0..ctx.n_nodes).map(|_| None).collect(); //C:635
    let mut count = 0usize;
    for n in g.nodes() {                                       //C:637-646
        if g.subtree(n).is_none() {
            tree[count] = find_tight_subtree(ctx, g, n);
            if tree[count].is_none() { cleanup(...); return 2; }
            count += 1;
        }
    }
    let mut heap: Vec<SubtreeRef> = tree[..count] (refs);
    for i in 0..count { heap[i].heap_index = i; }
    for i in (0..=count/2).rev() { st_heapify(&mut heap, i); } //C:557 (size/2 … 0)
    let mut error = 0;
    while heap.len() > 1 {                                     //C:650
        let t0 = st_extract_min(&mut heap);
        let Some(ee) = inter_tree_edge(&t0) else { error = 1; break; };          //C:652-655
        let Some(t1) = merge_trees(ctx, g, ee) else { error = 2; break; };       //C:656-660
        st_heapify(&mut heap, t1.heap_index);                  //C:661
    }
    // free subtrees/heap …
    if error != 0 { return error; }
    debug_assert!(ctx.tree_edge.len() == ctx.n_nodes - 1);     //C:669
    init_cutvalues(g);                                         //C:670
    0
}

fn treeupdate(g: &mut G, mut v: N, w: N, cutvalue: i32, dir: bool) -> N {        //C:675-689
    while !seq(g.low(v), g.lim(w), g.lim(v)) {                 //C:676
        let e = g.par(v);                                      //C:677
        let d = if v == g.tail(e) { dir } else { !dir };       //C:678
        if d { g.cutvalue(e) += cutvalue; } else { g.cutvalue(e) -= cutvalue; }  //C:679-682
        v = if g.lim(g.tail(e)) > g.lim(g.head(e)) { g.tail(e) } else { g.head(e) }; //C:683-686
    }
    v                                                          //C:688
}

fn rerank(g: &mut G, v: N, delta: i32) {                       //C:691-702
    g.rank(v) -= delta;                                        //C:695
    for i in 0..g.tree_out(v).len() { let e = g.tree_out(v)[i];
        if e != g.par(v) { rerank(g, g.head(e), delta); } }    //C:696-698
    for i in 0..g.tree_in(v).len() { let e = g.tree_in(v)[i];
        if e != g.par(v) { rerank(g, g.tail(e), delta); } }    //C:699-701
}

fn invalidate_path(g: &mut G, lca: N, mut to_node: N) {        //C:90-112
    loop {
        if g.low(to_node) == -1 { break; }                     //C:92-93
        g.low(to_node) = -1;                                   //C:95
        let e = match g.par(to_node) { None => break, Some(e) => e };  //C:97-99
        if g.lim(to_node) >= g.lim(lca) {                      //C:101-105
            if to_node != lca { error!("invalidate_path: skipped over LCA\n"); }
            break;
        }
        to_node = if g.lim(g.tail(e)) > g.lim(g.head(e)) { g.tail(e) } else { g.head(e) }; //C:107-110
    }
}

fn update(ctx: &mut NsCtx, g: &mut G, e: E, f: E) -> i32 {     //C:707-746
    let delta = slack(g, f);                                   //C:710
    if delta > 0 {                                             //C:712
        let s = g.tree_in(g.tail(e)).len() + g.tree_out(g.tail(e)).len();        //C:713
        if s == 1 { rerank(g, g.tail(e), delta); }             //C:714-715
        else {
            let s = g.tree_in(g.head(e)).len() + g.tree_out(g.head(e)).len();    //C:717
            if s == 1 { rerank(g, g.head(e), -delta); }        //C:718-719
            else if g.lim(g.tail(e)) < g.lim(g.head(e)) { rerank(g, g.tail(e), delta); }   //C:721-722
            else { rerank(g, g.head(e), -delta); }             //C:723-724
        }
    }
    let cutvalue = g.cutvalue(e);                              //C:729
    let lca = treeupdate(g, g.tail(f), g.head(f), cutvalue, true);   //C:730
    if treeupdate(g, g.head(f), g.tail(f), cutvalue, false) != lca { //C:731-734
        error!("update: mismatched lca in treeupdates\n");
        return 2;
    }
    let lca_low = g.low(lca);                                  //C:737
    invalidate_path(g, lca, g.head(f));                        //C:738
    invalidate_path(g, lca, g.tail(f));                        //C:739
    g.cutvalue(f) = -cutvalue;                                 //C:741
    g.cutvalue(e) = 0;                                         //C:742
    exchange_tree_edges(ctx, g, e, f);                         //C:743
    dfs_range(g, lca, g.par(lca), lca_low);                    //C:744
    0                                                          //C:745
}

fn scan_and_normalize(g: &mut G) -> i32 {                      //C:748-761
    let (mut minrank, mut maxrank) = (i32::MAX, i32::MIN);
    for n in g.nodes() { if g.node_type(n) == NORMAL {
        minrank = minrank.min(g.rank(n)); maxrank = maxrank.max(g.rank(n)); } }  //C:751-756
    for n in g.nodes() { g.rank(n) -= minrank; }               //C:757-758 (all nodes)
    maxrank - minrank                                          //C:759-760
}

fn free_tree_list(ctx: &mut NsCtx, g: &mut G) {                //C:767-776
    for n in g.nodes() { g.tree_in(n).clear(); g.tree_out(n).clear(); g.mark(n) = false; }
    ctx.tree_edge.clear();                                     // reset_lists //C:763-765
}

fn lr_balance(ctx: &mut NsCtx, g: &mut G) {                    //C:778-796
    for i in 0..ctx.tree_edge.len() {                          //C:780 index order
        let e = ctx.tree_edge[i];
        if g.cutvalue(e) == 0 {                                //C:782
            let Some(f) = enter_edge(g, e) else { continue; }; //C:783-785
            let delta = slack(g, f);                           //C:786
            if delta <= 1 { continue; }                        //C:787-788
            if g.lim(g.tail(e)) < g.lim(g.head(e)) { rerank(g, g.tail(e), delta / 2); }  //C:789-790
            else { rerank(g, g.head(e), -delta / 2); }         //C:791-792
        }
    }
    free_tree_list(ctx, g);                                    //C:795
}

fn tb_balance(ctx: &mut NsCtx, g: &mut G) {                    //C:814-888
    let mut adj = 0i32;                                        //C:817
    let maxrank = scan_and_normalize(g);                       //C:820
    debug_assert!(maxrank >= 0);                               //C:823
    let mut nrank = vec![0i32; (maxrank + 1) as usize];        //C:824
    if let Some(s) = g.agget("TBbalance") {                    //C:825
        if s == "min" { adj = 1; } else if s == "max" { adj = 2; }
        if adj != 0 { for n in g.nodes() { if g.node_type(n) == NORMAL {
            if g.in_(n).is_empty() && adj == 1 { g.rank(n) = 0; }         //C:830-832
            if g.out(n).is_empty() && adj == 2 { g.rank(n) = maxrank; }   //C:833-835
        } } }
    }
    let mut tree_node: Vec<N> = g.nodes().collect();           //C:840-842
    if adj > 1 { tree_node.sort_unstable_by(|a,b| rank_cmp_desc(*a,*b)); }   //C:843 qsort == unstable sort
    else       { tree_node.sort_unstable_by(|a,b| rank_cmp_asc (*a,*b)); }
    for &n in &tree_node { if g.node_type(n) == NORMAL { nrank[g.rank(n) as usize] += 1; } } //C:844-848
    for &n in &tree_node {                                     //C:849-885
        if g.node_type(n) != NORMAL { continue; }              //C:851-852
        let (mut inweight, mut outweight) = (0i32, 0i32);
        let mut low = 0i32; let mut high = maxrank;
        for &e in g.in_(n)  { inweight += g.weight(e); low  = low.max(g.rank(g.tail(e)) + g.minlen(e)); }  //C:857-860
        for &e in g.out(n) { outweight += g.weight(e); high = high.min(g.rank(g.head(e)) - g.minlen(e)); } //C:861-864
        if low < 0 { low = 0; }                                //C:865-866
        if inweight == outweight {
            if adj != 0 { g.rank(n) = if adj == 1 { low } else { high }; }      //C:867-870
            else {
                let mut choice = low;                          //C:873
                for i in (low + 1)..=high { if nrank[i as usize] < nrank[choice as usize] { choice = i; } } //C:874-876
                nrank[g.rank(n) as usize] -= 1;                //C:877
                nrank[choice as usize] += 1;                   //C:878
                g.rank(n) = choice;                            //C:879
            }
        }
        // tree lists dropped; mark cleared (NORMAL nodes only)//C:882-884
    }
}

fn dfs_range_init(g: &mut G, v0: N) -> i32 {                   //C:1176-1237
    let mut lim_last = 0i32;                                   //C:1177
    g.par(v0) = None; g.low(v0) = 1;                           //C:1181-1182
    let mut todo = vec![Dfs { v: v0, par: None, lim: 1, out_i: 0, in_i: 0 }];    //C:1183-1184
    while !todo.is_empty() {                                   //C:1186
        let mut pushed = false;
        { let s = todo.last_mut().unwrap();
          while s.out_i < g.tree_out(s.v).len() {              //C:1190-1202
              let e = g.tree_out(s.v)[s.out_i]; s.out_i += 1;  //C:1191-1192
              if Some(e) != s.par {
                  let n = g.head(e);
                  g.par(n) = Some(e); g.low(n) = s.lim;      //C:1194-1196
                  todo.push(Dfs { v: n, par: Some(e), lim: s.lim, out_i: 0, in_i: 0 }); //C:1197-1198
                  pushed = true; break;
              }
          } }
        if pushed { continue; }
        { let s = todo.last_mut().unwrap();
          while s.in_i < g.tree_in(s.v).len() {                //C:1207-1219
              let e = g.tree_in(s.v)[s.in_i]; s.in_i += 1;
              if Some(e) != s.par {
                  let n = g.tail(e);
                  g.par(n) = Some(e); g.low(n) = s.lim;
                  todo.push(Dfs { v: n, par: Some(e), lim: s.lim, out_i: 0, in_i: 0 });
                  pushed = true; break;
              }
          } }
        if pushed { continue; }
        let s = todo.pop().unwrap();                           //C:1224-1231
        g.lim(s.v) = s.lim;                                    //C:1224
        lim_last = s.lim;                                      //C:1226
        if let Some(parent) = todo.last_mut() { parent.lim = lim_last + 1; }     //C:1229-1231
    }
    lim_last + 1                                               //C:1236
}

fn dfs_range(g: &mut G, v0: N, par: Option<E>, low: i32) -> i32 {                //C:1242-1316
    if g.par(v0) == par && g.low(v0) == low { return g.lim(v0) + 1; }            //C:1246-1248 memo hit
    g.par(v0) = par; g.low(v0) = low;                          //C:1252-1253
    let mut todo = vec![Dfs { v: v0, par, lim: low, out_i: 0, in_i: 0 }];        //C:1254-1255
    let mut lim_last = 0i32;
    while !todo.is_empty() {                                   //C:1257
        let mut descended = false;
        { let s = todo.last_mut().unwrap();
          while s.out_i < g.tree_out(s.v).len() {              //C:1261-1277
              let e = g.tree_out(s.v)[s.out_i]; s.out_i += 1;
              if Some(e) != s.par {
                  let n = g.head(e);
                  if g.par(n) == Some(e) && g.low(n) == s.lim { s.lim = g.lim(n) + 1; }  //C:1266-1267 reuse
                  else { g.par(n) = Some(e); g.low(n) = s.lim;
                         todo.push(Dfs { v: n, par: Some(e), lim: s.lim, out_i: 0, in_i: 0 }); }
                  descended = true; break;
              }
          } }
        if descended { continue; }
        { /* mirror over tree_in/agtail with same reuse test */ }                //C:1282-1298
        if descended { continue; }
        let s = todo.pop().unwrap();
        g.lim(s.v) = s.lim;                                    //C:1303
        lim_last = s.lim;                                      //C:1305
        if let Some(parent) = todo.last_mut() { parent.lim = lim_last + 1; }     //C:1308-1310
    }
    lim_last + 1                                               //C:1315
}

fn dfs_cutval(g: &mut G, v0: N, par0: Option<E>) {             //C:1110-1159
    let mut todo = vec![St { v: v0, par: par0, out_i: 0, in_i: 0 }];
    while !todo.is_empty() {                                   //C:1123
        let mut descended = false;
        { let top = todo.last_mut().unwrap();
          while top.out_i < g.tree_out(top.v).len() {          //C:1128-1135
              let e = g.tree_out(top.v)[top.out_i];
              if Some(e) != top.par { top.out_i += 1;
                  todo.push(St { v: g.head(e), par: Some(e), out_i: 0, in_i: 0 });
                  descended = true; break; }
              top.out_i += 1;
          } }
        if descended { continue; }
        { let top = todo.last_mut().unwrap();
          while top.in_i < g.tree_in(top.v).len() {            //C:1140-1147
              let e = g.tree_in(top.v)[top.in_i];
              if Some(e) != top.par { top.in_i += 1;
                  todo.push(St { v: g.tail(e), par: Some(e), out_i: 0, in_i: 0 });
                  descended = true; break; }
              top.in_i += 1;
          } }
        if descended { continue; }
        let top = todo.pop().unwrap();
        if let Some(e) = top.par { x_cutval(g, e); }           //C:1152-1153 post-order
    }
}

fn x_cutval(g: &mut G, f: E) {                                 //C:1043-1070
    let (v, dir) = if g.par(g.tail(f)) == Some(f) { (g.tail(f),  1) } //C:1050-1056
                   else                          { (g.head(f), -1) };
    let mut sum: i32 = 0;                                      //C:1058
    for i in 0..g.out(v).len() {                               //C:1059-1063
        let e = g.out(v)[i];
        let (r, ovf) = sum.overflowing_add(x_val(g, e, v, dir));
        if ovf { error!("overflow when computing edge weight sum\n"); std::process::exit(1); } //C:1061-1062
        sum = r;
    }
    for i in 0..g.in_(v).len() {                               //C:1064-1068 (same)
        let e = g.in_(v)[i];
        let (r, ovf) = sum.overflowing_add(x_val(g, e, v, dir));
        if ovf { error!("overflow when computing edge weight sum\n"); std::process::exit(1); }
        sum = r;
    }
    g.cutvalue(f) = sum;                                       //C:1069
}

fn x_val(g: &G, e: E, v: N, dir: i32) -> i32 {                 //C:1072-1108
    let other = if g.tail(e) == v { g.head(e) } else { g.tail(e) };              //C:1077-1080
    let (f, mut rv) = if !seq(g.low(v), g.lim(other), g.lim(v)) {                //C:1081-1091
        (true, g.weight(e))
    } else {
        (false, if is_tree_edge(e) { g.cutvalue(e) } else { 0 } - g.weight(e))
    };
    let mut d = if dir > 0 { if g.head(e) == v { 1 } else { -1 } }               //C:1092-1102
                else   { if g.tail(e) == v { 1 } else { -1 } };
    if f { d = -d; }                                           //C:1103-1104
    if d < 0 { rv = -rv; }                                     //C:1105-1106
    rv                                                         //C:1107
}

// ---- public entry points ----
pub fn rank2(g: &mut G, balance: i32, maxiter: i32, search_size: i32) -> i32 {   //C:951-1027
    let mut iter = 0i32;
    let mut ctx = NsCtx::zeroed();
    if verbose() { eprintln!("network simplex: {} nodes {} edges maxiter={} balance={}",
                             n_nodes, n_edges, maxiter, balance); start_timer(); }  //C:961-967
    let feasible = init_graph(&mut ctx, g);
    if !feasible { init_rank(&mut ctx, g); }                   //C:968-970
    ctx.search_size = if search_size >= 0 { search_size } else { SEARCHSIZE };   //C:972-975
    let err = feasible_tree(&mut ctx, g);
    if err != 0 { free_tree_list(&mut ctx, g); return err; }   //C:977-983
    if maxiter <= 0 { free_tree_list(&mut ctx, g); return 0; } //C:984-987
    loop {
        let Some(e) = leave_edge(&mut ctx, g) else { break; }; //C:989
        let f = enter_edge(g, e);                              //C:990
        let err = update(&mut ctx, g, e, f.unwrap());          //C:991 f != NULL whenever e leaves a non-optimal tree
        if err != 0 { free_tree_list(&mut ctx, g); return err; }                 //C:992-995
        iter += 1;                                             //C:996
        if verbose() && iter % 100 == 0 { /* progress spam, ns.c:997-1003 */ }
        if iter >= maxiter { break; }                          //C:1004-1005
    }
    match balance {                                            //C:1007-1019
        1 => { tb_balance(&mut ctx, g); ctx.tree_edge.clear(); }
        2 => { lr_balance(&mut ctx, g); }
        _ => { scan_and_normalize(g); free_tree_list(&mut ctx, g); }
    }
    if verbose() { /* "network simplex: N nodes M edges I iter S.SS sec" */ }    //C:1020-1025
    0                                                          //C:1026
}

pub fn rank(g: &mut G, balance: i32, maxiter: i32) -> i32 {    //C:1029-1040
    let search_size = match g.agget("searchsize") {
        Some(s) => atoi(s),            // C atoi semantics
        None => SEARCHSIZE,
    };
    rank2(g, balance, maxiter, search_size)
}
```

One deliberate deviation is marked above (`update(..., f.unwrap())`): in C,
`enter_edge` may in principle return NULL and `update` would dereference it; it cannot
happen for a negative-cut leaving edge (some crossing edge must exist in a feasible
tree), but the Rust port should mirror C's trust or return code 2 defensively.

---

## 6. Edge cases and behavioral corner cases

1. **Empty graph** (`N_nodes == 0`): `init_graph` is a no-op; `feasible_tree` allocates
   zero subtrees, `STbuildheap(…, 0)` is a no-op, the merge `while` doesn't run; the
   debug-only `assert(LIST_SIZE == N_nodes − 1)` compares `0 == SIZE_MAX` and would
   fire in a debug build (ns.c:669; `N_nodes - 1` underflows `size_t`). Release builds
   proceed and return 0 without touching ranks.
2. **Single node**: one singleton tight subtree; heap size 1 ⇒ no merges;
   `Tree_edge.len() == 0 == N_nodes − 1` ✓; `dfs_range_init` assigns
   `low = 1`, `lim = 1`, returns 2; `leave_edge` finds nothing; balance paths no-op
   (TB_balance: `nrank` length `Maxrank+1`; a lone node moves nowhere because
   `inweight == outweight == 0` but `choice == low == its rank`).
3. **Disconnected constraint graph**: `init_rank` prints
   `"trouble in init_rank\n"` + per-node leftovers if infeasible-with-cycle;
   `feasible_tree` returns **1** the first time `inter_tree_edge` returns NULL for a
   smallest tree (ns.c:652-655), and `rank2` returns 1 immediately (ns.c:979-982) with
   tree lists freed — **ranks may be partially adjusted by prior `tree_adjust` calls**.
4. **Already-feasible input**: `init_rank` skipped; caller's ranks (including negative
   virtual-node ranks) are kept; `scan_and_normalize` (default/TB paths) later shifts
   them so the minimum NORMAL rank is 0. In the `balance == 2` path no normalization
   occurs (ns.c:1007-1019 switch has no scan_and_normalize for case 2).
5. **Zero-weight edges** (`ED_weight == 0`): the edge's own weight contributes 0 to
   every cut-value sum (`x_val` returns ±`ED_weight` for crossing edges and
   `cutvalue − weight` for inside edges, ns.c:1081-1091) and 0 to TB_balance's
   in/out weights. A zero-weight **tree** edge's cut value is nonetheless determined
   by the *other* edges crossing its cut, so it can still go negative and leave the
   tree; zero-weight edges are also ordinary candidates for `enter_edge`/LR_balance. Zero **minlen** edges are the tight-tree backbone:
   `SLACK(e) == 0` admits them into the initial tree (ns.c:347, 371).
6. **Zero slack entering edges** (`SLACK(f) == 0`): `update` performs the cut-value/
   swap bookkeeping with `delta == 0` (no rerank) — degenerate pivot, still counts as
   one iteration.
7. **`maxiter <= 0`**: feasible tree is built (and its cut values computed), then
   rank2 returns 0 with **no normalization and no balance** (ns.c:984-987).
8. **`search_size` variations**: `0` ⇒ leave_edge returns the *first* negative-cut
   edge from the cursor (`cnt` becomes 1 ≥ 0). Negative ⇒ SEARCHSIZE. Large ⇒ whole
   list scanned. The scan budget counts only negative-cut edges, and the cursor is
   *not* reset when the budget triggers early (see §2.5 end-states).
9. **`TBbalance` attribute**: only exact strings `"min"`/`"max"` are honored;
   `adj == 1` pins all source-ish NORMAL nodes (empty in-list) to rank 0, `adj == 2`
   pins sink-ish ones to Maxrank — *before* the sorted pass, so `nrank` counts reflect
   the pinned ranks (ns.c:825-848).
10. **Self-loops**: counted in `N_edges`; would break the DAG invariants (they can never
    be tight unless minlen ≤ 0 and are not special-cased here) — dot guarantees they
    are removed/flat-edged before ranking.
11. **Ranks of VIRTUAL nodes** in TB_balance: `low` is clamped to ≥ 0 ("vnodes can have
    ranks < 0", ns.c:865-866) but virtual nodes are skipped from the balancing loop
    entirely; only the rank array bounds assume post-normalization ranks ∈ [0, Maxrank]
    for NORMAL nodes.
12. **`invalidate_path` error path**: "skipped over LCA" (ns.c:103) indicates a corrupted
    tree; it's reported and the walk stops — no return-code change.
13. **`update` LCA mismatch**: returns 2 and rank2 aborts ranking with freeTreeList
    (ns.c:731-734, 992-995).
14. **Overflow**: only the `x_cutval` summation is guarded (exit(1) on overflow);
    `treeupdate`, `rerank`, `tree_adjust`, `LENGTH`, `SLACK`, and
    `scan_and_normalize` are unguarded 32-bit arithmetic.

## 7. Determinism requirements for a bit-identical Rust port

- Preserve the **node list order** (`GD_nlist`/`ND_next`) as the primary iteration
  order everywhere, and the **in/out edge list orders** exactly as built by the
  front-end (compile_edges/add_fast_edges in dot).
- Preserve `Tree_edge` index semantics: append-order, slot-swap in
  `exchange_tree_edges`, and the persistent `S_i` cursor with the exact end-states of
  §2.5.
- Keep the tie-break rules: earliest-wins in `leave_edge`, `dfs_enter_*`,
  `inter_tree_edge_search`; lowest-rank-index-wins in TB_balance's `choice`; left-bias
  (parent-wins) in `STheapify`; "not-on-heap first, then smaller-size-wins" in
  `STsetUnion`.
- Keep the LIFO pop order and full re-scan semantics of the four explicit-stack DFS
  routines (no visited sets where C has none — duplicated visits are load-bearing for
  output-identical behavior).
- The only non-determinism in the C code is `qsort` in TB_balance on equal-rank nodes
  (glibc merge-sort-like for small arrays? No — glibc qsort is an introsort;
  equal-rank order is unspecified) and `elapsed_sec()` timings; both are output-neutral
  except for the iteration order of the balancing loop, which can differ on
  equal-rank ties across libc implementations.
