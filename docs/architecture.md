# Architecture & development notes

Developer-facing documentation: how `dotv` is structured, how the layout
engine maps onto the Graphviz C sources, and how the output is verified.
User-facing docs live in the [README](../README.md).

## Module map

```
src/
├── main.rs        entry point: CLI (clap), app bootstrap, window creation
├── actions.rs     global actions + default keybindings
├── app.rs         GraphView: owns the open tabs (GraphTab: document, navigation
│                  state, load/re-layout state), settings and the load/open
│                  flows; failures report through a centered dialog
├── document.rs    the loaded graph + its DotView + drag offsets; the
│                  Send load/re-layout pipelines (`load_parts`,
│                  `relayout_graph`) for the background executor
├── dump.rs        headless `--dump` JSON output (layout verification)
├── svgdump.rs     headless `--svg` export
├── file_dialog.rs file-picker wiring
├── fonts.rs       the embedded MiSans Latin typeface
├── graph/         DOT data layer
│   ├── parser.rs    a port of cgraph's scanner + grammar (subgraphs, clusters,
│   │                ports, strict graphs, HTML strings, `+` concatenation)
│   └── model.rs     Graph/Node/Edge/Subgraph — cgraph's membership semantics
├── dotgen/        the layout engine (pure logic, no UI)
│   ├── mod.rs       the pipeline entry, attribute resolution, arena construction
│   ├── model.rs     the fast-graph arena (Fg/DGraph/DNode/DEdge — dot.h fields)
│   ├── rank.rs      ranking phase, component decomposition
│   ├── newrank.rs   dot2_rank (newrank=true constraint graph)
│   ├── ns.rs        network simplex (ranking and x-coordinates)
│   ├── classes.rs   fast-graph operations and edge classification
│   ├── cluster.rs   cluster construction helpers
│   ├── mincross.rs  crossing minimization (cluster-aware)
│   ├── position.rs  y/x coordinates, aspect
│   ├── conc.rs      concentrate=true edge merging
│   ├── flat.rs      flat-edge label nodes
│   ├── sameport.rs  samehead/sametail merging
│   ├── dot_splines.rs  the edge router's driver
│   ├── splines.rs   channel routing core + clip_and_install
│   ├── pathplan.rs  shortest path in a polygon + spline fitting
│   ├── arrows.rs    arrowheads
│   ├── shapes.rs    node shapes, record layout, compass ports
│   ├── postprocess.rs  final rotation/translation
│   └── geom.rs      points/boxes
├── viz/           visualization layer
│   ├── dotview.rs   dot output → render model (world coords, colors, arrows)
│   ├── layout/      RankDir/NodeBox vocabulary, lattice snapping, text measurement
│   ├── primitives.rs  rounded rects, circles, polygons, cubic flattening, text
│   ├── transform.rs   world ↔ screen transform, adaptive grid spacing
│   ├── interact.rs    pointer behavior (pan, zoom, drag, hover, selection)
│   ├── paint.rs       composing the frame: clusters, splines, arrows, shapes, labels
│   ├── x11colors.rs   X11 color names
│   └── mod.rs         the self-drawn GPUI Element
├── settings/      the settings model, generic card and app rows
└── ui/            title bar chrome, tab strip (+ button), zoom cluster, status
                    bar, loading overlay (first load) and drop-target veil
```

## The layout engine

`src/dotgen` is a faithful Rust port of Graphviz' `dot` (`lib/dotgen` + the
`lib/common`/`lib/pathplan` code it depends on). The pipeline mirrors
`dotLayout` in `dotgen/dotinit.c` phase by phase; every function is a port of
the C original with the C file and line cited in its doc comment. The specs
written while porting live in [`graphviz-specs/`](graphviz-specs/)
(per-file, line-referenced).

```
rank      class1/class2 fast graph, cycle breaking, network simplex ranking,
          rank sets (rank=same/min/max/source/sink), newrank (dot2_rank:
          separate constraint graph, compact clusters), clusters        rank.c ns.c decomp.c newrank.c
mincross  rank arrays, median + transpose crossing minimization,
          flat-edge ordering, cluster-aware pipeline (per-cluster
          mincross, component merging)                                  mincross.c
position  y packing, x via the shared network simplex on an aux graph,
          flat-edge label nodes, aspect/ratio scaling, concentration
          (concentrate=true: merge parallel edges onto one chain)       position.c flat.c conc.c
splines   channel boxes, multi-edge fan-out, flat edges, self loops,
          ports, channel polygon → shortest path → Hermite spline fit   dotsplines.c routespl.c
                                                                        splines.c pathplan/
arrows    arrow flag parsing, 4-head packing, per-type geometry,
          endpoint clipping by arrow length                             arrows.c
shapes    shape registry, poly_init sizing, inside tests, record
          labels, compass ports                                         shapes.c
postproc  per-rankdir rotation/translation into final coordinates       postproc.c
```

Everything operates on the `Fg` arena (`dotgen/model.rs`), mirroring the C
data model field for field so the algorithms could be transcribed verbatim.

### Text measurement

The GUI measures label text with the real text system and feeds the extents
to the engine, which applies Graphviz' own sizing rules (`poly_init`: label +
padding, attribute lower bounds, shape fits) — so the laid-out geometry
matches the painted glyphs. The headless modes (`--dump`/`--svg`) use an
analytic estimator instead (~0.5 em per glyph at the label's font size,
dot's 1.2 line spacing), which is what makes the Graphviz comparison
deterministic and CI-friendly.

### Porting status

Supported: `rankdir=TB|LR|BT|RL`, `minlen`, `weight`, `constraint`,
`rank=same|min|max|source|sink`, `newrank=true` (dot2 ranking),
`concentrate=true`, clusters (boxes + labels, nested), self loops,
parallel/multi edges with `Multisep` fan-out, flat edges, `splines=line` and
`splines=polyline`, `dir`/`arrowhead`/`arrowtail` (all arrow types),
`headport`/`tailport` (compass ports, record fields) and
`samehead`/`sametail`, node shapes (the builtin registry, incl. record
shapes with HTML-like/record labels), `style`
(filled/rounded/dashed/dotted/invis), `penwidth`, `arrowsize`,
`nodesep`/`ranksep`, `nslimit`/`quantum`, colors (X11 names, `#rrggbb`, HSV,
gradients take their first stop), `bgcolor`.

Not yet ported: `lib/ortho` (`splines=ortho` falls through to polyline
routing, exactly as C compiled without `#ifdef ORTHO`), the
`make_flat_adj_edges` nested-layout path (adjacent-node flat edges with ports
get a simple spline through the mid-gap), `dot_sameports`, head/tail port
labels (`labelangle`/`labeldistance`), and the record-shape `pboxfn`
(`record_path`).

## Verification against Graphviz

The engine is checked against real `dot` on a 51-file corpus
(`tests/corpus`, `c01`–`c51`): ranks, ports, clusters, concentration,
newrank, splines modes, record shapes.

Method: run both engines on the same file and diff their output —

```bash
dot -Tjson graph.dot      # reference: ranks + coordinates
dotv --dump graph.dot     # ours: ranks, node boxes, edge geometry as JSON

# remove font-metric differences: feed dot's own node sizes back in
dot -Tplain graph.dot > sizes.plain
dotv --dump graph.dot --sizes sizes.plain
```

- **Ranks** must match exactly (this is the pass/fail criterion);
- **Node coordinates** are compared as mean/max deviation in points.

Status at the time of writing: **51/51 pass**, 37 of them with **0.0 pt**
coordinate deviation. The largest remaining deviations are the synthetic
diamond cases (ties broken differently, ~27 pt) and one concentrate+cluster
case (~14 pt).

`dotv --svg` renders the same `DotView` the canvas paints, so drawings can
also be diffed as `dot -Tsvg` vs `dotv --svg`.

## Tests

```bash
cargo test    # engine unit tests + view/document tests + UI integration tests
```

Engine tests cover the parser (cgraph grammar), each pipeline phase against
hand-computed expectations, and the view/document layer. The app's UI
integration tests (`#[gpui_kit::test]`, headless windows over the real
`GraphView`) cover the load flow: the loading animation covers the canvas
until the off-thread layout lands, and a parse/I-O failure opens a centered
error dialog, dismissable with OK, with the failure kept on the tab. The
corpus comparison above is the integration check against the C original.

### Performance model

The engine is output-compatible with `dot` and gets its speed from the same
idea everywhere: keep the C data model as the source of truth, but give the
hot loops dense arrays that mirror it.

* **mincross SoA snapshot** (`mincross.rs`) — `in_cross`/`out_cross`/`medians`
  read `ND_order`, `ED_xpenalty` and port data millions of times per layout.
  A `Snap` mirrors those fields into contiguous arrays with per-adjacency
  payloads (CSR over `ND_out`/`ND_in`, preserving elist order), so the inner
  crossing loops scan pure slices. `exchange` patches the two swapped nodes'
  payloads in O(deg); any structural change (flat-edge rewiring, cluster
  expansion, component merge) drops a `valid` flag and the next hot-loop
  entry rebuilds the snapshot in O(V+E). An adjacent-pair crossing cache
  skips re-evaluating untouched pairs on transpose rescans.
* **Fused AVX2 pair kernels** (`mincross.rs`) — `in_cross(v,w)` and
  `in_cross(w,v)` scan the same pairs with mirrored conditions and a
  commutative product, so one fused pass computes both directions exactly.
  The no-port fast path (any_port = false ⇒ the port tie-break can never
  fire) runs an 8-wide AVX2 kernel via runtime detection
  (`core::arch`, stable) with the scalar loop as fallback and test oracle;
  small degrees stay scalar by design (`v1 - v0 >= 16` gate).
* **Network simplex SoA** (`ns.rs`) — `NsCtx` mirrors `ND_rank/low/lim/par`
  and `ED_cutvalue/tindex` into column arrays plus a CSR snapshot of the
  aux adjacency; the arena is snapshotted on entry and written back on
  every return path (`sync_back`), so the observable state is unchanged.
  `rerank`/`tree_adjust` become worklist walks (the per-node shift is
  independent, so the traversal order is free) and the DFS frame stacks are
  reused across pivots.
* **Parallelism** — layout itself is an inherently sequential chain of
  dependent decisions (pivot → tree exchange → cut updates), exactly as in
  C. The one clean win is `ncross`: the invalidated ranks' `rcross`
  computations are independent integer sums, so `ncross` chunks them across
  scoped threads (std-only, no new dependencies) once the work exceeds a
  threshold, and the result is bit-identical to the sequential scan.
* **Pair cache invalidation** — `MinCross::exchange` is the single mutation
  point for orders inside the hot loops; it keeps `Snap`, the pair cache
  and the cluster row mirrors coherent in O(deg(v)+deg(w)).

Measured on the benchmark set (`benches/graphs`, best of 3, release):
dense 1k-node DAG 2.7×, 50×50 grid 2.3×, 5.4k-node tree 1.4×, clustered 1k
1.4×; the chain-heavy stress graph (`b03b_randag_1k`) — pathological for
`dot` itself — drops from >30 min (did not finish) to ~18 min, dominated by
the simplex's O(subtree) walks per pivot, which are algorithmic. Outputs
are byte-identical to the pre-optimization engine on the whole corpus and
benchmark set (`dotv --dump` diffs, `benches/baseline`).

### Load and re-layout pipeline

Opening a file runs parse → measure → layout on the background executor
(`document::load_parts`, assembled into a `Document` on the main thread —
the parts are `Send`, the assembled `Document` is not). While it runs, the
tab is `loading`: a first load covers the canvas with the loading animation,
a settings re-layout (`document::relayout_graph`) keeps the previous drawing
and shows the status-bar spinner. The engine's output is validated
(`document::validate_layout`): a drawing that lost nodes or went non-finite
is reported as a layout error through the same centered dialog as parse and
I-O errors, instead of painting garbage.
