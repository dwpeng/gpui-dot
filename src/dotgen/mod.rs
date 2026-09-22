//! A faithful Rust port of the Graphviz **dot** layout engine
//! (`lib/dotgen` + supporting `lib/common` code).
//!
//! The pipeline mirrors `dotLayout` in `dotinit.c`:
//!
//! 1. **rank** — `rank.rs`: `dot1_rank` = collapse_sets → class1 →
//!    minmax_edges → decompose → acyclic → network simplex (`ns.rs`) →
//!    expand_ranksets → cleanup1;
//! 2. **mincross** — crossing minimization (median + transpose);
//! 3. **position** — x/y coordinates (aux-graph simplex for x);
//! 4. **splines** — edge routing (`dotsplines.c`), then postprocessing.
//!
//! Everything operates on the [`Fg`] arena (`model.rs`), mirroring the C
//! data model field for field so the algorithms can be transcribed
//! verbatim. `docs/graphviz-specs/*.md` documents each C source file with
//! line citations.

pub mod arrows;
pub mod classes;
pub mod conc;
pub mod cluster;
pub mod flat;
pub mod geom;
pub mod mincross;
pub mod newrank;
pub mod model;
pub mod pathplan;
pub mod position;
pub mod postprocess;
pub mod splines;
pub mod dot_splines;
pub mod ns;
pub mod rank;
pub mod sameport;
pub mod shapes;

use crate::graph::model::Graph;
use geom::PointF;
use model::{ClustType, DEdge, DGraph, DNode, Fg, GId, RankDir, DEF_FONTSIZE};

/// Measured sizes in points, produced by the text system (or the analytic
/// estimator in tests). Parallel to `Graph::nodes` / `Graph::edges`.
#[derive(Debug, Clone, Default)]
pub struct Measured {
    /// Node sizes `(width, height)` in points. When [`Self::label`] is empty
    /// these are the *final* box sizes (test/oracle mode); otherwise they are
    /// ignored in favour of the shape's own sizing.
    pub node: Vec<(f64, f64)>,
    /// Node label text extents in points. When present, `build` runs dot's
    /// `poly_init` sizing (`label + PAD`, attribute lower bounds, shapes) —
    /// the mode the GUI uses, since it measures real text.
    pub label: Vec<(f64, f64)>,
    /// Edge label text sizes in points, when the edge has a label.
    pub edge_label: Vec<Option<(f64, f64)>>,
    /// Per-string text measurement in points, used for record-field sizing.
    /// Without it the engine falls back to an analytic estimate
    /// ([`analytic_measure`]), which is what the headless `--dump`/`--svg`
    /// paths use; the GUI passes the real text system here.
    pub measure: Option<TextMeasure>,
}

/// A text measurer the engine may call for per-field record sizing.
#[derive(Clone)]
pub struct TextMeasure(pub std::rc::Rc<dyn Fn(&str) -> PointF>);

impl std::fmt::Debug for TextMeasure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("TextMeasure")
    }
}

/// Analytic per-string estimate: `0.5 em` per character, one `1.2 em` line.
/// Matches [`crate::dump`]'s estimator, so the headless harnesses stay
/// self-consistent.
pub fn analytic_measure(fontsize: f64) -> TextMeasure {
    TextMeasure(std::rc::Rc::new(move |text: &str| {
        let lines = text.chars().filter(|&c| c == '\n').count() + 1;
        let width = text
            .split('\n')
            .map(|l| l.chars().count() as f64 * fontsize * 0.5)
            .fold(0.0f64, f64::max);
        PointF::new(width, lines as f64 * fontsize * 1.2)
    }))
}

/// gv_math.h `scale_clamp`.
pub fn scale_clamp(original: usize, scale: f64) -> i32 {
    assert!(original as i64 >= 0);
    if scale < 0.0 {
        return 0;
    }
    let product = original as f64 * scale;
    if product > i32::MAX as f64 {
        return i32::MAX;
    }
    product as i32
}

/// `POINTS()` — inches to integer points, rounding half away from zero.
fn points(inches: f64) -> i32 {
    geom::round_i(inches * 72.0)
}

/// Parses a double attribute with a default (`late_double`).
fn late_double(value: Option<&String>, def: f64) -> f64 {
    value
        .and_then(|s| s.trim().parse::<f64>().ok())
        .unwrap_or(def)
}

/// Parses an int attribute with a default and a minimum (`late_int`).
fn late_int(value: Option<&String>, def: i32, min: i32) -> i32 {
    match value {
        Some(s) => {
            let v = s.trim().parse::<i32>().unwrap_or(def);
            v.max(min)
        }
        None => def,
    }
}

/// `mapbool` — "true"/"yes"/nonzero numbers…; empty means false.
pub(crate) fn mapbool(value: Option<&String>) -> bool {
    match value {
        None => false,
        Some(s) => match s.trim().to_lowercase().as_str() {
            "" | "false" | "no" | "0" => false,
            "true" | "yes" => true,
            other => other.parse::<f64>().map(|f| f != 0.0).unwrap_or(true),
        },
    }
}

/// `chkPort` (common/utils.c:478-495) for one edge endpoint: resolve the
/// port text through the node's shape (`poly_port`/`record_port`), record it
/// on the fast node as `ND_has_port`, and honour `tailclip`/`headclip`.
///
/// The port text comes from the `:port` reference syntax or an explicit
/// `tailport`/`headport` attribute (the parser already gives the attribute
/// priority, mirroring cgraph).
fn resolve_edge_port(
    fg: &mut Fg,
    graph: &Graph,
    node: usize,
    syntax_port: Option<&str>,
    attr_port: Option<&str>,
    edge_attrs: &crate::graph::model::Attributes,
    rankdir: RankDir,
    clip_attr: &str,
) -> model::Port {
    let raw = match attr_port.filter(|s| !s.is_empty()) {
        Some(attr) => attr,
        None => match syntax_port.filter(|s| !s.is_empty()) {
            Some(syntax) => syntax,
            None => return model::Port::default(),
        },
    };
    let input = &graph.nodes[node];
    let shape_name = input
        .attrs
        .get("shape")
        .map(String::as_str)
        .unwrap_or("ellipse");
    let desc = shapes::shape_of(shape_name);
    let dnode = &fg.nodes[node];
    let info = dnode.shape_info.clone();
    let (lw, rw, ht) = (dnode.lw, dnode.rw, dnode.ht);
    let record = fg.records.get(&node);
    let resolved = shapes::compass_port_rankdir(
        &desc,
        lw,
        rw,
        ht,
        Some(&info),
        record,
        raw,
        rankdir,
    );
    fg.nodes[node].has_port = true;
    let mut port = resolved.into_model_port();
    // `noClip(e, E_tailclip)`: an explicit false turns clipping off.
    if edge_attrs
        .get(clip_attr)
        .is_some_and(|v| !mapbool(Some(v)))
    {
        port.clip = false;
    }
    port
}

/// Builds the fast graph arena from a parsed graph. Mirrors
/// `dot_init_subg` / `dot_init_node` / `dot_init_edge`.
pub fn build(graph: &Graph, measured: &Measured) -> Fg {
    let mut fg = Fg::new();

    // -- graph attributes ---------------------------------------------------
    let attrs = graph.graph_attrs();
    let rankdir = match attrs.get("rankdir").map(String::as_str) {
        Some("LR") => RankDir::Lr,
        Some("BT") => RankDir::Bt,
        Some("RL") => RankDir::Rl,
        _ => RankDir::Tb,
    };
    let nodesep = points(late_double(attrs.get("nodesep"), 0.25).max(0.02));
    let ranksep = points(late_double(attrs.get("ranksep"), 0.5).max(0.02));
    let cl_type = match attrs.get("clusterrank").map(String::as_str) {
        Some("global") => ClustType::Global,
        Some("none") => ClustType::NoClust,
        _ => ClustType::Local,
    };
    fg.cl_type = cl_type;
    fg.has_edge_labels = graph.edges.iter().any(|e| e.label().is_some());
    // `Concentrate` (dotinit.c) — the global `concentrate` attribute.
    fg.concentrate = attrs.get("concentrate").is_some_and(|v| mapbool(Some(v)));

    // -- real nodes ----------------------------------------------------------
    for (i, input) in graph.nodes.iter().enumerate() {
        // dot sizes a node from its label plus the shape's padding, with the
        // `width`/`height` attributes as lower bounds — `poly_init`.
        let shape_name = input
            .attrs
            .get("shape")
            .map(String::as_str)
            .unwrap_or("ellipse");
        let desc = shapes::shape_of(shape_name);
        let penwidth = late_double(input.attrs.get("penwidth"), 1.0);
        let fontsize = late_double(input.attrs.get("fontsize"), DEF_FONTSIZE);
        // `record_init` (shapes.c:3625-3710): parse the label into fields,
        // size each from its text, and add the +1pt height kluge. The layout
        // is kept for port resolution and clipping.
        let record = (desc.fns == shapes::ShapeFns::Record).then(|| {
            let measure = measured
                .measure
                .clone()
                .unwrap_or_else(|| analytic_measure(fontsize));
            let measure = measure.0;
            // `flip = !GD_realflip(...)` (shapes.c:3672): records are laid
            // out left-to-right unless the graph is rotated.
            shapes::record_layout(&input.label(), &input.attrs, !rankdir.flip(), &|s| measure(s))
        });
        let record_size = record.as_ref().map(shapes::record_node_size);
        let (w, h, init_vertices, computed) = match (record_size, measured.label.get(i).copied())
        {
            // `record_init` sized the fields already (+1pt height kluge).
            (Some((rw, rh)), _) => (rw, rh, None, None),
            (None, Some((tw, th))) => {
                let init = shapes::poly_init_info(
                    &desc,
                    PointF::new(tw, th),
                    &input.attrs,
                    penwidth,
                    late_double(attrs.get("quantum"), 0.0),
                );
                (
                    init.width_in * 72.0,
                    init.height_in * 72.0,
                    Some(init.poly.vertices.clone()),
                    Some(init.poly),
                )
            }
            // final-size mode (tests, the oracle harness): the caller already
            // resolved the box.
            (None, None) => (
                measured.node.get(i).copied().unwrap_or((54.0, 36.0)).0,
                measured.node.get(i).copied().unwrap_or((54.0, 36.0)).1,
                None,
                None,
            ),
        };
        let flip = rankdir.flip();
        let (lw, rw, ht) = if flip {
            (h / 2.0, h / 2.0, w)
        } else {
            (w / 2.0, w / 2.0, h)
        };
        // Label mode resolves the real shape (so clipping/ports follow
        // shapes.c); final-size mode leaves the shape unset, which the
        // rectangle fallback in `shape_inside` expects. The fields come from
        // the *computed* polygon — poly_init resolves user `sides`/`skew`/
        // `distortion`/`orientation`/`regular`/`peripheries` on top of the
        // table defaults, and `shape=polygon` has table `sides` 0, so only
        // the computed value can tell a 9-gon from an ellipse.
        let shape_info = if init_vertices.is_some() {
            let poly = computed.expect("label mode computed the polygon");
            model::ShapeInfo {
                kind: desc.name.to_string(),
                sides: poly.sides,
                skew: poly.skew,
                distortion: poly.distortion,
                regular: poly.regular,
                peripheries: poly.peripheries,
                orientation: poly.orientation,
                vertices: init_vertices,
            }
        } else {
            model::ShapeInfo::default()
        };
        let node_id = fg.nodes.len();
        fg.nodes.push(DNode {
            name: input.name.clone(),
            real: Some(i),
            width: w / 72.0,
            height: h / 72.0,
            penwidth,
            lw,
            rw,
            ht,
            shape_info,
            ..Default::default()
        });
        if let Some(record) = record {
            fg.records.insert(node_id, record);
        }
    }
    let n_real = fg.nodes.len();

    // -- original edges (dot_init_edge) --------------------------------------
    for (i, input) in graph.edges.iter().enumerate() {
        let mut weight = late_int(input.attrs.get("weight"), 1, 0);
        let mut xpenalty = 1;
        let tail_group = graph.nodes[input.tail].attrs.get("group").cloned();
        let head_group = graph.nodes[input.head].attrs.get("group").cloned();
        if let (Some(tg), Some(hg)) = (&tail_group, &head_group) {
            if !tg.is_empty() && tg == hg {
                xpenalty = model::CL_CROSS;
                weight *= 100;
            }
        }
        // nonconstraint_edge: constraint attribute present and false
        let constraint_false = input
            .attrs
            .get("constraint")
            .map(|v| !mapbool(Some(v)))
            .unwrap_or(false);
        if constraint_false {
            xpenalty = 0;
            weight = 0;
        }
        let minlen = late_int(input.attrs.get("minlen"), 1, 0);

        fg.nonconstraint.push(constraint_false);
        let dir_attr = input.attrs.get("dir").cloned();
        let arrowhead_attr = input.attrs.get("arrowhead").cloned();
        let arrowtail_attr = input.attrs.get("arrowtail").cloned();

        // label record
        let label = input.label().map(|text| {
            let (tw, th) = measured
                .edge_label
                .get(i)
                .copied()
                .flatten()
                .unwrap_or((text.len() as f64 * 7.0, 14.0));
            let label = model::Label {
                dimen: PointF::new(tw, th),
                pos: PointF::ZERO,
                fontsize: late_double(input.attrs.get("fontsize"), DEF_FONTSIZE),
            };
            fg.labels.push(label);
            fg.labels.len() - 1
        });

        // `common_init_edge`'s port half (common/utils.c:534-560): `chkPort`
        // turns the edge's `tailport`/`headport` (written either as an
        // attribute or by the `a:port -> b` syntax) into a resolved port, and
        // flags the endpoint node as having one.
        let tail_port = resolve_edge_port(
            &mut fg,
            graph,
            input.tail,
            input.tail_port.as_ref().map(|p| p.raw.as_str()),
            input.attrs.get("tailport").map(String::as_str),
            &input.attrs,
            rankdir,
            "tailclip",
        );
        let head_port = resolve_edge_port(
            &mut fg,
            graph,
            input.head,
            input.head_port.as_ref().map(|p| p.raw.as_str()),
            input.attrs.get("headport").map(String::as_str),
            &input.attrs,
            rankdir,
            "headclip",
        );
        fg.edges.push(DEdge {
            tail: input.tail,
            head: input.head,
            tail_port,
            head_port,
            input: Some(i),
            weight,
            minlen,
            xpenalty,
            count: 1,
            label,
            seq: i,
            arrowsize: late_double(input.attrs.get("arrowsize"), 1.0),
            penwidth: late_double(input.attrs.get("penwidth"), 1.0),
            dir_attr,
            arrowhead_attr,
            arrowtail_attr,
            samehead: input.attrs.get("samehead").cloned(),
            sametail: input.attrs.get("sametail").cloned(),
            ..Default::default()
        });
    }
    fg.n_orig_edges = fg.edges.len();

    // adjacency in declaration order (`agfstout` / `agfstin` iteration)
    fg.input_out = vec![Vec::new(); n_real];
    fg.input_in = vec![Vec::new(); n_real];
    for (e, input) in graph.edges.iter().enumerate() {
        if (input.tail as usize) < n_real {
            fg.input_out[input.tail].push(e);
        }
        if (input.head as usize) < n_real {
            fg.input_in[input.head].push(e);
        }
    }

    // -- graphs (root + one per input subgraph) ------------------------------
    fg.sg_to_g = vec![0; graph.subgraphs.len()];
    let root = DGraph {
        name: graph.name.clone().unwrap_or_default(),
        input: None,
        level: 0,
        nodesep,
        ranksep,
        rankdir,
        nodes_order: (0..n_real).collect(),
        has_labels: if fg.has_edge_labels { 1 } else { 0 },
        newrank: mapbool(attrs.get("newrank")),
        search_size: attrs
            .get("searchsize")
            .and_then(|s| s.trim().parse::<i32>().ok())
            .unwrap_or(-1),
        tbbalance: attrs.get("TBbalance").cloned(),
        nslimit1: attrs.get("nslimit1").and_then(|s| s.trim().parse::<f64>().ok()),
        nslimit: attrs.get("nslimit").and_then(|s| s.trim().parse::<f64>().ok()),
        ..Default::default()
    };
    fg.graphs.push(root);

    // create a DGraph per input subgraph, parents before children
    for (sgi, sg) in graph.subgraphs.iter().enumerate().skip(1) {
        // Cluster labels: dot reserves their space in `do_graph_label`; the
        // engine only needs the text and an extent (the GUI's text system
        // measures the real glyphs at paint time).
        let label = sg.attrs.get("label").filter(|l| !l.is_empty()).map(|text| {
            let lines: usize = text.chars().filter(|&c| c == '\n').count() + 1;
            let width = text
                .split('\n')
                .map(|l| l.chars().count() as f64 * 7.0)
                .fold(0.0f64, f64::max);
            let fontsize = sg
                .attrs
                .get("fontsize")
                .and_then(|v| v.trim().parse::<f64>().ok())
                .unwrap_or(DEF_FONTSIZE);
            model::Label {
                dimen: PointF::new(width, lines as f64 * fontsize * 1.2),
                pos: PointF::ZERO,
                fontsize,
            }
        });
        // `do_graph_label` (common/input.c:830-891): a subgraph label's
        // placement flags and — for clusters — the border band it reserves.
        // Clusters default to a *top* label, the root to a bottom one.
        let label_pos: u8 = {
            let mut pos = if sg.is_cluster {
                model::LABEL_AT_TOP
            } else {
                model::LABEL_AT_BOTTOM
            };
            match sg.attrs.get("labelloc").map(|v| v.trim().to_lowercase()) {
                Some(v) if v.starts_with('b') => pos = model::LABEL_AT_BOTTOM,
                Some(v) if v.starts_with('t') => pos = model::LABEL_AT_TOP,
                _ => {}
            }
            match sg.attrs.get("labeljust").map(|v| v.trim().to_lowercase()) {
                Some(v) if v.starts_with('l') => pos |= model::LABEL_AT_LEFT,
                Some(v) if v.starts_with('r') => pos |= model::LABEL_AT_RIGHT,
                _ => {}
            }
            pos
        };
        let mut border = [geom::PointF::ZERO; 4]; // bottom, right, top, left
        if let Some(label) = &label {
            let mut dimen = label.dimen;
            dimen.x += 16.0; // PAD(dimen): x += 4*GAP
            dimen.y += 8.0; //              y += 2*GAP
            if !rankdir.flip() {
                let ix = if label_pos & model::LABEL_AT_TOP != 0 { 2 } else { 0 };
                border[ix] = dimen;
            } else {
                let ix = if label_pos & model::LABEL_AT_TOP != 0 { 1 } else { 3 };
                border[ix] = geom::PointF::new(dimen.y, dimen.x);
            }
        }
        let g = DGraph {
            name: sg.name.clone(),
            label,
            label_pos,
            border,
            input: Some(sgi),
            // `is_a_cluster` (common/utils.c:684): the name starts with
            // "cluster" (case insensitive) or the `cluster` attribute is true.
            // The parser records both on the subgraph.
            is_cluster_name: sg.is_cluster,
            remincross: sg.attrs.get("remincross").cloned(),
            cluster_flag: sg
                .attrs
                .get("cluster")
                .is_some_and(|v| mapbool(Some(v))),
            rank_attr: sg.attrs.get("rank").cloned(),
            compact: sg.attrs.get("compact").is_some_and(|v| mapbool(Some(v))),
            // `rank()` (ns.c:1029-1040) reads `searchsize` *per graph* and
            // falls back to SEARCHSIZE, so a subgraph without the attribute
            // must not collapse to 0 (which would take the first negative
            // edge instead of the best of 30 on every pivot).
            search_size: sg
                .attrs
                .get("searchsize")
                .and_then(|v| v.trim().parse::<i32>().ok())
                .unwrap_or(-1),
            nodesep,
            ranksep,
            rankdir,
            nodes_order: sg.nodes.clone(),
            edges_order: sg.edges.clone(),
            ..Default::default()
        };
        fg.graphs.push(g);
        fg.sg_to_g[sgi] = fg.graphs.len() - 1;
    }
    // wire children (in creation order), root included
    for (sgi, sg) in graph.subgraphs.iter().enumerate() {
        let g = fg.sg_to_g[sgi];
        let children: Vec<GId> = sg.children.iter().map(|&c| fg.sg_to_g[c]).collect();
        fg.graphs[g].children = children;
    }

    // edge label sizes recorded for later phases
    fg.edge_label_sizes = measured.edge_label.clone();
    fg.node_sizes = measured.node.clone();

    // resolve arrow flags on every original edge
    for e in 0..fg.n_orig_edges {
        arrows::resolve_into(&mut fg, e);
    }

    fg
}

/// The layout result handed to the rendering layer.
#[derive(Debug, Clone, Default)]
pub struct DotLayout {
    /// Node centers in points (y up), parallel to `Graph::nodes`.
    pub coords: Vec<PointF>,
    /// Node sizes in points.
    pub sizes: Vec<(f64, f64)>,
    /// Rank assignment per real node (from the rank phase).
    pub ranks: Vec<i32>,
    /// Drawing bounding box.
    pub bb: geom::BoxF,
    /// Render-facing per-node geometry (shape, size, label).
    pub nodes: Vec<NodeOut>,
    /// Render-facing per-edge geometry (Béziers, arrow flags, label).
    pub edges: Vec<EdgeOut>,
    /// Cluster boxes and labels.
    pub clusters: Vec<ClusterOut>,
}

/// A placed label.
#[derive(Debug, Clone, Default)]
pub struct LabelOut {
    /// Text with the object escapes (`\N`, `\G`, …) resolved and the line
    /// breaks turned into real newlines — `make_simple_label`'s output.
    pub text: String,
    /// Per-line justification (`'l'`, `'r'` or `'n'`), one entry per line of
    /// [`Self::text`](common/labels.c:245-269 `emit_label`).
    pub just: Vec<char>,
    /// Center of the label in dot coordinates (y up).
    pub pos: PointF,
    pub dimen: PointF,
    pub fontsize: f64,
}

/// The render-facing text of a node label: object escapes substituted and the
/// `\n`/`\l`/`\r` line breaks split out with their justification. HTML labels
/// keep their source text (`make_label` skips both steps for them).
fn node_label_text(graph: &Graph, index: usize, raw: &str, html: bool) -> (String, Vec<char>) {
    if html {
        return (raw.to_string(), vec!['n']);
    }
    lines_text(graph.node_label_lines(index), raw)
}

/// The render-facing text of an edge label.
fn edge_label_text(graph: &Graph, index: usize, raw: &str, html: bool) -> (String, Vec<char>) {
    if html {
        return (raw.to_string(), vec!['n']);
    }
    match graph.edge_label_lines(index) {
        Some(lines) => lines_text(lines, raw),
        None => (raw.to_string(), vec!['n']),
    }
}

/// The render-facing text of a (cluster) subgraph label.
fn sg_label_text(graph: &Graph, sg: usize, raw: &str, html: bool) -> (String, Vec<char>) {
    if html {
        return (raw.to_string(), vec!['n']);
    }
    match graph.subgraph_label_lines(sg) {
        Some(lines) => lines_text(lines, raw),
        None => (raw.to_string(), vec!['n']),
    }
}

fn lines_text(lines: Vec<crate::graph::model::LabelLine>, raw: &str) -> (String, Vec<char>) {
    if lines.is_empty() {
        return (raw.to_string(), vec!['n']);
    }
    let just = lines.iter().map(|l| l.just).collect();
    let text = lines.iter().map(|l| l.text.as_str()).collect::<Vec<_>>().join("\n");
    (text, just)
}

/// Render-facing node geometry (dot coordinates, y up, points).
#[derive(Debug, Clone, Default)]
pub struct NodeOut {
    /// Index into `Graph::nodes`.
    pub input: usize,
    pub coord: PointF,
    pub lw: f64,
    pub rw: f64,
    pub ht: f64,
    pub shape: model::ShapeInfo,
    pub penwidth: f64,
    pub label: Option<LabelOut>,
    /// For `shape=record`: the painted field boxes, divider segments and
    /// per-field labels (layout frame, node-relative). Empty otherwise.
    pub record: Option<RecordOut>,
}

/// A record node's painted structure: leaf fields (box + text) and the
/// segments separating siblings.
#[derive(Debug, Clone, Default)]
pub struct RecordOut {
    pub fields: Vec<shapes::RecordPart>,
    pub dividers: Vec<(PointF, PointF)>,
}

/// Render-facing edge geometry (dot coordinates, y up, points).
#[derive(Debug, Clone, Default)]
pub struct EdgeOut {
    /// Index into `Graph::edges`.
    pub input: usize,
    pub tail: usize,
    pub head: usize,
    /// Cubic Bézier segments sharing endpoints (3k+1 control points).
    pub segments: Vec<[PointF; 4]>,
    pub sflag: u32,
    pub eflag: u32,
    /// Arrow tip anchors (`sp` = tail-side, `ep` = head-side).
    pub sp: PointF,
    pub ep: PointF,
    pub penwidth: f64,
    pub label: Option<LabelOut>,
}

/// A cluster box (dot coordinates).
#[derive(Debug, Clone, Default)]
pub struct ClusterOut {
    pub name: String,
    pub bb: geom::BoxF,
    pub label: Option<LabelOut>,
}

/// Runs the full dot pipeline on a parsed graph.
pub fn layout(graph: &Graph, measured: &Measured) -> DotLayout {
    let timing = std::env::var_os("GD_TIMING").is_some();
    let t0 = std::time::Instant::now();
    let mut fg = build(graph, measured);
    if timing {
        eprintln!("[timing] build: {:.3}s", t0.elapsed().as_secs_f64());
    }
    let t1 = std::time::Instant::now();
    dot_layout(&mut fg);
    if timing {
        eprintln!("[timing] dot_layout: {:.3}s", t1.elapsed().as_secs_f64());
    }
    let t2 = std::time::Instant::now();
    let out = to_layout(&fg, graph);
    if timing {
        eprintln!("[timing] to_layout: {:.3}s", t2.elapsed().as_secs_f64());
    }
    out
}

/// `dot_layout` — the full pipeline, mirroring `dotLayout` in `dotinit.c`:
/// rank → mincross → position → (sameports) → splines → postprocess.
pub fn dot_layout(fg: &mut Fg) {
    let timing = std::env::var_os("GD_TIMING").is_some();
    let mut t = std::time::Instant::now();
    let mut lap = |name: &str| {
        if timing {
            eprintln!("[timing] {name}: {:.3}s", t.elapsed().as_secs_f64());
            t = std::time::Instant::now();
        }
    };
    rank::dot_rank(fg, 0);
    lap("rank");
    let _ = mincross::dot_mincross(fg, 0);
    lap("mincross");
    let _ = position::dot_position(fg, 0);
    lap("position");
    // `dot_sameports` (sameport.c) — merge samehead/sametail groups onto one
    // port now that the coordinates are final (:296 in dotinit.c).
    sameport::dot_sameports(fg, 0);
    lap("sameports");
    let _ = dot_splines::dot_splines(fg, 0);
    lap("splines");
    postprocess::gv_postprocess(fg, true);
    lap("postprocess");
}

/// Extracts the current layout state (partial pipelines yield coordinates
/// only for the phases that have run).
fn to_layout(fg: &Fg, graph: &Graph) -> DotLayout {
    let real = fg.nodes.iter().take_while(|n| n.real.is_some()).count();
    let coords: Vec<PointF> = fg.nodes[..real].iter().map(|n| n.coord).collect();
    let ranks: Vec<i32> = fg.nodes[..real].iter().map(|n| n.rank).collect();
    let sizes: Vec<(f64, f64)> = fg.nodes[..real]
        .iter()
        .map(|n| (n.lw + n.rw, n.ht))
        .collect();

    let label_out = |rec: Option<usize>, text: (String, Vec<char>)| -> Option<LabelOut> {
        let idx = rec?;
        let l = fg.labels.get(idx)?;
        let mut l = l.clone();
        if l.fontsize <= 0.0 {
            l.fontsize = DEF_FONTSIZE;
        }
        Some(LabelOut {
            text: text.0,
            just: text.1,
            pos: l.pos,
            dimen: l.dimen,
            fontsize: l.fontsize,
        })
    };

    let nodes: Vec<NodeOut> = fg.nodes[..real]
        .iter()
        .enumerate()
        .map(|(i, n)| NodeOut {
            input: i,
            coord: n.coord,
            lw: n.lw,
            rw: n.rw,
            ht: n.ht,
            shape: n.shape_info.clone(),
            penwidth: late_double(graph.nodes[i].attrs.get("penwidth"), 1.0),
            record: fg.records.get(&i).map(|record| {
                let (fields, dividers) = shapes::record_parts(record);
                RecordOut { fields, dividers }
            }),
            label: label_out(
                n.label,
                node_label_text(
                    graph,
                    i,
                    &graph.nodes[i].label(),
                    graph.nodes[i].label_is_html(),
                ),
            )
            .or_else(|| {
                let text = graph.nodes[i].label();
                (!text.is_empty()).then(|| {
                    let (text, just) =
                        node_label_text(graph, i, &text, graph.nodes[i].label_is_html());
                    LabelOut {
                        text,
                        just,
                        pos: n.coord,
                        dimen: PointF::new(n.lw + n.rw, n.ht),
                        fontsize: late_double(graph.nodes[i].attrs.get("fontsize"), DEF_FONTSIZE),
                    }
                })
            }),
        })
        .collect();

    let mut edges: Vec<EdgeOut> = Vec::with_capacity(fg.n_orig_edges);
    for e in 0..fg.n_orig_edges {
        let d = &fg.edges[e];
        let input = d.input.unwrap_or(e);
        let segments = d
            .spl
            .as_ref()
            .map(|s| s.list.clone())
            .unwrap_or_default();
        let label = d.label.and_then(|idx| {
            let raw = graph.edges[input].label().unwrap_or_default();
            let html = graph.edges[input].label_is_html();
            label_out(Some(idx), edge_label_text(graph, input, &raw, html))
        });
        edges.push(EdgeOut {
            input,
            tail: graph.edges[input].tail,
            head: graph.edges[input].head,
            segments,
            sflag: d.sflag,
            eflag: d.eflag,
            sp: d.spl.as_ref().map(|s| s.sp).unwrap_or(PointF::ZERO),
            ep: d.spl.as_ref().map(|s| s.ep).unwrap_or(PointF::ZERO),
            penwidth: d.penwidth,
            label,
        });
    }

    let clusters: Vec<ClusterOut> = (1..graph.subgraphs.len())
        .filter_map(|sgi| {
            let gid = *fg.sg_to_g.get(sgi)?;
            let g = fg.graphs.get(gid)?;
            (g.bb.ur.x > g.bb.ll.x).then_some((sgi, g))
        })
        .map(|(sgi, g)| ClusterOut {
            name: g.name.clone(),
            bb: g.bb,
            label: g.label.clone().map(|l| {
                let raw = graph.subgraphs[sgi].attrs.get("label").cloned().unwrap_or_default();
                let html = graph.label_is_html(sgi);
                let (text, just) = sg_label_text(graph, sgi, &raw, html);
                LabelOut {
                    text,
                    just,
                    pos: l.pos,
                    dimen: l.dimen,
                    fontsize: l.fontsize,
                }
            }),
        })
        .collect();

    DotLayout {
        coords,
        ranks,
        sizes,
        bb: fg.graphs[0].bb,
        nodes,
        edges,
        clusters,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::parser::parse;

    fn default_measured(n: usize) -> Measured {
        Measured {
            label: Vec::new(),
            node: vec![(54.0, 36.0); n],
            edge_label: Vec::new(),
            measure: None,
        }
    }

    #[test]
    fn ranks_a_simple_dag() {
        let g = parse("digraph { a -> b -> c; }").unwrap();
        let mut fg = build(&g, &default_measured(g.nodes.len()));
        dot_layout(&mut fg);
        let ranks: Vec<i32> = fg.nodes[..3].iter().map(|n| n.rank).collect();
        assert_eq!(ranks, vec![0, 1, 2]);
    }

    #[test]
    fn ranks_reverse_edges_and_cycles() {
        let g = parse("digraph { a -> b; c -> b; c -> a; }").unwrap();
        let mut fg = build(&g, &default_measured(g.nodes.len()));
        dot_layout(&mut fg);
        // longest path from sources: c=0, a=1, b=2
        let ranks: Vec<i32> = fg.nodes[..3].iter().map(|n| n.rank).collect();
        assert_eq!(ranks, vec![1, 2, 0]);
    }

    #[test]
    fn minlen_is_respected() {
        let g = parse("digraph { a -> b [minlen=3]; a -> c; }").unwrap();
        let mut fg = build(&g, &default_measured(g.nodes.len()));
        dot_layout(&mut fg);
        let ranks: Vec<i32> = fg.nodes[..3].iter().map(|n| n.rank).collect();
        assert_eq!(ranks[1] - ranks[0], 3);
        assert_eq!(ranks[2] - ranks[0], 1);
    }

    #[test]
    fn lr_layout_unflips_node_sizes() {
        // Regression: with rankdir=LR the engine swaps a node's dimensions
        // internally (`gv_nodesize(n, flip)`), and the postprocess pass must
        // swap them back — otherwise a box is painted as tall as it is wide
        // and the label spills out of it.
        let g = parse("digraph { rankdir=LR; a -> b; }").unwrap();
        let measured = Measured {
            label: Vec::new(),
            node: vec![(54.0, 36.0); 2],
            edge_label: Vec::new(),
                    measure: None,
        };
        let out = layout(&g, &measured);
        for (w, h) in &out.sizes {
            assert!(
                (*w - 54.0).abs() < 1e-6 && (*h - 36.0).abs() < 1e-6,
                "node size not unflipped: ({w}, {h})"
            );
        }
        // and the layers advance along x
        assert!(out.coords[1].x > out.coords[0].x);
    }

    #[test]
    fn rank_same_merges() {
        let g = parse("digraph { a -> b -> c; { rank=same; b; x; } }").unwrap();
        let mut fg = build(&g, &default_measured(g.nodes.len()));
        dot_layout(&mut fg);
        let rank_of = |name: &str| {
            let i = g.nodes.iter().position(|n| n.name == name).unwrap();
            fg.nodes[i].rank
        };
        assert_eq!(rank_of("b"), rank_of("x"));
    }
}
