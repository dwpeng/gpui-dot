//! The fast-graph data model — a faithful Rust mirror of the `Agraphinfo_t` /
//! `Agnodeinfo_t` / `Agedgeinfo_t` fields dotgen uses (`lib/common/types.h`,
//! `lib/dotgen/fastgr.c`).
//!
//! C reaches everything through interior pointers; this port uses arena
//! indices. The pipeline owns one [`Fg`] (fast graph) holding every node
//! (real + virtual), every edge (original + virtual + aux) and one
//! [`DGraph`] per root/cluster. Field names and iteration semantics —
//! including the order-destroying `zapinlist` swap-removal and the
//! prepend-order fast node list — mirror the C exactly so layout results are
//! deterministic in the same way.

use super::geom::{BoxF, PointF};

/// Fast node id (arena index).
pub type NId = usize;
/// Fast edge id (arena index).
pub type EId = usize;
/// Graph id (root is 0, clusters follow in `GD_clust` order).
pub type GId = usize;

/// `ND_node_type` (const.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NodeType {
    /// An original input node.
    #[default]
    Normal = 0,
    /// Virtual nodes in long edge chains.
    Virtual = 1,
    /// Encodes inter-cluster constraints during ranking.
    Slacknode = 2,
}

/// `ND_ranktype` (const.h "collapsed node classifications").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RankType {
    #[default]
    Normal = 0,
    SameRank = 1,
    MinRank = 2,
    SourceRank = 3,
    MaxRank = 4,
    SinkRank = 5,
    LeafSet = 6,
    Cluster = 7,
}

/// `ED_edge_type` (const.h "node,edge types" — REVERSED etc.).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeType {
    #[default]
    Normal = 0,
    Virtual = 1,
    Reversed = 3,
    FlatOrder = 4,
    ClusterEdge = 5,
    Ignored = 6,
}

/// `EDGETYPE_*` — the splines flavor set by `setEdgeType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SplineType {
    #[default]
    None = 0,
    Line = 1,
    Curved = 2,
    Pline = 3,
    Spline = 5,
}

/// rankdir constants (const.h) — `GD_rankdir2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RankDir {
    #[default]
    Tb = 0,
    Lr = 1,
    Bt = 2,
    Rl = 3,
}

impl RankDir {
    /// `GD_flip(g) = GD_rankdir(g) & 1` (types.h:378) — the *transposing*
    /// rankdirs. `RANKDIR_LR = 1`, `RANKDIR_RL = 3`, `RANKDIR_BT = 2`, so only
    /// LR and RL swap the axes; BT merely reverses the edge direction.
    /// Treating BT as flipped (as an earlier revision did) lays it out
    /// transposed, and missing RL leaves it transposed.
    pub fn flip(self) -> bool {
        matches!(self, RankDir::Lr | RankDir::Rl)
    }
}

/// A rendered text label (`textlabel_t` subset dotgen relies on).
#[derive(Debug, Clone, Default)]
pub struct Label {
    /// Text extent in points.
    pub dimen: PointF,
    /// Position of the label center in points.
    pub pos: PointF,
    /// Font size in points (`fontsize`, default 14).
    pub fontsize: f64,
}

/// A resolved edge port (`port` struct subset).
#[derive(Debug, Clone, Copy)]
pub struct Port {
    /// Port position in node-relative coordinates (from `tailport`/compass).
    pub p: PointF,
    /// Bounding box for deferred port resolution (record/HTML cells).
    pub bp: BoxF,
    pub defined: bool,
    pub clip: bool,
    /// `MC_SCALE`-ordered sort key used by portcmp.
    pub order: u8,
    /// `_` compass — resolve the side at routing time (`resolvePort`).
    pub dyna: bool,
    pub theta: f64,
    /// Which side of the node (bit flags), for dyna ports.
    pub side: u8,
    pub constrained: bool,
}

impl Default for Port {
    /// C's `static port Center = {.theta = -1, .clip = true}`
    /// (shapes.c:38) — what every edge starts with: aim at the node centre
    /// and clip the spline to the node outline. An explicit port turns
    /// `clip` off.
    fn default() -> Self {
        Port {
            p: PointF::ZERO,
            bp: BoxF::default(),
            defined: false,
            clip: true,
            order: 0,
            dyna: false,
            theta: -1.0,
            side: 0,
            constrained: false,
        }
    }
}

/// A routed edge — the final `splines` result: one or more cubic Beziers.
#[derive(Debug, Clone, Default)]
pub struct Splines {
    /// Each segment: `p0, p1, p2, p3` control points.
    pub list: Vec<[PointF; 4]>,
    /// Arrow tip anchors (`bezier.sp` / `.ep`), set when the corresponding
    /// arrow flag is non-zero.
    pub sp: PointF,
    pub ep: PointF,
}

/// Fast node — mirrors `Agnodeinfo_t` (dotgen-relevant fields only).
#[derive(Debug, Clone, Default)]
pub struct DNode {
    /// Name for real nodes (virtual nodes get a synthetic one).
    pub name: String,
    /// Index into `Graph::nodes` when this is an input node.
    pub real: Option<usize>,
    pub node_type: NodeType,
    pub ranktype: RankType,
    /// Owning cluster (`ND_clust`), if any.
    pub clust: Option<GId>,
    /// Final layout coordinate (points, y up).
    pub coord: PointF,
    /// Half-widths and height in points (`ND_lw`, `ND_rw`, `ND_ht`).
    pub lw: f64,
    pub rw: f64,
    pub ht: f64,
    /// Attribute size in inches (`ND_width`, `ND_height`).
    pub width: f64,
    pub height: f64,
    /// `penwidth` attribute (`N_penwidth`, default 1.0) — widens the outline
    /// used by the inside tests and record clipping.
    pub penwidth: f64,
    /// Label record index (`ND_label` — a shared textlabel_t record).
    pub label: Option<usize>,
    /// Union-find over collapsed nodes (`ND_UF_parent`, `ND_UF_size`);
    /// `None` mirrors C's NULL parent (own root, uninitialized).
    pub uf_parent: Option<NId>,
    pub uf_size: usize,
    /// dot2 rank-set union-find (`ND_set`); `None` mirrors C's NULL parent
    /// (the node is its own root).
    pub set: Option<NId>,
    pub rank: i32,
    pub order: i32,
    pub mval: f64,
    /// `ND_hops`, overloaded by dot2 ranking as the connected-component id.
    pub hops: i32,
    /// Generic DFS stamp (`ND_mark`) — size_t in C, monotonic counter here.
    pub mark: usize,
    pub onstack: bool,
    pub weight_class: i32,
    pub has_port: bool,
    /// `ND_rep` — the Xg node representing this node in dot2 ranking.
    pub rep: Option<NId>,
    // --- fast graph elists ---
    pub out: Vec<EId>,
    pub in_: Vec<EId>,
    pub flat_out: Vec<EId>,
    pub flat_in: Vec<EId>,
    pub other: Vec<EId>,
    pub save_in: Vec<EId>,
    pub save_out: Vec<EId>,
    // --- fast node list ---
    pub next: Option<NId>,
    pub prev: Option<NId>,
    // --- network simplex ---
    pub tree_in: Vec<EId>,
    pub tree_out: Vec<EId>,
    pub par: Option<EId>,
    pub low: i32,
    pub lim: i32,
    pub priority: i32,
    /// Shape geometry (polygons/ellipses), filled by the shapes phase.
    pub shape_info: ShapeInfo,
}

/// Node shape resolution (from `shape_desc`/`polygon_t`).
#[derive(Debug, Clone, Default)]
pub struct ShapeInfo {
    /// Shape kind name as resolved (e.g. "box", "ellipse").
    pub kind: String,
    /// For polygon shapes: side count, skew, distortion, orientation.
    pub sides: i32,
    pub skew: f64,
    pub distortion: f64,
    pub regular: bool,
    pub peripheries: i32,
    pub orientation: f64,
    /// Precomputed unit vertices (normalized polygon), when polygonal.
    pub vertices: Option<Vec<PointF>>,
}

/// Fast edge — mirrors `Agedgeinfo_t` plus endpoints.
#[derive(Debug, Clone, Default)]
pub struct DEdge {
    pub tail: NId,
    pub head: NId,
    /// Index into `Graph::edges` for original edges.
    pub input: Option<usize>,
    /// `ED_to_orig` — the original fast edge a virtual edge represents.
    pub to_orig: Option<EId>,
    /// `ED_to_virt` — the first virtual edge representing this edge.
    pub to_virt: Option<EId>,
    pub edge_type: EdgeType,
    pub weight: i32,
    pub minlen: i32,
    pub xpenalty: i32,
    pub count: i32,
    pub cutvalue: i32,
    pub tree_index: i32,
    pub tail_port: Port,
    pub head_port: Port,
    /// Shared label record index (`Fg.labels`) when the edge has a label.
    pub label: Option<usize>,
    pub label_ontop: bool,
    /// True for flat edges with adjacent endpoints.
    pub adjacent: bool,
    pub conc_opp_flag: bool,
    /// AGSEQ — creation order used for tie-breaking.
    pub seq: usize,
    /// Final routing (set by the splines phase).
    pub spl: Option<Splines>,
    /// Resolved arrow flags (arrow_flags at build time; slots packed 4×8).
    pub sflag: u32,
    pub eflag: u32,
    /// `arrowsize` / `penwidth` attributes (defaults 1.0).
    pub arrowsize: f64,
    pub penwidth: f64,
    /// Raw arrow attributes for the flags resolution.
    pub dir_attr: Option<String>,
    pub arrowhead_attr: Option<String>,
    pub arrowtail_attr: Option<String>,
    /// `samehead` / `sametail` group ids (`dot_sameports`).
    pub samehead: Option<String>,
    pub sametail: Option<String>,
}

/// `rank_t` — one layer of the rank array.
#[derive(Debug, Clone, Default)]
pub struct Rank {
    pub v: Vec<NId>,
    pub n: usize,
    pub valid: bool,
    /// Flat-edge ordering bit matrix for this rank (`GD_rank[r].flat`),
    /// used by mincross's flat_search; indexed by flatindex.
    pub flat: Option<Vec<Vec<bool>>>,
    /// ht1/ht2 accumulators used by y-coordination (position.c) — these
    /// include cluster nesting and labels.
    pub ht1: f64,
    pub ht2: f64,
    /// `pht1`/`pht2` — the same accumulators restricted to *primitive* nodes
    /// (never raised by clusters or labels). `set_ycoords` uses them for the
    /// primitive rank separation `d0` and `ht1/ht2` for the cluster one `d1`.
    pub pht1: f64,
    pub pht2: f64,
    /// Cache slot mincross uses (`GD_rank[r].cache`).
    pub cache_nc: Option<NId>,
}

/// Per-graph (root or cluster) state — mirrors `Agraphinfo_t`.
#[derive(Debug, Clone, Default)]
pub struct DGraph {
    pub name: String,
    /// Index into `Graph::subgraphs` for clusters.
    pub input: Option<usize>,
    pub parent: Option<GId>,
    pub level: usize,
    /// `GD_clust[1..n_cluster]` — child clusters in declaration order.
    pub clust: Vec<GId>,
    /// Child subgraph graph-ids in creation order (`agfstsubg` iteration).
    pub children: Vec<GId>,
    /// Name starts with `cluster` (case-insensitive).
    pub is_cluster_name: bool,
    /// Explicit `cluster=true` graph attribute.
    pub cluster_flag: bool,
    /// The subgraph's `rank` attribute (rank sets).
    pub rank_attr: Option<String>,
    /// `TBbalance` attribute ("min"/"max").
    pub tbbalance: Option<String>,
    /// `searchsize` attribute; -1 = built-in default.
    pub search_size: i32,
    /// `nslimit1` attribute (scale factor).
    pub nslimit1: Option<f64>,
    /// `nslimit` attribute (scale factor).
    pub nslimit: Option<f64>,
    /// `newrank` attribute.
    pub newrank: bool,
    /// `compact` attribute — a "strong" cluster for dot2 ranking.
    pub compact: bool,
    /// `remincross` attribute — unset means "on" (mincross.c:386).
    pub remincross: Option<String>,
    /// Head of the fast node list (`GD_nlist`); prepend order via next/prev.
    pub nlist: Option<NId>,
    pub rank: Vec<Rank>,
    pub minrank: i32,
    pub maxrank: i32,
    /// Connected components (heads) — `GD_comp`.
    pub comp: Vec<NId>,
    pub minset: Option<NId>,
    pub maxset: Option<NId>,
    pub minrep: Option<NId>,
    pub maxrep: Option<NId>,
    pub leader: Option<NId>,
    /// Cluster skeleton rank leaders (`GD_rankleader`).
    pub rankleader: Vec<Option<NId>>,
    /// For an *expanded* cluster: the index in the root's row where this
    /// cluster's own slice starts, per rank (indexed like [`Self::rank`]).
    /// C aliases `GD_rank(subg)[r].v` into the root's row; this port keeps a
    /// copy plus this offset and refreshes the copy on every root write
    /// (see `cluster::refresh_expanded_clusters`).
    pub rank_offset: Vec<i32>,
    /// Input-node iteration order for this graph (`agfstnode`): the root
    /// lists every real node in declaration order; a cluster lists its
    /// direct members.
    pub nodes_order: Vec<NId>,
    /// Input-edge membership for clusters (`agfstout` of the subgraph).
    pub edges_order: Vec<EId>,
    pub expanded: bool,
    pub installed: u8,
    /// `GD_set_type` — the rank-set class of the subgraph.
    pub set_type: u8,
    pub label_pos: u8,
    pub exact_ranksep: bool,
    pub has_flat_edges: bool,
    pub has_labels: u8,
    pub nodesep: i32,
    pub ranksep: i32,
    /// Cluster label, if any.
    pub label: Option<Label>,
    /// Graph label margins (`GD_border[4]`), bottom/right/top/left order.
    pub border: [PointF; 4],
    pub bb: BoxF,
    pub ht1: f64,
    pub ht2: f64,
    pub rankdir: RankDir,
}

/// The fast graph arena.
#[derive(Debug, Clone, Default)]
pub struct Fg {
    pub graphs: Vec<DGraph>,
    pub nodes: Vec<DNode>,
    pub edges: Vec<DEdge>,
    /// Shared label records (`ND_label`/`ED_label` point at these).
    pub labels: Vec<Label>,
    /// Parsed record layouts per fast node — `ND_shape_info` for
    /// `shape=record`, needed for record ports and record clipping.
    pub records: std::collections::HashMap<NId, super::shapes::RecordLayout>,
    /// Monotonic edge sequence counter for virtual/aux edges without orig.
    pub seq: usize,
    /// Number of original edges (they occupy `edges[0..n_orig_edges]` in
    /// input order); virtual/aux edges follow.
    pub n_orig_edges: usize,
    /// Input-graph adjacency: per real node, its original edges in
    /// declaration order (`agfstout` iteration).
    pub input_out: Vec<Vec<EId>>,
    /// Reverse input adjacency: per real node, its original in-edges in
    /// declaration order (`agfstin` iteration) — used by `cluster.c`'s
    /// `interclexp`, whose `agfstedge` walks out-edges then in-edges.
    pub input_in: Vec<Vec<EId>>,
    /// Resolved `constraint=false` flags per input edge.
    pub nonconstraint: Vec<bool>,
    /// Global `concentrate` attribute.
    pub concentrate: bool,
    /// Monotonic stamp counter for DFS marking (`ND_mark` size_t protocol).
    pub stamp_next: usize,
    /// Resolved node sizes in points, per input node (from measurement).
    pub node_sizes: Vec<(f64, f64)>,
    /// Resolved edge label sizes in points per input edge, when labeled.
    pub edge_label_sizes: Vec<Option<(f64, f64)>>,
    /// `GD_has_labels(& EDGE_LABEL)` — any edge label exists.
    pub has_edge_labels: bool,
    /// `decomp.c` static `Cmark`.
    pub cmark: usize,
    /// `CL_type` — the root's `clusterrank` attribute.
    pub cl_type: ClustType,
    /// Input subgraph index → graph id.
    pub sg_to_g: Vec<GId>,
}

/// `CL_type` (const.h) — `clusterrank` attribute values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClustType {
    #[default]
    Local = 100,
    Global = 101,
    NoClust = 102,
}

/// Node constants (const.h).
pub const VIRTUAL: NodeType = NodeType::Virtual;
pub const SLACKNODE: NodeType = NodeType::Slacknode;

/// Cluster cost constants (const.h).
pub const CL_BACK: i32 = 10;
pub const CL_OFFSET: f64 = 8.0;
#[cfg(not(target_os = "windows"))]
pub const CL_CROSS: i32 = 1000;
#[cfg(target_os = "windows")]
pub const CL_CROSS: i32 = 100;

/// `LABEL_AT_*` (common/const.h:175-178) — graph label placement bits.
pub const LABEL_AT_BOTTOM: u8 = 0;
pub const LABEL_AT_TOP: u8 = 1;
pub const LABEL_AT_LEFT: u8 = 2;
pub const LABEL_AT_RIGHT: u8 = 4;

/// `MC_SCALE` (const.h) — virtual_weight scaling.
pub const MC_SCALE: i64 = 256;

/// Default attribute values (dots/attrs; see docs/graphviz-specs).
pub const DEF_FONTSIZE: f64 = 14.0;

impl Fg {
    pub fn new() -> Self {
        Self::default()
    }

    // -- union-find (utils.c UF_*) ------------------------------------------

    /// `UF_find` — follows ND_UF_parent to the root. `None` parent means
    /// uninitialized, which is its own root (utils.c treats NULL that way).
    pub fn uf_find(&self, mut n: NId) -> NId {
        loop {
            match self.nodes[n].uf_parent {
                Some(p) if p != n => n = p,
                _ => return n,
            }
        }
    }

    /// `UF_union` — the smaller-id root becomes the leader (utils.c).
    /// Returns the leader.
    pub fn uf_union(&mut self, u: NId, v: NId) -> NId {
        if u == v {
            return u;
        }
        let u = if self.nodes[u].uf_parent.is_none() {
            self.nodes[u].uf_parent = Some(u);
            self.nodes[u].uf_size = 1;
            u
        } else {
            self.uf_find(u)
        };
        let v = if self.nodes[v].uf_parent.is_none() {
            self.nodes[v].uf_parent = Some(v);
            self.nodes[v].uf_size = 1;
            v
        } else {
            self.uf_find(v)
        };
        if u == v {
            return u;
        }
        // ND_id order == arena creation order
        if u > v {
            self.nodes[u].uf_parent = Some(v);
            self.nodes[v].uf_size += self.nodes[u].uf_size;
            v
        } else {
            self.nodes[v].uf_parent = Some(u);
            self.nodes[u].uf_size += self.nodes[v].uf_size;
            u
        }
    }

    /// `UF_singleton`.
    pub fn uf_singleton(&mut self, n: NId) {
        self.nodes[n].uf_size = 1;
        self.nodes[n].uf_parent = None;
        self.nodes[n].ranktype = RankType::Normal;
    }

    /// `UF_setname` — make `leader` the root of `n`'s set (`n` must be a
    /// root already).
    pub fn uf_setname(&mut self, n: NId, leader: NId) {
        debug_assert_eq!(self.uf_find(n), n);
        self.nodes[n].uf_parent = Some(leader);
        self.nodes[leader].uf_size += self.nodes[n].uf_size;
    }
}
