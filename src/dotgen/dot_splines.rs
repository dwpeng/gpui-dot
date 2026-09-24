//! Edge spline routing — a faithful Rust port of `lib/dotgen/dotsplines.c`
//! (Graphviz `main` @ `2e92c7f`, 2316 lines), the phase that converts the
//! ranked/ordered/positioned fast graph into Bézier control-point lists
//! stored on the original edges (`ED_spl`).
//!
//! Pipeline position (`dotgen/dotinit.c:293-303`): runs after rank, mincross
//! and position; `dot_sameports` (not ported) would run first. The driver
//! (`dot_splines_`, dotsplines.c:228-480) works in two passes:
//!
//! 1. classify + collect every routable edge (`setflags`, the `rw`/`mval`
//!    swap trick for self-loop-inflated nodes, virtual node identity via its
//!    in/out chains) and sort them into equivalence groups with `edgecmp`
//!    (descending type bits, ascending |rank diff|, |Δx|, main-edge AGSEQ,
//!    ports, ascending graph bits, label, AGSEQ);
//! 2. route one group at a time: self loops → `makeSelfEdge`, flat edges →
//!    `make_flat_edge` family, regular (inter-rank) edges →
//!    `make_regular_edge` (the box-channel algorithm), CURVED →
//!    `makeStraightEdges`.
//!
//! The C model reaches interior pointers freely; this port uses the arena
//! (`Fg`). The one structural adaptation: C's `makefwdedge` builds a stack
//! copy of an edge record; here [`makefwdedge`] appends a materialized
//! reversed copy to `Fg::edges` (nothing iterates the arena during routing,
//! so the semantics are identical).
//!
//! Deviations from C (all noted at the site):
//! * `EDGETYPE_ORTHO` — `lib/ortho/ortho.c` is not ported. As when C is
//!   compiled without `#ifdef ORTHO`, the code falls through and routes with
//!   the polyline machinery (`routepolylines`).
//! * `make_flat_adj_edges` (dotsplines.c:1129-1288) runs a complete
//!   recursive dot layout on a rotated clone graph for flat edges with
//!   ports between adjacent nodes; that machinery is out of scope for this
//!   milestone and replaced by a simple spline through the mid-gap.
//! * `ED_alg` (flat-edge label vnodes) does not exist on `DNode`; it is
//!   reconstructed as "virtual node whose first out-edge is a FLATORDER
//!   edge" (exactly what `flat.c:flat_node` builds).
//! * `rank_t::pht1/pht2` are the primitive-only rank half-heights (see
//!   [`super::position`]); the flat-edge helpers below use the full
//!   `ht1`/`ht2`, matching dotsplines.c's reads.
//! * Head/tail port labels (`labelangle`/`labeldistance` globals) and the
//!   record-shape `pboxfn` (`record_path`) are not modeled.

// This module is the splines-phase public API; it is dead from the binary's
// perspective until the pipeline integration wires `dot_splines` into
// `dot_layout` (a later milestone).
#![allow(dead_code)]

use std::cmp::Ordering;

use super::geom::{BoxF, PointF, round};
use super::model::{self, EId, EdgeType, Fg, GId, NId, NodeType, Port, SplineType};
use super::pathplan::{make_polyline, proutespline, pshortestpath};
use super::splines::{Path, clip_and_install, routesplines_};

// ---------------------------------------------------------------------------
// constants (dotsplines.c:36-46, common/const.h:111-123, 150-155)
// ---------------------------------------------------------------------------

/// dotsplines.c:36 — allocation fudge only; irrelevant with growable Vecs.
#[allow(dead_code)]
const NSUB: usize = 9;
/// dotsplines.c:38 — minimum width of a box in the edge path.
const MINW: f64 = 16.0;
/// dotsplines.c:39 — `MINW / 2`.
const HALFMINW: f64 = 8.0;
/// dotsplines.c:41 — edge stored in the forward direction.
const FWDEDGE: i32 = 16;
/// dotsplines.c:42 — edge is reversed (needs `makefwdedge`).
const BWDEDGE: i32 = 32;
/// dotsplines.c:44 — edge belongs to the main graph.
const MAINGRAPH: i32 = 64;
/// dotsplines.c:45 — edge belongs to an aux graph (flat/self/other).
const AUXGRAPH: i32 = 128;
/// dotsplines.c:46 — `MAINGRAPH | AUXGRAPH`.
const GRAPHTYPEMASK: i32 = 192;
/// const.h:150 — tree_index edge-type bits.
const REGULAREDGE: i32 = 1;
/// const.h:151.
const FLATEDGE: i32 = 2;
/// const.h:152 — self edge with ports.
const SELFWPEDGE: i32 = 4;
/// const.h:153 — self edge without ports.
const SELFNPEDGE: i32 = 8;
/// const.h:155 — OR of the four edge-type bits.
const EDGETYPEMASK: i32 = 15;
/// dotsplines.c:944 — vertical gap between stacked flat-edge labels.
const LBL_SPACE: f64 = 6.0;
/// dotsplines.c:2173 — extra x-space in `maximal_bbox`.
const FUDGE: f64 = 4.0;
/// splines.c beginpath/endpath `FUDGE` (2) — used as `FUDGE-2 == 0` offsets.
const BEGIN_FUDGE: f64 = 2.0;
/// const.h — `SELF_EDGE_SIZE` (selfRightSpace).
#[allow(dead_code)]
const SELF_EDGE_SIZE: f64 = 16.0;

/// const.h:111-123 — sides of boxes.
const BOTTOM: i32 = 1;
const RIGHT: i32 = 2;
const TOP: i32 = 4;
const LEFT: i32 = 8;

// ---------------------------------------------------------------------------
// spline_info_t (dotsplines.c:64-72)
// ---------------------------------------------------------------------------

/// `spline_info_t` — the routing state shared by all groups.
struct SplineInfo {
    /// Graph x-extent, grown by `MINW` once per rank processed (cumulative —
    /// dotsplines.c:285-286 applies it inside the rank loop).
    left_bound: f64,
    right_bound: f64,
    /// `GD_nodesep(g) / 4` (dotsplines.c:274).
    splinesep: f64,
    /// `GD_nodesep(g)` (dotsplines.c:275).
    multisep: f64,
    /// One cached inter-rank band box per rank (`rank_box`, :2016-2028).
    rank_box: Vec<BoxF>,
}

impl Default for SplineInfo {
    fn default() -> Self {
        SplineInfo {
            left_bound: 0.0,
            right_bound: 0.0,
            splinesep: 0.0,
            multisep: 0.0,
            rank_box: Vec::new(),
        }
    }
}

/// `pathend_t` (types.h:73-79). C uses a fixed 20-slot `boxes` array with an
/// explicit `boxn`; the Vec here is always truncated to `boxn` semantics by
/// `beginpath`/`endpath` (which clear it before writing, exactly like the C
/// code's `endp->boxes[0] = …; endp->boxn = …`).
#[derive(Debug, Clone, Default)]
struct PathEnd {
    nb: BoxF,
    #[allow(dead_code)]
    np: PointF,
    sidemask: i32,
    boxes: Vec<BoxF>,
}

// ---------------------------------------------------------------------------
// small helpers (dotsplines.c:48-226)
// ---------------------------------------------------------------------------

/// dotsplines.c:48 `makefwdedge` — a reversed copy of `old`: tail/head and
/// ports swapped, `ED_edge_type = VIRTUAL`, `ED_to_orig = old`. C builds it
/// in a stack `Agedgepair_t`; here it is materialized in the arena (the
/// arena is not iterated during routing, so no observable difference).
fn makefwdedge(fg: &mut Fg, old: EId) -> EId {
    let mut e = fg.edges[old].clone();
    let (t, h) = (e.tail, e.head);
    let (tp, hp) = (e.tail_port, e.head_port);
    e.tail = h;
    e.head = t;
    e.tail_port = hp;
    e.head_port = tp;
    e.edge_type = EdgeType::Virtual;
    e.to_orig = Some(old);
    fg.edges.push(e);
    fg.edges.len() - 1
}

/// dotsplines.c:99 `getmainedge` — follow `ED_to_virt` to the innermost
/// virtual edge, then `ED_to_orig` to the canonical user edge.
fn getmainedge(fg: &Fg, e: EId) -> EId {
    let mut le = e;
    while let Some(v) = fg.edges[le].to_virt {
        le = v;
    }
    while let Some(o) = fg.edges[le].to_orig {
        le = o;
    }
    le
}

/// dotsplines.c:108 `spline_merge` — a virtual node where several chain
/// segments merge (only via `concentrate`).
fn spline_merge(fg: &Fg, n: NId) -> bool {
    fg.nodes[n].node_type == NodeType::Virtual
        && (fg.nodes[n].in_.len() > 1 || fg.nodes[n].out.len() > 1)
}

/// dotsplines.c:113 `swap_ends_p` — walk to the original edge; true iff the
/// head is drawn above/left of the tail (needs spline direction reversal).
fn swap_ends_p(fg: &Fg, e: EId) -> bool {
    let mut e = e;
    while let Some(o) = fg.edges[e].to_orig {
        e = o;
    }
    let (tr, hr) = (
        fg.nodes[fg.edges[e].tail].rank,
        fg.nodes[fg.edges[e].head].rank,
    );
    if hr > tr {
        return false;
    }
    if hr < tr {
        return true;
    }
    let (to, ho) = (
        fg.nodes[fg.edges[e].tail].order,
        fg.nodes[fg.edges[e].head].order,
    );
    ho < to
}

/// dotsplines.c:128 `portcmp` — three-way compare of two ports. Undefined
/// handling first, then `.p.x`, then `.p.y`. Does not compare theta/side/bp.
pub fn portcmp(p0: &Port, p1: &Port) -> i32 {
    if !p1.defined {
        return if p0.defined { 1 } else { 0 };
    }
    if !p0.defined {
        return -1;
    }
    if p0.p.x < p1.p.x {
        return -1;
    }
    if p0.p.x > p1.p.x {
        return 1;
    }
    if p0.p.y < p1.p.y {
        return -1;
    }
    if p0.p.y > p1.p.y {
        return 1;
    }
    0
}

/// The `(tail_port, head_port)` pair as seen after a `makefwdedge` reversal
/// (used wherever C compares ports of a possibly `BWDEDGE` edge).
fn ports_fwd(fg: &Fg, e: EId) -> (&Port, &Port) {
    let d = &fg.edges[e];
    if d.tree_index & BWDEDGE != 0 {
        (&d.head_port, &d.tail_port)
    } else {
        (&d.tail_port, &d.head_port)
    }
}

/// dotsplines.c:504 `setflags`.
fn setflags(fg: &mut Fg, e: EId, hint1: i32, hint2: i32, f3: i32) {
    let (t, h) = (fg.edges[e].tail, fg.edges[e].head);
    let f1 = if hint1 != 0 {
        hint1
    } else if t == h {
        if fg.edges[e].tail_port.defined || fg.edges[e].head_port.defined {
            SELFWPEDGE
        } else {
            SELFNPEDGE
        }
    } else if fg.nodes[t].rank == fg.nodes[h].rank {
        FLATEDGE
    } else {
        REGULAREDGE
    };
    let f2 = if hint2 != 0 {
        hint2
    } else if f1 == REGULAREDGE {
        if fg.nodes[t].rank < fg.nodes[h].rank {
            FWDEDGE
        } else {
            BWDEDGE
        }
    } else if f1 == FLATEDGE {
        if fg.nodes[t].order < fg.nodes[h].order {
            FWDEDGE
        } else {
            BWDEDGE
        }
    } else {
        // f1 == SELF*EDGE
        FWDEDGE
    };
    fg.edges[e].tree_index = f1 | f2 | f3;
}

/// dotsplines.c:542 `edgecmp` — total order of the edge list. NOTE the
/// *descending* edge-type comparison (et0 < et1 → Greater). The final
/// `AGSEQ` tie-break makes the order total, so Rust's stable `sort_by`
/// matches C's qsort.
fn edgecmp(fg: &Fg, e0: EId, e1: EId) -> Ordering {
    let et0 = fg.edges[e0].tree_index & EDGETYPEMASK;
    let et1 = fg.edges[e1].tree_index & EDGETYPEMASK;
    if et0 < et1 {
        return Ordering::Greater;
    }
    if et0 > et1 {
        return Ordering::Less;
    }

    let le0 = getmainedge(fg, e0);
    let le1 = getmainedge(fg, e1);

    // |rank difference| ascending
    let rd0 = (fg.nodes[fg.edges[le0].tail].rank - fg.nodes[fg.edges[le0].head].rank).abs();
    let rd1 = (fg.nodes[fg.edges[le1].tail].rank - fg.nodes[fg.edges[le1].head].rank).abs();
    if rd0 < rd1 {
        return Ordering::Less;
    }
    if rd0 > rd1 {
        return Ordering::Greater;
    }

    // |x difference| ascending
    let dx0 = (fg.nodes[fg.edges[le0].tail].coord.x - fg.nodes[fg.edges[le0].head].coord.x).abs();
    let dx1 = (fg.nodes[fg.edges[le1].tail].coord.x - fg.nodes[fg.edges[le1].head].coord.x).abs();
    if dx0 < dx1 {
        return Ordering::Less;
    }
    if dx0 > dx1 {
        return Ordering::Greater;
    }

    // AGSEQ of the main edge — the per-endpoint-pair witness id
    match fg.edges[le0].seq.cmp(&fg.edges[le1].seq) {
        Ordering::Equal => {}
        other => return other,
    }

    // ports (with makefwdedge reversal applied where BWDEDGE)
    let ea = if fg.edges[e0].tail_port.defined || fg.edges[e0].head_port.defined {
        e0
    } else {
        le0
    };
    let eb = if fg.edges[e1].tail_port.defined || fg.edges[e1].head_port.defined {
        e1
    } else {
        le1
    };
    let (ta, ha) = ports_fwd(fg, ea);
    let (tb, hb) = ports_fwd(fg, eb);
    let rv = portcmp(ta, tb);
    if rv != 0 {
        return ordering_of(rv);
    }
    let rv = portcmp(ha, hb);
    if rv != 0 {
        return ordering_of(rv);
    }

    // graph bits ascending (MAINGRAPH 64 before AUXGRAPH 128)
    let gt0 = fg.edges[e0].tree_index & GRAPHTYPEMASK;
    let gt1 = fg.edges[e1].tree_index & GRAPHTYPEMASK;
    if gt0 < gt1 {
        return Ordering::Less;
    }
    if gt0 > gt1 {
        return Ordering::Greater;
    }

    // for flat edges, compare the label records (C compares pointers; the
    // arena index has the same NULL-lowest ordering)
    if et0 == FLATEDGE {
        match fg.edges[e0].label.cmp(&fg.edges[e1].label) {
            Ordering::Equal => {}
            other => return other,
        }
    }

    fg.edges[e0].seq.cmp(&fg.edges[e1].seq)
}

fn ordering_of(rv: i32) -> Ordering {
    match rv {
        x if x < 0 => Ordering::Less,
        x if x > 0 => Ordering::Greater,
        _ => Ordering::Equal,
    }
}

/// dotsplines.c:144-180 — reverse the Bézier list and each Bézier's points
/// so control points run tail→head of the original edge. C also swaps
/// `sflag↔eflag` / `sp↔ep` inside each bezier; the arrow flags live on the
/// edge record here.
fn swap_spline(fg: &mut Fg, e: EId) {
    {
        let spl = match fg.edges[e].spl.as_mut() {
            Some(s) => s,
            None => return,
        };
        spl.list.reverse();
        for seg in spl.list.iter_mut() {
            seg.reverse();
        }
        // swap_bezier also swaps the arrow tip anchors and the per-bezier
        // arrow flags (`SWAP(&b->sp, &b->ep)`); the flags live on the edge
        // record here, the anchors on the Splines record.
        std::mem::swap(&mut spl.sp, &mut spl.ep);
    }
    let d = &mut fg.edges[e];
    std::mem::swap(&mut d.sflag, &mut d.eflag);
}

/// dotsplines.c:155 `edge_normalize` — for every original edge in
/// `agfstnode`/`agfstout` order, reverse swapped splines.
fn edge_normalize(fg: &mut Fg, g: GId) {
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        let outs = fg.input_out.get(n).cloned().unwrap_or_default();
        for e in outs {
            if swap_ends_p(fg, e) && fg.edges[e].spl.is_some() {
                swap_spline(fg, e);
            }
        }
    }
}

/// dotsplines.c:187 `resetRW` — restore the pre-inflation `rw` (self loops
/// inflate `rw` during position with the original kept in `mval`).
fn reset_rw(fg: &mut Fg, g: GId) {
    for &n in fg.graphs[g].nodes_order.clone().iter() {
        if !fg.nodes[n].other.is_empty() {
            let m = fg.nodes[n].mval;
            fg.nodes[n].mval = fg.nodes[n].rw;
            fg.nodes[n].rw = m;
        }
    }
}

/// The `ND_alg(n)` test (flat-edge label vnode, flat.c:181). The model has
/// no `ND_alg` slot; `flat_node` builds the vnode with two FLATORDER out
/// edges whose `ED_to_orig` is the flat edge, so reconstruct it from that.
fn nd_alg(fg: &Fg, n: NId) -> Option<EId> {
    if fg.nodes[n].node_type != NodeType::Virtual {
        return None;
    }
    let e = *fg.nodes[n].out.first()?;
    if fg.edges[e].edge_type == EdgeType::FlatOrder {
        fg.edges[e].to_orig
    } else {
        None
    }
}

/// dotsplines.c:491 `place_vnlabel` — place an edge label from its virtual
/// node (regular edges only: flat label vnodes have empty `ND_in`).
fn place_vnlabel(fg: &mut Fg, g: GId, n: NId) {
    if fg.nodes[n].in_.is_empty() {
        return; // skip flat edge labels here
    }
    // find the original edge by following ED_to_orig until NORMAL
    let mut e = match fg.nodes[n].out.first() {
        Some(&e) => e,
        None => return,
    };
    loop {
        if fg.edges[e].edge_type == EdgeType::Normal {
            break;
        }
        match fg.edges[e].to_orig {
            Some(o) => e = o,
            None => return,
        }
    }
    let li = match fg.edges[e].label {
        Some(li) => li,
        None => return,
    };
    let dimen = fg.labels[li].dimen;
    let width = if fg.graphs[g].rankdir.flip() {
        dimen.y
    } else {
        dimen.x
    };
    // the vnode's center is the LEFT edge of the text (position widened it
    // leftwards), hence +width/2
    let pos = PointF::new(fg.nodes[n].coord.x + width / 2.0, fg.nodes[n].coord.y);
    fg.labels[li].pos = pos;
}

/// common/utils.c:613 `updateBB` / `addLabelBB` — grow the graph bbox by a
/// placed label.
fn update_bb_label(fg: &mut Fg, g: GId, li: usize) {
    let flip = fg.graphs[g].rankdir.flip();
    let (width, height) = {
        let l = &fg.labels[li];
        if flip {
            (l.dimen.y, l.dimen.x)
        } else {
            (l.dimen.x, l.dimen.y)
        }
    };
    let p = fg.labels[li].pos;
    let mut bb = fg.graphs[g].bb;
    bb.ll.x = bb.ll.x.min(p.x - width / 2.0);
    bb.ur.x = bb.ur.x.max(p.x + width / 2.0);
    bb.ll.y = bb.ll.y.min(p.y - height / 2.0);
    bb.ur.y = bb.ur.y.max(p.y + height / 2.0);
    fg.graphs[g].bb = bb;
}

/// splines.c:316 `conc_slope` — average slope angle at a concentrate-merge
/// node.
fn conc_slope(fg: &Fg, n: NId) -> f64 {
    let mut s_in = 0.0;
    let mut cnt_in = 0usize;
    for &e in &fg.nodes[n].in_ {
        s_in += fg.nodes[fg.edges[e].tail].coord.x;
        cnt_in += 1;
    }
    let mut s_out = 0.0;
    let mut cnt_out = 0usize;
    for &e in &fg.nodes[n].out {
        s_out += fg.nodes[fg.edges[e].head].coord.x;
        cnt_out += 1;
    }
    if cnt_in == 0 || cnt_out == 0 {
        // merge nodes always have both; C would divide by zero here
        return 0.0;
    }
    let x1 = fg.nodes[n].coord.x - s_in / cnt_in as f64;
    let y1 = fg.nodes[n].coord.y - fg.nodes[fg.edges[fg.nodes[n].in_[0]].tail].coord.y;
    let m_in = y1.atan2(x1);
    let x2 = s_out / cnt_out as f64 - fg.nodes[n].coord.x;
    let y2 = fg.nodes[fg.edges[fg.nodes[n].out[0]].head].coord.y - fg.nodes[n].coord.y;
    let m_out = y2.atan2(x2);
    (m_in + m_out) / 2.0
}

// ---------------------------------------------------------------------------
// beginpath / endpath (common/splines.c:376-769) — the REGULAREDGE and
// FLATEDGE cases used by dotsplines. The `pboxfn` hook is the shape's
// `poly_path`, which returns 0 for every polygon-family shape (shapes.c:2537),
// so the fallback branch is always taken; record_path (records) is not
// modeled.
// ---------------------------------------------------------------------------

/// `for (orig = e; ED_to_orig(orig) != NULL && ED_edge_type(orig) != NORMAL;
///  orig = ED_to_orig(orig));`
fn walk_to_orig_stop_normal(fg: &Fg, e: EId) -> EId {
    let mut e = e;
    loop {
        if fg.edges[e].edge_type == EdgeType::Normal {
            return e;
        }
        match fg.edges[e].to_orig {
            Some(o) => e = o,
            None => return e,
        }
    }
}

fn set_orig_port_clip(fg: &mut Fg, e: EId, n: NId, tail_side: bool) {
    let orig = walk_to_orig_stop_normal(fg, e);
    if tail_side {
        if fg.edges[orig].tail == n {
            fg.edges[orig].tail_port.clip = false;
        } else {
            fg.edges[orig].head_port.clip = false;
        }
    } else if fg.edges[orig].head == n {
        fg.edges[orig].head_port.clip = false;
    } else {
        fg.edges[orig].tail_port.clip = false;
    }
}

/// `resolvePort` for edge `e`'s port at node `n`, using the node's shape and
/// the other endpoint's position (`GD_tail_port`/`GD_head_port` are assigned
/// in `beginpath`/`endpath`).
fn resolve_port_of(
    fg: &mut Fg,
    e: EId,
    n: NId,
    other: NId,
    tail_side: bool,
) -> Option<model::Port> {
    let old = if tail_side {
        fg.edges[e].tail_port
    } else {
        fg.edges[e].head_port
    };
    let node = &fg.nodes[n];
    let desc = crate::dotgen::shapes::shape_of(if node.shape_info.kind.is_empty() {
        "ellipse"
    } else {
        &node.shape_info.kind
    });
    let (lw, rw, ht) = (node.lw, node.rw, node.ht);
    let info = node.shape_info.clone();
    let rd = fg.graphs[fg.root_g()].rankdir;
    Some(crate::dotgen::shapes::resolve_port(
        &desc,
        lw,
        rw,
        ht,
        Some(&info),
        &old,
        rd.flip(),
        fg.nodes[n].coord,
        fg.nodes[other].coord,
        rd,
    ))
}

/// splines.c:376 `beginpath` — set up boxes near the tail node. Sets
/// `P.start.p/theta`, clears `P.boxes` (`P->nbox = 0`) and fills
/// `endp->boxes`.
fn beginpath(
    fg: &mut Fg,
    g: GId,
    path: &mut Path,
    e: EId,
    et: i32,
    endp: &mut PathEnd,
    merge: bool,
) {
    // splines.c:382-383 — a `_` port picks its side now that the other
    // endpoint's position is known.
    if fg.edges[e].tail_port.dyna {
        let n = fg.edges[e].tail;
        let other = fg.edges[e].head;
        if let Some(resolved) = resolve_port_of(fg, e, n, other, true) {
            fg.edges[e].tail_port = resolved;
        }
    }
    let n = fg.edges[e].tail;
    let port = fg.edges[e].tail_port;

    path.start.p = fg.nodes[n].coord.add(port.p);
    if merge {
        path.start.theta = conc_slope(fg, n);
        path.start.constrained = true;
    } else if port.constrained {
        path.start.theta = port.theta;
        path.start.constrained = true;
    } else {
        path.start.constrained = false;
    }
    path.boxes.clear(); // P->nbox = 0
    path.data = e;
    endp.np = path.start.p;

    let side = port.side as i32;
    if et == REGULAREDGE && fg.nodes[n].node_type == NodeType::Normal && side != 0 {
        let mut b = endp.nb;
        let coord = fg.nodes[n].coord;
        let ht2 = fg.nodes[n].ht / 2.0;
        let ranksep2 = fg.graphs[g].ranksep as f64 / 2.0;
        let mut b0 = BoxF::default();
        if side & TOP != 0 {
            endp.sidemask = TOP;
            if path.start.p.x < coord.x {
                // go left
                b0.ll.x = b.ll.x - 1.0;
                b0.ll.y = path.start.p.y;
                b0.ur.x = b.ur.x;
                b0.ur.y = coord.y + ht2 + ranksep2;
                b.ur.x = coord.x - fg.nodes[n].lw - (BEGIN_FUDGE - 2.0);
                b.ur.y = b0.ll.y;
                b.ll.y = coord.y - ht2;
                b.ll.x -= 1.0;
            } else {
                b0.ll.x = b.ll.x;
                b0.ll.y = path.start.p.y;
                b0.ur.x = b.ur.x + 1.0;
                b0.ur.y = coord.y + ht2 + ranksep2;
                b.ll.x = coord.x + fg.nodes[n].rw + (BEGIN_FUDGE - 2.0);
                b.ur.y = b0.ll.y;
                b.ll.y = coord.y - ht2;
                b.ur.x += 1.0;
            }
            path.start.p.y += 1.0;
            endp.boxes.clear();
            endp.boxes.push(b0);
            endp.boxes.push(b);
        } else if side & BOTTOM != 0 {
            endp.sidemask = BOTTOM;
            b.ur.y = b.ur.y.max(path.start.p.y);
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.y -= 1.0;
        } else if side & LEFT != 0 {
            endp.sidemask = LEFT;
            b.ur.x = path.start.p.x;
            b.ll.y = coord.y - ht2;
            b.ur.y = path.start.p.y;
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.x -= 1.0;
        } else {
            endp.sidemask = RIGHT;
            b.ll.x = path.start.p.x;
            b.ll.y = coord.y - ht2;
            b.ur.y = path.start.p.y;
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.x += 1.0;
        }
        set_orig_port_clip(fg, e, n, true);
        return;
    }

    if et == FLATEDGE && side != 0 {
        let mut b = endp.nb;
        let coord = fg.nodes[n].coord;
        let ht2 = fg.nodes[n].ht / 2.0;
        let ranksep2 = fg.graphs[g].ranksep as f64 / 2.0;
        let mut b0 = BoxF::default();
        if side & TOP != 0 {
            b.ll.y = b.ll.y.min(path.start.p.y);
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.y += 1.0;
        } else if side & BOTTOM != 0 {
            if endp.sidemask == TOP {
                b0.ur.y = coord.y - ht2;
                b0.ur.x = b.ur.x + 1.0;
                b0.ll.x = path.start.p.x;
                b0.ll.y = b0.ur.y - ranksep2;
                b.ll.x = coord.x + fg.nodes[n].rw + (BEGIN_FUDGE - 2.0);
                b.ll.y = b0.ur.y;
                b.ur.y = coord.y + ht2;
                b.ur.x += 1.0;
                endp.boxes.clear();
                endp.boxes.push(b0);
                endp.boxes.push(b);
            } else {
                b.ur.y = b.ur.y.max(path.start.p.y);
                endp.boxes.clear();
                endp.boxes.push(b);
            }
            path.start.p.y -= 1.0;
        } else if side & LEFT != 0 {
            b.ur.x = path.start.p.x + 1.0;
            if endp.sidemask == TOP {
                b.ur.y = coord.y + ht2;
                b.ll.y = path.start.p.y - 1.0;
            } else {
                b.ll.y = coord.y - ht2;
                b.ur.y = path.start.p.y + 1.0;
            }
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.x -= 1.0;
        } else {
            b.ll.x = path.start.p.x;
            if endp.sidemask == TOP {
                b.ur.y = coord.y + ht2;
                b.ll.y = path.start.p.y;
            } else {
                b.ll.y = coord.y - ht2;
                b.ur.y = path.start.p.y + 1.0;
            }
            endp.boxes.clear();
            endp.boxes.push(b);
            path.start.p.x += 1.0;
        }
        set_orig_port_clip(fg, e, n, true);
        endp.sidemask = side;
        return;
    }

    // fallback (pboxfn returns 0 for all modeled shapes)
    let mut b0 = endp.nb;
    endp.boxes.clear();
    match et {
        REGULAREDGE => {
            b0.ur.y = path.start.p.y;
            endp.sidemask = BOTTOM;
            path.start.p.y -= 1.0;
        }
        FLATEDGE => {
            if endp.sidemask == TOP {
                b0.ll.y = path.start.p.y;
            } else {
                b0.ur.y = path.start.p.y;
            }
        }
        // SELFEDGE asserts(0) in C ("at present, we don't use beginpath for
        // selfedges"); dotsplines never reaches it.
        _ => {}
    }
    endp.boxes.push(b0);
}

/// splines.c:573 `endpath` — the head-side mirror. Does not touch
/// `P->nbox`/`P->data`.
#[allow(clippy::too_many_arguments)]
fn endpath(fg: &mut Fg, g: GId, path: &mut Path, e: EId, et: i32, endp: &mut PathEnd, merge: bool) {
    // splines.c:584-585 — the head-side `resolvePort`.
    if fg.edges[e].head_port.dyna {
        let n = fg.edges[e].head;
        let other = fg.edges[e].tail;
        if let Some(resolved) = resolve_port_of(fg, e, n, other, false) {
            fg.edges[e].head_port = resolved;
        }
    }
    let n = fg.edges[e].head;
    let port = fg.edges[e].head_port;

    path.end.p = fg.nodes[n].coord.add(port.p);
    if merge {
        path.end.theta = conc_slope(fg, n) + std::f64::consts::PI;
        path.end.constrained = true;
    } else if port.constrained {
        path.end.theta = port.theta;
        path.end.constrained = true;
    } else {
        path.end.constrained = false;
    }
    endp.np = path.end.p;

    let side = port.side as i32;
    if et == REGULAREDGE && fg.nodes[n].node_type == NodeType::Normal && side != 0 {
        let mut b = endp.nb;
        let coord = fg.nodes[n].coord;
        let ht2 = fg.nodes[n].ht / 2.0;
        let ranksep2 = fg.graphs[g].ranksep as f64 / 2.0;
        let mut b0 = BoxF::default();
        if side & TOP != 0 {
            endp.sidemask = TOP;
            b.ll.y = b.ll.y.min(path.end.p.y);
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.y += 1.0;
        } else if side & BOTTOM != 0 {
            endp.sidemask = BOTTOM;
            if path.end.p.x < coord.x {
                // go left
                b0.ll.x = b.ll.x - 1.0;
                b0.ur.y = path.end.p.y;
                b0.ur.x = b.ur.x;
                b0.ll.y = coord.y - ht2 - ranksep2;
                b.ur.x = coord.x - fg.nodes[n].lw - (BEGIN_FUDGE - 2.0);
                b.ll.y = b0.ur.y;
                b.ur.y = coord.y + ht2;
                b.ll.x -= 1.0;
            } else {
                b0.ll.x = b.ll.x;
                b0.ur.y = path.end.p.y;
                b0.ur.x = b.ur.x + 1.0;
                b0.ll.y = coord.y - ht2 - ranksep2;
                b.ll.x = coord.x + fg.nodes[n].rw + (BEGIN_FUDGE - 2.0);
                b.ll.y = b0.ur.y;
                b.ur.y = coord.y + ht2;
                b.ur.x += 1.0;
            }
            endp.boxes.clear();
            endp.boxes.push(b0);
            endp.boxes.push(b);
            path.end.p.y -= 1.0;
        } else if side & LEFT != 0 {
            endp.sidemask = LEFT;
            b.ur.x = path.end.p.x;
            b.ur.y = coord.y + ht2;
            b.ll.y = path.end.p.y;
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.x -= 1.0;
        } else {
            endp.sidemask = RIGHT;
            b.ll.x = path.end.p.x;
            b.ur.y = coord.y + ht2;
            b.ll.y = path.end.p.y;
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.x += 1.0;
        }
        set_orig_port_clip(fg, e, n, false);
        endp.sidemask = side;
        return;
    }

    if et == FLATEDGE && side != 0 {
        let mut b = endp.nb;
        let coord = fg.nodes[n].coord;
        let ht2 = fg.nodes[n].ht / 2.0;
        let ranksep2 = fg.graphs[g].ranksep as f64 / 2.0;
        let mut b0 = BoxF::default();
        if side & TOP != 0 {
            b.ll.y = b.ll.y.min(path.end.p.y);
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.y += 1.0;
        } else if side & BOTTOM != 0 {
            if endp.sidemask == TOP {
                b0.ll.x = b.ll.x - 1.0;
                b0.ur.y = coord.y - ht2;
                b0.ur.x = path.end.p.x;
                b0.ll.y = b0.ur.y - ranksep2;
                b.ur.x = coord.x - fg.nodes[n].lw - 2.0;
                b.ll.y = b0.ur.y;
                b.ur.y = coord.y + ht2;
                b.ll.x -= 1.0;
                endp.boxes.clear();
                endp.boxes.push(b0);
                endp.boxes.push(b);
            } else {
                b.ur.y = b.ur.y.max(path.start.p.y);
                endp.boxes.clear();
                endp.boxes.push(b);
            }
            path.end.p.y -= 1.0;
        } else if side & LEFT != 0 {
            b.ur.x = path.end.p.x + 1.0;
            if endp.sidemask == TOP {
                b.ur.y = coord.y + ht2;
                b.ll.y = path.end.p.y - 1.0;
            } else {
                b.ll.y = coord.y - ht2;
                b.ur.y = path.end.p.y + 1.0;
            }
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.x -= 1.0;
        } else {
            b.ll.x = path.end.p.x - 1.0;
            if endp.sidemask == TOP {
                b.ur.y = coord.y + ht2;
                b.ll.y = path.end.p.y - 1.0;
            } else {
                b.ll.y = coord.y - ht2;
                b.ur.y = path.end.p.y;
            }
            endp.boxes.clear();
            endp.boxes.push(b);
            path.end.p.x += 1.0;
        }
        set_orig_port_clip(fg, e, n, false);
        endp.sidemask = side;
        return;
    }

    // fallback
    let mut b0 = endp.nb;
    endp.boxes.clear();
    match et {
        REGULAREDGE => {
            b0.ll.y = path.end.p.y;
            endp.sidemask = TOP;
            path.end.p.y += 1.0;
        }
        FLATEDGE => {
            if endp.sidemask == TOP {
                b0.ll.y = path.end.p.y;
            } else {
                b0.ur.y = path.end.p.y;
            }
        }
        _ => {}
    }
    endp.boxes.push(b0);
}

/// splines.c:336 `add_box` — silently drops empty boxes.
fn add_box(path: &mut Path, b: BoxF) {
    if b.ll.x < b.ur.x && b.ll.y < b.ur.y {
        path.boxes.push(b);
    }
}

/// dotsplines.c:1959 `makeregularend` — a box filling between the node and
/// the inter-rank space (nodes in a rank can differ in height). Regular
/// edges always go top-to-bottom here.
fn makeregularend(b: BoxF, side: i32, y: f64) -> BoxF {
    if side == BOTTOM {
        BoxF {
            ll: PointF::new(b.ll.x, y),
            ur: PointF::new(b.ur.x, b.ll.y),
        }
    } else {
        BoxF {
            ll: PointF::new(b.ll.x, b.ur.y),
            ur: PointF::new(b.ur.x, y),
        }
    }
}

fn box_nonempty(b: BoxF) -> bool {
    b.ll.x < b.ur.x && b.ll.y < b.ur.y
}

// rank helpers ------------------------------------------------------------

fn rank_pair(fg: &Fg, g: GId, r: i32) -> (f64, f64) {
    match fg.graphs[g].rank.get(r.max(0) as usize) {
        Some(rank) => (rank.ht1, rank.ht2),
        None => (0.0, 0.0),
    }
}

fn rank_v0_y(fg: &Fg, g: GId, r: i32) -> Option<f64> {
    let rank = fg.graphs[g].rank.get(r.max(0) as usize)?;
    let n = *rank.v.first()?;
    Some(fg.nodes[n].coord.y)
}

// ---------------------------------------------------------------------------
// The driver (dotsplines.c:228-486)
// ---------------------------------------------------------------------------

/// `EDGE_TYPE(g)` (macros.h:25). The `splines` graph attribute is not
/// carried into `DGraph` yet; dot's default (`splines=true`) is
/// `EDGETYPE_SPLINE`.
fn edge_type_of(_fg: &Fg, _g: GId) -> SplineType {
    SplineType::Spline
}

/// dotsplines.c:486 `dot_splines` — set edge splines. Returns 0 on success
/// (mirroring `dot_splines_(g, 1)`).
pub fn dot_splines(fg: &mut Fg, g: GId) -> Result<(), i32> {
    dot_splines_(fg, g, true)
}

/// dotsplines.c:228 `dot_splines_`. `normalize` is false only for the
/// recursive call from `make_flat_adj_edges` (not ported — see module doc).
fn dot_splines_(fg: &mut Fg, g: GId, normalize: bool) -> Result<(), i32> {
    // The routing flags live in `tree_index` (a stand-in for C's
    // `ED_edge_type`, which cgraph zero-initializes). The rank phase's
    // network simplex left every edge at -1, whose all-ones bit pattern would
    // read as BWDEDGE/MAINGRAPH for *unclassified* edges — clear it first,
    // exactly as a fresh cgraph edge would be.
    for e in 0..fg.edges.len() {
        fg.edges[e].tree_index = 0;
    }
    let et = edge_type_of(fg, g);
    if et == SplineType::None {
        return Ok(()); // "splines=" empty ⇒ no routing at all (:239-240)
    }
    if et == SplineType::Curved {
        reset_rw(fg, g);
        // C warns "edge labels with splines=curved not supported in dot -
        // use xlabels"; warnings are not modeled here.
    }
    // ORTHO (dotsplines.c:250-266): lib/ortho is not ported. When C is
    // compiled without #ifdef ORTHO it falls through here and routes with
    // the polyline machinery — reproduced below via `is_spline`/`polyline`.
    // mark_lowclusters(g) (cluster.c): no clusters modeled in this milestone.
    // routesplinesinit() (routespl.c): stateless in this port.

    let mut sp = SplineInfo {
        splinesep: fg.graphs[g].nodesep as f64 / 4.0,
        multisep: fg.graphs[g].nodesep as f64,
        ..Default::default()
    };
    let mut edges: Vec<EId> = Vec::new();
    let mut path = Path::default();

    // ---- pass 1: compute boundaries, classify and collect edges (:277-327)
    let (ti_pass1, t_pass1) = (
        std::env::var_os("GD_TIMING").is_some(),
        std::time::Instant::now(),
    );
    let minrank = fg.graphs[g].minrank;
    let maxrank = fg.graphs[g].maxrank;
    for r in minrank..=maxrank {
        if let Some(rank) = fg.graphs[g].rank.get(r as usize) {
            let n = rank.n;
            if n > 0 {
                let first = rank.v[0];
                let last = rank.v[n - 1];
                sp.left_bound = sp
                    .left_bound
                    .min(fg.nodes[first].coord.x - fg.nodes[first].lw);
                sp.right_bound = sp
                    .right_bound
                    .max(fg.nodes[last].coord.x + fg.nodes[last].rw);
            }
        }
        // NOTE: applied every rank iteration (cumulative padding) — :285-286
        sp.left_bound -= MINW;
        sp.right_bound += MINW;

        let members: Vec<NId> = match fg.graphs[g].rank.get(r as usize) {
            Some(rank) => rank.v.iter().take(rank.n).copied().collect(),
            None => continue,
        };
        for nid in members {
            // if nid is the label vnode of a flat edge, copy its position
            if let Some(fe) = nd_alg(fg, nid) {
                if let Some(li) = fg.edges[fe].label {
                    let pos = fg.nodes[nid].coord;
                    fg.labels[li].pos = pos;
                }
            }
            if fg.nodes[nid].node_type != NodeType::Normal && !spline_merge(fg, nid) {
                continue;
            }
            for e in fg.nodes[nid].out.clone() {
                if fg.edges[e].edge_type == EdgeType::FlatOrder
                    || fg.edges[e].edge_type == EdgeType::Ignored
                {
                    continue;
                }
                setflags(fg, e, REGULAREDGE, FWDEDGE, MAINGRAPH);
                edges.push(e);
            }
            for e in fg.nodes[nid].flat_out.clone() {
                setflags(fg, e, FLATEDGE, 0, AUXGRAPH);
                edges.push(e);
            }
            if !fg.nodes[nid].other.is_empty() {
                // restore the pre-inflation rw (position.c saved it in mval)
                if fg.nodes[nid].node_type == NodeType::Normal {
                    let m = fg.nodes[nid].mval;
                    fg.nodes[nid].mval = fg.nodes[nid].rw;
                    fg.nodes[nid].rw = m;
                }
                for e in fg.nodes[nid].other.clone() {
                    setflags(fg, e, 0, 0, AUXGRAPH);
                    edges.push(e);
                }
            }
        }
    }

    // sort so equivalent edges are contiguous (:335)
    edges.sort_by(|&a, &b| edgecmp(fg, a, b));
    if ti_pass1 {
        eprintln!(
            "[timing]   splines::classify+sort: {:.3}s",
            t_pass1.elapsed().as_secs_f64()
        );
    }

    sp.rank_box = vec![BoxF::default(); (maxrank + 1).max(0) as usize];

    if et == SplineType::Line {
        // place regular edge labels (:341-348)
        let mut next = fg.graphs[g].nlist;
        while let Some(n) = next {
            next = fg.nodes[n].next;
            if fg.nodes[n].node_type == NodeType::Virtual && fg.nodes[n].label.is_some() {
                place_vnlabel(fg, g, n);
            }
        }
    }

    // ---- pass 2: route one equivalence group at a time (:350-427)
    let (ti_spl, ts_route) = (
        std::env::var_os("GD_TIMING").is_some(),
        std::time::Instant::now(),
    );
    let (mut n_self, mut n_flat, mut n_reg, mut n_curved) = (0usize, 0usize, 0usize, 0usize);
    let mut l = 0usize;
    while l < edges.len() {
        let ind = l;
        let e0 = edges[l];
        l += 1;
        let le0 = getmainedge(fg, e0);
        let ea = if fg.edges[e0].tail_port.defined || fg.edges[e0].head_port.defined {
            e0
        } else {
            le0
        };
        let mut cnt = 1usize;
        while l < edges.len() {
            let e1 = edges[l];
            let le1 = getmainedge(fg, e1);
            if le1 != le0 {
                break;
            }
            if fg.edges[e0].adjacent {
                // all flat adjacent edges at once (:367)
                cnt += 1;
                l += 1;
                continue;
            }
            let eb = if fg.edges[e1].tail_port.defined || fg.edges[e1].head_port.defined {
                e1
            } else {
                le1
            };
            let (ta, ha) = ports_fwd(fg, ea);
            let (tb, hb) = ports_fwd(fg, eb);
            if portcmp(ta, tb) != 0 {
                break;
            }
            if portcmp(ha, hb) != 0 {
                break;
            }
            if (fg.edges[e0].tree_index & EDGETYPEMASK) == FLATEDGE
                && fg.edges[e0].label != fg.edges[e1].label
            {
                break;
            }
            if fg.edges[e1].tree_index & MAINGRAPH != 0 {
                break; // "Aha! -C is on"
            }
            cnt += 1;
            l += 1;
        }

        let group: Vec<EId> = edges[ind..ind + cnt].to_vec();
        if et == SplineType::Curved {
            n_curved += 1;
            let mut edgelist = vec![getmainedge(fg, group[0])];
            edgelist.extend_from_slice(&group[1..]);
            make_straight_edges(fg, g, &edgelist, et);
        } else if fg.edges[group[0]].tail == fg.edges[group[0]].head {
            n_self += 1;
            // self loop (:395-416)
            let n = fg.edges[group[0]].tail;
            let r = fg.nodes[n].rank;
            let sizey: f64 = if r == maxrank {
                if r > 0 {
                    rank_v0_y(fg, g, r - 1).unwrap_or(fg.nodes[n].ht) - fg.nodes[n].coord.y
                } else {
                    fg.nodes[n].ht
                }
            } else if r == minrank {
                fg.nodes[n].coord.y - rank_v0_y(fg, g, r + 1).unwrap_or(fg.nodes[n].coord.y)
            } else {
                let upy =
                    rank_v0_y(fg, g, r - 1).unwrap_or(fg.nodes[n].coord.y) - fg.nodes[n].coord.y;
                let dwny =
                    fg.nodes[n].coord.y - rank_v0_y(fg, g, r + 1).unwrap_or(fg.nodes[n].coord.y);
                upy.min(dwny)
            };
            make_self_edge(fg, g, &group, sp.multisep, sizey / 2.0);
            for &e in &group {
                if let Some(li) = fg.edges[e].label {
                    update_bb_label(fg, g, li);
                }
            }
        } else if fg.nodes[fg.edges[group[0]].tail].rank == fg.nodes[fg.edges[group[0]].head].rank {
            n_flat += 1;
            make_flat_edge(fg, g, &mut sp, &mut path, &group, et)?;
        } else {
            n_reg += 1;
            make_regular_edge(fg, g, &mut sp, &mut path, &group, et);
        }
    }
    if ti_spl {
        eprintln!(
            "[timing]   splines::route: {:.3}s ({n_self} self, {n_flat} flat, {n_reg} regular, {n_curved} straight groups)",
            ts_route.elapsed().as_secs_f64()
        );
    }

    // place regular edge labels (:429-435)
    let mut next = fg.graphs[g].nlist;
    while let Some(n) = next {
        next = fg.nodes[n].next;
        if fg.nodes[n].node_type == NodeType::Virtual && fg.nodes[n].label.is_some() {
            place_vnlabel(fg, g, n);
            if let Some(li) = fg.nodes[n].label {
                update_bb_label(fg, g, li);
            }
        }
    }

    // normalize splines so they always go from tail to head (:439-440)
    if normalize {
        edge_normalize(fg, g);
    }

    // head/tail port labels (:447-465): the labelangle/labeldistance globals
    // are not modeled; makePortLabels is a no-op without them anyway.
    // routesplinesterm(): no-op. State = GVSPLINES: not modeled.
    Ok(())
}

// ---------------------------------------------------------------------------
// regular (inter-rank) edges (dotsplines.c:1707-2330)
// ---------------------------------------------------------------------------

/// dotsplines.c:1707 `make_regular_edge` — the box-channel router.
fn make_regular_edge(
    fg: &mut Fg,
    g: GId,
    sp: &mut SplineInfo,
    path: &mut Path,
    edges: &[EId],
    et: SplineType,
) {
    let mut pointfs: Vec<PointF> = Vec::new();
    let e_raw = edges[0];
    let has_labels = fg.graphs[0].has_labels & 1 != 0;
    let is_spline = et == SplineType::Spline;

    // ---- step 1: the cross-rank "hack" edge (:1723-1760)
    let rank_span =
        (fg.nodes[fg.edges[e_raw].tail].rank - fg.nodes[fg.edges[e_raw].head].rank).abs();
    let (mut e, fe, fwd_b, hackflag);
    if rank_span > 1 {
        let bw = fg.edges[e_raw].tree_index & BWDEDGE != 0;
        // innermost virtual edge of the main edge
        let le_inner = {
            let mut l = getmainedge(fg, e_raw);
            while let Some(v) = fg.edges[l].to_virt {
                l = v;
            }
            l
        };
        // fwdedgea.out: a copy of e re-aimed at the LAST vnode/real head of
        // the chain, with no head port
        let fwd_a = {
            let mut ve = fg.edges[e_raw].clone();
            if bw {
                ve.tail = fg.edges[e_raw].head;
                ve.tail_port = fg.edges[e_raw].head_port;
            }
            ve.head = fg.edges[le_inner].head;
            // dotsplines.c:1748-1750 — only `defined` and `p` are cleared;
            // clip/side/theta keep their copied values.
            ve.head_port.defined = false;
            ve.head_port.p = PointF::ZERO;
            ve.edge_type = EdgeType::Virtual;
            ve.to_orig = Some(e_raw);
            fg.edges.push(ve);
            fg.edges.len() - 1
        };
        // fwdedgeb.out keeps the final head-end info for endpath
        fwd_b = if bw {
            makefwdedge(fg, e_raw)
        } else {
            let ve = fg.edges[e_raw].clone();
            fg.edges.push(ve);
            fg.edges.len() - 1
        };
        e = fwd_a;
        hackflag = true;
    } else if fg.edges[e_raw].tree_index & BWDEDGE != 0 {
        e = makefwdedge(fg, e_raw);
        fwd_b = e;
        hackflag = false;
    } else {
        e = e_raw;
        fwd_b = e_raw;
        hackflag = false;
    }
    fe = e;

    // ---- step 2: build the box path and route (:1762-1881)
    let hn_final;
    let line_hn = if et == SplineType::Line {
        make_line_edge(fg, g, fe, &mut pointfs)
    } else {
        None
    };
    if let Some(h) = line_hn {
        hn_final = h;
    } else {
        let mut boxes: Vec<BoxF> = Vec::new();
        let mut segfirst = e;
        let mut tn = fg.edges[e].tail;
        let mut hn = fg.edges[e].head;
        let mut tend = PathEnd::default();
        let mut hend;
        let mut b = {
            tend.nb = maximal_bbox(fg, g, sp, tn, None, Some(e));
            tend.nb
        };
        beginpath(fg, g, path, e, REGULAREDGE, &mut tend, spline_merge(fg, tn));
        b.ur.y = tend.boxes.last().unwrap().ur.y;
        b.ll.y = tend.boxes.last().unwrap().ll.y;
        let (ht1, _) = rank_pair(fg, g, fg.nodes[tn].rank);
        let bb = makeregularend(b, BOTTOM, fg.nodes[tn].coord.y - ht1);
        if box_nonempty(bb) {
            tend.boxes.push(bb);
        }
        let mut smode = false;
        let mut si = false;
        let mut sl: i32 = 0;
        while fg.nodes[hn].node_type == NodeType::Virtual && !spline_merge(fg, hn) {
            boxes.push(rank_box(fg, g, sp, fg.nodes[tn].rank));
            if !smode {
                sl = straight_len(fg, hn);
                let threshold = if has_labels { 4 + 1 } else { 2 + 1 };
                if sl >= threshold {
                    smode = true;
                    si = true;
                    sl -= 2;
                }
            }
            if !smode || si {
                si = false;
                let out0 = fg.nodes[hn].out[0];
                boxes.push(maximal_bbox(fg, g, sp, hn, Some(e), Some(out0)));
                e = out0;
                tn = fg.edges[e].tail;
                hn = fg.edges[e].head;
                continue;
            }
            // terminate this segment at hn, route it, then skip the
            // straight vertical run
            hend = PathEnd::default();
            let out0 = fg.nodes[hn].out[0];
            hend.nb = maximal_bbox(fg, g, sp, hn, Some(e), Some(out0));
            endpath(
                fg,
                g,
                path,
                e,
                REGULAREDGE,
                &mut hend,
                spline_merge(fg, fg.edges[e].head),
            );
            let (_, ht2h) = rank_pair(fg, g, fg.nodes[hn].rank);
            let hb = makeregularend(
                *hend.boxes.last().unwrap(),
                TOP,
                fg.nodes[hn].coord.y + ht2h,
            );
            if box_nonempty(hb) {
                hend.boxes.push(hb);
            }
            path.end.theta = std::f64::consts::FRAC_PI_2;
            path.end.constrained = true; // forced vertical arrival (:1802)
            completeregularpath(fg, g, path, segfirst, e, &tend, &hend, &mut boxes);
            let mut ps = if is_spline {
                routesplines_(path, false)
            } else {
                let mut r = routesplines_(path, true);
                if let Some(pts) = r.as_mut() {
                    if et == SplineType::Line && pts.len() > 4 {
                        straighten_line(pts);
                    }
                }
                r
            };
            match ps {
                Some(ref pts) if pts.is_empty() => return,
                None => return,
                Some(ref mut pts) => pointfs.append(pts),
            }
            e = straight_path(fg, fg.nodes[hn].out[0], sl, &mut pointfs);
            recover_slack(fg, segfirst, path);
            segfirst = e;
            tn = fg.edges[e].tail;
            hn = fg.edges[e].head;
            boxes.clear();
            tend = PathEnd::default();
            tend.nb = maximal_bbox(fg, g, sp, tn, Some(fg.nodes[tn].in_[0]), Some(e));
            beginpath(fg, g, path, e, REGULAREDGE, &mut tend, spline_merge(fg, tn));
            let (ht1t, _) = rank_pair(fg, g, fg.nodes[tn].rank);
            let tb = makeregularend(
                *tend.boxes.last().unwrap(),
                BOTTOM,
                fg.nodes[tn].coord.y - ht1t,
            );
            if box_nonempty(tb) {
                tend.boxes.push(tb);
            }
            path.start.theta = -std::f64::consts::FRAC_PI_2;
            path.start.constrained = true; // forced vertical departure (:1840)
            smode = false;
        }
        // final segment (:1843-1868)
        boxes.push(rank_box(fg, g, sp, fg.nodes[tn].rank));
        hend = PathEnd::default();
        hend.nb = maximal_bbox(fg, g, sp, hn, Some(e), None);
        endpath(
            fg,
            g,
            path,
            if hackflag { fwd_b } else { e },
            REGULAREDGE,
            &mut hend,
            spline_merge(fg, fg.edges[e].head),
        );
        let mut b2 = hend.nb;
        b2.ur.y = hend.boxes.last().unwrap().ur.y;
        b2.ll.y = hend.boxes.last().unwrap().ll.y;
        let (_, ht2h) = rank_pair(fg, g, fg.nodes[hn].rank);
        let hb2 = makeregularend(b2, TOP, fg.nodes[hn].coord.y + ht2h);
        if box_nonempty(hb2) {
            hend.boxes.push(hb2);
        }
        completeregularpath(fg, g, path, segfirst, e, &tend, &hend, &mut boxes);
        let mut ps = if is_spline {
            routesplines_(path, false)
        } else {
            let mut r = routesplines_(path, true);
            if let Some(pts) = r.as_mut() {
                if et == SplineType::Line && pts.len() > 4 {
                    straighten_line(pts);
                }
            }
            r
        };
        match ps {
            Some(ref pts) if pts.is_empty() => return,
            None => return,
            Some(ref mut pts) => pointfs.append(pts),
        }
        recover_slack(fg, segfirst, path);
        hn_final = if hackflag { fg.edges[fwd_b].head } else { hn };
    }

    // ---- step 3: multi-edge fan-out (:1883-1917)
    fan_out(fg, sp, fe, hn_final, &mut pointfs, edges);
}

/// dotsplines.c:1892-1914 — duplicate the routed points once per group
/// member, stepping interior control points one `Multisep` to the right per
/// edge (endpoints are NOT shifted).
fn fan_out(
    fg: &mut Fg,
    sp: &SplineInfo,
    fe: EId,
    hn: NId,
    pointfs: &mut Vec<PointF>,
    edges: &[EId],
) {
    let cnt = edges.len();
    if cnt == 1 {
        let sw = swap_ends_p(fg, fe);
        clip_and_install(fg, fe, hn, pointfs, sw, false);
        return;
    }
    let dx = sp.multisep * (cnt as f64 - 1.0) / 2.0;
    for k in 1..pointfs.len().saturating_sub(1) {
        pointfs[k].x -= dx;
    }
    let mut pts2 = pointfs.clone();
    let sw = swap_ends_p(fg, fe);
    clip_and_install(fg, fe, hn, &mut pts2, sw, false);
    for &ej in edges.iter().skip(1) {
        let mut e = ej;
        if fg.edges[e].tree_index & BWDEDGE != 0 {
            e = makefwdedge(fg, e);
        }
        for k in 1..pointfs.len().saturating_sub(1) {
            pointfs[k].x += sp.multisep;
        }
        let head = fg.edges[e].head;
        let mut pts2 = pointfs.clone();
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut pts2, sw, false);
    }
}

/// dotsplines.c:1921 `completeregularpath` — append the tail boxes, the
/// inter-rank boxes and the head boxes to the path, then adjust widths.
#[allow(clippy::too_many_arguments)]
fn completeregularpath(
    fg: &Fg,
    _g: GId,
    path: &mut Path,
    first: EId,
    last: EId,
    tendp: &PathEnd,
    hendp: &PathEnd,
    boxes: &mut Vec<BoxF>,
) {
    let uleft = top_bound(fg, first, -1);
    let uright = top_bound(fg, first, 1);
    if let Some(u) = uleft {
        if !getsplinepoints(fg, u) {
            return; // neighbor not routed yet
        }
    }
    if let Some(u) = uright {
        if !getsplinepoints(fg, u) {
            return;
        }
    }
    let lleft = bot_bound(fg, last, -1);
    let lright = bot_bound(fg, last, 1);
    if let Some(u) = lleft {
        if !getsplinepoints(fg, u) {
            return;
        }
    }
    if let Some(u) = lright {
        if !getsplinepoints(fg, u) {
            return;
        }
    }
    for b in &tendp.boxes {
        add_box(path, *b);
    }
    let fb = path.boxes.len() as i64 + 1;
    let lb = fb + boxes.len() as i64 - 3;
    for b in boxes.iter() {
        add_box(path, *b);
    }
    for b in hendp.boxes.iter().rev() {
        add_box(path, *b);
    }
    adjustregularpath(path, fb, lb);
}

/// dotsplines.c:1981 `adjustregularpath` — enforce MINW widths. The second
/// loop "was intended to guarantee an overlap between adjacent boxes of at
/// least MINW. It doesn't do this." (C comment :1967-1980) — reproduced
/// as-is. `fb`/`lb` use i64 because C's `lb = fb + size - 3` can index below
/// `fb` for single-rank edges.
fn adjustregularpath(path: &mut Path, fb: i64, lb: i64) {
    // first loop: interrank boxes only
    let mut i = fb - 1;
    while i < lb + 1 && i >= 0 && (i as usize) < path.boxes.len() {
        let bp1 = &mut path.boxes[i as usize];
        if (i - fb) % 2 == 0 {
            if bp1.ll.x >= bp1.ur.x {
                let x = (bp1.ll.x + bp1.ur.x) / 2.0;
                bp1.ll.x = x - HALFMINW;
                bp1.ur.x = x + HALFMINW;
            }
        } else if bp1.ll.x + MINW > bp1.ur.x {
            let x = (bp1.ll.x + bp1.ur.x) / 2.0;
            bp1.ll.x = x - HALFMINW;
            bp1.ur.x = x + HALFMINW;
        }
        i += 1;
    }
    // second loop: adjacent box pairs
    let nbox = path.boxes.len() as i64;
    for i in 0..nbox.saturating_sub(1) {
        let (a, b2) = if i >= fb && i <= lb && (i - fb) % 2 == 0 {
            (false, true)
        } else if i + 1 >= fb && i < lb && (i + 1 - fb) % 2 == 0 {
            (true, false)
        } else {
            continue;
        };
        let (mut bp1, mut bp2) = (path.boxes[i as usize], path.boxes[i as usize + 1]);
        if !a {
            // modify bp2
            if bp1.ll.x + MINW > bp2.ur.x {
                bp2.ur.x = bp1.ll.x + MINW;
            }
            if bp1.ur.x - MINW < bp2.ll.x {
                bp2.ll.x = bp1.ur.x - MINW;
            }
        } else {
            // modify bp1
            if bp1.ll.x + MINW > bp2.ur.x {
                bp1.ll.x = bp2.ur.x - MINW;
            }
            if bp1.ur.x - MINW < bp2.ll.x {
                bp1.ur.x = bp2.ll.x + MINW;
            }
        }
        path.boxes[i as usize] = bp1;
        path.boxes[i as usize + 1] = bp2;
        let _ = b2;
    }
}

/// dotsplines.c:2016 `rank_box` — the full-width inter-rank band box, cached
/// per rank (uninitialized slots have `LL.x == UR.x`).
fn rank_box(fg: &Fg, g: GId, sp: &mut SplineInfo, r: i32) -> BoxF {
    let ri = r as usize;
    if ri >= sp.rank_box.len() {
        return BoxF::default();
    }
    let mut b = sp.rank_box[ri];
    if b.ll.x == b.ur.x {
        let rank = &fg.graphs[g].rank[ri];
        let left0 = rank.v.first().copied();
        let left1 = fg.graphs[g]
            .rank
            .get(ri + 1)
            .and_then(|rk| rk.v.first().copied());
        b.ll.x = sp.left_bound;
        b.ur.x = sp.right_bound;
        b.ll.y = match left1 {
            Some(n1) => {
                let (_, ht2n) = rank_pair(fg, g, r + 1);
                fg.nodes[n1].coord.y + ht2n
            }
            None => 0.0,
        };
        b.ur.y = match left0 {
            Some(n0) => {
                let (ht1n, _) = rank_pair(fg, g, r);
                fg.nodes[n0].coord.y - ht1n
            }
            None => 0.0,
        };
        sp.rank_box[ri] = b;
    }
    b
}

/// dotsplines.c:2031 `straight_len` — count of vertically aligned virtual
/// successors starting at vnode n (x compared against the original n).
fn straight_len(fg: &Fg, n: NId) -> i32 {
    let mut cnt = 0;
    let mut v = n;
    loop {
        match fg.nodes[v].out.first() {
            Some(&e) => v = fg.edges[e].head,
            None => break,
        }
        if fg.nodes[v].node_type != NodeType::Virtual {
            break;
        }
        if fg.nodes[v].out.len() != 1 || fg.nodes[v].in_.len() != 1 {
            break;
        }
        if fg.nodes[v].coord.x != fg.nodes[n].coord.x {
            break;
        }
        cnt += 1;
    }
    cnt
}

/// dotsplines.c:2049 `straight_path` — advance `cnt` chain hops and append
/// the last point twice (so clip_and_install sees two 4-point Béziers with a
/// zero-length first segment).
fn straight_path(fg: &Fg, e: EId, cnt: i32, plist: &mut Vec<PointF>) -> EId {
    let mut f = e;
    let mut c = cnt;
    while c > 0 {
        c -= 1;
        match fg.nodes[fg.edges[f].head].out.first() {
            Some(&nx) => f = nx,
            None => break,
        }
    }
    if let Some(last) = plist.last().copied() {
        plist.push(last);
        plist.push(last);
    }
    f
}

/// dotsplines.c:2061 `recover_slack` — re-center/slide label vnodes inside
/// the slack the routed spline left unused. Mutates layout, so it must run
/// in exactly this order relative to later groups' maximal_bbox calls.
fn recover_slack(fg: &mut Fg, e: EId, p: &Path) {
    let mut b = 0usize; // skip first rank box
    let mut vn = fg.edges[e].head;
    loop {
        if !(fg.nodes[vn].node_type == NodeType::Virtual && !spline_merge(fg, vn)) {
            break;
        }
        while b < p.boxes.len() && p.boxes[b].ll.y > fg.nodes[vn].coord.y {
            b += 1;
        }
        if b >= p.boxes.len() {
            break;
        }
        if p.boxes[b].ur.y < fg.nodes[vn].coord.y {
            // vnode not inside this box — advance to the next vnode
            match fg.nodes[vn].out.first() {
                Some(&nx) => vn = fg.edges[nx].head,
                None => break,
            }
            continue;
        }
        if fg.nodes[vn].label.is_some() {
            resize_vn(
                fg,
                vn,
                p.boxes[b].ll.x,
                p.boxes[b].ur.x,
                p.boxes[b].ur.x + fg.nodes[vn].rw,
            );
        } else {
            resize_vn(
                fg,
                vn,
                p.boxes[b].ll.x,
                (p.boxes[b].ll.x + p.boxes[b].ur.x) / 2.0,
                p.boxes[b].ur.x,
            );
        }
        match fg.nodes[vn].out.first() {
            Some(&nx) => vn = fg.edges[nx].head,
            None => break,
        }
    }
}

/// dotsplines.c:2082 `resize_vn`.
fn resize_vn(fg: &mut Fg, vn: NId, lx: f64, cx: f64, rx: f64) {
    fg.nodes[vn].coord.x = cx;
    fg.nodes[vn].lw = cx - lx;
    fg.nodes[vn].rw = rx - cx;
}

/// dotsplines.c:2088 `top_bound` — among all out-edges of tail(e), the
/// nearest already-routed neighbor on `side` (side > 0 = right).
fn top_bound(fg: &Fg, e: EId, side: i32) -> Option<EId> {
    let t = fg.edges[e].tail;
    let he_order = fg.nodes[fg.edges[e].head].order;
    let mut ans: Option<EId> = None;
    for &f in &fg.nodes[t].out {
        let hf = fg.edges[f].head;
        if side * (fg.nodes[hf].order - he_order) <= 0 {
            continue;
        }
        if !has_spl_2level(fg, f) {
            continue;
        }
        let better = match ans {
            None => true,
            Some(a) => side * (fg.nodes[fg.edges[a].head].order - fg.nodes[hf].order) > 0,
        };
        if better {
            ans = Some(f);
        }
    }
    ans
}

/// dotsplines.c:2104 `bot_bound` — `top_bound` mirrored on the in-edges of
/// head(e), comparing tail orders.
fn bot_bound(fg: &Fg, e: EId, side: i32) -> Option<EId> {
    let h = fg.edges[e].head;
    let te_order = fg.nodes[fg.edges[e].tail].order;
    let mut ans: Option<EId> = None;
    for &f in &fg.nodes[h].in_ {
        let tf = fg.edges[f].tail;
        if side * (fg.nodes[tf].order - te_order) <= 0 {
            continue;
        }
        if !has_spl_2level(fg, f) {
            continue;
        }
        let better = match ans {
            None => true,
            Some(a) => side * (fg.nodes[fg.edges[a].tail].order - fg.nodes[tf].order) > 0,
        };
        if better {
            ans = Some(f);
        }
    }
    ans
}

/// top_bound/bot_bound's spline test: `ED_spl(f) == NULL &&
/// (ED_to_orig(f) == NULL || ED_spl(ED_to_orig(f)) == NULL)`.
fn has_spl_2level(fg: &Fg, f: EId) -> bool {
    if fg.edges[f].spl.is_some() {
        return true;
    }
    match fg.edges[f].to_orig {
        Some(o) => fg.edges[o].spl.is_some(),
        None => false,
    }
}

/// splines.c:1361 `getsplinepoints` — whether a routed spline exists on `e`
/// or anywhere up its `ED_to_orig` chain.
fn getsplinepoints(fg: &Fg, e: EId) -> bool {
    let mut le = e;
    loop {
        if fg.edges[le].spl.is_some() {
            return true;
        }
        if fg.edges[le].edge_type == EdgeType::Normal {
            return false;
        }
        match fg.edges[le].to_orig {
            Some(o) => le = o,
            None => return false,
        }
    }
}

// ---------------------------------------------------------------------------
// box geometry (dotsplines.c:2122-2299)
// ---------------------------------------------------------------------------

/// dotsplines.c:2122 `cl_vninside`.
fn cl_vninside(fg: &Fg, cl: GId, n: NId) -> bool {
    let bb = &fg.graphs[cl].bb;
    let c = fg.nodes[n].coord;
    bb.ll.x <= c.x && c.x <= bb.ur.x && bb.ll.y <= c.y && c.y <= bb.ur.y
}

/// dotsplines.c:2132 `REAL_CLUSTER` — `ND_clust(n) == g ? NULL : ND_clust(n)`.
fn real_cluster(fg: &Fg, g: GId, n: NId) -> Option<GId> {
    match fg.nodes[n].clust {
        Some(c) if c == g => None,
        other => other,
    }
}

/// dotsplines.c:2136 `cl_bound` — the cluster of the adjacent node that
/// interferes with `n`.
fn cl_bound(fg: &Fg, g: GId, n: NId, adj: NId) -> Option<GId> {
    let (tcl, hcl) = if fg.nodes[n].node_type == NodeType::Normal {
        (fg.nodes[n].clust, fg.nodes[n].clust)
    } else {
        match fg.nodes[n].out.first() {
            Some(&e) => match fg.edges[e].to_orig {
                Some(orig) => (
                    fg.nodes[fg.edges[orig].tail].clust,
                    fg.nodes[fg.edges[orig].head].clust,
                ),
                None => (None, None),
            },
            None => (None, None),
        }
    };
    if fg.nodes[adj].node_type == NodeType::Normal {
        let cl = real_cluster(fg, g, adj);
        if let Some(c) = cl {
            if Some(c) != tcl && Some(c) != hcl {
                return Some(c);
            }
        }
        return None;
    }
    // virtual adjacent: the tail-side then head-side cluster of its first
    // out edge's original, additionally requiring cl_vninside
    let orig = match fg.nodes[adj].out.first() {
        Some(&e) => fg.edges[e].to_orig,
        None => None,
    };
    if let Some(orig) = orig {
        for node in [fg.edges[orig].tail, fg.edges[orig].head] {
            let cl = real_cluster(fg, g, node);
            if let Some(c) = cl {
                if Some(c) != tcl && Some(c) != hcl && cl_vninside(fg, c, adj) {
                    return Some(c);
                }
            }
        }
    }
    None
}

/// dotsplines.c:2175 `maximal_bbox` — the maximal axis-aligned box around
/// `vn` that the path may occupy on vn's rank. Virtual neighbors "give" only
/// `Splinesep = nodesep/4`; real nodes `nodesep/2`; clusters truncate via
/// their bbox ± Splinesep.
fn maximal_bbox(
    fg: &Fg,
    g: GId,
    sp: &SplineInfo,
    vn: NId,
    ie: Option<EId>,
    oe: Option<EId>,
) -> BoxF {
    let coord = fg.nodes[vn].coord;
    let nodesep2 = fg.graphs[g].nodesep as f64 / 2.0;

    // left
    let mut b = coord.x - fg.nodes[vn].lw - FUDGE;
    let ll_x = match neighbor(fg, g, vn, ie, oe, -1) {
        Some(l) => {
            let nb = match cl_bound(fg, g, vn, l) {
                Some(cl) => fg.graphs[cl].bb.ur.x + sp.splinesep,
                None => {
                    let mut nb = fg.nodes[l].coord.x + fg.nodes[l].mval;
                    if fg.nodes[l].node_type == NodeType::Normal {
                        nb += nodesep2;
                    } else {
                        nb += sp.splinesep;
                    }
                    nb
                }
            };
            if nb < b {
                b = nb;
            }
            round(b)
        }
        None => round(b).min(sp.left_bound),
    };

    // right — "we have to leave room for our own label!"
    let b0 = if fg.nodes[vn].node_type == NodeType::Virtual && fg.nodes[vn].label.is_some() {
        coord.x + 10.0
    } else {
        coord.x + fg.nodes[vn].rw + FUDGE
    };
    let mut b = b0;
    let mut ur_x = match neighbor(fg, g, vn, ie, oe, 1) {
        Some(r) => {
            let nb = match cl_bound(fg, g, vn, r) {
                Some(cl) => fg.graphs[cl].bb.ll.x - sp.splinesep,
                None => {
                    let mut nb = fg.nodes[r].coord.x - fg.nodes[r].lw;
                    if fg.nodes[r].node_type == NodeType::Normal {
                        nb -= nodesep2;
                    } else {
                        nb -= sp.splinesep;
                    }
                    nb
                }
            };
            if nb > b {
                b = nb;
            }
            round(b)
        }
        None => round(b).max(sp.right_bound),
    };

    if fg.nodes[vn].node_type == NodeType::Virtual && fg.nodes[vn].label.is_some() {
        ur_x -= fg.nodes[vn].rw;
        if ur_x < ll_x {
            ur_x = coord.x;
        }
    }

    let (ht1, ht2) = rank_pair(fg, g, fg.nodes[vn].rank);
    BoxF {
        ll: PointF::new(ll_x, coord.y - ht1),
        ur: PointF::new(ur_x, coord.y + ht2),
    }
}

/// dotsplines.c:2234 `neighbor` — scan the rank in `dir` for the first node
/// that must not be crossed: a labeled vnode, a real node, or a vnode whose
/// paths do not cross ours.
fn neighbor(fg: &Fg, g: GId, vn: NId, ie: Option<EId>, oe: Option<EId>, dir: i32) -> Option<NId> {
    let r = fg.nodes[vn].rank;
    let rank = fg.graphs[g].rank.get(r.max(0) as usize)?;
    let n = rank.n as i32;
    let mut i = fg.nodes[vn].order + dir;
    while i >= 0 && i < n {
        let cand = rank.v[i as usize];
        if fg.nodes[cand].node_type == NodeType::Virtual && fg.nodes[cand].label.is_some() {
            return Some(cand);
        }
        if fg.nodes[cand].node_type == NodeType::Normal {
            return Some(cand);
        }
        if !pathscross(fg, cand, vn, ie, oe) {
            return Some(cand);
        }
        i += dir;
    }
    None
}

/// dotsplines.c:2258 `pathscross` — do the chains out of (or into) n0 and
/// the ie/oe counterpart swap left/right order within 2 hops?
fn pathscross(fg: &Fg, n0: NId, n1: NId, ie1: Option<EId>, oe1: Option<EId>) -> bool {
    let order = fg.nodes[n0].order > fg.nodes[n1].order;
    if fg.nodes[n0].out.len() != 1 && fg.nodes[n1].out.len() != 1 {
        return false;
    }
    // walk up to 2 hops along out-edges comparing heads
    if fg.nodes[n0].out.len() == 1 {
        if let Some(mut e1) = oe1 {
            let mut e0 = fg.nodes[n0].out[0];
            for _ in 0..2 {
                let na = fg.edges[e0].head;
                let nb = fg.edges[e1].head;
                if na == nb {
                    break;
                }
                if order != (fg.nodes[na].order > fg.nodes[nb].order) {
                    return true;
                }
                if fg.nodes[na].out.len() != 1 || fg.nodes[na].node_type == NodeType::Normal {
                    break;
                }
                e0 = fg.nodes[na].out[0];
                if fg.nodes[nb].out.len() != 1 || fg.nodes[nb].node_type == NodeType::Normal {
                    break;
                }
                e1 = fg.nodes[nb].out[0];
            }
        }
    }
    // same walk on in-edges comparing tails
    if fg.nodes[n0].in_.len() == 1 {
        if let Some(mut e1) = ie1 {
            let mut e0 = fg.nodes[n0].in_[0];
            for _ in 0..2 {
                let na = fg.edges[e0].tail;
                let nb = fg.edges[e1].tail;
                if na == nb {
                    break;
                }
                if order != (fg.nodes[na].order > fg.nodes[nb].order) {
                    return true;
                }
                if fg.nodes[na].in_.len() != 1 || fg.nodes[na].node_type == NodeType::Normal {
                    break;
                }
                e0 = fg.nodes[na].in_[0];
                if fg.nodes[nb].in_.len() != 1 || fg.nodes[nb].node_type == NodeType::Normal {
                    break;
                }
                e1 = fg.nodes[nb].in_[0];
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------
// EDGETYPE_LINE (dotsplines.c:1625-1705)
// ---------------------------------------------------------------------------

/// dotsplines.c:1625 `leftOf` — true if p3 is to the left of ray p1→p2.
fn left_of(p1: PointF, p2: PointF, p3: PointF) -> bool {
    (p1.y - p2.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p1.x - p2.x) > 0.0
}

/// dotsplines.c:1643 `makeLineEdge` — route a long edge as a straight line
/// (two segments with the bend near the label when labeled). Returns the
/// far node and appends the points, or None to fall through to box routing
/// (adjacent ranks, or span 2 with edge labels).
fn make_line_edge(fg: &Fg, g: GId, fe: EId, points: &mut Vec<PointF>) -> Option<NId> {
    let mut e = fe;
    loop {
        if fg.edges[e].edge_type == EdgeType::Normal {
            break;
        }
        e = fg.edges[e].to_orig?;
    }
    let hn = fg.edges[e].head;
    let tn = fg.edges[e].tail;
    let delr = (fg.nodes[hn].rank - fg.nodes[tn].rank).abs();
    if delr == 1 || (delr == 2 && fg.graphs[0].has_labels & 1 != 0) {
        return None;
    }
    let (hp, startp, endp) = if fg.edges[fe].tail == tn {
        (
            hn,
            fg.nodes[tn].coord.add(fg.edges[e].tail_port.p),
            fg.nodes[hn].coord.add(fg.edges[e].head_port.p),
        )
    } else {
        (
            tn,
            fg.nodes[hn].coord.add(fg.edges[e].head_port.p),
            fg.nodes[tn].coord.add(fg.edges[e].tail_port.p),
        )
    };

    if let Some(li) = fg.edges[e].label {
        let dimen = fg.labels[li].dimen;
        let (width, height) = if fg.graphs[g].rankdir.flip() {
            (dimen.y, dimen.x)
        } else {
            (dimen.x, dimen.y)
        };
        let mut lp = fg.labels[li].pos;
        if left_of(endp, startp, lp) {
            lp.x += width / 2.0;
            lp.y -= height / 2.0;
        } else {
            lp.x -= width / 2.0;
            lp.y += height / 2.0;
        }
        points.extend_from_slice(&[startp, startp, lp, lp, lp, endp, endp]);
    } else {
        points.extend_from_slice(&[startp, startp, endp, endp]);
    }
    Some(hp)
}

/// dotsplines.c:1810-1814/1860-1868 — collapse an adjacent-rank polyline to
/// a single straight Bézier (`ps[1]=ps[0]; ps[3]=ps[2]=ps[pn-1]; pn=4`).
fn straighten_line(ps: &mut Vec<PointF>) {
    let n = ps.len();
    let first = ps[0];
    let last = ps[n - 1];
    ps[1] = first;
    ps[2] = last;
    ps[3] = last;
    ps.truncate(4);
}

// ---------------------------------------------------------------------------
// flat edges (dotsplines.c:951-1622)
// ---------------------------------------------------------------------------

/// dotsplines.c:1509 `make_flat_edge` — the flat dispatcher.
fn make_flat_edge(
    fg: &mut Fg,
    g: GId,
    sp: &mut SplineInfo,
    path: &mut Path,
    edges: &[EId],
    et: SplineType,
) -> Result<(), i32> {
    // Get sample edge; normalize to go from left to right
    let e_raw = edges[0];
    let mut is_adjacent = fg.edges[e_raw].adjacent;
    let mut e = e_raw;
    if fg.edges[e_raw].tree_index & BWDEDGE != 0 {
        e = makefwdedge(fg, e_raw);
    }
    // The lead edge might not have been marked earlier as adjacent, so
    // check them all (dotsplines.c:1527-1533).
    for &x in edges.iter().skip(1) {
        if fg.edges[x].adjacent {
            is_adjacent = true;
            break;
        }
    }
    if is_adjacent {
        return make_flat_adj_edges(fg, g, sp, path, edges, e, et);
    }
    if fg.edges[e].label.is_some() {
        // edges with labels aren't multi-edges
        make_flat_labeled_edge(fg, g, sp, path, e, et);
        return Ok(());
    }
    if et == SplineType::Line {
        make_simple_flat(fg, e, edges, et);
        return Ok(());
    }
    let tside = fg.edges[e].tail_port.side as i32;
    let hside = fg.edges[e].head_port.side as i32;
    if (tside == BOTTOM && hside != TOP) || (hside == BOTTOM && tside != TOP) {
        make_flat_bottom_edges(fg, g, sp, path, edges, e, et == SplineType::Spline);
        return Ok(());
    }
    // default: the "top" route (:1554-1621)
    let tn = fg.edges[e].tail;
    let hn = fg.edges[e].head;
    let r = fg.nodes[tn].rank;
    let vspace: f64 = if r > 0 {
        // label vnodes occupy the rank above flat edges
        let prevr = if fg.graphs[0].has_labels & 1 != 0 {
            r - 2
        } else {
            r - 1
        };
        let py = rank_v0_y(fg, g, prevr).unwrap_or(0.0);
        let (pht1, _) = rank_pair(fg, g, prevr);
        let (_, ht2) = rank_pair(fg, g, r);
        py - pht1 - fg.nodes[tn].coord.y - ht2
    } else {
        fg.graphs[g].ranksep as f64
    };
    let stepx = sp.multisep / (edges.len() as f64 + 1.0);
    let stepy = vspace / (edges.len() as f64 + 1.0);

    let mut tend = PathEnd::default();
    let mut hend = PathEnd::default();
    make_flat_end(fg, g, sp, path, tn, e, &mut tend, true);
    make_flat_end(fg, g, sp, path, hn, e, &mut hend, false);

    for (i, &ei) in edges.iter().enumerate() {
        path.boxes.clear(); // P->nbox = 0
        let mut boxes: [BoxF; 3] = [BoxF::default(); 3];
        let b = *tend.boxes.last().unwrap();
        boxes[0].ll.x = b.ll.x;
        boxes[0].ll.y = b.ur.y;
        boxes[0].ur.x = b.ur.x + (i as f64 + 1.0) * stepx;
        boxes[0].ur.y = b.ur.y + (i as f64 + 1.0) * stepy;
        boxes[1].ll.x = tend.boxes.last().unwrap().ll.x;
        boxes[1].ll.y = boxes[0].ur.y;
        boxes[1].ur.x = hend.boxes.last().unwrap().ur.x;
        boxes[1].ur.y = boxes[1].ll.y + stepy;
        let b = *hend.boxes.last().unwrap();
        boxes[2].ur.x = b.ur.x;
        boxes[2].ll.y = b.ur.y;
        boxes[2].ll.x = b.ll.x - (i as f64 + 1.0) * stepx;
        boxes[2].ur.y = boxes[1].ll.y;

        for bb in &tend.boxes {
            add_box(path, *bb);
        }
        for bb in &boxes {
            add_box(path, *bb);
        }
        for bb in hend.boxes.iter().rev() {
            add_box(path, *bb);
        }
        let mut ps = if et == SplineType::Spline {
            routesplines_(path, false)
        } else {
            routesplines_(path, true)
        };
        match ps {
            Some(ref pts) if pts.is_empty() => return Ok(()),
            None => return Ok(()),
            Some(ref mut pts) => {
                let head = fg.edges[ei].head;
                let sw = swap_ends_p(fg, ei);
                clip_and_install(fg, ei, head, pts, sw, false);
            }
        }
    }
    Ok(())
}

/// dotsplines.c:1129 `make_flat_adj_edges` — flat edges between rank-adjacent
/// nodes. C runs a full recursive dot layout on a rotated clone for the
/// ported case (:1177-1239); that machinery is out of scope for this
/// milestone, so the ported case falls back to a simple spline through the
/// mid-gap (documented deviation).
fn make_flat_adj_edges(
    fg: &mut Fg,
    g: GId,
    _sp: &mut SplineInfo,
    _path: &mut Path,
    edges: &[EId],
    e: EId,
    et: SplineType,
) -> Result<(), i32> {
    let tn = fg.edges[e].tail;
    let hn = fg.edges[e].head;
    // C warns and bails for record shapes (dotsplines.c:1144-1152); records
    // are not modeled.
    let labels = edges
        .iter()
        .filter(|&&x| fg.edges[x].label.is_some())
        .count();
    let ports = edges
        .iter()
        .any(|&x| fg.edges[x].tail_port.defined || fg.edges[x].head_port.defined);

    if !ports {
        if labels == 0 {
            make_simple_flat(fg, e, edges, et);
        } else {
            make_simple_flat_labels(fg, g, tn, hn, edges, et, labels);
        }
        return Ok(());
    }

    // DEVIATION (dotsplines.c:1129-1288): with ports, C clones the graph,
    // flips rankdir, runs dot_rank/dot_mincross/dot_position/dot_splines_
    // recursively and copies the splines back via transformf. Emit a simple
    // one-Bézier route per edge through the mid-gap instead.
    for &ei in edges {
        let t = fg.edges[ei].tail;
        let h = fg.edges[ei].head;
        let tp = fg.nodes[t].coord.add(fg.edges[ei].tail_port.p);
        let hp = fg.nodes[h].coord.add(fg.edges[ei].head_port.p);
        let mut pts = vec![
            tp,
            PointF::new((2.0 * tp.x + hp.x) / 3.0, tp.y),
            PointF::new((2.0 * hp.x + tp.x) / 3.0, hp.y),
            hp,
        ];
        let head = fg.edges[ei].head;
        let sw = swap_ends_p(fg, ei);
        clip_and_install(fg, ei, head, &mut pts, sw, false);
    }
    Ok(())
}

/// dotsplines.c:1082 `makeSimpleFlat` — the multi-edge spindle between
/// adjacent nodes without ports/labels: `(2a+b)/3` control points, fanned
/// vertically by `ND_ht(tn)/(cnt-1)`.
fn make_simple_flat(fg: &mut Fg, e: EId, edges: &[EId], et: SplineType) {
    let tn = fg.edges[e].tail;
    let hn = fg.edges[e].head;
    let tp = fg.nodes[tn].coord.add(fg.edges[e].tail_port.p);
    let hp = fg.nodes[hn].coord.add(fg.edges[e].head_port.p);
    let cnt = edges.len();

    let stepy = if cnt > 1 {
        fg.nodes[tn].ht / (cnt as f64 - 1.0)
    } else {
        0.0
    };
    let mut dy = tp.y - if cnt > 1 { fg.nodes[tn].ht / 2.0 } else { 0.0 };

    for &ei in edges {
        let mut points: Vec<PointF> = Vec::with_capacity(10);
        if et == SplineType::Spline || et == SplineType::Line {
            points.push(tp);
            points.push(PointF::new((2.0 * tp.x + hp.x) / 3.0, dy));
            points.push(PointF::new((2.0 * hp.x + tp.x) / 3.0, dy));
            points.push(hp);
        } else {
            // EDGETYPE_PLINE: degenerate 10-point form
            points.push(tp);
            points.push(tp);
            let c1 = PointF::new((2.0 * tp.x + hp.x) / 3.0, dy);
            let c2 = PointF::new((2.0 * hp.x + tp.x) / 3.0, dy);
            points.push(c1);
            points.push(c1);
            points.push(c1);
            points.push(c2);
            points.push(c2);
            points.push(c2);
            points.push(hp);
            points.push(hp);
        }
        dy += stepy;
        let head = fg.edges[ei].head;
        let sw = swap_ends_p(fg, ei);
        clip_and_install(fg, ei, head, &mut points, sw, false);
    }
}

/// dotsplines.c:914 `edgelblcmpfn` — labeled first; wider `dimen.x` first;
/// then wider `dimen.y`.
fn edgelblcmpfn(fg: &Fg, e0: EId, e1: EId) -> Ordering {
    match (fg.edges[e0].label, fg.edges[e1].label) {
        (Some(l0), Some(l1)) => {
            let (s0, s1) = (fg.labels[l0].dimen, fg.labels[l1].dimen);
            if s0.x > s1.x {
                return Ordering::Less;
            }
            if s0.x < s1.x {
                return Ordering::Greater;
            }
            if s0.y > s1.y {
                return Ordering::Less;
            }
            if s0.y < s1.y {
                return Ordering::Greater;
            }
            Ordering::Equal
        }
        (Some(_), None) => Ordering::Less,
        (None, Some(_)) => Ordering::Greater,
        (None, None) => Ordering::Equal,
    }
}

/// pathplan `simpleSplineRoute` (routespl.c:168) — route tp→hp through an
/// 8-gon (zero end-slope vectors, as in C).
fn simple_spline_route(
    tp: PointF,
    hp: PointF,
    poly: &[PointF],
    polyline: bool,
) -> Option<Vec<PointF>> {
    let eps = [tp, hp];
    let pl = pshortestpath(poly, &eps).ok()?;
    if polyline {
        Some(make_polyline(&pl))
    } else {
        let n = poly.len();
        let edges: Vec<(PointF, PointF)> = (0..n).map(|i| (poly[i], poly[(i + 1) % n])).collect();
        proutespline(&edges, &pl, &[PointF::ZERO, PointF::ZERO]).ok()
    }
}

/// dotsplines.c:951 `makeSimpleFlatLabels` — adjacent flat edges with labels
/// (no ports): first edge straight, subsequent edges octagon-routed
/// alternately below/above with `LBL_SPACE` gaps.
fn make_simple_flat_labels(
    fg: &mut Fg,
    _g: GId,
    tn: NId,
    hn: NId,
    edges: &[EId],
    et: SplineType,
    n_lbls: usize,
) {
    let e0 = edges[0];
    let tp = fg.nodes[tn].coord.add(fg.edges[e0].tail_port.p);
    let hp = fg.nodes[hn].coord.add(fg.edges[e0].head_port.p);

    let mut earray: Vec<EId> = edges.to_vec();
    earray.sort_by(|&a, &b| edgelblcmpfn(fg, a, b));

    let leftend = tp.x + fg.nodes[tn].rw;
    let rightend = hp.x - fg.nodes[hn].lw;
    let ctrx = (leftend + rightend) / 2.0;

    let lbl0 = fg.edges[earray[0]].label.unwrap();
    let (dim0x, dim0y) = (fg.labels[lbl0].dimen.x, fg.labels[lbl0].dimen.y);

    // first edge
    let mut points = vec![tp, tp, hp, hp];
    let head0 = fg.edges[earray[0]].head;
    let sw0 = swap_ends_p(fg, earray[0]);
    clip_and_install(fg, earray[0], head0, &mut points, sw0, false);
    fg.labels[lbl0].pos = PointF::new(ctrx, tp.y + (dim0y + LBL_SPACE) / 2.0);

    let mut miny = tp.y + LBL_SPACE / 2.0;
    let mut maxy = miny + dim0y;
    let uminx = ctrx - dim0x / 2.0;
    let umaxx = ctrx + dim0x / 2.0;
    let mut lminx = 0.0;
    let mut lmaxx = 0.0;

    let mut i = 1usize;
    while i < n_lbls {
        let e = earray[i];
        let li = fg.edges[e].label.unwrap();
        let (dx2, dy2) = (fg.labels[li].dimen.x, fg.labels[li].dimen.y);
        let mut poly = [PointF::ZERO; 8];
        let ctry;
        if i % 2 == 1 {
            // down
            if i == 1 {
                lminx = ctrx - dx2 / 2.0;
                lmaxx = ctrx + dx2 / 2.0;
            }
            miny -= LBL_SPACE + dy2;
            poly[0] = tp;
            poly[1] = PointF::new(tp.x, miny - LBL_SPACE);
            poly[2] = PointF::new(hp.x, poly[1].y);
            poly[3] = hp;
            poly[4] = PointF::new(lmaxx, hp.y);
            poly[5] = PointF::new(lmaxx, miny);
            poly[6] = PointF::new(lminx, miny);
            poly[7] = PointF::new(lminx, tp.y);
            ctry = miny + dy2 / 2.0;
        } else {
            // up
            poly[0] = tp;
            poly[1] = PointF::new(uminx, tp.y);
            poly[2] = PointF::new(uminx, maxy);
            poly[3] = PointF::new(umaxx, maxy);
            poly[4] = PointF::new(umaxx, hp.y);
            poly[5] = hp;
            poly[6] = PointF::new(hp.x, maxy + LBL_SPACE);
            poly[7] = PointF::new(tp.x, maxy + LBL_SPACE);
            ctry = maxy + dy2 / 2.0 + LBL_SPACE;
            maxy += dy2 + LBL_SPACE;
        }
        let polyline = et == SplineType::Pline;
        let ps = match simple_spline_route(tp, hp, &poly, polyline) {
            Some(ps) if !ps.is_empty() => ps,
            _ => return, // edge silently unrouted, as in C
        };
        fg.labels[li].pos = PointF::new(ctrx, ctry);
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut ps.clone(), sw, false);
        i += 1;
    }

    // edges with no labels
    while i < earray.len() {
        let e = earray[i];
        let mut poly = [PointF::ZERO; 8];
        if i % 2 == 1 {
            // down
            if i == 1 {
                lminx = (2.0 * leftend + rightend) / 3.0;
                lmaxx = (leftend + 2.0 * rightend) / 3.0;
            }
            miny -= LBL_SPACE;
            poly[0] = tp;
            poly[1] = PointF::new(tp.x, miny - LBL_SPACE);
            poly[2] = PointF::new(hp.x, poly[1].y);
            poly[3] = hp;
            poly[4] = PointF::new(lmaxx, hp.y);
            poly[5] = PointF::new(lmaxx, miny);
            poly[6] = PointF::new(lminx, miny);
            poly[7] = PointF::new(lminx, tp.y);
        } else {
            // up
            poly[0] = tp;
            poly[1] = PointF::new(uminx, tp.y);
            poly[2] = PointF::new(uminx, maxy);
            poly[3] = PointF::new(umaxx, maxy);
            poly[4] = PointF::new(umaxx, hp.y);
            poly[5] = hp;
            poly[6] = PointF::new(hp.x, maxy + LBL_SPACE);
            poly[7] = PointF::new(tp.x, maxy + LBL_SPACE);
            maxy += LBL_SPACE;
        }
        let polyline = et == SplineType::Pline;
        let ps = match simple_spline_route(tp, hp, &poly, polyline) {
            Some(ps) if !ps.is_empty() => ps,
            _ => return,
        };
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut ps.clone(), sw, false);
        i += 1;
    }
}

/// dotsplines.c:1290 `makeFlatEnd` — endpoint boxes for a flat edge routed
/// along the top.
#[allow(clippy::too_many_arguments)]
fn make_flat_end(
    fg: &mut Fg,
    g: GId,
    sp: &SplineInfo,
    path: &mut Path,
    n: NId,
    e: EId,
    endp: &mut PathEnd,
    is_begin: bool,
) {
    endp.nb = maximal_bbox(fg, g, sp, n, None, Some(e));
    endp.sidemask = TOP;
    if is_begin {
        beginpath(fg, g, path, e, FLATEDGE, endp, false);
    } else {
        endpath(fg, g, path, e, FLATEDGE, endp, false);
    }
    let mut b = endp.nb;
    b.ur.y = endp.boxes.last().unwrap().ur.y;
    b.ll.y = endp.boxes.last().unwrap().ll.y;
    let (_, ht2) = rank_pair(fg, g, fg.nodes[n].rank);
    let bb = makeregularend(b, TOP, fg.nodes[n].coord.y + ht2);
    if box_nonempty(bb) {
        endp.boxes.push(bb);
    }
}

/// dotsplines.c:1305 `makeBottomFlatEnd` — the bottom-side mirror.
#[allow(clippy::too_many_arguments)]
fn make_bottom_flat_end(
    fg: &mut Fg,
    g: GId,
    sp: &SplineInfo,
    path: &mut Path,
    n: NId,
    e: EId,
    endp: &mut PathEnd,
    is_begin: bool,
) {
    endp.nb = maximal_bbox(fg, g, sp, n, None, Some(e));
    endp.sidemask = BOTTOM;
    if is_begin {
        beginpath(fg, g, path, e, FLATEDGE, endp, false);
    } else {
        endpath(fg, g, path, e, FLATEDGE, endp, false);
    }
    let mut b = endp.nb;
    b.ur.y = endp.boxes.last().unwrap().ur.y;
    b.ll.y = endp.boxes.last().unwrap().ll.y;
    let (_, ht2) = rank_pair(fg, g, fg.nodes[n].rank);
    let bb = makeregularend(b, BOTTOM, fg.nodes[n].coord.y - ht2);
    if box_nonempty(bb) {
        endp.boxes.push(bb);
    }
}

/// dotsplines.c:1321 `make_flat_labeled_edge` — one labeled non-adjacent
/// flat edge, routed through boxes around its label vnode.
fn make_flat_labeled_edge(
    fg: &mut Fg,
    g: GId,
    sp: &SplineInfo,
    path: &mut Path,
    e: EId,
    et: SplineType,
) {
    let tn = fg.edges[e].tail;
    let hn = fg.edges[e].head;
    let li = match fg.edges[e].label {
        Some(li) => li,
        None => return,
    };

    // ln = tail of the deepest ED_to_virt edge = the label vnode
    let ln = match fg.edges[e].to_virt {
        Some(mut f) => {
            while let Some(nx) = fg.edges[f].to_virt {
                f = nx;
            }
            fg.edges[f].tail
        }
        None => {
            // C assumes the flat.c label vnode chain exists; without it,
            // fall back to the unlabeled spindle and place the label in the
            // mid-gap (documented deviation).
            make_simple_flat(fg, e, &[e], et);
            let mid = (fg.nodes[tn].coord.x + fg.nodes[hn].coord.x) / 2.0;
            fg.labels[li].pos = PointF::new(mid, fg.nodes[tn].coord.y);
            return;
        }
    };
    fg.labels[li].pos = fg.nodes[ln].coord;

    if et == SplineType::Line {
        let startp = fg.nodes[tn].coord.add(fg.edges[e].tail_port.p);
        let endp = fg.nodes[hn].coord.add(fg.edges[e].head_port.p);
        let mut lp = fg.labels[li].pos;
        lp.y -= fg.labels[li].dimen.y / 2.0;
        let mut points = vec![startp, startp, lp, lp, lp, endp, endp];
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut points, sw, false);
        return;
    }

    // label box
    let mut lb = BoxF::default();
    lb.ll.x = fg.nodes[ln].coord.x - fg.nodes[ln].lw;
    lb.ur.x = fg.nodes[ln].coord.x + fg.nodes[ln].rw;
    lb.ur.y = fg.nodes[ln].coord.y + fg.nodes[ln].ht / 2.0;
    let (_, ht2t) = rank_pair(fg, g, fg.nodes[tn].rank);
    let (ht1t, _) = rank_pair(fg, g, fg.nodes[tn].rank);
    let mut ydelta = fg.nodes[ln].coord.y - ht1t - fg.nodes[tn].coord.y + ht2t;
    ydelta /= 6.0;
    lb.ll.y = lb.ur.y - 5.0f64.max(ydelta);

    let mut tend = PathEnd::default();
    let mut hend = PathEnd::default();
    make_flat_end(fg, g, sp, path, tn, e, &mut tend, true);
    make_flat_end(fg, g, sp, path, hn, e, &mut hend, false);

    let tlast = *tend.boxes.last().unwrap();
    let hlast = *hend.boxes.last().unwrap();
    let boxes = [
        BoxF {
            ll: PointF::new(tlast.ll.x, tlast.ur.y),
            ur: lb.ll,
        },
        BoxF {
            ll: PointF::new(tlast.ll.x, lb.ll.y),
            ur: PointF::new(hlast.ur.x, lb.ur.y),
        },
        BoxF {
            ll: PointF::new(lb.ur.x, hlast.ur.y),
            ur: PointF::new(hlast.ur.x, lb.ll.y),
        },
    ];

    for b in &tend.boxes {
        add_box(path, *b);
    }
    for b in &boxes {
        add_box(path, *b);
    }
    for b in hend.boxes.iter().rev() {
        add_box(path, *b);
    }

    let mut ps = if et == SplineType::Spline {
        routesplines_(path, false)
    } else {
        routesplines_(path, true)
    };
    match ps {
        Some(ref pts) if pts.is_empty() => {}
        Some(ref mut pts) => {
            let head = fg.edges[e].head;
            let sw = swap_ends_p(fg, e);
            clip_and_install(fg, e, head, pts, sw, false);
        }
        None => {}
    }
}

/// dotsplines.c:1425 `make_flat_bottom_edges` — flat edges whose ports put
/// them on the bottom side, stacked downward. NOTE: `rank_t::pht1/pht2` are
/// not in the model; `ht1`/`ht2` are used instead (documented deviation).
fn make_flat_bottom_edges(
    fg: &mut Fg,
    g: GId,
    sp: &SplineInfo,
    path: &mut Path,
    edges: &[EId],
    e: EId,
    use_splines: bool,
) {
    let tn = fg.edges[e].tail;
    let hn = fg.edges[e].head;
    let r = fg.nodes[tn].rank;
    let vspace: f64 = if r < fg.graphs[g].maxrank {
        let ny = rank_v0_y(fg, g, r + 1).unwrap_or(0.0);
        let (pht1, _) = rank_pair(fg, g, r);
        let (_, pht2) = rank_pair(fg, g, r + 1);
        fg.nodes[tn].coord.y - pht1 - (ny + pht2)
    } else {
        fg.graphs[g].ranksep as f64
    };
    let stepx = sp.multisep / (edges.len() as f64 + 1.0);
    let stepy = vspace / (edges.len() as f64 + 1.0);

    let mut tend = PathEnd::default();
    let mut hend = PathEnd::default();
    make_bottom_flat_end(fg, g, sp, path, tn, e, &mut tend, true);
    make_bottom_flat_end(fg, g, sp, path, hn, e, &mut hend, false);

    for (i, &ei) in edges.iter().enumerate() {
        path.boxes.clear();
        let mut boxes: [BoxF; 3] = [BoxF::default(); 3];
        let b = *tend.boxes.last().unwrap();
        boxes[0].ll.x = b.ll.x;
        boxes[0].ur.y = b.ll.y;
        boxes[0].ur.x = b.ur.x + (i as f64 + 1.0) * stepx;
        boxes[0].ll.y = b.ll.y - (i as f64 + 1.0) * stepy;
        boxes[1].ll.x = tend.boxes.last().unwrap().ll.x;
        boxes[1].ur.y = boxes[0].ll.y;
        boxes[1].ur.x = hend.boxes.last().unwrap().ur.x;
        boxes[1].ll.y = boxes[1].ur.y - stepy;
        let b = *hend.boxes.last().unwrap();
        boxes[2].ur.x = b.ur.x;
        boxes[2].ur.y = b.ll.y;
        boxes[2].ll.x = b.ll.x - (i as f64 + 1.0) * stepx;
        boxes[2].ll.y = boxes[1].ur.y;

        for bb in &tend.boxes {
            add_box(path, *bb);
        }
        for bb in &boxes {
            add_box(path, *bb);
        }
        for bb in hend.boxes.iter().rev() {
            add_box(path, *bb);
        }
        let mut ps = if use_splines {
            routesplines_(path, false)
        } else {
            routesplines_(path, true)
        };
        match ps {
            Some(ref pts) if pts.is_empty() => return,
            None => return,
            Some(ref mut pts) => {
                let head = fg.edges[ei].head;
                let sw = swap_ends_p(fg, ei);
                clip_and_install(fg, ei, head, pts, sw, false);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// self loops (dotsplines.c:395-416 → common/splines.c:772-1201)
// ---------------------------------------------------------------------------

/// splines.c:772 `convert_sides_to_points` — the vertices table
/// {12,4,6,2,3,1,9,8} gives the cumulative side value of each node point
/// (TL, T, TR, R, BR, B, BL, L); `pair_a` maps tail/head vertex indices to
/// a pair code.
fn convert_sides_to_points(tail_side: i32, head_side: i32) -> i32 {
    const VERTICES: [i32; 8] = [12, 4, 6, 2, 3, 1, 9, 8];
    const PAIR_A: [[i32; 8]; 8] = [
        [11, 12, 13, 14, 15, 16, 17, 18],
        [21, 22, 23, 24, 25, 26, 27, 28],
        [31, 32, 33, 34, 35, 36, 37, 38],
        [41, 42, 43, 44, 45, 46, 47, 48],
        [51, 52, 53, 54, 55, 56, 57, 58],
        [61, 62, 63, 64, 65, 66, 67, 68],
        [71, 72, 73, 74, 75, 76, 77, 78],
        [81, 82, 83, 84, 85, 86, 87, 88],
    ];
    let mut tail_i = -1i32;
    let mut head_i = -1i32;
    for (i, &v) in VERTICES.iter().enumerate() {
        if head_side == v {
            head_i = i as i32;
            break;
        }
    }
    for (i, &v) in VERTICES.iter().enumerate() {
        if tail_side == v {
            tail_i = i as i32;
            break;
        }
    }
    if tail_i < 0 || head_i < 0 {
        0
    } else {
        PAIR_A[tail_i as usize][head_i as usize]
    }
}

/// splines.c:1162 `makeSelfEdge` — dispatch on port sides.
fn make_self_edge(fg: &mut Fg, g: GId, edges: &[EId], sizex: f64, sizey: f64) {
    let e = edges[0];
    let tp_def = fg.edges[e].tail_port.defined;
    let hp_def = fg.edges[e].head_port.defined;
    let ts = fg.edges[e].tail_port.side as i32;
    let hs = fg.edges[e].head_port.side as i32;

    // self edge without ports, or with all ports inside/on the right, or at
    // most 1 on top and at most 1 on bottom
    let right_ok = (!tp_def && !hp_def)
        || ((ts & LEFT == 0) && (hs & LEFT == 0) && (ts != hs || (ts & (TOP | BOTTOM) == 0)));
    if right_ok {
        self_right(fg, g, edges, sizex, sizey);
    } else if (ts & LEFT != 0) || (hs & LEFT != 0) {
        // handle L-R specially
        if (ts & RIGHT != 0) || (hs & RIGHT != 0) {
            self_top(fg, g, edges, sizex, sizey);
        } else {
            self_left(fg, g, edges, sizex, sizey);
        }
    } else if ts & TOP != 0 {
        self_top(fg, g, edges, sizex, sizey);
    } else if ts & BOTTOM != 0 {
        self_bottom(fg, g, edges, sizex, sizey);
    } else {
        // C: assert(0)
        panic!("makeSelfEdge: unreachable port configuration");
    }
}

/// splines.c:984 `selfRight` — loop bulging to the right. 7-point control
/// arrays (2 Béziers), per-iteration offsets.
fn self_right(fg: &mut Fg, g: GId, edges: &[EId], stepx: f64, sizey: f64) {
    let e0 = edges[0];
    let n = fg.edges[e0].tail;
    let stepy = (sizey / 2.0 / edges.len() as f64).max(2.0);
    let np = fg.nodes[n].coord;
    let mut tp = fg.edges[e0].tail_port.p;
    tp.x += np.x;
    tp.y += np.y;
    let mut hp = fg.edges[e0].head_port.p;
    hp.x += np.x;
    hp.y += np.y;
    let mut sgn: f64 = if tp.y >= hp.y { 1.0 } else { -1.0 };
    let mut dx = fg.nodes[n].rw;
    let mut dy = 0.0f64;
    let point_pair = convert_sides_to_points(
        fg.edges[e0].tail_port.side as i32,
        fg.edges[e0].head_port.side as i32,
    );
    if point_pair == 32 || point_pair == 65 {
        if tp.y == hp.y {
            sgn = -sgn;
        }
    }
    let mut tx = dx.min(3.0 * (np.x + dx - tp.x));
    let mut hx = dx.min(3.0 * (np.x + dx - hp.x));
    for &e in edges {
        dx += stepx;
        tx += stepx;
        hx += stepx;
        dy += sgn * stepy;
        let mut points = [
            tp,
            PointF::new(tp.x + tx / 3.0, tp.y + dy),
            PointF::new(np.x + dx, tp.y + dy),
            PointF::new(np.x + dx, (tp.y + hp.y) / 2.0),
            PointF::new(np.x + dx, hp.y - dy),
            PointF::new(hp.x + hx / 3.0, hp.y - dy),
            hp,
        ];
        if let Some(li) = fg.edges[e].label {
            let width = if fg.graphs[g].rankdir.flip() {
                fg.labels[li].dimen.y
            } else {
                fg.labels[li].dimen.x
            };
            let pos = PointF::new(np.x + dx + width / 2.0, np.y);
            fg.labels[li].pos = pos;
            if width > stepx {
                dx += width - stepx;
            }
        }
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut points, sw, false);
    }
}

/// splines.c:1055 `selfLeft`.
fn self_left(fg: &mut Fg, g: GId, edges: &[EId], stepx: f64, sizey: f64) {
    let e0 = edges[0];
    let n = fg.edges[e0].tail;
    let stepy = (sizey / 2.0 / edges.len() as f64).max(2.0);
    let np = fg.nodes[n].coord;
    let mut tp = fg.edges[e0].tail_port.p;
    tp.x += np.x;
    tp.y += np.y;
    let mut hp = fg.edges[e0].head_port.p;
    hp.x += np.x;
    hp.y += np.y;
    let mut sgn: f64 = if tp.y >= hp.y { 1.0 } else { -1.0 };
    let mut dx = fg.nodes[n].lw;
    let mut dy = 0.0f64;
    let point_pair = convert_sides_to_points(
        fg.edges[e0].tail_port.side as i32,
        fg.edges[e0].head_port.side as i32,
    );
    if point_pair == 12 || point_pair == 67 {
        if tp.y == hp.y {
            sgn = -sgn;
        }
    }
    let mut tx = dx.min(3.0 * (tp.x + dx - np.x));
    let mut hx = dx.min(3.0 * (hp.x + dx - np.x));
    for &e in edges {
        dx += stepx;
        tx += stepx;
        hx += stepx;
        dy += sgn * stepy;
        let mut points = [
            tp,
            PointF::new(tp.x - tx / 3.0, tp.y + dy),
            PointF::new(np.x - dx, tp.y + dy),
            PointF::new(np.x - dx, (tp.y + hp.y) / 2.0),
            PointF::new(np.x - dx, hp.y - dy),
            PointF::new(hp.x - hx / 3.0, hp.y - dy),
            hp,
        ];
        if let Some(li) = fg.edges[e].label {
            let width = if fg.graphs[g].rankdir.flip() {
                fg.labels[li].dimen.y
            } else {
                fg.labels[li].dimen.x
            };
            let pos = PointF::new(np.x - dx - width / 2.0, np.y);
            fg.labels[li].pos = pos;
            if width > stepx {
                dx += width - stepx;
            }
        }
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut points, sw, false);
    }
}

/// splines.c:877 `selfTop`.
fn self_top(fg: &mut Fg, g: GId, edges: &[EId], sizex: f64, stepy: f64) {
    let e0 = edges[0];
    let n = fg.edges[e0].tail;
    let stepx = (sizex / 2.0 / edges.len() as f64).max(2.0);
    let np = fg.nodes[n].coord;
    let mut tp = fg.edges[e0].tail_port.p;
    tp.x += np.x;
    tp.y += np.y;
    let mut hp = fg.edges[e0].head_port.p;
    hp.x += np.x;
    hp.y += np.y;
    let sgn: f64 = if tp.x >= hp.x { 1.0 } else { -1.0 };
    let mut dy = fg.nodes[n].ht / 2.0;
    let mut dx = 0.0f64;
    let point_pair = convert_sides_to_points(
        fg.edges[e0].tail_port.side as i32,
        fg.edges[e0].head_port.side as i32,
    );
    let lw = fg.nodes[n].lw;
    let rw = fg.nodes[n].rw;
    match point_pair {
        15 => dx = sgn * (rw - (hp.x - np.x) + stepx),
        38 => dx = sgn * (lw - (np.x - hp.x) + stepx),
        41 => dx = sgn * (rw - (tp.x - np.x) + stepx),
        48 => dx = sgn * (rw - (tp.x - np.x) + stepx),
        14 | 37 | 47 | 51 | 57 | 58 => {
            dx = sgn * ((lw - (np.x - tp.x) + (rw - (hp.x - np.x))) / 3.0)
        }
        73 => dx = sgn * (lw - (np.x - tp.x) + stepx),
        83 => dx = sgn * (lw - (np.x - tp.x)),
        84 => dx = sgn * ((lw - (np.x - tp.x) + (rw - (hp.x - np.x))) / 2.0 + stepx),
        74 | 75 | 85 => {
            dx = sgn * ((lw - (np.x - tp.x) + (rw - (hp.x - np.x))) / 2.0 + 2.0 * stepx)
        }
        _ => {}
    }
    let mut ty = dy.min(3.0 * (np.y + dy - tp.y));
    let mut hy = dy.min(3.0 * (np.y + dy - hp.y));
    for &e in edges {
        dy += stepy;
        ty += stepy;
        hy += stepy;
        dx += sgn * stepx;
        let mut points = [
            tp,
            PointF::new(tp.x + dx, tp.y + ty / 3.0),
            PointF::new(tp.x + dx, np.y + dy),
            PointF::new((tp.x + hp.x) / 2.0, np.y + dy),
            PointF::new(hp.x - dx, np.y + dy),
            PointF::new(hp.x - dx, hp.y + hy / 3.0),
            hp,
        ];
        if let Some(li) = fg.edges[e].label {
            let height = if fg.graphs[g].rankdir.flip() {
                fg.labels[li].dimen.x
            } else {
                fg.labels[li].dimen.y
            };
            let pos = PointF::new(np.x, np.y + dy + height / 2.0);
            fg.labels[li].pos = pos;
            if height > stepy {
                dy += height - stepy;
            }
        }
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut points, sw, false);
    }
}

/// splines.c:807 `selfBottom`.
fn self_bottom(fg: &mut Fg, g: GId, edges: &[EId], sizex: f64, stepy: f64) {
    let e0 = edges[0];
    let n = fg.edges[e0].tail;
    let stepx = (sizex / 2.0 / edges.len() as f64).max(2.0);
    let np = fg.nodes[n].coord;
    let mut tp = fg.edges[e0].tail_port.p;
    tp.x += np.x;
    tp.y += np.y;
    let mut hp = fg.edges[e0].head_port.p;
    hp.x += np.x;
    hp.y += np.y;
    let mut sgn: f64 = if tp.x >= hp.x { 1.0 } else { -1.0 };
    let mut dy = fg.nodes[n].ht / 2.0;
    let mut dx = 0.0f64;
    let point_pair = convert_sides_to_points(
        fg.edges[e0].tail_port.side as i32,
        fg.edges[e0].head_port.side as i32,
    );
    if point_pair == 67 {
        sgn = -sgn;
    }
    let mut ty = dy.min(3.0 * (tp.y + dy - np.y));
    let mut hy = dy.min(3.0 * (hp.y + dy - np.y));
    for &e in edges {
        dy += stepy;
        ty += stepy;
        hy += stepy;
        dx += sgn * stepx;
        let mut points = [
            tp,
            PointF::new(tp.x + dx, tp.y - ty / 3.0),
            PointF::new(tp.x + dx, np.y - dy),
            PointF::new((tp.x + hp.x) / 2.0, np.y - dy),
            PointF::new(hp.x - dx, np.y - dy),
            PointF::new(hp.x - dx, hp.y - hy / 3.0),
            hp,
        ];
        if let Some(li) = fg.edges[e].label {
            let height = if fg.graphs[g].rankdir.flip() {
                fg.labels[li].dimen.x
            } else {
                fg.labels[li].dimen.y
            };
            let pos = PointF::new(np.x, np.y - dy - height / 2.0);
            fg.labels[li].pos = pos;
            if height > stepy {
                dy += height - stepy;
            }
        }
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut points, sw, false);
    }
}

/// splines.c:1137 `selfRightSpace` — the extra right-side room a self edge
/// needs. Used by position.c when inflating `ND_rw`; kept here so the two
/// ports agree on the formula.
#[allow(dead_code)]
fn self_right_space(fg: &Fg, g: GId, e: EId) -> f64 {
    let tp = &fg.edges[e].tail_port;
    let hp = &fg.edges[e].head_port;
    let cond = (!tp.defined && !hp.defined)
        || ((tp.side as i32 & LEFT == 0)
            && (hp.side as i32 & LEFT == 0)
            && (tp.side != hp.side || (tp.side as i32 & (TOP | BOTTOM) == 0)));
    if cond {
        let mut sw = SELF_EDGE_SIZE;
        if let Some(li) = fg.edges[e].label {
            let label_width = if fg.graphs[g].rankdir.flip() {
                fg.labels[li].dimen.y
            } else {
                fg.labels[li].dimen.x
            };
            sw += label_width;
        }
        sw
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// EDGETYPE_CURVED (common/routespl.c:913-1042)
// ---------------------------------------------------------------------------

/// routespl.c:919 `bend` — pull the waist of a straight 4-point spline
/// toward the cycle centroid (`dist/5`).
fn bend(spl: &mut [PointF; 4], centroid: PointF) {
    let midpt = spl[0].mid(spl[3]);
    let dist = ((spl[3].x - spl[0].x).powi(2) + (spl[3].y - spl[0].y).powi(2)).sqrt();
    let r = dist / 5.0;
    let vx = centroid.x - midpt.x;
    let vy = centroid.y - midpt.y;
    let mag_v = vx.hypot(vy);
    if mag_v == 0.0 {
        return;
    }
    let a = PointF::new(midpt.x - vx / mag_v * r, midpt.y - vy / mag_v * r);
    spl[1].x = a.x;
    spl[1].y = a.y;
    spl[2].x = a.x;
    spl[2].y = a.y;
}

/// routespl.c:889 `get_cycle_centroid` — centroid of the shortest directed
/// cycle of length ≥ 3 containing the edge, else the graph centroid.
fn get_cycle_centroid(fg: &Fg, g: GId, edge: EId) -> PointF {
    let cycles = find_all_cycles(fg, g);
    let cycle = find_shortest_cycle_with_edge(&cycles, fg, edge, 3);
    match cycle {
        None => get_centroid(fg, g),
        Some(c) => {
            let mut sum = PointF::ZERO;
            for &n in &c {
                sum = sum.add(fg.nodes[n].coord);
            }
            let cnt = c.len() as f64;
            PointF::new(sum.x / cnt, sum.y / cnt)
        }
    }
}

fn get_centroid(fg: &Fg, g: GId) -> PointF {
    let bb = &fg.graphs[g].bb;
    PointF::new((bb.ll.x + bb.ur.x) / 2.0, (bb.ll.y + bb.ur.y) / 2.0)
}

/// routespl.c `find_all_cycles` — enumerate simple directed cycles with a
/// DFS from every input node (C iterates `agfstnode`, i.e. real nodes only).
fn find_all_cycles(fg: &Fg, g: GId) -> Vec<Vec<NId>> {
    let mut cycles: Vec<Vec<NId>> = Vec::new();
    let mut visited: Vec<NId> = Vec::new();
    let starts: Vec<NId> = fg.graphs[g].nodes_order.clone();
    for start in starts {
        dfs_cycles(fg, start, &mut visited, start, &mut cycles);
    }
    cycles
}

fn dfs_cycles(fg: &Fg, search: NId, visited: &mut Vec<NId>, end: NId, cycles: &mut Vec<Vec<NId>>) {
    if visited.contains(&search) {
        if search == end && is_cycle_unique(cycles, visited) {
            cycles.push(visited.clone());
        }
        return;
    }
    visited.push(search);
    for &e in &fg.nodes[search].out {
        dfs_cycles(fg, fg.edges[e].head, visited, end, cycles);
    }
    visited.pop();
}

/// routespl.c `is_cycle_unique` — a cycle is a duplicate if some recorded
/// cycle of the same length has all its nodes inside `visited`.
fn is_cycle_unique(cycles: &[Vec<NId>], visited: &[NId]) -> bool {
    for cur in cycles {
        if cur.len() != visited.len() {
            continue;
        }
        let all_items_match = cur.iter().all(|c| visited.contains(c));
        if all_items_match {
            return false;
        }
    }
    true
}

/// routespl.c `find_shortest_cycle_with_edge`.
fn find_shortest_cycle_with_edge(
    cycles: &[Vec<NId>],
    fg: &Fg,
    edge: EId,
    min_size: usize,
) -> Option<Vec<NId>> {
    let start = fg.edges[edge].tail;
    let end = fg.edges[edge].head;
    let mut shortest: Option<&Vec<NId>> = None;
    for c in cycles {
        if c.len() < min_size {
            continue;
        }
        let better = match shortest {
            None => true,
            Some(s) => c.len() < s.len(),
        };
        if better && cycle_contains_edge(fg, c, start, end) {
            shortest = Some(c);
        }
    }
    shortest.cloned()
}

fn cycle_contains_edge(fg: &Fg, cycle: &[NId], start: NId, end: NId) -> bool {
    let n = cycle.len();
    for i in 0..n {
        let c_start = cycle[if i == 0 { n - 1 } else { i - 1 }];
        let c_end = cycle[i];
        let (t, h) = (fg.nodes[c_start].real, fg.nodes[c_end].real);
        // the fast nodes here are real input nodes (DFS starts from them);
        // compare by arena id
        let _ = (t, h);
        if c_start == start && c_end == end {
            return true;
        }
    }
    false
}

/// routespl.c:937 `makeStraightEdges` — the LINE/CURVED router.
fn make_straight_edges(fg: &mut Fg, g: GId, edge_list: &[EId], et: SplineType) {
    let curved = et == SplineType::Curved;
    let e = edge_list[0];
    let n = fg.edges[e].tail;
    let head = fg.edges[e].head;
    let mut dumb = [PointF::ZERO; 4];
    dumb[0] = fg.nodes[n].coord.add(fg.edges[e].tail_port.p);
    dumb[1] = dumb[0];
    dumb[3] = fg.nodes[head].coord.add(fg.edges[e].head_port.p);
    dumb[2] = dumb[3];

    if edge_list.len() == 1 || fg.concentrate {
        if curved {
            let c = get_cycle_centroid(fg, g, edge_list[0]);
            bend(&mut dumb, c);
        }
        let head = fg.edges[e].head;
        let sw = swap_ends_p(fg, e);
        clip_and_install(fg, e, head, &mut dumb, sw, false);
        // addEdgeLabels → makePortLabels: no-op without the
        // labelangle/labeldistance globals
        return;
    }

    let mut del = PointF::ZERO;
    if super::geom::approx_eqpt(dumb[0], dumb[3]) {
        // degenerate
        dumb[1] = dumb[0];
        dumb[2] = dumb[3];
    } else {
        let perp = PointF::new(dumb[0].y - dumb[3].y, dumb[3].x - dumb[0].x);
        let l_perp = perp.x.hypot(perp.y);
        let xstep = fg.graphs[0].nodesep; // GD_nodesep(g->root), integer in C
        let dx = xstep * (edge_list.len() as i32 - 1) / 2;
        dumb[1].x = dumb[0].x + dx as f64 * perp.x / l_perp;
        dumb[1].y = dumb[0].y + dx as f64 * perp.y / l_perp;
        dumb[2].x = dumb[3].x + dx as f64 * perp.x / l_perp;
        dumb[2].y = dumb[3].y + dx as f64 * perp.y / l_perp;
        del = PointF::new(
            -(xstep as f64) * perp.x / l_perp,
            -(xstep as f64) * perp.y / l_perp,
        );
    }

    for &e0 in edge_list {
        let mut dumber = [PointF::ZERO; 4];
        if fg.edges[e0].head == head {
            dumber = dumb;
        } else {
            for j in 0..4 {
                dumber[3 - j] = dumb[j];
            }
        }
        let head0 = fg.edges[e0].head;
        let sw = swap_ends_p(fg, e0);
        if et == SplineType::Pline {
            let pts = make_polyline(&dumber);
            clip_and_install(fg, e0, head0, &mut pts.clone(), sw, false);
        } else {
            clip_and_install(fg, e0, head0, &mut dumber, sw, false);
        }
        // march one nodesep toward the line
        dumb[1] = dumb[1].add(del);
        dumb[2] = dumb[2].add(del);
    }
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dotgen::model::{Rank, Splines};
    use crate::dotgen::rank::dot_rank;
    use crate::dotgen::{Measured, build};
    use crate::graph::parser::parse;

    /// build + dot_rank, then restore the state cleanup1 cleared: the
    /// ranking virtual edge object (index `n_orig_edges`) with
    /// `ED_to_orig = orig` still exists, only the elists were emptied.
    fn setup(dot: &str) -> Fg {
        let g = parse(dot).unwrap();
        let measured = Measured {
            node: vec![(54.0, 36.0); g.nodes.len()],
            label: vec![],
            edge_label: vec![],
            measure: None,
        };
        let mut fg = build(&g, &measured);
        dot_rank(&mut fg, 0);
        fg
    }

    fn mk_rank(v: Vec<NId>, ht1: f64, ht2: f64) -> Rank {
        Rank {
            n: v.len(),
            v,
            valid: true,
            ht1,
            ht2,
            ..Default::default()
        }
    }

    fn push_rank_edge(fg: &mut Fg, t: NId, h: NId) -> EId {
        // reuse the virtual edge created by class1 during dot_rank
        let ve = fg.n_orig_edges;
        assert_eq!(fg.edges[ve].tail, t);
        assert_eq!(fg.edges[ve].head, h);
        fg.nodes[t].out.push(ve);
        fg.nodes[h].in_.push(ve);
        ve
    }

    fn place(fg: &mut Fg, ranks: &[(Vec<NId>, f64, f64)], coords: &[(NId, f64, f64)]) {
        fg.graphs[0].rank = ranks
            .iter()
            .map(|(v, ht1, ht2)| mk_rank(v.clone(), *ht1, *ht2))
            .collect();
        // GD_rank has maxrank+2 slots in C; keep a spare
        fg.graphs[0].rank.push(Rank::default());
        for (i, &(n, x, y)) in coords.iter().enumerate() {
            fg.nodes[n].coord = PointF::new(x, y);
            fg.nodes[n].order = i as i32;
        }
        // position.c:248 keeps the real rw in ND_mval
        for n in fg.nodes.iter_mut() {
            n.mval = n.rw;
        }
    }

    fn flat_pts(spl: &Splines) -> Vec<PointF> {
        spl.list.iter().flat_map(|s| s.iter().copied()).collect()
    }

    /// (a) `a -> b`: the routed spline starts at a's border, ends at b's
    /// border, stays in the channel and is monotone in y.
    #[test]
    fn regular_edge_a_to_b() {
        let mut fg = setup("digraph { a -> b }");
        push_rank_edge(&mut fg, 0, 1);
        place(
            &mut fg,
            &[(vec![0], 18.0, 18.0), (vec![1], 18.0, 18.0)],
            &[(0, 27.0, 36.0), (1, 27.0, -36.0)],
        );
        dot_splines(&mut fg, 0).unwrap();

        let spl = fg.edges[0].spl.as_ref().expect("a->b must get a spline");
        let pts = flat_pts(spl);
        assert_eq!(pts.len(), 4, "single Bézier for a straight channel");
        // inside the channel x≈27
        for p in &pts {
            assert!((p.x - 27.0).abs() < 3.0, "x off channel: {:?}", p);
        }
        // monotone in y (control points move downward)
        for w in pts.windows(2) {
            assert!(w[0].y >= w[1].y, "not monotone: {:?}", pts);
        }
        // The head arrow occupies the last ~arrow-length of the channel, so
        // the *line* stops at the arrow's base, short of b's top border
        // (y = -36 + 18 = -18); `arrowEndClip` is what pulls it back.
        let last = *pts.last().unwrap();
        assert!(
            last.y > -18.0 && last.y < -18.0 + 14.0,
            "head end not at the arrow base: {:?}",
            pts
        );
        // every point lies between the two borders
        for p in &pts {
            assert!(
                p.y <= 18.0 + 2.0 && p.y >= -18.0 - 2.0,
                "outside band: {:?}",
                p
            );
        }
        // GD_bb was grown
        let bb = fg.graphs[0].bb;
        assert!(bb.contains(last), "bbox misses the spline: {:?}", bb);
    }

    /// (a2) With `dir=none` no arrow pulls the visible end back, so the
    /// spline's first point lands on a's border exactly.
    #[test]
    fn regular_edge_no_arrow_reaches_tail_border() {
        let mut fg = setup("digraph { a -> b [dir=none] }");
        push_rank_edge(&mut fg, 0, 1);
        place(
            &mut fg,
            &[(vec![0], 18.0, 18.0), (vec![1], 18.0, 18.0)],
            &[(0, 27.0, 36.0), (1, 27.0, -36.0)],
        );
        dot_splines(&mut fg, 0).unwrap();
        let spl = fg.edges[0].spl.as_ref().expect("spline");
        let pts = flat_pts(spl);
        let first = pts[0];
        let last = *pts.last().unwrap();
        assert!(
            (first.y - 18.0).abs() < 2.0 && (first.x - 27.0).abs() < 2.0,
            "tail end not on a's border: {:?}",
            first
        );
        assert!(
            (last.y - (-18.0)).abs() < 2.0,
            "head end not on b's border: {:?}",
            last
        );
    }

    /// (b) two parallel a->b edges fan out: the routed Béziers are distinct
    /// at their midpoint.
    #[test]
    fn multiedge_fanout() {
        let mut fg = setup("digraph { a -> b; a -> b; }");
        let ve = push_rank_edge(&mut fg, 0, 1);
        // class2 merges the second edge into the chain and puts it in other
        fg.edges[1].to_virt = Some(ve);
        fg.nodes[0].other.push(1);
        place(
            &mut fg,
            &[(vec![0], 18.0, 18.0), (vec![1], 18.0, 18.0)],
            &[(0, 27.0, 36.0), (1, 27.0, -36.0)],
        );
        dot_splines(&mut fg, 0).unwrap();

        let s0 = fg.edges[0].spl.as_ref().expect("first edge spline");
        let s1 = fg.edges[1].spl.as_ref().expect("second edge spline");
        let m0 = super::super::arrows::bezier_eval(&s0.list[0], 0.5, None, None);
        let m1 = super::super::arrows::bezier_eval(&s1.list[0], 0.5, None, None);
        assert!(
            (m1.x - m0.x).abs() >= 3.0,
            "fan-out midpoints not distinct: {:?} vs {:?}",
            m0,
            m1
        );
        // the fan is centered: first edge bends left, second right
        assert!(
            m0.x < 27.0 && m1.x > 27.0,
            "fan not centered: {:?} {:?}",
            m0,
            m1
        );
    }

    /// (c) a self loop routed by selfRight: a 7-point two-Bézier loop
    /// bulging to the right of the node.
    #[test]
    fn self_loop() {
        let mut fg = setup("digraph { b -> b }");
        // class2 puts self edges in ND_other
        fg.nodes[0].other.push(0);
        place(&mut fg, &[(vec![0], 18.0, 18.0)], &[(0, 27.0, 0.0)]);
        dot_splines(&mut fg, 0).unwrap();

        let spl = fg.edges[0].spl.as_ref().expect("self loop spline");
        assert_eq!(
            spl.list.len(),
            2,
            "selfRight emits 7 points = 2 Béziers (splines.c:984)"
        );
        let pts = flat_pts(spl);
        let max_x = pts.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        // the loop leaves the node's right border (x = 27 + 27 = 54)
        assert!(
            max_x > 52.0,
            "loop does not clear the node: max_x={}",
            max_x
        );
        // both endpoints at the node (center (27,0)); y bulges both ways
        let min_y = pts.iter().map(|p| p.y).fold(f64::MAX, f64::min);
        let max_y = pts.iter().map(|p| p.y).fold(f64::MIN, f64::max);
        assert!(max_y > 0.0 && min_y < 0.0, "loop not balanced: {:?}", pts);
    }

    /// edgecmp orders descending by type bits: self edges before flat before
    /// regular; within a type, by |rank diff| etc.
    #[test]
    fn edgecmp_orders_types_descending() {
        let mut fg = setup("digraph { a -> b; c -> c; }");
        // c=2. self edge (SELFNPEDGE=8) must sort before REGULAREDGE(1).
        fg.edges[0].tree_index = REGULAREDGE | FWDEDGE | MAINGRAPH;
        fg.edges[1].tree_index = SELFNPEDGE | FWDEDGE | AUXGRAPH;
        assert_eq!(edgecmp(&fg, 1, 0), Ordering::Less);
        assert_eq!(edgecmp(&fg, 0, 1), Ordering::Greater);
        // identical tree_index → the next key decides: |rank diff|, which is
        // 1 for a->b and 0 for the self loop c->c, so the a->b edge sorts
        // later (dotsplines.c:562-572).
        fg.edges[0].tree_index = REGULAREDGE | FWDEDGE | MAINGRAPH;
        fg.edges[1].tree_index = REGULAREDGE | FWDEDGE | MAINGRAPH;
        assert_eq!(edgecmp(&fg, 0, 1), Ordering::Greater);
    }

    /// portcmp semantics (dotsplines.c:128).
    #[test]
    fn portcmp_semantics() {
        let undef = Port::default();
        let mut def = Port::default();
        def.defined = true;
        def.p = PointF::new(3.0, 4.0);
        let mut def2 = def;
        def2.p.y = 5.0;
        assert_eq!(portcmp(&undef, &undef), 0);
        assert_eq!(portcmp(&undef, &def), -1);
        assert_eq!(portcmp(&def, &undef), 1);
        assert_eq!(portcmp(&def, &def2), -1);
        assert_eq!(portcmp(&def2, &def), 1);
        assert_eq!(portcmp(&def, &def), 0);
    }
}

#[cfg(test)]
fn shape_clip_dbg(
    fg: &crate::dotgen::model::Fg,
    n: usize,
    curve: &mut [PointF; 4],
    coord: PointF,
    left_inside: bool,
) {
    for p in curve.iter_mut() {
        p.x -= coord.x;
        p.y -= coord.y;
    }
    let mut sp = *curve;
    let inside = |p: PointF| crate::dotgen::splines::shape_inside(fg, n, p);
    super::arrows::bezier_clip(&mut sp, inside, left_inside);
    for p in sp.iter_mut() {
        p.x += coord.x;
        p.y += coord.y;
    }
    *curve = sp;
}

#[cfg(test)]
mod debug_tmp {
    use super::*;
    use crate::dotgen::model::Rank;
    use crate::dotgen::rank::dot_rank;
    use crate::dotgen::{Measured, build};
    use crate::graph::parser::parse;

    #[test]
    fn debug_route() {
        let g = parse("digraph { a -> b }").unwrap();
        let measured = Measured {
            label: Vec::new(),
            node: vec![(54.0, 36.0); 2],
            edge_label: vec![],
            measure: None,
        };
        let mut fg = build(&g, &measured);
        dot_rank(&mut fg, 0);
        let ve = fg.n_orig_edges;
        fg.nodes[0].out.push(ve);
        fg.nodes[1].in_.push(ve);
        fg.graphs[0].rank = vec![
            Rank {
                n: 1,
                v: vec![0],
                valid: true,
                ht1: 18.0,
                ht2: 18.0,
                ..Default::default()
            },
            Rank {
                n: 1,
                v: vec![1],
                valid: true,
                ht1: 18.0,
                ht2: 18.0,
                ..Default::default()
            },
        ];
        fg.nodes[0].coord = PointF::new(27.0, 36.0);
        fg.nodes[1].coord = PointF::new(27.0, -36.0);
        fg.nodes[0].order = 0;
        fg.nodes[1].order = 0;
        for n in fg.nodes.iter_mut() {
            n.mval = n.rw;
        }
        setflags(&mut fg, ve, REGULAREDGE, FWDEDGE, MAINGRAPH);

        // hand-build the same path make_regular_edge would build, and print
        // the RAW routesplines_ output
        let mut sp = SplineInfo {
            splinesep: 18.0 / 4.0,
            multisep: 18.0,
            ..Default::default()
        };
        sp.left_bound = -32.0;
        sp.right_bound = 86.0;
        sp.rank_box = vec![BoxF::default(); 2];
        let mut path = Path::default();
        let mut tend = PathEnd::default();
        tend.nb = BoxF {
            ll: PointF::new(-4.0, 18.0),
            ur: PointF::new(86.0, 54.0),
        };
        beginpath(&mut fg, 0, &mut path, ve, REGULAREDGE, &mut tend, false);
        println!("start.p = {:?}", path.start.p);
        let rb = rank_box(&mut fg, 0, &mut sp, 0);
        println!("rank_box = {:?}", rb);
        let mut hend = PathEnd::default();
        hend.nb = BoxF {
            ll: PointF::new(-4.0, -54.0),
            ur: PointF::new(86.0, -18.0),
        };
        endpath(&mut fg, 0, &mut path, ve, REGULAREDGE, &mut hend, false);
        println!("end.p = {:?}", path.end.p);
        for b in &tend.boxes {
            add_box(&mut path, *b);
        }
        add_box(&mut path, rb);
        for b in hend.boxes.iter().rev() {
            add_box(&mut path, *b);
        }
        println!("boxes = {:?}", path.boxes);
        let mut ps = routesplines_(&mut path, false).unwrap();
        println!("raw ps = {:?}", ps);
        // replicate splines.rs clip_and_install step by step
        let pn = ps.len();
        let tn = 0usize;
        let hn = 1usize;
        let mut start = 0usize;
        while start < pn - 4 {
            let p2 = PointF::new(
                ps[start + 3].x - fg.nodes[tn].coord.x,
                ps[start + 3].y - fg.nodes[tn].coord.y,
            );
            println!(
                "tail check ps[{}]={:?} rel={:?} inside={}",
                start + 3,
                ps[start + 3],
                p2,
                crate::dotgen::splines::shape_inside(&fg, tn, p2)
            );
            if !crate::dotgen::splines::shape_inside(&fg, tn, p2) {
                break;
            }
            start += 3;
        }
        println!("start = {}", start);
        let mut curve = [ps[start], ps[start + 1], ps[start + 2], ps[start + 3]];
        shape_clip_dbg(&fg, tn, &mut curve, fg.nodes[tn].coord, true);
        println!("after tail clip: {:?}", curve);
        ps[start..start + 4].copy_from_slice(&curve);
        let mut end = pn - 4;
        while end > 0 {
            let p2 = PointF::new(
                ps[end].x - fg.nodes[hn].coord.x,
                ps[end].y - fg.nodes[hn].coord.y,
            );
            if !crate::dotgen::splines::shape_inside(&fg, hn, p2) {
                break;
            }
            end -= 3;
        }
        println!("end = {}", end);
        let mut curve2 = [ps[end], ps[end + 1], ps[end + 2], ps[end + 3]];
        shape_clip_dbg(&fg, hn, &mut curve2, fg.nodes[hn].coord, false);
        println!("after head clip: {:?}", curve2);
        ps[end..end + 4].copy_from_slice(&curve2);
        println!("final ps = {:?}", ps);
    }
}
