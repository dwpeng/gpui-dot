//! `lib/dotgen/sameport.c` — merge edges that share a `samehead`/`sametail`
//! group onto one port.
//!
//! Run after `dot_position` (the endpoints' coordinates are final) and before
//! `dot_splines` (which reads the ports). `dot_sameports` groups each node's
//! incident edges by their `samehead` (edges whose *head* is the node) or
//! `sametail` (edges whose *tail* is the node) id, then [`sameport`] aims the
//! whole group at the average direction of its far endpoints, clips that ray
//! to the node outline and installs the resulting port on every edge of the
//! group — including the virtual edges of each original's chain, so the
//! routing and the arrows agree.

use super::geom::{PointF, round};
use super::model::{EId, Fg, GId, MC_SCALE, NId, NodeType, Port};
use super::splines::shape_clip0;

/// `dot_sameports` (sameport.c:43-88) — merge the ports of `samehead` /
/// `sametail` groups.
pub fn dot_sameports(fg: &mut Fg, g: GId) {
    // `agattr_text(g, AGEDGE, "samehead", NULL)` is non-NULL only once some
    // edge declares the attribute; with neither present the pass is a no-op.
    let has_samehead = fg.edges[..fg.n_orig_edges]
        .iter()
        .any(|e| e.samehead.is_some());
    let has_sametail = fg.edges[..fg.n_orig_edges]
        .iter()
        .any(|e| e.sametail.is_some());
    if !has_samehead && !has_sametail {
        return;
    }

    for &n in fg.graphs[g].nodes_order.clone().iter() {
        // `agfstedge(g, n)`: n's out-edges then its in-edges.
        let mut incident: Vec<EId> = fg.input_out[n].clone();
        incident.extend(fg.input_in[n].iter().copied());

        // groups in first-seen order, like C's `same_list_t`
        let mut samehead: Vec<(String, Vec<EId>)> = Vec::new();
        let mut sametail: Vec<(String, Vec<EId>)> = Vec::new();
        for &e in incident.iter() {
            // Don't support same* for loops.
            if fg.edges[e].head == fg.edges[e].tail {
                continue;
            }
            if fg.edges[e].head == n {
                if let Some(id) = fg.edges[e].samehead.clone().filter(|s| !s.is_empty()) {
                    push_group(&mut samehead, e, id);
                }
            } else if fg.edges[e].tail == n
                && let Some(id) = fg.edges[e].sametail.clone().filter(|s| !s.is_empty())
            {
                push_group(&mut sametail, e, id);
            }
        }
        for (_, group) in samehead.iter() {
            if group.len() > 1 {
                sameport(fg, g, n, group);
            }
        }
        for (_, group) in sametail.iter() {
            if group.len() > 1 {
                sameport(fg, g, n, group);
            }
        }
    }
}

/// `sameedge` (sameport.c:90-100) — append `e` to the group with `id`.
fn push_group(groups: &mut Vec<(String, Vec<EId>)>, e: EId, id: String) {
    for (existing, group) in groups.iter_mut() {
        if *existing == id {
            group.push(e);
            return;
        }
    }
    groups.push((id, vec![e]));
}

/// `sameport` (sameport.c:102-189) — aim every edge in `l` at one port on `u`.
fn sameport(fg: &mut Fg, g: GId, u: NId, l: &[EId]) {
    // Average direction of the far endpoints (unit vectors, so angles more
    // than PI apart do not cancel).
    let mut x = 0.0f64;
    let mut y = 0.0f64;
    for &e in l {
        let v = if fg.edges[e].head == u {
            fg.edges[e].tail
        } else {
            fg.edges[e].head
        };
        let x1 = fg.nodes[v].coord.x - fg.nodes[u].coord.x;
        let y1 = fg.nodes[v].coord.y - fg.nodes[u].coord.y;
        let r = (x1 * x1 + y1 * y1).sqrt();
        x += x1 / r;
        y += y1 / r;
    }
    let r = (x * x + y * y).sqrt();
    x /= r;
    y /= r;

    // Ray from the node centre far enough to leave the shape, clipped back
    // onto the outline.
    let mut x1 = fg.nodes[u].coord.x;
    let mut y1 = fg.nodes[u].coord.y;
    let r = f64::max(
        fg.nodes[u].lw + fg.nodes[u].rw,
        fg.nodes[u].ht + fg.graphs[g].ranksep as f64,
    );
    let x2 = x * r + fg.nodes[u].coord.x;
    let y2 = y * r + fg.nodes[u].coord.y;
    let mut curve = [
        PointF::new(x1, y1),
        PointF::new((2.0 * x1 + x2) / 3.0, (2.0 * y1 + y2) / 3.0),
        PointF::new((2.0 * x2 + x1) / 3.0, (2.0 * y2 + y1) / 3.0),
        PointF::new(x2, y2),
    ];
    shape_clip0(fg, u, &mut curve, fg.nodes[u].coord, true);
    x1 = curve[0].x - fg.nodes[u].coord.x;
    y1 = curve[0].y - fg.nodes[u].coord.y;

    let order = if fg.nodes[u].lw + fg.nodes[u].rw == 0.0 {
        (MC_SCALE / 2) as u8
    } else {
        (MC_SCALE as f64 * (fg.nodes[u].lw + round(x1)) / (fg.nodes[u].lw + fg.nodes[u].rw)) as u8
    };
    let prt = Port {
        p: PointF::new(round(x1), round(y1)),
        bp: super::geom::BoxF::default(),
        defined: true,
        clip: false,
        order,
        dyna: false,
        theta: 0.0,
        side: 0,
        constrained: false,
    };

    // Assign to every edge of the group and to all virtual edges of its
    // chain, walking both directions.
    for &e in l {
        let mut cur = Some(e);
        while let Some(c) = cur {
            let mut f = Some(c);
            while let Some(ff) = f {
                if fg.edges[ff].head == u {
                    fg.edges[ff].head_port = prt;
                }
                if fg.edges[ff].tail == u {
                    fg.edges[ff].tail_port = prt;
                }
                f = next_virtual(fg, ff, true);
            }
            f = Some(c);
            while let Some(ff) = f {
                if fg.edges[ff].head == u {
                    fg.edges[ff].head_port = prt;
                }
                if fg.edges[ff].tail == u {
                    fg.edges[ff].tail_port = prt;
                }
                f = next_virtual(fg, ff, false);
            }
            cur = fg.edges[c].to_virt;
        }
    }
    // "kinda pointless, because mincross is already done"
    fg.nodes[u].has_port = true;
}

/// The C loop `for (f = e; f; f = ED_edge_type(f) == VIRTUAL &&
/// ND_node_type(aghead(ff)) == VIRTUAL && ND_out(aghead(ff)).size == 1 ?
/// ND_out(aghead(ff)).list[0] : NULL)` for `head_side = true`, and its
/// in-edge mirror for `head_side = false`.
fn next_virtual(fg: &Fg, e: EId, head_side: bool) -> Option<EId> {
    if fg.edges[e].edge_type != super::model::EdgeType::Virtual {
        return None;
    }
    if head_side {
        let h = fg.edges[e].head;
        if fg.nodes[h].node_type == NodeType::Virtual && fg.nodes[h].out.len() == 1 {
            return Some(fg.nodes[h].out[0]);
        }
    } else {
        let t = fg.edges[e].tail;
        if fg.nodes[t].node_type == NodeType::Virtual && fg.nodes[t].in_.len() == 1 {
            return Some(fg.nodes[t].in_[0]);
        }
    }
    None
}
