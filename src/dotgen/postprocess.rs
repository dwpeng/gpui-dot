//! Postprocessing — a faithful port of `lib/common/postproc.c`
//! (`gv_postprocess` minus the exterior-label placement algorithm): cluster
//! label placement, root-label bounding-box growth, the per-rankdir
//! `Offset`, and `translate_drawing` (rotation + translation of every
//! coordinate into final drawing space).

use super::geom::{BoxF, PointF};
use super::model::{self, Fg, GId, RankDir};

/// `PAD(dimen)` (macros.h) — label padding: x += 4·GAP, y += 2·GAP, GAP=4.
fn pad(d: PointF) -> PointF {
    PointF::new(d.x + 16.0, d.y + 8.0)
}

/// `ccwrotatepf` (geom.c) — exact per-angle transforms.
fn ccwrotatepf(p: PointF, deg: i32) -> PointF {
    match deg {
        0 => p,
        90 => PointF::new(-p.y, p.x), // perp
        180 => PointF::new(p.x, -p.y),
        270 => PointF::new(p.y, p.x), // exch_xyf
        _ => unreachable!(),
    }
}

/// `place_graph_label` (postproc.c:731-758) — place cluster labels inside
/// their box, recursively. `LABEL_AT_TOP` is bit 1 (`const.h:176`); the label
/// is centred horizontally unless `labeljust` set LEFT/RIGHT. The offset band
/// comes from the cluster's `GD_border`, which `do_graph_label` filled and the
/// y-coordination reserved space for.
fn place_graph_label(fg: &mut Fg, g: GId) {
    for c in fg.graphs[g].clust.clone() {
        place_graph_label(fg, c);
        if fg.graphs[c].label.is_none() {
            continue;
        }
        let label_pos = fg.graphs[c].label_pos;
        let bb = fg.graphs[c].bb;
        let border = fg.graphs[c].border;
        let d = if label_pos & model::LABEL_AT_TOP != 0 {
            border[2] // TOP_IX
        } else {
            border[0] // BOTTOM_IX
        };
        let y = if label_pos & model::LABEL_AT_TOP != 0 {
            bb.ur.y - d.y / 2.0
        } else {
            bb.ll.y + d.y / 2.0
        };
        let x = if label_pos & model::LABEL_AT_RIGHT != 0 {
            bb.ur.x - d.x / 2.0
        } else if label_pos & model::LABEL_AT_LEFT != 0 {
            bb.ll.x + d.x / 2.0
        } else {
            (bb.ll.x + bb.ur.x) / 2.0
        };
        if let Some(l) = fg.graphs[c].label.as_mut() {
            l.pos = PointF::new(x, y);
        }
    }
}

/// `place_flip_graph_label` (postproc.c:696-728) — the rotated (LR/BT) case:
/// the band runs along x and the LEFT/RIGHT flags become the vertical ones.
fn place_flip_graph_label(fg: &mut Fg, g: GId) {
    for c in fg.graphs[g].clust.clone() {
        place_flip_graph_label(fg, c);
        if fg.graphs[c].label.is_none() {
            continue;
        }
        let label_pos = fg.graphs[c].label_pos;
        let bb = fg.graphs[c].bb;
        let border = fg.graphs[c].border;
        let d = if label_pos & model::LABEL_AT_TOP != 0 {
            border[1] // RIGHT_IX
        } else {
            border[3] // LEFT_IX
        };
        let x = if label_pos & model::LABEL_AT_TOP != 0 {
            bb.ur.x - d.x / 2.0
        } else {
            bb.ll.x + d.x / 2.0
        };
        let y = if label_pos & model::LABEL_AT_RIGHT != 0 {
            bb.ll.y + d.y / 2.0
        } else if label_pos & model::LABEL_AT_LEFT != 0 {
            bb.ur.y - d.y / 2.0
        } else {
            (bb.ll.y + bb.ur.y) / 2.0
        };
        if let Some(l) = fg.graphs[c].label.as_mut() {
            l.pos = PointF::new(x, y);
        }
    }
}

/// `gv_postprocess(g, allowTranslation)` — the essential flow. `state`
/// mirrors `State == GVSPLINES` (whether splines exist to translate).
pub fn gv_postprocess(fg: &mut Fg, allow_translation: bool) {
    let rankdir = fg.graphs[0].rankdir;
    let flip = rankdir.flip();
    let rank = match rankdir {
        RankDir::Tb => 0,
        RankDir::Lr => 1,
        RankDir::Bt => 2,
        RankDir::Rl => 3,
    };

    // cluster labels
    if flip {
        place_flip_graph_label(fg, 0);
    } else {
        place_graph_label(fg, 0);
    }

    // (addXLabels: exterior label placement — xlabels are not used by this
    // milestone's renderer; see postproc.c:400-483)

    // root graph label space (postproc.c:608-637)
    if let Some(label) = fg.graphs[0].label.as_ref() {
        let dimen = pad(label.dimen);
        let flip = flip;
        let label_pos = fg.graphs[0].label_pos;
        let bb = fg.graphs[0].bb;
        let mut bb = bb;
        if flip {
            if label_pos & 2 != 0 {
                bb.ur.x += dimen.y;
            } else {
                bb.ll.x -= dimen.y;
            }
            if dimen.x > bb.ur.y - bb.ll.y {
                let diff = (dimen.x - (bb.ur.y - bb.ll.y)) / 2.0;
                bb.ll.y -= diff;
                bb.ur.y += diff;
            }
        } else if label_pos & 2 != 0 {
            if rank == 0 {
                bb.ur.y += dimen.y;
            } else {
                bb.ll.y -= dimen.y;
            }
            if dimen.x > bb.ur.x - bb.ll.x {
                let diff = (dimen.x - (bb.ur.x - bb.ll.x)) / 2.0;
                bb.ll.x -= diff;
                bb.ur.x += diff;
            }
        } else {
            if rank == 0 {
                bb.ll.y -= dimen.y;
            } else {
                bb.ur.y += dimen.y;
            }
            if dimen.x > bb.ur.x - bb.ll.x {
                let diff = (dimen.x - (bb.ur.x - bb.ll.x)) / 2.0;
                bb.ll.x -= diff;
                bb.ur.x += diff;
            }
        }
        fg.graphs[0].bb = bb;
        // place_root_label happens AFTER translation in C
    }

    if !allow_translation {
        return;
    }

    // Offset per rankdir (postproc.c:639-655)
    let bb = fg.graphs[0].bb;
    let offset = match rankdir {
        RankDir::Tb => bb.ll,
        RankDir::Lr => PointF::new(-bb.ur.y, bb.ll.x),
        RankDir::Bt => PointF::new(bb.ll.x, -bb.ur.y),
        RankDir::Rl => PointF::new(bb.ll.y, bb.ll.x),
    };

    translate_drawing(fg, rank, offset);

    // place_root_label (postproc.c:207-227) — after translation
    if let Some(label) = fg.graphs[0].label.as_ref() {
        let d = label.dimen;
        let label_pos = fg.graphs[0].label_pos;
        let bb = fg.graphs[0].bb;
        let p = if label_pos & 1 != 0 {
            PointF::new(bb.ur.x - d.x / 2.0, 0.0)
        } else if label_pos & 4 != 0 {
            PointF::new(bb.ll.x + d.x / 2.0, 0.0)
        } else {
            PointF::new((bb.ll.x + bb.ur.x) / 2.0, 0.0)
        };
        let p = if label_pos & 2 != 0 {
            PointF::new(p.x, bb.ur.y - d.y / 2.0)
        } else {
            PointF::new(p.x, bb.ll.y + d.y / 2.0)
        };
        if let Some(l) = fg.graphs[0].label.as_mut() {
            l.pos = p;
        }
    }
}

fn map_point(p: PointF, rank: i32, offset: PointF) -> PointF {
    let p = ccwrotatepf(p, rank * 90);
    PointF::new(p.x - offset.x, p.y - offset.y)
}

/// `translate_drawing` — nodes (sizes reset unflipped), splines and the
/// bounding boxes.
fn translate_drawing(fg: &mut Fg, rank: i32, offset: PointF) {
    let shift = offset.x != 0.0 || offset.y != 0.0;
    if !shift && rank == 0 {
        return;
    }
    let flip = fg.graphs[0].rankdir.flip();
    for n in 0..fg.nodes.len() {
        if rank != 0 && flip {
            // `gv_nodesize(v, false)`: restore the unflipped dimensions from
            // the attribute size (ND_width/ND_height are never flipped), so a
            // rotated (LR/BT) layout paints boxes that match their labels.
            let w = fg.nodes[n].width * 72.0;
            let h = fg.nodes[n].height * 72.0;
            fg.nodes[n].lw = w / 2.0;
            fg.nodes[n].rw = w / 2.0;
            fg.nodes[n].ht = h;
        }
        let c = fg.nodes[n].coord;
        fg.nodes[n].coord = map_point(c, rank, offset);
    }
    // splines (State == GVSPLINES) — including the arrow tip anchors
    // (`bezier.sp`/`.ep`), which postproc.c's map_edge maps alongside the
    // control points; leaving them behind puts the arrowheads in a different
    // frame from the curves.
    for e in 0..fg.edges.len() {
        let (sflag, eflag) = (fg.edges[e].sflag, fg.edges[e].eflag);
        if let Some(spl) = fg.edges[e].spl.as_mut() {
            for seg in spl.list.iter_mut() {
                for p in seg.iter_mut() {
                    *p = map_point(*p, rank, offset);
                }
            }
            if sflag != 0 {
                spl.sp = map_point(spl.sp, rank, offset);
            }
            if eflag != 0 {
                spl.ep = map_point(spl.ep, rank, offset);
            }
        }
    }
    // Labels: postproc.c's map_edge maps every attached label position
    // (`ED_label`/`ND_label`/xlabels) alongside the splines. The records in
    // `fg.labels` are owned one-to-one by an edge or node, so mapping each
    // once is exactly C's per-object mapping.
    for l in fg.labels.iter_mut() {
        if l.pos.x != 0.0 || l.pos.y != 0.0 {
            l.pos = map_point(l.pos, rank, offset);
        }
    }
    translate_bb(fg, 0, rank, offset);
}

/// `translate_bb`.
fn translate_bb(fg: &mut Fg, g: GId, rank: i32, offset: PointF) {
    let bb = fg.graphs[g].bb;
    let new_bb = if rank == 1 || rank == 2 {
        // LR or BT: swap LL/UR corners before mapping
        BoxF {
            ll: map_point(PointF::new(bb.ll.x, bb.ur.y), rank, offset),
            ur: map_point(PointF::new(bb.ur.x, bb.ll.y), rank, offset),
        }
    } else {
        BoxF {
            ll: map_point(bb.ll, rank, offset),
            ur: map_point(bb.ur, rank, offset),
        }
    };
    fg.graphs[g].bb = new_bb;
    if let Some(label) = fg.graphs[g].label.as_mut() {
        label.pos = map_point(label.pos, rank, offset);
    }
    for c in fg.graphs[g].clust.clone() {
        translate_bb(fg, c, rank, offset);
    }
}
