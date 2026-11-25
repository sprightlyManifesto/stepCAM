//! Step milling: Face and Pocket operations on faces picked from a STEP model,
//! plus mill tabs that the cutter lifts over.
//!
//! * Climb milling throughout (M3 spindle): pocket walls are cut CCW, islands
//!   CW, facing rows run +X while stepping towards -Y.
//! * Pockets are roughed leaving `finish` on all walls, then a finishing pass
//!   at each depth level cuts the walls to size.
//! * Adaptive strategy = trochoidal loops along offset centre paths.

use crate::engrave::{self, EngraveOp};
use crate::gcode::fmt;
use crate::model::{Cutter, Engraver, Settings};
use cavalier_contours::polyline::*;
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

pub type P2 = [f64; 2];
pub type P3 = [f64; 3];

// ------------------------------------------------------------------ operations
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Strategy {
    Standard,
    Adaptive,
    /// Roughing: adaptive clearing round the part's outline (the band out to
    /// the stock), standard passes everywhere else.
    Mixed,
}

/// How a facing pass covers the area.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum FacingPattern {
    /// Rows all one way (+X, climb), lifting back between rows.
    #[default]
    Unidirectional,
    /// Rows alternately +X and -X, stepping over at depth.
    Zigzag,
    /// Loops following the area's edge, outside in.
    Spiral,
    /// Rectangles shrinking from the area's box, outside in.
    SquareSpiral,
}

impl FacingPattern {
    pub const ALL: [FacingPattern; 4] = [FacingPattern::Unidirectional, FacingPattern::Zigzag, FacingPattern::Spiral, FacingPattern::SquareSpiral];
    pub fn name(self) -> &'static str {
        match self {
            FacingPattern::Unidirectional => "Unidirectional",
            FacingPattern::Zigzag => "Zigzag",
            FacingPattern::Spiral => "Spiral",
            FacingPattern::SquareSpiral => "Square spiral",
        }
    }
    pub fn hover(self) -> &'static str {
        match self {
            FacingPattern::Unidirectional => "Rows all cut +X (climb), lifting and returning between rows",
            FacingPattern::Zigzag => "Rows alternately +X and -X, stepping over without lifting (half the rows are conventional milling)",
            FacingPattern::Spiral => "Loops following the area's edge, from the outside in, stepping in without lifting (climb)",
            FacingPattern::SquareSpiral => "Rectangles shrinking from the area's bounding box, outside in, stepping in without lifting (climb)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Entry {
    Helix,
    Zigzag,
    Plunge,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct FaceOp {
    pub face_id: usize,
    /// Z of the selected surface (final depth).
    pub z: f64,
    /// Part extents in XY (min x, min y, max x, max y): faced when there is
    /// no `outline` (older jobs).
    pub region: [f64; 4],
    /// The part's outside profile seen from above (holes filled). Facing
    /// clears everything above `z` inside it, offset outwards by half the
    /// tool diameter (+ `overlap`).
    pub outline: Vec<Vec<P2>>,
    pub cutter: Cutter,
    pub stepover_pct: f64,
    pub depth_per_pass: f64,
    /// Extra distance the tool centre may go past the outline + radius (mm);
    /// without an outline, how far the cutter edge runs past `region`.
    pub overlap: f64,
    pub feed: f64,
    pub speed: f64,
    pub start_z: f64,
    pub pattern: FacingPattern,
}

impl Default for FaceOp {
    fn default() -> Self {
        FaceOp {
            face_id: 0,
            z: 0.0,
            region: [0.0; 4],
            outline: vec![],
            cutter: Cutter::C3_175x17,
            stepover_pct: 50.0,
            depth_per_pass: 1.0,
            overlap: 0.0,
            feed: 500.0,
            speed: 12000.0,
            start_z: 0.0,
            pattern: FacingPattern::Unidirectional,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PocketOp {
    pub face_id: usize,
    pub z: f64,
    /// Outer boundary of the pocket floor (XY).
    pub outer: Vec<P2>,
    /// Inner loops of the floor face (only used when `keep_islands`).
    pub islands: Vec<Vec<P2>>,
    pub keep_islands: bool,
    pub cutter: Cutter,
    pub stepover_pct: f64,
    pub feed: f64,
    pub speed: f64,
    pub ramp_angle: f64,
    pub start_z: f64,
    pub depth_per_pass: f64,
    /// Material left on the walls by roughing and removed by the finish pass.
    pub finish: f64,
    pub strategy: Strategy,
    pub entry: Entry,
    /// Edge-defined pocket: the picked edge chain (empty = defined by `face_id`).
    pub edges: Vec<usize>,
    /// Highest Z of the edge chain when the op was made (to follow model moves).
    pub edge_z: f64,
    /// Cut the walls to size in a final contour pass (false: the contour pass
    /// still runs but leaves `finish` on the walls - used by roughing).
    pub finish_pass: bool,
    /// Sloped floor: triangles of the picked surface (empty = flat floor at `z`).
    /// Each level clears where the surface is below it, cutting the walls to
    /// that slice of the surface.
    pub floor: Vec<[P3; 3]>,
    /// Lowest Z of each island loop (sloped floors: an ignored island is
    /// cleared from the levels at or below its lowest point).
    pub islands_z: Vec<f64>,
    /// Roughing layers: no contour pass on the walls at all.
    #[serde(skip)]
    pub no_contour: bool,
    /// Roughing layers: the outer boundary is the clearing band's limit, not
    /// a wall, so the contour pass leaves it out.
    #[serde(skip)]
    pub open_outer: bool,
    /// Adaptive (roughing, mixed strategy): trochoids only on the outermost
    /// loop (round the outer boundary), standard passes for the rest.
    #[serde(skip)]
    pub adaptive_outer_only: bool,
}

impl Default for PocketOp {
    fn default() -> Self {
        PocketOp {
            face_id: 0,
            z: 0.0,
            outer: vec![],
            islands: vec![],
            keep_islands: false,
            cutter: Cutter::C3_175x17,
            stepover_pct: 40.0,
            feed: 500.0,
            speed: 12000.0,
            ramp_angle: 3.0,
            start_z: 0.0,
            depth_per_pass: 1.0,
            finish: 0.2,
            strategy: Strategy::Standard,
            entry: Entry::Helix,
            edges: vec![],
            edge_z: 0.0,
            finish_pass: true,
            floor: vec![],
            islands_z: vec![],
            no_contour: false,
            open_outer: false,
            adaptive_outer_only: false,
        }
    }
}

/// Outline of a model face seen from above, protected while roughing
/// below `z` (its highest point).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RoughFace {
    pub z: f64,
    pub outer: Vec<P2>,
    pub holes: Vec<Vec<P2>>,
    /// Sloped planar and curved faces: the surface triangles. Each layer
    /// then protects only the part of the face above it, so roughing follows
    /// the surface. Rebuilt from the model, not saved.
    #[serde(skip)]
    pub tris: Vec<[P3; 3]>,
    /// `tris` are a curved surface (not one plane).
    #[serde(skip)]
    pub curved: bool,
}

impl RoughFace {
    /// Lowest Z of the face (= `z` for flat and curved faces).
    pub fn low(&self) -> f64 {
        if self.tris.is_empty() {
            self.z
        } else {
            self.tris.iter().flatten().map(|p| p[2]).fold(f64::INFINITY, f64::min)
        }
    }
}

/// Layer-by-layer roughing: at every layer the material inside the part
/// outline is cleared (pockets, steps) and every exterior contour is
/// profiled - stock outside the part is not cleared. Leaves `allowance` on
/// walls and floors.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct RoughOp {
    pub cutter: Cutter,
    pub layer_height: f64,
    pub stepover_pct: f64,
    /// Stock to leave on walls and floors (mm).
    pub allowance: f64,
    pub feed: f64,
    pub speed: f64,
    pub ramp_angle: f64,
    pub entry: Entry,
    pub strategy: Strategy,
    pub start_z: f64,
    pub bottom_z: f64,
    /// Extra depth below Bottom Z for the last layer (e.g. to cut through).
    pub overcut_z: f64,
    /// Clear all material inside the part (true) or only profile the holes
    /// that go right through it (false).
    pub clear_voids: bool,
    /// After clearing each layer, a contour pass round the part's walls at
    /// tool radius + stock to leave (cleans up adaptive clearing's scallops).
    pub finish_layers: bool,
    /// Inside only: no band round the outside profile - the clearing stays
    /// within the part's outline (seen from above), so the outside walls are
    /// not cut, but the top and anything inside the outline (pockets, the
    /// fillets and chamfers on the outside edges) are.
    pub inside_only: bool,
    /// Pattern of the facing pass over stock taller than the part.
    pub facing: FacingPattern,
    /// Model faces seen from above - kept in sync with the model.
    pub faces: Vec<RoughFace>,
    /// Stock extents in XY (min x, min y, max x, max y) - kept in sync with
    /// the stock. Stock above the part top is faced over all of it.
    pub stock_region: [f64; 4],
}

impl Default for RoughOp {
    fn default() -> Self {
        RoughOp {
            cutter: Cutter::C3_175x17,
            layer_height: 1.0,
            stepover_pct: 40.0,
            allowance: 0.0,
            feed: 500.0,
            speed: 12000.0,
            ramp_angle: 3.0,
            entry: Entry::Helix,
            strategy: Strategy::Standard,
            start_z: 0.0,
            bottom_z: -1.0,
            overcut_z: 0.0,
            clear_voids: true,
            finish_layers: true,
            inside_only: false,
            facing: FacingPattern::Unidirectional,
            faces: vec![],
            stock_region: [0.0; 4],
        }
    }
}

impl RoughOp {
    /// Deepest Z actually cut: Bottom Z less the overcut.
    pub fn final_z(&self) -> f64 {
        self.bottom_z - self.overcut_z.max(0.0)
    }
}

/// Faces of a model as seen from above, for roughing. Planar faces facing
/// up count at their Z, sloped planar and tessellated curved faces follow
/// their surface (triangles), curved faces without a mesh count at their
/// highest point (conservative); vertical and down-facing planar faces are
/// skipped.
pub fn rough_faces(model: &crate::step::Model) -> Vec<RoughFace> {
    let xy = |l: &crate::step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<P2>>();
    let mut out = vec![];
    for f in &model.faces {
        if f.planar && f.normal[2] < 1e-6 {
            continue; // vertical or facing down
        }
        let Some(outer) = f.loops.iter().find(|l| l.outer).or(f.loops.first()) else { continue };
        let o = xy(outer);
        let curved = !f.planar && !f.tris.is_empty();
        let seen = if curved { f.tris.iter().map(|t| tri_area(t)).sum::<f64>() } else if o.len() < 3 { 0.0 } else { signed_area(&o).abs() };
        if seen < 1e-6 {
            continue; // seen edge-on (vertical walls, cylinders standing up)
        }
        let z = if f.is_up_horizontal() {
            f.z()
        } else {
            f.loops.iter().flat_map(|l| l.pts.iter()).chain(f.tris.iter().flatten()).map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max)
        };
        let holes = f.loops.iter().filter(|l| !std::ptr::eq(*l, outer)).map(xy).filter(|h| h.len() >= 3).collect();
        let tris = if !f.is_up_horizontal() { f.tris.clone() } else { vec![] };
        out.push(RoughFace { z, outer: o, holes, tris, curved });
    }
    out
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProfileSide {
    /// Tool centre on the edges.
    Center,
    /// Tool inside the closed loop (cuts the inside wall, CCW = climb).
    Inside,
    /// Tool outside the closed loop (cuts the outside wall, CW = climb).
    Outside,
}

impl ProfileSide {
    pub fn name(self) -> &'static str {
        match self {
            ProfileSide::Center => "on centre line",
            ProfileSide::Inside => "inside",
            ProfileSide::Outside => "outside",
        }
    }
}

/// Contour along a chain of edges, ramped down level by level.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ProfileOp {
    pub edges: Vec<usize>,
    /// Edge chain as an ordered polyline in XY.
    pub path: Vec<P2>,
    pub closed: bool,
    pub edge_z: f64,
    pub side: ProfileSide,
    /// Final depth.
    pub z: f64,
    pub cutter: Cutter,
    pub feed: f64,
    pub speed: f64,
    pub ramp_angle: f64,
    pub start_z: f64,
    pub depth_per_pass: f64,
}

impl Default for ProfileOp {
    fn default() -> Self {
        ProfileOp {
            edges: vec![],
            path: vec![],
            closed: false,
            edge_z: 0.0,
            side: ProfileSide::Center,
            z: -1.0,
            cutter: Cutter::C3_175x17,
            feed: 500.0,
            speed: 12000.0,
            ramp_angle: 3.0,
            start_z: 0.0,
            depth_per_pass: 1.0,
        }
    }
}

/// Rectangular tab extruded up from an edge: `width` along the edge tangent,
/// `depth` across it, `z_height` up from the edge point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MillTab {
    pub center: P3,
    /// Tangent direction in XY (radians).
    pub angle: f64,
    pub z_height: f64,
    pub width: f64,
    pub depth: f64,
    pub edges: Vec<usize>,
}

impl Default for MillTab {
    fn default() -> Self {
        MillTab { center: [0.0; 3], angle: 0.0, z_height: 1.5, width: 5.0, depth: 3.0, edges: vec![] }
    }
}

impl MillTab {
    pub fn top(&self) -> f64 {
        self.center[2] + self.z_height
    }
    /// The 4 XY corners of the tab rectangle.
    pub fn corners(&self) -> [P2; 4] {
        let (s, c) = self.angle.sin_cos();
        let (hw, hd) = (self.width / 2.0, self.depth / 2.0);
        let u = [c, s];
        let v = [-s, c];
        let p = |a: f64, b: f64| [self.center[0] + u[0] * a + v[0] * b, self.center[1] + u[1] * a + v[1] * b];
        [p(-hw, -hd), p(hw, -hd), p(hw, hd), p(-hw, hd)]
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum OpKind {
    Face(FaceOp),
    Pocket(PocketOp),
    Tab(MillTab),
    Profile(ProfileOp),
    Rough(RoughOp),
    /// Finishing after roughing: laps round the part's walls only, layer by
    /// layer (fine steps). Uses the roughing fields: layer_height = Z step,
    /// allowance = stock to leave; stepover / strategy / entry / clear voids /
    /// overcut are not used.
    Finish(RoughOp),
    Engrave(EngraveOp),
    Thread(ThreadOp),
    /// Areas every later op treats as filled in (left uncut).
    Omit(OmitOp),
}

/// Areas picked on the model that the ops after this one treat as filled in.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OmitOp {
    pub fills: Vec<Fill>,
}

/// One omitted area: the cavity round a picked face (a pocket, a hole, a
/// step), filled up to the lowest surface round its top.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Fill {
    /// The face picked.
    pub face: usize,
    /// The cavity's faces (shown in the 3D view).
    pub faces: Vec<usize>,
    /// Outline seen from above (CCW loops).
    pub outer: Vec<Vec<P2>>,
    pub top: f64,
    pub bottom: f64,
}

impl Fill {
    /// Point inside the filled outline.
    pub fn covers(&self, p: P2) -> bool {
        self.outer.iter().any(|l| inside(std::slice::from_ref(l), p))
    }
}

/// The area omitting face `fi` fills: the faces reached from it across
/// shared edges without going below its lowest point or through a flat
/// surface above it (those are the rims: the fill goes up to the lowest),
/// seen from above with the rims' holes round it.
pub fn omit_fill(model: &crate::step::Model, fi: usize) -> Result<Fill, String> {
    let f = model.faces.get(fi).ok_or("no such face")?;
    let zr = |g: &crate::step::Face| {
        g.loops.iter().flat_map(|l| l.pts.iter()).chain(g.tris.iter().flatten()).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])))
    };
    let down = |g: &crate::step::Face| g.planar && g.normal[2] < -1e-6;
    if down(f) {
        return Err("this surface faces down".into());
    }
    let lo0 = zr(f).0;
    // faces by edge
    let mut by_edge: std::collections::HashMap<usize, Vec<usize>> = Default::default();
    for (gi, g) in model.faces.iter().enumerate() {
        for e in g.loops.iter().flat_map(|l| l.edges.iter()) {
            by_edge.entry(*e).or_default().push(gi);
        }
    }
    let mut set = vec![fi];
    let mut rims: Vec<usize> = vec![];
    let mut k = 0;
    while k < set.len() {
        let g = &model.faces[set[k]];
        k += 1;
        for e in g.loops.iter().flat_map(|l| l.edges.iter()) {
            for &n in by_edge.get(e).into_iter().flatten() {
                if set.contains(&n) || rims.contains(&n) {
                    continue;
                }
                let h = &model.faces[n];
                if down(h) {
                    continue;
                }
                if h.is_up_horizontal() && h.z() > lo0 + 1e-6 {
                    rims.push(n);
                    continue;
                }
                if zr(h).0 < lo0 - 1e-6 {
                    continue; // goes below: the part's outside, or a hole further down
                }
                set.push(n);
            }
        }
    }
    let top = rims.iter().map(|&r| model.faces[r].z()).fold(f64::INFINITY, f64::min);
    if !top.is_finite() {
        return Err("nothing to fill: no higher flat surface round it".into());
    }
    // seen from above: the faces that are not vertical, and the rims' holes
    // round the cavity
    let xy = |l: &crate::step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<P2>>();
    let mut polys: Vec<geo::Polygon<f64>> = vec![];
    for &gi in &set {
        let g = &model.faces[gi];
        if g.planar {
            if g.normal[2] > 1e-6 {
                if let Some(o) = g.loops.iter().find(|l| l.outer).or(g.loops.first()) {
                    let o = xy(o);
                    if o.len() >= 3 && signed_area(&o).abs() > 1e-6 {
                        polys.push(to_geo(&ccw(o), &[]));
                    }
                }
            }
        } else if !g.tris.is_empty() {
            polys.extend(mesh_above(&g.tris, f64::NEG_INFINITY));
        }
    }
    for &ri in &rims {
        for l in model.faces[ri].loops.iter().filter(|l| !l.outer) {
            if l.edges.iter().any(|e| by_edge.get(e).map_or(false, |fs| fs.iter().any(|x| set.contains(x)))) {
                let o = xy(l);
                if o.len() >= 3 && signed_area(&o).abs() > 1e-6 {
                    polys.push(to_geo(&ccw(o), &[]));
                }
            }
        }
    }
    let u = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        use geo::Simplify;
        union_tree(polys).simplify(&MESH_SIMPLIFY)
    }))
    .map_err(|_| "could not work out the area's outline".to_string())?;
    let outer: Vec<Vec<P2>> = u.0.iter().map(|p| ccw(from_geo_ring(p.exterior()))).filter(|l| l.len() >= 3 && signed_area(l).abs() > 1e-3).collect();
    if outer.is_empty() {
        return Err("the area has no extent seen from above".into());
    }
    let bottom = set.iter().map(|&g| zr(&model.faces[g]).0).fold(f64::INFINITY, f64::min);
    Ok(Fill { face: fi, faces: set, outer, top, bottom })
}

fn ccw(mut l: Vec<P2>) -> Vec<P2> {
    if signed_area(&l) < 0.0 {
        l.reverse();
    }
    l
}

/// Areas filled in by the enabled Omit ops before op `i`.
pub fn fills_before(ops: &[StepOp], i: usize) -> Vec<Fill> {
    ops[..i.min(ops.len())]
        .iter()
        .filter(|o| o.enabled)
        .filter_map(|o| if let OpKind::Omit(m) = &o.kind { Some(m.fills.iter().cloned()) } else { None })
        .flatten()
        .collect()
}

/// Roughing / finishing with the omitted areas as solid up to their top.
fn rough_with_fills(op: &RoughOp, fills: &[Fill]) -> RoughOp {
    let mut op = op.clone();
    for f in fills {
        for l in &f.outer {
            op.faces.push(RoughFace { z: f.top, outer: l.clone(), holes: vec![], tris: vec![], curved: false });
        }
    }
    op
}

/// A pocket with the omitted areas inside it as islands. None: the pocket
/// lies in an omitted area (nothing to cut).
fn pocket_with_fills(op: &PocketOp, fills: &[Fill]) -> Option<PocketOp> {
    let mut op = op.clone();
    for f in fills.iter().filter(|f| f.top > op.z + 1e-6) {
        // the pocket inside the fill
        let probe = if op.outer.is_empty() { None } else { Some(op.outer.clone()) };
        if let Some(o) = probe {
            let c = o.iter().fold([0.0, 0.0], |a, p| [a[0] + p[0] / o.len() as f64, a[1] + p[1] / o.len() as f64]);
            let all_in = o.iter().all(|&p| f.covers(p) || f.outer.iter().any(|l| boundary_dist(std::slice::from_ref(l), p) < 0.01));
            if all_in && (f.covers(c) || o.len() < 3) {
                return None;
            }
            for l in &f.outer {
                let inner = l.iter().all(|&p| inside(std::slice::from_ref(&o), p) && boundary_dist(std::slice::from_ref(&o), p) > 1e-3);
                if inner && f.bottom < op.start_z {
                    if !op.keep_islands {
                        op.islands.clear();
                        op.islands_z.clear();
                        op.keep_islands = true;
                    }
                    op.islands.push(l.clone());
                    op.islands_z.push(f.bottom);
                }
            }
        }
    }
    Some(op)
}

/// Lift moves over an omitted area: wherever the cutter (radius `tool_r`)
/// would reach into it below its top, it goes over at the top instead.
fn apply_fill(moves: &[Mv], start: P3, f: &Fill, tool_r: f64) -> Vec<Mv> {
    // tool centres that would cut into the area (a hair less, so a pass
    // exactly a tool radius off its edge is left alone)
    let zone: Vec<Vec<P2>> = grow_loops(&f.outer, (tool_r - 1e-3).max(0.0));
    if zone.is_empty() {
        return moves.to_vec();
    }
    let ins = |p: P2| inside(&zone, p);
    let lift = |p: P3| if ins([p[0], p[1]]) { [p[0], p[1], p[2].max(f.top)] } else { p };
    let mut out = Vec::with_capacity(moves.len());
    let mut pos = start;
    for &m in moves {
        match m {
            Mv::Rapid(p) => {
                out.push(Mv::Rapid(lift(p)));
                pos = p;
            }
            Mv::Feed(p, fd) => {
                let a = pos;
                if a[2] < f.top || p[2] < f.top {
                    // where the segment crosses the zone's edge
                    let mut ts: Vec<f64> = vec![];
                    for l in &zone {
                        for k in 0..l.len() {
                            let (c, d) = (l[k], l[(k + 1) % l.len()]);
                            let (rx, ry, sx, sy) = (p[0] - a[0], p[1] - a[1], d[0] - c[0], d[1] - c[1]);
                            let den = rx * sy - ry * sx;
                            if den.abs() < 1e-15 {
                                continue;
                            }
                            let t = ((c[0] - a[0]) * sy - (c[1] - a[1]) * sx) / den;
                            let u = ((c[0] - a[0]) * ry - (c[1] - a[1]) * rx) / den;
                            if t > 1e-9 && t < 1.0 - 1e-9 && (-1e-12..=1.0 + 1e-12).contains(&u) {
                                ts.push(t);
                            }
                        }
                    }
                    ts.sort_by(|x, y| x.total_cmp(y));
                    let at = |t: f64| [a[0] + (p[0] - a[0]) * t, a[1] + (p[1] - a[1]) * t, a[2] + (p[2] - a[2]) * t];
                    let mut was_in = ins([a[0], a[1]]);
                    for t in ts {
                        let q = at(t);
                        let lifted = [q[0], q[1], q[2].max(f.top)];
                        if was_in {
                            out.push(Mv::Feed(lifted, fd));
                            out.push(Mv::Feed(q, fd));
                        } else {
                            out.push(Mv::Feed(q, fd));
                            out.push(Mv::Feed(lifted, fd));
                        }
                        was_in = !was_in;
                    }
                }
                out.push(Mv::Feed(lift(p), fd));
                pos = p;
            }
        }
    }
    // zero-length moves left by the lifting
    out.dedup_by(|b, a| matches!((a, b), (Mv::Feed(p, _), Mv::Feed(q, _)) if p == q));
    out
}

/// Thread milling of holes picked on the model: the same bore + thread
/// cycle and fields as the old Thread Milling tab's groups.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThreadOp {
    /// Cylindrical hole faces the locations were taken from.
    pub faces: Vec<usize>,
    pub group: crate::model::ThreadGroup,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StepOp {
    pub enabled: bool,
    pub kind: OpKind,
}

impl StepOp {
    pub fn label(&self) -> String {
        match &self.kind {
            OpKind::Face(f) => format!("Face to Z{}", fmt(f.z)),
            OpKind::Pocket(p) => format!(
                "Pocket{} to Z{} ({})",
                if !p.edges.is_empty() {
                    " (edges)"
                } else if !p.floor.is_empty() {
                    " (sloped)"
                } else {
                    ""
                },
                fmt(p.z),
                match p.strategy {
                    Strategy::Standard => "standard",
                    Strategy::Adaptive => "adaptive",
                    Strategy::Mixed => "mixed",
                }
            ),
            OpKind::Tab(t) => format!("Mill tab at X{} Y{} Z{}", fmt(t.center[0]), fmt(t.center[1]), fmt(t.center[2])),
            OpKind::Omit(m) => format!("Omit {} area{}", m.fills.len(), if m.fills.len() == 1 { "" } else { "s" }),
            OpKind::Profile(p) => format!("Profile {} to Z{}", p.side.name(), fmt(p.z)),
            OpKind::Rough(r) => format!("Rough to Z{} ({} mm layers)", fmt(r.final_z()), fmt(r.layer_height)),
            OpKind::Finish(r) => format!("Finish walls to Z{} ({} mm steps)", fmt(r.bottom_z), fmt(r.layer_height)),
            OpKind::Engrave(e) => {
                let first = e.text.lines().next().unwrap_or("");
                let short: String = first.chars().take(20).collect();
                let more = if first.chars().count() > 20 || e.text.trim_end().lines().count() > 1 { "…" } else { "" };
                format!("Engrave \"{short}{more}\" {} deep at Z{}", fmt(e.depth), fmt(e.z))
            }
            OpKind::Thread(t) => {
                let n = t.group.parse_locations().map(|l| l.len()).unwrap_or(0);
                format!("Thread {} x{} - {} hole(s) from Z{}", t.group.thread.name(), fmt(t.group.thread.pitch()), n, fmt(t.group.start_z))
            }
        }
    }
    /// Surface the op was made from (None for edge-based ops).
    pub fn face_id(&self) -> Option<usize> {
        match &self.kind {
            OpKind::Face(f) => Some(f.face_id),
            OpKind::Pocket(p) if p.edges.is_empty() => Some(p.face_id),
            OpKind::Engrave(e) => Some(e.face_id),
            _ => None,
        }
    }
    pub fn cutter_mut(&mut self) -> Option<&mut Cutter> {
        match &mut self.kind {
            OpKind::Face(f) => Some(&mut f.cutter),
            OpKind::Pocket(p) => Some(&mut p.cutter),
            OpKind::Profile(p) => Some(&mut p.cutter),
            OpKind::Rough(r) | OpKind::Finish(r) => Some(&mut r.cutter),
            OpKind::Engrave(e) => match &mut e.tool {
                engrave::EngraveTool::EndMill(c) => Some(c),
                engrave::EngraveTool::Bit(_) => None,
            },
            OpKind::Thread(t) => Some(&mut t.group.cutter),
            OpKind::Tab(_) | OpKind::Omit(_) => None,
        }
    }
    /// Faces to highlight for the op.
    pub fn faces(&self) -> Vec<usize> {
        match &self.kind {
            OpKind::Thread(t) => t.faces.clone(),
            OpKind::Omit(m) => m.fills.iter().flat_map(|f| f.faces.iter().copied()).collect(),
            _ => self.face_id().into_iter().collect(),
        }
    }
    pub fn engraver_mut(&mut self) -> Option<&mut Engraver> {
        match &mut self.kind {
            OpKind::Engrave(e) => match &mut e.tool {
                engrave::EngraveTool::Bit(b) => Some(b),
                engrave::EngraveTool::EndMill(_) => None,
            },
            _ => None,
        }
    }
    /// Edges the op was made from (empty for surface-based ops).
    pub fn edges(&self) -> &[usize] {
        match &self.kind {
            OpKind::Pocket(p) => &p.edges,
            OpKind::Tab(t) => &t.edges,
            OpKind::Profile(p) => &p.edges,
            OpKind::Face(_) | OpKind::Rough(_) | OpKind::Finish(_) | OpKind::Engrave(_) | OpKind::Thread(_) | OpKind::Omit(_) => &[],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StepJob {
    pub step_path: String,
    /// Where the imported model sits in machine coordinates.
    pub placement: crate::step::Placement,
    pub stock: Stock,
    /// Rapid plane height above the stock or part top, whichever is higher.
    pub rapid_above: f64,
    /// Program zero: every X / Y / Z in the G-code is relative to it.
    pub origin: Origin,
    pub ops: Vec<StepOp>,
}

impl Default for StepJob {
    fn default() -> Self {
        StepJob { step_path: String::new(), placement: Default::default(), stock: Default::default(), rapid_above: 10.0, origin: Default::default(), ops: vec![] }
    }
}

/// Where program X0 Y0 is: the model's own zero, a point of the stock box
/// (seen from above: bottom = -Y, left = -X) or a typed / picked position.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OriginXY {
    #[default]
    Model,
    Center,
    BottomLeft,
    BottomRight,
    TopLeft,
    TopRight,
    Custom,
}

impl OriginXY {
    pub const ALL: [OriginXY; 7] = [OriginXY::Model, OriginXY::Center, OriginXY::BottomLeft, OriginXY::BottomRight, OriginXY::TopLeft, OriginXY::TopRight, OriginXY::Custom];

    pub fn name(self) -> &'static str {
        match self {
            OriginXY::Model => "Model 0",
            OriginXY::Center => "Stock centre",
            OriginXY::BottomLeft => "Stock bottom left",
            OriginXY::BottomRight => "Stock bottom right",
            OriginXY::TopLeft => "Stock top left",
            OriginXY::TopRight => "Stock top right",
            OriginXY::Custom => "Custom",
        }
    }
}

/// Where program Z0 is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OriginZ {
    #[default]
    Model,
    StockTop,
    StockBase,
    Custom,
}

impl OriginZ {
    pub const ALL: [OriginZ; 4] = [OriginZ::Model, OriginZ::StockTop, OriginZ::StockBase, OriginZ::Custom];

    pub fn name(self) -> &'static str {
        match self {
            OriginZ::Model => "Model 0",
            OriginZ::StockTop => "Stock top",
            OriginZ::StockBase => "Stock base",
            OriginZ::Custom => "Custom",
        }
    }
}

/// Program zero. `at` is where it sits in model (machine view) coordinates:
/// the generated G-code is `at` subtracted from every position, written out
/// directly (no G10 / G53 / work offsets). Presets follow the stock via
/// `resolve`; Custom axes keep their value.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Origin {
    pub xy: OriginXY,
    pub z: OriginZ,
    pub at: P3,
}

impl Origin {
    /// Update `at` for preset modes from the stock box.
    pub fn resolve(&mut self, stock_min: P3, stock_max: P3) {
        let (lo, hi) = (stock_min, stock_max);
        let mid = |k: usize| (lo[k] + hi[k]) / 2.0;
        let xy = match self.xy {
            OriginXY::Model => Some([0.0, 0.0]),
            OriginXY::Center => Some([mid(0), mid(1)]),
            OriginXY::BottomLeft => Some([lo[0], lo[1]]),
            OriginXY::BottomRight => Some([hi[0], lo[1]]),
            OriginXY::TopLeft => Some([lo[0], hi[1]]),
            OriginXY::TopRight => Some([hi[0], hi[1]]),
            OriginXY::Custom => None,
        };
        if let Some([x, y]) = xy {
            self.at[0] = x;
            self.at[1] = y;
        }
        match self.z {
            OriginZ::Model => self.at[2] = 0.0,
            OriginZ::StockTop => self.at[2] = hi[2],
            OriginZ::StockBase => self.at[2] = lo[2],
            OriginZ::Custom => {}
        }
    }

    /// Put program zero at these coordinates (None = leave that axis);
    /// the axes set become Custom.
    pub fn set(&mut self, x: Option<f64>, y: Option<f64>, z: Option<f64>) {
        if x.is_some() || y.is_some() {
            self.xy = OriginXY::Custom;
        }
        for (k, v) in [x, y, z].into_iter().enumerate() {
            if let Some(v) = v {
                self.at[k] = v;
            }
        }
        if z.is_some() {
            self.z = OriginZ::Custom;
        }
    }
}

/// Rewrite every X / Y / Z word (outside comments) as relative to `origin`.
/// I / J are relative to the arc start already and stay as they are.
pub fn shift_code(code: &str, origin: P3) -> String {
    if origin == [0.0; 3] {
        return code.to_string();
    }
    let mut out = String::with_capacity(code.len() + code.len() / 8);
    for line in code.lines() {
        // split off the comment: ( ... ) or ; ...
        let cut = line.find(|c| c == '(' || c == ';').unwrap_or(line.len());
        let (words, comment) = line.split_at(cut);
        let mut first = true;
        for w in words.split_whitespace() {
            if !first {
                out.push(' ');
            }
            first = false;
            let mut ch = w.chars();
            let k = match ch.next().map(|c| c.to_ascii_uppercase()) {
                Some('X') => Some(0),
                Some('Y') => Some(1),
                Some('Z') => Some(2),
                _ => None,
            };
            match (k, ch.as_str().parse::<f64>()) {
                (Some(k), Ok(v)) => {
                    out.push_str(&w[..1]);
                    out.push_str(&fmt(v - origin[k]));
                }
                _ => out.push_str(w),
            }
        }
        if !comment.is_empty() {
            if !first {
                out.push(' ');
            }
            out.push_str(comment);
        }
        out.push('\n');
    }
    out
}

/// Raw material block. With `offset` the X/Y/Z values are added to the part's
/// bounding box (X/Y split evenly on both sides, Z as set by `z_anchor`);
/// without it they are the absolute stock size, centred on the part in XY.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Stock {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub offset: bool,
    /// Where the part sits in the stock's height.
    pub z_anchor: ZAnchor,
}

impl Default for Stock {
    fn default() -> Self {
        Stock { x: 0.0, y: 0.0, z: 0.0, offset: true, z_anchor: ZAnchor::Bottom }
    }
}

/// Where the part sits in the stock's height: Top = part top at the stock
/// top (extra stock below), Middle = extra split evenly, Bottom = part
/// bottom on the stock base (extra stock on top).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZAnchor {
    Top,
    Middle,
    #[default]
    Bottom,
}

impl ZAnchor {
    pub const ALL: [ZAnchor; 3] = [ZAnchor::Top, ZAnchor::Middle, ZAnchor::Bottom];

    pub fn name(self) -> &'static str {
        match self {
            ZAnchor::Top => "Top",
            ZAnchor::Middle => "Middle",
            ZAnchor::Bottom => "Bottom",
        }
    }
}

impl Stock {
    /// Stock box (min, max) around a part with bounding box `min`..`max`.
    pub fn bounds(&self, min: P3, max: P3) -> (P3, P3) {
        let size = |k: usize, v: f64| if self.offset { max[k] - min[k] + v } else { v };
        let (sx, sy, sz) = (size(0, self.x), size(1, self.y), size(2, self.z));
        let c = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0];
        let z0 = match self.z_anchor {
            ZAnchor::Bottom => min[2],
            ZAnchor::Top => max[2] - sz,
            ZAnchor::Middle => (min[2] + max[2]) / 2.0 - sz / 2.0,
        };
        ([c[0] - sx / 2.0, c[1] - sy / 2.0, z0], [c[0] + sx / 2.0, c[1] + sy / 2.0, z0 + sz])
    }

    /// Problems with the stock compared with the part.
    pub fn issues(&self, min: P3, max: P3) -> Vec<String> {
        let (smin, smax) = self.bounds(min, max);
        let mut e = vec![];
        for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
            let (s, p) = (smax[k] - smin[k], max[k] - min[k]);
            if s < p - 1e-6 {
                e.push(format!("stock {a} {} is smaller than the part {}", fmt(s), fmt(p)));
            }
        }
        e
    }
}

// ------------------------------------------------------------------ 2D geometry
fn signed_area(p: &[P2]) -> f64 {
    let mut a = 0.0;
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        a += q[0] * r[1] - r[0] * q[1];
    }
    a / 2.0
}

fn to_pline(p: &[P2], ccw: bool) -> Polyline<f64> {
    let mut pts: Vec<P2> = p.to_vec();
    pts.dedup_by(|a, b| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9);
    if pts.len() > 1 && (pts[0][0] - pts[pts.len() - 1][0]).hypot(pts[0][1] - pts[pts.len() - 1][1]) < 1e-9 {
        pts.pop();
    }
    if (signed_area(&pts) > 0.0) != ccw {
        pts.reverse();
    }
    let mut pl = Polyline::with_capacity(pts.len(), true);
    for q in pts {
        pl.add(q[0], q[1], 0.0);
    }
    pl
}

fn from_pline(pl: &Polyline<f64>) -> Vec<P2> {
    // chords sit inside the arcs: keep the error well under the clearances
    let lines = pl.arcs_to_approx_lines(0.001).unwrap_or_else(|| pl.clone());
    lines.iter_vertexes().map(|v| [v.x, v.y]).collect()
}

/// Offset a region (CCW outer + CW islands) inward by `d`. Returns all
/// resulting loops: CCW outer loops and CW island loops.
pub fn offset_region(outer: &[P2], islands: &[Vec<P2>], d: f64) -> Vec<Vec<P2>> {
    if outer.len() < 3 {
        return vec![];
    }
    let mut plines = vec![to_pline(outer, true)];
    for isl in islands.iter().filter(|i| i.len() >= 3) {
        plines.push(to_pline(isl, false));
    }
    let shape = Shape::from_plines(plines);
    // degenerate input can make the offsetter panic: treat as nothing left
    let Ok(off) = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| shape.parallel_offset(d, &ShapeOffsetOptions::new()))) else {
        return vec![];
    };
    let mut out: Vec<Vec<P2>> = off.ccw_plines.iter().map(|ip| from_pline(&ip.polyline)).collect();
    if out.is_empty() {
        return vec![];
    }
    out.extend(off.cw_plines.iter().map(|ip| from_pline(&ip.polyline)));
    out.retain(|l| l.len() >= 3);
    out
}

/// Even-odd point in region test over a set of closed loops.
fn inside(loops: &[Vec<P2>], p: P2) -> bool {
    let mut c = false;
    for l in loops {
        for i in 0..l.len() {
            let (a, b) = (l[i], l[(i + 1) % l.len()]);
            if (a[1] > p[1]) != (b[1] > p[1]) {
                let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                if p[0] < x {
                    c = !c;
                }
            }
        }
    }
    c
}

fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-18 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

fn boundary_dist(loops: &[Vec<P2>], p: P2) -> f64 {
    let mut d = f64::INFINITY;
    for l in loops {
        for i in 0..l.len() {
            d = d.min(seg_dist(p, l[i], l[(i + 1) % l.len()]));
        }
    }
    d
}

fn d2(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Rotate a closed loop to start at the vertex nearest `p`, and close it.
/// The whole of segment p-q lies in one of the regions (each a set of loops,
/// even-odd), allowing a hair on the boundary.
fn seg_in(regions: &[Vec<Vec<P2>>], p: P2, q: P2) -> bool {
    seg_in_tol(regions, p, q, SEG_SLACK)
}

/// `seg_in` allowing `tol` on the boundary.
fn seg_in_tol(regions: &[Vec<Vec<P2>>], p: P2, q: P2, tol: f64) -> bool {
    let n = ((d2(p, q) / 0.25).ceil() as usize).max(1);
    (0..=n).all(|k| {
        let t = k as f64 / n as f64;
        let x = [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
        regions.iter().any(|l| inside(l, x) || boundary_dist(l, x) < tol)
    })
}

/// Boundary tolerance of roughing's hop check (on the exact tool-centre area).
const HOP_TOL: f64 = 1e-3;

/// Boundary slack in `seg_in` (points on the boundary count as inside).
const SEG_SLACK: f64 = 0.05;

/// Distance from `p` to the nearest vertex of `l`.
fn near_dist(l: &[P2], p: P2) -> f64 {
    l.iter().map(|q| d2(*q, p)).fold(f64::INFINITY, f64::min)
}

/// Where a list of moves leaves the tool (XY), or `or` for none.
fn end_xy(moves: &[Mv], or: P2) -> P2 {
    match moves.last() {
        Some(Mv::Rapid(p)) | Some(Mv::Feed(p, _)) => [p[0], p[1]],
        None => or,
    }
}

/// Order items greedily: always the nearest one to where the last ended.
/// `dist(item, here)` is the travel to start it, `end(item, here)` where it
/// leaves the tool.
fn nearest_first<T>(mut items: Vec<T>, mut here: P2, dist: impl Fn(&T, P2) -> f64, end: impl Fn(&T, P2) -> P2) -> Vec<T> {
    let mut out = Vec::with_capacity(items.len());
    while !items.is_empty() {
        let k = (0..items.len()).min_by(|&a, &b| dist(&items[a], here).total_cmp(&dist(&items[b], here))).unwrap();
        let it = items.swap_remove(k);
        here = end(&it, here);
        out.push(it);
    }
    out
}

fn start_near(l: &[P2], p: P2) -> Vec<P2> {
    let i = (0..l.len()).min_by(|&a, &b| d2(l[a], p).total_cmp(&d2(l[b], p))).unwrap_or(0);
    let mut v: Vec<P2> = l[i..].iter().chain(l[..i].iter()).copied().collect();
    v.push(v[0]);
    v
}

fn path_len(p: &[P2]) -> f64 {
    p.windows(2).map(|w| d2(w[0], w[1])).sum()
}

fn point_at(p: &[P2], mut s: f64) -> P2 {
    for w in p.windows(2) {
        let l = d2(w[0], w[1]);
        if s <= l || l == 0.0 && s <= 0.0 {
            let t = if l > 0.0 { s / l } else { 0.0 };
            return [w[0][0] + (w[1][0] - w[0][0]) * t, w[0][1] + (w[1][1] - w[0][1]) * t];
        }
        s -= l;
    }
    *p.last().unwrap()
}

/// Arc-length stations along `p` from s0 to s1 (either direction): the
/// path's own vertices strictly between them, then s1. Following these
/// keeps a move on the path instead of cutting across its corners.
fn stations(p: &[P2], s0: f64, s1: f64) -> Vec<f64> {
    let (lo, hi) = (s0.min(s1), s0.max(s1));
    let mut acc = 0.0;
    let mut mid: Vec<f64> = vec![];
    for w in p.windows(2) {
        acc += d2(w[0], w[1]);
        if acc > lo + 1e-9 && acc < hi - 1e-9 {
            mid.push(acc);
        }
    }
    if s1 < s0 {
        mid.reverse();
    }
    mid.push(s1);
    mid
}

/// How a clearing loop is cut.
#[derive(Clone, Debug)]
enum LoopCut {
    /// Followed as it is.
    Plain,
    /// Trochoidal loops of a fixed radius along it.
    Trochoid,
    /// Trochoidal loops along it whose radius (one per vertex) spans the
    /// area's width there.
    Varying(Vec<f64>),
}

/// Closest point on a closed loop to `p`.
fn closest_on_loop(l: &[P2], p: P2) -> P2 {
    let mut best = (f64::INFINITY, l[0]);
    for i in 0..l.len() {
        let (a, b) = (l[i], l[(i + 1) % l.len()]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1e-18 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
        let q = [a[0] + dx * t, a[1] + dy * t];
        let d = d2(p, q);
        if d < best.0 {
            best = (d, q);
        }
    }
    best.1
}

/// A closed loop resampled at about `h` spacing (not closed).
fn resample_loop(l: &[P2], h: f64) -> Vec<P2> {
    let mut closed = l.to_vec();
    closed.push(l[0]);
    let len = path_len(&closed);
    let n = ((len / h).ceil() as usize).max(8);
    (0..n).map(|i| point_at(&closed, len * i as f64 / n as f64)).collect()
}

/// Adaptive clearing of an area with one island (e.g. the band round a
/// part) in a single pass: a centre path midway between the island's side
/// (`inner`, CW) and the outer side (`outer`, CCW) of where the tool centre
/// may go, with a loop radius at each point that spans the width there but
/// never leaves that area. Returns the centre path, the radii, and the area
/// the tool cuts (tool radius `r` added) for the cleanup of what is left.
fn single_pass(outer: &[P2], inner: &[P2], allowed: &[Vec<P2>], r: f64) -> Option<(Vec<P2>, Vec<f64>, geo::MultiPolygon<f64>)> {
    let ps = resample_loop(inner, 0.25);
    let qs: Vec<P2> = ps.iter().map(|&p| closest_on_loop(outer, p)).collect();
    let mid: Vec<P2> = ps.iter().zip(&qs).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0]).collect();
    // smooth the centre path only where it jumps (nearest points jump at
    // corners): elsewhere the exact midpoint lets the loops reach both sides.
    // Then fit the radius to the room actually there
    let n = mid.len();
    let jump = |i: usize, j: usize| d2(mid[i % n], mid[j % n]) > 0.5;
    let c: Vec<P2> = (0..n)
        .map(|i| {
            if !jump(i + n - 1, i) && !jump(i, i + 1) {
                return mid[i];
            }
            let (mut x, mut y) = (0.0, 0.0);
            for k in 0..5 {
                let q = mid[(i + n + k - 2) % n];
                x += q[0];
                y += q[1];
            }
            [x / 5.0, y / 5.0]
        })
        .collect();
    let rad: Vec<f64> = c.iter().zip(ps.iter().zip(&qs)).map(|(c, (p, q))| (d2(*p, *q) / 2.0).min(boundary_dist(allowed, *c)) - 1e-6).map(|v| v.max(0.0)).collect();
    if !c.iter().all(|&q| inside(allowed, q) || boundary_dist(allowed, q) < 1e-3) {
        return None; // not a simple band: use the rings
    }
    // what the loops cut: a band of half-width radius + r round the path,
    // as quads between consecutive samples (an under-estimate: safe for the
    // cleanup)
    let swept = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut quads = vec![];
        for i in 0..n {
            let j = (i + 1) % n;
            let side = |k: usize| {
                let (a, b) = (ps[k], qs[k]);
                let l = d2(a, b).max(1e-9);
                let u = [(b[0] - a[0]) / l, (b[1] - a[1]) / l];
                // (+ a hair: the loops reach the walls, so the cleanup
                // does not chase numerical slivers along them)
                let w = rad[k] + r + 0.002;
                ([c[k][0] - u[0] * w, c[k][1] - u[1] * w], [c[k][0] + u[0] * w, c[k][1] + u[1] * w])
            };
            let ((a0, b0), (a1, b1)) = (side(i), side(j));
            let mut q = vec![a0, b0, b1, a1];
            if signed_area(&q).abs() < 1e-9 {
                continue;
            }
            if signed_area(&q) < 0.0 {
                q.reverse();
            }
            quads.push(to_geo(&q, &[]));
        }
        union_tree(quads)
    }))
    .ok()?;
    Some((c, rad, swept))
}

/// Trochoidal loops along a closed centre path with a radius per vertex,
/// `advance` along the path per loop.
fn trochoid_var(center: &[P2], radius: &[f64], advance: f64) -> Vec<P2> {
    let mut cl = center.to_vec();
    cl.push(center[0]);
    let mut rl = radius.to_vec();
    rl.push(radius[0]);
    let mut cum = vec![0.0];
    for w in cl.windows(2) {
        cum.push(cum.last().unwrap() + d2(w[0], w[1]));
    }
    let l = *cum.last().unwrap();
    let rad_at = |s: f64| {
        let k = cum.partition_point(|&c| c <= s).clamp(1, cum.len() - 1);
        let (s0, s1) = (cum[k - 1], cum[k]);
        let t = if s1 > s0 { ((s - s0) / (s1 - s0)).clamp(0.0, 1.0) } else { 0.0 };
        rl[k - 1] + (rl[k] - rl[k - 1]) * t
    };
    let revs = (l / advance.max(0.01)).ceil().max(1.0) as usize;
    let per_rev = 24;
    let n = revs * per_rev;
    (0..=n)
        .map(|i| {
            let s = l * i as f64 / n as f64;
            let c = point_at(&cl, s);
            let rr = rad_at(s);
            let th = TAU * i as f64 / per_rev as f64 - PI / 2.0;
            [c[0] + rr * th.cos(), c[1] + rr * th.sin()]
        })
        .collect()
}

/// Points within `w` of a closed loop (either orientation).
fn ring_around(l: &[P2], w: f64) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let mut l = l.to_vec();
    if signed_area(&l) < 0.0 {
        l.reverse();
    }
    let ccw = |v: Vec<Vec<P2>>| union_all(v.into_iter().filter(|x| signed_area(x) > 0.0).map(|x| to_geo(&x, &[])));
    ccw(offset_region(&l, &[], -w)).difference(&ccw(offset_region(&l, &[], w)))
}

/// Standard loops for what adaptive clearing's trochoidal loops (which cut
/// `swept`) leave of `area` (the material to clear): a pocket of the tool
/// centres that reach it (the
/// left-over material grown by the tool radius `r`, kept to where the tool
/// centre may go, `allowed`), both as offset loops (CCW outers, CW holes).
/// Offset index from `k0` up (inside out, so they are cut first).
fn adaptive_cleanup(swept: &geo::MultiPolygon<f64>, area: &[Vec<P2>], allowed: &[Vec<P2>], r: f64, step: f64, k0: usize) -> Vec<(usize, Vec<P2>, LoopCut)> {
    use geo::BooleanOps;
    let region = |ls: &[Vec<P2>]| {
        let outers = union_all(ls.iter().filter(|l| signed_area(l) > 0.0).map(|l| to_geo(l, &[])));
        let holes = union_all(ls.iter().filter(|l| signed_area(l) < 0.0).map(|l| to_geo(&l.iter().rev().copied().collect::<Vec<_>>(), &[])));
        outers.difference(&holes)
    };
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let left = region(area).difference(swept);
        // tool centres within a radius of what is left (a hair less, so the
        // outer loop just touches it), where the tool centre may go
        let mut grown: Vec<Vec<P2>> = vec![];
        for p in &left.0 {
            let ext = from_geo_ring(p.exterior());
            if ext.len() < 3 || signed_area(&ext).abs() < 1e-6 {
                continue;
            }
            let hs: Vec<Vec<P2>> = p.interiors().iter().map(from_geo_ring).collect();
            grown.extend(offset_region(&ext, &hs, -(r - 0.05).max(0.0)));
        }
        // the union of many rings is jagged: smooth it (well inside the
        // overlaps) so the offsetter copes
        use geo::Simplify;
        region(&grown).intersection(&region(allowed)).simplify(&MESH_SIMPLIFY)
    }));
    let Ok(left) = res else { return vec![] };
    let mut out = vec![];
    for p in &left.0 {
        let ext = from_geo_ring(p.exterior());
        if ext.len() < 3 || signed_area(&ext).abs() < 1e-4 {
            continue; // slivers
        }
        let hs: Vec<Vec<P2>> = p.interiors().iter().map(from_geo_ring).collect();
        // the edge (a hair in: a zero offset gives nothing), then inwards a
        // stepover at a time
        let mut j = 0;
        loop {
            let ls = offset_region(&ext, &hs, 1e-3 + j as f64 * step);
            if ls.is_empty() || j > 10000 {
                break;
            }
            out.extend(ls.into_iter().map(|l| (k0 + j, l, LoopCut::Plain)));
            j += 1;
        }
    }
    out
}

/// Default stepovers (% of the cutter Ø): standard offset passes, and
/// adaptive clearing (its advance per trochoidal loop).
pub const STEPOVER_STANDARD: f64 = 40.0;
pub const STEPOVER_ADAPTIVE: f64 = 20.0;

/// The stepover after switching strategy: a stepover still at the old
/// strategy's default takes the new one's.
pub fn stepover_for(old: Strategy, new: Strategy, stepover: f64) -> f64 {
    let default = |s: Strategy| if s == Strategy::Adaptive { STEPOVER_ADAPTIVE } else { STEPOVER_STANDARD };
    if old != new && (stepover - default(old)).abs() < 1e-9 {
        default(new)
    } else {
        stepover
    }
}

/// Radius of adaptive clearing's trochoidal loops (fraction of the cutter Ø).
const TROCHOID_R: f64 = 0.4;

/// Trochoidal path along a closed centre path (already closed).
fn trochoid(center: &[P2], radius: f64, advance: f64) -> Vec<P2> {
    let l = path_len(center);
    let revs = (l / advance.max(0.01)).ceil().max(1.0) as usize;
    let per_rev = 24;
    let n = revs * per_rev;
    (0..=n)
        .map(|i| {
            let c = point_at(center, l * i as f64 / n as f64);
            let th = TAU * i as f64 / per_rev as f64 - PI / 2.0;
            [c[0] + radius * th.cos(), c[1] + radius * th.sin()]
        })
        .collect()
}

fn levels(start: f64, floor: f64, step: f64) -> Vec<f64> {
    let n = ((start - floor) / step.max(0.01) - 1e-9).ceil().max(1.0) as usize;
    (1..=n).map(|i| start - (start - floor) * i as f64 / n as f64).collect()
}

// ------------------------------------------------------------------ moves
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mv {
    Rapid(P3),
    Feed(P3, f64),
}

struct Path {
    moves: Vec<Mv>,
    pos: P3,
}

impl Path {
    fn new(pos: P3) -> Self {
        Path { moves: vec![], pos }
    }
    fn rapid(&mut self, p: P3) {
        self.moves.push(Mv::Rapid(p));
        self.pos = p;
    }
    fn feed(&mut self, p: P3, f: f64) {
        if (p[0] - self.pos[0]).abs() + (p[1] - self.pos[1]).abs() + (p[2] - self.pos[2]).abs() < 1e-9 {
            return;
        }
        self.moves.push(Mv::Feed(p, f));
        self.pos = p;
    }
    /// Retract and rapid to above `p`, then down to `z_near`.
    fn reposition(&mut self, retract: f64, p: P2, z_near: f64) {
        self.rapid([self.pos[0], self.pos[1], retract]);
        self.rapid([p[0], p[1], retract]);
        self.rapid([p[0], p[1], z_near]);
    }
}

fn helix_turns(depth: f64, r: f64, angle: f64) -> usize {
    let per = TAU * r * angle.to_radians().tan();
    if per <= 1e-9 {
        1
    } else {
        ((depth / per) - 1e-9).ceil().max(1.0) as usize
    }
}

struct PocketCtx<'a> {
    op: &'a PocketOp,
    allowed: Vec<Vec<P2>>,
    r: f64,
}

impl PocketCtx<'_> {
    fn seg_ok(&self, a: P2, b: P2) -> bool {
        let n = ((d2(a, b) / (self.r * 0.25).max(0.05)).ceil() as usize).max(1);
        (0..=n).all(|i| {
            let t = i as f64 / n as f64;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            inside(&self.allowed, p) || boundary_dist(&self.allowed, p) < 1e-6
        })
    }

    /// Descend from z_from to z_to at the start of `path` using the entry style.
    fn entry(&self, out: &mut Path, path: &[P2], z_from: f64, z_to: f64) {
        let op = self.op;
        let p0 = path[0];
        let depth = z_from - z_to;
        let mut style = op.entry;
        if style == Entry::Helix {
            let room = boundary_dist(&self.allowed, p0);
            let rho = (op.cutter.dia() * 0.45).min(room * 0.95);
            if rho < op.cutter.dia() * 0.05 || !inside(&self.allowed, p0) {
                style = Entry::Zigzag;
            } else {
                let turns = helix_turns(depth, rho, op.ramp_angle);
                let n = turns * 24;
                out.feed([p0[0] + rho, p0[1], z_from], op.feed);
                for i in 1..=n {
                    let a = TAU * i as f64 / 24.0;
                    let z = z_from - depth * i as f64 / n as f64;
                    out.feed([p0[0] + rho * a.cos(), p0[1] + rho * a.sin(), z], op.feed);
                }
                out.feed([p0[0], p0[1], z_to], op.feed);
                return;
            }
        }
        if style == Entry::Zigzag {
            let total = path_len(path);
            let lz = (total * 0.5).min((op.cutter.dia() * 3.0).max(5.0));
            let tan = op.ramp_angle.to_radians().tan().max(1e-3);
            if lz > op.cutter.dia() * 0.25 {
                let mut z = z_from;
                let mut s = 0.0;
                let mut forward = true;
                while z > z_to + 1e-9 {
                    let (s_next, len) = if forward { (lz, lz - s) } else { (0.0, s) };
                    let dz = len * tan;
                    // follow the path's own vertices (never across its corners),
                    // descending in proportion to the distance along it
                    let (s_end, z_end) = if z - dz <= z_to {
                        // reach depth part way along this pass
                        (s + (s_next - s) * (z - z_to) / dz, z_to)
                    } else {
                        (s_next, z - dz)
                    };
                    let span = s_end - s;
                    for ss in stations(path, s, s_end) {
                        let f = if span.abs() > 1e-12 { (ss - s) / span } else { 1.0 };
                        let p = point_at(path, ss);
                        out.feed([p[0], p[1], z + (z_end - z) * f], op.feed);
                    }
                    if z_end > z_to {
                        forward = !forward;
                    }
                    z = z_end;
                    s = s_end;
                }
                // back to the path start at depth
                for ss in stations(path, s, 0.0) {
                    let p = point_at(path, ss);
                    out.feed([p[0], p[1], z_to], op.feed);
                }
                return;
            }
        }
        // plunge
        out.feed([p0[0], p0[1], z_to], (op.feed * 0.3).max(1.0));
    }
}

/// Highest Z of a sloped pocket floor.
pub fn floor_top(tris: &[[P3; 3]]) -> f64 {
    tris.iter().flatten().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max)
}

/// Where a planar face (outline `outer` less `holes`, plane from `tris`) is
/// at or below `z`, seen from above, plus the `extra` polygons: (outer, holes)
/// regions.
pub fn slice_below(outer: &[P2], holes: &[Vec<P2>], tris: &[[P3; 3]], z: f64, extra: &[Vec<P2>]) -> Result<Vec<(Vec<P2>, Vec<Vec<P2>>)>, String> {
    let mut polys = vec![];
    for e in extra.iter().filter(|e| e.len() >= 3) {
        let mut e = e.clone();
        if signed_area(&e) < 0.0 {
            e.reverse();
        }
        polys.push(to_geo(&e, &[]));
    }
    let m = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        polys.extend(clip_face(outer, holes, tris, z, true).0);
        union_tree(polys)
    }))
        .map_err(|_| format!("could not work out the surface outline at Z{}", fmt(z)))?;
    Ok(m.0
        .iter()
        .map(|p| {
            let holes = p.interiors().iter().map(from_geo_ring).filter(|h| h.len() >= 3 && signed_area(h).abs() > 1e-4).collect();
            (from_geo_ring(p.exterior()), holes)
        })
        .filter(|(o, _)| o.len() >= 3 && signed_area(o).abs() > 1e-4)
        .collect())
}

/// Seen from above, the half of the XY plane where the plane of `tris` is
/// at or below (`below`) / above `z`, as a large polygon (None: nowhere).
/// `big` must be larger than the face (kept small for precision).
fn half_plane(tris: &[[P3; 3]], z: f64, below: bool, big: f64) -> Option<geo::Polygon<f64>> {
    // plane from the triangle with the largest area seen from above
    let t = tris.iter().max_by(|a, b| tri_area(a).total_cmp(&tri_area(b)))?;
    let (e1, e2) = ([t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]], [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]]);
    let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
    if n[2].abs() < 1e-12 {
        return None;
    }
    // height = t0.z + g . (xy - t0.xy)
    let g = [-n[0] / n[2], -n[1] / n[2]];
    let gl = g[0].hypot(g[1]);
    if gl < 1e-9 {
        let kept = if below { t[0][2] <= z } else { t[0][2] > z };
        return kept.then(|| to_geo(&[[-big, -big], [big, -big], [big, big], [-big, big]], &[]));
    }
    let u = [g[0] / gl, g[1] / gl]; // uphill
    let s = (z - t[0][2]) / gl;
    let c = [t[0][0] + u[0] * s, t[0][1] + u[1] * s]; // on the level line
    let side = if below { -1.0 } else { 1.0 };
    let v = [-u[1], u[0]];
    let p = |a: f64, b: f64| [c[0] + v[0] * a + u[0] * side * b, c[1] + v[1] * a + u[1] * side * b];
    let mut q = vec![p(-big, 0.0), p(big, 0.0), p(big, big), p(-big, big)];
    if signed_area(&q) < 0.0 {
        q.reverse();
    }
    Some(to_geo(&q, &[]))
}

fn tri_area(t: &[P3; 3]) -> f64 {
    ((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1])).abs() / 2.0
}

/// The part of a planar face (outline `outer` less `holes`, plane from
/// `tris`) at or below (`below`) / above `z`, seen from above.
fn clip_face(outer: &[P2], holes: &[Vec<P2>], tris: &[[P3; 3]], z: f64, below: bool) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let pts = || outer.iter().chain(holes.iter().flatten());
    let big = pts().map(|p| p[0].abs().max(p[1].abs())).fold(1.0, f64::max) * 4.0;
    let Some(h) = half_plane(tris, z, below, big) else { return geo::MultiPolygon::new(vec![]) };
    let mut m = to_geo(outer, holes).intersection(&h);
    // put corners of the face back exactly where they were, so the slice
    // joins neighbouring faces without slivers
    let snap = |c: &mut geo::Coord<f64>| {
        if let Some(p) = pts().find(|p| (p[0] - c.x).abs() < 1e-6 && (p[1] - c.y).abs() < 1e-6) {
            (c.x, c.y) = (p[0], p[1]);
        }
    };
    use geo::MapCoordsInPlace;
    m.map_coords_in_place(|mut c| {
        snap(&mut c);
        c
    });
    m
}

/// Union of many polygons by merging pairs (much faster than one at a time).
fn union_tree(polys: Vec<geo::Polygon<f64>>) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let mut parts: Vec<geo::MultiPolygon<f64>> = polys.into_iter().map(|p| geo::MultiPolygon::new(vec![p])).collect();
    if parts.is_empty() {
        return geo::MultiPolygon::new(vec![]);
    }
    while parts.len() > 1 {
        let mut next = Vec::with_capacity(parts.len() / 2 + 1);
        let mut it = parts.into_iter();
        while let Some(a) = it.next() {
            next.push(match it.next() {
                Some(b) => a.union(&b),
                None => a,
            });
        }
        parts = next;
    }
    parts.pop().unwrap()
}

/// Z levels of a sloped pocket: even steps down to the lowest point of the
/// surface, plus one at its highest point.
pub fn sloped_levels(op: &PocketOp) -> Vec<f64> {
    let top = floor_top(&op.floor);
    let mut zs = levels(op.start_z, op.z, op.depth_per_pass);
    if top < op.start_z - 1e-6 && top > op.z + 1e-6 {
        zs.push(top);
    }
    zs.sort_by(|a, b| b.total_cmp(a));
    zs.dedup_by(|a, b| (*a - *b).abs() < 0.01);
    zs
}

/// Sloped floor: at each level clear (and finish the walls of) the slice of
/// the pocket where the surface lies below the level.
fn sloped_pocket_moves(op: &PocketOp, retract: f64, from: P2) -> Result<Vec<Mv>, String> {
    let mut out = vec![];
    let mut z_prev = op.start_z;
    for z in sloped_levels(op) {
        let extra: Vec<Vec<P2>> = if op.keep_islands {
            vec![]
        } else {
            op.islands.iter().enumerate().filter(|(k, _)| op.islands_z.get(*k).map_or(true, |&iz| iz <= z + 1e-6)).map(|(_, l)| l.clone()).collect()
        };
        let slices = slice_below(&op.outer, &op.islands, &op.floor, z, &extra)?;
        let here = end_xy(&out, from);
        for (outer, islands) in nearest_first(slices, here, |(o, _), p| near_dist(o, p), |(o, _), _| o[0]) {
            let level = PocketOp {
                z,
                start_z: z_prev,
                depth_per_pass: z_prev - z + 1.0,
                outer,
                islands,
                keep_islands: true,
                floor: vec![],
                islands_z: vec![],
                edges: vec![],
                ..op.clone()
            };
            // slices too narrow for the cutter are left
            if let Ok(m) = pocket_moves(&level, retract, end_xy(&out, from)) {
                out.extend(m);
            }
        }
        z_prev = z;
    }
    if out.is_empty() {
        return Err(format!("cutter Ø{} does not fit in the sloped pocket at any level", fmt(op.cutter.dia())));
    }
    Ok(out)
}

/// `from`: where the tool is (XY), so clearing starts nearby.
fn pocket_moves(op: &PocketOp, retract: f64, from: P2) -> Result<Vec<Mv>, String> {
    if !op.floor.is_empty() {
        return sloped_pocket_moves(op, retract, from);
    }
    let d = op.cutter.dia();
    let r = d / 2.0;
    let islands: Vec<Vec<P2>> = if op.keep_islands { op.islands.clone() } else { vec![] };
    let rough0 = r + op.finish.max(0.0);
    let allowed = offset_region(&op.outer, &islands, rough0);
    if allowed.is_empty() {
        return Err(format!("cutter Ø{} does not fit in the pocket (with {} finish allowance)", fmt(d), fmt(op.finish)));
    }
    let finish_loops = offset_region(&op.outer, &islands, if op.finish_pass { r } else { rough0 });

    // centre loops for roughing, with their offset index
    let step = (op.stepover_pct / 100.0 * d).max(0.01);
    // (offset index, loop, how it is cut)
    let mut centre: Vec<(usize, Vec<P2>, LoopCut)> = Vec::new();
    let mut troch_r = 0.0;
    // advance per loop of the single-pass trochoids
    let mut advance = step;
    // standard passes clearing what the trochoids leave
    let clean_step = step.max(0.4 * d);
    if op.strategy == Strategy::Adaptive {
        troch_r = TROCHOID_R * d;
        let finish_area = offset_region(&op.outer, &islands, op.finish.max(0.0));
        // one island (e.g. the band round a part): a single pass down the
        // middle, its loops as wide as the band
        let ccw: Vec<&Vec<P2>> = allowed.iter().filter(|l| signed_area(l) > 0.0).collect();
        let cw: Vec<&Vec<P2>> = allowed.iter().filter(|l| signed_area(l) < 0.0).collect();
        if op.adaptive_outer_only {
            // mixed: one ring of loops round the outer boundary (advancing at
            // the adaptive share of the stepover), then standard passes at the
            // stepover for the rest, cut first
            advance = step * STEPOVER_ADAPTIVE / STEPOVER_STANDARD;
            let ring: Vec<Vec<P2>> = offset_region(&op.outer, &islands, rough0 + troch_r).into_iter().filter(|l| signed_area(l) > 0.0).collect();
            if !ring.is_empty() {
                let swept = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| union_tree(ring.iter().flat_map(|l| ring_around(l, troch_r + r).0).collect())))
                    .unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
                centre.extend(ring.into_iter().map(|l| (0, l, LoopCut::Trochoid)));
                centre.extend(adaptive_cleanup(&swept, &finish_area, &allowed, r, step, 1));
            } else {
                troch_r = 0.0; // too narrow for trochoids: standard loops
            }
        } else if ccw.len() == 1 && cw.len() == 1 {
            if let Some((path, rad, swept)) = single_pass(ccw[0], cw[0], &allowed, r) {
                // the outer side travels further per loop: keep its cut to the stepover
                let mut cp = path.clone();
                cp.push(path[0]);
                let adv = step * (path_len(&cp) / path_len(&[ccw[0].clone(), vec![ccw[0][0]]].concat())).min(1.0);
                // a loop as wide as the band cuts mostly air when the band is
                // wide: use it only where it is the shorter way (narrow bands,
                // like the one round a part's outside profile)
                let single: f64 = cp.windows(2).zip(&rad).map(|(w, rr)| d2(w[0], w[1]) / adv.max(0.01) * TAU * rr.max(0.05 * d)).sum();
                let rings: f64 = {
                    let mut t = 0.0;
                    let mut k = 0;
                    loop {
                        let ls = offset_region(&op.outer, &islands, rough0 + troch_r + k as f64 * 1.4 * d);
                        if ls.is_empty() || k > 10000 {
                            break;
                        }
                        t += ls.iter().map(|l| (path_len(l) + d2(l[0], *l.last().unwrap())) / step * TAU * troch_r).sum::<f64>();
                        k += 1;
                    }
                    t
                };
                // (with the standard passes it needs where it cannot reach)
                let cleanup = adaptive_cleanup(&swept, &finish_area, &allowed, r, clean_step, 1);
                let single = single + cleanup.iter().map(|(_, l, _)| path_len(l) + d2(l[0], *l.last().unwrap())).sum::<f64>();
                let narrow = rad.iter().cloned().fold(0.0, f64::max) <= 1.6 * troch_r;
                if rings <= 0.0 || (narrow && single <= rings) {
                    advance = adv;
                    centre.push((0, path, LoopCut::Varying(rad)));
                    centre.extend(cleanup);
                }
            }
        }
        if centre.is_empty() && !op.adaptive_outer_only {
            let spacing = 1.4 * d;
            let mut k = 0;
            loop {
                let ls = offset_region(&op.outer, &islands, rough0 + troch_r + k as f64 * spacing);
                if ls.is_empty() || k > 10000 {
                    break;
                }
                centre.extend(ls.into_iter().map(|l| (k, l, LoopCut::Trochoid)));
                k += 1;
            }
            if centre.is_empty() {
                troch_r = 0.0; // too narrow for trochoids: fall back to standard loops
            } else {
                // the loops from opposite walls stop up to a spacing short of
                // each other: clear what they leave with standard loops (cut
                // first, innermost first)
                let swept = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| union_tree(centre.iter().flat_map(|(_, l, _)| ring_around(l, troch_r + r).0).collect())))
                    .unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
                centre.extend(adaptive_cleanup(&swept, &finish_area, &allowed, r, clean_step, k));
            }
        }
    }
    if centre.is_empty() {
        let mut k = 0;
        loop {
            let ls = if k == 0 { allowed.clone() } else { offset_region(&op.outer, &islands, rough0 + k as f64 * step) };
            if ls.is_empty() || k > 10000 {
                break;
            }
            centre.extend(ls.into_iter().map(|l| (k, l, LoopCut::Plain)));
            k += 1;
        }
    }

    let ctx = PocketCtx { op, allowed: allowed.clone(), r };
    let mut out = Path::new([from[0], from[1], retract]);
    let zs = levels(op.start_z, op.z, op.depth_per_pass);
    let mut z_prev = op.start_z;
    let mut first = true;
    for &z in &zs {
        let mut remaining: Vec<usize> = (0..centre.len()).collect();
        let mut level_started = false;
        while !remaining.is_empty() {
            let kmax = remaining.iter().map(|&i| centre[i].0).max().unwrap();
            let here = [out.pos[0], out.pos[1]];
            let (ri, _) = remaining
                .iter()
                .enumerate()
                .filter(|(_, &i)| centre[i].0 == kmax)
                .map(|(ri, &i)| (ri, centre[i].1.iter().map(|q| d2(*q, here)).fold(f64::INFINITY, f64::min)))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap();
            let ci = remaining.remove(ri);
            let path: Vec<P2> = match &centre[ci].2 {
                LoopCut::Plain => start_near(&centre[ci].1, here),
                LoopCut::Trochoid => trochoid(&start_near(&centre[ci].1, here), troch_r, advance),
                LoopCut::Varying(rad) => {
                    // start nearest the tool, radii turned with the path
                    let l = &centre[ci].1;
                    let i0 = (0..l.len()).min_by(|&a, &b| d2(l[a], here).total_cmp(&d2(l[b], here))).unwrap_or(0);
                    let c: Vec<P2> = l[i0..].iter().chain(l[..i0].iter()).copied().collect();
                    let rr: Vec<f64> = rad[i0..].iter().chain(rad[..i0].iter()).copied().collect();
                    trochoid_var(&c, &rr, advance)
                }
            };
            let p0 = path[0];

            if first {
                out.reposition(retract, p0, op.start_z + 0.5_f64.min(retract - op.start_z));
                ctx.entry(&mut out, &path, op.start_z, z);
                first = false;
            } else if !level_started {
                // new level: start where the region is cleared at z_prev
                if out.pos[2] <= z_prev + 1e-9 && ctx.seg_ok(here, p0) {
                    out.feed([p0[0], p0[1], z_prev], op.feed);
                } else {
                    out.reposition(retract, p0, z_prev + 0.5);
                    out.feed([p0[0], p0[1], z_prev], op.feed);
                }
                ctx.entry(&mut out, &path, z_prev, z);
            } else if ctx.seg_ok(here, p0) {
                out.feed([p0[0], p0[1], z], op.feed);
            } else {
                out.reposition(retract, p0, z_prev + 0.5);
                out.feed([p0[0], p0[1], z_prev], op.feed);
                ctx.entry(&mut out, &path, z_prev, z);
            }
            level_started = true;
            for q in &path[1..] {
                out.feed([q[0], q[1], z], op.feed);
            }
        }
        z_prev = z;
    }

    // finishing pass on all walls, each level top-down (roughing: only the
    // part's walls, or none)
    let finish_loops: Vec<Vec<P2>> = if op.no_contour {
        vec![]
    } else {
        finish_loops.into_iter().filter(|l| !(op.open_outer && signed_area(l) > 0.0)).collect()
    };
    let mut z_above = op.start_z;
    for &z in &zs {
        for fl in &finish_loops {
            let here = [out.pos[0], out.pos[1]];
            let lp = start_near(fl, here);
            let s = lp[0];
            // lead-in point: nearest point inside the roughed region
            let lead = allowed
                .iter()
                .flat_map(|l| l.iter())
                .copied()
                .min_by(|a, b| d2(*a, s).total_cmp(&d2(*b, s)))
                .unwrap_or(s);
            if out.pos[2] <= z + 1e-9 && ctx.seg_ok(here, lead) {
                out.feed([lead[0], lead[1], z], op.feed);
            } else {
                // down to just above the level before (clear everywhere the
                // tool can be), then feed: at this level the clearing may
                // stop a hair short of the tool radius in corners
                out.reposition(retract, lead, (z_above + 0.5).min(retract));
                out.feed([lead[0], lead[1], z], op.feed);
            }
            for q in &lp {
                out.feed([q[0], q[1], z], op.feed);
            }
            out.feed([lead[0], lead[1], z], op.feed);
        }
        z_above = z;
    }
    out.rapid([out.pos[0], out.pos[1], retract]);
    Ok(out.moves)
}

fn face_moves(op: &FaceOp, retract: f64) -> Vec<Mv> {
    let r = op.cutter.dia() / 2.0;
    if !op.outline.is_empty() {
        let centres = grow_loops(&op.outline, r + op.overlap.max(0.0) + OPEN_EDGE);
        return facing_moves(&centres, &op.outline, FacingCut { cutter: op.cutter, stepover_pct: op.stepover_pct, start_z: op.start_z, z: op.z, depth_per_pass: op.depth_per_pass, feed: op.feed, edge_lap: false, pattern: op.pattern }, retract);
    }
    // no outline (older jobs): the region box, edge `overlap` past it
    let [minx, miny, maxx, maxy] = op.region;
    let ov = op.overlap.max(0.0);
    let (x0, x1) = (minx - ov - r, maxx + ov + r);
    let rect = vec![[x0, miny - ov - r], [x1, miny - ov - r], [x1, maxy + ov + r], [x0, maxy + ov + r]];
    let inner = vec![vec![[minx - ov, miny - ov], [maxx + ov, miny - ov], [maxx + ov, maxy + ov], [minx - ov, maxy + ov]]];
    facing_moves(&[rect], &inner, FacingCut { cutter: op.cutter, stepover_pct: op.stepover_pct, start_z: op.start_z, z: op.z, depth_per_pass: op.depth_per_pass, feed: op.feed, edge_lap: false, pattern: op.pattern }, retract)
}

/// Loops grown outwards by `d`, overlaps merged.
pub fn grow_loops(loops: &[Vec<P2>], d: f64) -> Vec<Vec<P2>> {
    let grown: Vec<geo::Polygon<f64>> = loops.iter().flat_map(|l| offset_region(l, &[], -d)).filter(|l| signed_area(l) > 0.0).map(|l| to_geo(&l, &[])).collect();
    let u = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| union_tree(grown))).unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
    u.0.iter().flat_map(|p| std::iter::once(from_geo_ring(p.exterior())).chain(p.interiors().iter().map(from_geo_ring))).filter(|l| l.len() >= 3).collect()
}

/// The part's outside profile seen from above: every face, holes filled.
pub fn part_outline(faces: &[RoughFace]) -> Vec<Vec<P2>> {
    let polys: Vec<geo::Polygon<f64>> = faces
        .iter()
        .flat_map(|f| {
            if f.curved {
                use geo::Simplify;
                union_tree(mesh_above(&f.tris, f64::NEG_INFINITY)).simplify(&MESH_SIMPLIFY).0
            } else {
                vec![to_geo(&f.outer, &[])]
            }
        })
        .map(|p| geo::Polygon::new(p.exterior().clone(), vec![]))
        .collect();
    let u = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| union_tree(polys))).unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
    u.0.iter().map(|p| from_geo_ring(p.exterior())).filter(|l| l.len() >= 3 && signed_area(l).abs() > 1e-6).collect()
}

/// What a facing pass cuts with and to.
struct FacingCut {
    cutter: Cutter,
    stepover_pct: f64,
    start_z: f64,
    z: f64,
    depth_per_pass: f64,
    feed: f64,
    /// After the rows of each level, a lap round the edge of the centre area,
    /// so everything a tool radius past it is faced (the rows end on a round
    /// edge a stepover apart and fall short of it between them).
    edge_lap: bool,
    pattern: FacingPattern,
}

/// Facing rows clipped to `centres` (where the tool centre may go, even-odd
/// loops), covering `material` (the outline rows must reach). Rows run +X
/// stepping towards -Y (climb); a row split by the outline is cut piece by
/// piece. Moves between pieces stay just above the last level when they only
/// cross faced ground (inside `centres`), else go up to `retract`.
fn facing_moves(centres: &[Vec<P2>], material: &[Vec<P2>], c: FacingCut, retract: f64) -> Vec<Mv> {
    let d = c.cutter.dia();
    let (mut ylo, mut yhi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in material.iter().flatten() {
        ylo = ylo.min(p[1]);
        yhi = yhi.max(p[1]);
    }
    if !ylo.is_finite() || centres.is_empty() {
        return vec![];
    }
    let step = (c.stepover_pct / 100.0 * d).max(0.01);
    // the cuts of one level, in order (each a polyline at depth)
    let runs: Vec<Vec<P2>> = match c.pattern {
        FacingPattern::Unidirectional | FacingPattern::Zigzag => {
            // row centres from the top of the material to its bottom
            let n = (((yhi - ylo) / step) - 1e-9).ceil().max(0.0) as usize;
            let rows: Vec<f64> = (0..=n).map(|k| if n == 0 { yhi } else { yhi - (yhi - ylo) * k as f64 / n as f64 }).collect();
            let mut runs = vec![];
            for (k, &y) in rows.iter().enumerate() {
                let mut ps = row_pieces(centres, y);
                // +X with material towards -Y = climb; zigzag comes back -X
                if c.pattern == FacingPattern::Zigzag && k % 2 == 1 {
                    ps.reverse();
                    runs.extend(ps.into_iter().map(|(a, b)| vec![[b, y], [a, y]]));
                } else {
                    runs.extend(ps.into_iter().map(|(a, b)| vec![[a, y], [b, y]]));
                }
            }
            runs
        }
        // a point by a loop's corner is up to √2 x half a step from the
        // loops: keep that within the tool radius
        FacingPattern::Spiral => spiral_runs(centres, material, step.min(SPIRAL_MAX_STEP * d), d / 2.0),
        FacingPattern::SquareSpiral => square_spiral_runs(centres, step.min(SPIRAL_MAX_STEP * d)),
    };
    // unidirectional lifts after each row; the others stay down where the
    // way on is over the area being faced
    let stay_down = c.pattern != FacingPattern::Unidirectional;
    let region = vec![centres.to_vec()];
    let mut out = Path::new([0.0, 0.0, retract]);
    let mut z_prev = c.start_z;
    let mut first = true;
    for z in levels(c.start_z, c.z, c.depth_per_pass) {
        let lift = z_prev + 0.5;
        for run in &runs {
            let here = [out.pos[0], out.pos[1]];
            let s0 = run[0];
            if stay_down && !first && (out.pos[2] - z).abs() < 1e-9 && d2(here, s0) <= 2.0 * d + step && seg_in(&region, here, s0) {
                out.feed([s0[0], s0[1], z], c.feed);
            } else {
                // above the first level nothing stands over the start height;
                // below it only faced ground inside the centre area is clear
                let low_ok = !first && out.pos[2] <= lift + 1e-9 && (z_prev >= c.start_z - 1e-9 || seg_in(&region, here, s0));
                if first || !low_ok {
                    out.reposition(retract, s0, lift);
                } else {
                    out.rapid([here[0], here[1], lift.max(out.pos[2])]);
                    out.rapid([s0[0], s0[1], lift]);
                }
                first = false;
                out.feed([s0[0], s0[1], z], (c.feed * 0.5).max(1.0));
            }
            for q in &run[1..] {
                out.feed([q[0], q[1], z], c.feed);
            }
            if !stay_down {
                out.feed([out.pos[0], out.pos[1], lift], c.feed);
            }
        }
        // (a spiral's first loop is the edge)
        if c.edge_lap && c.pattern != FacingPattern::Spiral {
            for (k, l) in centres.iter().enumerate() {
                let here = [out.pos[0], out.pos[1]];
                // CCW round the outside (climb, stock outside), CW round
                // holes (loops inside an odd number of others)
                let depth = centres.iter().enumerate().filter(|(j, o)| *j != k && inside(std::slice::from_ref(*o), l[0])).count();
                let mut l = l.clone();
                if (signed_area(&l) > 0.0) != (depth % 2 == 0) {
                    l.reverse();
                }
                let seq = start_near(&l, here);
                let s0 = seq[0];
                if !first && out.pos[2] <= lift + 1e-9 && seg_in(&region, here, s0) {
                    out.rapid([here[0], here[1], lift.max(out.pos[2])]);
                    out.rapid([s0[0], s0[1], lift]);
                } else {
                    out.reposition(retract, s0, lift);
                }
                first = false;
                out.feed([s0[0], s0[1], z], (c.feed * 0.5).max(1.0));
                for q in &seq[1..] {
                    out.feed([q[0], q[1], z], c.feed);
                }
                out.feed([s0[0], s0[1], lift], c.feed);
            }
        }
        z_prev = z;
    }
    out.rapid([out.pos[0], out.pos[1], retract]);
    out.moves
}

/// What the spiral's loops are taken to clear: within the tool radius less
/// this (the offsets' arcs are chords, so a hair less than they reach).
const SPIRAL_SWEPT_MARGIN: f64 = 0.05;

/// Largest stepover of the spiral facing patterns (fraction of the cutter Ø).
const SPIRAL_MAX_STEP: f64 = 0.7;

/// A row at `y`: its pieces inside the area (even-odd loops), left to right.
fn row_pieces(centres: &[Vec<P2>], y: f64) -> Vec<(f64, f64)> {
    let mut xs: Vec<f64> = vec![];
    for l in centres {
        for k in 0..l.len() {
            let (a, b) = (l[k], l[(k + 1) % l.len()]);
            if (a[1] > y) != (b[1] > y) {
                xs.push(a[0] + (y - a[1]) / (b[1] - a[1]) * (b[0] - a[0]));
            }
        }
    }
    xs.sort_by(|a, b| a.total_cmp(b));
    xs.chunks(2).filter(|w| w.len() == 2 && w[1] - w[0] > 1e-6).map(|w| (w[0], w[1])).collect()
}

/// Even-odd loops as areas: (CCW outer, CW holes).
fn even_odd_regions(loops: &[Vec<P2>]) -> Vec<(Vec<P2>, Vec<Vec<P2>>)> {
    let depth = |k: usize| loops.iter().enumerate().filter(|(j, o)| *j != k && !loops[k].is_empty() && inside(std::slice::from_ref(*o), loops[k][0])).count();
    let ds: Vec<usize> = (0..loops.len()).map(depth).collect();
    let mut out: Vec<(Vec<P2>, Vec<Vec<P2>>)> = vec![];
    let mut outers = vec![];
    for (k, l) in loops.iter().enumerate().filter(|(k, l)| ds[*k] % 2 == 0 && l.len() >= 3) {
        out.push((ccw(l.clone()), vec![]));
        outers.push(k);
    }
    for (k, l) in loops.iter().enumerate().filter(|(k, l)| ds[*k] % 2 == 1 && l.len() >= 3) {
        // the hole's outer: the containing outer one level up
        if let Some(n) = outers.iter().position(|&o| ds[o] + 1 == ds[k] && inside(std::slice::from_ref(&loops[o]), l[0])) {
            let mut h = l.clone();
            if signed_area(&h) > 0.0 {
                h.reverse();
            }
            out[n].1.push(h);
        }
    }
    out
}

/// Spiral facing: loops following the area's edge a stepover apart, outside
/// in (each outer loop CW and hole loop CCW: climb with the uncut middle
/// inside), nearest next. Past the last full step a loop half a step in;
/// what the loops still leave of `material` (along the middle of narrow
/// parts, where the next loop in has vanished) is cleared last with loops of
/// its own.
fn spiral_runs(centres: &[Vec<P2>], material: &[Vec<P2>], step: f64, r: f64) -> Vec<Vec<P2>> {
    let mut rings: Vec<Vec<Vec<P2>>> = vec![];
    for (o, hs) in even_odd_regions(centres) {
        let mut k = 0usize;
        loop {
            let at = |t: f64| offset_region(&o, &hs, 1e-3 + t * step);
            let ls = at(k as f64);
            if ls.is_empty() || k > 100_000 {
                if k > 0 {
                    let half = at(k as f64 - 0.5);
                    if !half.is_empty() {
                        if rings.len() < k + 1 {
                            rings.resize(k + 1, vec![]);
                        }
                        rings[k].extend(half);
                    }
                }
                break;
            }
            if rings.len() < k + 1 {
                rings.resize(k + 1, vec![]);
            }
            rings[k].extend(ls);
            k += 1;
        }
    }
    // what the loops leave
    let swept = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| union_tree(rings.iter().flatten().flat_map(|l| ring_around(l, r - SPIRAL_SWEPT_MARGIN).0).collect())))
        .unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
    let area: Vec<Vec<P2>> = even_odd_regions(material).into_iter().flat_map(|(o, hs)| std::iter::once(o).chain(hs)).collect();
    // tool centres run inside each leftover piece (it is all material, where
    // they may go), loops at most a radius apart: every point of it is then
    // within a radius of one
    let left = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        use geo::BooleanOps;
        loops_region(&area).difference(&swept)
    }))
    .unwrap_or_else(|_| geo::MultiPolygon::new(vec![]));
    let mut rest: Vec<Vec<P2>> = vec![];
    for p in &left.0 {
        let ext = from_geo_ring(p.exterior());
        if ext.len() < 3 || signed_area(&ext).abs() < 1e-8 {
            continue;
        }
        let hs: Vec<Vec<P2>> = p.interiors().iter().map(from_geo_ring).collect();
        let mut k = 0;
        loop {
            let ls = offset_region(&ext, &hs, 1e-4 + k as f64 * step.min(r));
            if ls.is_empty() || k > 10_000 {
                break;
            }
            rest.extend(ls);
            k += 1;
        }
    }
    rings.push(rest);
    let mut runs = vec![];
    let mut here: Option<P2> = None;
    for ring in rings {
        let mut left: Vec<Vec<P2>> = ring
            .into_iter()
            .map(|mut l| {
                // outer loops (CCW from the offset) go CW, holes CCW
                l.reverse();
                l
            })
            .collect();
        while !left.is_empty() {
            let h = here.unwrap_or(left[0][0]);
            let k = (0..left.len()).min_by(|&a, &b| near_dist(&left[a], h).total_cmp(&near_dist(&left[b], h))).unwrap();
            let seq = start_near(&left.swap_remove(k), h);
            here = Some(seq[0]);
            runs.push(seq);
        }
    }
    runs
}

/// Square spiral facing: rectangles shrinking from the area's bounding box a
/// stepover at a time, each CW from its top-left corner (climb with the
/// uncut middle inside), ending with a line down the middle; clipped to the
/// area.
fn square_spiral_runs(centres: &[Vec<P2>], step: f64) -> Vec<Vec<P2>> {
    let (mut x0, mut y0, mut x1, mut y1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in centres.iter().flatten() {
        (x0, y0, x1, y1) = (x0.min(p[0]), y0.min(p[1]), x1.max(p[0]), y1.max(p[1]));
    }
    if !x0.is_finite() {
        return vec![];
    }
    let mut paths: Vec<Vec<P2>> = vec![];
    let mut k = 0;
    loop {
        let e = 1e-3 + k as f64 * step;
        let (a, b, c, d) = (x0 + e, y0 + e, x1 - e, y1 - e);
        if c - a <= 1e-9 || d - b <= 1e-9 || k > 100_000 {
            // what is left is narrower than a step: a line down its middle
            let (a, b, c, d) = (x0 + e.min((x1 - x0) / 2.0), y0 + e.min((y1 - y0) / 2.0), x1 - e.min((x1 - x0) / 2.0), y1 - e.min((y1 - y0) / 2.0));
            let (pa, pb) = if x1 - x0 >= y1 - y0 { ([a, (b + d) / 2.0], [c, (b + d) / 2.0]) } else { ([(a + c) / 2.0, d], [(a + c) / 2.0, b]) };
            if d2(pa, pb) > 1e-6 && (k == 0 || paths.last().map_or(true, |l| d2(l[0], pa) > 1e-6)) {
                paths.push(vec![pa, pb]);
            }
            break;
        }
        paths.push(vec![[a, d], [c, d], [c, b], [a, b], [a, d]]);
        k += 1;
    }
    paths.iter().flat_map(|p| clip_polyline(p, centres)).collect()
}

/// The parts of polyline `p` inside the area (even-odd loops).
fn clip_polyline(p: &[P2], loops: &[Vec<P2>]) -> Vec<Vec<P2>> {
    let mut runs: Vec<Vec<P2>> = vec![];
    let mut cur: Vec<P2> = vec![];
    for w in p.windows(2) {
        let (a, b) = (w[0], w[1]);
        let mut ts = vec![0.0, 1.0];
        for l in loops {
            for k in 0..l.len() {
                let (c, d) = (l[k], l[(k + 1) % l.len()]);
                let (rx, ry, sx, sy) = (b[0] - a[0], b[1] - a[1], d[0] - c[0], d[1] - c[1]);
                let den = rx * sy - ry * sx;
                if den.abs() < 1e-15 {
                    continue;
                }
                let t = ((c[0] - a[0]) * sy - (c[1] - a[1]) * sx) / den;
                let u = ((c[0] - a[0]) * ry - (c[1] - a[1]) * rx) / den;
                if t > 0.0 && t < 1.0 && (0.0..=1.0).contains(&u) {
                    ts.push(t);
                }
            }
        }
        ts.sort_by(|x, y| x.total_cmp(y));
        let at = |t: f64| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        for t in ts.windows(2) {
            if t[1] - t[0] < 1e-12 {
                continue;
            }
            let (q0, q1) = (at(t[0]), at(t[1]));
            if inside(loops, at((t[0] + t[1]) / 2.0)) {
                if cur.is_empty() {
                    cur.push(q0);
                } else if d2(*cur.last().unwrap(), q0) > 1e-9 {
                    runs.push(std::mem::take(&mut cur));
                    cur.push(q0);
                }
                cur.push(q1);
            } else if !cur.is_empty() {
                runs.push(std::mem::take(&mut cur));
            }
        }
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    runs.into_iter().filter(|r| r.len() >= 2 && path_len(r) > 1e-6).collect()
}

// ------------------------------------------------------------------ engrave
/// Each glyph contour is followed at every depth level before moving on to
/// the next one; the cutter plunges at the contour start.
fn engrave_moves(op: &EngraveOp, retract: f64) -> Result<Vec<Mv>, String> {
    let paths = engrave::text_paths(op)?;
    let lift = op.z + 0.5;
    let plunge = (op.feed * 0.5).max(1.0);
    // nearest contour next, entered at its nearest point (closed loops)
    let first = paths[0][0];
    let paths = nearest_first(paths, first, |c, p| near_dist(c, p), |c, p| {
        let i = (0..c.len()).min_by(|&a, &b| d2(c[a], p).total_cmp(&d2(c[b], p))).unwrap();
        c[i]
    });
    let mut here = first;
    let paths: Vec<Vec<P2>> = paths
        .into_iter()
        .map(|c| {
            let open: Vec<P2> = c[..c.len() - 1].to_vec(); // drop the closing point
            let l = start_near(&open, here);
            here = l[0];
            l
        })
        .collect();
    let mut out = Path::new([first[0], first[1], retract]);
    for c in &paths {
        let p0 = c[0];
        out.reposition(retract, p0, lift);
        for z in levels(op.z, op.bottom(), op.depth_per_pass) {
            out.feed([p0[0], p0[1], z], plunge);
            for p in &c[1..] {
                out.feed([p[0], p[1], z], op.feed);
            }
        }
    }
    out.rapid([out.pos[0], out.pos[1], retract]);
    Ok(out.moves)
}

// ------------------------------------------------------------------ profile
/// Centre paths for a profile: closed paths end where they start.
pub fn profile_paths(op: &ProfileOp) -> Result<Vec<Vec<P2>>, String> {
    let r = op.cutter.dia() / 2.0;
    let mut path = op.path.clone();
    path.dedup_by(|a, b| d2(*a, *b) < 1e-9);
    if path.len() < 2 || path_len(&path) < 1e-6 {
        return Err("edge chain is empty".into());
    }
    if !op.closed {
        if op.side != ProfileSide::Center {
            return Err("inside / outside needs a closed loop of edges - use centre line for open chains".into());
        }
        return Ok(vec![path]);
    }
    let first = path[0];
    let loops = match op.side {
        ProfileSide::Center => {
            if d2(path[0], *path.last().unwrap()) < 1e-9 {
                path.pop();
            }
            if path.len() < 2 {
                return Err("edge chain is empty".into());
            }
            if path.len() >= 3 && signed_area(&path) < 0.0 {
                path.reverse();
            }
            vec![path]
        }
        ProfileSide::Inside => offset_region(&path, &[], r),
        ProfileSide::Outside => offset_region(&path, &[], -r),
    };
    if loops.is_empty() {
        return Err(format!("cutter Ø{} does not fit inside the loop", fmt(op.cutter.dia())));
    }
    Ok(loops
        .into_iter()
        .filter(|l| signed_area(l) >= 0.0 || op.side == ProfileSide::Center)
        .map(|l| {
            let mut v = start_near(&l, first);
            if op.side == ProfileSide::Outside {
                v.reverse(); // CW around the outside = climb with M3
            }
            v
        })
        .collect())
}

/// Feed `dist` along `seq` (closed: round and round; open: back and forth),
/// with Z given by distance travelled.
fn walk(out: &mut Path, seq: &[P2], closed: bool, dist: f64, z_of: &dyn Fn(f64) -> f64, feed: f64) {
    let m = seq.len() - 1;
    let v = |k: usize| {
        if closed {
            seq[k % m]
        } else {
            let r = k % (2 * m);
            seq[if r <= m { r } else { 2 * m - r }]
        }
    };
    let mut d = 0.0;
    let mut k = 0;
    while d < dist - 1e-9 && k < 10_000_000 {
        let (a, b) = (v(k), v(k + 1));
        let l = d2(a, b);
        k += 1;
        if l < 1e-12 {
            continue;
        }
        if d + l >= dist - 1e-9 {
            let t = ((dist - d) / l).min(1.0);
            out.feed([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, z_of(dist)], feed);
            return;
        }
        d += l;
        out.feed([b[0], b[1], z_of(d)], feed);
    }
}

fn profile_moves(op: &ProfileOp, retract: f64) -> Result<Vec<Mv>, String> {
    let paths = profile_paths(op)?;
    let tan = op.ramp_angle.to_radians().tan().max(1e-3);
    let mut out = Path::new([0.0, 0.0, retract]);
    let ends = |s: &Vec<P2>| (s[0], *s.last().unwrap());
    let paths: Vec<Vec<P2>> = if paths.iter().any(|p| p.len() < 2) { paths } else {
        nearest_first(paths, [0.0, 0.0], |s, p| if op.closed { near_dist(s, p) } else { d2(ends(s).0, p).min(d2(ends(s).1, p)) }, |s, _| ends(s).1)
    };
    let mut here = [0.0, 0.0];
    for mut seq in paths {
        if seq.len() >= 2 {
            if op.closed && seq.len() > 2 {
                seq = start_near(&seq[..seq.len() - 1], here);
            } else if !op.closed && d2(seq[seq.len() - 1], here) < d2(seq[0], here) && op.side == ProfileSide::Center {
                seq.reverse();
            }
            here = *seq.last().unwrap();
        }
        if seq.len() < 2 {
            continue;
        }
        let closed = op.closed;
        let len = path_len(&seq);
        if len < 1e-6 {
            continue;
        }
        out.reposition(retract, seq[0], op.start_z + 0.5_f64.min(retract - op.start_z));
        out.feed([seq[0][0], seq[0][1], op.start_z], (op.feed * 0.3).max(1.0));
        let mut z_prev = op.start_z;
        for z in levels(op.start_z, op.z, op.depth_per_pass) {
            let ramp = (z_prev - z) / tan;
            let z_of = |d: f64| (z_prev - d * tan).max(z);
            if closed {
                // ramp down round the loop, then a full loop at depth, ending at the start
                let dist = ((ramp + len) / len - 1e-9).ceil() * len;
                walk(&mut out, &seq, true, dist, &z_of, op.feed);
            } else {
                // ramp back and forth, then a full pass at depth ending at an end
                let passes = (ramp / len - 1e-9).ceil().max(0.0) + 1.0;
                walk(&mut out, &seq, false, passes * len, &z_of, op.feed);
                if passes as usize % 2 == 1 {
                    seq.reverse(); // we are at the far end
                }
            }
            z_prev = z;
        }
        out.rapid([out.pos[0], out.pos[1], retract]);
    }
    if out.moves.is_empty() {
        return Err("profile produced no toolpath".into());
    }
    Ok(out.moves)
}

// ------------------------------------------------------------------ roughing
fn to_geo(outer: &[P2], holes: &[Vec<P2>]) -> geo::Polygon<f64> {
    let ls = |p: &[P2]| geo::LineString::from(p.iter().map(|q| (q[0], q[1])).collect::<Vec<_>>());
    geo::Polygon::new(ls(outer), holes.iter().map(|h| ls(h)).collect())
}

fn from_geo_ring(ls: &geo::LineString<f64>) -> Vec<P2> {
    let mut v: Vec<P2> = ls.coords().map(|c| [c.x, c.y]).collect();
    if v.len() > 1 && d2(v[0], *v.last().unwrap()) < 1e-12 {
        v.pop();
    }
    v
}

/// Z levels for roughing: even layers plus every protected floor (+ allowance).
pub fn rough_levels(op: &RoughOp) -> Vec<f64> {
    let bottom = op.final_z();
    let mut zs = levels(op.start_z, bottom, op.layer_height);
    for f in &op.faces {
        for z in [f.z, f.low()] {
            let z = z + op.allowance.max(0.0);
            if z < op.start_z - 1e-6 && z > bottom + 1e-6 {
                zs.push(z);
            }
        }
    }
    zs.sort_by(|a, b| b.total_cmp(a));
    zs.dedup_by(|a, b| (*a - *b).abs() < 0.01);
    zs
}

/// How far past the part outline the clearing may reach (beyond tool
/// radius + allowance), so open edges are cut cleanly without leaving a fin.
const OPEN_EDGE: f64 = 0.05;

fn union_all(polys: impl IntoIterator<Item = geo::Polygon<f64>>) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let mut all = geo::MultiPolygon::<f64>::new(vec![]);
    for p in polys {
        all = all.union(&geo::MultiPolygon::new(vec![p]));
    }
    all
}

/// One layer of roughing: areas to clear (outer, islands = the part's walls).
pub struct RoughLayer {
    pub regions: Vec<(Vec<P2>, Vec<Vec<P2>>)>,
    /// The clearing's outer limit at this layer (not a wall of the part).
    pub limit: Vec<Vec<P2>>,
    /// Holes that go right through the part (no face above them at any Z).
    pub through: Vec<Vec<P2>>,
    /// `regions` less the through holes: only areas with a floor (a face
    /// below the layer) - cleared even without "clear voids".
    pub floored: Vec<(Vec<P2>, Vec<Vec<P2>>)>,
}

/// How much clipped mesh triangles are grown so neighbours overlap and
/// merge into one region (material side: conservative).
const MESH_GROW: f64 = 0.06;

/// Simplification of mesh outlines (well inside MESH_GROW).
const MESH_SIMPLIFY: f64 = 0.01;

/// Holes in the material smaller than this (mm²) are gaps between mesh
/// pieces, not real holes: far too small for any cutter.
const PINHOLE: f64 = 0.05;

/// A convex CCW polygon grown by `eps` along its edge normals (miter
/// corners, capped at 4 x eps so sliver tips don't spike).
fn grow_convex(q: &[P2], eps: f64) -> Vec<P2> {
    let n = q.len();
    let normal = |a: P2, b: P2| {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy).max(1e-12);
        [dy / l, -dx / l] // outward for CCW
    };
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let (p, c, nx) = (q[(i + n - 1) % n], q[i], q[(i + 1) % n]);
        let (n1, n2) = (normal(p, c), normal(c, nx));
        let dot = n1[0] * n2[0] + n1[1] * n2[1];
        let m = [n1[0] + n2[0], n1[1] + n2[1]];
        let k = eps / (1.0 + dot).max(1e-9);
        // mitre only mild corners: slivers' sharp corners would spike out
        // (finishing laps follow these outlines)
        if (m[0] * k).hypot(m[1] * k) <= 1.5 * eps {
            out.push([c[0] + m[0] * k, c[1] + m[1] * k]);
        } else {
            // sharp corner: bevel with both edge offsets
            out.push([c[0] + n1[0] * eps, c[1] + n1[1] * eps]);
            out.push([c[0] + n2[0] * eps, c[1] + n2[1] * eps]);
        }
    }
    out
}

/// The parts of mesh triangles above `z` (all of them for `z` = -inf),
/// seen from above, grown slightly so they join up.
fn mesh_above(tris: &[[P3; 3]], z: f64) -> Vec<geo::Polygon<f64>> {
    let mut out = vec![];
    for t in tris {
        if t.iter().all(|p| p[2] <= z) {
            continue;
        }
        let mut q: Vec<P2> = vec![];
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            let (ia, ib) = (a[2] > z, b[2] > z);
            if ia {
                q.push([a[0], a[1]]);
            }
            if ia != ib {
                let s = (z - a[2]) / (b[2] - a[2]);
                q.push([a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s]);
            }
        }
        if q.len() < 3 || signed_area(&q).abs() < 1e-8 {
            continue; // seen edge-on
        }
        if signed_area(&q) < 0.0 {
            q.reverse();
        }
        out.push(to_geo(&grow_convex(&q, MESH_GROW), &[]));
    }
    out
}

/// Inside-only roughing: material inside the outline narrower than twice this
/// is left (it is mostly mesh slivers along vertical outside walls).
const INSIDE_SLIVER: f64 = 0.1;

/// A region (CCW outers, CW holes from an offset) as a multipolygon.
fn loops_region(ls: &[Vec<P2>]) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let outers = union_all(ls.iter().filter(|l| signed_area(l) > 0.0).map(|l| to_geo(l, &[])));
    let holes = union_all(ls.iter().filter(|l| signed_area(l) < 0.0).map(|l| to_geo(&l.iter().rev().copied().collect::<Vec<_>>(), &[])));
    outers.difference(&holes)
}

/// Each polygon of `m` offset by `d` (> 0 shrinks, < 0 grows), unioned.
fn offset_multi(m: &geo::MultiPolygon<f64>, d: f64) -> geo::MultiPolygon<f64> {
    use geo::BooleanOps;
    let mut all = geo::MultiPolygon::<f64>::new(vec![]);
    for p in &m.0 {
        let ext = from_geo_ring(p.exterior());
        if ext.len() < 3 {
            continue;
        }
        let hs: Vec<Vec<P2>> = p.interiors().iter().map(from_geo_ring).collect();
        let r = loops_region(&offset_region(&ext, &hs, d));
        all = if all.0.is_empty() { r } else { all.union(&r) };
    }
    all
}

/// Per-op geometry that does not depend on the layer.
pub struct RoughGeom {
    /// Each face seen from above (curved faces: the union of their mesh).
    full: Vec<geo::MultiPolygon<f64>>,
    /// Area cleared at each layer (less the material there): the part's
    /// outside profile grown so the tool centre reaches half a tool diameter
    /// (+ stock to leave + OPEN_EDGE) outside it - enough to run round the
    /// outer walls; stock further out is left.
    grown: geo::MultiPolygon<f64>,
    /// The part's outside profile seen from above (holes filled).
    outline: geo::MultiPolygon<f64>,
    through: Vec<Vec<P2>>,
    voids: geo::MultiPolygon<f64>,
}

pub fn rough_geom(op: &RoughOp) -> Result<RoughGeom, String> {
    let a = op.allowance.max(0.0);
    // the clearing keeps the tool centre r + a inside this boundary, so grow
    // by 2(r + a) + OPEN_EDGE for centres out to profile + r + a + OPEN_EDGE;
    // adaptive clearing also needs room for its trochoidal loops round the
    // outside profile (else it would slot it full width)
    let g = op.cutter.dia() + 2.0 * a + OPEN_EDGE + op.band_extra();
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let full: Vec<geo::MultiPolygon<f64>> = op
            .faces
            .iter()
            .map(|f| {
                use geo::Simplify;
                if f.curved {
                    union_tree(mesh_above(&f.tris, f64::NEG_INFINITY)).simplify(&MESH_SIMPLIFY)
                } else {
                    geo::MultiPolygon::new(vec![to_geo(&f.outer, &f.holes)])
                }
            })
            .collect();
        // part outline: every face seen from above, holes filled
        let outline = union_tree(full.iter().flat_map(|m| m.0.iter().map(|p| geo::Polygon::new(p.exterior().clone(), vec![]))).collect());
        let grown = union_all(
            outline
                .0
                .iter()
                .flat_map(|p| offset_region(&from_geo_ring(p.exterior()), &[], -g))
                .filter(|l| signed_area(l) > 0.0)
                .map(|l| to_geo(&l, &[])),
        );
        let all = union_tree(full.iter().flat_map(|m| m.0.iter().cloned()).collect());
        let through: Vec<Vec<P2>> = all.0.iter().flat_map(|p| p.interiors().iter().map(from_geo_ring)).filter(|h| h.len() >= 3 && signed_area(h).abs() > 1e-3).collect();
        // Through holes shrunk by twice the allowance (+ a hair) so the floored
        // clearing meets the through-hole profile without leaving a thin wall.
        let voids = union_all(
            through
                .iter()
                .flat_map(|h| offset_region(h, &[], 2.0 * a + OPEN_EDGE))
                .filter(|l| signed_area(l) > 0.0)
                .map(|l| to_geo(&l, &[])),
        );
        RoughGeom { full, grown, outline, through, voids }
    }))
    .map_err(|_| "could not work out the part outline".to_string())
}

/// Work out layer `z`. The part's outside profile (all faces seen from
/// above, holes filled) grown so the tool centre reaches half a tool diameter
/// past it bounds the clearing; material is every face above the layer (less
/// the allowance) - only the part above the layer for sloped and curved
/// faces. The material is the clearing's islands, so its walls are cut by the
/// clearing's contour pass; stock further out than the band is left.
#[cfg(test)]
pub fn rough_layer(op: &RoughOp, z: f64) -> Result<RoughLayer, String> {
    rough_layer_in(op, &rough_geom(op)?, z)
}

pub fn rough_layer_in(op: &RoughOp, geom: &RoughGeom, z: f64) -> Result<RoughLayer, String> {
    use geo::BooleanOps;
    let a = op.allowance.max(0.0);
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // material at this layer: flat faces above it, and the part of each
        // sloped / curved face that is above it
        let zc = z - a + 1e-6;
        let mut polys: Vec<geo::Polygon<f64>> = vec![];
        for (f, full) in op.faces.iter().zip(&geom.full) {
            if f.z <= zc {
                continue;
            }
            if f.tris.is_empty() || f.low() > zc {
                polys.extend(full.0.iter().cloned());
            } else if f.curved {
                polys.extend(mesh_above(&f.tris, zc));
            } else {
                polys.extend(clip_face(&f.outer, &f.holes, &f.tris, zc, false).0);
            }
        }
        // mesh slices have many tiny edges: simplify (within less than MESH_GROW),
        // and fill pinholes left between mesh pieces (material either side)
        let keep = {
            use geo::Simplify;
            let k = union_tree(polys).simplify(&MESH_SIMPLIFY);
            geo::MultiPolygon::new(
                k.0.into_iter()
                    .map(|p| {
                        let (ext, holes) = p.into_inner();
                        geo::Polygon::new(ext, holes.into_iter().filter(|h| signed_area(&from_geo_ring(h)).abs() > PINHOLE).collect())
                    })
                    .collect(),
            )
        };
        let (cut, limit) = if op.inside_only {
            // inside only: the material inside the outline (less slivers),
            // with tool centres up to a radius past it - so a chamfer or
            // fillet narrower than the tool is still cleared to the edge,
            // and nothing is cut beside a vertical outside wall
            use geo::Simplify;
            let inside = geom.outline.difference(&keep);
            let opened = offset_multi(&offset_multi(&inside, INSIDE_SLIVER), -INSIDE_SLIVER);
            let limit = offset_multi(&opened, -(op.cutter.dia() + a)).simplify(&MESH_SIMPLIFY);
            (limit.difference(&keep), limit)
        } else {
            (geom.grown.difference(&keep), geom.grown.clone())
        };
        let floored = cut.difference(&geom.voids);
        (cut, floored, limit)
    }))
    .map_err(|_| format!("could not work out the part outline at Z{}", fmt(z)))?;
    let (cut, floored, limit) = res;
    let regions_of = |m: &geo::MultiPolygon<f64>| -> Vec<(Vec<P2>, Vec<Vec<P2>>)> {
        m.0.iter()
            .map(|p| (from_geo_ring(p.exterior()), p.interiors().iter().map(from_geo_ring).collect::<Vec<_>>()))
            .filter(|(o, _)| o.len() >= 3)
            .collect()
    };
    Ok(RoughLayer {
        regions: regions_of(&cut),
        floored: regions_of(&floored),
        through: geom.through.clone(),
        limit: limit.0.iter().map(|p| from_geo_ring(p.exterior())).collect(),
    })
}

/// One closed loop (already closed, in cutting direction) from `z_from` down
/// to `z`, ramping along it, then a full lap at depth; retracts at the end.
fn ramp_loop(seq: &[P2], z_from: f64, z: f64, feed: f64, tan: f64, retract: f64) -> Vec<Mv> {
    let len = path_len(seq);
    if seq.len() < 2 || len < 1e-6 {
        return vec![];
    }
    let mut path = Path::new([seq[0][0], seq[0][1], retract]);
    path.reposition(retract, seq[0], z_from + 0.5_f64.min(retract - z_from));
    path.feed([seq[0][0], seq[0][1], z_from], feed);
    let ramp = (z_from - z) / tan;
    let dist = ((ramp + len) / len - 1e-9).ceil() * len;
    walk(&mut path, seq, true, dist, &|d: f64| (z_from - d * tan).max(z), feed);
    path.rapid([path.pos[0], path.pos[1], retract]);
    path.moves
}

impl RoughOp {
    /// How much wider than one tool diameter (+ stock to leave) the band
    /// cleared round the outside profile is: adaptive clearing needs room for
    /// its trochoidal loops (else it would slot the profile full width).
    pub fn band_extra(&self) -> f64 {
        if self.inside_only {
            0.0
        } else if matches!(self.strategy, Strategy::Adaptive | Strategy::Mixed) {
            2.0 * TROCHOID_R * self.cutter.dia() + OPEN_EDGE
        } else {
            0.0
        }
    }

    /// Highest point of the part.
    pub fn part_top(&self) -> f64 {
        self.faces.iter().map(|f| f.z).fold(f64::NEG_INFINITY, f64::max)
    }
    /// Z the stock above the part is faced down to (part top + stock to
    /// leave), when the stock is taller than that.
    pub fn facing_z(&self) -> Option<f64> {
        let z = self.part_top() + self.allowance.max(0.0);
        (z.is_finite() && self.start_z > z + 1e-6).then_some(z)
    }
    /// The area faced: the stock, or the part outline's box without one.
    fn facing_region(&self) -> [f64; 4] {
        let [x0, y0, x1, y1] = self.stock_region;
        if x1 - x0 > 1e-6 && y1 - y0 > 1e-6 {
            return self.stock_region;
        }
        self.faces.iter().flat_map(|f| f.outer.iter()).fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |b, p| {
            [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])]
        })
    }
}

fn rough_moves(op: &RoughOp, retract: f64) -> Result<Vec<Mv>, String> {
    let r = op.cutter.dia() / 2.0;
    let a = op.allowance.max(0.0);
    let tan = op.ramp_angle.to_radians().tan().max(1e-3);
    let mut out = vec![];
    let mut z_prev = op.start_z;
    // stock taller than the part: face down to the part top first, over the
    // same band as the layers (outline + half a tool diameter)
    if let Some(fz) = op.facing_z() {
        let outline = part_outline(&op.faces);
        let cut = FacingCut { cutter: op.cutter, stepover_pct: op.stepover_pct, start_z: op.start_z, z: fz, depth_per_pass: op.layer_height, feed: op.feed, edge_lap: false, pattern: op.facing };
        if outline.is_empty() {
            let [x0, y0, x1, y1] = op.facing_region();
            let e = r + OPEN_EDGE;
            let rect = vec![vec![[x0 - e, y0 - e], [x1 + e, y0 - e], [x1 + e, y1 + e], [x0 - e, y1 + e]]];
            out.extend(facing_moves(&rect, &[vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]], cut, retract));
        } else {
            // over the same band the layers clear, rows across all of it
            // (inside only: tool centres out to a radius past the outline,
            // as far as the layers' centres go round a chamfer or fillet)
            let band = if op.inside_only { grow_loops(&outline, r + a) } else { grow_loops(&outline, r + a + OPEN_EDGE + op.band_extra()) };
            // and a lap round its edge: the layers below may hop just above it
            out.extend(facing_moves(&band, &band, FacingCut { edge_lap: true, ..cut }, retract));
        }
        z_prev = fz;
    }
    let geom = rough_geom(op)?;
    // the area faced (hops just above the faced height stay over it)
    let band: Vec<Vec<P2>> = if op.inside_only { grow_loops(&part_outline(&op.faces), r + a) } else { geom.grown.0.iter().map(|p| from_geo_ring(p.exterior())).collect() };
    let below = z_prev; // levels above the faced height are already done
    let faced = op.facing_z().is_some();
    let zs: Vec<f64> = rough_levels(op).into_iter().filter(|&z| z < below - 1e-6).collect();
    // Z a layer starts from (layer -1: the stock top / faced height)
    let z_of = |m: isize| if m < 0 { below } else { zs[m as usize] };

    // every cut of every layer, each one pocket / band / hole at one depth
    enum Cut {
        /// CCW loop round a through hole (its core is left).
        Loop(Vec<P2>),
        /// Area, its islands, and whether its outer boundary is the
        /// clearing's limit (not a wall).
        Clear(Vec<P2>, Vec<Vec<P2>>, bool),
    }
    struct RJob {
        layer: usize,
        cut: Cut,
        /// Material the cut takes out (for what it must wait for).
        area: geo::MultiPolygon<f64>,
        bbox: [f64; 4],
        /// Where the tool may travel low once it is cut (tool centres
        /// r + stock to leave off the walls).
        safe: Vec<Vec<Vec<P2>>>,
        deps: Vec<usize>,
    }
    let bbox_of = |ls: &[Vec<P2>]| {
        ls.iter().flatten().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |b, p| [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])])
    };
    let mut jobs: Vec<RJob> = vec![];
    // first job of each layer
    let mut layer_start = vec![];
    for (k, &z) in zs.iter().enumerate() {
        layer_start.push(jobs.len());
        let layer = rough_layer_in(op, &geom, z)?;
        if !op.clear_voids {
            // without clear voids, areas over a through hole are left and the hole is only profiled
            for h in &layer.through {
                // CCW inside the hole = climb
                for l in offset_region(h, &[], r + a).into_iter().filter(|l| signed_area(l) > 0.0) {
                    let grown = offset_region(&l, &[], -r);
                    let area = union_all(grown.iter().filter(|g| signed_area(g) > 0.0).map(|g| to_geo(g, &[])));
                    jobs.push(RJob { layer: k, bbox: bbox_of(&grown), area, cut: Cut::Loop(l), safe: vec![], deps: vec![] });
                }
            }
        }
        let regions = if op.clear_voids { layer.regions } else { layer.floored };
        for (o, i) in regions {
            // (checked to a micron: the tool may leave from the wall contour,
            // corners and all)
            let safe: Vec<Vec<Vec<P2>>> = vec![offset_region(&o, &i, r + a)].into_iter().filter(|l| !l.is_empty()).collect();
            let open = o.iter().take(3).all(|&p| boundary_dist(&layer.limit, p) < 1e-4);
            jobs.push(RJob { layer: k, bbox: bbox_of(std::slice::from_ref(&o)), area: geo::MultiPolygon::new(vec![to_geo(&o, &i)]), cut: Cut::Clear(o, i, open), safe, deps: vec![] });
        }
    }
    layer_start.push(jobs.len());
    // a cut waits for the cuts of the layer above that it lies under (the
    // nearest layer above with any; under none of them: all of them)
    {
        use geo::Intersects;
        for j in 0..jobs.len() {
            let k = jobs[j].layer;
            let Some(up) = (0..k).rev().find(|&m| layer_start[m + 1] > layer_start[m]) else { continue };
            let (b, area) = (jobs[j].bbox, &jobs[j].area);
            let over = |q: &RJob| q.bbox[0] <= b[2] + 1e-6 && b[0] <= q.bbox[2] + 1e-6 && q.bbox[1] <= b[3] + 1e-6 && b[1] <= q.bbox[3] + 1e-6 && q.area.intersects(area);
            let mut deps: Vec<usize> = (layer_start[up]..layer_start[up + 1]).filter(|&q| over(&jobs[q])).collect();
            if deps.is_empty() {
                deps = (layer_start[up]..layer_start[up + 1]).collect();
            }
            jobs[j].deps = deps;
        }
    }

    // depth first: each pocket is cut all the way down before the next; next
    // is a cut whose cuts above are all done, under the latest one cut (then
    // the nearest)
    let start3 = match out.last() {
        Some(Mv::Rapid(p)) | Some(Mv::Feed(p, _)) => *p,
        None => [0.0, 0.0, retract],
    };
    let n0 = out.len();
    let mut here = end_xy(&out, [0.0, 0.0]);
    let mut rank: Vec<Option<usize>> = vec![None; jobs.len()];
    // job of each move cut
    let mut owner: Vec<usize> = vec![];
    for n in 0..jobs.len() {
        let next = (0..jobs.len())
            .filter(|&j| rank[j].is_none() && jobs[j].deps.iter().all(|&d| rank[d].is_some()))
            .map(|j| {
                let latest = jobs[j].deps.iter().map(|&d| rank[d].unwrap() as isize).max().unwrap_or(-1);
                let start = match &jobs[j].cut {
                    Cut::Loop(l) | Cut::Clear(l, _, _) => near_dist(l, here),
                };
                (j, latest, start)
            })
            .min_by(|x, y| y.1.cmp(&x.1).then(x.2.total_cmp(&y.2)));
        let Some((j, _, _)) = next else { break };
        rank[j] = Some(n);
        let k = jobs[j].layer;
        let (z, z_from) = (zs[k], z_of(k as isize - 1));
        let moves = match &jobs[j].cut {
            Cut::Loop(l) => ramp_loop(&start_near(l, here), z_from, z, op.feed, tan, retract),
            Cut::Clear(outer, islands, open) => {
                let open_outer = *open;
                // mixed: adaptive round the outline, standard inside the part
                let strategy = match op.strategy {
                    Strategy::Mixed if open_outer => Strategy::Adaptive,
                    Strategy::Mixed => Strategy::Standard,
                    s => s,
                };
                let pocket = PocketOp {
                    no_contour: !op.finish_layers,
                    open_outer,
                    z,
                    start_z: z_from,
                    depth_per_pass: z_from - z + 1.0,
                    outer: outer.clone(),
                    islands: islands.clone(),
                    keep_islands: true,
                    finish: a,
                    finish_pass: false,
                    cutter: op.cutter,
                    stepover_pct: op.stepover_pct,
                    adaptive_outer_only: op.strategy == Strategy::Mixed,
                    feed: op.feed,
                    speed: op.speed,
                    ramp_angle: op.ramp_angle,
                    strategy,
                    entry: op.entry,
                    ..Default::default()
                };
                // areas too narrow for the cutter (e.g. just the open edge) are left
                pocket_moves(&pocket, retract, here).unwrap_or_default()
            }
        };
        owner.extend(std::iter::repeat(j).take(moves.len()));
        out.extend(moves);
        here = end_xy(&out, here);
    }

    // hops between cuts stay low where they only cross cleared ground: at the
    // deepest layer cleared (so far) all along the hop - nothing stands above
    // it there. A hop within a cut: the layer above it. Layer -1: the stock
    // top / faced height, clear everywhere (faced: over the faced band).
    let mut cut: Vec<Mv> = out.split_off(n0);
    // a cut ending with a retract and the next starting with one repeat it
    let mut k = 1;
    while k < cut.len() {
        if matches!((cut[k - 1], cut[k]), (Mv::Rapid(p), Mv::Rapid(q)) if p == q) {
            cut.remove(k);
            owner.remove(k);
        } else {
            k += 1;
        }
    }
    let ok = |m: isize, before: usize, p: P2, q: P2| {
        if m < 0 {
            return !faced || seg_in(std::slice::from_ref(&band), p, q);
        }
        let m = m as usize;
        let safe: Vec<Vec<Vec<P2>>> = (layer_start[m]..layer_start[m + 1]).filter(|&q| rank[q].map_or(false, |r| r < before)).flat_map(|q| jobs[q].safe.iter().cloned()).collect();
        !safe.is_empty() && seg_in_tol(&safe, p, q, HOP_TOL)
    };
    let mut pos = start3;
    let mut pos_owner: Option<usize> = None;
    let mut i = 0;
    while i < cut.len() {
        // up and straight back down (the travel deduped away): the travel is nil
        if let (Mv::Rapid(a3), Some(Mv::Rapid(c))) = (cut[i], cut.get(i + 1).copied()) {
            let from_below = pos[2] < retract - 1e-9 && (a3[0] - pos[0]).abs() < 1e-9 && (a3[1] - pos[1]).abs() < 1e-9;
            if from_below && (a3[2] - retract).abs() < 1e-9 && c[2] < retract - 1e-9 && (a3[0] - c[0]).abs() < 1e-9 && (a3[1] - c[1]).abs() < 1e-9 {
                cut.insert(i + 1, Mv::Rapid(a3));
                owner.insert(i + 1, owner[i + 1]);
            }
        }
        if let (Mv::Rapid(a3), Some(Mv::Rapid(b)), Some(Mv::Rapid(c))) = (cut[i], cut.get(i + 1).copied(), cut.get(i + 2).copied()) {
            let up = (a3[0] - pos[0]).abs() < 1e-9 && (a3[1] - pos[1]).abs() < 1e-9 && (a3[2] - retract).abs() < 1e-9;
            let across = (b[2] - retract).abs() < 1e-9;
            let down = (c[0] - b[0]).abs() < 1e-9 && (c[1] - b[1]).abs() < 1e-9;
            if up && across && down {
                let j = owner[i + 2];
                let k = jobs[j].layer as isize;
                let before = rank[j].unwrap();
                // a hop into a new cut may go over any layer cleared; one within a cut, its own
                let lowest = if pos_owner == Some(j) { k - 1 } else { -1 };
                // going down a pocket: the tool is on the floor of the layer
                // above, which it may cross at that depth - feed over there
                // instead of hopping up and back
                if pos_owner != Some(j) && k > 0 && (pos[2] - z_of(k - 1)).abs() < 1e-9 && c[2] >= pos[2] - 1e-9 && (d2([pos[0], pos[1]], [b[0], b[1]]) < 1e-9 || ok(k - 1, before, [pos[0], pos[1]], [b[0], b[1]])) {
                    if d2([pos[0], pos[1]], [b[0], b[1]]) < 1e-9 {
                        // straight back down where it is: no hop at all
                        cut.drain(i..i + 3);
                        owner.drain(i..i + 3);
                    } else {
                        cut[i] = Mv::Feed([b[0], b[1], pos[2]], op.feed);
                        cut.drain(i + 1..i + 3);
                        owner.drain(i + 1..i + 3);
                        pos = [b[0], b[1], pos[2]];
                        i += 1;
                    }
                    pos_owner = Some(j);
                    // the cut's own drop to its start depth is now a no-op
                    while i < cut.len() && matches!(cut[i], Mv::Feed(q, _) if (q[0] - pos[0]).abs() + (q[1] - pos[1]).abs() + (q[2] - pos[2]).abs() < 1e-9) {
                        cut.remove(i);
                        owner.remove(i);
                    }
                    continue;
                }
                for m in (lowest..k).rev() {
                    let low = z_of(m) + 0.5;
                    if low >= retract - 1e-9 {
                        break;
                    }
                    if pos[2] <= low + 1e-9 && c[2] <= low + 1e-9 && ok(m, before, [pos[0], pos[1]], [b[0], b[1]]) {
                        cut[i] = Mv::Rapid([a3[0], a3[1], low]);
                        cut[i + 1] = Mv::Rapid([b[0], b[1], low]);
                        break;
                    }
                }
            }
        }
        pos = match cut[i] {
            Mv::Rapid(p) | Mv::Feed(p, _) => p,
        };
        pos_owner = Some(owner[i]);
        i += 1;
    }
    out.extend(cut);
    if out.is_empty() {
        return Err("roughing found nothing to cut".into());
    }
    Ok(out)
}

/// Default Z step of a new Finish op (mm).
pub const FINISH_STEP: f64 = 0.1;

/// Finish ops with no enabled Rough op before them: (op index, warning).
pub fn finish_warnings(ops: &[StepOp]) -> Vec<(usize, String)> {
    let mut roughed = false;
    let mut out = vec![];
    for (i, o) in ops.iter().enumerate().filter(|(_, o)| o.enabled) {
        match o.kind {
            OpKind::Rough(_) => roughed = true,
            OpKind::Finish(_) if !roughed => out.push((i, "no Rough op before this Finish: it only laps round the part walls, so uncut stock is left (and met) elsewhere".to_string())),
            _ => {}
        }
    }
    out
}

/// How far above a layer a Finish op may rapid down to before a lap: the
/// last enabled Rough op before it (with a cutter no bigger) left nothing
/// higher than one roughing layer above any finishing layer; without one,
/// only down to the Finish start.
fn finish_clearance(before: &[StepOp], fin: &RoughOp) -> Option<f64> {
    before.iter().rev().filter(|o| o.enabled).find_map(|o| match &o.kind {
        OpKind::Rough(r) if r.cutter.dia() <= fin.cutter.dia() + 1e-9 => Some(r.layer_height + r.allowance.max(0.0) + 0.5),
        _ => None,
    })
}

/// Finishing: at every layer (Z step `layer_height`, plus every flat floor
/// and the top and foot of every slope), one climb-milled lap round each of
/// the part's walls at tool radius + stock to leave - outside walls CW,
/// pocket and hole walls CCW. Nothing else is cut. Laps of the same wall on
/// consecutive layers link with a short feed move; other moves go up to the
/// retract height and come down to `clearance` above the layer (None: to the
/// op's start Z) before feeding down.
fn finish_moves(op: &RoughOp, retract: f64, clearance: Option<f64>) -> Result<Vec<Mv>, String> {
    let r = op.cutter.dia() / 2.0;
    let a = op.allowance.max(0.0);
    let geom = rough_geom(op)?;
    // the clearing band's outer limit is not part geometry
    let band: Vec<Vec<P2>> = geom.grown.0.iter().map(|p| from_geo_ring(p.exterior())).collect();
    // segment inside a set of allowed regions (tool centre clear of the part)
    let clear = |safe: &[Vec<Vec<P2>>], p: P2, q: P2| {
        let n = ((d2(p, q) / 0.1).ceil() as usize).max(1);
        (0..=n).all(|k| {
            let t = k as f64 / n as f64;
            let x = [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
            safe.iter().any(|l| inside(l, x) || boundary_dist(l, x) < 1e-3)
        })
    };
    // every lap of every layer; each wall is then finished all the way down
    // before the next (a lap waits for the laps of the layer above beside it)
    let mut zs = vec![];
    let mut safes: Vec<Vec<Vec<Vec<P2>>>> = vec![];
    // (layer, loop, bbox)
    let mut laps: Vec<(usize, Vec<P2>, [f64; 4])> = vec![];
    let mut layer_start = vec![];
    for z in rough_levels(op).into_iter().filter(|&z| z < op.start_z - 1e-6) {
        let layer = rough_layer_in(op, &geom, z)?;
        let k = zs.len();
        zs.push(z);
        layer_start.push(laps.len());
        let mut safe: Vec<Vec<Vec<P2>>> = vec![];
        for (outer, islands) in &layer.regions {
            let allowed = offset_region(outer, islands, r + a);
            if allowed.is_empty() {
                continue;
            }
            let band_edge = outer.iter().take(3).all(|&p| boundary_dist(&band, p) < 1e-4);
            for l in allowed.iter().filter(|l| !(band_edge && signed_area(l) > 0.0)) {
                let b = l.iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |b, p| [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])]);
                laps.push((k, l.clone(), b));
            }
            safe.push(allowed);
        }
        safes.push(safe);
    }
    layer_start.push(laps.len());
    let deps: Vec<Vec<usize>> = (0..laps.len())
        .map(|deps_of| {
            let (k, b) = (&laps[deps_of].0, laps[deps_of].2);
            let Some(up) = (0..*k).rev().find(|&m| layer_start[m + 1] > layer_start[m]) else { return vec![] };
            let e = r;
            let l = &laps[deps_of].1;
            (layer_start[up]..layer_start[up + 1])
                .filter(|&q| {
                    let c = laps[q].2;
                    // within a tool radius of each other (the same wall one layer up)
                    c[0] <= b[2] + e && b[0] <= c[2] + e && c[1] <= b[3] + e && b[1] <= c[3] + e && {
                        let m = &laps[q].1;
                        l.iter().any(|&p| boundary_dist(std::slice::from_ref(m), p) <= e) || m.iter().any(|&p| boundary_dist(std::slice::from_ref(l), p) <= e)
                    }
                })
                .collect()
        })
        .collect();
    let mut rank: Vec<Option<usize>> = vec![None; laps.len()];
    let mut path = Path::new([0.0, 0.0, retract]);
    let mut started = false;
    // layer the tool is on
    let mut at: Option<usize> = None;
    for n in 0..laps.len() {
        let here = [path.pos[0], path.pos[1]];
        let next = (0..laps.len())
            .filter(|&j| rank[j].is_none() && deps[j].iter().all(|&d| rank[d].is_some()))
            .map(|j| (j, deps[j].iter().map(|&d| rank[d].unwrap() as isize).max().unwrap_or(-1), near_dist(&laps[j].1, here)))
            .min_by(|x, y| y.1.cmp(&x.1).then(x.2.total_cmp(&y.2)));
        let Some((j, _, _)) = next else { break };
        rank[j] = Some(n);
        let (k, l) = (laps[j].0, &laps[j].1);
        let z = zs[k];
        let seq = start_near(l, here);
        let s0 = seq[0];
        let zh = path.pos[2];
        // the same wall one layer down (or a neighbour close by): a short
        // feed across at the depth the tool is at, never closer to the part
        // than the laps
        let near = started && d2(here, s0) <= 2.0 * r;
        let low = near && at.map_or(false, |m| (zh - zs[m]).abs() < 1e-9 && clear(&safes[m], here, s0) || (zh - z).abs() < 1e-9 && clear(&safes[k], here, s0));
        if low {
            path.feed([s0[0], s0[1], zh], op.feed);
        } else {
            let down = clearance.map_or(op.start_z + 0.5, |c| z + c).max(z + 0.5).min(retract);
            path.reposition(retract, s0, down);
        }
        path.feed([s0[0], s0[1], z], op.feed * 0.5);
        walk(&mut path, &seq, true, path_len(&seq), &|_| z, op.feed);
        started = true;
        at = Some(k);
    }
    if !started {
        return Err("finishing found no part walls to cut".into());
    }
    path.rapid([path.pos[0], path.pos[1], retract]);
    Ok(path.moves)
}

// ------------------------------------------------------------------ tabs
/// Lift feed moves over a tab (rectangle expanded by the tool radius).
fn apply_tab(moves: &[Mv], start: P3, tab: &MillTab, tool_r: f64) -> Vec<Mv> {
    let top = tab.top();
    let (s, c) = tab.angle.sin_cos();
    let (hw, hd) = (tab.width / 2.0 + tool_r, tab.depth / 2.0 + tool_r);
    let local = |p: P3| {
        let (dx, dy) = (p[0] - tab.center[0], p[1] - tab.center[1]);
        (dx * c + dy * s, -dx * s + dy * c)
    };
    let inside = |p: P3| {
        let (u, v) = local(p);
        u.abs() <= hw && v.abs() <= hd
    };
    let lift = |p: P3| if inside(p) { [p[0], p[1], p[2].max(top)] } else { p };
    let mut out = Vec::with_capacity(moves.len());
    let mut pos = start;
    for &m in moves {
        match m {
            Mv::Rapid(p) => {
                out.push(Mv::Rapid(lift(p)));
                pos = p;
            }
            Mv::Feed(p, f) => {
                let a = pos;
                // Liang-Barsky clip of segment a->p against the expanded rect
                let (au, av) = local(a);
                let (bu, bv) = local(p);
                let (du, dv) = (bu - au, bv - av);
                let mut t0: f64 = 0.0;
                let mut t1: f64 = 1.0;
                let mut hit = true;
                for (pk, qk) in [(-du, au + hw), (du, hw - au), (-dv, av + hd), (dv, hd - av)] {
                    if pk.abs() < 1e-15 {
                        if qk < 0.0 {
                            hit = false;
                        }
                    } else {
                        let t = qk / pk;
                        if pk < 0.0 {
                            t0 = t0.max(t);
                        } else {
                            t1 = t1.min(t);
                        }
                    }
                }
                let at = |t: f64| [a[0] + (p[0] - a[0]) * t, a[1] + (p[1] - a[1]) * t, a[2] + (p[2] - a[2]) * t];
                if hit && t0 < t1 && (a[2] < top || p[2] < top) {
                    if t0 > 1e-9 {
                        let q = at(t0);
                        out.push(Mv::Feed(q, f));
                        out.push(Mv::Feed([q[0], q[1], q[2].max(top)], f));
                    }
                    if t1 < 1.0 - 1e-9 {
                        let q = at(t1);
                        out.push(Mv::Feed([q[0], q[1], q[2].max(top)], f));
                        out.push(Mv::Feed(q, f));
                    }
                }
                out.push(Mv::Feed(lift(p), f));
                pos = p;
            }
        }
    }
    out
}

// ------------------------------------------------------------------ threaded holes
/// The volume of a thread-milled hole (thread major Ø, hole start Z down to
/// the hole bottom): no other operation may cut into it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Keepout {
    pub c: P2,
    pub r: f64,
    pub top: f64,
    pub bottom: f64,
}

impl Keepout {
    /// A polygon just outside the circle (circumscribed), CCW.
    fn polygon(&self, grow: f64) -> Vec<P2> {
        let n = 48;
        let r = (self.r + grow) / (PI / n as f64).cos();
        (0..n).map(|i| {
            let a = TAU * i as f64 / n as f64;
            [self.c[0] + r * a.cos(), self.c[1] + r * a.sin()]
        }).collect()
    }
}

/// Keep-out volumes of all enabled thread milling items.
pub fn keepouts(job: &StepJob) -> Vec<Keepout> {
    job.ops
        .iter()
        .filter(|o| o.enabled)
        .filter_map(|o| if let OpKind::Thread(t) = &o.kind { Some(&t.group) } else { None })
        .flat_map(|g| {
            let (r, top, bottom) = (g.thread.major_dia() / 2.0, g.start_z, g.start_z - g.hole_depth());
            g.parse_locations().unwrap_or_default().into_iter().map(move |(x, y)| Keepout { c: [x, y], r, top, bottom })
        })
        .collect()
}

/// Roughing that treats every threaded hole as solid below its top.
fn rough_with_keepouts(op: &RoughOp, ks: &[Keepout]) -> RoughOp {
    let mut op = op.clone();
    for k in ks {
        op.faces.push(RoughFace { z: k.top, outer: k.polygon(0.0), holes: vec![], tris: vec![], curved: false });
    }
    op
}

/// A pocket with the threaded holes inside it (and below its start) as
/// islands, so the clearing goes round them.
fn pocket_with_keepouts(op: &PocketOp, ks: &[Keepout]) -> PocketOp {
    let mut op = op.clone();
    for k in ks {
        let poly = k.polygon(0.0);
        let inner = inside(&[op.outer.clone()], k.c) && boundary_dist(&[op.outer.clone()], k.c) > k.r + 1e-3;
        if k.top > op.z + 1e-6 && k.bottom < op.start_z && inner {
            if !op.keep_islands {
                op.islands.clear();
                op.islands_z.clear();
                op.keep_islands = true;
            }
            op.islands.push(poly);
            op.islands_z.push(k.bottom);
        }
    }
    op
}

/// Lift moves over a threaded hole: wherever the cutter (radius `tool_r`)
/// would reach into the hole below its top, it goes over at the top instead.
fn apply_keepout(moves: &[Mv], start: P3, k: &Keepout, tool_r: f64) -> Vec<Mv> {
    let rr = k.r + tool_r;
    let inside = |p: P3| (p[0] - k.c[0]).hypot(p[1] - k.c[1]) < rr;
    let lift = |p: P3| if inside(p) { [p[0], p[1], p[2].max(k.top)] } else { p };
    let mut out = Vec::with_capacity(moves.len());
    let mut pos = start;
    for &m in moves {
        match m {
            Mv::Rapid(p) => {
                out.push(Mv::Rapid(lift(p)));
                pos = p;
            }
            Mv::Feed(p, f) => {
                let a = pos;
                let (dx, dy) = (p[0] - a[0], p[1] - a[1]);
                let (fx, fy) = (a[0] - k.c[0], a[1] - k.c[1]);
                let qa = dx * dx + dy * dy;
                let qb = 2.0 * (fx * dx + fy * dy);
                let qc = fx * fx + fy * fy - rr * rr;
                let disc = qb * qb - 4.0 * qa * qc;
                let at = |t: f64| [a[0] + (p[0] - a[0]) * t, a[1] + (p[1] - a[1]) * t, a[2] + (p[2] - a[2]) * t];
                if qa > 1e-18 && disc > 0.0 && (a[2] < k.top || p[2] < k.top) {
                    let sq = disc.sqrt();
                    let (t0, t1) = (((-qb - sq) / (2.0 * qa)).max(0.0), ((-qb + sq) / (2.0 * qa)).min(1.0));
                    if t0 < t1 {
                        if t0 > 1e-9 {
                            let q = at(t0);
                            out.push(Mv::Feed(q, f));
                            out.push(Mv::Feed([q[0], q[1], q[2].max(k.top)], f));
                        }
                        if t1 < 1.0 - 1e-9 {
                            let q = at(t1);
                            out.push(Mv::Feed([q[0], q[1], q[2].max(k.top)], f));
                            out.push(Mv::Feed(q, f));
                        }
                    }
                }
                out.push(Mv::Feed(lift(p), f));
                pos = p;
            }
        }
    }
    out
}

// ------------------------------------------------------------------ validation & output
pub fn validate(op: &StepOp, s: &Settings, tabs_top: f64) -> Vec<String> {
    let mut e = vec![];
    let (start, floor, cutter, dpp, feed, speed) = match &op.kind {
        OpKind::Face(f) => (f.start_z, f.z, f.cutter, f.depth_per_pass, f.feed, f.speed),
        OpKind::Pocket(p) => (p.start_z, p.z, p.cutter, p.depth_per_pass, p.feed, p.speed),
        OpKind::Profile(p) => (p.start_z, p.z, p.cutter, p.depth_per_pass, p.feed, p.speed),
        OpKind::Rough(r) => (r.start_z, r.final_z(), r.cutter, r.layer_height, r.feed, r.speed),
        OpKind::Finish(r) => (r.start_z, r.bottom_z, r.cutter, r.layer_height, r.feed, r.speed),
        OpKind::Engrave(g) => {
            if !g.tool.in_table(s) {
                e.push(format!("T{} is not in the tool table", g.tool.tool_number()));
            }
            if !(g.depth > 0.0) {
                e.push("engraving depth must be > 0".into());
            } else if g.depth > g.tool.max_depth() + 1e-9 {
                e.push(format!("depth {} exceeds the tool's max depth {}", fmt(g.depth), fmt(g.tool.max_depth())));
            }
            if !(g.depth_per_pass > 0.0) || !(g.feed > 0.0) || !(g.speed > 0.0) {
                e.push("depth per pass, feed and speed must be > 0".into());
            }
            if let Err(err) = engrave::text_paths(g) {
                e.push(err);
            }
            let retract = g.z.max(tabs_top) + s.clearance;
            if retract > s.safe_z {
                e.push(format!("retract height {} is above the rapid plane Z {}", fmt(retract), fmt(s.safe_z)));
            }
            return e;
        }
        OpKind::Thread(t) => {
            for is in crate::gcode::validate_group(&t.group, s) {
                e.push(is.msg);
            }
            return e;
        }
        OpKind::Tab(t) => {
            if !(t.width > 0.0 && t.depth > 0.0 && t.z_height > 0.0) {
                e.push("tab width, depth and Z height must be > 0".into());
            }
            return e;
        }
        OpKind::Omit(_) => return e,
    };
    if s.lookup(cutter).is_none() {
        e.push(format!("T{} is not in the tool table", cutter.tool));
    }
    if start <= floor + 1e-9 {
        e.push(format!("start Z {} must be above the surface Z {}", fmt(start), fmt(floor)));
    }
    if start - floor > cutter.flute_len() {
        e.push(format!("depth {} exceeds cutter flute length {}", fmt(start - floor), fmt(cutter.flute_len())));
    }
    if !(dpp > 0.0) || !(feed > 0.0) || !(speed > 0.0) {
        e.push("depth per pass, feed and speed must be > 0".into());
    }
    if let OpKind::Pocket(p) = &op.kind {
        if !(p.ramp_angle > 0.0 && p.ramp_angle <= 45.0) {
            e.push("ramp angle must be > 0 and <= 45 degrees".into());
        }
        if p.finish < 0.0 {
            e.push("finish allowance must be >= 0".into());
        }
        if !(p.stepover_pct > 0.0 && p.stepover_pct <= 100.0) {
            e.push("stepover must be > 0 and <= 100 %".into());
        }
        if offset_region(&p.outer, if p.keep_islands { &p.islands } else { &[] }, cutter.dia() / 2.0 + p.finish.max(0.0)).is_empty() {
            e.push(format!("cutter Ø{} does not fit in the pocket", fmt(cutter.dia())));
        }
    }
    if let OpKind::Finish(r) = &op.kind {
        if r.allowance < 0.0 {
            e.push("stock to leave must be >= 0".into());
        }
        if r.faces.is_empty() {
            e.push("no model faces - import a STEP model".into());
        }
    }
    if let OpKind::Rough(r) = &op.kind {
        if !(r.ramp_angle > 0.0 && r.ramp_angle <= 45.0) {
            e.push("ramp angle must be > 0 and <= 45 degrees".into());
        }
        if !(r.stepover_pct > 0.0 && r.stepover_pct <= 100.0) {
            e.push("stepover must be > 0 and <= 100 %".into());
        }
        if r.allowance < 0.0 {
            e.push("stock to leave must be >= 0".into());
        }
        if r.overcut_z < 0.0 {
            e.push("overcut Z must be >= 0".into());
        }
        if r.faces.is_empty() {
            e.push("no model faces - import a STEP model".into());
        }
    }
    if let OpKind::Profile(p) = &op.kind {
        if !(p.ramp_angle > 0.0 && p.ramp_angle <= 90.0) {
            e.push("ramp angle must be > 0 and <= 90 degrees".into());
        }
        if let Err(err) = profile_paths(p) {
            e.push(err);
        }
    }
    if let OpKind::Face(f) = &op.kind {
        if !(f.stepover_pct > 0.0 && f.stepover_pct <= 100.0) {
            e.push("stepover must be > 0 and <= 100 %".into());
        }
    }
    let retract = start.max(tabs_top) + s.clearance;
    if retract > s.safe_z {
        e.push(format!("retract height {} is above the rapid plane Z {}", fmt(retract), fmt(s.safe_z)));
    }
    e
}

pub fn generate(job: &StepJob, s: &Settings) -> Result<String, Vec<String>> {
    let tabs: Vec<&MillTab> = job
        .ops
        .iter()
        .filter(|o| o.enabled)
        .filter_map(|o| if let OpKind::Tab(t) = &o.kind { Some(t) } else { None })
        .collect();
    let tabs_top = tabs.iter().map(|t| t.top()).fold(f64::NEG_INFINITY, f64::max);
    let holes = keepouts(job);
    let mut errors = vec![];
    for (i, op) in job.ops.iter().enumerate().filter(|(_, o)| o.enabled) {
        for e in validate(op, s, tabs_top) {
            errors.push(format!("op {}: {}", i + 1, e));
        }
    }
    let cutting: Vec<(usize, &StepOp)> = job.ops.iter().enumerate().filter(|(_, o)| o.enabled && !matches!(o.kind, OpKind::Tab(_) | OpKind::Omit(_))).collect();
    if cutting.is_empty() && errors.is_empty() {
        errors.push("no Face, Pocket, Profile, Rough, Finish, Engrave or Thread operations - right-click a surface or an edge in the 3D view".into());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let mut w = Out { s: String::new(), tool: None, rpm: -1.0 };
    w.line("%".into());
    w.line("(stepCAM - step milling)".into());
    if !job.step_path.is_empty() {
        let name = std::path::Path::new(&job.step_path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        w.line(format!("(model {})", name.replace(['(', ')'], "")));
    }
    for t in &tabs {
        w.line(format!("(mill tab X{} Y{} Z{} to Z{} {}x{})", fmt(t.center[0]), fmt(t.center[1]), fmt(t.center[2]), fmt(t.top()), fmt(t.width), fmt(t.depth)));
    }
    w.line("G21 G90 G17 G94 G40 G49 G80".into());
    w.line(format!("G0 Z{}", fmt(s.safe_z)));

    // where the last op left the tool (XY), so the next one starts nearby
    let mut here: P2 = [0.0, 0.0];
    for (i, op) in cutting {
        // areas omitted before this op: treated as filled in
        let fills = fills_before(&job.ops, i);
        // holes: bore with the end mill, then thread with the thread mill
        if let OpKind::Thread(th) = &op.kind {
            let g = &th.group;
            // (holes in an omitted area are left)
            let bottom = g.start_z - g.hole_depth();
            let locs: Vec<(f64, f64)> = g.parse_locations().unwrap_or_default().into_iter().filter(|&(x, y)| !fills.iter().any(|f| f.top > bottom + 1e-6 && f.covers([x, y]))).collect();
            if locs.is_empty() {
                continue;
            }
            let tm = s.thread_mill(g.thread);
            w.line(String::new());
            w.change(s, g.cutter.tool_number(), &format!("endmill Ø{} flute {}", fmt(g.cutter.dia()), fmt(g.cutter.flute_len())), s.spindle_rpm);
            w.line(format!("(op {}: {} - bore)", i + 1, op.label()));
            w.line(format!("G0 Z{}", fmt(s.safe_z)));
            w.raw(&crate::gcode::bore_code(s, g, &locs));
            w.line(format!("G0 Z{}", fmt(s.safe_z)));
            w.line(String::new());
            w.change(s, tm.tool, &format!("thread mill {} Ø{} reach {}", g.thread.name(), fmt(tm.dia), fmt(tm.reach)), s.spindle_rpm);
            w.line(format!("(op {}: {} - thread)", i + 1, op.label()));
            w.line(format!("G0 Z{}", fmt(s.safe_z)));
            w.raw(&crate::gcode::thread_code(s, g, &locs));
            w.line(format!("G0 Z{}", fmt(s.safe_z)));
            continue;
        }
        let mill = |c: Cutter| (c.tool_number(), c.dia() / 2.0, format!("endmill Ø{} flute {}", fmt(c.dia()), fmt(c.flute_len())));
        let ((t, tool_r, tool_desc), speed, start_z) = match &op.kind {
            OpKind::Face(f) => (mill(f.cutter), f.speed, f.start_z),
            OpKind::Pocket(p) => (mill(p.cutter), p.speed, p.start_z),
            OpKind::Profile(p) => (mill(p.cutter), p.speed, p.start_z),
            OpKind::Rough(r) | OpKind::Finish(r) => (mill(r.cutter), r.speed, r.start_z),
            OpKind::Engrave(g) => ((g.tool.tool_number(), g.tool.width_at(g.depth) / 2.0, g.tool.describe()), g.speed, g.z),
            OpKind::Tab(_) | OpKind::Thread(_) | OpKind::Omit(_) => unreachable!(),
        };
        let retract = start_z.max(tabs_top) + s.clearance;
        let mut moves = match &op.kind {
            OpKind::Face(f) => face_moves(f, retract),
            OpKind::Pocket(p) => match pocket_with_fills(p, &fills) {
                // the pocket is in an omitted area
                None => vec![],
                Some(p) => match pocket_moves(&pocket_with_keepouts(&p, &holes), retract, here) {
                    Ok(m) => m,
                    Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
                },
            },
            OpKind::Rough(r) => match rough_moves(&rough_with_keepouts(&rough_with_fills(r, &fills), &holes), retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Finish(r) => match finish_moves(&rough_with_keepouts(&rough_with_fills(r, &fills), &holes), retract, finish_clearance(&job.ops[..i], r)) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Profile(p) => match profile_moves(p, retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Engrave(g) => match engrave_moves(g, retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Tab(_) | OpKind::Thread(_) | OpKind::Omit(_) => unreachable!(),
        };
        let start = match moves.first() {
            Some(Mv::Rapid(p)) | Some(Mv::Feed(p, _)) => [p[0], p[1], s.safe_z],
            None => continue,
        };
        // threaded holes are left to their thread milling item
        for k in &holes {
            moves = apply_keepout(&moves, start, k, tool_r);
        }
        // and omitted areas are gone over
        for f in &fills {
            moves = apply_fill(&moves, start, f, tool_r);
        }
        for t in &tabs {
            moves = apply_tab(&moves, start, t, tool_r);
        }

        w.line(String::new());
        w.change(s, t, &tool_desc, speed);
        w.line(format!("(op {}: {})", i + 1, op.label()));
        w.line(format!("G0 Z{}", fmt(s.safe_z)));
        // go to the op's first point at the rapid plane: the moves start there
        w.line(format!("G0 X{} Y{}", fmt(start[0]), fmt(start[1])));

        let mut pos = start;
        let mut last_f = -1.0;
        for m in moves {
            match m {
                Mv::Rapid(p) => {
                    let xy = (p[0] - pos[0]).abs() > 1e-9 || (p[1] - pos[1]).abs() > 1e-9;
                    let zc = (p[2] - pos[2]).abs() > 1e-9;
                    if zc && p[2] > pos[2] {
                        w.line(format!("G0 Z{}", fmt(p[2])));
                    }
                    if xy {
                        w.line(format!("G0 X{} Y{}", fmt(p[0]), fmt(p[1])));
                    }
                    if zc && p[2] < pos[2] {
                        w.line(format!("G0 Z{}", fmt(p[2])));
                    }
                    pos = p;
                }
                Mv::Feed(p, f) => {
                    let mut cmd = String::from("G1");
                    for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
                        if (p[k] - pos[k]).abs() > 1e-9 {
                            cmd.push_str(&format!(" {a}{}", fmt(p[k])));
                        }
                    }
                    if cmd == "G1" {
                        continue;
                    }
                    if (f - last_f).abs() > 1e-9 {
                        cmd.push_str(&format!(" F{}", fmt(f)));
                        last_f = f;
                    }
                    w.line(cmd);
                    pos = p;
                }
            }
        }
        here = [pos[0], pos[1]];
        w.line(format!("G0 Z{}", fmt(s.safe_z)));
    }
    w.line(String::new());
    w.line("M5".into());
    w.line("M9".into());
    w.line(format!("G0 Z{}", fmt(s.safe_z)));
    w.line("M30".into());
    w.line("%".into());
    Ok(shift_code(&w.s, job.origin.at))
}

/// Program text with tool / spindle state.
struct Out {
    s: String,
    tool: Option<u32>,
    rpm: f64,
}

impl Out {
    fn line(&mut self, l: String) {
        self.s.push_str(&l.replace('Ø', "D"));
        self.s.push('\n');
    }
    /// Lines already in program form.
    fn raw(&mut self, code: &str) {
        self.s.push_str(&code.replace('Ø', "D"));
    }
    /// Change to tool `t` (or just the spindle speed) when needed.
    fn change(&mut self, s: &Settings, t: u32, desc: &str, speed: f64) {
        if self.tool != Some(t) {
            if self.tool.is_some() {
                self.line("M5".into());
                self.line("M9".into());
            }
            self.line(format!("G0 Z{}", fmt(s.safe_z)));
            self.line(format!("T{t} M6 ({desc})"));
            if s.tool_length_comp {
                self.line(format!("G43 H{t}"));
            }
            self.line(format!("S{} M3", speed.round() as i64));
            if s.coolant {
                self.line("M8".into());
            }
            self.tool = Some(t);
            self.rpm = speed;
        } else if (self.rpm - speed).abs() > 0.5 {
            self.line(format!("S{} M3", speed.round() as i64));
            self.rpm = speed;
        }
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolpath;

    fn pocket(strategy: Strategy, entry: Entry) -> StepJob {
        StepJob {
            step_path: String::new(),
            placement: Default::default(),
            stock: Default::default(),
            origin: Default::default(),
            rapid_above: 10.0,
            ops: vec![StepOp {
                enabled: true,
                kind: OpKind::Pocket(PocketOp {
                    z: -4.0,
                    outer: vec![[15.0, 10.0], [45.0, 10.0], [45.0, 30.0], [15.0, 30.0]],
                    cutter: Cutter::C3x17,
                    strategy,
                    entry,
                    ..Default::default()
                }),
            }],
        }
    }

    fn feed_points(code: &str) -> Vec<[f32; 3]> {
        let tp = toolpath::parse(code);
        tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.clone()).collect()
    }

    #[test]
    fn pocket_stays_inside_walls_and_reaches_floor() {
        let s = Settings::default();
        for strategy in [Strategy::Standard, Strategy::Adaptive] {
            for entry in [Entry::Helix, Entry::Zigzag, Entry::Plunge] {
                let code = generate(&pocket(strategy, entry), &s).unwrap();
                let pts = feed_points(&code);
                let mut minz = f32::INFINITY;
                for p in &pts {
                    if p[2] < -1e-6 {
                        // tool centre at least r=1.5 from every wall
                        assert!(p[0] >= 16.5 - 1e-3 && p[0] <= 43.5 + 1e-3 && p[1] >= 11.5 - 1e-3 && p[1] <= 28.5 + 1e-3, "{strategy:?}/{entry:?} gouge at {p:?}");
                    }
                    minz = minz.min(p[2]);
                }
                assert!((minz + 4.0).abs() < 1e-4, "{strategy:?}/{entry:?} floor {minz}");
                // finish pass touches the walls
                assert!(pts.iter().any(|p| (p[0] - 16.5).abs() < 1e-3 && p[2] < -3.99));
            }
        }
    }

    #[test]
    fn roughing_leaves_finish_allowance() {
        let op = match &pocket(Strategy::Standard, Entry::Plunge).ops[0].kind {
            OpKind::Pocket(p) => p.clone(),
            _ => unreachable!(),
        };
        let moves = pocket_moves(&PocketOp { finish: 0.5, ..op }, 2.0, [0.0, 0.0]).unwrap();
        // before the first visit to the finish wall x=16.5, nothing goes past x=17.0
        let mut finishing = false;
        for m in moves {
            if let Mv::Feed(p, _) = m {
                if (p[0] - 16.5).abs() < 1e-6 {
                    finishing = true;
                }
                if !finishing && p[2] < 0.0 {
                    assert!(p[0] >= 17.0 - 1e-6, "rough pass cut into allowance at {p:?}");
                }
            }
        }
        assert!(finishing);
    }

    #[test]
    fn tabs_are_lifted_over() {
        let mut job = pocket(Strategy::Standard, Entry::Plunge);
        let tab = MillTab { center: [30.0, 10.0, -4.0], angle: 0.0, z_height: 2.0, width: 4.0, depth: 2.0, edges: vec![] };
        job.ops.push(StepOp { enabled: true, kind: OpKind::Tab(tab) });
        let code = generate(&job, &Settings::default()).unwrap();
        let pts = feed_points(&code);
        for p in &pts {
            if (p[0] - 30.0).abs() < 2.0 + 1.5 - 1e-3 && (p[1] - 10.0).abs() < 1.0 + 1.5 - 1e-3 {
                assert!(p[2] >= -2.0 - 1e-4, "cut into tab at {p:?}");
            }
        }
        // and the pocket still reaches the floor elsewhere
        assert!(pts.iter().any(|p| (p[2] + 4.0).abs() < 1e-4));
    }

    #[test]
    fn facing_without_outline_covers_region_with_overlap() {
        let job = StepJob {
            step_path: String::new(),
            placement: Default::default(),
            stock: Default::default(),
            origin: Default::default(),
            rapid_above: 10.0,
            ops: vec![StepOp {
                enabled: true,
                // an older job: no outline, so the region box is faced
                kind: OpKind::Face(FaceOp { z: -1.5, region: [0.0, 0.0, 60.0, 40.0], overlap: 2.0, cutter: Cutter::C3x17, ..Default::default() }),
            }],
        };
        let code = generate(&job, &Settings::default()).unwrap();
        let pts = feed_points(&code);
        let at_floor: Vec<_> = pts.iter().filter(|p| (p[2] + 1.5).abs() < 1e-4).collect();
        assert!(!at_floor.is_empty());
        let minx = at_floor.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min);
        let maxy = at_floor.iter().map(|p| p[1]).fold(f32::NEG_INFINITY, f32::max);
        let miny = at_floor.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
        // edge 2 mm past the box at the ends; rows from 2 mm above it to 2 mm below
        assert!((minx - (0.0 - 2.0 - 1.5)).abs() < 1e-3);
        assert!((maxy - (40.0 + 2.0)).abs() < 1e-3);
        assert!((miny - (0.0 - 2.0)).abs() < 1e-3);
        // two depth levels (1 mm per pass, 1.5 deep)
        assert!(pts.iter().any(|p| (p[2] + 0.75).abs() < 1e-4));
    }

    #[test]
    fn rejects_bad_ops() {
        let mut job = pocket(Strategy::Standard, Entry::Helix);
        if let OpKind::Pocket(p) = &mut job.ops[0].kind {
            p.outer = vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]; // too small for Ø3
        }
        assert!(generate(&job, &Settings::default()).is_err());
        let mut job = pocket(Strategy::Standard, Entry::Helix);
        if let OpKind::Pocket(p) = &mut job.ops[0].kind {
            p.start_z = -5.0; // below floor
        }
        assert!(generate(&job, &Settings::default()).is_err());
    }

    #[test]
    fn end_to_end_from_step_file() {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let xy = |l: &crate::step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
        let mut ops = vec![];
        for (fi, f) in m.faces.iter().enumerate().filter(|(_, f)| f.is_up_horizontal()) {
            if f.z() < -1e-9 {
                ops.push(StepOp {
                    enabled: true,
                    kind: OpKind::Pocket(PocketOp {
                        face_id: fi,
                        z: f.z(),
                        outer: f.outer_loop().map(xy).unwrap(),
                        islands: f.loops.iter().filter(|l| !l.outer).map(xy).collect(),
                        cutter: Cutter::C3x17,
                        strategy: Strategy::Adaptive,
                        ..Default::default()
                    }),
                });
            } else {
                ops.insert(0, StepOp {
                    enabled: true,
                    kind: OpKind::Face(FaceOp { face_id: fi, z: f.z(), region: [m.min[0], m.min[1], m.max[0], m.max[1]], start_z: 0.5, ..Default::default() }),
                });
            }
        }
        // tab on the pocket floor edge y=10 at x=30
        let e = m.edges.iter().position(|e| e.pts.iter().all(|p| (p[1] - 10.0).abs() < 1e-9 && (p[2] + 4.0).abs() < 1e-9)).unwrap();
        let (chain, closed) = m.edge_chain(e);
        assert!(closed);
        ops.push(StepOp { enabled: true, kind: OpKind::Tab(MillTab { center: [30.0, 10.0, -4.0], edges: chain, ..Default::default() }) });
        assert_eq!(ops.len(), 4); // face, 2 pockets, tab
        let job = StepJob { step_path: "testdata/pocket_block.step".into(), placement: Default::default(), stock: Default::default(), rapid_above: 10.0, origin: Default::default(), ops };
        let code = generate(&job, &Settings::default()).expect("valid program");
        let pts = feed_points(&code);
        // round hole (r=4 at 52,32) pocketed to -3 without touching its wall
        let hole: Vec<_> = pts.iter().filter(|p| p[2] < -0.01 && ((p[0] - 52.0).powi(2) + (p[1] - 32.0).powi(2)).sqrt() < 5.0).collect();
        assert!(!hole.is_empty());
        for p in &hole {
            assert!(((p[0] - 52.0).powi(2) + (p[1] - 32.0).powi(2)).sqrt() <= 4.0 - 1.5 + 2e-3, "hole wall gouge at {p:?}");
            assert!(p[2] >= -3.0 - 1e-4);
        }
        assert!(code.contains("T4 M6") && code.contains("T5 M6"));
    }

    fn profile_job(side: ProfileSide, closed: bool, path: Vec<P2>) -> StepJob {
        StepJob {
            step_path: String::new(),
            placement: Default::default(),
            stock: Default::default(),
            origin: Default::default(),
            rapid_above: 10.0,
            ops: vec![StepOp {
                enabled: true,
                kind: OpKind::Profile(ProfileOp { path, closed, side, z: -10.0, start_z: 0.0, depth_per_pass: 3.0, cutter: Cutter::C3x17, ..Default::default() }),
            }],
        }
    }

    fn rect() -> Vec<P2> {
        vec![[0.0, 0.0], [60.0, 0.0], [60.0, 40.0], [0.0, 40.0], [0.0, 0.0]]
    }

    /// Distance from the rectangle 0..60 x 0..40 boundary (negative inside).
    fn rect_sd(p: [f32; 3]) -> f64 {
        let (x, y) = (p[0] as f64, p[1] as f64);
        let dx = (-x).max(x - 60.0);
        let dy = (-y).max(y - 40.0);
        if dx <= 0.0 && dy <= 0.0 {
            dx.max(dy)
        } else {
            dx.max(0.0).hypot(dy.max(0.0))
        }
    }

    #[test]
    fn profile_sides_offset_by_tool_radius() {
        let s = Settings::default();
        for (side, want) in [(ProfileSide::Outside, 1.5), (ProfileSide::Inside, -1.5), (ProfileSide::Center, 0.0)] {
            let code = generate(&profile_job(side, true, rect()), &s).unwrap();
            let pts = feed_points(&code);
            let deep: Vec<_> = pts.iter().filter(|p| p[2] < -1e-3).collect();
            assert!(!deep.is_empty());
            for p in &deep {
                assert!((rect_sd(**p) - want).abs() < 2e-3, "{side:?}: {p:?} at {}", rect_sd(**p));
            }
            let minz = pts.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min);
            assert!((minz + 10.0).abs() < 1e-4, "{side:?} depth {minz}");
            // full loop at final depth: all four sides visited at Z-10
            let bottom: Vec<_> = deep.iter().filter(|p| (p[2] + 10.0).abs() < 1e-4).collect();
            for (x, y) in [(30.0, -1.0), (61.0, 20.0), (30.0, 41.0), (-1.0, 20.0)] {
                assert!(bottom.iter().any(|p| ((p[0] - x) as f64).abs() < 31.0 && ((p[1] - y) as f64).abs() < 21.0 && rect_sd(***p) >= want - 1e-3));
            }
        }
        // outside is cut clockwise (climb with M3)
        let moves = profile_moves(match &profile_job(ProfileSide::Outside, true, rect()).ops[0].kind { OpKind::Profile(p) => p, _ => unreachable!() }, 2.0).unwrap();
        let pts: Vec<P2> = moves.iter().filter_map(|m| if let Mv::Feed(p, _) = m { Some([p[0], p[1]]) } else { None }).collect();
        let area: f64 = pts.windows(2).map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1]).sum();
        assert!(area < 0.0, "outside profile should run CW");
    }

    #[test]
    fn profile_ramps_at_angle() {
        let op = ProfileOp { path: rect(), closed: true, side: ProfileSide::Center, z: -2.0, start_z: 0.0, depth_per_pass: 2.0, ramp_angle: 3.0, ..Default::default() };
        let moves = profile_moves(&op, 2.0).unwrap();
        let mut prev: Option<P3> = None;
        let tan = 3f64.to_radians().tan();
        for m in moves {
            if let Mv::Feed(p, _) = m {
                if let Some(a) = prev {
                    let h = (p[0] - a[0]).hypot(p[1] - a[1]);
                    if h > 1e-6 {
                        assert!(a[2] - p[2] <= h * tan + 1e-6, "too steep {a:?} -> {p:?}");
                    }
                }
                prev = Some(p);
            } else if let Mv::Rapid(p) = m {
                prev = Some(p);
            }
        }
    }

    #[test]
    fn open_profile_center_only_and_covers_chain() {
        let path = vec![[0.0, 0.0], [0.0, 20.0], [-10.0, 20.0]];
        assert!(generate(&profile_job(ProfileSide::Outside, false, path.clone()), &Settings::default()).is_err());
        let code = generate(&profile_job(ProfileSide::Center, false, path), &Settings::default()).unwrap();
        let pts = feed_points(&code);
        for p in pts.iter().filter(|p| p[2] < -1e-3) {
            let on = (p[0].abs() < 1e-3 && p[1] >= -1e-3 && p[1] <= 20.001) || ((p[1] - 20.0).abs() < 1e-3 && p[0] <= 1e-3 && p[0] >= -10.001);
            assert!(on, "off the chain at {p:?}");
        }
        let bottom: Vec<_> = pts.iter().filter(|p| (p[2] + 10.0).abs() < 1e-4).collect();
        assert!(bottom.iter().any(|p| p[1] < 0.01) && bottom.iter().any(|p| p[0] < -9.99));
    }
}

#[cfg(test)]
mod rough_tests {
    use super::*;
    use crate::toolpath;

    #[test]
    fn roughing_clears_inside_and_a_band_outside() {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let stock = Stock { x: 4.0, y: 4.0, z: 1.0, offset: true, ..Default::default() };
        let (smin, smax) = stock.bounds(m.min, m.max);
        let op = RoughOp { allowance: 0.2,
            stock_region: [smin[0], smin[1], smax[0], smax[1]],
            cutter: Cutter::C3x17,
            start_z: smax[2],
            bottom_z: m.min[2],
            faces: rough_faces(&m),
            ..Default::default()
        };
        let levels = rough_levels(&op);
        // floors at 0, -3 and -4 get their own layer (+0.2 allowance)
        for z in [0.2, -2.8, -3.8, -10.0] {
            assert!(levels.iter().any(|l| (l - z).abs() < 1e-9), "missing layer {z} in {levels:?}");
        }
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let mut s = Settings::default();
        s.safe_z = 12.0;
        let code = generate(&job, &s).unwrap();
        let tp = toolpath::parse(&code);
        let pts: Vec<[f64; 3]> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64])).collect();
        let keep = 1.5 + 0.2 - 2e-3; // tool radius + stock to leave
        // signed distance to an axis-aligned box (negative inside)
        let sd = |p: [f64; 3], x0: f64, y0: f64, x1: f64, y1: f64| {
            let dx = (x0 - p[0]).max(p[0] - x1);
            let dy = (y0 - p[1]).max(p[1] - y1);
            if dx <= 0.0 && dy <= 0.0 { dx.max(dy) } else { dx.max(0.0).hypot(dy.max(0.0)) }
        };
        let (mut pocket, mut hole, mut outside) = (false, false, false);
        for p in &pts {
            let hr = (p[0] - 52.0).hypot(p[1] - 32.0);
            if p[2] < 0.2 - 1e-6 {
                let in_pocket = sd(*p, 15.0, 10.0, 45.0, 30.0) <= -keep;
                let in_hole = hr <= 4.0 - keep;
                let out = sd(*p, 0.0, 0.0, 60.0, 40.0) >= keep;
                assert!(in_pocket || in_hole || out, "gouge at {p:?}");
                if in_pocket {
                    assert!(p[2] >= -3.8 - 1e-6, "pocket floor gouge at {p:?}");
                    pocket |= (p[2] + 3.8).abs() < 1e-6;
                }
                if in_hole {
                    assert!(p[2] >= -2.8 - 1e-6, "hole floor gouge at {p:?}");
                    hole |= (p[2] + 2.8).abs() < 1e-6;
                }
                if out {
                    // outside the part only a band: the tool centre at most
                    // r + a + OPEN_EDGE out, enough to run round the walls
                    let d = sd(*p, 0.0, 0.0, 60.0, 40.0);
                    assert!(d < 1.7 + OPEN_EDGE + 0.01, "cleared stock beyond the band at {p:?} ({d})");
                    outside |= (p[2] + 10.0).abs() < 1e-6;
                }
            }
            // never below the part bottom, never further out than the tool
            // radius (+ open edge) past the stock
            assert!(p[2] >= -10.0 - 1e-6);
            // the facing pass above Z0.2 runs 2 mm + radius past the stock
            // (the model's loose wires reach X-20, so the stock runs from X-22)
            if p[2] >= 0.2 - 1e-6 {
                let out_x = (smin[0] - p[0]).max(p[0] - smax[0]);
                let out_y = (smin[1] - p[1]).max(p[1] - smax[1]);
                assert!(out_x <= 3.5 + 1e-6 && out_y <= 3.5 + 1e-6, "beyond the stock at {p:?}");
            }
        }
        assert!(pocket && hole && outside, "pocket {pocket} hole {hole} stock outside {outside}");
        // the 1 mm of stock over the part is faced down to part top + allowance
        // over the same band: right across the part, no further than r + a
        // (+ open edge) past its outline
        let faced: Vec<&[f64; 3]> = pts.iter().filter(|p| (p[2] - 0.2).abs() < 1e-6).collect();
        assert!(faced.iter().any(|p| sd(**p, 0.0, 0.0, 60.0, 40.0) < -5.0));
        assert!(faced.iter().any(|p| p[0] < 0.0) && faced.iter().any(|p| p[0] > 60.0), "facing does not cross the part");
        let (ylo, yhi) = faced.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[1]), hi.max(p[1])));
        assert!(ylo <= 0.0 + 1e-6 && yhi >= 40.0 - 1e-6, "facing rows {ylo}..{yhi} miss the part edges");
        assert!(faced.iter().all(|p| sd(**p, 0.0, 0.0, 60.0, 40.0) <= 1.7 + OPEN_EDGE + 0.01), "facing beyond the band");
    }

    #[test]
    fn stock_to_leave_defaults_to_zero() {
        assert_eq!(RoughOp::default().allowance, 0.0);
    }

    #[test]
    fn no_facing_when_stock_is_level_with_the_part() {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let mut op = RoughOp { allowance: 0.2, cutter: Cutter::C3x17, start_z: 0.0, bottom_z: m.min[2], faces: rough_faces(&m), stock_region: [-2.0, -2.0, 62.0, 42.0], ..Default::default() };
        assert_eq!(op.facing_z(), None);
        op.start_z = 0.1; // within the stock to leave: left for finishing
        assert_eq!(op.facing_z(), None);
        op.start_z = 0.5;
        assert_eq!(op.facing_z(), Some(0.2));
        // facing replaces the roughing layers above the faced height
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 12.0, ..Default::default() };
        let tp = toolpath::parse(&generate(&job, &s).unwrap());
        let feed_z: Vec<f32> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.iter().map(|q| q[2])).collect();
        assert!(feed_z.iter().any(|z| (z - 0.2).abs() < 1e-5));
        // the only feed moves above the stock are the 0.5 mm lifts between facing rows
        assert!(feed_z.iter().all(|&z| z <= 0.5 + 0.5 + 1e-5), "cut above the stock");
    }

    #[test]
    fn roughing_clears_open_steps_without_fins() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        // 20x20 boss (top Z0) on a 40x40 flange (top Z-5), bottom Z-10
        let op = RoughOp {
            cutter: Cutter::C3x17,
            allowance: 0.2,
            layer_height: 2.0,
            start_z: 0.0,
            bottom_z: -10.0,
            faces: vec![RoughFace { z: 0.0, outer: sq(10.0, 30.0), holes: vec![], tris: vec![], curved: false }, RoughFace { z: -5.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 30.0)], tris: vec![], curved: false }],
            ..Default::default()
        };
        let moves = rough_moves(&op, 2.0).unwrap();
        let pts: Vec<P3> = moves.iter().filter_map(|m| if let Mv::Feed(p, _) = m { Some(*p) } else { None }).collect();
        let sd = |p: P3, a: f64, b: f64| {
            let dx = (a - p[0]).max(p[0] - b);
            let dy = (a - p[1]).max(p[1] - b);
            if dx <= 0.0 && dy <= 0.0 { dx.max(dy) } else { dx.max(0.0).hypot(dy.max(0.0)) }
        };
        let flange_top = -4.8;
        let mut reached_edge = false;
        for p in &pts {
            assert!(sd(*p, 10.0, 30.0) >= 1.7 - 2e-3, "boss gouge at {p:?}");
            if p[2] < flange_top - 1e-6 {
                // below the flange top: only round the flange, the tool centre
                // between r + a and r + a + OPEN_EDGE outside it
                let d = sd(*p, 0.0, 40.0);
                assert!(d > 1.7 - 2e-3 && d < 1.7 + OPEN_EDGE + 0.01, "flange gouge / extra cut at {p:?}");
            }
            if (p[2] - flange_top).abs() < 1e-6 && sd(*p, 0.0, 40.0) > -0.01 {
                reached_edge = true;
            }
            assert!(sd(*p, 0.0, 40.0) < 1.7 + OPEN_EDGE + 0.01, "cleared outside the part at {p:?}");
        }
        assert!(reached_edge, "step above the flange not cleared out to its edge");
    }

    #[test]
    fn roughing_overcut_goes_below_bottom() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        let base = RoughOp {
            cutter: Cutter::C3x17,
            start_z: 0.0,
            bottom_z: -10.0,
            layer_height: 3.0,
            faces: vec![RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![], tris: vec![], curved: false }],
            ..Default::default()
        };
        let minz = |op: &RoughOp| {
            rough_moves(op, 2.0).unwrap().iter().filter_map(|m| if let Mv::Feed(p, _) = m { Some(p[2]) } else { None }).fold(f64::INFINITY, f64::min)
        };
        assert!((minz(&base) + 10.0).abs() < 1e-9);
        let over = RoughOp { overcut_z: 0.5, ..base.clone() };
        assert!((over.final_z() + 10.5).abs() < 1e-12);
        assert!((minz(&over) + 10.5).abs() < 1e-9);
        assert_eq!(*rough_levels(&over).last().unwrap(), -10.5);
        let bad = RoughOp { overcut_z: -1.0, ..base };
        assert!(validate(&StepOp { enabled: true, kind: OpKind::Rough(bad) }, &Settings::default(), f64::NEG_INFINITY).iter().any(|e| e.contains("overcut")));
    }

    #[test]
    fn roughing_without_clear_voids_profiles_through_holes_but_clears_floors() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        let rect = |x0: f64, y0: f64, x1: f64, y1: f64| vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        // 40x40 block, top Z0: through hole 10..20 square, blind pocket 25..35 x 5..15 to Z-4
        let op = RoughOp { allowance: 0.2,
            cutter: Cutter::C3x17,
            start_z: 0.0,
            bottom_z: -10.0,
            layer_height: 5.0,
            clear_voids: false,
            faces: vec![
                RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 20.0), rect(25.0, 5.0, 35.0, 15.0)], tris: vec![], curved: false },
                RoughFace { z: -4.0, outer: rect(25.0, 5.0, 35.0, 15.0), holes: vec![], tris: vec![], curved: false },
            ],
            ..Default::default()
        };
        let pts = |op: &RoughOp| -> Vec<P3> { rough_moves(op, 2.0).unwrap().iter().filter_map(|m| if let Mv::Feed(p, _) = m { Some(*p) } else { None }).collect() };
        let sd = |p: P3, x0: f64, y0: f64, x1: f64, y1: f64| {
            let dx = (x0 - p[0]).max(p[0] - x1);
            let dy = (y0 - p[1]).max(p[1] - y1);
            if dx <= 0.0 && dy <= 0.0 { dx.max(dy) } else { dx.max(0.0).hypot(dy.max(0.0)) }
        };
        let mut hole_profiled = false;
        let unticked = pts(&op);
        for p in unticked.iter().filter(|p| p[2] < -1e-6) {
            let in_hole = sd(*p, 10.0, 10.0, 20.0, 20.0) < 0.0;
            let outside = sd(*p, 0.0, 0.0, 40.0, 40.0) > 0.0;
            let in_pocket = sd(*p, 25.0, 5.0, 35.0, 15.0) < 0.0;
            assert!(in_hole || outside || in_pocket, "cut inside the part at {p:?}");
            if in_hole {
                // only the profile: tool radius + allowance inside the hole wall
                assert!((sd(*p, 10.0, 10.0, 20.0, 20.0) + 1.7).abs() < 0.01, "cleared the through hole at {p:?}");
                hole_profiled |= (p[2] + 10.0).abs() < 1e-9;
            }
        }
        assert!(hole_profiled);
        // the blind pocket has a floor, so it is cleared either way
        assert!(unticked.iter().any(|p| sd(*p, 25.0, 5.0, 35.0, 15.0) < -1.7 && (p[2] + 3.8).abs() < 1e-9));
        // ticked: the blind pocket and the hole centre are cleared
        let all = pts(&RoughOp { allowance: 0.2, clear_voids: true, ..op });
        assert!(all.iter().any(|p| sd(*p, 25.0, 5.0, 35.0, 15.0) < -1.7 && (p[2] + 3.8).abs() < 1e-9));
        assert!(all.iter().any(|p| sd(*p, 10.0, 10.0, 20.0, 20.0) < -2.0 && p[2] < -1.0));
    }

    #[test]
    fn roughing_without_clear_voids_clears_counterbore_around_through_hole() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        // counterbore 10..30 to Z-3, through hole 16..24 below it
        let op = RoughOp { allowance: 0.2,
            cutter: Cutter::C3x17,
            start_z: 0.0,
            bottom_z: -10.0,
            layer_height: 5.0,
            clear_voids: false,
            faces: vec![
                RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 30.0)], tris: vec![], curved: false },
                RoughFace { z: -3.0, outer: sq(10.0, 30.0), holes: vec![sq(16.0, 24.0)], tris: vec![], curved: false },
            ],
            ..Default::default()
        };
        let layer = rough_layer(&op, -2.8).unwrap();
        assert_eq!(layer.through.len(), 1);
        let feeds: Vec<P3> = rough_moves(&op, 2.0).unwrap().iter().filter_map(|m| if let Mv::Feed(p, _) = m { Some(*p) } else { None }).collect();
        let at = |z: f64| feeds.iter().filter(move |p| (p[2] - z).abs() < 1e-9).copied().collect::<Vec<_>>();
        // inside the through hole only the profile ring (tool centre 1.7 in from the wall)
        let in_core = |p: &P3| p[0] > 17.8 && p[0] < 22.2 && p[1] > 17.8 && p[1] < 22.2;
        assert!(!feeds.iter().any(in_core), "cleared the through hole");
        // the counterbore floor layer is cleared right up to the through hole profile:
        // pocket passes reach inside the hole edge by no more than the allowance
        let cb = at(-2.8);
        let near_edge = cb.iter().filter(|p| {
            let d = (16.0 - p[0]).max(p[0] - 24.0).max((16.0 - p[1]).max(p[1] - 24.0));
            d > -0.4 && d < 1.6
        });
        assert!(near_edge.count() > 0, "counterbore not cleared up to the hole");
        // and the through hole is profiled at the bottom
        assert!(at(-10.0).iter().any(|p| (p[0] - 17.7).abs() < 0.01 || (p[0] - 22.3).abs() < 0.01));
    }
}

#[cfg(test)]
mod stock_tests {
    use super::*;

    #[test]
    fn stock_offset_and_absolute() {
        let (min, max) = ([0.0, 0.0, -10.0], [60.0, 40.0, 0.0]);
        let s = Stock { x: 4.0, y: 6.0, z: 2.0, offset: true, ..Default::default() };
        assert_eq!(s.bounds(min, max), ([-2.0, -3.0, -10.0], [62.0, 43.0, 2.0]));
        assert!(s.issues(min, max).is_empty());
        let a = Stock { x: 70.0, y: 40.0, z: 12.0, offset: false, ..Default::default() };
        assert_eq!(a.bounds(min, max), ([-5.0, 0.0, -10.0], [65.0, 40.0, 2.0]));
        let small = Stock { x: 50.0, y: 40.0, z: 12.0, offset: false, ..Default::default() };
        assert_eq!(small.issues(min, max).len(), 1);
        assert_eq!(Stock::default().bounds(min, max), (min, max));
    }
}

#[cfg(test)]
mod engrave_tests {
    use super::*;
    use crate::toolpath;

    #[test]
    fn engraving_cuts_to_depth_in_passes() {
        let job = StepJob {
            ops: vec![StepOp {
                enabled: true,
                kind: OpKind::Engrave(EngraveOp { z: -2.0, origin: [30.0, 20.0], text: "AB".into(), depth: 0.5, depth_per_pass: 0.25, ..Default::default() }),
            }],
            ..Default::default()
        };
        let mut s = Settings::default();
        s.safe_z = 10.0;
        let code = generate(&job, &s).unwrap();
        let tp = toolpath::parse(&code);
        let feeds: Vec<[f32; 3]> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.clone()).collect();
        let lo = feeds.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min);
        let hi = feeds.iter().filter(|p| p[2] < -1.9).map(|p| p[2]).fold(f32::NEG_INFINITY, f32::max);
        assert!((lo + 2.5).abs() < 1e-4, "{lo}");
        assert!(feeds.iter().any(|p| (p[2] + 2.25).abs() < 1e-4), "no first pass at -2.25");
        assert!(hi <= -2.0 + 1e-4);
        assert!(feeds.iter().all(|p| (p[0] - 30.0).abs() < 10.0 && (p[1] - 20.0).abs() < 5.0));
    }
}

#[cfg(test)]
mod engrave_bit_tests {
    use super::*;
    use crate::engrave::EngraveTool;

    #[test]
    fn engraving_bit_tool_change_and_depth_limit() {
        let s = Settings { safe_z: 10.0, ..Default::default() };
        let bit = s.engravers[2];
        let mut job = StepJob {
            ops: vec![StepOp { enabled: true, kind: OpKind::Engrave(EngraveOp { tool: EngraveTool::Bit(bit), ..Default::default() }) }],
            ..Default::default()
        };
        let code = generate(&job, &s).unwrap();
        assert!(code.contains(&format!("T{} M6 (engraver 90deg tip D0.3", bit.tool)), "{code}");
        let set = |job: &mut StepJob, depth: f64, tool: EngraveTool| {
            if let OpKind::Engrave(g) = &mut job.ops[0].kind {
                (g.depth, g.tool) = (depth, tool);
            }
        };
        set(&mut job, bit.max_depth + 0.1, EngraveTool::Bit(bit));
        let errs = generate(&job, &s).unwrap_err();
        assert!(errs.iter().any(|e| e.contains("max depth")), "{errs:?}");
        set(&mut job, 0.3, EngraveTool::Bit(crate::model::Engraver { id: 99, tool: 99, ..bit }));
        assert!(generate(&job, &s).unwrap_err().iter().any(|e| e.contains("T99 is not in the tool table")));
    }
}

#[cfg(test)]
mod sloped_tests {
    use super::*;
    use crate::toolpath;

    /// Ramp x 10..40, y 10..30, falling from Z-1 at x=10 to Z-4 at x=40.
    fn surf(x: f64) -> f64 {
        -1.0 - 3.0 * (x - 10.0) / 30.0
    }
    fn ramp() -> Vec<[P3; 3]> {
        let p = |x: f64, y: f64| [x, y, surf(x)];
        vec![[p(10.0, 10.0), p(40.0, 10.0), p(40.0, 30.0)], [p(10.0, 10.0), p(40.0, 30.0), p(10.0, 30.0)]]
    }

    #[test]
    fn slice_follows_the_slope() {
        let sq = vec![[10.0, 10.0], [40.0, 10.0], [40.0, 30.0], [10.0, 30.0]];
        let r = slice_below(&sq, &[], &ramp(), -2.5, &[]).unwrap();
        assert_eq!(r.len(), 1);
        let (lo, hi) = r[0].0.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[0]), hi.max(p[0])));
        assert!((lo - 25.0).abs() < 1e-6 && (hi - 40.0).abs() < 1e-6, "{lo} {hi}");
        assert!(slice_below(&sq, &[], &ramp(), -4.5, &[]).unwrap().is_empty());
    }

    #[test]
    fn sloped_pocket_steps_down_without_gouging() {
        let op = PocketOp {
            z: -4.0,
            start_z: 0.0,
            outer: vec![[10.0, 10.0], [40.0, 10.0], [40.0, 30.0], [10.0, 30.0]],
            floor: ramp(),
            cutter: Cutter::C3x17,
            ..Default::default()
        };
        assert_eq!(sloped_levels(&op), vec![-1.0, -2.0, -3.0, -4.0]);
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Pocket(op) }], ..Default::default() };
        let s = Settings { safe_z: 10.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let tp = toolpath::parse(&code);
        let r = 1.5;
        let mut levels_cut = std::collections::BTreeSet::new();
        for p in tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.clone()) {
            let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
            if z > -0.999 {
                continue; // above the ramp: entry moves
            }
            assert!(x >= 10.0 + r - 1e-3 && x <= 40.0 - r + 1e-3 && y >= 10.0 + r - 1e-3 && y <= 30.0 - r + 1e-3, "outside walls {p:?}");
            // highest surface point under the tool is on its -X side
            assert!(z >= surf(x - r) - 1e-3, "gouge at {p:?}: surface Z{}", surf(x - r));
            levels_cut.insert((z * 1000.0).round() as i64);
        }
        for z in [-1000, -2000, -3000] {
            assert!(levels_cut.contains(&z), "level {z} not cut: {levels_cut:?}");
        }
    }
}

#[cfg(test)]
mod rough_slope_tests {
    use super::*;
    use crate::toolpath;

    /// Part 40 x 20, bottom Z-10: flat top Z0 for x 0..20, then a ramp down
    /// to Z-6 at x = 40 (vertical walls elsewhere).
    fn height(x: f64) -> f64 {
        if x <= 20.0 {
            0.0
        } else if x <= 40.0 {
            -0.3 * (x - 20.0)
        } else {
            f64::NEG_INFINITY
        }
    }

    #[test]
    fn roughing_follows_slopes_and_profiles_each_height() {
        let rect = |x0: f64, y0: f64, x1: f64, y1: f64| vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        let p = |x: f64, y: f64| [x, y, height(x)];
        let ramp = vec![[p(20.0, 0.0), p(40.0, 0.0), p(40.0, 20.0)], [p(20.0, 0.0), p(40.0, 20.0), p(20.0, 20.0)]];
        let op = RoughOp { allowance: 0.2,
            cutter: Cutter::C3x17,
            start_z: 1.0,
            bottom_z: -10.0,
            faces: vec![
                RoughFace { z: 0.0, outer: rect(0.0, 0.0, 20.0, 20.0), holes: vec![], tris: vec![], curved: false },
                RoughFace { z: 0.0, outer: rect(20.0, 0.0, 40.0, 20.0), holes: vec![], tris: ramp, curved: false },
            ],
            ..Default::default()
        };
        let (r, a) = (1.5, op.allowance);
        let levels = rough_levels(&op);
        assert!(levels.iter().any(|z| (z + 5.8).abs() < 1e-9), "no layer at the foot of the slope: {levels:?}");
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 12.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let tp = toolpath::parse(&code);
        let pts: Vec<[f64; 3]> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64])).collect();
        for q in &pts {
            if q[1] >= r && q[1] <= 20.0 - r && q[0] - r < 40.0 {
                let h = height((q[0] - r).max(0.0));
                assert!(q[2] >= h + a - 1e-3, "gouge at {q:?}: surface {h} under the tool edge");
            }
        }
        // the outside profile steps down the slope: at Z-3 it runs just past
        // where the ramp is at Z-3.2 (x = 30.67), not around the full outline
        let wall = 20.0 + 3.2 / 0.3 + r + a;
        assert!(pts.iter().any(|q| (q[2] + 3.0).abs() < 1e-6 && (q[0] - wall).abs() < 0.05), "no cut at x {wall} on Z-3");
        // and the slope is cut down to its foot
        assert!(pts.iter().any(|q| q[2] < -5.0 && q[0] > 38.0 && q[1] > -2.0 && q[1] < 22.0), "slope not cut down to its foot");
    }
}

#[cfg(test)]
mod rough_curved_tests {
    use super::*;
    use crate::surface::{tessellate, Frame, Surf};
    use crate::toolpath;

    /// Block 40 x 40, top at Z0, bottom Z-10, with a spherical dome
    /// (R 12.5, centre Z-7.5) rising to Z5 in the middle.
    fn height(rho: f64) -> f64 {
        if rho < 10.0 {
            -7.5 + (12.5f64 * 12.5 - rho * rho).sqrt()
        } else {
            0.0
        }
    }

    #[test]
    fn roughing_steps_down_a_dome() {
        let f = Frame { o: [20.0, 20.0, -7.5], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
        let rim: Vec<P3> = (0..72).map(|i| {
            let a = TAU * i as f64 / 72.0;
            [20.0 + 10.0 * a.cos(), 20.0 + 10.0 * a.sin(), 0.0]
        }).collect();
        let tris = tessellate(&Surf::Sphere { f, r: 12.5 }, &[rim.clone()], true);
        assert!(tris.len() > 50);
        let hole: Vec<P2> = rim.iter().map(|p| [p[0], p[1]]).collect();
        let op = RoughOp {
            cutter: Cutter::C3x17,
            start_z: 6.0,
            bottom_z: -10.0,
            faces: vec![
                RoughFace { z: 0.0, outer: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]], holes: vec![hole.clone()], tris: vec![], curved: false },
                RoughFace { z: 5.0, outer: hole, holes: vec![], tris, curved: true },
            ],
            ..Default::default()
        };
        let (r, a) = (1.5, op.allowance);
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 16.0, ..Default::default() };
        let tp = toolpath::parse(&generate(&job, &s).unwrap());
        let pts: Vec<[f64; 3]> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64])).collect();
        let mut stepped = std::collections::BTreeSet::new();
        for q in &pts {
            if q[0] < r || q[0] > 40.0 - r || q[1] < r || q[1] > 40.0 - r {
                continue;
            }
            let rho = (q[0] - 20.0).hypot(q[1] - 20.0);
            let h = height((rho - r).max(0.0));
            assert!(q[2] >= h + a - 0.02, "gouge at {q:?}: dome Z{h} under the tool");
            if rho < 10.0 + r && q[2] > 0.5 && q[2] < 4.5 {
                stepped.insert((q[2] * 10.0).round() as i64);
            }
        }
        assert!(stepped.len() >= 3, "dome not stepped down: {stepped:?}");
    }
}


#[cfg(test)]
mod keepout_tests {
    use super::*;
    use crate::model::{ThreadGroup, ThreadSize};
    use crate::toolpath;

    fn thread_at(x: f64, y: f64, start_z: f64, thread: ThreadSize, length: f64) -> StepOp {
        StepOp {
            enabled: true,
            kind: OpKind::Thread(ThreadOp {
                faces: vec![],
                group: ThreadGroup { thread, start_z, length, cutter: Cutter::C2x12, locations: format!("{x}, {y}\n"), ..Default::default() },
            }),
        }
    }

    /// Feed paths of one op; each path is a run of connected feed moves.
    fn feeds(job: &StepJob, op_index: usize) -> Vec<Vec<[f64; 3]>> {
        // feed moves of one op only: generate a job with just that op + the thread items
        let mut j = job.clone();
        for (i, o) in j.ops.iter_mut().enumerate() {
            if i != op_index && !matches!(o.kind, OpKind::Thread(_)) {
                o.enabled = false;
            }
        }
        let mut s = Settings { safe_z: 15.0, ..Default::default() };
        s.thread_mills.iter_mut().for_each(|t| t.reach = 50.0);
        let code = generate(&j, &s).unwrap();
        // drop the thread item's own moves: they follow the "- bore" comment
        let cut = code.split("- bore)").next().unwrap();
        let tp = toolpath::parse(cut);
        tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64]).collect()).collect()
    }

    /// Every point along the feed moves within `rr` of `c` is at or above `top`;
    /// returns whether any move passed over that circle.
    fn assert_clear(paths: &[Vec<[f64; 3]>], c: P2, rr: f64, top: f64) -> bool {
        let mut over = false;
        for path in paths {
            for w in path.windows(2) {
                for k in 0..=20 {
                    let t = k as f64 / 20.0;
                    let q = [0, 1, 2].map(|i| w[0][i] + (w[1][i] - w[0][i]) * t);
                    if (q[0] - c[0]).hypot(q[1] - c[1]) < rr - 1e-3 {
                        assert!(q[2] >= top - 1e-6, "cut into the threaded hole at {q:?}");
                        over = true;
                    }
                }
            }
        }
        over
    }

    #[test]
    fn roughing_leaves_threaded_hole_alone() {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let rough = RoughOp { allowance: 0.2, cutter: Cutter::C3x17, start_z: 1.0, bottom_z: m.min[2], faces: rough_faces(&m), ..Default::default() };
        // thread item listed after the roughing
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(rough) }, thread_at(52.0, 32.0, 0.0, ThreadSize::M6, 2.0)], ..Default::default() };
        let paths = feeds(&job, 0);
        // major radius + tool radius + stock to leave
        assert_clear(&paths, [52.0, 32.0], 3.0 + 1.5 + 0.2, 0.0);
        // the pocket next to it is still roughed
        assert!(paths.iter().flatten().any(|p| p[2] < -3.0 && p[0] > 15.0 && p[0] < 45.0));
    }

    #[test]
    fn pocket_goes_round_threaded_hole() {
        let pocket = PocketOp { z: -4.0, start_z: 0.0, outer: vec![[15.0, 10.0], [45.0, 10.0], [45.0, 30.0], [15.0, 30.0]], cutter: Cutter::C3x17, ..Default::default() };
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Pocket(pocket) }, thread_at(30.0, 20.0, 0.0, ThreadSize::M3, 6.0)], ..Default::default() };
        let paths = feeds(&job, 0);
        assert_clear(&paths, [30.0, 20.0], 1.5 + 1.5, 0.0);
        assert!(paths.iter().flatten().any(|p| (p[2] + 4.0).abs() < 1e-6));
    }

    #[test]
    fn other_cuts_lift_over_threaded_hole() {
        let face = FaceOp { z: -1.0, start_z: 0.0, region: [0.0, 0.0, 60.0, 40.0], cutter: Cutter::C3x17, ..Default::default() };
        // thread item first this time
        let job = StepJob { ops: vec![thread_at(30.0, 20.0, 0.0, ThreadSize::M5, 5.0), StepOp { enabled: true, kind: OpKind::Face(face) }], ..Default::default() };
        let mut j = job.clone();
        j.ops.swap(0, 1);
        let paths = feeds(&j, 0);
        assert!(assert_clear(&paths, [30.0, 20.0], 2.5 + 1.5, 0.0), "facing never passed over the hole");
        assert!(paths.iter().flatten().any(|p| (p[2] + 1.0).abs() < 1e-6));
        // same result with the thread item first in the list
        let code_a = generate(&job, &Settings { safe_z: 15.0, ..Default::default() }).unwrap();
        let code_b = generate(&j, &Settings { safe_z: 15.0, ..Default::default() }).unwrap();
        let body = |c: &str| c.lines().filter(|l| !l.starts_with("(op")).filter(|l| l.starts_with('G')).filter(|l| !l.contains('I')).count();
        assert_eq!(body(&code_a), body(&code_b));
    }
}

#[cfg(test)]
mod travel_tests {
    use super::*;

    /// No rapid below `retract` comes within the tool radius of material that
    /// stands higher than the rapid (any face above it, seen from above).
    fn assert_rapids_clear(moves: &[Mv], faces: &[RoughFace], r: f64, retract: f64) -> usize {
        let mut pos = [0.0, 0.0, retract];
        let mut low = 0;
        for m in moves {
            let p = match *m {
                Mv::Rapid(p) => {
                    let z = pos[2].min(p[2]);
                    if z < retract - 1e-6 && ((p[0] - pos[0]).abs() > 1e-9 || (p[1] - pos[1]).abs() > 1e-9) {
                        low += 1;
                        let n = ((pos[0] - p[0]).hypot(pos[1] - p[1]) / 0.2).ceil() as usize;
                        for k in 0..=n {
                            let t = k as f64 / n.max(1) as f64;
                            let x = [pos[0] + (p[0] - pos[0]) * t, pos[1] + (p[1] - pos[1]) * t];
                            for f in faces.iter().filter(|f| f.z > z + 1e-6) {
                                let mut loops = vec![f.outer.clone()];
                                loops.extend(f.holes.iter().cloned());
                                let hit = inside(&loops, x) || boundary_dist(&loops, x) < r - 1e-6;
                                assert!(!hit, "rapid at Z{z} over material up to Z{} at {x:?}", f.z);
                            }
                        }
                    }
                    p
                }
                Mv::Feed(p, _) => p,
            };
            pos = p;
        }
        low
    }

    #[test]
    fn low_links_never_cross_higher_material() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        // boss on a flange, plus a second boss: hops between the bosses go low
        let faces = vec![
            RoughFace { z: 0.0, outer: sq(10.0, 30.0), holes: vec![], tris: vec![], curved: false },
            RoughFace { z: -1.0, outer: vec![[45.0, 10.0], [60.0, 10.0], [60.0, 30.0], [45.0, 30.0]], holes: vec![], tris: vec![], curved: false },
            RoughFace { z: -5.0, outer: vec![[0.0, 0.0], [70.0, 0.0], [70.0, 40.0], [0.0, 40.0]], holes: vec![sq(10.0, 30.0), vec![[45.0, 10.0], [60.0, 10.0], [60.0, 30.0], [45.0, 30.0]]], tris: vec![], curved: false },
        ];
        let op = RoughOp { cutter: Cutter::C3x17, layer_height: 1.0, start_z: 1.0, bottom_z: -10.0, faces: faces.clone(), stock_region: [-2.0, -2.0, 72.0, 42.0], ..Default::default() };
        let retract = 3.0;
        let moves = rough_moves(&op, retract).unwrap();
        let low = assert_rapids_clear(&moves, &faces, 1.5, retract);
        assert!(low > 0, "no hop was lowered");
        // the sample part too
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let op = RoughOp { cutter: Cutter::C3x17, start_z: 1.0, bottom_z: m.min[2], faces: rough_faces(&m), stock_region: [-2.0, -2.0, 62.0, 42.0], ..Default::default() };
        assert_rapids_clear(&rough_moves(&op, retract).unwrap(), &op.faces, 1.5, retract);
    }

    #[test]
    fn engraving_and_holes_are_visited_nearest_first() {
        // holes listed back and forth across the part are visited in a sweep
        let order = crate::gcode::hole_order(&[(0.0, 0.0), (100.0, 0.0), (10.0, 0.0), (90.0, 0.0), (20.0, 0.0)]);
        assert_eq!(order, vec![(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (90.0, 0.0), (100.0, 0.0)]);
        // engraving: total travel between letters is well under the font-order travel
        let op = EngraveOp { text: "OXO\nXOX".into(), height: 5.0, ..Default::default() };
        let paths = engrave::text_paths(&op).unwrap();
        let font_order: f64 = paths.windows(2).map(|w| d2(*w[0].last().unwrap(), w[1][0])).sum();
        let mut travel = 0.0;
        let mut pos: Option<P3> = None;
        for m in engrave_moves(&op, 2.0).unwrap() {
            if let Mv::Rapid(p) = m {
                if let Some(q) = pos {
                    travel += (p[0] - q[0]).hypot(p[1] - q[1]);
                }
            }
            pos = Some(match m {
                Mv::Rapid(p) | Mv::Feed(p, _) => p,
            });
        }
        assert!(travel <= font_order + 1e-6, "{travel} vs {font_order}");
    }
}


#[cfg(test)]
mod rough_stock_tests {
    use super::*;
    use crate::toolpath;

    /// a.step: Ø20 x 5 disc with an R3 top fillet, Ø10 pin to Z14 with a 3 mm
    /// chamfer. Height of the part at radius `rho` from its axis.
    fn height(rho: f64) -> f64 {
        if rho <= 2.0 {
            14.0
        } else if rho <= 5.0 {
            14.0 - (rho - 2.0)
        } else if rho <= 7.0 {
            5.0
        } else if rho <= 10.0 {
            2.0 + (9.0 - (rho - 7.0).powi(2)).max(0.0).sqrt()
        } else {
            f64::NEG_INFINITY
        }
    }

    #[test]
    fn a_step_rough_clears_round_the_part_but_not_the_stock_corners() {
        let m = crate::step::parse_model(include_str!("../testdata/a.step")).unwrap();
        let (smin, smax) = Stock::default().bounds(m.min, m.max); // the part's box: 20 x 20 x 14
        let op = RoughOp { cutter: Cutter::C3x17, start_z: smax[2], bottom_z: m.min[2], faces: rough_faces(&m), stock_region: [smin[0], smin[1], smax[0], smax[1]], ..Default::default() };
        let r = 1.5;
        let levels = rough_levels(&op);
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let tp = toolpath::parse(&generate(&job, &s).unwrap());
        let paths: Vec<Vec<[f64; 3]>> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64]).collect()).collect();
        // no gouge: the highest part point under the tool is at or below it;
        // and the tool centre never goes more than r (+ open edge) outside
        // the Ø20 outline - the stock corners are left
        for p in paths.iter().flatten() {
            let rho = p[0].hypot(p[1]);
            let h = height((rho - r).max(0.0));
            assert!(p[2] >= h - 0.03, "gouge at {p:?}: part Z{h} under the tool");
            // (the outline here comes from the fillet mesh, grown by MESH_GROW)
            assert!(rho <= 10.0 + MESH_GROW + r + OPEN_EDGE + 0.01, "cut beyond the band round the part at {p:?}");
        }
        // every layer reaches all the material that is not part out to half a
        // tool diameter past the outline (sampled on a 1 mm grid, away from
        // the part walls)
        let segs: Vec<([f64; 3], [f64; 3])> = paths.iter().flat_map(|p| p.windows(2).map(|w| (w[0], w[1]))).collect();
        for &z in &levels {
            let at_z: Vec<&([f64; 3], [f64; 3])> = segs.iter().filter(|(a, b)| (a[2] - z).abs() < 1e-6 && (b[2] - z).abs() < 1e-6).collect();
            for i in 0..=20 {
                for j in 0..=20 {
                    let q = [-10.0 + i as f64, -10.0 + j as f64];
                    let rho = q[0].hypot(q[1]);
                    if height((rho - 0.3).max(0.0)) > z || rho > 10.0 + r {
                        continue; // part (or within 0.3 mm of its wall), or beyond the band
                    }
                    let d = at_z
                        .iter()
                        .map(|(a, b)| {
                            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                            let l2 = dx * dx + dy * dy;
                            let t = if l2 < 1e-12 { 0.0 } else { (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
                            (a[0] + dx * t - q[0]).hypot(a[1] + dy * t - q[1])
                        })
                        .fold(f64::INFINITY, f64::min);
                    assert!(d <= r + 1e-3, "stock at X{} Y{} left above the base on layer Z{z} ({d:.2} mm from the cutter)", q[0], q[1]);
                }
            }
        }
    }
}

#[cfg(test)]
mod start_tests {
    use super::*;

    #[test]
    fn every_op_moves_to_its_start_before_going_down() {
        let pocket = |x: f64| StepOp {
            enabled: true,
            kind: OpKind::Pocket(PocketOp { z: -2.0, outer: vec![[x, 0.0], [x + 20.0, 0.0], [x + 20.0, 20.0], [x, 20.0]], cutter: Cutter::C3x17, ..Default::default() }),
        };
        let engrave = StepOp { enabled: true, kind: OpKind::Engrave(EngraveOp { origin: [50.0, 50.0], ..Default::default() }) };
        let job = StepJob { ops: vec![pocket(0.0), pocket(100.0), engrave], ..Default::default() };
        let s = Settings { safe_z: 10.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let lines: Vec<&str> = code.lines().collect();
        let mut ops = 0;
        for (i, l) in lines.iter().enumerate().filter(|(_, l)| l.starts_with("(op ")) {
            ops += 1;
            // the first move after the op header that is not at the rapid
            // plane must come after an XY move at the rapid plane
            let rest = &lines[i + 1..];
            let xy = rest.iter().position(|m| m.starts_with("G0 X")).expect("no XY move");
            let down = rest.iter().position(|m| m.starts_with("G0 Z") && !m.ends_with(&format!("Z{}", fmt(s.safe_z))) || m.starts_with("G1")).unwrap();
            assert!(xy < down, "{l}: goes down before moving to its start");
        }
        assert_eq!(ops, 3);
    }
}

#[cfg(test)]
mod facing_outline_tests {
    use super::*;
    use crate::toolpath;

    #[test]
    fn face_op_clears_above_the_outline_band_only() {
        // a round Ø20 part (outline seen from above) in square stock
        let circle: Vec<P2> = (0..90).map(|i| {
            let a = TAU * i as f64 / 90.0;
            [10.0 * a.cos(), 10.0 * a.sin()]
        }).collect();
        let face = FaceOp { z: 10.0, start_z: 12.0, region: [-10.0, -10.0, 10.0, 10.0], outline: vec![circle], cutter: Cutter::C3x17, ..Default::default() };
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Face(face) }], ..Default::default() };
        let s = Settings { safe_z: 20.0, ..Default::default() };
        let tp = toolpath::parse(&generate(&job, &s).unwrap());
        let r = 1.5;
        let paths: Vec<Vec<[f64; 3]>> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).map(|p| p.pts.iter().map(|q| [q[0] as f64, q[1] as f64, q[2] as f64]).collect()).collect();
        // the tool centre stays within outline + half a diameter (+ open edge)
        for p in paths.iter().flatten() {
            assert!(p[0].hypot(p[1]) <= 10.0 + r + OPEN_EDGE + 0.01, "faced beyond the band at {p:?}");
        }
        // all of the outline is faced at both levels
        let segs: Vec<([f64; 3], [f64; 3])> = paths.iter().flat_map(|p| p.windows(2).map(|w| (w[0], w[1]))).collect();
        for z in [11.0, 10.0] {
            let at: Vec<&([f64; 3], [f64; 3])> = segs.iter().filter(|(a, b)| (a[2] - z).abs() < 1e-6 && (b[2] - z).abs() < 1e-6).collect();
            assert!(!at.is_empty(), "no level at Z{z}");
            for i in -10..=10 {
                for j in -10..=10 {
                    let q = [i as f64, j as f64];
                    if q[0].hypot(q[1]) > 10.0 {
                        continue;
                    }
                    let d = at
                        .iter()
                        .map(|(a, b)| {
                            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                            let l2 = dx * dx + dy * dy;
                            let t = if l2 < 1e-12 { 0.0 } else { (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
                            (a[0] + dx * t - q[0]).hypot(a[1] + dy * t - q[1])
                        })
                        .fold(f64::INFINITY, f64::min);
                    assert!(d <= r + 1e-3, "X{} Y{} not faced at Z{z}", q[0], q[1]);
                }
            }
        }
        // every row is climb milled: cutting moves at depth run +X
        for p in &paths {
            for w in p.windows(2) {
                if (w[0][2] - w[1][2]).abs() < 1e-9 && (w[0][2] - 10.0).abs() < 1e-6 {
                    assert!(w[1][0] >= w[0][0] - 1e-9, "conventional cut {w:?}");
                }
            }
        }
    }

    #[test]
    fn rows_split_by_the_outline_go_up_between_pieces() {
        // two separate squares: rows cross the gap between them
        let sq = |x: f64| vec![[x, 0.0], [x + 10.0, 0.0], [x + 10.0, 10.0], [x, 10.0]];
        let face = FaceOp { z: 0.0, start_z: 2.0, depth_per_pass: 1.0, outline: vec![sq(0.0), sq(30.0)], cutter: Cutter::C3x17, ..Default::default() };
        let retract = 5.0;
        let moves = face_moves(&face, retract);
        // the gap between the parts (x 11.5..28.5, outside both bands) is never
        // faced, so stock may stand there up to the start height: hops across
        // it stay above that (0.5 over it on the first level, the retract later)
        let mut pos = [0.0, 0.0, retract];
        let mut hops = 0;
        for m in &moves {
            let p = match *m {
                Mv::Rapid(p) | Mv::Feed(p, _) => p,
            };
            if let Mv::Rapid(q) = m {
                if (pos[0] < 15.0 && q[0] > 25.0) || (pos[0] > 25.0 && q[0] < 15.0) {
                    hops += 1;
                    assert!(q[2] >= 2.0 + 0.5 - 1e-9, "low hop over unfaced stock {pos:?} -> {q:?}");
                }
            }
            pos = p;
        }
        assert!(hops > 0);
        assert!(moves.iter().any(|m| matches!(m, Mv::Feed(p, _) if p[0] > 30.0 && p[2].abs() < 1e-9)));
        assert!(moves.iter().any(|m| matches!(m, Mv::Feed(p, _) if p[0] < 10.0 && p[2].abs() < 1e-9)));
    }
}


#[cfg(test)]
mod origin_tests {
    use super::*;
    use crate::toolpath;

    #[test]
    fn presets_follow_the_stock_and_set_makes_custom() {
        let (lo, hi) = ([-10.0, -5.0, 0.0], [30.0, 15.0, 8.0]);
        let mut o = Origin::default();
        o.resolve(lo, hi);
        assert_eq!(o.at, [0.0; 3]);
        for (xy, want) in [(OriginXY::Center, [10.0, 5.0]), (OriginXY::BottomLeft, [-10.0, -5.0]), (OriginXY::BottomRight, [30.0, -5.0]), (OriginXY::TopLeft, [-10.0, 15.0]), (OriginXY::TopRight, [30.0, 15.0])] {
            o.xy = xy;
            o.resolve(lo, hi);
            assert_eq!([o.at[0], o.at[1]], want, "{xy:?}");
        }
        o.z = OriginZ::StockTop;
        o.resolve(lo, hi);
        assert_eq!(o.at[2], 8.0);
        o.z = OriginZ::StockBase;
        o.resolve(lo, hi);
        assert_eq!(o.at[2], 0.0);
        // picking an edge sets one axis, keeps the other where it was
        o.set(Some(3.0), None, None);
        assert_eq!((o.xy, o.at), (OriginXY::Custom, [3.0, 15.0, 0.0]));
        o.resolve([0.0; 3], [1.0; 3]);
        assert_eq!(o.at, [3.0, 15.0, 0.0], "custom stays put");
    }

    #[test]
    fn shift_rewrites_positions_but_not_arcs_or_comments() {
        let code = "G0 X10 Y-2.5 Z5 (op 1: Face to Z8)\nG3 X1 Y2 I-0.5 J0 F100\nT3 M6 (tool X1)\nG1 Z-1\n";
        let s = shift_code(code, [10.0, -2.5, 8.0]);
        assert_eq!(s, "G0 X0 Y0 Z-3 (op 1: Face to Z8)\nG3 X-9 Y4.5 I-0.5 J0 F100\nT3 M6 (tool X1)\nG1 Z-9\n");
        assert_eq!(shift_code(code, [0.0; 3]), code);
    }

    #[test]
    fn generated_program_moves_with_the_origin_and_maps_back() {
        let pocket = PocketOp { z: -2.0, outer: vec![[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]], cutter: Cutter::C3x17, ..Default::default() };
        let mut job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Pocket(pocket) }], ..Default::default() };
        let s = Settings { safe_z: 10.0, ..Default::default() };
        let plain = toolpath::parse(&generate(&job, &s).unwrap());
        job.origin = Origin { xy: OriginXY::Custom, z: OriginZ::Custom, at: [5.0, -3.0, 1.5] };
        let code = generate(&job, &s).unwrap();
        assert!(code.contains("G0 Z8.5"), "rapid plane Z10 written relative to Z1.5");
        let mut moved = toolpath::parse(&code);
        assert!((moved.min[0] - (plain.min[0] - 5.0)).abs() < 1e-4);
        moved.translate([5.0, -3.0, 1.5]);
        for (a, b) in plain.paths.iter().zip(&moved.paths) {
            for (p, q) in a.pts.iter().zip(&b.pts) {
                for k in 0..3 {
                    assert!((p[k] - q[k]).abs() < 1e-3, "{p:?} vs {q:?}");
                }
            }
        }
    }
}

#[cfg(test)]
mod z_anchor_tests {
    use super::*;

    #[test]
    fn anchor_places_the_part_in_the_stock_height() {
        // part Z 2..10, 4 mm extra stock
        let (min, max) = ([0.0, 0.0, 2.0], [10.0, 10.0, 10.0]);
        let z = |a: ZAnchor, offset: bool, v: f64| {
            let (lo, hi) = Stock { x: 0.0, y: 0.0, z: v, offset, z_anchor: a }.bounds(min, max);
            (lo[2], hi[2])
        };
        assert_eq!(z(ZAnchor::Bottom, true, 4.0), (2.0, 14.0));
        assert_eq!(z(ZAnchor::Top, true, 4.0), (-2.0, 10.0));
        assert_eq!(z(ZAnchor::Middle, true, 4.0), (0.0, 12.0));
        // absolute size works the same way
        assert_eq!(z(ZAnchor::Top, false, 12.0), (-2.0, 10.0));
        assert_eq!(z(ZAnchor::Middle, false, 12.0), (0.0, 12.0));
        // old jobs without the field keep the part on the stock base
        let old: Stock = serde_json::from_str(r#"{"x": 1, "y": 1, "z": 3, "offset": true}"#).unwrap();
        assert_eq!(old.z_anchor, ZAnchor::Bottom);
        let s: Stock = serde_json::from_str(r#"{"z": 3, "z_anchor": "middle"}"#).unwrap();
        assert_eq!(s.z_anchor, ZAnchor::Middle);
    }
}

#[cfg(test)]
mod finish_tests {
    use super::*;
    use crate::toolpath;

    /// a.step's height at radius `rho` (see rough_stock_tests).
    fn height(rho: f64) -> f64 {
        if rho <= 2.0 {
            14.0
        } else if rho <= 5.0 {
            14.0 - (rho - 2.0)
        } else if rho <= 7.0 {
            5.0
        } else if rho <= 10.0 {
            2.0 + (9.0 - (rho - 7.0).powi(2)).max(0.0).sqrt()
        } else {
            f64::NEG_INFINITY
        }
    }

    /// Distance from radius `rho` to the part's material at height `z`
    /// (material: where the part stands above z; 0 inside it).
    fn dist_to_material(rho: f64, z: f64) -> f64 {
        let mut d = f64::INFINITY;
        let mut m = 0.0;
        while m <= 10.0 {
            if height(m) > z + 1e-6 {
                d = d.min((rho - m).abs());
            }
            m += 0.0005;
        }
        d
    }

    fn a_step_job(with_rough: bool) -> (StepJob, crate::step::Model, (P3, P3)) {
        let m = crate::step::parse_model(include_str!("../testdata/a.step")).unwrap();
        let stock = Stock { z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        let base = RoughOp { cutter: Cutter::C3x17, start_z: stock.1[2], bottom_z: m.min[2], faces: rough_faces(&m), stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]], ..Default::default() };
        let finish = RoughOp { start_z: m.max[2], layer_height: FINISH_STEP, ..base.clone() };
        let mut ops = vec![];
        if with_rough {
            ops.push(StepOp { enabled: true, kind: OpKind::Rough(base) });
        }
        ops.push(StepOp { enabled: true, kind: OpKind::Finish(finish) });
        (StepJob { ops, ..Default::default() }, m, stock)
    }

    #[test]
    fn laps_run_round_the_walls_at_tool_radius() {
        let (job, _, _) = a_step_job(false);
        let code = generate(&job, &Settings { safe_z: 25.0, ..Default::default() }).unwrap();
        let tp = toolpath::parse(&code);
        let (r, mut laps) = (1.5, 0);
        for p in tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid) {
            for (k, w) in p.pts.windows(2).enumerate() {
                // the laps: level moves along the wall (not the short links
                // across to the next layer's lap, which drop straight after)
                if (w[0][2] - w[1][2]).abs() > 1e-6 || ((w[0][0] - w[1][0]).hypot(w[0][1] - w[1][1])) < 0.05 {
                    continue;
                }
                if let Some(n) = p.pts.get(k + 2) {
                    if (n[0] - w[1][0]).abs() < 1e-6 && (n[1] - w[1][1]).abs() < 1e-6 && n[2] < w[1][2] {
                        continue;
                    }
                }
                let (z, rho) = (w[1][2] as f64, (w[1][0] as f64).hypot(w[1][1] as f64));
                // r from the nearest wall (mesh outlines are grown by up to
                // 1.5 MESH_GROW on the material side)
                let d = dist_to_material(rho, z);
                assert!(d >= r - 0.03 && d <= r + 1.5 * MESH_GROW + 0.03, "lap at Z{z} rho {rho}: {d} from the part");
                laps += 1;
            }
        }
        assert!(laps > 1000, "{laps}");
        // 0.1 mm steps from the part top down: about 140 levels
        let OpKind::Finish(f) = &job.ops[0].kind else { unreachable!() };
        assert!(rough_levels(f).len() >= 140);
    }

    #[test]
    fn rough_then_finish_leaves_the_part_at_size_without_gouges() {
        let (job, m, stock) = a_step_job(true);
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let left = |job: &StepJob| {
            let code = generate(job, &s).unwrap();
            let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 250);
            while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
            assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
            let part = sim.map.surface_of(crate::sim::top_tris(&m));
            let d: Vec<f32> = sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()).map(|(z, p)| z - p).collect();
            assert!(d.iter().all(|&d| d > -0.07), "gouge {}", d.iter().cloned().fold(0f32, f32::min));
            // waterline steps leave cusps up to the Z step (plus the mesh
            // margin on curved walls): count what is clearly more
            d.iter().filter(|&&d| d > 0.25).count()
        };
        let rough_only = StepJob { ops: job.ops[..1].to_vec(), ..job.clone() };
        let (before, after) = (left(&rough_only), left(&job));
        assert!(after * 4 < before, "cells with > 0.25 mm left: rough {before}, rough + finish {after}");
    }

    #[test]
    fn warns_without_a_rough_before_it() {
        let (job, _, _) = a_step_job(false);
        assert_eq!(finish_warnings(&job.ops).len(), 1);
        let (mut job, _, _) = a_step_job(true);
        assert!(finish_warnings(&job.ops).is_empty());
        job.ops[0].enabled = false;
        assert_eq!(finish_warnings(&job.ops)[0].0, 1);
        job.ops.swap(0, 1);
        job.ops[1].enabled = true;
        assert_eq!(finish_warnings(&job.ops).len(), 1, "rough after the finish does not count");
    }
}

#[cfg(test)]
mod adaptive_band_tests {
    use super::*;
    use crate::toolpath;

    fn a_step(strategy: Strategy, finish_layers: bool) -> (StepJob, crate::step::Model, (P3, P3)) {
        let m = crate::step::parse_model(include_str!("../testdata/a.step")).unwrap();
        let stock = Stock { x: 10.0, y: 10.0, z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        let op = RoughOp {
            cutter: Cutter::C3x17,
            start_z: stock.1[2],
            bottom_z: m.min[2],
            faces: rough_faces(&m),
            stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
            strategy,
            finish_layers,
            ..Default::default()
        };
        (StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() }, m, stock)
    }

    fn feed_rho(job: &StepJob) -> (Vec<f64>, usize) {
        let code = generate(job, &Settings { safe_z: 25.0, ..Default::default() }).unwrap();
        let tp = toolpath::parse(&code);
        let pts: Vec<f64> = tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid).flat_map(|p| p.pts.iter()).filter(|p| p[2] < 4.0).map(|p| (p[0] as f64).hypot(p[1] as f64)).collect();
        (pts, code.lines().count())
    }

    #[test]
    fn adaptive_mills_the_outside_profile_with_trochoids() {
        let (r, d) = (1.5, 3.0);
        let (std_rho, _) = feed_rho(&a_step(Strategy::Standard, true).0);
        let (ad_rho, _) = feed_rho(&a_step(Strategy::Adaptive, true).0);
        let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
        let edge = 10.0 + 1.5 * MESH_GROW + r + OPEN_EDGE;
        // standard: one pass round the outside at tool radius
        assert!(max(&std_rho) <= edge + 0.01, "{}", max(&std_rho));
        // adaptive: the loops swing out over the widened band, but no further
        let m = max(&ad_rho);
        assert!(m > edge + 0.5 * d && m <= edge + 2.0 * TROCHOID_R * d + OPEN_EDGE + 0.01, "adaptive reaches rho {m}");
        // and many points out there: loops, not one pass
        assert!(ad_rho.iter().filter(|&&p| p > edge + 0.3 * d).count() > 200);
    }

    #[test]
    fn adaptive_rough_does_not_gouge_and_finish_each_layer_toggles_the_contours() {
        for finish in [true, false] {
            let (job, m, stock) = a_step(Strategy::Adaptive, finish);
            let s = Settings { safe_z: 25.0, ..Default::default() };
            let code = generate(&job, &s).unwrap();
            let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 250);
            while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
            assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
            let part = sim.map.surface_of(crate::sim::top_tris(&m));
            let worst = sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()).map(|(z, p)| p - z).fold(0f32, f32::max);
            assert!(worst < 0.07, "finish_layers {finish}: gouge {worst}");
        }
        let (_, with) = feed_rho(&a_step(Strategy::Adaptive, true).0);
        let (_, without) = feed_rho(&a_step(Strategy::Adaptive, false).0);
        assert!(without < with, "contour passes add moves: {without} vs {with}");
        // old jobs without the field finish each layer
        let old: RoughOp = serde_json::from_str("{}").unwrap();
        assert!(old.finish_layers);
    }

    #[test]
    fn mixed_is_adaptive_round_the_outline_and_standard_inside() {
        let (r, d) = (1.5, 3.0);
        let edge = 10.0 + 1.5 * MESH_GROW + r + OPEN_EDGE;
        let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
        let (std_rho, std_n) = feed_rho(&a_step(Strategy::Standard, true).0);
        // (adaptive at its own default stepover, as the UI would set it)
        let mut ad = a_step(Strategy::Adaptive, true).0;
        if let OpKind::Rough(o) = &mut ad.ops[0].kind {
            o.stepover_pct = STEPOVER_ADAPTIVE;
        }
        let (ad_rho, ad_n) = feed_rho(&ad);
        let (mx_rho, mx_n) = feed_rho(&a_step(Strategy::Mixed, true).0);
        // the outline band is cut with loops, as adaptive
        assert!(max(&mx_rho) > edge + 0.5 * d, "{}", max(&mx_rho));
        assert!(mx_rho.iter().filter(|&&p| p > edge + 0.3 * d).count() > 200);
        // inside the part: standard passes
        let inner = |v: &[f64]| v.iter().filter(|&&p| p < 6.0).count();
        eprintln!("std {std_n} {} ad {ad_n} {} mixed {mx_n} {}", inner(&std_rho), inner(&ad_rho), inner(&mx_rho));
        assert!(mx_n < ad_n && mx_n > std_n, "lines std {std_n} mixed {mx_n} adaptive {ad_n}");
        // and it does not gouge or rapid into the stock
        let (job, m, stock) = a_step(Strategy::Mixed, true);
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 250);
        while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
        assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
        let part = sim.map.surface_of(crate::sim::top_tris(&m));
        let worst = sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()).map(|(z, p)| p - z).fold(0f32, f32::max);
        assert!(worst < 0.07, "gouge {worst}");
    }

    #[test]
    fn rough_cuts_each_pocket_all_the_way_down_before_the_next() {
        let m = crate::step::parse_model(include_str!("../testdata/bracket.step")).unwrap();
        let stock = Stock { x: 5.0, y: 5.0, z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        for strategy in [Strategy::Standard, Strategy::Adaptive, Strategy::Mixed] {
            let op = RoughOp {
                cutter: Cutter::C3x17,
                start_z: stock.1[2],
                bottom_z: m.min[2],
                faces: rough_faces(&m),
                stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
                strategy,
                ..Default::default()
            };
            let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
            let s = Settings { safe_z: 25.0, ..Default::default() };
            let code = generate(&job, &s).unwrap();
            // the cuts between climbs: each goes down layer by layer, so a
            // new one (Z back up) starts only when the one before is done
            let tp = toolpath::parse(&code);
            let mut climbs = 0;
            for p in &tp.paths {
                if p.kind == toolpath::MoveKind::Rapid {
                    let up = p.pts.windows(2).any(|w| w[1][2] > w[0][2] + 1e-4);
                    climbs += up as usize;
                }
            }
            // facing, the band round the outside, the pocket and two holes:
            // a handful of climbs (cut layer by layer it was one per cut)
            assert!(climbs <= 8, "{strategy:?}: {climbs} climbs");
            let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 250);
            while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
            assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{strategy:?}: {:?}", sim.events);
            let part = sim.map.surface_of(crate::sim::top_tris(&m));
            let worst = sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()).map(|(z, p)| p - z).fold(0f32, f32::max);
            assert!(worst < 0.07, "{strategy:?}: gouge {worst}");
        }
        // finishing too: each wall all the way down before the next
        let mut op = RoughOp {
            cutter: Cutter::C3x17,
            start_z: stock.1[2],
            bottom_z: m.min[2],
            faces: rough_faces(&m),
            stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
            layer_height: FINISH_STEP,
            ..Default::default()
        };
        op.start_z = op.part_top();
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Finish(op) }], ..Default::default() };
        let code = generate(&job, &Settings { safe_z: 25.0, ..Default::default() }).unwrap();
        let climbs = toolpath::parse(&code).paths.iter().filter(|p| p.kind == toolpath::MoveKind::Rapid && p.pts.windows(2).any(|w| w[1][2] > w[0][2] + 1e-4)).count();
        assert!(climbs <= 10, "finish: {climbs} climbs");
    }

    #[test]
    fn inside_only_clears_the_chamfer_but_not_beside_the_outside_walls() {
        // 40 x 30 x 10 box at the origin, top edges chamfered 2 mm, a pocket
        let m = crate::step::parse_model(include_str!("../testdata/chamfer_box.step")).unwrap();
        let stock = Stock { x: 14.0, y: 14.0, z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        // (to X±27, Y±22)
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let run = |inside_only: bool| {
            let op = RoughOp {
                cutter: Cutter::C3x17,
                layer_height: 0.5,
                start_z: stock.1[2],
                bottom_z: m.min[2],
                faces: rough_faces(&m),
                stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
                inside_only,
                ..Default::default()
            };
            let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
            let code = generate(&job, &s).unwrap();
            let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 300);
            while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
            assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
            let part = sim.map.surface_of(crate::sim::top_tris(&m));
            let (mut cells, mut at, mut worst) = (0, 0, 0f32);
            for (z, p) in sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()) {
                cells += 1;
                at += (z - p < 0.6) as usize;
                worst = worst.max(p - z);
            }
            assert!(worst < 0.07, "inside_only {inside_only}: gouge {worst}");
            let zat = |x: f64, y: f64| {
                let (i, j) = (((x - sim.map.x0) / sim.map.cell) as usize, ((y - sim.map.y0) / sim.map.cell) as usize);
                sim.map.z[j * sim.map.nx + i] as f64
            };
            (code, 100.0 * at as f64 / cells as f64, zat(20.3, 0.0), zat(23.5, 0.0))
        };
        let (_, full_at, full_beside, _) = run(false);
        let (code, at, beside, out) = run(true);
        // the top, the chamfer and the pocket are roughed just as well
        assert!(at >= full_at - 0.5, "at size {at}% vs {full_at}%");
        // beside the vertical walls (below the chamfer) nothing is cut, and
        // stock further out is left at the top
        assert!(beside >= 8.0 - 0.01 && full_beside < 1.0, "cut beside the wall to Z{beside} (full roughing: Z{full_beside})");
        // (the chamfer's loops reach a tool radius past the outline at most)
        assert!((out - stock.1[2]).abs() < 1e-3, "stock out at Z{out}");
        let tp = toolpath::parse(&code);
        for p in tp.paths.iter().filter(|p| p.kind != toolpath::MoveKind::Rapid) {
            for q in &p.pts {
                if (q[2] as f64) < 8.0 - 1e-3 {
                    assert!((q[0].abs() as f64) < 20.0 && (q[1].abs() as f64) < 15.0, "cuts outside the wall at {q:?}");
                }
            }
        }
    }

    #[test]
    fn mixed_band_has_trochoids_only_on_its_outermost_loop() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        let pocket = |outer_only: bool| PocketOp {
            outer: sq(0.0, 60.0),
            islands: vec![sq(20.0, 40.0).into_iter().rev().collect()],
            keep_islands: true,
            open_outer: true,
            no_contour: true,
            finish: 0.0,
            cutter: Cutter::C3x17,
            stepover_pct: 40.0,
            start_z: 0.0,
            z: -1.0,
            depth_per_pass: 2.0,
            strategy: Strategy::Adaptive,
            entry: Entry::Plunge,
            adaptive_outer_only: outer_only,
            ..Default::default()
        };
        let (r, troch_r) = (1.5, TROCHOID_R * 3.0);
        // feed points on the cutting level, by distance from the outer boundary
        let split = |op: &PocketOp| {
            let m = pocket_moves(op, 5.0, [0.0, 0.0]).unwrap();
            let pts: Vec<P2> = m.iter().filter_map(|mv| match mv { Mv::Feed(p, _) if p[2] < -0.5 => Some([p[0], p[1]]), _ => None }).collect();
            let zone = |p: &P2| p[0].min(p[1]).min(60.0 - p[0]).min(60.0 - p[1]) <= r + 2.0 * troch_r + 0.01;
            // total turning outside the ring: trochoids turn a full circle per loop
            let turn: f64 = pts.windows(3).filter(|w| !zone(&w[1])).map(|w| {
                let (a, b) = ([w[1][0] - w[0][0], w[1][1] - w[0][1]], [w[2][0] - w[1][0], w[2][1] - w[1][1]]);
                (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]).abs()
            }).sum();
            (pts.iter().filter(|p| zone(p)).count(), turn / TAU)
        };
        let (ring, rest) = split(&pocket(true));
        let (_, rest_ad) = split(&pocket(false));
        eprintln!("ring {ring} turns off ring mixed {rest} adaptive {rest_ad}");
        assert!(ring > 500 && rest < 60.0, "mixed: {ring} points in the outer ring, {rest} turns elsewhere");
        // full adaptive loops all over the band
        assert!(rest_ad > 5.0 * rest, "adaptive {rest_ad} vs mixed {rest} turns off the outer ring");
    }
}


#[cfg(test)]
mod adaptive_cleanup_tests {
    use super::*;

    #[test]
    fn wide_adaptive_pocket_is_cleared_everywhere() {
        // 40 x 26 pocket, 2 deep, Ø3: the trochoid loops from opposite walls
        // stop short of each other; the cleanup must clear the middle
        for (w, h) in [(40.0, 26.0), (30.0, 30.0), (37.0, 11.0)] {
            let op = PocketOp {
                z: -2.0,
                outer: vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]],
                cutter: Cutter::C3x17,
                strategy: Strategy::Adaptive,
                finish: 0.0,
                ..Default::default()
            };
            let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Pocket(op) }], ..Default::default() };
            let s = Settings { safe_z: 10.0, ..Default::default() };
            let code = generate(&job, &s).unwrap();
            let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), [-2.0, -2.0, -5.0], [w + 2.0, h + 2.0, 0.0], 300);
            while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
            assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{w}x{h}: {:?}", sim.events);
            let m = &sim.map;
            let mut left = 0;
            for j in 0..m.ny {
                for i in 0..m.nx {
                    let [x, y] = m.centre(i, j);
                    let wall = x.min(w - x).min(y).min(h - y);
                    // reachable: within a radius (less a margin) of where the
                    // tool centre may go (corners stay round)
                    let r = 1.5;
                    let (dx, dy) = ((r - x).max(x - (w - r)).max(0.0), (r - y).max(y - (h - r)).max(0.0));
                    if dx.hypot(dy) < r - 0.1 && m.z[j * m.nx + i] > -2.0 + 1e-3 {
                        left += 1;
                    }
                    // and never into the walls
                    if wall < -0.05 {
                        assert!(m.z[j * m.nx + i] > -1e-3, "{w}x{h}: wall cut at ({x}, {y})");
                    }
                }
            }
            assert_eq!(left, 0, "{w}x{h}: cells left above the floor");
        }
    }
}

#[cfg(test)]
mod single_pass_tests {
    use super::*;

    #[test]
    fn adaptive_stepover_default() {
        assert_eq!(stepover_for(Strategy::Standard, Strategy::Adaptive, 40.0), 20.0);
        assert_eq!(stepover_for(Strategy::Adaptive, Strategy::Standard, 20.0), 40.0);
        assert_eq!(stepover_for(Strategy::Standard, Strategy::Adaptive, 33.0), 33.0, "a chosen stepover stays");
        assert_eq!(stepover_for(Strategy::Adaptive, Strategy::Adaptive, 20.0), 20.0);
    }

    /// A 5 mm wide square band (40 x 40 less a 30 x 30 island), 2 deep, Ø3.
    fn band_job() -> StepJob {
        let op = PocketOp {
            z: -2.0,
            outer: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]],
            islands: vec![vec![[5.0, 5.0], [35.0, 5.0], [35.0, 35.0], [5.0, 35.0]]],
            keep_islands: true,
            cutter: Cutter::C3x17,
            strategy: Strategy::Adaptive,
            stepover_pct: STEPOVER_ADAPTIVE,
            finish: 0.0,
            ..Default::default()
        };
        StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Pocket(op) }], ..Default::default() }
    }

    #[test]
    fn single_pass_runs_down_the_middle_as_wide_as_the_band() {
        // where the tool centre may go in the band: 1.5..38.5 less the
        // island grown by 1.5 (6.5..33.5 with round corners): 2 mm wide
        let allowed = offset_region(&[[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]], &[vec![[5.0, 5.0], [35.0, 5.0], [35.0, 35.0], [5.0, 35.0]]], 1.5);
        let outer = allowed.iter().find(|l| signed_area(l) > 0.0).unwrap();
        let inner = allowed.iter().find(|l| signed_area(l) < 0.0).unwrap();
        let (path, rad, _) = single_pass(outer, inner, &allowed, 1.5).unwrap();
        // along the straight sides: centred in the band, radius half its width
        let mut sides = 0;
        for (c, r) in path.iter().zip(&rad) {
            let on_side = (c[0] > 8.0 && c[0] < 32.0) || (c[1] > 8.0 && c[1] < 32.0);
            if on_side {
                let d = (c[0] - 1.5).min(38.5 - c[0]).min(c[1] - 1.5).min(38.5 - c[1]);
                assert!((d - 1.0).abs() < 0.01 && (r - 1.0).abs() < 0.01, "centre {c:?} {d} from the edge, radius {r}");
                sides += 1;
            }
            // never past where the tool centre may go
            assert!(boundary_dist(&allowed, *c) >= *r - 1e-6);
        }
        assert!(sides > 100);
    }

    #[test]
    fn a_narrow_band_is_cleared_in_one_pass_per_level() {
        let s = Settings { safe_z: 10.0, ..Default::default() };
        let code = generate(&band_job(), &s).unwrap();
        let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), [-2.0, -2.0, -5.0], [42.0, 42.0, 0.0], 300);
        while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
        assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
        let m = &sim.map;
        let r = 1.5;
        let mut left = 0;
        for j in 0..m.ny {
            for i in 0..m.nx {
                let [x, y] = m.centre(i, j);
                let z = m.z[j * m.nx + i];
                let in_island = x > 5.05 && x < 34.95 && y > 5.05 && y < 34.95;
                let outside = x < -0.05 || x > 40.05 || y < -0.05 || y > 40.05;
                if in_island || outside {
                    assert!(z > -1e-3, "wall cut at ({x}, {y})");
                    continue;
                }
                // in the band and within a radius of where the tool centre may go
                let (dx0, dy0) = ((r - x).max(x - (40.0 - r)).max(0.0), (r - y).max(y - (40.0 - r)).max(0.0));
                let near_outer = dx0.hypot(dy0) < r - 0.1;
                let to_island = {
                    let dx = (5.0 - r - x).max(x - (35.0 + r)).max(0.0);
                    let dy = (5.0 - r - y).max(y - (35.0 + r)).max(0.0);
                    dx.hypot(dy)
                };
                let in_band = x > 0.1 && x < 39.9 && y > 0.1 && y < 39.9 && !(x > 4.9 && x < 35.1 && y > 4.9 && y < 35.1);
                if in_band && near_outer && to_island > 0.0 && z > -2.0 + 1e-3 {
                    left += 1;
                }
            }
        }
        assert_eq!(left, 0, "cells left in the band");
    }
}

#[cfg(test)]
mod omit_tests {
    use super::*;
    use crate::toolpath;

    // pocket_block: 60 x 40 block, top Z0, a 30 x 20 pocket (X15..45 Y10..30)
    // to Z-4 and a Ø8 hole at X52 Y32 to Z-3
    fn block() -> (crate::step::Model, usize, usize, usize) {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let floor = m.faces.iter().position(|f| f.is_up_horizontal() && (f.z() + 4.0).abs() < 1e-6).unwrap();
        let wall = m.faces.iter().position(|f| f.cyl.is_some()).unwrap();
        let pocket_wall = m.faces.iter().position(|f| f.planar && f.normal[2].abs() < 1e-6 && f.loops[0].pts.iter().all(|p| p[0] >= 15.0 - 1e-6 && p[0] <= 45.0 + 1e-6 && p[1] >= 10.0 - 1e-6 && p[1] <= 30.0 + 1e-6)).unwrap();
        (m, floor, wall, pocket_wall)
    }

    #[test]
    fn the_fill_is_the_cavity_up_to_its_rim() {
        let (m, floor, wall, pocket_wall) = block();
        for fi in [floor, pocket_wall] {
            let f = omit_fill(&m, fi).unwrap();
            assert!((f.top - 0.0).abs() < 1e-6 && (f.bottom + 4.0).abs() < 1e-6, "{} {}", f.top, f.bottom);
            let area: f64 = f.outer.iter().map(|l| signed_area(l)).sum();
            assert!((area - 600.0).abs() < 1.0, "pocket area {area}");
            assert_eq!(f.faces.len(), 5, "floor and four walls");
        }
        let h = omit_fill(&m, wall).unwrap();
        let area: f64 = h.outer.iter().map(|l| signed_area(l)).sum();
        assert!((h.top).abs() < 1e-6 && (h.bottom + 3.0).abs() < 1e-6 && (area - PI * 16.0).abs() < 1.0, "hole {area} {} {}", h.top, h.bottom);
        assert!(h.covers([52.0, 32.0]) && !h.covers([30.0, 20.0]));
        // the top has nothing round it to fill up to
        let top = m.faces.iter().position(|f| f.is_up_horizontal() && f.z().abs() < 1e-6).unwrap();
        assert!(omit_fill(&m, top).is_err());
    }

    #[test]
    fn ops_after_an_omit_leave_the_area_uncut() {
        let (m, floor, wall, _) = block();
        let stock = Stock { x: 4.0, y: 4.0, z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        let rough = RoughOp {
            cutter: Cutter::C3x17,
            start_z: stock.1[2],
            bottom_z: m.min[2],
            faces: rough_faces(&m),
            stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
            ..Default::default()
        };
        let pocket = PocketOp { face_id: floor, z: -4.0, start_z: 0.0, outer: rect(15.0, 10.0, 45.0, 30.0), cutter: Cutter::C3x17, ..Default::default() };
        let omit = OmitOp { fills: vec![omit_fill(&m, floor).unwrap(), omit_fill(&m, wall).unwrap()] };
        let op = |k: OpKind| StepOp { enabled: true, kind: k };
        let s = Settings { safe_z: 25.0, ..Default::default() };
        // feed moves below the top whose tool would reach into the pocket or the hole
        let into = |code: &str| {
            toolpath::parse(code)
                .paths
                .iter()
                .filter(|p| p.kind != toolpath::MoveKind::Rapid)
                .flat_map(|p| p.pts.iter())
                .filter(|q| {
                    let (x, y) = (q[0] as f64, q[1] as f64);
                    let r = 1.5 - 0.01;
                    (q[2] as f64) < -1e-3 && ((x > 15.0 - r && x < 45.0 + r && y > 10.0 - r && y < 30.0 + r) || (x - 52.0).hypot(y - 32.0) < 4.0 + r)
                })
                .count()
        };
        let cut = generate(&StepJob { ops: vec![op(OpKind::Rough(rough.clone())), op(OpKind::Pocket(pocket.clone()))], ..Default::default() }, &s).unwrap();
        assert!(into(&cut) > 100, "without the omit both are cut");
        let job = StepJob { ops: vec![op(OpKind::Omit(omit.clone())), op(OpKind::Rough(rough.clone())), op(OpKind::Pocket(pocket.clone()))], ..Default::default() };
        let code = generate(&job, &s).unwrap();
        assert_eq!(into(&code), 0, "the omitted areas are cut");
        // roughing still clears the band round the outside
        assert!(code.lines().count() > 200);
        // ops before the omit are not affected
        let job = StepJob { ops: vec![op(OpKind::Rough(rough)), op(OpKind::Omit(omit)), op(OpKind::Pocket(pocket))], ..Default::default() };
        assert!(into(&generate(&job, &s).unwrap()) > 100);
        // and it simulates clean
        let mut sim = crate::sim::Sim::new(crate::sim::Program::parse(&code, &s), stock.0, stock.1, 250);
        while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
        assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
    }

    #[test]
    fn moves_into_an_omitted_area_go_over_it() {
        let f = Fill { outer: vec![rect(0.0, 0.0, 10.0, 10.0)], top: 2.0, bottom: -5.0, ..Default::default() };
        // a pass at Z-1 straight across it
        let moves = vec![Mv::Feed([-10.0, 5.0, -1.0], 100.0), Mv::Feed([20.0, 5.0, -1.0], 100.0)];
        let out = apply_fill(&moves, [-10.0, 5.0, -1.0], &f, 1.0);
        let pts: Vec<P3> = out.iter().map(|m| match m { Mv::Feed(p, _) | Mv::Rapid(p) => *p }).collect();
        // up just before the tool would touch it, over at its top, down after
        assert!(pts.iter().any(|p| (p[0] + 1.0).abs() < 0.01 && (p[2] - 2.0).abs() < 1e-9), "{pts:?}");
        assert!(pts.iter().any(|p| (p[0] - 11.0).abs() < 0.01 && (p[2] - 2.0).abs() < 1e-9), "{pts:?}");
        assert!(pts.iter().all(|p| !(p[0] > -1.0 + 0.01 && p[0] < 11.0 - 0.01) || p[2] >= 2.0 - 1e-9), "{pts:?}");
        assert_eq!(*pts.last().unwrap(), [20.0, 5.0, -1.0]);
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<P2> {
        vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    }
}

#[cfg(test)]
mod facing_pattern_tests {
    use super::*;

    fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1e-18 { 0.0 } else { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) };
        d2(p, [a[0] + dx * t, a[1] + dy * t])
    }

    #[test]
    fn every_pattern_covers_the_area_and_stays_in_it() {
        // an L-shaped outline with a notch, faced down 1 mm in one pass
        let outline = vec![vec![[0.0, 0.0], [40.0, 0.0], [40.0, 12.0], [18.0, 12.0], [18.0, 30.0], [0.0, 30.0]]];
        let cutter = Cutter::C3x17;
        let r = cutter.dia() / 2.0;
        let centres = grow_loops(&outline, r + OPEN_EDGE);
        for pattern in FacingPattern::ALL {
            for stepover in [40.0, 60.0, 90.0] {
                let c = FacingCut { cutter, stepover_pct: stepover, start_z: 1.0, z: 0.0, depth_per_pass: 2.0, feed: 500.0, edge_lap: false, pattern };
                let moves = facing_moves(&centres, &outline, c, 10.0);
                // feed segments at depth
                let mut pos = [0.0, 0.0, 10.0];
                let mut segs = vec![];
                let mut lifts = 0;
                for m in &moves {
                    let p = match m {
                        Mv::Feed(p, _) | Mv::Rapid(p) => *p,
                    };
                    if let Mv::Feed(..) = m {
                        if pos[2] < 1e-6 && p[2] < 1e-6 {
                            segs.push(([pos[0], pos[1]], [p[0], p[1]]));
                            // at depth the centre stays in the centre area
                            let mid = [(pos[0] + p[0]) / 2.0, (pos[1] + p[1]) / 2.0];
                            assert!(inside(&centres, mid) || boundary_dist(&centres, mid) < 1e-3, "{pattern:?} {stepover}%: outside at {mid:?}");
                        }
                    }
                    if p[2] > pos[2] + 1e-6 {
                        lifts += 1;
                    }
                    pos = p;
                }
                // every point of the outline area is within a tool radius of a cut
                let mut worst: f64 = 0.0;
                let mut worst_at = [0.0, 0.0];
                for i in 0..=80 {
                    for j in 0..=60 {
                        let p = [i as f64 * 0.5, j as f64 * 0.5];
                        if !inside(&outline, p) && boundary_dist(&outline, p) > 1e-9 {
                            continue;
                        }
                        let dmin = segs.iter().map(|(a, b)| seg_dist(p, *a, *b)).fold(f64::INFINITY, f64::min);
                        if dmin > worst {
                            worst = dmin;
                            worst_at = p;
                        }
                    }
                }
                assert!(worst <= r + 1e-6, "{pattern:?} {stepover}%: {worst_at:?} is {worst} from the nearest cut");
                if pattern != FacingPattern::Unidirectional {
                    // the others step over at depth: far fewer lifts than rows
                    // (a square spiral's rectangles are cut in two by the L)
                    let rows = (30.0 + 2.0 * r) / (stepover / 100.0 * cutter.dia());
                    let most = if pattern == FacingPattern::SquareSpiral { rows } else { rows / 2.0 };
                    assert!((lifts as f64) < most, "{pattern:?} {stepover}%: {lifts} lifts");
                }
            }
        }
    }

    #[test]
    fn spirals_climb_with_the_middle_inside() {
        let sq = vec![vec![[0.0, 0.0], [30.0, 0.0], [30.0, 20.0], [0.0, 20.0]]];
        // (the material a tool radius in from where the centres may go)
        let mat = vec![vec![[1.5, 1.5], [28.5, 1.5], [28.5, 18.5], [1.5, 18.5]]];
        for runs in [spiral_runs(&sq, &mat, 3.0, 1.5), square_spiral_runs(&sq, 3.0)] {
            // the first (outer) loop runs clockwise
            assert!(signed_area(&runs[0]) < 0.0, "{:?}", &runs[0][..3]);
            // and they start from the outside
            let inset = |l: &Vec<P2>| l.iter().map(|p| p[0].min(p[1]).min(30.0 - p[0]).min(20.0 - p[1])).fold(f64::INFINITY, f64::min);
            assert!(runs.iter().all(|l| inset(l) >= inset(&runs[0]) - 1e-9) && inset(&runs[1]) > inset(&runs[0]));
        }
    }
}
