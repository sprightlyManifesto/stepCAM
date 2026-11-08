//! Step milling: Face and Pocket operations on faces picked from a STEP model,
//! plus mill tabs that the cutter lifts over.
//!
//! * Climb milling throughout (M3 spindle): pocket walls are cut CCW, islands
//!   CW, facing rows run +X while stepping towards -Y.
//! * Pockets are roughed leaving `finish` on all walls, then a finishing pass
//!   at each depth level cuts the walls to size.
//! * Adaptive strategy = trochoidal loops along offset centre paths.

use crate::gcode::fmt;
use crate::model::{Cutter, Settings};
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
    /// Part extents in XY (min x, min y, max x, max y).
    pub region: [f64; 4],
    pub cutter: Cutter,
    pub stepover_pct: f64,
    pub depth_per_pass: f64,
    /// How far the cutter edge runs past the part (mm).
    pub overlap: f64,
    pub feed: f64,
    pub speed: f64,
    pub start_z: f64,
}

impl Default for FaceOp {
    fn default() -> Self {
        FaceOp {
            face_id: 0,
            z: 0.0,
            region: [0.0; 4],
            cutter: Cutter::C3_175x17,
            stepover_pct: 50.0,
            depth_per_pass: 1.0,
            overlap: 2.0,
            feed: 500.0,
            speed: 12000.0,
            start_z: 0.0,
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
    /// Model faces seen from above - kept in sync with the model.
    pub faces: Vec<RoughFace>,
}

impl Default for RoughOp {
    fn default() -> Self {
        RoughOp {
            cutter: Cutter::C3_175x17,
            layer_height: 1.0,
            stepover_pct: 40.0,
            allowance: 0.2,
            feed: 500.0,
            speed: 12000.0,
            ramp_angle: 3.0,
            entry: Entry::Helix,
            strategy: Strategy::Standard,
            start_z: 0.0,
            bottom_z: -1.0,
            overcut_z: 0.0,
            clear_voids: true,
            faces: vec![],
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
/// up count at their Z, sloped / curved ones at their highest point
/// (conservative); vertical and down-facing planar faces are skipped.
pub fn rough_faces(model: &crate::step::Model) -> Vec<RoughFace> {
    let xy = |l: &crate::step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<P2>>();
    let mut out = vec![];
    for f in &model.faces {
        if f.planar && f.normal[2] < 1e-6 {
            continue; // vertical or facing down
        }
        let Some(outer) = f.loops.iter().find(|l| l.outer).or(f.loops.first()) else { continue };
        let o = xy(outer);
        if o.len() < 3 || signed_area(&o).abs() < 1e-6 {
            continue; // seen edge-on
        }
        let z = if f.is_up_horizontal() {
            f.z()
        } else {
            f.loops.iter().flat_map(|l| l.pts.iter()).map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max)
        };
        let holes = f.loops.iter().filter(|l| !std::ptr::eq(*l, outer)).map(xy).filter(|h| h.len() >= 3).collect();
        out.push(RoughFace { z, outer: o, holes });
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
                if p.edges.is_empty() { "" } else { " (edges)" },
                fmt(p.z),
                match p.strategy {
                    Strategy::Standard => "standard",
                    Strategy::Adaptive => "adaptive",
                }
            ),
            OpKind::Tab(t) => format!("Mill tab at X{} Y{} Z{}", fmt(t.center[0]), fmt(t.center[1]), fmt(t.center[2])),
            OpKind::Profile(p) => format!("Profile {} to Z{}", p.side.name(), fmt(p.z)),
            OpKind::Rough(r) => format!("Rough to Z{} ({} mm layers)", fmt(r.final_z()), fmt(r.layer_height)),
        }
    }
    /// Surface the op was made from (None for edge-based ops).
    pub fn face_id(&self) -> Option<usize> {
        match &self.kind {
            OpKind::Face(f) => Some(f.face_id),
            OpKind::Pocket(p) if p.edges.is_empty() => Some(p.face_id),
            _ => None,
        }
    }
    pub fn cutter_mut(&mut self) -> Option<&mut Cutter> {
        match &mut self.kind {
            OpKind::Face(f) => Some(&mut f.cutter),
            OpKind::Pocket(p) => Some(&mut p.cutter),
            OpKind::Profile(p) => Some(&mut p.cutter),
            OpKind::Rough(r) => Some(&mut r.cutter),
            OpKind::Tab(_) => None,
        }
    }
    /// Edges the op was made from (empty for surface-based ops).
    pub fn edges(&self) -> &[usize] {
        match &self.kind {
            OpKind::Pocket(p) => &p.edges,
            OpKind::Tab(t) => &t.edges,
            OpKind::Profile(p) => &p.edges,
            OpKind::Face(_) | OpKind::Rough(_) => &[],
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
    pub ops: Vec<StepOp>,
}

impl Default for StepJob {
    fn default() -> Self {
        StepJob { step_path: String::new(), placement: Default::default(), stock: Default::default(), rapid_above: 10.0, ops: vec![] }
    }
}

/// Raw material block. With `offset` the X/Y/Z values are added to the part's
/// bounding box (X/Y split evenly on both sides, Z all on top); without it
/// they are the absolute stock size (centred on the part in XY, bottom level
/// with the part bottom).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Stock {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub offset: bool,
}

impl Default for Stock {
    fn default() -> Self {
        Stock { x: 0.0, y: 0.0, z: 0.0, offset: true }
    }
}

impl Stock {
    /// Stock box (min, max) around a part with bounding box `min`..`max`.
    pub fn bounds(&self, min: P3, max: P3) -> (P3, P3) {
        let size = |k: usize, v: f64| if self.offset { max[k] - min[k] + v } else { v };
        let (sx, sy, sz) = (size(0, self.x), size(1, self.y), size(2, self.z));
        let c = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0];
        ([c[0] - sx / 2.0, c[1] - sy / 2.0, min[2]], [c[0] + sx / 2.0, c[1] + sy / 2.0, min[2] + sz])
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
    let lines = pl.arcs_to_approx_lines(0.005).unwrap_or_else(|| pl.clone());
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
    let off = shape.parallel_offset(d, &ShapeOffsetOptions::new());
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
                    if z - dz <= z_to {
                        // reach depth part way along this pass
                        let frac = (z - z_to) / dz;
                        let s_end = s + (s_next - s) * frac;
                        let p = point_at(path, s_end);
                        out.feed([p[0], p[1], z_to], op.feed);
                        s = s_end;
                        z = z_to;
                    } else {
                        // follow the path between s and s_next with intermediate vertices
                        let steps = ((len / (op.cutter.dia() * 0.5).max(0.2)).ceil() as usize).max(1);
                        for k in 1..=steps {
                            let ss = s + (s_next - s) * k as f64 / steps as f64;
                            let p = point_at(path, ss);
                            out.feed([p[0], p[1], z - dz * k as f64 / steps as f64], op.feed);
                        }
                        z -= dz;
                        s = s_next;
                        forward = !forward;
                    }
                }
                // back to the path start at depth
                let steps = ((s / (op.cutter.dia() * 0.5).max(0.2)).ceil() as usize).max(1);
                for k in 1..=steps {
                    let p = point_at(path, s * (1.0 - k as f64 / steps as f64));
                    out.feed([p[0], p[1], z_to], op.feed);
                }
                return;
            }
        }
        // plunge
        out.feed([p0[0], p0[1], z_to], (op.feed * 0.3).max(1.0));
    }
}

fn pocket_moves(op: &PocketOp, retract: f64) -> Result<Vec<Mv>, String> {
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
    let mut centre: Vec<(usize, Vec<P2>)> = Vec::new();
    let mut troch_r = 0.0;
    if op.strategy == Strategy::Adaptive {
        troch_r = 0.4 * d;
        let spacing = 1.4 * d;
        let mut k = 0;
        loop {
            let ls = offset_region(&op.outer, &islands, rough0 + troch_r + k as f64 * spacing);
            if ls.is_empty() || k > 10000 {
                break;
            }
            centre.extend(ls.into_iter().map(|l| (k, l)));
            k += 1;
        }
        if centre.is_empty() {
            troch_r = 0.0; // too narrow for trochoids: fall back to standard loops
        }
    }
    if centre.is_empty() {
        let mut k = 0;
        loop {
            let ls = if k == 0 { allowed.clone() } else { offset_region(&op.outer, &islands, rough0 + k as f64 * step) };
            if ls.is_empty() || k > 10000 {
                break;
            }
            centre.extend(ls.into_iter().map(|l| (k, l)));
            k += 1;
        }
    }

    let ctx = PocketCtx { op, allowed: allowed.clone(), r };
    let mut out = Path::new([0.0, 0.0, retract]);
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
            let loop_c = start_near(&centre[ci].1, here);
            let path: Vec<P2> = if troch_r > 0.0 { trochoid(&loop_c, troch_r, step) } else { loop_c };
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

    // finishing pass on all walls, each level top-down
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
                out.reposition(retract, lead, z + 0.5);
                out.feed([lead[0], lead[1], z], op.feed);
            }
            for q in &lp {
                out.feed([q[0], q[1], z], op.feed);
            }
            out.feed([lead[0], lead[1], z], op.feed);
        }
    }
    out.rapid([out.pos[0], out.pos[1], retract]);
    Ok(out.moves)
}

fn face_moves(op: &FaceOp, retract: f64) -> Vec<Mv> {
    let d = op.cutter.dia();
    let r = d / 2.0;
    let [minx, miny, maxx, maxy] = op.region;
    let ov = op.overlap.max(0.0);
    let (x0, x1) = (minx - ov - r, maxx + ov + r);
    let y_top = maxy + ov - r;
    let y_bot = miny - ov + r;
    let step = (op.stepover_pct / 100.0 * d).max(0.01);
    let mut rows = vec![];
    let mut y = y_top;
    while y > y_bot + 1e-9 {
        rows.push(y);
        y -= step;
    }
    rows.push(y_bot.min(y_top));
    let mut out = Path::new([x0, rows[0], retract]);
    let mut z_prev = op.start_z;
    let mut first = true;
    for z in levels(op.start_z, op.z, op.depth_per_pass) {
        for &y in &rows {
            let lift = z_prev + 0.5;
            if first {
                out.reposition(retract, [x0, y], lift);
                first = false;
            } else {
                out.rapid([out.pos[0], out.pos[1], lift.max(out.pos[2])]);
                out.rapid([x0, y, lift]);
            }
            out.feed([x0, y, z], (op.feed * 0.5).max(1.0));
            out.feed([x1, y, z], op.feed); // +X with material towards -Y = climb
            out.feed([x1, y, lift], op.feed);
        }
        z_prev = z;
    }
    out.rapid([out.pos[0], out.pos[1], retract]);
    out.moves
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
    for mut seq in paths {
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
        let z = f.z + op.allowance.max(0.0);
        if z < op.start_z - 1e-6 && z > bottom + 1e-6 {
            zs.push(z);
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

/// One layer of roughing: areas to clear inside the part (outer, islands)
/// and the exterior contours of the material to outline.
pub struct RoughLayer {
    pub regions: Vec<(Vec<P2>, Vec<Vec<P2>>)>,
    pub contours: Vec<Vec<P2>>,
    /// Holes that go right through the part (no face above them at any Z).
    pub through: Vec<Vec<P2>>,
    /// `regions` less the through holes: only areas with a floor (a face
    /// below the layer) - cleared even without "clear voids".
    pub floored: Vec<(Vec<P2>, Vec<Vec<P2>>)>,
}

/// Work out layer `z`. The part outline (all faces seen from above, holes
/// filled) bounds the clearing; material is every face above the layer
/// (less the allowance). Outside the outline nothing is cleared - the
/// exterior contours of the material are only outlined.
pub fn rough_layer(op: &RoughOp, z: f64) -> Result<RoughLayer, String> {
    use geo::BooleanOps;
    let a = op.allowance.max(0.0);
    let g = op.cutter.dia() / 2.0 + a + OPEN_EDGE;
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let outline = union_all(op.faces.iter().map(|f| to_geo(&f.outer, &[])));
        let grown = union_all(
            outline
                .0
                .iter()
                .flat_map(|p| offset_region(&from_geo_ring(p.exterior()), &[], -g))
                .filter(|l| signed_area(l) > 0.0)
                .map(|l| to_geo(&l, &[])),
        );
        let keep = union_all(op.faces.iter().filter(|f| f.z > z - a + 1e-6).map(|f| to_geo(&f.outer, &f.holes)));
        let cut = grown.difference(&keep);
        let all = union_all(op.faces.iter().map(|f| to_geo(&f.outer, &f.holes)));
        let through: Vec<Vec<P2>> =
            all.0.iter().flat_map(|p| p.interiors().iter().map(from_geo_ring)).filter(|h| h.len() >= 3).collect();
        // Through holes shrunk by twice the allowance (+ a hair) so the floored
        // clearing meets the through-hole profile without leaving a thin wall.
        let voids = union_all(
            through
                .iter()
                .flat_map(|h| offset_region(h, &[], 2.0 * a + OPEN_EDGE))
                .filter(|l| signed_area(l) > 0.0)
                .map(|l| to_geo(&l, &[])),
        );
        let floored = cut.difference(&voids);
        (cut, floored, keep, through)
    }))
    .map_err(|_| format!("could not work out the part outline at Z{}", fmt(z)))?;
    let (cut, floored, keep, through) = res;
    let regions_of = |m: &geo::MultiPolygon<f64>| -> Vec<(Vec<P2>, Vec<Vec<P2>>)> {
        m.0.iter()
            .map(|p| (from_geo_ring(p.exterior()), p.interiors().iter().map(from_geo_ring).collect::<Vec<_>>()))
            .filter(|(o, _)| o.len() >= 3)
            .collect()
    };
    let contours = keep.0.iter().map(|p| from_geo_ring(p.exterior())).filter(|o| o.len() >= 3).collect();
    Ok(RoughLayer { regions: regions_of(&cut), floored: regions_of(&floored), contours, through })
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

fn rough_moves(op: &RoughOp, retract: f64) -> Result<Vec<Mv>, String> {
    let r = op.cutter.dia() / 2.0;
    let a = op.allowance.max(0.0);
    let tan = op.ramp_angle.to_radians().tan().max(1e-3);
    let mut out = vec![];
    let mut z_prev = op.start_z;
    for z in rough_levels(op) {
        let layer = rough_layer(op, z)?;
        // clear inside the part; without clear voids, areas over a through
        // hole are left and the hole is only profiled
        let regions = if op.clear_voids { layer.regions } else { layer.floored };
        if !op.clear_voids {
            for h in &layer.through {
                for l in offset_region(h, &[], r + a).into_iter().filter(|l| signed_area(l) > 0.0) {
                    // CCW inside the hole = climb
                    out.extend(ramp_loop(&start_near(&l, l[0]), z_prev, z, op.feed, tan, retract));
                }
            }
        }
        for (outer, islands) in regions {
            let pocket = PocketOp {
                z,
                start_z: z_prev,
                depth_per_pass: z_prev - z + 1.0,
                outer,
                islands,
                keep_islands: true,
                finish: a,
                finish_pass: false,
                cutter: op.cutter,
                stepover_pct: op.stepover_pct,
                feed: op.feed,
                speed: op.speed,
                ramp_angle: op.ramp_angle,
                strategy: op.strategy,
                entry: op.entry,
                ..Default::default()
            };
            // areas too narrow for the cutter (e.g. just the open edge) are left
            if let Ok(m) = pocket_moves(&pocket, retract) {
                out.extend(m);
            }
        }
        // outline every exterior contour: CW (climb), ramping down along it
        for c in &layer.contours {
            for l in offset_region(c, &[], -(r + a)).into_iter().filter(|l| signed_area(l) > 0.0) {
                let mut seq = start_near(&l, l[0]);
                seq.reverse();
                out.extend(ramp_loop(&seq, z_prev, z, op.feed, tan, retract));
            }
        }
        z_prev = z;
    }
    if out.is_empty() {
        return Err("roughing found nothing to cut".into());
    }
    Ok(out)
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

// ------------------------------------------------------------------ validation & output
pub fn validate(op: &StepOp, s: &Settings, tabs_top: f64) -> Vec<String> {
    let mut e = vec![];
    let (start, floor, cutter, dpp, feed, speed) = match &op.kind {
        OpKind::Face(f) => (f.start_z, f.z, f.cutter, f.depth_per_pass, f.feed, f.speed),
        OpKind::Pocket(p) => (p.start_z, p.z, p.cutter, p.depth_per_pass, p.feed, p.speed),
        OpKind::Profile(p) => (p.start_z, p.z, p.cutter, p.depth_per_pass, p.feed, p.speed),
        OpKind::Rough(r) => (r.start_z, r.final_z(), r.cutter, r.layer_height, r.feed, r.speed),
        OpKind::Tab(t) => {
            if !(t.width > 0.0 && t.depth > 0.0 && t.z_height > 0.0) {
                e.push("tab width, depth and Z height must be > 0".into());
            }
            return e;
        }
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
    let mut errors = vec![];
    for (i, op) in job.ops.iter().enumerate().filter(|(_, o)| o.enabled) {
        for e in validate(op, s, tabs_top) {
            errors.push(format!("op {}: {}", i + 1, e));
        }
    }
    let cutting: Vec<(usize, &StepOp)> = job.ops.iter().enumerate().filter(|(_, o)| o.enabled && !matches!(o.kind, OpKind::Tab(_))).collect();
    if cutting.is_empty() && errors.is_empty() {
        errors.push("no Face, Pocket, Profile or Rough operations - right-click a flat surface or an edge in the 3D view".into());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let mut out = String::new();
    let mut line = |l: String| {
        out.push_str(&l.replace('Ø', "D"));
        out.push('\n');
    };
    line("%".into());
    line("(stepCAM - step milling)".into());
    if !job.step_path.is_empty() {
        let name = std::path::Path::new(&job.step_path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        line(format!("(model {})", name.replace(['(', ')'], "")));
    }
    for t in &tabs {
        line(format!("(mill tab X{} Y{} Z{} to Z{} {}x{})", fmt(t.center[0]), fmt(t.center[1]), fmt(t.center[2]), fmt(t.top()), fmt(t.width), fmt(t.depth)));
    }
    line("G21 G90 G17 G94 G40 G49 G80".into());
    line(format!("G0 Z{}", fmt(s.safe_z)));

    let mut tool: Option<u32> = None;
    let mut rpm: f64 = -1.0;
    for (i, op) in cutting {
        let (cutter, speed, start_z) = match &op.kind {
            OpKind::Face(f) => (f.cutter, f.speed, f.start_z),
            OpKind::Pocket(p) => (p.cutter, p.speed, p.start_z),
            OpKind::Profile(p) => (p.cutter, p.speed, p.start_z),
            OpKind::Rough(r) => (r.cutter, r.speed, r.start_z),
            OpKind::Tab(_) => unreachable!(),
        };
        let retract = start_z.max(tabs_top) + s.clearance;
        let mut moves = match &op.kind {
            OpKind::Face(f) => face_moves(f, retract),
            OpKind::Pocket(p) => match pocket_moves(p, retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Rough(r) => match rough_moves(r, retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Profile(p) => match profile_moves(p, retract) {
                Ok(m) => m,
                Err(e) => return Err(vec![format!("op {}: {e}", i + 1)]),
            },
            OpKind::Tab(_) => unreachable!(),
        };
        let start = match moves.first() {
            Some(Mv::Rapid(p)) | Some(Mv::Feed(p, _)) => [p[0], p[1], s.safe_z],
            None => continue,
        };
        for t in &tabs {
            moves = apply_tab(&moves, start, t, cutter.dia() / 2.0);
        }

        let t = cutter.tool_number();
        line(String::new());
        if tool != Some(t) {
            if tool.is_some() {
                line("M5".into());
                line("M9".into());
            }
            line(format!("G0 Z{}", fmt(s.safe_z)));
            line(format!("T{t} M6 (endmill Ø{} flute {})", fmt(cutter.dia()), fmt(cutter.flute_len())));
            if s.tool_length_comp {
                line(format!("G43 H{t}"));
            }
            line(format!("S{} M3", speed.round() as i64));
            if s.coolant {
                line("M8".into());
            }
            tool = Some(t);
            rpm = speed;
        } else if (rpm - speed).abs() > 0.5 {
            line(format!("S{} M3", speed.round() as i64));
            rpm = speed;
        }
        line(format!("(op {}: {})", i + 1, op.label()));
        line(format!("G0 Z{}", fmt(s.safe_z)));

        let mut pos = start;
        let mut last_f = -1.0;
        for m in moves {
            match m {
                Mv::Rapid(p) => {
                    let xy = (p[0] - pos[0]).abs() > 1e-9 || (p[1] - pos[1]).abs() > 1e-9;
                    let zc = (p[2] - pos[2]).abs() > 1e-9;
                    if zc && p[2] > pos[2] {
                        line(format!("G0 Z{}", fmt(p[2])));
                    }
                    if xy {
                        line(format!("G0 X{} Y{}", fmt(p[0]), fmt(p[1])));
                    }
                    if zc && p[2] < pos[2] {
                        line(format!("G0 Z{}", fmt(p[2])));
                    }
                    pos = p;
                }
                Mv::Feed(p, f) => {
                    let mut w = String::from("G1");
                    for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
                        if (p[k] - pos[k]).abs() > 1e-9 {
                            w.push_str(&format!(" {a}{}", fmt(p[k])));
                        }
                    }
                    if w == "G1" {
                        continue;
                    }
                    if (f - last_f).abs() > 1e-9 {
                        w.push_str(&format!(" F{}", fmt(f)));
                        last_f = f;
                    }
                    line(w);
                    pos = p;
                }
            }
        }
        line(format!("G0 Z{}", fmt(s.safe_z)));
    }
    line(String::new());
    line("M5".into());
    line("M9".into());
    line(format!("G0 Z{}", fmt(s.safe_z)));
    line("M30".into());
    line("%".into());
    Ok(out)
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
        let moves = pocket_moves(&PocketOp { finish: 0.5, ..op }, 2.0).unwrap();
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
    fn facing_covers_part_with_overlap() {
        let job = StepJob {
            step_path: String::new(),
            placement: Default::default(),
            stock: Default::default(),
            rapid_above: 10.0,
            ops: vec![StepOp {
                enabled: true,
                kind: OpKind::Face(FaceOp { z: -1.5, region: [0.0, 0.0, 60.0, 40.0], cutter: Cutter::C3x17, ..Default::default() }),
            }],
        };
        let code = generate(&job, &Settings::default()).unwrap();
        let pts = feed_points(&code);
        let at_floor: Vec<_> = pts.iter().filter(|p| (p[2] + 1.5).abs() < 1e-4).collect();
        assert!(!at_floor.is_empty());
        let minx = at_floor.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min);
        let maxy = at_floor.iter().map(|p| p[1]).fold(f32::NEG_INFINITY, f32::max);
        let miny = at_floor.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
        assert!((minx - (0.0 - 2.0 - 1.5)).abs() < 1e-3);
        assert!((maxy - (40.0 + 2.0 - 1.5)).abs() < 1e-3);
        assert!((miny - (0.0 - 2.0 + 1.5)).abs() < 1e-3);
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
        let job = StepJob { step_path: "testdata/pocket_block.step".into(), placement: Default::default(), stock: Default::default(), rapid_above: 10.0, ops };
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
    fn roughing_clears_inside_and_profiles_outside() {
        let m = crate::step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap();
        let stock = Stock { x: 4.0, y: 4.0, z: 1.0, offset: true };
        let (_, smax) = stock.bounds(m.min, m.max);
        let op = RoughOp {
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
                    // outside the part only the profile: tool radius + allowance off the wall
                    let d = sd(*p, 0.0, 0.0, 60.0, 40.0);
                    assert!(d < 1.7 + 0.01, "cleared stock outside the part at {p:?} ({d})");
                    outside |= (p[2] + 10.0).abs() < 1e-6;
                }
            }
            // nothing is cleared beyond the part outline (the open edge allowance)
            assert!(sd(*p, 0.0, 0.0, 60.0, 40.0) < 1.7 + 0.01, "outside the outline at {p:?}");
            // never below the part bottom, never outside the grown stock
            assert!(p[2] >= -10.0 - 1e-6);
        }
        assert!(pocket && hole && outside, "pocket {pocket} hole {hole} outline {outside}");
        // the top layer faces the 1 mm of stock over the part
        assert!(pts.iter().any(|p| (p[2] - 0.2).abs() < 1e-6 && sd(*p, 0.0, 0.0, 60.0, 40.0) < -5.0));
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
            faces: vec![RoughFace { z: 0.0, outer: sq(10.0, 30.0), holes: vec![] }, RoughFace { z: -5.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 30.0)] }],
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
                // below the flange top: only the flange profile
                assert!((sd(*p, 0.0, 40.0) - 1.7).abs() < 0.01, "flange gouge / extra cut at {p:?}");
            }
            if (p[2] - flange_top).abs() < 1e-6 && sd(*p, 0.0, 40.0) > -0.01 {
                reached_edge = true;
            }
            assert!(sd(*p, 0.0, 40.0) < 1.7 + 0.01, "cleared outside the part at {p:?}");
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
            faces: vec![RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![] }],
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
        let op = RoughOp {
            cutter: Cutter::C3x17,
            start_z: 0.0,
            bottom_z: -10.0,
            layer_height: 5.0,
            clear_voids: false,
            faces: vec![
                RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 20.0), rect(25.0, 5.0, 35.0, 15.0)] },
                RoughFace { z: -4.0, outer: rect(25.0, 5.0, 35.0, 15.0), holes: vec![] },
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
        let all = pts(&RoughOp { clear_voids: true, ..op });
        assert!(all.iter().any(|p| sd(*p, 25.0, 5.0, 35.0, 15.0) < -1.7 && (p[2] + 3.8).abs() < 1e-9));
        assert!(all.iter().any(|p| sd(*p, 10.0, 10.0, 20.0, 20.0) < -2.0 && p[2] < -1.0));
    }

    #[test]
    fn roughing_without_clear_voids_clears_counterbore_around_through_hole() {
        let sq = |a: f64, b: f64| vec![[a, a], [b, a], [b, b], [a, b]];
        // counterbore 10..30 to Z-3, through hole 16..24 below it
        let op = RoughOp {
            cutter: Cutter::C3x17,
            start_z: 0.0,
            bottom_z: -10.0,
            layer_height: 5.0,
            clear_voids: false,
            faces: vec![
                RoughFace { z: 0.0, outer: sq(0.0, 40.0), holes: vec![sq(10.0, 30.0)] },
                RoughFace { z: -3.0, outer: sq(10.0, 30.0), holes: vec![sq(16.0, 24.0)] },
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
        let s = Stock { x: 4.0, y: 6.0, z: 2.0, offset: true };
        assert_eq!(s.bounds(min, max), ([-2.0, -3.0, -10.0], [62.0, 43.0, 2.0]));
        assert!(s.issues(min, max).is_empty());
        let a = Stock { x: 70.0, y: 40.0, z: 12.0, offset: false };
        assert_eq!(a.bounds(min, max), ([-5.0, 0.0, -10.0], [65.0, 40.0, 2.0]));
        let small = Stock { x: 50.0, y: 40.0, z: 12.0, offset: false };
        assert_eq!(small.issues(min, max).len(), 1);
        assert_eq!(Stock::default().bounds(min, max), (min, max));
    }
}
