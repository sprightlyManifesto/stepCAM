//! G-code simulator: the stock is a Z-map (a height field on a regular XY
//! grid) and every move sweeps the tool's real shape through it, lowering
//! the material wherever the cutter passes below it.
//!
//! The program is split into straight pieces (arcs into short chords) with
//! the time each takes from the run-time estimate, so playback runs at
//! machine speed. Snapshots of the Z-map let playback jump backwards
//! without re-cutting from the start.

use crate::model::Settings;
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

pub type P3 = [f64; 3];

/// Cutting profile of a tool.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    /// Flat end mill (also thread mills): radius.
    Flat { r: f64 },
    /// Tapered engraving bit: tip flat radius, tan of the half angle, radius
    /// where the taper meets the shank.
    Vee { tip: f64, tan: f64, r: f64 },
}

impl Shape {
    pub fn radius(self) -> f64 {
        match self {
            Shape::Flat { r } | Shape::Vee { r, .. } => r,
        }
    }

    /// Height of the cutting surface above the tip at distance `d` from the
    /// axis (None outside the tool).
    pub fn height(self, d: f64) -> Option<f64> {
        match self {
            Shape::Flat { r } => (d <= r).then_some(0.0),
            Shape::Vee { tip, tan, r } => {
                if d > r {
                    None
                } else if d <= tip {
                    Some(0.0)
                } else {
                    Some((d - tip) / tan.max(1e-6))
                }
            }
        }
    }

    /// Length of the taper (Vee) from the tip to the full radius.
    pub fn taper_len(self) -> f64 {
        match self {
            Shape::Flat { .. } => 0.0,
            Shape::Vee { tip, tan, r } => (r - tip) / tan.max(1e-6),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SimTool {
    pub number: u32,
    pub shape: Shape,
    /// Cutting length: deeper engagement is reported.
    pub flute: f64,
    pub name: String,
    /// Found in the tool tables (else a stand-in Ø3 end mill).
    pub known: bool,
}

/// Shank of engraving bits (1/8").
const ENGRAVER_SHANK_R: f64 = 1.5875;

/// The tool with machine number `t` from the three tool tables.
pub fn tool_for(s: &Settings, t: u32) -> SimTool {
    if let Some(c) = s.cutters.iter().find(|c| c.tool == t) {
        return SimTool { number: t, shape: Shape::Flat { r: c.dia / 2.0 }, flute: c.flute, name: format!("T{t} end mill Ø{}", crate::gcode::fmt(c.dia)), known: true };
    }
    for (i, m) in s.thread_mills.iter().enumerate() {
        let n = if m.tool == 0 { 11 + i as u32 } else { m.tool };
        if n == t {
            return SimTool { number: t, shape: Shape::Flat { r: m.dia / 2.0 }, flute: m.reach, name: format!("T{t} thread mill Ø{}", crate::gcode::fmt(m.dia)), known: true };
        }
    }
    if let Some(e) = s.engravers.iter().find(|e| e.tool == t) {
        let tan = (e.angle.clamp(1.0, 179.0).to_radians() / 2.0).tan();
        let r = ENGRAVER_SHANK_R.max(e.width_at(e.max_depth) / 2.0).max(e.tip / 2.0 + 0.01);
        return SimTool { number: t, shape: Shape::Vee { tip: e.tip / 2.0, tan, r }, flute: e.max_depth, name: format!("T{t} engraver {}", e.name()), known: true };
    }
    SimTool { number: t, shape: Shape::Flat { r: 1.5 }, flute: 1e9, name: format!("T{t} (not in tool table, shown as Ø3)"), known: false }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Motion {
    Rapid,
    Feed,
}

/// One straight piece of a move.
#[derive(Clone, Copy, Debug)]
pub struct Piece {
    pub a: P3,
    pub b: P3,
    /// 0-based source line.
    pub line: usize,
    pub motion: Motion,
    /// Index into `Program::tools` (None before the first tool change).
    pub tool: Option<usize>,
    /// Feed (mm/min) of feed moves.
    pub feed: f64,
    /// Start time and duration (s).
    pub t0: f64,
    pub dt: f64,
}

impl Piece {
    pub fn at(&self, f: f64) -> P3 {
        lerp(self.a, self.b, f)
    }
}

fn lerp(a: P3, b: P3, f: f64) -> P3 {
    [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f]
}

#[derive(Clone, Debug, Default)]
pub struct Program {
    pub pieces: Vec<Piece>,
    pub tools: Vec<SimTool>,
    pub lines: Vec<String>,
    /// Total run time (s).
    pub total: f64,
    /// Bounds of all moves / of feed moves only (None if there are none).
    pub bounds: Option<(P3, P3)>,
    pub feed_bounds: Option<(P3, P3)>,
    /// Problems found while reading (unsupported codes, unknown tools).
    pub warnings: Vec<String>,
}

/// Chord error allowed when splitting arcs (mm).
const ARC_TOL: f64 = 0.002;
const ARC_MAX_STEP: f64 = std::f64::consts::PI / 36.0;

fn grow(b: &mut Option<(P3, P3)>, p: P3) {
    match b {
        None => *b = Some((p, p)),
        Some((lo, hi)) => {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
}

impl Program {
    pub fn parse(code: &str, s: &Settings) -> Program {
        let mut p = Program { lines: code.lines().map(str::to_owned).collect(), ..Default::default() };
        let times = crate::estimate::line_times(code, s.rapid_rate, s.max_accel);
        let mut pos: [Option<f64>; 3] = [None; 3];
        let mut motion = 0u8;
        let mut feed = 0.0;
        let mut tool: Option<usize> = None;
        let mut pending: u32 = 0;
        let mut t = 0.0;
        let mut warned_incremental = false;

        for (ln, raw) in code.lines().enumerate() {
            let mut line = String::with_capacity(raw.len());
            let mut depth = 0;
            for c in raw.chars() {
                match c {
                    '(' => depth += 1,
                    ')' if depth > 0 => depth -= 1,
                    ';' if depth == 0 => break,
                    _ if depth == 0 => line.push(c),
                    _ => {}
                }
            }
            let mut target: [Option<f64>; 3] = [None; 3];
            let (mut i, mut j) = (0.0, 0.0);
            let mut change = false;
            for word in line.split_whitespace() {
                let mut ch = word.chars();
                let Some(letter) = ch.next() else { continue };
                let Ok(val) = ch.as_str().parse::<f64>() else { continue };
                match letter.to_ascii_uppercase() {
                    'G' => {
                        let g = val.round() as i64;
                        if (val - g as f64).abs() < 1e-6 {
                            match g {
                                0..=3 => motion = g as u8,
                                20 | 91 if !warned_incremental => {
                                    warned_incremental = true;
                                    p.warnings.push(format!("line {}: G{g} (inch / incremental) is not simulated", ln + 1));
                                }
                                _ => {}
                            }
                        }
                    }
                    'X' => target[0] = Some(val),
                    'Y' => target[1] = Some(val),
                    'Z' => target[2] = Some(val),
                    'I' => i = val,
                    'J' => j = val,
                    'F' => feed = val,
                    'T' => pending = val.max(0.0) as u32,
                    'M' if val.round() as i64 == 6 => change = true,
                    _ => {}
                }
            }
            if change {
                let idx = match p.tools.iter().position(|tl| tl.number == pending) {
                    Some(k) => k,
                    None => {
                        let tl = tool_for(s, pending);
                        if !tl.known {
                            p.warnings.push(format!("line {}: T{pending} is not in the tool table (simulated as a Ø3 end mill)", ln + 1));
                        }
                        p.tools.push(tl);
                        p.tools.len() - 1
                    }
                };
                tool = Some(idx);
            }
            if target.iter().all(|t| t.is_none()) {
                continue;
            }
            let end = [target[0].or(pos[0]), target[1].or(pos[1]), target[2].or(pos[2])];
            if let (Some(sx), Some(sy), Some(sz)) = (pos[0], pos[1], pos[2]) {
                let start = [sx, sy, sz];
                let e = [end[0].unwrap_or(sx), end[1].unwrap_or(sy), end[2].unwrap_or(sz)];
                let mut pts = vec![start];
                if motion == 2 || motion == 3 {
                    let c = [start[0] + i, start[1] + j];
                    let r0 = (start[0] - c[0]).hypot(start[1] - c[1]);
                    let r1 = (e[0] - c[0]).hypot(e[1] - c[1]);
                    let a0 = (start[1] - c[1]).atan2(start[0] - c[0]);
                    let a1 = (e[1] - c[1]).atan2(e[0] - c[0]);
                    let tau = std::f64::consts::TAU;
                    let mut sweep = a1 - a0;
                    if motion == 3 {
                        while sweep <= 1e-9 {
                            sweep += tau;
                        }
                    } else {
                        while sweep >= -1e-9 {
                            sweep -= tau;
                        }
                    }
                    let r = r0.max(r1).max(1e-6);
                    let step = if ARC_TOL >= r { ARC_MAX_STEP } else { (2.0 * (1.0 - ARC_TOL / r).acos()).min(ARC_MAX_STEP) };
                    let n = (sweep.abs() / step).ceil().max(1.0) as usize;
                    for k in 1..=n {
                        let f = k as f64 / n as f64;
                        let a = a0 + sweep * f;
                        let rr = r0 + (r1 - r0) * f;
                        pts.push(if k == n { e } else { [c[0] + rr * a.cos(), c[1] + rr * a.sin(), start[2] + (e[2] - start[2]) * f] });
                    }
                } else {
                    pts.push(e);
                }
                let lens: Vec<f64> = pts.windows(2).map(|w| dist(w[0], w[1])).collect();
                let total_len: f64 = lens.iter().sum();
                let line_t = times.get(ln).copied().unwrap_or(0.0);
                let m = if motion == 0 { Motion::Rapid } else { Motion::Feed };
                for (w, len) in pts.windows(2).zip(&lens) {
                    if *len < 1e-9 {
                        continue;
                    }
                    let dt = if total_len > 0.0 { line_t * len / total_len } else { 0.0 };
                    p.pieces.push(Piece { a: w[0], b: w[1], line: ln, motion: m, tool, feed, t0: t, dt });
                    t += dt;
                    grow(&mut p.bounds, w[0]);
                    grow(&mut p.bounds, w[1]);
                    if m == Motion::Feed {
                        grow(&mut p.feed_bounds, w[0]);
                        grow(&mut p.feed_bounds, w[1]);
                    }
                }
            }
            pos = end;
        }
        p.total = t;
        if p.pieces.iter().any(|pc| pc.tool.is_none() && pc.motion == Motion::Feed) {
            p.warnings.push("feed moves before the first tool change (M6) are not cut".into());
        }
        p
    }

    /// Move every piece by `d` (program -> model coordinates).
    pub fn translate(&mut self, d: P3) {
        if d == [0.0; 3] {
            return;
        }
        let mv = |p: &mut P3| {
            for k in 0..3 {
                p[k] += d[k];
            }
        };
        for pc in &mut self.pieces {
            mv(&mut pc.a);
            mv(&mut pc.b);
        }
        for b in [&mut self.bounds, &mut self.feed_bounds].into_iter().flatten() {
            mv(&mut b.0);
            mv(&mut b.1);
        }
    }

    /// Index of the piece running at time `t` and the fraction done of it.
    pub fn locate(&self, t: f64) -> (usize, f64) {
        if self.pieces.is_empty() || t <= 0.0 {
            return (0, 0.0);
        }
        if t >= self.total {
            return (self.pieces.len(), 0.0);
        }
        let k = self.pieces.partition_point(|p| p.t0 + p.dt <= t);
        if k >= self.pieces.len() {
            return (self.pieces.len(), 0.0);
        }
        let p = &self.pieces[k];
        (k, if p.dt > 0.0 { ((t - p.t0) / p.dt).clamp(0.0, 1.0) } else { 0.0 })
    }

    /// Time when piece `k` starts (total for k past the end).
    pub fn start_of(&self, k: usize) -> f64 {
        self.pieces.get(k).map_or(self.total, |p| p.t0)
    }

    /// Time at the end of source line `line`'s move (None if it has none).
    pub fn end_of_line(&self, line: usize) -> Option<f64> {
        let k = self.pieces.partition_point(|p| p.line <= line);
        (k > 0 && self.pieces[k - 1].line == line).then(|| self.start_of(k))
    }

    /// Start time of the next / previous move (G-code line) from time `t`.
    pub fn next_move(&self, t: f64) -> f64 {
        let (k, _) = self.locate(t);
        let Some(p) = self.pieces.get(k) else { return self.total };
        let m = self.pieces.partition_point(|q| q.line <= p.line);
        self.start_of(m)
    }

    pub fn prev_move(&self, t: f64) -> f64 {
        let (k, f) = self.locate(t);
        let k = k.min(self.pieces.len());
        // start of the line holding piece k (or the last piece at the end)
        let cur = if k < self.pieces.len() { k } else if k > 0 { k - 1 } else { return 0.0 };
        let line = self.pieces[cur].line;
        let first = self.pieces.partition_point(|q| q.line < line);
        if self.start_of(first) < t - 1e-9 && !(first == k && f == 0.0) {
            return self.start_of(first);
        }
        if first == 0 {
            return 0.0;
        }
        let pl = self.pieces[first - 1].line;
        self.start_of(self.pieces.partition_point(|q| q.line < pl))
    }
}

fn dist(a: P3, b: P3) -> f64 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
}

/// Height field over the stock: `z[j * nx + i]` is the material top over
/// cell (i, j), centred at (x0 + (i + 0.5) cell, y0 + (j + 0.5) cell).
#[derive(Clone, Debug)]
pub struct ZMap {
    pub x0: f64,
    pub y0: f64,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    pub z: Vec<f32>,
    pub bottom: f64,
    pub top: f64,
}

/// Result of cutting one piece.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Cut {
    /// Volume removed (mm³).
    pub removed: f64,
    /// Highest material touching the tool (under it or in the wall right
    /// beside it), measured up from its tip (mm).
    pub depth: f64,
}

impl ZMap {
    /// Stock box `min`..`max` with at most `max_cells` cells along the longer side.
    pub fn new(min: P3, max: P3, max_cells: usize) -> ZMap {
        let (w, h) = ((max[0] - min[0]).max(1e-3), (max[1] - min[1]).max(1e-3));
        let cell = w.max(h) / max_cells.max(2) as f64;
        let nx = ((w / cell).round() as usize).max(1);
        let ny = ((h / cell).round() as usize).max(1);
        ZMap { x0: min[0], y0: min[1], cell, nx, ny, z: vec![max[2] as f32; nx * ny], bottom: min[2], top: max[2] }
    }

    pub fn centre(&self, i: usize, j: usize) -> [f64; 2] {
        [self.x0 + (i as f64 + 0.5) * self.cell, self.y0 + (j as f64 + 0.5) * self.cell]
    }

    /// Cells whose centres lie within [lo, hi] (clamped; empty if outside).
    fn range(&self, lo: f64, hi: f64, o: f64, n: usize) -> Option<(usize, usize)> {
        let a = ((lo - o) / self.cell - 0.5).ceil().max(0.0);
        let b = ((hi - o) / self.cell - 0.5).floor().min(n as f64 - 1.0);
        (a <= b).then(|| (a as usize, b as usize))
    }

    /// Sweep the tool tip from `a` to `b`, lowering the material under it.
    pub fn cut(&mut self, a: P3, b: P3, shape: Shape) -> Cut {
        let r = shape.radius();
        // walls within one cell outside the cutter count towards the depth
        let rw = r + self.cell;
        let mut out = Cut::default();
        let Some((i0, i1)) = self.range(a[0].min(b[0]) - rw, a[0].max(b[0]) + rw, self.x0, self.nx) else { return out };
        let Some((j0, j1)) = self.range(a[1].min(b[1]) - rw, a[1].max(b[1]) + rw, self.y0, self.ny) else { return out };
        let lowest = a[2].min(b[2]);
        let (ux, uy, uz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
        let l2 = ux * ux + uy * uy;
        let area = self.cell * self.cell;
        let bottom = self.bottom as f32;
        for j in j0..=j1 {
            for i in i0..=i1 {
                let idx = j * self.nx + i;
                let old = self.z[idx];
                if (lowest as f32) >= old {
                    continue;
                }
                let [cx, cy] = self.centre(i, j);
                let (dx, dy) = (cx - a[0], cy - a[1]);
                // lowest point of the tool over this cell: (surface z, tip z there)
                let hit = if l2 < 1e-18 {
                    let d = dx.hypot(dy);
                    if d > r && d <= rw {
                        out.depth = out.depth.max(old as f64 - lowest);
                    }
                    shape.height(d).map(|h| (lowest + h, lowest))
                } else {
                    let ts = (dx * ux + dy * uy) / l2;
                    let perp2 = (dx * dx + dy * dy - ts * ts * l2).max(0.0);
                    if perp2 > r * r {
                        // wall beside the cutter: its height over the tip next to it
                        let tc = ts.clamp(0.0, 1.0);
                        if (dx - ux * tc).hypot(dy - uy * tc) <= rw {
                            out.depth = out.depth.max(old as f64 - (a[2] + uz * tc));
                        }
                        continue;
                    }
                    let half = ((r * r - perp2) / l2).sqrt();
                    let (t0, t1) = ((ts - half).max(0.0), (ts + half).min(1.0));
                    if t0 > t1 {
                        continue;
                    }
                    let zt = |t: f64| a[2] + uz * t;
                    match shape {
                        Shape::Flat { .. } => {
                            let z = zt(t0).min(zt(t1));
                            Some((z, z))
                        }
                        Shape::Vee { .. } => {
                            let eval = |t: f64| {
                                let (px, py) = (dx - ux * t, dy - uy * t);
                                shape.height(px.hypot(py)).map(|h| (zt(t) + h, zt(t)))
                            };
                            let mut best: Option<(f64, f64)> = None;
                            let mut try_t = |t: f64| {
                                if let Some(v) = eval(t) {
                                    if best.map_or(true, |b| v.0 < b.0) {
                                        best = Some(v);
                                    }
                                }
                            };
                            try_t(ts.clamp(t0, t1));
                            if uz.abs() > 1e-9 {
                                try_t(t0);
                                try_t(t1);
                                for k in 1..8 {
                                    try_t(t0 + (t1 - t0) * k as f64 / 8.0);
                                }
                            }
                            best
                        }
                    }
                };
                let Some((z, tip)) = hit else { continue };
                let new = (z as f32).max(bottom);
                if new < old {
                    out.removed += (old - new) as f64 * area;
                    out.depth = out.depth.max(old as f64 - tip);
                    self.z[idx] = new;
                }
            }
        }
        out
    }

    /// Material volume left (mm³) above the stock bottom.
    pub fn volume(&self) -> f64 {
        self.z.iter().map(|&z| (z as f64 - self.bottom).max(0.0)).sum::<f64>() * self.cell * self.cell
    }

    /// Height of the top surface of `tris` over each cell (NaN where there is none).
    pub fn surface_of<'a>(&self, tris: impl Iterator<Item = &'a [P3; 3]>) -> Vec<f32> {
        let mut out = vec![f32::NAN; self.nx * self.ny];
        for t in tris {
            let (a, b, c) = (t[0], t[1], t[2]);
            let den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
            if den.abs() < 1e-12 {
                continue; // vertical
            }
            let (lo_x, hi_x) = (a[0].min(b[0]).min(c[0]), a[0].max(b[0]).max(c[0]));
            let (lo_y, hi_y) = (a[1].min(b[1]).min(c[1]), a[1].max(b[1]).max(c[1]));
            let (Some((i0, i1)), Some((j0, j1))) = (self.range(lo_x, hi_x, self.x0, self.nx), self.range(lo_y, hi_y, self.y0, self.ny)) else { continue };
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let [x, y] = self.centre(i, j);
                    let l0 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / den;
                    let l1 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den;
                    let l2 = 1.0 - l0 - l1;
                    if l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9 {
                        continue;
                    }
                    let z = (l0 * a[2] + l1 * b[2] + l2 * c[2]) as f32;
                    let o = &mut out[j * self.nx + i];
                    if o.is_nan() || z > *o {
                        *o = z;
                    }
                }
            }
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EventKind {
    /// A rapid (G0) move went through material.
    RapidCut,
    /// The tool cut deeper than its flute length.
    TooDeep,
}

#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub kind: EventKind,
    pub piece: usize,
    pub line: usize,
    /// Depth of material met (mm).
    pub depth: f64,
    pub flute: f64,
}

/// Ignore engagement shallower than this (numerical noise on grazing moves).
const MIN_DEPTH: f64 = 0.005;

/// Program + stock + playback position.
pub struct Sim {
    pub prog: Program,
    pub map: ZMap,
    /// Pieces fully cut, and the fraction of the next one cut so far.
    pub done: usize,
    pub partial: f64,
    /// Volume of the uncut stock (mm³).
    pub stock_volume: f64,
    pub events: BTreeMap<(usize, EventKind), Event>,
    /// Z-map state before piece `k`, every `snap_every` pieces.
    snaps: Vec<(usize, ZMap)>,
    snap_every: usize,
    /// Bumped whenever the Z-map changes (render caches key on it).
    pub version: u64,
}

impl Sim {
    pub fn new(prog: Program, stock_min: P3, stock_max: P3, max_cells: usize) -> Sim {
        let map = ZMap::new(stock_min, stock_max, max_cells);
        // about 24 snapshots, but at most ~96 MB of them
        let bytes = map.z.len() * 4;
        let count = (96_000_000 / bytes.max(1)).clamp(2, 24);
        let snap_every = (prog.pieces.len() / count).max(256);
        let stock_volume = map.volume();
        Sim { prog, snaps: vec![(0, map.clone())], map, done: 0, partial: 0.0, stock_volume, events: BTreeMap::new(), snap_every, version: 0 }
    }

    pub fn time(&self) -> f64 {
        match self.prog.pieces.get(self.done) {
            Some(p) => p.t0 + p.dt * self.partial,
            None => self.prog.total,
        }
    }

    /// Tool position now (None before the first move).
    pub fn position(&self) -> Option<P3> {
        let ps = &self.prog.pieces;
        match ps.get(self.done) {
            Some(p) if self.done > 0 || self.partial > 0.0 => Some(p.at(self.partial)),
            Some(p) => Some(p.a),
            None => ps.last().map(|p| p.b),
        }
    }

    /// The piece running now (the last one at the end).
    pub fn current(&self) -> Option<&Piece> {
        self.prog.pieces.get(self.done).or(self.prog.pieces.last())
    }

    pub fn removed(&self) -> f64 {
        self.stock_volume - self.map.volume()
    }

    /// Cut piece `k` from fraction f0 to f1.
    fn cut_piece(&mut self, k: usize, f0: f64, f1: f64) {
        let p = self.prog.pieces[k];
        let Some(ti) = p.tool else { return };
        let tool = &self.prog.tools[ti];
        let c = self.map.cut(p.at(f0), p.at(f1), tool.shape);
        if c.removed > 0.0 {
            self.version += 1;
        }
        let mut note = |kind: EventKind| {
            let e = self.events.entry((k, kind)).or_insert(Event { kind, piece: k, line: p.line, depth: 0.0, flute: tool.flute });
            e.depth = e.depth.max(c.depth);
        };
        if p.motion == Motion::Rapid && c.removed > 0.0 && c.depth > MIN_DEPTH {
            note(EventKind::RapidCut);
        }
        if c.depth > tool.flute + 1e-3 {
            note(EventKind::TooDeep);
        }
    }

    /// Move the simulation towards time `t`, spending at most `budget`.
    /// Returns true once it is there.
    pub fn seek(&mut self, t: f64, budget: Duration) -> bool {
        let start = Instant::now();
        let (k, f) = self.prog.locate(t);
        if (k, f) < (self.done, self.partial) {
            // back to the latest snapshot at or before k
            let s = self.snaps.partition_point(|(p, _)| *p <= k).max(1) - 1;
            let (p, m) = &self.snaps[s];
            self.done = *p;
            self.partial = 0.0;
            self.map = m.clone();
            self.version += 1;
        }
        let mut n = 0usize;
        while self.done < k {
            let f0 = self.partial;
            self.cut_piece(self.done, f0, 1.0);
            self.done += 1;
            self.partial = 0.0;
            if self.done % self.snap_every == 0 && self.snaps.last().map_or(true, |(p, _)| *p < self.done) {
                self.snaps.push((self.done, self.map.clone()));
            }
            n += 1;
            if n % 32 == 0 && start.elapsed() > budget {
                return false;
            }
        }
        if self.done < self.prog.pieces.len() && f > self.partial {
            let f0 = self.partial;
            self.cut_piece(self.done, f0, f);
            self.partial = f;
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings() -> Settings {
        Settings::default()
    }

    fn sim(code: &str, min: P3, max: P3, cells: usize) -> Sim {
        Sim::new(Program::parse(code, &settings()), min, max, cells)
    }

    fn run(s: &mut Sim) {
        while !s.seek(f64::INFINITY, Duration::from_secs(10)) {}
    }

    fn z_at(m: &ZMap, x: f64, y: f64) -> f32 {
        let i = ((x - m.x0) / m.cell) as usize;
        let j = ((y - m.y0) / m.cell) as usize;
        m.z[j * m.nx + i]
    }

    #[test]
    fn slot_with_a_flat_end_mill() {
        // T4 = Ø3: slot along X at Y10, 1 mm deep
        let code = "T4 M6\nG0 X5 Y10 Z5\nG1 Z-1 F100\nG1 X25 F600\nG0 Z5\n";
        let mut s = sim(code, [0.0, 0.0, -5.0], [30.0, 20.0, 0.0], 300);
        run(&mut s);
        let m = &s.map;
        assert!((z_at(m, 15.0, 10.0) + 1.0).abs() < 1e-6);
        assert!((z_at(m, 15.0, 11.4) + 1.0).abs() < 1e-6);
        assert_eq!(z_at(m, 15.0, 11.7), 0.0);
        assert_eq!(z_at(m, 27.0, 10.0), 0.0);
        // removed volume ~ (20 x 3 + pi 1.5^2) x 1
        let want = 20.0 * 3.0 + std::f64::consts::PI * 2.25;
        assert!((s.removed() - want).abs() < want * 0.03, "{} vs {want}", s.removed());
        assert!(s.events.is_empty(), "{:?}", s.events);
    }

    #[test]
    fn helix_bores_a_round_hole() {
        // Ø3 tool on a 2.5 mm radius helix: hole Ø8, 2 turns to Z-2
        let code = "T4 M6\nG0 X12.5 Y10 Z1\nG1 Z0 F100\nG3 X12.5 Y10 Z-1 I-2.5 J0\nG3 X12.5 Y10 Z-2 I-2.5 J0\nG3 X12.5 Y10 I-2.5 J0\nG0 Z5\n";
        let mut s = sim(code, [0.0, 0.0, -5.0], [20.0, 20.0, 0.0], 400);
        run(&mut s);
        let m = &s.map;
        assert_eq!(z_at(m, 10.0, 10.0), 0.0); // core left: the ring is 1 mm wide each side of r 2.5
        assert!((z_at(m, 12.5, 10.0) + 2.0).abs() < 1e-6);
        assert!((z_at(m, 11.1, 10.0) + 2.0).abs() < 1e-6);
        assert!((z_at(m, 13.8, 10.0) + 2.0).abs() < 1e-6);
        assert!((z_at(m, 10.0, 6.2) + 2.0).abs() < 1e-6);
        assert_eq!(z_at(m, 14.3, 10.0), 0.0);
        assert_eq!(z_at(m, 10.0, 14.3), 0.0);
    }

    #[test]
    fn vee_bit_cuts_a_v_groove() {
        // T23 = 90 deg, tip 0.3: 0.5 deep -> top width 0.3 + 2 x 0.5 = 1.3
        let code = "T23 M6\nG0 X2 Y5 Z1\nG1 Z-0.5 F100\nG1 X8 F300\nG0 Z5\n";
        let mut s = sim(code, [0.0, 0.0, -3.0], [10.0, 10.0, 0.0], 500);
        run(&mut s);
        let m = &s.map;
        assert!((z_at(m, 5.0, 5.0) + 0.5).abs() < 0.03);
        // 90 deg: depth falls 1:1 past the tip flat
        let z = z_at(m, 5.0, 5.4) as f64;
        assert!((z - (-0.5 + (0.4 - 0.15))).abs() < 0.03, "{z}");
        assert_eq!(z_at(m, 5.0, 5.75), 0.0);
    }

    #[test]
    fn rapids_through_material_and_too_deep_cuts_are_reported() {
        let code = "T1 M6\nG0 X5 Y5 Z5\nG0 Z-1\nG1 Z-4 F100\nG0 Z5\n";
        let mut s = sim(code, [0.0, 0.0, -10.0], [10.0, 10.0, 0.0], 100);
        run(&mut s);
        let kinds: Vec<EventKind> = s.events.values().map(|e| e.kind).collect();
        assert!(kinds.contains(&EventKind::RapidCut), "{kinds:?}");
        // T1 = Ø1 x 3 flute, 4 mm deep
        let deep = s.events.values().find(|e| e.kind == EventKind::TooDeep).expect("too deep");
        assert_eq!(deep.line, 3);
        assert!((deep.depth - 4.0).abs() < 1e-6 && deep.flute == 3.0);
    }

    #[test]
    fn seeking_back_and_forth_gives_the_same_result() {
        let mut code = String::from("T4 M6\nG0 X2 Y2 Z1\nG1 Z-1 F600\n");
        for k in 0..400 {
            let y = 2.0 + (k % 40) as f64 * 0.4;
            code += &format!("G1 X{} Y{y}\n", if k % 2 == 0 { 28.0 } else { 2.0 });
        }
        let mut s = sim(&code, [0.0, 0.0, -3.0], [30.0, 20.0, 0.0], 200);
        let full = {
            let mut t = sim(&code, [0.0, 0.0, -3.0], [30.0, 20.0, 0.0], 200);
            run(&mut t);
            t.map.z
        };
        let total = s.prog.total;
        assert!(s.snap_every >= 256);
        for &t in &[total * 0.7, total * 0.2, total * 0.95, total * 0.5, f64::INFINITY] {
            while !s.seek(t, Duration::from_secs(10)) {}
            assert!((s.time() - t.min(total)).abs() < 1e-6);
        }
        assert_eq!(s.map.z, full);
        // a partly cut piece is finished exactly when playback passes it
        let mut p = sim(&code, [0.0, 0.0, -3.0], [30.0, 20.0, 0.0], 200);
        let mut t = 0.0;
        while t < total {
            t += total / 997.0;
            p.seek(t, Duration::from_secs(10));
        }
        run(&mut p);
        assert_eq!(p.map.z, full);
    }

    #[test]
    fn time_follows_the_estimate_and_moves_step() {
        let code = "T4 M6\nG0 X0 Y0 Z0\nG1 X10 F600\nG2 X20 Y0 I5 J0\nG0 Z5\n";
        let p = Program::parse(code, &settings());
        let est = crate::estimate::estimate(code, settings().rapid_rate, settings().max_accel);
        assert!((p.total - est).abs() < 1e-9);
        // arc: chords within ARC_TOL of a radius 5
        let arc: Vec<&Piece> = p.pieces.iter().filter(|q| q.line == 3).collect();
        assert!(arc.len() > 30);
        for q in &arc {
            let m = [(q.a[0] + q.b[0]) / 2.0 - 15.0, (q.a[1] + q.b[1]) / 2.0];
            assert!(5.0 - m[0].hypot(m[1]) <= ARC_TOL + 1e-9);
        }
        let e2 = p.end_of_line(2).unwrap();
        assert_eq!(p.next_move(0.0), e2);
        assert_eq!(p.next_move(e2), p.end_of_line(3).unwrap());
        assert_eq!(p.prev_move(e2 + 0.01), e2);
        assert_eq!(p.prev_move(e2), 0.0);
        assert_eq!(p.prev_move(p.total), p.end_of_line(3).unwrap());
        assert!(p.end_of_line(0).is_none());
    }

    #[test]
    fn surface_of_a_model() {
        let m = ZMap::new([0.0, 0.0, 0.0], [10.0, 10.0, 5.0], 10);
        let tris = [[[0.0, 0.0, 2.0], [10.0, 0.0, 2.0], [0.0, 10.0, 4.0]]];
        let s = m.surface_of(tris.iter());
        assert!((s[0] - (2.0 + 0.1)).abs() < 1e-5);
        assert!(s[99].is_nan());
    }

    #[test]
    fn simulates_a_generated_step_job() {
        // a.step (2 mm extra stock on top) roughed: no rapid ever touches material
        use crate::stepmill::*;
        let model = crate::step::parse_model(include_str!("../testdata/a.step")).unwrap();
        let (smin, smax) = Stock { z: 2.0, ..Default::default() }.bounds(model.min, model.max);
        let op = RoughOp { cutter: crate::model::Cutter::C3x17, start_z: smax[2], bottom_z: model.min[2], faces: rough_faces(&model), stock_region: [smin[0], smin[1], smax[0], smax[1]], ..Default::default() };
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let mut sim = Sim::new(Program::parse(&code, &s), smin, smax, 200);
        run(&mut sim);
        assert!(sim.removed() > 100.0);
        assert!(!sim.events.values().any(|e| e.kind == EventKind::RapidCut), "{:?}", sim.events);
        // and the part is not gouged: material everywhere at or above the model top
        let part = sim.map.surface_of(model.faces.iter().flat_map(|f| f.tris.iter()));
        let worst = sim.map.z.iter().zip(&part).filter(|(_, p)| !p.is_nan()).map(|(z, p)| p - z).fold(0f32, f32::max);
        assert!(worst < 0.07, "gouge {worst}");
    }
}
