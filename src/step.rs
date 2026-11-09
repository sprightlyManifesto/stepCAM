//! Minimal STEP (ISO 10303-21) reader for 2.5D machining.
//!
//! Supports B-rep faces (ADVANCED_FACE / FACE_SURFACE) bounded by EDGE_LOOPs
//! of EDGE_CURVEs whose geometry is LINE, CIRCLE, ELLIPSE, B_SPLINE_CURVE
//! (incl. rational), or SURFACE_CURVE/SEAM_CURVE wrapping those. Planar faces
//! are triangulated directly; cylindrical, conical, spherical, toroidal and
//! B-spline faces are tessellated through their surface parameters
//! (`surface.rs`); other faces show their edges only.
//! Free wire geometry (unused EDGE_CURVEs, POLYLINEs and TRIMMED_CURVEs in
//! GEOMETRIC_(CURVE_)SETs) becomes free edges. Assembly transforms are ignored.

use std::collections::{HashMap, HashSet};

pub type P3 = [f64; 3];

// ------------------------------------------------------------------ vectors
fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn mul(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: P3, b: P3) -> P3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: P3) -> P3 {
    let n = norm(a);
    if n < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        mul(a, 1.0 / n)
    }
}
pub fn dist(a: P3, b: P3) -> f64 {
    norm(sub(a, b))
}

// ------------------------------------------------------------------ part 21 parser
#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    Ref(u64),
    Num(f64),
    Str(String),
    Enum(String),
    List(Vec<Val>),
    Typed(String, Vec<Val>),
    Null,
}

impl Val {
    fn as_ref(&self) -> Option<u64> {
        match self {
            Val::Ref(r) => Some(*r),
            _ => None,
        }
    }
    fn as_num(&self) -> Option<f64> {
        match self {
            Val::Num(n) => Some(*n),
            Val::Typed(_, v) => v.first().and_then(|x| x.as_num()),
            _ => None,
        }
    }
    fn as_list(&self) -> &[Val] {
        match self {
            Val::List(v) => v,
            _ => &[],
        }
    }
    fn as_bool(&self) -> Option<bool> {
        match self {
            Val::Enum(e) if e == "T" => Some(true),
            Val::Enum(e) if e == "F" => Some(false),
            _ => None,
        }
    }
}

/// Entity record: one (name, args) part, or several for complex entities.
#[derive(Clone, Debug)]
pub struct Ent {
    pub parts: Vec<(String, Vec<Val>)>,
}

impl Ent {
    fn name(&self) -> &str {
        &self.parts[0].0
    }
    fn args(&self) -> &[Val] {
        &self.parts[0].1
    }
    fn part(&self, name: &str) -> Option<&[Val]> {
        self.parts.iter().find(|p| p.0 == name).map(|p| p.1.as_slice())
    }
    fn has(&self, name: &str) -> bool {
        self.parts.iter().any(|p| p.0 == name)
    }
}

struct Cur<'a> {
    s: &'a [u8],
    i: usize,
}

impl<'a> Cur<'a> {
    fn ws(&mut self) {
        while self.i < self.s.len() && (self.s[self.i] as char).is_ascii_whitespace() {
            self.i += 1;
        }
    }
    fn peek(&mut self) -> Option<u8> {
        self.ws();
        self.s.get(self.i).copied()
    }
    fn eat(&mut self, c: u8) -> bool {
        if self.peek() == Some(c) {
            self.i += 1;
            true
        } else {
            false
        }
    }
    fn ident(&mut self) -> String {
        self.ws();
        let st = self.i;
        while self.i < self.s.len() && ((self.s[self.i] as char).is_ascii_alphanumeric() || self.s[self.i] == b'_' || self.s[self.i] == b'-') {
            self.i += 1;
        }
        String::from_utf8_lossy(&self.s[st..self.i]).to_ascii_uppercase()
    }
    fn list(&mut self) -> Result<Vec<Val>, String> {
        if !self.eat(b'(') {
            return Err(format!("expected '(' at {}", self.i));
        }
        let mut v = Vec::new();
        if self.eat(b')') {
            return Ok(v);
        }
        loop {
            v.push(self.val()?);
            if self.eat(b',') {
                continue;
            }
            if self.eat(b')') {
                return Ok(v);
            }
            return Err(format!("expected ',' or ')' at {}", self.i));
        }
    }
    fn val(&mut self) -> Result<Val, String> {
        match self.peek() {
            Some(b'#') => {
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i].is_ascii_digit() {
                    self.i += 1;
                }
                let n = std::str::from_utf8(&self.s[st..self.i]).unwrap_or("0").parse().unwrap_or(0);
                Ok(Val::Ref(n))
            }
            Some(b'\'') => {
                self.i += 1;
                let mut out = Vec::new();
                while self.i < self.s.len() {
                    if self.s[self.i] == b'\'' {
                        if self.s.get(self.i + 1) == Some(&b'\'') {
                            out.push(b'\'');
                            self.i += 2;
                            continue;
                        }
                        self.i += 1;
                        break;
                    }
                    out.push(self.s[self.i]);
                    self.i += 1;
                }
                Ok(Val::Str(String::from_utf8_lossy(&out).into()))
            }
            Some(b'.') => {
                self.i += 1;
                let id = self.ident();
                self.eat(b'.');
                Ok(Val::Enum(id))
            }
            Some(b'$') | Some(b'*') => {
                self.i += 1;
                Ok(Val::Null)
            }
            Some(b'(') => Ok(Val::List(self.list()?)),
            Some(c) if c == b'-' || c == b'+' || c.is_ascii_digit() => {
                let st = self.i;
                self.i += 1;
                while self.i < self.s.len() {
                    let c = self.s[self.i];
                    if c.is_ascii_digit() || c == b'.' || c == b'E' || c == b'e' || ((c == b'-' || c == b'+') && matches!(self.s[self.i - 1], b'E' | b'e')) {
                        self.i += 1;
                    } else {
                        break;
                    }
                }
                let t = std::str::from_utf8(&self.s[st..self.i]).unwrap_or("0");
                Ok(Val::Num(t.parse().map_err(|_| format!("bad number '{t}'"))?))
            }
            Some(c) if c.is_ascii_alphabetic() => {
                let name = self.ident();
                let args = self.list()?;
                Ok(Val::Typed(name, args))
            }
            other => Err(format!("unexpected {:?} at {}", other.map(|c| c as char), self.i)),
        }
    }
}

fn strip_comments(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    let mut in_str = false;
    while i < b.len() {
        let c = b[i];
        if in_str {
            out.push(c as char);
            if c == b'\'' {
                in_str = false;
            }
            i += 1;
        } else if c == b'\'' {
            in_str = true;
            out.push('\'');
            i += 1;
        } else if c == b'/' && b.get(i + 1) == Some(&b'*') {
            i += 2;
            while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
                i += 1;
            }
            i += 2;
        } else {
            out.push(c as char);
            i += 1;
        }
    }
    out
}

/// Split the DATA section into "#id=..." records (split on ';' outside strings).
pub fn parse_entities(src: &str) -> Result<HashMap<u64, Ent>, String> {
    let src = strip_comments(src);
    let data_start = src.find("DATA;").ok_or("not a STEP file (no DATA section)")? + 5;
    let body = &src[data_start..];
    let mut ents = HashMap::new();
    let bytes = body.as_bytes();
    let mut st = 0;
    let mut in_str = false;
    for i in 0..bytes.len() {
        let c = bytes[i];
        if c == b'\'' {
            in_str = !in_str;
        } else if c == b';' && !in_str {
            let rec = body[st..i].trim();
            st = i + 1;
            if rec.starts_with("ENDSEC") {
                break;
            }
            if !rec.starts_with('#') {
                continue;
            }
            let Some(eq) = rec.find('=') else { continue };
            let Ok(id) = rec[1..eq].trim().parse::<u64>() else { continue };
            let mut cur = Cur { s: rec[eq + 1..].as_bytes(), i: 0 };
            let mut parts = Vec::new();
            if cur.peek() == Some(b'(') {
                cur.i += 1;
                while cur.peek().map_or(false, |c| c != b')') {
                    let name = cur.ident();
                    if name.is_empty() {
                        break;
                    }
                    match cur.list() {
                        Ok(a) => parts.push((name, a)),
                        Err(_) => break,
                    }
                }
            } else {
                let name = cur.ident();
                match cur.list() {
                    Ok(a) => parts.push((name, a)),
                    Err(e) => return Err(format!("#{id}: {e}")),
                }
            }
            if !parts.is_empty() {
                ents.insert(id, Ent { parts });
            }
        }
    }
    Ok(ents)
}

// ------------------------------------------------------------------ model
#[derive(Clone, Debug)]
pub struct Edge {
    /// Polyline from start to end vertex.
    pub pts: Vec<P3>,
    /// Faces this edge bounds (empty = free wire edge).
    pub faces: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct Loop {
    pub outer: bool,
    pub edges: Vec<usize>,
    pub pts: Vec<P3>,
}

#[derive(Clone, Debug)]
pub struct Face {
    pub planar: bool,
    /// Outward normal (planar faces only).
    pub normal: P3,
    pub origin: P3,
    pub loops: Vec<Loop>,
    pub tris: Vec<[P3; 3]>,
    /// Cylindrical faces: the cylinder.
    pub cyl: Option<Cyl>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cyl {
    /// A point on the axis and the axis direction.
    pub origin: P3,
    pub axis: P3,
    pub r: f64,
    /// Material outside the cylinder (a hole), not inside (a boss).
    pub hole: bool,
}

impl Cyl {
    /// Axis parallel to Z.
    pub fn vertical(&self) -> bool {
        self.axis[2].abs() > 0.9999
    }
}

impl Face {
    /// Planar, facing +Z (machinable from above).
    pub fn is_up_horizontal(&self) -> bool {
        self.planar && self.normal[2] > 0.9999
    }
    pub fn is_horizontal(&self) -> bool {
        self.planar && self.normal[2].abs() > 0.9999
    }
    pub fn z(&self) -> f64 {
        self.origin[2]
    }
    pub fn outer_loop(&self) -> Option<&Loop> {
        self.loops.iter().find(|l| l.outer)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Model {
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
    pub min: P3,
    pub max: P3,
    pub unit_scale: f64,
}

struct Ctx<'a> {
    e: &'a HashMap<u64, Ent>,
    scale: f64,
    /// Plane angle unit in radians (1, or pi/180 for degrees).
    angle: f64,
}

impl<'a> Ctx<'a> {
    fn get(&self, id: u64) -> Option<&'a Ent> {
        self.e.get(&id)
    }
    fn point(&self, id: u64) -> Option<P3> {
        let ent = self.get(id)?;
        match ent.name() {
            "CARTESIAN_POINT" => {
                let c = ent.args().get(1)?.as_list();
                let g = |k: usize| c.get(k).and_then(|v| v.as_num()).unwrap_or(0.0) * self.scale;
                Some([g(0), g(1), g(2)])
            }
            "VERTEX_POINT" => self.point(ent.args().get(1)?.as_ref()?),
            _ => None,
        }
    }
    fn dir(&self, id: u64) -> Option<P3> {
        let ent = self.get(id)?;
        match ent.name() {
            "DIRECTION" => {
                let c = ent.args().get(1)?.as_list();
                let g = |k: usize| c.get(k).and_then(|v| v.as_num()).unwrap_or(0.0);
                Some(unit([g(0), g(1), g(2)]))
            }
            "VECTOR" => self.dir(ent.args().get(1)?.as_ref()?),
            _ => None,
        }
    }
    /// (origin, z axis, x axis)
    fn placement(&self, id: u64) -> Option<(P3, P3, P3)> {
        let ent = self.get(id)?;
        let a = ent.args();
        let o = self.point(a.get(1)?.as_ref()?)?;
        let z = a.get(2).and_then(|v| v.as_ref()).and_then(|r| self.dir(r)).unwrap_or([0.0, 0.0, 1.0]);
        let mut x = a.get(3).and_then(|v| v.as_ref()).and_then(|r| self.dir(r)).unwrap_or_else(|| {
            if z[0].abs() < 0.9 {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            }
        });
        // make x orthogonal to z
        x = unit(sub(x, mul(z, dot(x, z))));
        Some((o, z, x))
    }

    /// Curved surface geometry (None for planes and unsupported surfaces).
    fn surface(&self, ent: &Ent) -> Option<crate::surface::Surf> {
        use crate::surface::{BSpline, Frame, Surf};
        let a = ent.args();
        let frame = || {
            let (o, z, x) = self.placement(a.get(1)?.as_ref()?)?;
            Some(Frame { o, x, y: cross(z, x), z })
        };
        let num = |k: usize| a.get(k).and_then(|v| v.as_num());
        match ent.name() {
            "CYLINDRICAL_SURFACE" => Some(Surf::Cylinder { f: frame()?, r: num(2)? * self.scale }),
            "CONICAL_SURFACE" => Some(Surf::Cone { f: frame()?, r: num(2)? * self.scale, tan: (num(3)? * self.angle).tan() }),
            "SPHERICAL_SURFACE" => Some(Surf::Sphere { f: frame()?, r: num(2)? * self.scale }),
            "TOROIDAL_SURFACE" => Some(Surf::Torus { f: frame()?, major: num(2)? * self.scale, minor: num(3)? * self.scale }),
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let (pts, closed) = self.curve_samples(a.get(1)?.as_ref()?)?;
                let d = self.dir(a.get(2)?.as_ref()?)?;
                Some(Surf::Extrusion { pts, closed, d })
            }
            _ if ent.has("B_SPLINE_SURFACE_WITH_KNOTS") || ent.has("B_SPLINE_SURFACE") => {
                // simple: B_SPLINE_SURFACE_WITH_KNOTS(name, ud, vd, pts, form, uc, vc, si, um, vm, uk, vk, spec)
                // complex: B_SPLINE_SURFACE(ud, vd, pts, form, uc, vc, si) + ..._WITH_KNOTS(um, vm, uk, vk, spec)
                let (b, k): (Vec<&Val>, Vec<&Val>) = if let Some(bs) = ent.part("B_SPLINE_SURFACE") {
                    let k = ent.part("B_SPLINE_SURFACE_WITH_KNOTS")?;
                    (bs.iter().collect(), k.iter().collect())
                } else {
                    (a.iter().skip(1).take(7).collect(), a.iter().skip(8).collect())
                };
                let deg = |v: &Val| v.as_num().map(|d| d as usize);
                let (udeg, vdeg) = (deg(b.first()?)?, deg(b.get(1)?)?);
                let pts: Vec<Vec<P3>> = b.get(2)?.as_list().iter().map(|row| row.as_list().iter().filter_map(|r| r.as_ref()).filter_map(|r| self.point(r)).collect()).collect();
                let weights: Option<Vec<Vec<f64>>> = ent
                    .part("RATIONAL_B_SPLINE_SURFACE")
                    .and_then(|w| w.first())
                    .map(|l| l.as_list().iter().map(|row| row.as_list().iter().filter_map(|v| v.as_num()).collect()).collect());
                let ctrl: Vec<Vec<[f64; 4]>> = pts
                    .iter()
                    .enumerate()
                    .map(|(i, row)| {
                        row.iter()
                            .enumerate()
                            .map(|(j, p)| {
                                let w = weights.as_ref().and_then(|w| w.get(i)?.get(j).copied()).unwrap_or(1.0);
                                [p[0] * w, p[1] * w, p[2] * w, w]
                            })
                            .collect()
                    })
                    .collect();
                let kv = |m: &Val, k: &Val| {
                    let mut v = vec![];
                    for (m, k) in m.as_list().iter().zip(k.as_list()) {
                        for _ in 0..m.as_num().unwrap_or(1.0) as usize {
                            v.push(k.as_num().unwrap_or(0.0));
                        }
                    }
                    v
                };
                let bs = BSpline {
                    udeg,
                    vdeg,
                    ctrl,
                    ukv: kv(k.first()?, k.get(2)?),
                    vkv: kv(k.get(1)?, k.get(3)?),
                    uclosed: b.get(4).and_then(|v| v.as_bool()).unwrap_or(false),
                    vclosed: b.get(5).and_then(|v| v.as_bool()).unwrap_or(false),
                };
                bs.is_valid().then_some(Surf::BSpline(bs))
            }
            _ => None,
        }
    }

    /// A whole curve as a polyline, and whether it is closed (circles,
    /// ellipses and B-splines; lines have no extent of their own).
    fn curve_samples(&self, id: u64) -> Option<(Vec<P3>, bool)> {
        let ent = self.get(self.base_curve(id)?)?;
        let (mut pts, closed) = match ent.name() {
            "CIRCLE" | "ELLIPSE" => {
                let args = ent.args();
                let (o, z, x) = self.placement(args.get(1)?.as_ref()?)?;
                let y = cross(z, x);
                let r1 = args.get(2)?.as_num()? * self.scale;
                let r2 = if ent.name() == "ELLIPSE" { args.get(3)?.as_num()? * self.scale } else { r1 };
                let n = 144;
                let v = (0..n).map(|k| {
                    let t = std::f64::consts::TAU * k as f64 / n as f64;
                    add(o, add(mul(x, r1 * t.cos()), mul(y, r2 * t.sin())))
                });
                (v.collect(), true)
            }
            _ if ent.has("B_SPLINE_CURVE_WITH_KNOTS") || ent.has("B_SPLINE_CURVE") => {
                let v = self.bspline_samples(ent);
                let closed = v.len() > 2 && dist(v[0], *v.last().unwrap()) < 1e-6;
                (v, closed)
            }
            _ => return None,
        };
        if closed && pts.len() > 1 && dist(pts[0], *pts.last().unwrap()) < 1e-6 {
            pts.pop();
        }
        (pts.len() >= 2).then_some((pts, closed))
    }

    /// Unwrap SURFACE_CURVE / SEAM_CURVE to their 3D curve.
    fn base_curve(&self, id: u64) -> Option<u64> {
        let ent = self.get(id)?;
        match ent.name() {
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" => self.base_curve(ent.args().get(1)?.as_ref()?),
            _ => Some(id),
        }
    }

    /// Tessellate a curve between two points. `forward` = the curve's parameter
    /// increases from `a` to `b`. Returns a polyline a..b.
    fn curve_pts(&self, curve: u64, a: P3, b: P3, forward: bool) -> Vec<P3> {
        let closed = dist(a, b) < 1e-7;
        let Some(cid) = self.base_curve(curve) else { return vec![a, b] };
        let Some(ent) = self.get(cid) else { return vec![a, b] };
        let mut pts = match ent.name() {
            "LINE" => vec![a, b],
            "CIRCLE" | "ELLIPSE" => {
                let args = ent.args();
                let Some((o, z, x)) = args.get(1).and_then(|v| v.as_ref()).and_then(|r| self.placement(r)) else {
                    return vec![a, b];
                };
                let y = cross(z, x);
                let r1 = args.get(2).and_then(|v| v.as_num()).unwrap_or(1.0) * self.scale;
                let r2 = if ent.name() == "ELLIPSE" { args.get(3).and_then(|v| v.as_num()).unwrap_or(1.0) * self.scale } else { r1 };
                let ang = |p: P3| {
                    let d = sub(p, o);
                    (dot(d, y) / r2).atan2(dot(d, x) / r1)
                };
                // walk in parameter direction from p0 to p1
                let (p0, p1) = if forward { (a, b) } else { (b, a) };
                let a0 = ang(p0);
                let mut sweep = ang(p1) - a0;
                let tau = std::f64::consts::TAU;
                if closed {
                    sweep = tau;
                } else {
                    while sweep <= 1e-9 {
                        sweep += tau;
                    }
                    while sweep > tau + 1e-9 {
                        sweep -= tau;
                    }
                }
                let n = ((sweep / 5f64.to_radians()).ceil() as usize).max(4);
                let mut v: Vec<P3> = (0..=n)
                    .map(|k| {
                        let t = a0 + sweep * k as f64 / n as f64;
                        add(o, add(mul(x, r1 * t.cos()), mul(y, r2 * t.sin())))
                    })
                    .collect();
                if !forward {
                    v.reverse();
                }
                v
            }
            "POLYLINE" => {
                let mut v: Vec<P3> = ent.args().get(1).map(|l| l.as_list().iter().filter_map(|r| r.as_ref()).filter_map(|r| self.point(r)).collect()).unwrap_or_default();
                if !forward {
                    v.reverse();
                }
                if v.len() < 2 {
                    vec![a, b]
                } else {
                    v
                }
            }
            _ if ent.has("B_SPLINE_CURVE_WITH_KNOTS") || ent.has("B_SPLINE_CURVE") => {
                let samples = self.bspline_samples(ent);
                if samples.len() < 2 {
                    vec![a, b]
                } else {
                    sub_range(&samples, a, b, forward, closed)
                }
            }
            _ => vec![a, b],
        };
        if let Some(f) = pts.first_mut() {
            *f = a;
        }
        if let Some(l) = pts.last_mut() {
            *l = b;
        }
        pts
    }

    fn bspline_samples(&self, ent: &Ent) -> Vec<P3> {
        // simple entity: B_SPLINE_CURVE_WITH_KNOTS(name, deg, pts, form, closed, selfint, mults, knots, spec)
        // complex entity: B_SPLINE_CURVE(deg, pts, form, closed, selfint) + B_SPLINE_CURVE_WITH_KNOTS(mults, knots, spec)
        let (deg, cps, mults, knots) = if let Some(bs) = ent.part("B_SPLINE_CURVE") {
            let Some(k) = ent.part("B_SPLINE_CURVE_WITH_KNOTS") else { return vec![] };
            (bs.first(), bs.get(1), k.first(), k.get(1))
        } else {
            let a = ent.args();
            (a.get(1), a.get(2), a.get(6), a.get(7))
        };
        let (Some(deg), Some(cps), Some(mults), Some(knots)) = (deg, cps, mults, knots) else { return vec![] };
        let p = deg.as_num().unwrap_or(1.0) as usize;
        let ctrl: Vec<P3> = cps.as_list().iter().filter_map(|r| r.as_ref()).filter_map(|r| self.point(r)).collect();
        let weights: Vec<f64> = ent
            .part("RATIONAL_B_SPLINE_CURVE")
            .and_then(|w| w.first())
            .map(|l| l.as_list().iter().filter_map(|v| v.as_num()).collect())
            .unwrap_or_else(|| vec![1.0; ctrl.len()]);
        let mut kv = Vec::new();
        for (m, k) in mults.as_list().iter().zip(knots.as_list()) {
            for _ in 0..m.as_num().unwrap_or(1.0) as usize {
                kv.push(k.as_num().unwrap_or(0.0));
            }
        }
        let n = ctrl.len();
        if n == 0 || kv.len() != n + p + 1 || weights.len() != n {
            return vec![];
        }
        let (t0, t1) = (kv[p], kv[n]);
        let m = (n * 8).clamp(32, 400);
        (0..=m)
            .map(|i| {
                let t = t0 + (t1 - t0) * i as f64 / m as f64;
                de_boor(p, &kv, &ctrl, &weights, t)
            })
            .collect()
    }
}

fn de_boor(p: usize, kv: &[f64], ctrl: &[P3], w: &[f64], t: f64) -> P3 {
    let n = ctrl.len();
    let mut k = p;
    while k + 1 < n && kv[k + 1] <= t {
        k += 1;
    }
    let mut d: Vec<[f64; 4]> = (0..=p)
        .map(|j| {
            let c = ctrl[j + k - p];
            let wj = w[j + k - p];
            [c[0] * wj, c[1] * wj, c[2] * wj, wj]
        })
        .collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = j + k - p;
            let den = kv[i + p + 1 - r] - kv[i];
            let alpha = if den.abs() < 1e-14 { 0.0 } else { (t - kv[i]) / den };
            for c in 0..4 {
                d[j][c] = (1.0 - alpha) * d[j - 1][c] + alpha * d[j][c];
            }
        }
    }
    let h = d[p];
    let wt = if h[3].abs() < 1e-14 { 1.0 } else { h[3] };
    [h[0] / wt, h[1] / wt, h[2] / wt]
}

/// Take the part of a sampled curve between the points nearest `a` and `b`.
fn sub_range(s: &[P3], a: P3, b: P3, forward: bool, closed: bool) -> Vec<P3> {
    if closed {
        let mut v = s.to_vec();
        if !forward {
            v.reverse();
        }
        return v;
    }
    let near = |p: P3| (0..s.len()).min_by(|&i, &j| dist(s[i], p).total_cmp(&dist(s[j], p))).unwrap_or(0);
    let (ia, ib) = (near(a), near(b));
    if forward && ib >= ia {
        s[ia..=ib].to_vec()
    } else if !forward && ia >= ib {
        let mut v = s[ib..=ia].to_vec();
        v.reverse();
        v
    } else {
        vec![a, b]
    }
}

/// Plane angle unit in radians: degrees when declared as a conversion-based unit.
fn angle_unit(e: &HashMap<u64, Ent>) -> f64 {
    for ent in e.values() {
        if ent.has("PLANE_ANGLE_UNIT") && ent.has("CONVERSION_BASED_UNIT") {
            if let Some(Val::Str(n)) = ent.part("CONVERSION_BASED_UNIT").and_then(|c| c.first()) {
                if n.to_ascii_uppercase().contains("DEG") {
                    return std::f64::consts::PI / 180.0;
                }
            }
        }
    }
    1.0
}

fn length_unit_scale(e: &HashMap<u64, Ent>) -> f64 {
    for ent in e.values() {
        if ent.has("LENGTH_UNIT") {
            if let Some(c) = ent.part("CONVERSION_BASED_UNIT") {
                if let Some(Val::Str(n)) = c.first() {
                    let n = n.to_ascii_uppercase();
                    if n.contains("INCH") {
                        return 25.4;
                    }
                    if n.contains("FOOT") {
                        return 304.8;
                    }
                }
            }
            if let Some(si) = ent.part("SI_UNIT") {
                return match si.first() {
                    Some(Val::Enum(p)) if p == "MILLI" => 1.0,
                    Some(Val::Enum(p)) if p == "CENTI" => 10.0,
                    Some(Val::Enum(p)) if p == "MICRO" => 0.001,
                    Some(Val::Enum(p)) if p == "KILO" => 1e6,
                    _ => 1000.0,
                };
            }
        }
    }
    1.0
}

fn get_edge(ctx: &Ctx, edge_idx: &mut HashMap<u64, usize>, model: &mut Model, id: u64) -> Option<usize> {
    if let Some(&i) = edge_idx.get(&id) {
        return Some(i);
    }
    let ent = ctx.get(id)?;
    if ent.name() != "EDGE_CURVE" {
        return None;
    }
    let a = ent.args();
    let p0 = ctx.point(a.get(1)?.as_ref()?)?;
    let p1 = ctx.point(a.get(2)?.as_ref()?)?;
    let fwd = a.get(4).and_then(|v| v.as_bool()).unwrap_or(true);
    let pts = match a.get(3).and_then(|v| v.as_ref()) {
        Some(c) => ctx.curve_pts(c, p0, p1, fwd),
        None => vec![p0, p1],
    };
    model.edges.push(Edge { pts, faces: vec![] });
    let i = model.edges.len() - 1;
    edge_idx.insert(id, i);
    Some(i)
}

pub fn load(path: &str) -> Result<Model, String> {
    let src = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    parse_model(&String::from_utf8_lossy(&src))
}

pub fn parse_model(src: &str) -> Result<Model, String> {
    let ents = parse_entities(src)?;
    let ctx = Ctx { e: &ents, scale: length_unit_scale(&ents), angle: angle_unit(&ents) };
    let mut model = Model { unit_scale: ctx.scale, ..Default::default() };
    let mut edge_idx: HashMap<u64, usize> = HashMap::new();

    let mut face_ids: Vec<u64> = ents
        .iter()
        .filter(|(_, e)| matches!(e.name(), "ADVANCED_FACE" | "FACE_SURFACE"))
        .map(|(id, _)| *id)
        .collect();
    face_ids.sort_unstable();

    for fid in face_ids {
        let ent = &ents[&fid];
        let a = ent.args();
        let same_sense = a.get(3).and_then(|v| v.as_bool()).unwrap_or(true);
        let surf = a.get(2).and_then(|v| v.as_ref()).and_then(|r| ctx.get(r));
        let plane = surf.filter(|s| s.name() == "PLANE").and_then(|s| s.args().get(1)?.as_ref()).and_then(|r| ctx.placement(r));
        let face_index = model.faces.len();
        let mut loops = Vec::new();
        // loop points in the direction the bound runs round the face
        let mut sensed: Vec<Vec<P3>> = Vec::new();
        for b in a.get(1).map(|v| v.as_list()).unwrap_or(&[]) {
            let Some(bent) = b.as_ref().and_then(|r| ctx.get(r)) else { continue };
            let outer = bent.name() == "FACE_OUTER_BOUND";
            let bound_sense = bent.args().get(2).and_then(|v| v.as_bool()).unwrap_or(true);
            let Some(lent) = bent.args().get(1).and_then(|v| v.as_ref()).and_then(|r| ctx.get(r)) else { continue };
            if lent.name() != "EDGE_LOOP" {
                continue;
            }
            let mut lp = Loop { outer, edges: vec![], pts: vec![] };
            for oe in lent.args().get(1).map(|v| v.as_list()).unwrap_or(&[]) {
                let Some(oent) = oe.as_ref().and_then(|r| ctx.get(r)) else { continue };
                if oent.name() != "ORIENTED_EDGE" {
                    continue;
                }
                let oa = oent.args();
                let Some(ei) = oa.get(3).and_then(|v| v.as_ref()).and_then(|r| get_edge(&ctx, &mut edge_idx, &mut model, r)) else { continue };
                let orient = oa.get(4).and_then(|v| v.as_bool()).unwrap_or(true);
                if !model.edges[ei].faces.contains(&face_index) {
                    model.edges[ei].faces.push(face_index);
                }
                lp.edges.push(ei);
                let mut ep = model.edges[ei].pts.clone();
                if !orient {
                    ep.reverse();
                }
                if !lp.pts.is_empty() {
                    ep.remove(0);
                }
                lp.pts.extend(ep);
            }
            if lp.pts.len() > 1 && dist(lp.pts[0], *lp.pts.last().unwrap()) < 1e-6 {
                lp.pts.pop();
            }
            if !lp.edges.is_empty() {
                let mut sp = lp.pts.clone();
                if !bound_sense {
                    sp.reverse();
                }
                sensed.push(sp);
                loops.push(lp);
            }
        }
        if loops.is_empty() {
            continue;
        }
        let (planar, normal, origin) = match plane {
            Some((o, z, _)) => (true, if same_sense { z } else { mul(z, -1.0) }, o),
            None => (false, [0.0, 0.0, 0.0], loops[0].pts[0]),
        };
        // without an explicit outer bound, the loop with the largest area is outer
        if planar && !loops.iter().any(|l| l.outer) {
            let areas: Vec<f64> = loops.iter().map(|l| poly_area_in_plane(&l.pts, normal).abs()).collect();
            if let Some(best) = (0..loops.len()).max_by(|&i, &j| areas[i].total_cmp(&areas[j])) {
                loops[best].outer = true;
            }
        }
        let curved = if planar { None } else { surf.and_then(|s| ctx.surface(s)) };
        let tris = if planar {
            triangulate(&loops, normal)
        } else {
            curved.as_ref().map(|s| crate::surface::tessellate(s, &sensed, same_sense)).unwrap_or_default()
        };
        // the surface normal of a cylinder points away from the axis; the face
        // normal points out of the material, so a hole has them opposed
        let cyl = match &curved {
            Some(crate::surface::Surf::Cylinder { f, r }) => Some(Cyl { origin: f.o, axis: f.z, r: *r, hole: !same_sense }),
            _ => None,
        };
        model.faces.push(Face { planar, normal, origin, loops, tris, cyl });
    }

    // free edges: EDGE_CURVEs not used by a face
    let mut unused: Vec<u64> = ents.iter().filter(|(id, e)| e.name() == "EDGE_CURVE" && !edge_idx.contains_key(id)).map(|(id, _)| *id).collect();
    unused.sort_unstable();
    for id in unused {
        get_edge(&ctx, &mut edge_idx, &mut model, id);
    }
    // free wire geometry in geometric sets
    let mut set_items: Vec<u64> = Vec::new();
    for e in ents.values() {
        if matches!(e.name(), "GEOMETRIC_CURVE_SET" | "GEOMETRIC_SET") {
            set_items.extend(e.args().get(1).map(|l| l.as_list().iter().filter_map(|v| v.as_ref()).collect::<Vec<_>>()).unwrap_or_default());
        }
    }
    set_items.sort_unstable();
    set_items.dedup();
    for id in set_items {
        let Some(ent) = ctx.get(id) else { continue };
        let pts = match ent.name() {
            "POLYLINE" => {
                let v: Vec<P3> = ent.args().get(1).map(|l| l.as_list().iter().filter_map(|r| r.as_ref()).filter_map(|r| ctx.point(r)).collect()).unwrap_or_default();
                v
            }
            "TRIMMED_CURVE" => {
                let a = ent.args();
                let trim_pt = |v: Option<&Val>| v.map(|l| l.as_list().iter().filter_map(|x| x.as_ref()).find_map(|r| ctx.point(r))).flatten();
                let (Some(basis), Some(t1), Some(t2)) = (a.get(1).and_then(|v| v.as_ref()), trim_pt(a.get(2)), trim_pt(a.get(3))) else { continue };
                let sense = a.get(4).and_then(|v| v.as_bool()).unwrap_or(true);
                ctx.curve_pts(basis, t1, t2, sense)
            }
            _ => continue,
        };
        if pts.len() >= 2 {
            model.edges.push(Edge { pts, faces: vec![] });
        }
    }

    if model.edges.is_empty() {
        return Err("no supported geometry (faces or edges) found in STEP file".into());
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in model.edges.iter().flat_map(|e| e.pts.iter()) {
        for k in 0..3 {
            min[k] = min[k].min(p[k]);
            max[k] = max[k].max(p[k]);
        }
    }
    model.min = min;
    model.max = max;
    Ok(model)
}

fn bbox(model: &Model) -> (P3, P3) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in model.edges.iter().flat_map(|e| e.pts.iter()) {
        for k in 0..3 {
            min[k] = min[k].min(p[k]);
            max[k] = max[k].max(p[k]);
        }
    }
    (min, max)
}

/// Rigid placement of the model in machine coordinates: p' = rot * p + offset.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Placement {
    pub rot: [[f64; 3]; 3],
    pub offset: P3,
}

impl Default for Placement {
    fn default() -> Self {
        Placement { rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], offset: [0.0; 3] }
    }
}

fn mat_vec(m: &[[f64; 3]; 3], v: P3) -> P3 {
    [0, 1, 2].map(|r| m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2])
}

fn mat_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [0, 1, 2].map(|r| [0, 1, 2].map(|c| (0..3).map(|k| a[r][k] * b[k][c]).sum()))
}

impl Placement {
    pub fn is_identity(&self) -> bool {
        *self == Placement::default()
    }
    pub fn dir(&self, d: P3) -> P3 {
        mat_vec(&self.rot, d)
    }
    pub fn point(&self, p: P3) -> P3 {
        let q = self.dir(p);
        [q[0] + self.offset[0], q[1] + self.offset[1], q[2] + self.offset[2]]
    }
    /// Inverse direction transform (rot is orthonormal: inverse = transpose).
    pub fn inv_dir(&self, d: P3) -> P3 {
        let m = &self.rot;
        [0, 1, 2].map(|c| m[0][c] * d[0] + m[1][c] * d[1] + m[2][c] * d[2])
    }
    pub fn inv_point(&self, p: P3) -> P3 {
        self.inv_dir([p[0] - self.offset[0], p[1] - self.offset[1], p[2] - self.offset[2]])
    }
    /// Move by `t` after this placement.
    pub fn translated(&self, t: P3) -> Placement {
        Placement { rot: self.rot, offset: [0, 1, 2].map(|k| self.offset[k] + t[k]) }
    }
    /// Rotate by `r` (about the origin) after this placement.
    pub fn rotated(&self, r: &[[f64; 3]; 3]) -> Placement {
        Placement { rot: mat_mul(r, &self.rot), offset: mat_vec(r, self.offset) }
    }
}

/// Smallest rotation taking unit vector `n` onto `-Z`.
pub fn rotation_to_down(n: P3) -> [[f64; 3]; 3] {
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    let n = [n[0] / l, n[1] / l, n[2] / l];
    let c = -n[2]; // n . (0,0,-1)
    if c < -0.999999 {
        // facing straight up: turn over about X
        return [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
    }
    // v = n x (0,0,-1)
    let v = [-n[1], n[0], 0.0];
    let k = [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]];
    let k2 = mat_mul(&k, &k);
    let f = 1.0 / (1.0 + c);
    [0, 1, 2].map(|r| [0, 1, 2].map(|col| if r == col { 1.0 } else { 0.0 } + k[r][col] + k2[r][col] * f))
}

impl Model {
    /// Copy of the model with `pl` applied to all geometry.
    pub fn transformed(&self, pl: &Placement) -> Model {
        let mut m = self.clone();
        if pl.is_identity() {
            return m;
        }
        let pt = |p: &mut P3| *p = pl.point(*p);
        for e in &mut m.edges {
            e.pts.iter_mut().for_each(pt);
        }
        for f in &mut m.faces {
            f.normal = pl.dir(f.normal);
            f.origin = pl.point(f.origin);
            for l in &mut f.loops {
                l.pts.iter_mut().for_each(pt);
            }
            for t in &mut f.tris {
                t.iter_mut().for_each(pt);
            }
            if let Some(c) = &mut f.cyl {
                c.origin = pl.point(c.origin);
                c.axis = pl.dir(c.axis);
            }
        }
        (m.min, m.max) = bbox(&m);
        m
    }
}

fn plane_basis(n: P3) -> (P3, P3) {
    let x = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let x = unit(sub(x, mul(n, dot(x, n))));
    (x, cross(n, x))
}

fn poly_area_in_plane(pts: &[P3], n: P3) -> f64 {
    let (u, v) = plane_basis(n);
    let p2: Vec<(f64, f64)> = pts.iter().map(|p| (dot(*p, u), dot(*p, v))).collect();
    let mut a = 0.0;
    for i in 0..p2.len() {
        let (x0, y0) = p2[i];
        let (x1, y1) = p2[(i + 1) % p2.len()];
        a += x0 * y1 - x1 * y0;
    }
    a / 2.0
}

fn triangulate(loops: &[Loop], n: P3) -> Vec<[P3; 3]> {
    let (u, v) = plane_basis(n);
    let mut flat = Vec::new();
    let mut pts3 = Vec::new();
    let mut holes = Vec::new();
    let mut ordered: Vec<&Loop> = loops.iter().filter(|l| l.outer).collect();
    ordered.truncate(1);
    ordered.extend(loops.iter().filter(|l| !l.outer));
    for (k, l) in ordered.iter().enumerate() {
        if l.pts.len() < 3 {
            continue;
        }
        if k > 0 {
            holes.push(pts3.len());
        }
        for p in &l.pts {
            flat.push(dot(*p, u));
            flat.push(dot(*p, v));
            pts3.push(*p);
        }
    }
    match earcutr::earcut(&flat, &holes, 2) {
        Ok(idx) => idx.chunks(3).filter(|c| c.len() == 3).map(|c| [pts3[c[0]], pts3[c[1]], pts3[c[2]]]).collect(),
        Err(_) => vec![],
    }
}

// ------------------------------------------------------------------ selection helpers
fn key(p: P3) -> (i64, i64, i64) {
    let q = |v: f64| (v * 1e4).round() as i64;
    (q(p[0]), q(p[1]), q(p[2]))
}

impl Model {
    /// Edges selected by clicking `edge`, and whether they form a closed loop.
    /// Face edges select the loop on their horizontal face (preferring an
    /// upward face, then the higher one); free edges select the chain of
    /// touching free edges.
    /// The edges of a chain joined end to end into one polyline
    /// (a closed chain ends at its start point).
    pub fn chain_polyline(&self, chain: &[usize]) -> Vec<P3> {
        let edges: Vec<&Vec<P3>> = chain.iter().filter_map(|&i| self.edges.get(i)).map(|e| &e.pts).filter(|p| p.len() >= 2).collect();
        if edges.is_empty() {
            return vec![];
        }
        let mut count: HashMap<(i64, i64, i64), usize> = HashMap::new();
        for p in &edges {
            *count.entry(key(p[0])).or_default() += 1;
            *count.entry(key(*p.last().unwrap())).or_default() += 1;
        }
        // open chain: start at a free end
        let (mut start, mut rev) = (0, false);
        'find: for (i, p) in edges.iter().enumerate() {
            for (end, r) in [(p[0], false), (*p.last().unwrap(), true)] {
                if count[&key(end)] == 1 {
                    (start, rev) = (i, r);
                    break 'find;
                }
            }
        }
        let mut used = vec![false; edges.len()];
        used[start] = true;
        let mut pts: Vec<P3> = edges[start].clone();
        if rev {
            pts.reverse();
        }
        loop {
            let at = key(*pts.last().unwrap());
            let Some((i, r)) = edges.iter().enumerate().filter(|(i, _)| !used[*i]).find_map(|(i, p)| {
                if key(p[0]) == at {
                    Some((i, false))
                } else if key(*p.last().unwrap()) == at {
                    Some((i, true))
                } else {
                    None
                }
            }) else {
                break;
            };
            used[i] = true;
            let mut q = edges[i].clone();
            if r {
                q.reverse();
            }
            pts.extend(q.into_iter().skip(1));
        }
        pts
    }

    pub fn edge_chain(&self, edge: usize) -> (Vec<usize>, bool) {
        let e = &self.edges[edge];
        if !e.faces.is_empty() {
            let best = e
                .faces
                .iter()
                .copied()
                .filter(|&f| self.faces[f].is_horizontal())
                .max_by(|&a, &b| {
                    let (fa, fb) = (&self.faces[a], &self.faces[b]);
                    (fa.is_up_horizontal(), fa.z()).partial_cmp(&(fb.is_up_horizontal(), fb.z())).unwrap_or(std::cmp::Ordering::Equal)
                });
            if let Some(f) = best {
                if let Some(l) = self.faces[f].loops.iter().find(|l| l.edges.contains(&edge)) {
                    return (l.edges.clone(), true);
                }
            }
            return (vec![edge], false);
        }
        // free edges: walk through endpoints shared by exactly two free edges
        let mut ends: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (i, e) in self.edges.iter().enumerate() {
            if e.faces.is_empty() && e.pts.len() >= 2 {
                ends.entry(key(e.pts[0])).or_default().push(i);
                ends.entry(key(*e.pts.last().unwrap())).or_default().push(i);
            }
        }
        let mut seen: HashSet<usize> = HashSet::from([edge]);
        let mut chain = vec![edge];
        let mut closed = false;
        for start_end in [*e.pts.last().unwrap(), e.pts[0]] {
            let mut at = key(start_end);
            loop {
                let next = ends.get(&at).and_then(|v| if v.len() == 2 { v.iter().copied().find(|i| !seen.contains(i)) } else { None });
                let Some(n) = next else {
                    if ends.get(&at).map_or(false, |v| v.len() == 2 && v.iter().all(|i| seen.contains(i))) && chain.len() > 1 {
                        closed = true;
                    }
                    break;
                };
                seen.insert(n);
                chain.push(n);
                let ne = &self.edges[n];
                at = if key(ne.pts[0]) == at { key(*ne.pts.last().unwrap()) } else { key(ne.pts[0]) };
            }
            if closed {
                break;
            }
        }
        if chain.len() == 1 && dist(e.pts[0], *e.pts.last().unwrap()) < 1e-6 {
            closed = true;
        }
        (chain, closed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOX: &str = include_str!("../testdata/pocket_block.step");

    #[test]
    fn placement_lay_flat_and_inverse() {
        let n = [0.0, -1.0, 0.0];
        let r = rotation_to_down(n);
        let d = mat_vec(&r, n);
        assert!((d[2] + 1.0).abs() < 1e-12 && d[0].abs() < 1e-12 && d[1].abs() < 1e-12);
        let pl = Placement::default().rotated(&r).translated([1.0, 2.0, 3.0]);
        let p = [4.0, -5.0, 6.0];
        let q = pl.inv_point(pl.point(p));
        assert!((0..3).all(|k| (q[k] - p[k]).abs() < 1e-12));
        let up = rotation_to_down([0.0, 0.0, 1.0]);
        assert!((mat_vec(&up, [0.0, 0.0, 1.0])[2] + 1.0).abs() < 1e-12);
    }

    #[test]
    fn parses_values() {
        let e = parse_entities("DATA;\n#1=CARTESIAN_POINT('a''b',(1.,-2.5E1,3));\n#2=(A(1)B(#1,.T.,$));ENDSEC;").unwrap();
        assert_eq!(e[&1].args()[1], Val::List(vec![Val::Num(1.0), Val::Num(-25.0), Val::Num(3.0)]));
        assert_eq!(e[&1].args()[0], Val::Str("a'b".into()));
        assert_eq!(e[&2].parts.len(), 2);
        assert_eq!(e[&2].part("B").unwrap()[1], Val::Enum("T".into()));
    }

    #[test]
    fn loads_test_block() {
        let m = parse_model(BOX).unwrap();
        assert_eq!(m.faces.len(), 13);
        let up: Vec<&Face> = m.faces.iter().filter(|f| f.is_up_horizontal()).collect();
        assert_eq!(up.len(), 3); // top, pocket floor, hole floor
        let top = up.iter().find(|f| f.z().abs() < 1e-9).unwrap();
        assert_eq!(top.loops.len(), 3);
        assert!(!top.tris.is_empty());
        assert!((m.min[2] + 10.0).abs() < 1e-9 && (m.max[0] - 60.0).abs() < 1e-9);
        // circle edges are tessellated on radius 4
        let circ = m.edges.iter().find(|e| e.pts.len() > 10).unwrap();
        for p in &circ.pts {
            let r = ((p[0] - 52.0).powi(2) + (p[1] - 32.0).powi(2)).sqrt();
            assert!((r - 4.0).abs() < 1e-6);
        }
    }

    #[test]
    fn edge_chains() {
        let m = parse_model(BOX).unwrap();
        // a pocket floor edge -> closed loop of 4
        let floor = m.faces.iter().position(|f| f.is_up_horizontal() && (f.z() + 4.0).abs() < 1e-9).unwrap();
        let e = m.faces[floor].loops[0].edges[0];
        let (chain, closed) = m.edge_chain(e);
        assert!(closed);
        assert_eq!(chain.len(), 4);
        // free wire: two touching edges -> open chain of 2
        let free = m.edges.iter().position(|e| e.faces.is_empty()).unwrap();
        let (chain, closed) = m.edge_chain(free);
        assert!(!closed);
        assert_eq!(chain.len(), 2);
        // L wire (-10,0)-(-10,20)-(-20,20) joined end to end
        let pl = m.chain_polyline(&chain);
        assert_eq!(pl.len(), 3);
        let ends = [pl[0], pl[2]];
        assert!(ends.iter().any(|p| dist(*p, [-10.0, 0.0, 0.0]) < 1e-9) && ends.iter().any(|p| dist(*p, [-20.0, 20.0, 0.0]) < 1e-9));
        assert!(dist(pl[1], [-10.0, 20.0, 0.0]) < 1e-9);
        // closed loop ends where it starts
        let (chain, _) = m.edge_chain(m.faces[floor].loops[0].edges[0]);
        let pl = m.chain_polyline(&chain);
        assert_eq!(pl.len(), 5);
        assert!(dist(pl[0], pl[4]) < 1e-9);
    }
}
