//! Tessellation of curved STEP faces (cylinder, cone, sphere, torus and
//! B-spline surfaces). The face's boundary loops are mapped to the surface
//! parameters (u, v), the trimmed parameter region is cut into small cells,
//! each cell is triangulated and evaluated back onto the surface.

use std::f64::consts::{FRAC_PI_2, TAU};

pub type P3 = [f64; 3];
type P2 = [f64; 2];

/// Max distance between the triangles and the true surface (mm).
const TOL: f64 = 0.02;
/// Cell budget per face (keeps big faces from exploding the mesh).
const MAX_CELLS: usize = 3000;

#[derive(Clone, Debug)]
pub struct Frame {
    pub o: P3,
    pub x: P3,
    pub y: P3,
    pub z: P3,
}

#[derive(Clone, Debug)]
pub struct BSpline {
    pub udeg: usize,
    pub vdeg: usize,
    /// ctrl[i][j]: i along u, j along v; homogeneous (x w, y w, z w, w).
    pub ctrl: Vec<Vec<[f64; 4]>>,
    pub ukv: Vec<f64>,
    pub vkv: Vec<f64>,
    pub uclosed: bool,
    pub vclosed: bool,
}

#[derive(Clone, Debug)]
pub enum Surf {
    Cylinder { f: Frame, r: f64 },
    /// Radius `r` at v = 0, growing by `tan` per unit of v along the axis.
    Cone { f: Frame, r: f64, tan: f64 },
    Sphere { f: Frame, r: f64 },
    Torus { f: Frame, major: f64, minor: f64 },
    BSpline(BSpline),
    /// Curve (sampled polyline, u = index along it) swept along unit `d` (v).
    Extrusion { pts: Vec<P3>, closed: bool, d: P3 },
}

fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn mul(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

impl Frame {
    fn at(&self, a: f64, b: f64, c: f64) -> P3 {
        add(self.o, add(mul(self.x, a), add(mul(self.y, b), mul(self.z, c))))
    }
    fn local(&self, p: P3) -> P3 {
        let d = sub(p, self.o);
        [dot(d, self.x), dot(d, self.y), dot(d, self.z)]
    }
}

/// Angle step for a circle of radius `r` so the chord stays within TOL.
fn ang_step(r: f64) -> f64 {
    if r <= TOL {
        return 0.5;
    }
    (2.0 * (1.0 - TOL / r).clamp(-1.0, 1.0).acos()).clamp(0.02, 0.5)
}

fn de_boor4(p: usize, kv: &[f64], ctrl: &[[f64; 4]], t: f64) -> [f64; 4] {
    let n = ctrl.len();
    let mut k = p;
    while k + 1 < n && kv[k + 1] <= t {
        k += 1;
    }
    let mut d: Vec<[f64; 4]> = (0..=p).map(|j| ctrl[j + k - p]).collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = j + k - p;
            let den = kv[i + p + 1 - r] - kv[i];
            let a = if den.abs() < 1e-14 { 0.0 } else { (t - kv[i]) / den };
            for c in 0..4 {
                d[j][c] = (1.0 - a) * d[j - 1][c] + a * d[j][c];
            }
        }
    }
    d[p]
}

impl BSpline {
    fn urange(&self) -> (f64, f64) {
        (self.ukv[self.udeg], self.ukv[self.ctrl.len()])
    }
    fn vrange(&self) -> (f64, f64) {
        (self.vkv[self.vdeg], self.vkv[self.ctrl[0].len()])
    }
    fn eval(&self, u: f64, v: f64) -> P3 {
        let (u0, u1) = self.urange();
        let (v0, v1) = self.vrange();
        let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
        let rows: Vec<[f64; 4]> = self.ctrl.iter().map(|row| de_boor4(self.vdeg, &self.vkv, row, v)).collect();
        let h = de_boor4(self.udeg, &self.ukv, &rows, u);
        let w = if h[3].abs() < 1e-14 { 1.0 } else { h[3] };
        [h[0] / w, h[1] / w, h[2] / w]
    }
    /// Valid when the knot vectors and the control net agree.
    pub fn is_valid(&self) -> bool {
        let nu = self.ctrl.len();
        nu > self.udeg
            && self.ctrl.iter().all(|r| r.len() == self.ctrl[0].len())
            && self.ctrl[0].len() > self.vdeg
            && self.ukv.len() == nu + self.udeg + 1
            && self.vkv.len() == self.ctrl[0].len() + self.vdeg + 1
            && self.urange().1 > self.urange().0
            && self.vrange().1 > self.vrange().0
    }
}

impl Surf {
    pub fn eval(&self, u: f64, v: f64) -> P3 {
        match self {
            Surf::Cylinder { f, r } => f.at(r * u.cos(), r * u.sin(), v),
            Surf::Cone { f, r, tan } => {
                let rho = r + v * tan;
                f.at(rho * u.cos(), rho * u.sin(), v)
            }
            Surf::Sphere { f, r } => f.at(r * v.cos() * u.cos(), r * v.cos() * u.sin(), r * v.sin()),
            Surf::Torus { f, major, minor } => {
                let rho = major + minor * v.cos();
                f.at(rho * u.cos(), rho * u.sin(), minor * v.sin())
            }
            Surf::BSpline(b) => b.eval(u, v),
            Surf::Extrusion { pts, closed, d } => add(poly_at(pts, *closed, u), mul(*d, v)),
        }
    }

    fn period_u(&self) -> Option<f64> {
        match self {
            Surf::BSpline(b) => b.uclosed.then(|| b.urange().1 - b.urange().0),
            Surf::Extrusion { pts, closed, .. } => closed.then_some(pts.len() as f64),
            _ => Some(TAU),
        }
    }
    fn period_v(&self) -> Option<f64> {
        match self {
            Surf::Torus { .. } => Some(TAU),
            Surf::BSpline(b) => b.vclosed.then(|| b.vrange().1 - b.vrange().0),
            _ => None,
        }
    }

    /// Parameters of a point on (or near) the surface; `guess` helps
    /// free-form surfaces. None at a singular point (pole / apex).
    fn invert(&self, p: P3, guess: Option<P2>) -> Option<P2> {
        let polar = |f: &Frame| {
            let l = f.local(p);
            (l, l[0].hypot(l[1]))
        };
        match self {
            Surf::Cylinder { f, .. } => {
                let (l, rho) = polar(f);
                (rho > 1e-9).then(|| [l[1].atan2(l[0]), l[2]])
            }
            Surf::Cone { f, .. } => {
                let (l, rho) = polar(f);
                (rho > 1e-6).then(|| [l[1].atan2(l[0]), l[2]])
            }
            Surf::Sphere { f, .. } => {
                let (l, rho) = polar(f);
                (rho > 1e-6).then(|| [l[1].atan2(l[0]), l[2].atan2(rho)])
            }
            Surf::Torus { f, major, .. } => {
                let (l, rho) = polar(f);
                (rho > 1e-9).then(|| [l[1].atan2(l[0]), l[2].atan2(rho - major)])
            }
            Surf::BSpline(b) => Some(b.invert(p, guess)),
            Surf::Extrusion { pts, closed, d } => {
                // nearest point of the curve seen along the sweep direction
                let flat = |q: P3| sub(q, mul(*d, dot(q, *d)));
                let fp = flat(p);
                let n = pts.len();
                let segs = if *closed { n } else { n - 1 };
                let mut best = (f64::INFINITY, 0.0);
                for i in 0..segs {
                    let (a, b) = (flat(pts[i]), flat(pts[(i + 1) % n]));
                    let ab = sub(b, a);
                    let l2 = dot(ab, ab);
                    let t = if l2 < 1e-18 { 0.0 } else { (dot(sub(fp, a), ab) / l2).clamp(0.0, 1.0) };
                    let q = add(a, mul(ab, t));
                    let e = dot(sub(fp, q), sub(fp, q));
                    if e < best.0 {
                        best = (e, i as f64 + t);
                    }
                }
                let u = best.1;
                Some([u, dot(sub(p, poly_at(pts, *closed, u)), *d)])
            }
        }
    }

    /// Parameter range covered by the surface itself (singular ends).
    fn v_pole(&self, top: bool) -> Option<f64> {
        match self {
            Surf::Sphere { .. } => Some(if top { FRAC_PI_2 } else { -FRAC_PI_2 }),
            Surf::Cone { r, tan, .. } if tan.abs() > 1e-12 => Some(-r / tan),
            Surf::BSpline(b) => Some(if top { b.vrange().1 } else { b.vrange().0 }),
            _ => None,
        }
    }

    /// Cell size in (u, v) for the chord tolerance (None: straight along v).
    fn steps(&self, vr: (f64, f64)) -> (f64, Option<f64>) {
        match self {
            Surf::Cylinder { r, .. } => (ang_step(*r), None),
            Surf::Cone { r, tan, .. } => {
                let rho = (r + vr.0 * tan).abs().max((r + vr.1 * tan).abs());
                (ang_step(rho), None)
            }
            Surf::Sphere { r, .. } => (ang_step(*r), Some(ang_step(*r))),
            Surf::Torus { major, minor, .. } => (ang_step(major + minor), Some(ang_step(*minor))),
            Surf::BSpline(b) => {
                let spans = |kv: &[f64], d: usize, n: usize| kv[d..=n].windows(2).filter(|w| w[1] > w[0]).count().max(1);
                let (u0, u1) = b.urange();
                let (v0, v1) = b.vrange();
                let nu = (spans(&b.ukv, b.udeg, b.ctrl.len()) * if b.udeg > 1 { 8 } else { 1 }).clamp(1, 64);
                let nv = (spans(&b.vkv, b.vdeg, b.ctrl[0].len()) * if b.vdeg > 1 { 8 } else { 1 }).clamp(1, 64);
                ((u1 - u0) / nu as f64, Some((v1 - v0) / nv as f64))
            }
            Surf::Extrusion { .. } => (1.0, None),
        }
    }
}

impl BSpline {
    /// Closest parameters to `p`: best grid sample, then Gauss-Newton.
    fn invert(&self, p: P3, guess: Option<P2>) -> P2 {
        let (u0, u1) = self.urange();
        let (v0, v1) = self.vrange();
        let d2 = |a: P3| {
            let d = sub(a, p);
            dot(d, d)
        };
        let mut best = guess.map(|g| (g, d2(self.eval(g[0], g[1]))));
        let n = 16;
        if best.map_or(true, |b| b.1 > 1e-4) {
            for i in 0..=n {
                for j in 0..=n {
                    let uv = [u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64];
                    let e = d2(self.eval(uv[0], uv[1]));
                    if best.map_or(true, |b| e < b.1) {
                        best = Some((uv, e));
                    }
                }
            }
        }
        let [mut u, mut v] = best.unwrap().0;
        let (hu, hv) = ((u1 - u0) * 1e-6, (v1 - v0) * 1e-6);
        for _ in 0..20 {
            let s = self.eval(u, v);
            let su = mul(sub(self.eval(u + hu, v), self.eval(u - hu, v)), 0.5 / hu);
            let sv = mul(sub(self.eval(u, v + hv), self.eval(u, v - hv)), 0.5 / hv);
            let r = sub(p, s);
            let (a, b, c) = (dot(su, su), dot(su, sv), dot(sv, sv));
            let det = a * c - b * b;
            if det.abs() < 1e-18 {
                break;
            }
            let (gu, gv) = (dot(su, r), dot(sv, r));
            let du = (c * gu - b * gv) / det;
            let dv = (a * gv - b * gu) / det;
            u = (u + du).clamp(u0, u1);
            v = (v + dv).clamp(v0, v1);
            if du.abs() < 1e-10 * (u1 - u0) && dv.abs() < 1e-10 * (v1 - v0) {
                break;
            }
        }
        [u, v]
    }
}

/// Point at parameter `u` (segment index + fraction) along a polyline.
fn poly_at(pts: &[P3], closed: bool, u: f64) -> P3 {
    let n = pts.len();
    if n == 0 {
        return [0.0; 3];
    }
    let segs = if closed { n } else { n - 1 };
    if segs == 0 {
        return pts[0];
    }
    let u = if closed { u.rem_euclid(n as f64) } else { u.clamp(0.0, segs as f64) };
    let i = (u.floor() as usize).min(segs - 1);
    let t = u - i as f64;
    let (a, b) = (pts[i], pts[(i + 1) % n]);
    add(a, mul(sub(b, a), t))
}

fn signed_area(p: &[P2]) -> f64 {
    let mut a = 0.0;
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        a += q[0] * r[1] - r[0] * q[1];
    }
    a / 2.0
}

/// Shift `x` by whole periods to lie closest to `near`.
fn wrap_near(x: f64, near: f64, period: Option<f64>) -> f64 {
    match period {
        Some(p) => x + ((near - x) / p).round() * p,
        None => x,
    }
}

/// A loop in parameter space and how many periods it winds in u / v.
struct ParamLoop {
    pts: Vec<P2>,
    wind_u: i64,
    wind_v: i64,
}

fn param_loop(s: &Surf, pts: &[P3]) -> Option<ParamLoop> {
    let (pu, pv) = (s.period_u(), s.period_v());
    let mut raw: Vec<Option<P2>> = Vec::with_capacity(pts.len());
    let mut prev: Option<P2> = None;
    for &p in pts {
        let q = s.invert(p, prev);
        if q.is_some() {
            prev = q;
        }
        raw.push(q);
    }
    let known: Vec<usize> = (0..raw.len()).filter(|&i| raw[i].is_some()).collect();
    if known.len() < 2 {
        return None;
    }
    // singular points (pole / apex): v from the surface, u from a neighbour
    let mut out: Vec<P2> = Vec::with_capacity(pts.len());
    for i in 0..raw.len() {
        let q = match raw[i] {
            Some(q) => q,
            None => {
                let nb = raw[i..].iter().chain(raw[..i].iter()).flatten().next().copied().unwrap();
                let l = match s {
                    Surf::Sphere { f, .. } => f.local(pts[i])[2],
                    _ => 0.0,
                };
                let v = match s {
                    Surf::Sphere { .. } => s.v_pole(l > 0.0).unwrap(),
                    Surf::Cone { .. } => s.v_pole(true).unwrap(),
                    _ => nb[1],
                };
                [nb[0], v]
            }
        };
        out.push(q);
    }
    // unwrap periodic parameters along the loop
    for i in 1..out.len() {
        out[i][0] = wrap_near(out[i][0], out[i - 1][0], pu);
        out[i][1] = wrap_near(out[i][1], out[i - 1][1], pv);
    }
    let first = out[0];
    let last = *out.last().unwrap();
    let close_u = wrap_near(first[0], last[0], pu);
    let close_v = wrap_near(first[1], last[1], pv);
    let wind = |closed: f64, start: f64, p: Option<f64>| p.map_or(0, |p| ((closed - start) / p).round() as i64);
    let (wind_u, wind_v) = (wind(close_u, first[0], pu), wind(close_v, first[1], pv));
    if wind_u != 0 || wind_v != 0 {
        // keep the closing segment: the loop ends a whole period from its start
        out.push([close_u, close_v]);
    }
    Some(ParamLoop { wind_u, wind_v, pts: out })
}

/// Trimmed parameter region of a face: closed polygons (outer and holes).
fn param_region(s: &Surf, loops: &[Vec<P3>], same_sense: bool) -> Option<Vec<Vec<P2>>> {
    let mut closed = vec![];
    let mut open = vec![];
    for l in loops {
        let pl = param_loop(s, l)?;
        if pl.wind_v != 0 {
            return None; // loops all the way round in v: not handled
        }
        if pl.wind_u == 0 {
            closed.push(pl.pts);
        } else {
            open.push(pl);
        }
    }
    let pu = s.period_u();
    // loops going right round in u: pair a +u loop with a -u loop into a band
    let (mut plus, mut minus): (Vec<ParamLoop>, Vec<ParamLoop>) = open.into_iter().partition(|l| l.wind_u > 0);
    while !plus.is_empty() && !minus.is_empty() {
        let (a, mut b) = (plus.pop().unwrap(), minus.pop().unwrap());
        let end = a.pts.last().unwrap()[0];
        let shift = wrap_near(b.pts[0][0], end, pu) - b.pts[0][0];
        b.pts.iter_mut().for_each(|p| p[0] += shift);
        let mut band = a.pts;
        band.extend(b.pts);
        closed.push(band);
    }
    // a single loop right round: closed off at the pole on the face side
    for a in plus.into_iter().chain(minus) {
        let face_above = (a.wind_u > 0) == same_sense;
        let vp = s.v_pole(face_above)?;
        let (u0, u1) = (a.pts[0][0], a.pts.last().unwrap()[0]);
        let mut cap = a.pts;
        cap.push([u1, vp]);
        cap.push([u0, vp]);
        closed.push(cap);
    }
    closed.retain(|l| l.len() >= 3);
    if closed.is_empty() {
        return None;
    }
    // outer = largest; shift the holes into its parameter range
    closed.sort_by(|a, b| signed_area(b).abs().total_cmp(&signed_area(a).abs()));
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &closed[0] {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let mid = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let pv = s.period_v();
    for h in closed.iter_mut().skip(1) {
        let c = h.iter().fold([0.0, 0.0], |a, p| [a[0] + p[0], a[1] + p[1]]);
        let c = [c[0] / h.len() as f64, c[1] / h.len() as f64];
        let (du, dv) = (wrap_near(c[0], mid[0], pu) - c[0], wrap_near(c[1], mid[1], pv) - c[1]);
        h.iter_mut().for_each(|p| {
            p[0] += du;
            p[1] += dv;
        });
    }
    Some(closed)
}

fn earcut_poly(rings: &[Vec<P2>]) -> Vec<[P2; 3]> {
    let mut flat = vec![];
    let mut holes = vec![];
    let mut pts = vec![];
    for (k, r) in rings.iter().enumerate() {
        if r.len() < 3 {
            continue;
        }
        if k > 0 {
            holes.push(pts.len());
        }
        for p in r {
            flat.push(p[0]);
            flat.push(p[1]);
            pts.push(*p);
        }
    }
    match earcutr::earcut(&flat, &holes, 2) {
        Ok(idx) => idx.chunks(3).filter(|c| c.len() == 3).map(|c| [pts[c[0]], pts[c[1]], pts[c[2]]]).collect(),
        Err(_) => vec![],
    }
}

/// Triangles of a curved face bounded by `loops` (3D polylines, each closed
/// implicitly, oriented as in the STEP file). Empty if it can't be done.
pub fn tessellate(s: &Surf, loops: &[Vec<P3>], same_sense: bool) -> Vec<[P3; 3]> {
    use geo::BooleanOps;
    let Some(region) = param_region(s, loops, same_sense) else { return vec![] };
    let ring = |r: &[P2]| {
        let mut v: Vec<(f64, f64)> = r.iter().map(|p| (p[0], p[1])).collect();
        v.push(v[0]);
        geo::LineString::from(v)
    };
    // the outer ring CCW, holes CW (geo normalises orientation itself)
    let poly = geo::Polygon::new(ring(&region[0]), region[1..].iter().map(|h| ring(h)).collect());
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &region[0] {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let (su, sv) = s.steps((lo[1], hi[1]));
    let mut nu = (((hi[0] - lo[0]) / su).ceil() as usize).max(1);
    let mut nv = sv.map_or(1, |sv| (((hi[1] - lo[1]) / sv).ceil() as usize).max(1));
    while nu * nv > MAX_CELLS {
        nu = (nu * 3 / 4).max(1);
        nv = (nv * 3 / 4).max(1);
    }
    let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut tris = vec![];
        for i in 0..nu {
            for j in 0..nv {
                let (a0, a1) = (lo[0] + (hi[0] - lo[0]) * i as f64 / nu as f64, lo[0] + (hi[0] - lo[0]) * (i + 1) as f64 / nu as f64);
                let (b0, b1) = (lo[1] + (hi[1] - lo[1]) * j as f64 / nv as f64, lo[1] + (hi[1] - lo[1]) * (j + 1) as f64 / nv as f64);
                let cell = geo::Polygon::new(geo::LineString::from(vec![(a0, b0), (a1, b0), (a1, b1), (a0, b1), (a0, b0)]), vec![]);
                for piece in poly.intersection(&cell).0 {
                    let mut rings: Vec<Vec<P2>> = vec![piece.exterior().coords().map(|c| [c.x, c.y]).collect()];
                    rings.extend(piece.interiors().iter().map(|r| r.coords().map(|c| [c.x, c.y]).collect()));
                    for r in &mut rings {
                        if r.len() > 1 && r.first() == r.last() {
                            r.pop();
                        }
                    }
                    for t in earcut_poly(&rings) {
                        tris.push([s.eval(t[0][0], t[0][1]), s.eval(t[1][0], t[1][1]), s.eval(t[2][0], t[2][1])]);
                    }
                }
            }
        }
        tris
    }));
    res.unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> Frame {
        Frame { o: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] }
    }
    fn area(t: &[[P3; 3]]) -> f64 {
        t.iter()
            .map(|t| {
                let (a, b) = (sub(t[1], t[0]), sub(t[2], t[0]));
                let c = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
                dot(c, c).sqrt() / 2.0
            })
            .sum()
    }
    fn circle(r: f64, z: f64, n: usize, ccw: bool) -> Vec<P3> {
        let mut v: Vec<P3> = (0..n).map(|i| {
            let a = TAU * i as f64 / n as f64;
            [r * a.cos(), r * a.sin(), z]
        }).collect();
        if !ccw {
            v.reverse();
        }
        v
    }

    #[test]
    fn quarter_cylinder_patch() {
        let s = Surf::Cylinder { f: frame(), r: 5.0 };
        // u 0..90deg, v 0..10
        let arc = |z: f64, rev: bool| {
            let mut v: Vec<P3> = (0..=18).map(|i| {
                let a = FRAC_PI_2 * i as f64 / 18.0;
                [5.0 * a.cos(), 5.0 * a.sin(), z]
            }).collect();
            if rev {
                v.reverse();
            }
            v
        };
        let mut lp = arc(0.0, false);
        lp.extend(arc(10.0, true));
        let t = tessellate(&s, &[lp], true);
        assert!(!t.is_empty());
        assert!((area(&t) - FRAC_PI_2 * 5.0 * 10.0).abs() < 0.5, "{}", area(&t));
        for p in t.iter().flatten() {
            assert!((p[0].hypot(p[1]) - 5.0).abs() < 1e-9 && p[0] >= -1e-9 && p[1] >= -1e-9);
        }
    }

    #[test]
    fn full_band_from_two_circles() {
        let s = Surf::Cylinder { f: frame(), r: 3.0 };
        let t = tessellate(&s, &[circle(3.0, 0.0, 72, true), circle(3.0, 4.0, 72, false)], true);
        assert!((area(&t) - TAU * 3.0 * 4.0).abs() < 0.3, "{}", area(&t));
    }

    #[test]
    fn sphere_cap_closes_at_the_pole() {
        let s = Surf::Sphere { f: frame(), r: 10.0 };
        // cap above z = 5: circle of radius sqrt(75)
        let rc = 75f64.sqrt();
        let t = tessellate(&s, &[circle(rc, 5.0, 72, true)], true);
        let cap = TAU * 10.0 * 5.0;
        assert!((area(&t) - cap).abs() < cap * 0.02, "{} vs {cap}", area(&t));
        assert!(t.iter().flatten().all(|p| p[2] >= 5.0 - 1e-6));
        // the other orientation is the rest of the sphere
        let rest = tessellate(&s, &[circle(rc, 5.0, 72, false)], true);
        assert!(rest.iter().flatten().all(|p| p[2] <= 5.0 + 1e-6));
    }

    #[test]
    fn tilted_extrusion_of_an_arc() {
        // half circle r 5 in XY swept along +Y/+Z (45 degrees) by 10
        let arc: Vec<P3> = (0..=36).map(|i| {
            let a = std::f64::consts::PI * i as f64 / 36.0;
            [5.0 * a.cos(), 0.0, 5.0 * a.sin()]
        }).collect();
        let d = [0.0, 0.5f64.sqrt(), 0.5f64.sqrt()];
        let s = Surf::Extrusion { pts: arc.clone(), closed: false, d };
        let mut lp = arc.clone();
        lp.extend(arc.iter().rev().map(|p| add(*p, mul(d, 10.0))));
        let t = tessellate(&s, &[lp], true);
        // area = sum over the arc segments of |segment x sweep|
        let want: f64 = arc
            .windows(2)
            .map(|w| {
                let e = sub(w[1], w[0]);
                let c = [e[1] * d[2] - e[2] * d[1], e[2] * d[0] - e[0] * d[2], e[0] * d[1] - e[1] * d[0]];
                dot(c, c).sqrt() * 10.0
            })
            .sum();
        assert!((area(&t) - want).abs() < want * 0.01, "{} vs {want}", area(&t));
    }

    #[test]
    fn bspline_patch_inverts_and_meshes() {
        // bilinear patch (degree 1) from z=0 at x=0 to z=2 at x=10, y 0..10
        let c = |x: f64, y: f64, z: f64| [x, y, z, 1.0];
        let b = BSpline {
            udeg: 1,
            vdeg: 1,
            ctrl: vec![vec![c(0.0, 0.0, 0.0), c(0.0, 10.0, 0.0)], vec![c(10.0, 0.0, 2.0), c(10.0, 10.0, 2.0)]],
            ukv: vec![0.0, 0.0, 1.0, 1.0],
            vkv: vec![0.0, 0.0, 1.0, 1.0],
            uclosed: false,
            vclosed: false,
        };
        assert!(b.is_valid());
        let s = Surf::BSpline(b);
        let lp = vec![[0.0, 0.0, 0.0], [10.0, 0.0, 2.0], [10.0, 10.0, 2.0], [0.0, 10.0, 0.0]];
        let t = tessellate(&s, &[lp], true);
        let want = 10.0 * 104f64.sqrt();
        assert!((area(&t) - want).abs() < 1e-6, "{}", area(&t));
    }
}
