//! A STEP model as the stock (a casting, a part from an earlier setup):
//! its top surface on a grid, so roughing only cuts where there is stock and
//! the simulator starts from its shape.

use crate::step::{Model, Placement};
use serde::{Deserialize, Serialize};
use std::sync::Mutex;

type P3 = [f64; 3];

/// The stock model's file and where it goes.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StockModel {
    /// STEP file, in the same coordinates as the part's file (the part's
    /// placement is applied to it too). Empty = a box stock.
    pub path: String,
    /// Moved by this much after the placement (mm).
    pub shift: P3,
}

impl StockModel {
    pub fn is_set(&self) -> bool {
        !self.path.is_empty()
    }
}

/// Most grid cells along the stock's longer side.
const MAX_CELLS: f64 = 400.0;

/// The stock model, placed: its triangles and edges (for drawing and the
/// simulator) and its top surface on a grid.
pub struct StockShape {
    pub min: P3,
    pub max: P3,
    pub tris: Vec<[P3; 3]>,
    pub edges: Vec<Vec<P3>>,
    x0: f64,
    y0: f64,
    pub cell: f64,
    nx: usize,
    ny: usize,
    /// Highest stock over each cell (NEG_INFINITY: none).
    top: Vec<f32>,
    /// Shadows worked out so far, by how many cells they cover (the cells
    /// above Z only grow as Z goes down, so the count says which they are).
    shadows: Mutex<Vec<(usize, geo::MultiPolygon<f64>)>>,
}

impl PartialEq for StockShape {
    fn eq(&self, o: &Self) -> bool {
        self.min == o.min && self.max == o.max && self.cell == o.cell && self.top == o.top
    }
}

impl std::fmt::Debug for StockShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "StockShape({:?}..{:?}, {} triangles)", self.min, self.max, self.tris.len())
    }
}

impl StockShape {
    /// The stock model `raw` (as read from its file) with the part's
    /// placement `pl`, then moved by `shift`.
    pub fn new(raw: &Model, pl: &Placement, shift: P3) -> Result<StockShape, String> {
        let m = raw.transformed(pl);
        let mv = |p: P3| [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]];
        let tris: Vec<[P3; 3]> = m.faces.iter().flat_map(|f| f.tris.iter()).map(|t| [mv(t[0]), mv(t[1]), mv(t[2])]).collect();
        let edges: Vec<Vec<P3>> = m.edges.iter().map(|e| e.pts.iter().map(|&p| mv(p)).collect()).collect();
        StockShape::from_tris(tris, edges)
    }

    pub fn from_tris(tris: Vec<[P3; 3]>, edges: Vec<Vec<P3>>) -> Result<StockShape, String> {
        if tris.is_empty() {
            return Err("the stock model has no surfaces".into());
        }
        let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in tris.iter().flatten() {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        if max[2] - min[2] < 1e-6 {
            return Err("the stock model is flat".into());
        }
        let cell = ((max[0] - min[0]).max(max[1] - min[1]) / MAX_CELLS).max(0.05);
        // a cell of margin all round
        let (x0, y0) = (min[0] - cell, min[1] - cell);
        let nx = ((max[0] - min[0]) / cell).ceil() as usize + 2;
        let ny = ((max[1] - min[1]) / cell).ceil() as usize + 2;
        let top = surface(&tris, x0, y0, cell, nx, ny);
        Ok(StockShape { min, max, tris, edges, x0, y0, cell, nx, ny, top, shadows: Mutex::new(vec![]) })
    }

    /// Highest stock at `p` (None: no stock there).
    #[cfg(test)]
    pub fn top_at(&self, p: [f64; 2]) -> Option<f64> {
        let i = ((p[0] - self.x0) / self.cell).floor();
        let j = ((p[1] - self.y0) / self.cell).floor();
        if i < 0.0 || j < 0.0 || i as usize >= self.nx || j as usize >= self.ny {
            return None;
        }
        let z = self.top[j as usize * self.nx + i as usize];
        (z > f32::NEG_INFINITY).then_some(z as f64)
    }

    /// The stock above `z` seen from above, a cell bigger all round than the
    /// cells it covers (so it holds all of it).
    pub fn shadow(&self, z: f64) -> geo::MultiPolygon<f64> {
        let inside = |i: usize, j: usize| self.top[j * self.nx + i] as f64 > z;
        let count = (0..self.ny).flat_map(|j| (0..self.nx).map(move |i| (i, j))).filter(|&(i, j)| inside(i, j)).count();
        if let Some((_, s)) = self.shadows.lock().unwrap().iter().find(|(n, _)| *n == count) {
            return s.clone();
        }
        // runs of cells along each row; a run carried on unchanged up the
        // rows makes one rectangle
        let mut rects: Vec<[usize; 4]> = vec![]; // i0, i1 (exclusive), j0, j1 (exclusive)
        let mut open: Vec<(usize, usize, usize)> = vec![];
        for j in 0..=self.ny {
            let mut runs = vec![];
            if j < self.ny {
                let mut i = 0;
                while i < self.nx {
                    if inside(i, j) {
                        let s = i;
                        while i < self.nx && inside(i, j) {
                            i += 1;
                        }
                        runs.push((s, i));
                    } else {
                        i += 1;
                    }
                }
            }
            let mut next = vec![];
            for &(a, b, j0) in &open {
                if runs.contains(&(a, b)) {
                    next.push((a, b, j0));
                } else {
                    rects.push([a, b, j0, j]);
                }
            }
            for &(a, b) in &runs {
                if !open.iter().any(|&(c, d, _)| (c, d) == (a, b)) {
                    next.push((a, b, j));
                }
            }
            open = next;
        }
        let c = self.cell;
        let polys: Vec<geo::Polygon<f64>> = rects
            .iter()
            .map(|&[i0, i1, j0, j1]| {
                let (xa, xb) = (self.x0 + (i0 as f64 - 1.0) * c, self.x0 + (i1 as f64 + 1.0) * c);
                let (ya, yb) = (self.y0 + (j0 as f64 - 1.0) * c, self.y0 + (j1 as f64 + 1.0) * c);
                geo::Polygon::new(vec![(xa, ya), (xb, ya), (xb, yb), (xa, yb), (xa, ya)].into(), vec![])
            })
            .collect();
        let s = {
            use geo::Simplify;
            union_tree(polys).simplify(&(c * 0.01))
        };
        self.shadows.lock().unwrap().push((count, s.clone()));
        s
    }
}

/// Union of many polygons by merging pairs.
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

/// Highest point of `tris` over the centre of each cell of the grid at
/// `x0`, `y0` (NEG_INFINITY where there is none). A cell no triangle covers
/// with stock all round it (a sliver between two faces' meshes) takes its
/// neighbours' height.
pub fn surface(tris: &[[P3; 3]], x0: f64, y0: f64, cell: f64, nx: usize, ny: usize) -> Vec<f32> {
    let none = f32::NEG_INFINITY;
    let mut out = vec![none; nx * ny];
    let range = |lo: f64, hi: f64, o: f64, n: usize| -> Option<(usize, usize)> {
        let a = ((lo - o) / cell - 0.5).ceil().max(0.0);
        let b = ((hi - o) / cell - 0.5).floor().min(n as f64 - 1.0);
        (a <= b).then(|| (a as usize, b as usize))
    };
    for t in tris {
        let (a, b, c) = (t[0], t[1], t[2]);
        let den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
        if den.abs() < 1e-12 {
            continue; // vertical
        }
        let (Some((i0, i1)), Some((j0, j1))) = (
            range(a[0].min(b[0]).min(c[0]), a[0].max(b[0]).max(c[0]), x0, nx),
            range(a[1].min(b[1]).min(c[1]), a[1].max(b[1]).max(c[1]), y0, ny),
        ) else {
            continue;
        };
        for j in j0..=j1 {
            for i in i0..=i1 {
                let (x, y) = (x0 + (i as f64 + 0.5) * cell, y0 + (j as f64 + 0.5) * cell);
                let l0 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / den;
                let l1 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den;
                let l2 = 1.0 - l0 - l1;
                if l0 < -1e-9 || l1 < -1e-9 || l2 < -1e-9 {
                    continue;
                }
                let z = (l0 * a[2] + l1 * b[2] + l2 * c[2]) as f32;
                let o = &mut out[j * nx + i];
                if z > *o {
                    *o = z;
                }
            }
        }
    }
    // slivers: an empty cell with stock on both sides along a row or column
    let get = |v: &[f32], i: isize, j: isize| if i < 0 || j < 0 || i as usize >= nx || j as usize >= ny { none } else { v[j as usize * nx + i as usize] };
    let before = out.clone();
    for j in 0..ny as isize {
        for i in 0..nx as isize {
            if get(&before, i, j) > none {
                continue;
            }
            let pairs = [((i - 1, j), (i + 1, j)), ((i, j - 1), (i, j + 1)), ((i - 1, j - 1), (i + 1, j + 1)), ((i - 1, j + 1), (i + 1, j - 1))];
            let fill = pairs
                .iter()
                .filter_map(|&((a, b), (c, d))| {
                    let (p, q) = (get(&before, a, b), get(&before, c, d));
                    (p > none && q > none).then(|| p.max(q))
                })
                .fold(none, f32::max);
            out[j as usize * nx + i as usize] = fill;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed box min..max as triangles (top, bottom and sides).
    pub fn box_tris(a: P3, b: P3) -> Vec<[P3; 3]> {
        let c = |x: usize, y: usize, z: usize| [if x == 0 { a[0] } else { b[0] }, if y == 0 { a[1] } else { b[1] }, if z == 0 { a[2] } else { b[2] }];
        let quad = |p: [P3; 4]| [[p[0], p[1], p[2]], [p[0], p[2], p[3]]];
        let mut t = vec![];
        t.extend(quad([c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)]));
        t.extend(quad([c(0, 0, 0), c(0, 1, 0), c(1, 1, 0), c(1, 0, 0)]));
        t.extend(quad([c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)]));
        t.extend(quad([c(1, 0, 0), c(1, 1, 0), c(1, 1, 1), c(1, 0, 1)]));
        t.extend(quad([c(1, 1, 0), c(0, 1, 0), c(0, 1, 1), c(1, 1, 1)]));
        t.extend(quad([c(0, 1, 0), c(0, 0, 0), c(0, 0, 1), c(0, 1, 1)]));
        t
    }

    #[test]
    fn shadow_holds_the_stock_above_z() {
        use geo::{Area, Contains};
        // a 40 x 20 block 10 high with a 10 x 10 boss 5 high on top
        let mut tris = box_tris([0.0, 0.0, 0.0], [40.0, 20.0, 10.0]);
        tris.extend(box_tris([5.0, 5.0, 10.0], [15.0, 15.0, 15.0]));
        let s = StockShape::from_tris(tris, vec![]).unwrap();
        assert_eq!((s.min, s.max), ([0.0, 0.0, 0.0], [40.0, 20.0, 15.0]));
        assert_eq!(s.top_at([10.0, 10.0]), Some(15.0));
        assert_eq!(s.top_at([30.0, 10.0]), Some(10.0));
        assert_eq!(s.top_at([50.0, 10.0]), None);
        let low = s.shadow(5.0);
        let high = s.shadow(12.0);
        assert!(s.shadow(16.0).0.is_empty());
        for p in [[0.0, 0.0], [40.0, 20.0], [20.0, 10.0]] {
            assert!(low.contains(&geo::Point::new(p[0], p[1])), "{p:?}");
        }
        assert!(high.contains(&geo::Point::new(5.0, 5.0)) && high.contains(&geo::Point::new(15.0, 15.0)));
        assert!(!high.contains(&geo::Point::new(20.0, 10.0)));
        // a cell or two bigger than the block, no more
        let c = s.cell;
        assert!(low.unsigned_area() < (40.0 + 4.0 * c) * (20.0 + 4.0 * c), "{}", low.unsigned_area());
        assert!(high.unsigned_area() < (10.0 + 4.0 * c) * (10.0 + 4.0 * c));
        // the same layer twice comes from the cache
        assert_eq!(s.shadow(4.0), low);
        assert_eq!(s.shadows.lock().unwrap().len(), 3);
    }
}
