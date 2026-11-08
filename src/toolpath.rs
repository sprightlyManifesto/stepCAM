//! Parse G-code back into 3D polylines for visualisation.
//!
//! Understands the subset stepCAM emits: G0/G1/G2/G3 (modal) with X Y Z I J,
//! helical arcs (Z on G2/G3), full circles, and T.. M6 tool changes.
//! Tools T11+ are thread mills; everything else is a hole-milling cutter.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveKind {
    Rapid,
    Bore,
    Thread,
}

#[derive(Clone, Debug)]
pub struct Path {
    pub kind: MoveKind,
    pub pts: Vec<[f32; 3]>,
}

#[derive(Clone, Debug, Default)]
pub struct Toolpath {
    pub paths: Vec<Path>,
    pub min: [f32; 3],
    pub max: [f32; 3],
    pub moves: usize,
    /// Polyline drawn for each source line (index = 0-based line number);
    /// empty for lines without a drawable move.
    pub by_line: Vec<Vec<[f32; 3]>>,
}

impl Toolpath {
    pub fn is_empty(&self) -> bool {
        self.paths.is_empty()
    }

    /// The move produced by the given 0-based source line, if any.
    pub fn line_move(&self, line: usize) -> Option<&[[f32; 3]]> {
        self.by_line.get(line).map(|v| v.as_slice()).filter(|v| v.len() >= 2)
    }

    fn push(&mut self, kind: MoveKind, from: [f64; 3], to: [f64; 3]) {
        let f = |p: [f64; 3]| [p[0] as f32, p[1] as f32, p[2] as f32];
        match self.paths.last_mut() {
            Some(last) if last.kind == kind && last.pts.last() == Some(&f(from)) => last.pts.push(f(to)),
            _ => self.paths.push(Path { kind, pts: vec![f(from), f(to)] }),
        }
    }

    fn finish(&mut self) {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for p in self.paths.iter().flat_map(|p| p.pts.iter()) {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        }
        if min[0].is_finite() {
            self.min = min;
            self.max = max;
        }
    }
}

/// Max angular step when tessellating arcs.
const ARC_STEP: f64 = std::f64::consts::PI / 12.0; // 15 degrees

pub fn parse(code: &str) -> Toolpath {
    let mut tp = Toolpath::default();
    let mut pos: [Option<f64>; 3] = [None, None, None];
    let mut motion = 0u8; // 0..=3
    let mut tool: u32 = 0;
    let mut pending_tool: u32 = 0;

    for (ln, raw) in code.lines().enumerate() {
        tp.by_line.push(Vec::new());
        // strip ( comments ) and ; comments
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

        let mut target: [Option<f64>; 3] = [None, None, None];
        let (mut i, mut j) = (0.0, 0.0);
        let mut tool_change = false;

        for word in line.split_whitespace() {
            let mut chars = word.chars();
            let Some(letter) = chars.next() else { continue };
            let Ok(val) = chars.as_str().parse::<f64>() else { continue };
            match letter.to_ascii_uppercase() {
                'G' => {
                    let g = val.round() as i64;
                    if (0..=3).contains(&g) && (val - g as f64).abs() < 1e-6 {
                        motion = g as u8;
                    }
                }
                'X' => target[0] = Some(val),
                'Y' => target[1] = Some(val),
                'Z' => target[2] = Some(val),
                'I' => i = val,
                'J' => j = val,
                'T' => pending_tool = val as u32,
                'M' if val as i64 == 6 => tool_change = true,
                _ => {}
            }
        }
        if tool_change {
            tool = pending_tool;
        }
        if target.iter().all(|t| t.is_none()) {
            continue;
        }

        let end = [
            target[0].or(pos[0]),
            target[1].or(pos[1]),
            target[2].or(pos[2]),
        ];
        // only draw once the start position is fully known
        if let (Some(sx), Some(sy), Some(sz)) = (pos[0], pos[1], pos[2]) {
            let start = [sx, sy, sz];
            let e = [end[0].unwrap(), end[1].unwrap(), end[2].unwrap()];
            let kind = match motion {
                0 => MoveKind::Rapid,
                _ if tool >= 11 => MoveKind::Thread,
                _ => MoveKind::Bore,
            };
            tp.moves += 1;
            let f = |p: [f64; 3]| [p[0] as f32, p[1] as f32, p[2] as f32];
            let mut poly = vec![f(start)];
            match motion {
                2 | 3 => {
                    let c = [start[0] + i, start[1] + j];
                    let r0 = ((start[0] - c[0]).powi(2) + (start[1] - c[1]).powi(2)).sqrt();
                    let r1 = ((e[0] - c[0]).powi(2) + (e[1] - c[1]).powi(2)).sqrt();
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
                    let n = (sweep.abs() / ARC_STEP).ceil().max(1.0) as usize;
                    let mut prev = start;
                    for k in 1..=n {
                        let t = k as f64 / n as f64;
                        let p = if k == n {
                            e
                        } else {
                            let a = a0 + sweep * t;
                            let r = r0 + (r1 - r0) * t;
                            [c[0] + r * a.cos(), c[1] + r * a.sin(), start[2] + (e[2] - start[2]) * t]
                        };
                        tp.push(kind, prev, p);
                        poly.push(f(p));
                        prev = p;
                    }
                }
                _ => {
                    tp.push(kind, start, e);
                    poly.push(f(e));
                }
            }
            tp.by_line[ln] = poly;
        }
        pos = end;
    }
    tp.finish();
    tp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_lines_and_helix() {
        let code = "G0 Z10\nG0 X1 Y0\nT11 M6\nG1 Z0 F100\nG3 X1 Y0 Z-1 I-1 J0\n";
        let tp = parse(code);
        // first G0 Z10 has unknown XY -> skipped; G0 X1 Y0 has unknown start -> skipped; feed + helix
        assert_eq!(tp.moves, 2);
        let thread: Vec<_> = tp.paths.iter().filter(|p| p.kind == MoveKind::Thread).collect();
        assert_eq!(thread.len(), 1);
        let pts = &thread[0].pts;
        // full circle: 24 segments + start point of the G1 that shares the path
        assert!(pts.len() >= 25);
        let last = pts.last().unwrap();
        assert!((last[2] + 1.0).abs() < 1e-6);
        // radius stays 1 around the centre (0,0)
        for p in pts.iter().skip(2) {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!((r - 1.0).abs() < 1e-4);
        }
        assert!((tp.min[2] + 1.0).abs() < 1e-6 && (tp.max[2] - 10.0).abs() < 1e-6);
    }

    #[test]
    fn ignores_comments() {
        let tp = parse("G0 X0 Y0 Z0 (X99 Y99)\nG1 X1 ; Y50\n");
        assert_eq!(tp.moves, 1);
        assert_eq!(tp.paths[0].pts, vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]);
    }

    #[test]
    fn maps_lines_to_moves() {
        let tp = parse("(start)\nG0 X0 Y0 Z0\nG1 X2 F100\nG3 X0 Y0 I-1 J0\nM5\n");
        assert!(tp.line_move(0).is_none());
        assert!(tp.line_move(1).is_none()); // start unknown
        assert_eq!(tp.line_move(2).unwrap(), &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
        let arc = tp.line_move(3).unwrap();
        assert!(arc.len() > 2 && arc[0] == [2.0, 0.0, 0.0]);
        assert!(tp.line_move(4).is_none() && tp.line_move(99).is_none());
    }
}
