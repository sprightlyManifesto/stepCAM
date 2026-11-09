//! Run-time estimate for a G-code program.
//!
//! Every move accelerates and decelerates at `accel` (mm/s², along the path)
//! up to its feed (G1/G2/G3, the modal F) or the rapid rate (G0). Moves are
//! joined with a junction-deviation corner limit like common controllers
//! (GRBL style), so near-tangent moves keep their speed and sharp corners
//! slow right down. The machine is at rest at the start, at the end and at
//! every tool change. Tool change and spindle spin-up times are not counted.

/// Junction deviation (mm): how far a corner may be rounded at speed.
const JUNCTION_DEVIATION: f64 = 0.01;
/// Arcs are split into chords of at most this angle.
const ARC_STEP: f64 = std::f64::consts::PI / 36.0; // 5 degrees

struct Seg {
    len: f64,
    dir: [f64; 3],
    /// Speed limit (mm/s).
    vmax: f64,
    /// Must stop before this segment (tool change).
    stop_before: bool,
}

/// Estimated run time in seconds. `rapid` and feeds in mm/min, `accel` in mm/s².
pub fn estimate(code: &str, rapid: f64, accel: f64) -> f64 {
    plan_time(&segments(code, rapid), accel.max(1e-3))
}

/// The program as straight segments with their speed limits.
fn segments(code: &str, rapid: f64) -> Vec<Seg> {
    let rapid = rapid.max(1e-3) / 60.0;
    let mut segs: Vec<Seg> = vec![];
    let mut pos: [Option<f64>; 3] = [None; 3];
    let mut motion = 0u8;
    let mut feed = 0.0f64; // mm/s
    let mut stop = false;

    for raw in code.lines() {
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
        for word in line.split_whitespace() {
            let mut ch = word.chars();
            let Some(letter) = ch.next() else { continue };
            let Ok(val) = ch.as_str().parse::<f64>() else { continue };
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
                'F' => feed = val / 60.0,
                'M' if val.round() as i64 == 6 => stop = true,
                _ => {}
            }
        }
        if target.iter().all(|t| t.is_none()) {
            continue;
        }
        let end = [target[0].or(pos[0]), target[1].or(pos[1]), target[2].or(pos[2])];
        if let (Some(sx), Some(sy), Some(sz)) = (pos[0], pos[1], pos[2]) {
            let start = [sx, sy, sz];
            // unknown axes (first moves) count as not moving
            let e = [end[0].unwrap_or(sx), end[1].unwrap_or(sy), end[2].unwrap_or(sz)];
            // feed moves before any F (not valid G-code) are timed at the rapid rate
            let vmax = if motion == 0 || feed <= 0.0 { rapid } else { feed };
            let mut pts = vec![];
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
                let n = (sweep.abs() / ARC_STEP).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let t = k as f64 / n as f64;
                    let a = a0 + sweep * t;
                    let r = r0 + (r1 - r0) * t;
                    pts.push(if k == n { e } else { [c[0] + r * a.cos(), c[1] + r * a.sin(), start[2] + (e[2] - start[2]) * t] });
                }
            } else {
                pts.push(e);
            }
            let mut prev = start;
            for p in pts {
                let d = [p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]];
                let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                if len > 1e-9 {
                    segs.push(Seg { len, dir: [d[0] / len, d[1] / len, d[2] / len], vmax: vmax.max(1e-6), stop_before: stop });
                    stop = false;
                }
                prev = p;
            }
        }
        pos = end;
    }
    segs
}

/// Time to run the segments with entry speeds planned forwards and backwards.
fn plan_time(segs: &[Seg], a: f64) -> f64 {
    let n = segs.len();
    if n == 0 {
        return 0.0;
    }
    // max entry speed of each segment from the corner with the one before
    let mut vin = vec![0.0f64; n + 1]; // vin[n] = speed at the end = 0
    for k in 1..n {
        let (p, s) = (&segs[k - 1], &segs[k]);
        if s.stop_before {
            continue;
        }
        let c = p.dir[0] * s.dir[0] + p.dir[1] * s.dir[1] + p.dir[2] * s.dir[2];
        let lim = p.vmax.min(s.vmax);
        let sin_half = (0.5 * (1.0 - c)).max(0.0).sqrt(); // sin of half the turn angle
        vin[k] = if sin_half < 1e-6 {
            lim
        } else if sin_half > 1.0 - 1e-6 {
            0.0
        } else {
            // GRBL: v^2 = a * dev * sin(theta/2) / (1 - sin(theta/2)), theta = angle between the reversed
            // incoming and the outgoing direction, so sin(theta/2) = cos(turn/2)
            let s2 = (1.0 - sin_half * sin_half).sqrt();
            (a * JUNCTION_DEVIATION * s2 / (1.0 - s2)).sqrt().min(lim)
        };
    }
    // backward pass: room to slow down for the next entry
    for k in (0..n).rev() {
        vin[k] = vin[k].min((vin[k + 1] * vin[k + 1] + 2.0 * a * segs[k].len).sqrt());
    }
    // forward pass: room to speed up from this entry
    for k in 0..n {
        vin[k + 1] = vin[k + 1].min((vin[k] * vin[k] + 2.0 * a * segs[k].len).sqrt());
    }
    segs.iter().enumerate().map(|(k, s)| seg_time(s.len, vin[k], vin[k + 1], s.vmax, a)).sum()
}

/// Trapezoid (or triangle) time over `len` from speed v0 to v1, cruising at vm.
fn seg_time(len: f64, v0: f64, v1: f64, vm: f64, a: f64) -> f64 {
    let (v0, v1) = (v0.min(vm), v1.min(vm));
    let d_acc = (vm * vm - v0 * v0) / (2.0 * a);
    let d_dec = (vm * vm - v1 * v1) / (2.0 * a);
    if d_acc + d_dec <= len {
        (vm - v0) / a + (vm - v1) / a + (len - d_acc - d_dec) / vm
    } else {
        let vp = ((2.0 * a * len + v0 * v0 + v1 * v1) / 2.0).sqrt().max(v0.max(v1));
        (vp - v0) / a + (vp - v1) / a
    }
}

/// "1 h 02 min", "12 min 05 s", "45 s".
pub fn format_time(secs: f64) -> String {
    let s = secs.max(0.0).round() as u64;
    let (h, m, s) = (s / 3600, (s / 60) % 60, s % 60);
    if h > 0 {
        format!("{h} h {m:02} min")
    } else if m > 0 {
        format!("{m} min {s:02} s")
    } else {
        format!("{s} s")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_move_trapezoid() {
        // 100 mm at 600 mm/min = 10 mm/s, a = 200: accel 0.05 s over 0.25 mm each end
        let t = estimate("G0 X0 Y0 Z0\nG1 X100 F600\n", 6000.0, 200.0);
        let want = 0.05 + 0.05 + (100.0 - 0.5) / 10.0;
        assert!((t - want).abs() < 1e-9, "{t} vs {want}");
    }

    #[test]
    fn short_move_never_reaches_feed() {
        // 1 mm at 6000 mm/min: triangle, peak sqrt(a * len) = sqrt(200) mm/s
        let t = estimate("G0 X0 Y0 Z0\nG1 X1 F6000\n", 6000.0, 200.0);
        let want = 2.0 * (200f64).sqrt() / 200.0;
        assert!((t - want).abs() < 1e-9, "{t} vs {want}");
    }

    #[test]
    fn corners_slow_down_but_straight_joins_do_not() {
        let line = estimate("G0 X0 Y0 Z0\nG1 X50 F3000\nG1 X100\n", 6000.0, 200.0);
        let one = estimate("G0 X0 Y0 Z0\nG1 X100 F3000\n", 6000.0, 200.0);
        assert!((line - one).abs() < 1e-9, "split straight line {line} vs {one}");
        let corner = estimate("G0 X0 Y0 Z0\nG1 X50 F3000\nG1 Y50\n", 6000.0, 200.0);
        let reverse = estimate("G0 X0 Y0 Z0\nG1 X50 F3000\nG1 X0\n", 6000.0, 200.0);
        assert!(corner > one && reverse > corner, "{one} {corner} {reverse}");
        // a 90 degree corner is nearly a full stop: close to two separate moves
        let two = 2.0 * estimate("G0 X0 Y0 Z0\nG1 X50 F3000\n", 6000.0, 200.0);
        assert!(corner < two && corner > two - 0.1, "{corner} vs {two}");
    }

    #[test]
    fn real_program_is_between_instant_and_stop_at_every_move() {
        let mut job = crate::model::Job::default();
        job.groups.push(crate::model::ThreadGroup { locations: "10, 10\n20, 10\n30, 25\n".into(), ..Default::default() });
        let code = crate::gcode::generate(&job).unwrap();
        let (rapid, a) = (3000.0, 200.0);
        let segs = segments(&code, rapid);
        let lo: f64 = segs.iter().map(|s| s.len / s.vmax).sum();
        let hi: f64 = segs.iter().map(|s| seg_time(s.len, 0.0, 0.0, s.vmax, a)).sum();
        let t = estimate(&code, rapid, a);
        assert!(lo > 1.0 && t >= lo - 1e-9 && t <= hi + 1e-9, "{lo} <= {t} <= {hi}");
        // slower acceleration takes longer
        assert!(estimate(&code, rapid, 50.0) > t);
    }

    #[test]
    fn rapids_tool_changes_and_arcs() {
        let rapid = estimate("G0 X0 Y0 Z0\nG0 X100\n", 3000.0, 200.0);
        // 50 mm/s: 0.25 s and 6.25 mm to speed up, the same to stop
        assert!((rapid - (0.25 + 0.25 + 87.5 / 50.0)).abs() < 1e-9, "{rapid}");
        // full circle r 10 at 600 mm/min: about 2 pi r / 10 mm/s plus a little accel
        let circle = estimate("G0 X10 Y0 Z0\nG3 X10 Y0 I-10 J0 F600\n", 3000.0, 200.0);
        let len = std::f64::consts::TAU * 10.0;
        assert!(circle > len / 10.0 && circle < len / 10.0 + 0.2, "{circle}");
        // a tool change between two moves forces a stop
        let joined = estimate("G0 X0 Y0 Z0\nG1 X50 F3000\nG1 X100\n", 6000.0, 200.0);
        let stopped = estimate("G0 X0 Y0 Z0\nG1 X50 F3000\nT2 M6\nG1 X100\n", 6000.0, 200.0);
        assert!(stopped > joined + 0.2);
        assert_eq!(format_time(3725.0), "1 h 02 min");
        assert_eq!(format_time(725.4), "12 min 05 s");
        assert_eq!(format_time(45.0), "45 s");
    }
}
