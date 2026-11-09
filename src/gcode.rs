//! G-code generation: helical boring followed by bottom-up thread milling.
//!
//! Conventions
//! * Metric, absolute, XY plane (G21 G90 G17), feed per minute (G94).
//! * Arcs use incremental centres (I/J), full circles end where they start.
//! * Right-hand internal threads are milled bottom-up with G03 (climb milling).
//! * Each group has a hole start Z; all hole geometry is below it.
//! * Hole depth = thread length + 1 pitch; the thread starts at
//!   Z = start - length, i.e. one pitch above the bottom of the hole.
//! * Boring descends on a helix whose angle (at the tool-centre path) is
//!   the group's descent angle.

use crate::model::{Cutter, Job, Settings, ThreadGroup, ThreadMill, ThreadSize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    pub error: bool,
    pub msg: String,
}

impl Issue {
    fn err(msg: impl Into<String>) -> Self {
        Issue { error: true, msg: msg.into() }
    }
    fn warn(msg: impl Into<String>) -> Self {
        Issue { error: false, msg: msg.into() }
    }
}

/// Check one group against the rules and the tooling.
pub fn validate_group(g: &ThreadGroup, s: &Settings) -> Vec<Issue> {
    let mut issues = Vec::new();
    let hole = g.thread.hole_dia();
    let depth = g.hole_depth();

    if !(g.ramp_angle > 0.0 && g.ramp_angle <= 45.0) {
        issues.push(Issue::err("descent angle must be > 0 and <= 45 degrees"));
    }
    if s.lookup(g.cutter).is_none() {
        issues.push(Issue::err(format!("cutter T{} is not in the tool table", g.cutter.tool)));
    }
    if !g.start_z.is_finite() {
        issues.push(Issue::err("hole start Z is not a number"));
    } else if s.safe_z < g.start_z + s.clearance {
        issues.push(Issue::err(format!(
            "rapid plane Z {} is below hole start Z + clearance ({})",
            fmt(s.safe_z),
            fmt(g.start_z + s.clearance)
        )));
    }
    if !(g.length > 0.0 && g.length <= 100.0) {
        issues.push(Issue::err("thread length must be > 0 and <= 100 mm"));
    }
    if !g.cutter.fits(g.thread) {
        issues.push(Issue::err(format!(
            "cutter Ø{} is not smaller than hole Ø{}",
            fmt(g.cutter.dia()),
            fmt(hole)
        )));
    }
    if depth > g.cutter.flute_len() {
        issues.push(Issue::err(format!(
            "hole depth {} mm exceeds cutter flute length {} mm",
            fmt(depth),
            fmt(g.cutter.flute_len())
        )));
    }
    let tm = s.thread_mill(g.thread);
    if !(tm.dia > 0.0 && tm.dia < hole) {
        issues.push(Issue::err(format!(
            "{} thread mill Ø{} must be > 0 and smaller than hole Ø{}",
            g.thread.name(),
            fmt(tm.dia),
            fmt(hole)
        )));
    }
    if g.length > tm.reach {
        issues.push(Issue::err(format!(
            "thread length {} mm exceeds {} thread mill reach {} mm",
            fmt(g.length),
            g.thread.name(),
            fmt(tm.reach)
        )));
    }
    match g.parse_locations() {
        Err(e) => issues.push(Issue::err(format!("locations: {e}"))),
        Ok(v) if v.is_empty() => issues.push(Issue::warn("no locations - group will be skipped")),
        Ok(_) => {}
    }
    issues
}

pub fn validate_settings(s: &Settings) -> Vec<Issue> {
    let mut v: Vec<Issue> = s.tool_issues().into_iter().map(Issue::err).collect();
    if !(s.ramp_angle > 0.0 && s.ramp_angle <= 45.0) {
        v.push(Issue::err("milling down angle must be > 0 and <= 45 degrees"));
    }
    for (name, val) in [
        ("clearance", s.clearance),
        ("rapid plane Z", s.safe_z),
        ("spindle speed", s.spindle_rpm),
        ("milling feed", s.bore_feed),
        ("plunge feed", s.plunge_feed),
        ("bore stepover", s.bore_stepover_pct),
        ("thread feed", s.thread_feed),
    ] {
        if !(val > 0.0) {
            v.push(Issue::err(format!("{name} must be > 0")));
        }
    }
    if s.bore_stepover_pct > 90.0 {
        v.push(Issue::err("bore stepover must be <= 90 % of cutter diameter"));
    }
    v
}

/// Format a coordinate: 4 decimals, trailing zeros trimmed, never "-0".
pub fn fmt(v: f64) -> String {
    let mut s = format!("{:.4}", v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}

/// Radii (tool-centre path) of the concentric helical rings needed to bore
/// a hole of `hole_dia` with a cutter of `tool_dia`. The last ring is the
/// finished wall. If the cutter is less than half the hole diameter a single
/// ring would leave a core, so inner rings are added.
pub fn bore_rings(hole_dia: f64, tool_dia: f64, stepover_pct: f64) -> Vec<f64> {
    let r = tool_dia / 2.0;
    let outer = hole_dia / 2.0 - r;
    if outer <= r {
        return vec![outer];
    }
    let first = r * 0.9;
    let step = (stepover_pct / 100.0 * tool_dia).max(0.01);
    let n = ((outer - first) / step).ceil().max(1.0) as usize;
    (0..=n)
        .map(|k| first + (outer - first) * k as f64 / n as f64)
        .collect()
}

struct Writer {
    out: String,
    current_tool: Option<u32>,
}

impl Writer {
    fn line(&mut self, s: impl AsRef<str>) {
        // keep the program plain ASCII for controller compatibility
        self.out.push_str(&s.as_ref().replace('Ø', "D"));
        self.out.push('\n');
    }

    fn tool_change(&mut self, s: &Settings, tool: u32, desc: &str, rpm: f64) {
        if self.current_tool == Some(tool) {
            return;
        }
        self.line("");
        if self.current_tool.is_some() {
            self.line("M5");
            self.line("M9");
        }
        self.line(format!("G0 Z{}", fmt(s.safe_z)));
        self.line(format!("T{tool} M6 ({desc})"));
        if s.tool_length_comp {
            self.line(format!("G43 H{tool}"));
        }
        self.line(format!("S{} M3", rpm.round() as i64));
        if s.coolant {
            self.line("M8");
        }
        self.line(format!("G0 Z{}", fmt(s.safe_z)));
        self.current_tool = Some(tool);
    }
}

/// Helically bore one hole at (cx, cy).
fn bore_hole(w: &mut Writer, s: &Settings, g: &ThreadGroup, cx: f64, cy: f64) {
    let tool = g.cutter.dia();
    let depth = g.hole_depth();
    let top = g.start_z;
    let clear = top + s.clearance;

    w.line(format!("(bore {} hole X{} Y{} Ø{} from Z{} depth {} at {} deg)", g.thread.name(), fmt(cx), fmt(cy), fmt(g.thread.hole_dia()), fmt(top), fmt(depth), fmt(g.ramp_angle)));
    for pr in bore_rings(g.thread.hole_dia(), tool, s.bore_stepover_pct) {
        let turns = helix_turns(depth, pr, g.ramp_angle);
        let sx = cx + pr;
        w.line(format!("G0 X{} Y{}", fmt(sx), fmt(cy)));
        w.line(format!("G0 Z{}", fmt(clear)));
        w.line(format!("G1 Z{} F{}", fmt(top), fmt(s.plunge_feed)));
        for i in 1..=turns {
            let z = top - depth * i as f64 / turns as f64;
            let feed = if i == 1 { format!(" F{}", fmt(s.bore_feed)) } else { String::new() };
            w.line(format!("G3 X{} Y{} Z{} I{} J0{}", fmt(sx), fmt(cy), fmt(z), fmt(-pr), feed));
        }
        // flat clean-up revolution at the bottom
        w.line(format!("G3 X{} Y{} I{} J0", fmt(sx), fmt(cy), fmt(-pr)));
        w.line(format!("G1 X{} Y{}", fmt(cx), fmt(cy)));
        w.line(format!("G0 Z{}", fmt(clear)));
    }
}

/// Number of full helix revolutions needed to descend `depth` on a circle of
/// radius `pr` without exceeding `angle_deg` (measured at the tool centre path).
pub fn helix_turns(depth: f64, pr: f64, angle_deg: f64) -> usize {
    let per_rev = 2.0 * std::f64::consts::PI * pr * angle_deg.to_radians().tan();
    if per_rev <= 1e-6 {
        return 1;
    }
    ((depth / per_rev) - 1e-9).ceil().max(1.0) as usize
}

/// Thread-mill one hole at (cx, cy), from the bottom up.
fn thread_hole(w: &mut Writer, s: &Settings, g: &ThreadGroup, tm: &ThreadMill, cx: f64, cy: f64) {
    let pitch = g.thread.pitch();
    let major_r = g.thread.major_dia() / 2.0;
    let pr = major_r - tm.dia / 2.0; // tool-centre path radius
    let z_start = g.start_z - g.length; // one pitch above the hole bottom
    let clear = g.start_z + s.clearance;
    let turns = (g.length / pitch).ceil().max(1.0) as usize;
    // programmed feed is at the tool centre: scale down from cutting-edge feed
    let feed = (s.thread_feed * pr / major_r).max(1.0);

    w.line(format!("(thread {}x{} X{} Y{} length {})", g.thread.name(), fmt(pitch), fmt(cx), fmt(cy), fmt(g.length)));
    w.line(format!("G0 X{} Y{}", fmt(cx), fmt(cy)));
    w.line(format!("G0 Z{}", fmt(clear)));
    w.line(format!("G1 Z{} F{}", fmt(z_start), fmt(s.plunge_feed)));
    // tangential 180 deg lead-in arc from the centre to the wall
    w.line(format!("G3 X{} Y{} I{} J0 F{}", fmt(cx + pr), fmt(cy), fmt(pr / 2.0), fmt(feed)));
    for i in 1..=turns {
        let z = z_start + pitch * i as f64;
        w.line(format!("G3 X{} Y{} Z{} I{} J0", fmt(cx + pr), fmt(cy), fmt(z), fmt(-pr)));
    }
    // tangential lead-out back to the centre
    w.line(format!("G3 X{} Y{} I{} J0", fmt(cx), fmt(cy), fmt(-pr / 2.0)));
    w.line(format!("G0 Z{}", fmt(clear)));
}

/// Helical boring of every location (no tool change), for other programs.
pub fn bore_code(s: &Settings, g: &ThreadGroup, locs: &[(f64, f64)]) -> String {
    let mut w = Writer { out: String::new(), current_tool: None };
    for &(x, y) in locs {
        bore_hole(&mut w, s, g, x, y);
    }
    w.out
}

/// Thread milling of every location (no tool change), for other programs.
pub fn thread_code(s: &Settings, g: &ThreadGroup, locs: &[(f64, f64)]) -> String {
    let mut w = Writer { out: String::new(), current_tool: None };
    let tm = s.thread_mill(g.thread);
    for &(x, y) in locs {
        thread_hole(&mut w, s, g, &tm, x, y);
    }
    w.out
}

/// Generate the full program. Returns the list of error messages if the job
/// is invalid. Boring is done first (grouped by cutter), then threading
/// (grouped by thread size), to minimise tool changes.
pub fn generate(job: &Job) -> Result<String, Vec<String>> {
    let s = &job.settings;
    let mut errors: Vec<String> = validate_settings(s)
        .into_iter()
        .filter(|i| i.error)
        .map(|i| format!("settings: {}", i.msg))
        .collect();

    let mut work: Vec<(usize, &ThreadGroup, Vec<(f64, f64)>)> = Vec::new();
    for (i, g) in job.groups.iter().enumerate() {
        for issue in validate_group(g, s).into_iter().filter(|i| i.error) {
            errors.push(format!("group {}: {}", i + 1, issue.msg));
        }
        if let Ok(locs) = g.parse_locations() {
            if !locs.is_empty() {
                work.push((i, g, locs));
            }
        }
    }
    if work.is_empty() && errors.is_empty() {
        errors.push("no groups with locations to machine".into());
    }
    if !errors.is_empty() {
        return Err(errors);
    }

    let mut w = Writer { out: String::new(), current_tool: None };
    w.line("%");
    w.line("(stepCAM - helical bore + thread mill)");
    w.line("(units mm)");
    // tool list
    let mut cutters: BTreeMap<u32, Cutter> = BTreeMap::new();
    let mut mills: BTreeMap<u32, ThreadSize> = BTreeMap::new();
    for (_, g, _) in &work {
        cutters.insert(g.cutter.tool_number(), g.cutter);
        mills.insert(s.thread_mill(g.thread).tool, g.thread);
    }
    for (t, c) in &cutters {
        w.line(format!("(T{t} endmill Ø{} flute {})", fmt(c.dia()), fmt(c.flute_len())));
    }
    for (t, th) in &mills {
        let tm = s.thread_mill(*th);
        w.line(format!("(T{t} thread mill {} Ø{} reach {})", th.name(), fmt(tm.dia), fmt(tm.reach)));
    }
    for (i, g, locs) in &work {
        w.line(format!(
            "(group {}: {} start Z{} length {} hole Ø{} depth {} cutter {} - {} holes)",
            i + 1,
            g.thread.name(),
            fmt(g.start_z),
            fmt(g.length),
            fmt(g.thread.hole_dia()),
            fmt(g.hole_depth()),
            g.cutter.name(),
            locs.len()
        ));
    }
    w.line("G21 G90 G17 G94 G40 G49 G80");
    w.line(format!("G0 Z{}", fmt(s.safe_z)));

    // --- phase 1: helical boring, grouped by cutter
    let mut bore_order: Vec<&(usize, &ThreadGroup, Vec<(f64, f64)>)> = work.iter().collect();
    bore_order.sort_by_key(|(i, g, _)| (g.cutter.tool_number(), *i));
    for (i, g, locs) in bore_order {
        let t = g.cutter.tool_number();
        w.tool_change(s, t, &format!("endmill Ø{}", fmt(g.cutter.dia())), s.spindle_rpm);
        w.line(format!("(group {} - bore)", i + 1));
        for &(x, y) in locs {
            bore_hole(&mut w, s, g, x, y);
        }
        w.line(format!("G0 Z{}", fmt(s.safe_z)));
    }

    // --- phase 2: thread milling, grouped by thread size
    let mut thread_order: Vec<&(usize, &ThreadGroup, Vec<(f64, f64)>)> = work.iter().collect();
    thread_order.sort_by_key(|(i, g, _)| (s.thread_mill(g.thread).tool, *i));
    for (i, g, locs) in thread_order {
        let t = s.thread_mill(g.thread).tool;
        let tm = s.thread_mill(g.thread);
        w.tool_change(s, t, &format!("thread mill {} Ø{}", g.thread.name(), fmt(tm.dia)), s.spindle_rpm);
        w.line(format!("(group {} - thread)", i + 1));
        for &(x, y) in locs {
            thread_hole(&mut w, s, g, &tm, x, y);
        }
        w.line(format!("G0 Z{}", fmt(s.safe_z)));
    }

    w.line("");
    w.line("M5");
    w.line("M9");
    w.line(format!("G0 Z{}", fmt(s.safe_z)));
    w.line("M30");
    w.line("%");
    Ok(w.out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn job_m3() -> Job {
        let mut job = Job::default();
        job.groups.push(ThreadGroup {
            thread: ThreadSize::M3,
            start_z: 0.0,
            ramp_angle: 3.0,
            length: 5.0,
            cutter: Cutter::C2x12,
            locations: "10, 20\n-5 7.5\n".into(),
        });
        job
    }

    fn z_values(code: &str) -> Vec<f64> {
        code.lines()
            .filter(|l| !l.starts_with('('))
            .flat_map(|l| l.split_whitespace())
            .filter_map(|w| w.strip_prefix('Z'))
            .filter_map(|v| v.parse().ok())
            .collect()
    }

    #[test]
    fn fmt_trims() {
        assert_eq!(fmt(1.5), "1.5");
        assert_eq!(fmt(2.0), "2");
        assert_eq!(fmt(-0.00001), "0");
        assert_eq!(fmt(0.12345), "0.1235");
    }

    #[test]
    fn hole_depth_is_length_plus_pitch() {
        let g = &job_m3().groups[0];
        assert!((g.hole_depth() - 5.5).abs() < 1e-9);
    }

    #[test]
    fn parses_locations() {
        let mut g = ThreadGroup::default();
        g.locations = "X1 Y2\n3,4\n\n5;6 # comment\n".into();
        assert_eq!(g.parse_locations().unwrap(), vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]);
        g.locations = "1,2,3".into();
        assert!(g.parse_locations().is_err());
    }

    #[test]
    fn cutter_must_be_smaller_than_hole() {
        assert!(!Cutter::C3x17.fits(ThreadSize::M3)); // 3 >= 2.5
        assert!(Cutter::C2x12.fits(ThreadSize::M3));
        assert!(!Cutter::C2x12.fits(ThreadSize::M2));
        for t in ThreadSize::ALL {
            assert!(Cutter::default_for(&Cutter::DEFAULTS, t).fits(t));
            assert!(ThreadMill::default_for(t).dia < t.hole_dia());
        }
    }

    #[test]
    fn rings_clear_core() {
        // big cutter: single ring
        assert_eq!(bore_rings(2.5, 2.0, 40.0).len(), 1);
        // small cutter in big hole: several rings, first one covers centre,
        // gaps never exceed the cutter diameter, last one is the wall
        let r = bore_rings(5.0, 1.0, 40.0);
        assert!(r.len() > 1);
        assert!(r[0] < 0.5);
        assert!((r.last().unwrap() - 2.0).abs() < 1e-9);
        for w in r.windows(2) {
            assert!(w[1] - w[0] <= 1.0);
        }
    }

    #[test]
    fn generates_program_within_depth_limits() {
        let job = job_m3();
        let code = generate(&job).expect("valid job");
        assert!(code.contains("T3 M6"));
        assert!(code.contains("T13 M6"));
        // bore first, then thread
        assert!(code.find("T3 M6").unwrap() < code.find("T13 M6").unwrap());
        let zs = z_values(&code);
        let min = zs.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((min - -5.5).abs() < 1e-6, "deepest Z should be hole bottom, got {min}");

        // thread section never goes below one pitch above the bottom
        let thread_part = &code[code.find("T13 M6").unwrap()..];
        let tz = z_values(thread_part);
        let tmin = tz.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((tmin - -5.0).abs() < 1e-6, "thread should start at -5, got {tmin}");
        // and climbs to the top of the hole
        assert!(thread_part.contains("Z0\n") || thread_part.contains(" Z0 "));
    }

    #[test]
    fn start_z_offsets_all_geometry() {
        let mut job = job_m3();
        job.groups[0].start_z = -3.0;
        let code = generate(&job).unwrap();
        let cut: Vec<f64> = z_values(&code).into_iter().filter(|&z| z < 0.0).collect();
        let min = cut.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((min - -8.5).abs() < 1e-6, "bottom should be -3 - 5.5, got {min}");
        // nothing feeds above the start Z except rapids to clearance/safe
        assert!(code.contains("G1 Z-3 F"));
        let thread_part = &code[code.find("T13 M6").unwrap()..];
        assert!(thread_part.contains("G1 Z-8 F"));
        assert!(thread_part.contains("Z-3 I"));
    }

    #[test]
    fn descent_angle_is_respected() {
        // M3 with 2mm cutter: path radius 0.25, 3 deg -> 0.0823 mm/rev
        let n = helix_turns(5.5, 0.25, 3.0);
        let per_rev = 5.5 / n as f64;
        let angle = (per_rev / (2.0 * std::f64::consts::PI * 0.25)).atan().to_degrees();
        assert!(angle <= 3.0 + 1e-9 && angle > 2.9, "angle {angle}");
    }

    #[test]
    fn rejects_bad_jobs() {
        let mut job = job_m3();
        job.groups[0].cutter = Cutter::C3x17; // too big for M3 hole
        let errs = generate(&job).unwrap_err();
        assert!(errs.iter().any(|e| e.contains("not smaller")));

        let mut job = job_m3();
        job.groups[0].cutter = Cutter::C1x3; // flute 3 < depth 5.5
        assert!(generate(&job).is_err());

        let mut job = job_m3();
        job.groups[0].locations.clear();
        assert!(generate(&job).is_err());

        let mut job = job_m3();
        job.groups[0].start_z = 20.0; // above safe Z
        assert!(generate(&job).is_err());
    }
}
