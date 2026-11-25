//! Headless batch mode: `stepcam batch spec.json`.
//!
//! Builds a step milling job from a STEP file and a list of operations
//! whose faces / edges are given as points on the model (what the CadQuery
//! module in `python/stepcam.py` sends). It makes each operation exactly
//! as the Step Milling tab's right-click menu does, applies parameters by
//! field name, writes the job (`.stepcam.json`, opens in the app) and the
//! G-code, optionally simulates it, and prints a JSON report on stdout.
//!
//! Spec (JSON):
//! ```text
//! { "step": "part.step",
//!   "stock": {"x": 4, "y": 4, "z": 2, "offset": true},      // optional
//!   "placement": {"rot": [[..],[..],[..]], "offset": [..]},  // optional
//!   "rapid_above": 10,                                       // optional
//!   "origin": {"xy": "bottom_left", "z": "stock_top"},       // optional program zero:
//!             // xy: model | center | bottom_left | bottom_right | top_left | top_right | custom
//!             // z: model | stock_top | stock_base | custom; "at": [x, y, z] for custom axes
//!   "settings_file": "stepcam-settings.json",                // optional
//!   "settings": {"spindle_rpm": 12000, ...},                 // optional overrides
//!   "ops": [ {"type": "rough" | "finish" | "face" | "pocket" | "profile" | "engrave" |
//!                     "thread" | "thread_alike" | "tab" | "omit",   // omit: no params
//!             "picks": [{"point": [x,y,z], "normal": [..]?, "tangent": [..]?}, ...],
//!             "on": "face" | "edge",                         // pocket only
//!             "enabled": true,
//!             "params": {"tool": 4, "stepover_pct": 40, ...}} ],
//!   "job_out": "part.stepcam.json", "gcode_out": "part.nc",  // optional
//!   "simulate": {"detail": 300} }                            // optional
//! ```

use crate::model::{Job, Settings, ThreadSize};
use crate::stepmill::{OpKind, StepJob, StepOp, Stock};
use crate::{estimate, gcode, sim, step, stepmill};
use serde::Deserialize;
use serde_json::{json, Map, Value};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Spec {
    step: String,
    #[serde(default)]
    stock: Option<Stock>,
    #[serde(default)]
    placement: Option<step::Placement>,
    #[serde(default)]
    rapid_above: Option<f64>,
    #[serde(default)]
    origin: Option<stepmill::Origin>,
    #[serde(default)]
    settings_file: Option<String>,
    #[serde(default)]
    settings: Map<String, Value>,
    #[serde(default)]
    ops: Vec<OpSpec>,
    #[serde(default)]
    job_out: Option<String>,
    #[serde(default)]
    gcode_out: Option<String>,
    #[serde(default)]
    simulate: Option<SimSpec>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct OpSpec {
    #[serde(rename = "type")]
    kind: String,
    #[serde(default)]
    picks: Vec<PickSpec>,
    #[serde(default)]
    on: Option<String>,
    #[serde(default)]
    enabled: Option<bool>,
    #[serde(default)]
    params: Map<String, Value>,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
struct PickSpec {
    point: [f64; 3],
    #[serde(default)]
    normal: Option<[f64; 3]>,
    #[serde(default)]
    tangent: Option<[f64; 3]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SimSpec {
    #[serde(default = "default_detail")]
    detail: usize,
    #[serde(default = "default_tol")]
    tolerance: f64,
}

fn default_detail() -> usize {
    300
}

fn default_tol() -> f64 {
    0.02
}

/// How far a pick point may be from the model (mm).
const PICK_TOL: f64 = 0.25;

/// `stepcam batch <spec.json>`: returns the process exit code
/// (0 ok, 1 the job has errors, 2 the spec could not be read).
pub fn main(args: &[String]) -> i32 {
    let Some(path) = args.first() else {
        eprintln!("usage: stepcam batch <spec.json>   (or - for stdin)");
        return 2;
    };
    let text = if path == "-" {
        let mut s = String::new();
        if let Err(e) = std::io::Read::read_to_string(&mut std::io::stdin(), &mut s) {
            eprintln!("cannot read stdin: {e}");
            return 2;
        }
        s
    } else {
        match std::fs::read_to_string(path) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("cannot read {path}: {e}");
                return 2;
            }
        }
    };
    let (report, code) = match serde_json::from_str::<Spec>(&text) {
        Ok(spec) => run(&spec),
        Err(e) => (json!({"ok": false, "errors": [format!("bad spec: {e}")]}), 2),
    };
    println!("{}", serde_json::to_string_pretty(&report).unwrap_or_default());
    code
}

fn run(spec: &Spec) -> (Value, i32) {
    let fail = |errors: Vec<String>| (json!({"ok": false, "errors": errors}), 1);
    // settings: defaults or a settings file, then overrides
    let mut settings = match &spec.settings_file {
        None => Settings::default(),
        Some(p) => match std::fs::read_to_string(p).map_err(|e| e.to_string()).and_then(|s| serde_json::from_str::<Settings>(&s).map_err(|e| e.to_string())) {
            Ok(s) => s,
            Err(e) => return fail(vec![format!("settings file {p}: {e}")]),
        },
    };
    if !spec.settings.is_empty() {
        let given: Map<String, Value> = spec.settings.iter().map(|(k, v)| (k.clone(), enum_name(k, v))).collect();
        match merge(&settings, &given, "settings") {
            Ok(s) => settings = s,
            Err(e) => return fail(vec![e]),
        }
    }
    settings.normalize();

    let raw = match step::load(&spec.step) {
        Ok(m) => m,
        Err(e) => return fail(vec![format!("cannot import {}: {e}", spec.step)]),
    };
    let placement = spec.placement.unwrap_or_default();
    let model = raw.transformed(&placement);
    let mut job = StepJob {
        step_path: std::fs::canonicalize(&spec.step).map(|p| p.to_string_lossy().into_owned()).unwrap_or_else(|_| spec.step.clone()),
        placement,
        stock: spec.stock.unwrap_or_default(),
        ..Default::default()
    };
    if let Some(r) = spec.rapid_above {
        job.rapid_above = r;
    }
    let stock = job.stock.bounds(model.min, model.max);
    let stock_top = stock.1[2];
    if let Some(o) = spec.origin {
        job.origin = o;
    }
    job.origin.resolve(stock.0, stock.1);
    let mut warnings: Vec<String> = job.stock.issues(model.min, model.max);

    let mut errors = vec![];
    for (i, o) in spec.ops.iter().enumerate() {
        let what = format!("op {} ({})", i + 1, o.kind);
        match build(o, &model, stock, stock_top, &settings, &job.ops) {
            Ok(ops) => {
                if ops.is_empty() {
                    warnings.push(format!("{what}: made no operations"));
                }
                for kind in ops {
                    let kind = match apply_params(kind, &o.params, &settings) {
                        Ok(k) => k,
                        Err(e) => {
                            errors.push(format!("{what}: {e}"));
                            continue;
                        }
                    };
                    job.ops.push(StepOp { enabled: o.enabled.unwrap_or(true), kind });
                }
            }
            Err(e) => errors.push(format!("{what}: {e}")),
        }
    }
    if !errors.is_empty() {
        return fail(errors);
    }

    for (k, w) in stepmill::finish_warnings(&job.ops) {
        warnings.push(format!("operation {} ({}): {w}", k + 1, job.ops[k].label()));
    }
    // the rapid plane as on the Step Milling tab
    let mut gen_settings = settings.clone();
    gen_settings.safe_z = model.max[2].max(stock_top) + job.rapid_above;
    let ops_report: Vec<Value> = job.ops.iter().map(|o| json!({"label": o.label(), "enabled": o.enabled})).collect();
    let code = match stepmill::generate(&job, &gen_settings) {
        Ok(c) => c,
        Err(e) => return (json!({"ok": false, "errors": e, "warnings": warnings, "ops": ops_report}), 1),
    };
    let secs = estimate::estimate(&code, gen_settings.rapid_rate, gen_settings.max_accel);
    let mut report = json!({
        "ok": true,
        "errors": [],
        "warnings": warnings,
        "ops": ops_report,
        "lines": code.lines().count(),
        "run_time": secs,
        "run_time_text": estimate::format_time(secs),
        "stock": {"min": stock.0, "max": stock.1},
        "origin": job.origin.at,
    });
    if let Some(p) = &spec.gcode_out {
        if let Err(e) = std::fs::write(p, &code) {
            return fail(vec![format!("cannot write {p}: {e}")]);
        }
        report["gcode"] = json!(p);
    } else {
        report["gcode_text"] = json!(code);
    }
    if let Some(p) = &spec.job_out {
        let full = Job { groups: vec![], settings: settings.clone(), step: job.clone() };
        let text = serde_json::to_string_pretty(&full).unwrap_or_default();
        if let Err(e) = std::fs::write(p, text) {
            return fail(vec![format!("cannot write {p}: {e}")]);
        }
        report["job"] = json!(p);
    }
    if let Some(s) = &spec.simulate {
        report["simulation"] = simulate(&code, &gen_settings, stock, &model, job.origin.at, s);
    }
    (report, 0)
}

/// Run the program through the simulator to the end.
fn simulate(code: &str, s: &Settings, stock: ([f64; 3], [f64; 3]), model: &step::Model, origin: [f64; 3], spec: &SimSpec) -> Value {
    // the program is relative to the origin: simulate it where it cuts
    let mut prog = sim::Program::parse(code, s);
    prog.translate(origin);
    let mut sm = sim::Sim::new(prog, stock.0, stock.1, spec.detail.clamp(20, 2000));
    while !sm.seek(f64::INFINITY, std::time::Duration::from_secs(3600)) {}
    let part = sm.map.surface_of(sim::top_tris(model));
    let tol = spec.tolerance as f32;
    let (mut cells, mut at, mut gouged, mut gouge, mut left) = (0usize, 0usize, 0usize, 0f32, 0f32);
    let mut worst: Option<[f64; 3]> = None;
    let mut left_at: Option<[f64; 3]> = None;
    for (k, (&z, &p)) in sm.map.z.iter().zip(&part).enumerate() {
        if p.is_nan() {
            continue;
        }
        cells += 1;
        let d = z - p;
        if d < -tol {
            gouged += 1;
            if -d > gouge {
                gouge = -d;
                let c = sm.map.centre(k % sm.map.nx, k / sm.map.nx);
                worst = Some([c[0], c[1], z as f64]);
            }
        } else if d <= tol {
            at += 1;
        }
        if d > left {
            left = d;
            let c = sm.map.centre(k % sm.map.nx, k / sm.map.nx);
            left_at = Some([c[0], c[1], z as f64]);
        }
    }
    let events: Vec<Value> = sm
        .events
        .values()
        .map(|e| {
            json!({
                "kind": match e.kind { sim::EventKind::RapidCut => "rapid_through_material", sim::EventKind::TooDeep => "deeper_than_flutes" },
                "line": e.line + 1,
                "depth": e.depth,
                "flute": e.flute,
            })
        })
        .collect();
    json!({
        "removed_mm3": sm.removed(),
        "stock_mm3": sm.stock_volume,
        "cell_mm": sm.map.cell,
        "at_size_pct": if cells > 0 { 100.0 * at as f64 / cells as f64 } else { 0.0 },
        "max_stock_left": left.max(0.0),
        "max_stock_left_at": left_at,
        "gouged_cells": gouged,
        "max_gouge": gouge,
        "max_gouge_at": worst,
        "events": events,
        "warnings": sm.prog.warnings,
    })
}

/// The operations for one spec entry, with the model's defaults.
fn build(o: &OpSpec, model: &step::Model, stock: ([f64; 3], [f64; 3]), stock_top: f64, s: &Settings, ops: &[StepOp]) -> Result<Vec<OpKind>, String> {
    let faces = || -> Result<Vec<usize>, String> {
        if o.picks.is_empty() {
            return Err("no faces given".into());
        }
        let mut v: Vec<usize> = vec![];
        for p in &o.picks {
            let f = find_face(model, p)?;
            if !v.contains(&f) {
                v.push(f);
            }
        }
        Ok(v)
    };
    let edges = || -> Result<Vec<(usize, [f64; 3], f64)>, String> {
        if o.picks.is_empty() {
            return Err("no edges given".into());
        }
        o.picks.iter().map(|p| find_edge(model, p)).collect()
    };
    // one op per distinct edge chain
    let chains = |es: Vec<(usize, [f64; 3], f64)>| -> Vec<usize> {
        let mut seen: Vec<Vec<usize>> = vec![];
        let mut out = vec![];
        for (e, _, _) in es {
            let mut c = model.edge_chain(e).0;
            c.sort_unstable();
            if !seen.contains(&c) {
                seen.push(c);
                out.push(e);
            }
        }
        out
    };
    let up_flat = |fi: usize| -> Result<(), String> {
        let f = &model.faces[fi];
        if f.is_up_horizontal() {
            Ok(())
        } else {
            Err(format!("face at {} is not a flat surface facing up", pt(model, fi)))
        }
    };
    Ok(match o.kind.as_str() {
        "rough" => vec![OpKind::Rough(crate::rough_op(model, stock, s))],
        "finish" => vec![OpKind::Finish(crate::finish_op(model, stock, s))],
        "face" => faces()?.into_iter().map(|fi| up_flat(fi).map(|_| OpKind::Face(crate::face_op(model, fi, stock_top, s)))).collect::<Result<_, _>>()?,
        "pocket" => match o.on.as_deref() {
            Some("edge") => chains(edges()?).into_iter().map(|e| crate::edge_pocket(model, e, stock_top).map(OpKind::Pocket)).collect::<Result<_, _>>()?,
            Some("face") | None => faces()?
                .into_iter()
                .map(|fi| crate::face_pocket(&model.faces[fi], fi, stock_top).map(OpKind::Pocket).ok_or_else(|| format!("face at {} is not a flat surface facing (or tilted) up", pt(model, fi))))
                .collect::<Result<_, _>>()?,
            Some(x) => return Err(format!("\"on\" must be \"face\" or \"edge\", not {x:?}")),
        },
        "profile" => chains(edges()?).into_iter().map(|e| crate::edge_profile(model, e, stock_top).map(OpKind::Profile)).collect::<Result<_, _>>()?,
        "engrave" => {
            let mut v = vec![];
            for p in &o.picks {
                let fi = find_face(model, p)?;
                up_flat(fi)?;
                v.push(OpKind::Engrave(crate::new_engrave(fi, &model.faces[fi], Some(p.point), 0.0)));
            }
            v
        }
        "thread" => {
            let fs = faces()?;
            let pick = crate::pick_holes(model, fs[0], &fs);
            if let Some(w) = pick.why {
                return Err(w);
            }
            if pick.skipped > 0 {
                return Err(format!("{} of the faces are not full vertical holes of a thread size", pick.skipped));
            }
            crate::thread_ops(&pick.holes, s).into_iter().map(OpKind::Thread).collect()
        }
        "thread_alike" => {
            let mut v = vec![];
            for fi in faces()? {
                let pick = crate::pick_holes(model, fi, &[]);
                if let Some(w) = pick.why {
                    return Err(w);
                }
                // skip holes threaded by earlier ops (or earlier faces of this one)
                let mut done: Vec<StepOp> = ops.to_vec();
                done.extend(v.iter().cloned().map(|k| StepOp { enabled: true, kind: k }));
                let (alike, _) = crate::alike_holes(model, fi, &done);
                v.extend(crate::thread_ops(&alike, s).into_iter().map(OpKind::Thread));
            }
            v
        }
        "tab" => edges()?.into_iter().map(|(e, p, a)| OpKind::Tab(crate::mill_tab(model, e, p, a))).collect(),
        "omit" => {
            let fills = faces()?.into_iter().map(|fi| crate::stepmill::omit_fill(model, fi).map_err(|e| format!("face at {}: {e}", pt(model, fi)))).collect::<Result<Vec<_>, _>>()?;
            vec![OpKind::Omit(crate::stepmill::OmitOp { fills })]
        }
        k => return Err(format!("unknown operation type {k:?} (rough, finish, face, pocket, profile, engrave, thread, thread_alike, tab, omit)")),
    })
}

fn pt(model: &step::Model, fi: usize) -> String {
    let c = crate::face_center(&model.faces[fi]);
    format!("X{} Y{}", gcode::fmt(c[0]), gcode::fmt(c[1]))
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: [f64; 3]) -> [f64; 3] {
    let l = dot(a, a).sqrt().max(1e-12);
    [a[0] / l, a[1] / l, a[2] / l]
}

/// Closest point on triangle `t` to `p` (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_tri(p: [f64; 3], t: &[[f64; 3]; 3]) -> [f64; 3] {
    let (a, b, c) = (t[0], t[1], t[2]);
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let at = |o: [f64; 3], d: [f64; 3], k: f64| [o[0] + d[0] * k, o[1] + d[1] * k, o[2] + d[2] * k];
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return at(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return at(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return at(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let den = 1.0 / (va + vb + vc);
    let (v, w) = (vb * den, vc * den);
    [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
}

/// The model face at a pick point: the nearest one, preferring a face whose
/// normal matches the pick's among faces about as near.
fn find_face(model: &step::Model, p: &PickSpec) -> Result<usize, String> {
    let mut cands: Vec<(f64, usize, [f64; 3])> = vec![];
    for (fi, f) in model.faces.iter().enumerate() {
        let mut best: Option<(f64, [f64; 3])> = None;
        for t in &f.tris {
            let q = closest_on_tri(p.point, t);
            let d = dot(sub(q, p.point), sub(q, p.point)).sqrt();
            if best.map_or(true, |b| d < b.0) {
                let n = if f.planar { f.normal } else { norm(cross(sub(t[1], t[0]), sub(t[2], t[0]))) };
                best = Some((d, n));
            }
        }
        if let Some((d, n)) = best {
            cands.push((d, fi, n));
        }
    }
    let near = cands.iter().map(|c| c.0).fold(f64::INFINITY, f64::min);
    if near > PICK_TOL {
        let [x, y, z] = p.point;
        return Err(format!("no model face at ({x:.3}, {y:.3}, {z:.3}) (nearest is {near:.3} mm away)"));
    }
    let score = |c: &(f64, usize, [f64; 3])| match p.normal {
        // curved faces' triangle normals may point either way: compare |dot|
        Some(n) => c.0 + 0.05 * (1.0 - dot(norm(n), c.2).abs()),
        None => c.0,
    };
    Ok(cands.iter().filter(|c| c.0 <= near + 0.02).min_by(|a, b| score(a).total_cmp(&score(b))).unwrap().1)
}

/// The model edge at a pick point: (edge, nearest point on it, XY tangent angle).
fn find_edge(model: &step::Model, p: &PickSpec) -> Result<(usize, [f64; 3], f64), String> {
    let mut best: Option<(f64, usize, [f64; 3], f64)> = None;
    for (ei, e) in model.edges.iter().enumerate() {
        for w in e.pts.windows(2) {
            let d = sub(w[1], w[0]);
            let l2 = dot(d, d);
            let t = if l2 < 1e-18 { 0.0 } else { (dot(sub(p.point, w[0]), d) / l2).clamp(0.0, 1.0) };
            let q = [w[0][0] + d[0] * t, w[0][1] + d[1] * t, w[0][2] + d[2] * t];
            let dist = dot(sub(q, p.point), sub(q, p.point)).sqrt();
            if best.map_or(true, |b| dist < b.0) {
                best = Some((dist, ei, q, d[1].atan2(d[0])));
            }
        }
    }
    match best {
        Some((d, e, q, a)) if d <= PICK_TOL => {
            let angle = p.tangent.filter(|t| t[0].hypot(t[1]) > 1e-6).map_or(a, |t| t[1].atan2(t[0]));
            Ok((e, q, angle))
        }
        Some((d, ..)) => {
            let [x, y, z] = p.point;
            Err(format!("no model edge at ({x:.3}, {y:.3}, {z:.3}) (nearest is {d:.3} mm away)"))
        }
        None => Err("the model has no edges".into()),
    }
}

/// `base` with the fields in `over` replaced (unknown names are errors).
fn merge<T: serde::Serialize + serde::de::DeserializeOwned>(base: &T, over: &Map<String, Value>, what: &str) -> Result<T, String> {
    let mut v = serde_json::to_value(base).map_err(|e| e.to_string())?;
    let obj = v.as_object_mut().ok_or("not an object")?;
    for (k, x) in over {
        if !obj.contains_key(k) {
            let mut names: Vec<&String> = obj.keys().collect();
            names.sort();
            return Err(format!("unknown {what} parameter {k:?} (valid: {})", names.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ")));
        }
        obj.insert(k.clone(), x.clone());
    }
    serde_json::from_value(v).map_err(|e| format!("{what}: {e}"))
}

/// Enum values may be given in any case ("adaptive", "outside", "m2.5").
fn enum_name(key: &str, v: &Value) -> Value {
    let Some(s) = v.as_str() else { return v.clone() };
    match key {
        // "square_spiral" / "square spiral" -> SquareSpiral
        "strategy" | "entry" | "side" | "h_align" | "v_align" | "pattern" | "facing" | "face_pattern" | "rough_strategy" => json!(s
            .split(['_', ' ', '-'])
            .map(|w| {
                let mut c = w.chars();
                c.next().map(|f| f.to_uppercase().collect::<String>() + &c.as_str().to_lowercase()).unwrap_or_default()
            })
            .collect::<String>()),
        "thread" => json!(s.to_uppercase().replace('.', "_")),
        _ => v.clone(),
    }
}

/// Apply user parameters to a new op. `tool` takes a tool number (engraving:
/// an engraving bit or an end mill); thread items take ThreadGroup fields.
fn apply_params(kind: OpKind, params: &Map<String, Value>, s: &Settings) -> Result<OpKind, String> {
    if params.is_empty() {
        return Ok(kind);
    }
    let mut p: Map<String, Value> = params.iter().map(|(k, v)| (k.clone(), enum_name(k, v))).collect();
    // a strategy given without a stepover takes its own default (adaptive
    // 20 %, else 40 %), whatever the settings' default strategy gave
    if let (Some(st), false) = (p.get("strategy").and_then(|v| v.as_str()), p.contains_key("stepover_pct")) {
        let so = if st == "Adaptive" { crate::stepmill::STEPOVER_ADAPTIVE } else { crate::stepmill::STEPOVER_STANDARD };
        p.insert("stepover_pct".into(), json!(so));
    }
    let tool = p.remove("tool");
    let cutter = |t: &Value| -> Result<crate::model::Cutter, String> {
        let n = t.as_u64().ok_or("tool must be a tool number")? as u32;
        s.cutters.iter().copied().find(|c| c.tool == n).ok_or_else(|| format!("T{n} is not an end mill in the tool table"))
    };
    Ok(match kind {
        OpKind::Face(mut o) => {
            if let Some(t) = &tool {
                o.cutter = cutter(t)?;
            }
            OpKind::Face(merge(&o, &p, "face")?)
        }
        OpKind::Pocket(mut o) => {
            if let Some(t) = &tool {
                o.cutter = cutter(t)?;
            }
            OpKind::Pocket(merge(&o, &p, "pocket")?)
        }
        OpKind::Profile(mut o) => {
            if let Some(t) = &tool {
                o.cutter = cutter(t)?;
            }
            OpKind::Profile(merge(&o, &p, "profile")?)
        }
        OpKind::Rough(mut o) => {
            if let Some(t) = &tool {
                o.cutter = cutter(t)?;
            }
            // the faces' meshes are not serialised: keep them
            let faces = o.faces.clone();
            let mut n: crate::stepmill::RoughOp = merge(&o, &p, "rough")?;
            n.faces = faces;
            OpKind::Rough(n)
        }
        OpKind::Finish(mut o) => {
            if let Some(t) = &tool {
                o.cutter = cutter(t)?;
            }
            let faces = o.faces.clone();
            let mut n: crate::stepmill::RoughOp = merge(&o, &p, "finish")?;
            n.faces = faces;
            OpKind::Finish(n)
        }
        OpKind::Engrave(mut o) => {
            if let Some(t) = &tool {
                let n = t.as_u64().ok_or("tool must be a tool number")? as u32;
                o.tool = match (s.engravers.iter().find(|e| e.tool == n), s.cutters.iter().find(|c| c.tool == n)) {
                    (Some(e), _) => crate::engrave::EngraveTool::Bit(*e),
                    (None, Some(c)) => crate::engrave::EngraveTool::EndMill(*c),
                    _ => return Err(format!("T{n} is not an engraving bit or end mill in the tool table")),
                };
            }
            OpKind::Engrave(merge(&o, &p, "engrave")?)
        }
        OpKind::Thread(mut o) => {
            if let Some(t) = &tool {
                o.group.cutter = cutter(t)?;
            }
            if let Some(v) = p.get("thread") {
                serde_json::from_value::<ThreadSize>(v.clone()).map_err(|_| format!("thread must be one of M2, M2.5, M3, M4, M5, M6, not {v}"))?;
            }
            o.group = merge(&o.group, &p, "thread")?;
            OpKind::Thread(o)
        }
        OpKind::Tab(o) => {
            if tool.is_some() {
                return Err("mill tabs have no tool".into());
            }
            OpKind::Tab(merge(&o, &p, "tab")?)
        }
        OpKind::Omit(_) => return Err("omit takes no parameters".into()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(ops: Value) -> Spec {
        serde_json::from_value(json!({"step": "testdata/pocket_block.step", "stock": {"x": 4, "y": 4, "z": 1, "offset": true}, "ops": ops})).unwrap()
    }

    #[test]
    fn builds_and_generates_from_picks() {
        let m = step::load("testdata/pocket_block.step").unwrap();
        let top = m.faces.iter().position(|f| f.is_up_horizontal() && (f.z() - m.max[2]).abs() < 1e-6).unwrap();
        let floor = m.faces.iter().enumerate().filter(|(_, f)| f.is_up_horizontal() && f.z() < m.max[2] - 0.1 && f.z() > m.min[2] + 0.1).map(|(i, _)| i).next().unwrap();
        let inside = |fi: usize| {
            let t = &m.faces[fi].tris[0];
            [0, 1, 2].map(|k| (t[0][k] + t[1][k] + t[2][k]) / 3.0)
        };
        // an outline edge at the bottom for a profile and a tab
        let bottom_edge = m.edges.iter().position(|e| !e.faces.is_empty() && e.pts.iter().all(|p| (p[2] - m.min[2]).abs() < 1e-6)).unwrap();
        let ep = m.edges[bottom_edge].pts[0];
        let ep2 = m.edges[bottom_edge].pts[1];
        let mid = [(ep[0] + ep2[0]) / 2.0, (ep[1] + ep2[1]) / 2.0, ep[2]];
        let sp = spec(json!([
            {"type": "rough", "params": {"tool": 4, "allowance": 0.2, "strategy": "adaptive"}},
            {"type": "face", "picks": [{"point": inside(top), "normal": [0, 0, 1]}], "params": {"stepover_pct": 60}},
            {"type": "pocket", "picks": [{"point": inside(floor)}], "params": {"entry": "zigzag"}},
            {"type": "profile", "picks": [{"point": mid}], "params": {"side": "outside", "tool": 3}},
            {"type": "tab", "picks": [{"point": mid}], "params": {"width": 4}},
            {"type": "engrave", "picks": [{"point": inside(top)}], "params": {"text": "HI", "tool": 22, "h_align": "left"}},
        ]));
        let (r, code) = run(&sp);
        assert_eq!(code, 0, "{r:#}");
        let ops = r["ops"].as_array().unwrap();
        assert_eq!(ops.len(), 6);
        let g = r["gcode_text"].as_str().unwrap();
        assert!(g.contains("T4 M6") && g.contains("T3 M6") && g.contains("T22 M6"), "{g}");
        assert!(r["run_time"].as_f64().unwrap() > 1.0);
    }

    #[test]
    fn reports_bad_picks_and_parameters() {
        let (r, code) = run(&spec(json!([{"type": "face", "picks": [{"point": [999, 0, 0]}]}])));
        assert_eq!(code, 1);
        assert!(r["errors"][0].as_str().unwrap().contains("no model face"), "{r}");
        let (r, code) = run(&spec(json!([{"type": "rough", "params": {"stepovr": 3}}])));
        assert_eq!(code, 1);
        assert!(r["errors"][0].as_str().unwrap().contains("unknown rough parameter \"stepovr\""), "{r}");
        let (r, _) = run(&spec(json!([{"type": "rough", "params": {"tool": 99}}])));
        assert!(r["errors"][0].as_str().unwrap().contains("T99"), "{r}");
        let (r, _) = run(&spec(json!([{"type": "drill"}])));
        assert!(r["errors"][0].as_str().unwrap().contains("unknown operation type"), "{r}");
    }

    #[test]
    fn simulates_and_writes_a_job_the_app_can_load() {
        let dir = std::env::temp_dir().join(format!("stepcam-batch-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let job = dir.join("t.stepcam.json");
        let mut sp = spec(json!([{"type": "rough"}]));
        sp.job_out = Some(job.to_string_lossy().into_owned());
        sp.simulate = Some(SimSpec { detail: 150, tolerance: 0.02 });
        let (r, code) = run(&sp);
        assert_eq!(code, 0, "{r:#}");
        let s = &r["simulation"];
        assert!(s["removed_mm3"].as_f64().unwrap() > 10.0, "{s}");
        assert_eq!(s["gouged_cells"], 0, "{s}");
        let j: Job = serde_json::from_str(&std::fs::read_to_string(&job).unwrap()).unwrap();
        assert_eq!(j.step.ops.len(), 1);
        assert!(std::path::Path::new(&j.step.step_path).is_absolute());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn rough_entries_do_not_cut_into_part_corners() {
        // testdata/bracket.step (python/example.py): the band round the
        // plate's sharp outer corners is too narrow for a helix, so entries
        // ramp along the corner arcs - and must stay on them
        for entry in ["zigzag", "helix"] {
            let mut sp: Spec = serde_json::from_value(json!({
                "step": "testdata/bracket.step", "stock": {"x": 6, "y": 6, "z": 1},
                "ops": [{"type": "rough", "params": {"tool": 4, "entry": entry}}],
                "simulate": {"detail": 300}
            }))
            .unwrap();
            sp.gcode_out = None;
            let (r, code) = run(&sp);
            assert_eq!(code, 0, "{r:#}");
            let s = &r["simulation"];
            assert_eq!(s["gouged_cells"], 0, "{entry}: {s}");
        }
    }

    #[test]
    fn origin_moves_the_gcode_but_not_the_cut() {
        let run_with = |origin: Value| {
            let mut sp: Spec = serde_json::from_value(json!({
                "step": "testdata/pocket_block.step", "stock": {"x": 4, "y": 4, "z": 1},
                "origin": origin, "ops": [{"type": "rough"}], "simulate": {"detail": 150}
            }))
            .unwrap();
            sp.gcode_out = None;
            let (r, code) = run(&sp);
            assert_eq!(code, 0, "{r:#}");
            r
        };
        let plain = run_with(json!({}));
        let moved = run_with(json!({"xy": "bottom_left", "z": "stock_top"}));
        let (lo, hi) = (&moved["stock"]["min"], &moved["stock"]["max"]);
        assert_eq!(moved["origin"], json!([lo[0], lo[1], hi[2]]));
        assert_ne!(plain["gcode_text"], moved["gcode_text"]);
        // the same program, relative to the origin: mapped back it matches
        let tp0 = crate::toolpath::parse(plain["gcode_text"].as_str().unwrap());
        let mut tp = crate::toolpath::parse(moved["gcode_text"].as_str().unwrap());
        tp.translate([lo[0].as_f64().unwrap() as f32, lo[1].as_f64().unwrap() as f32, hi[2].as_f64().unwrap() as f32]);
        for k in 0..3 {
            assert!((tp.min[k] - tp0.min[k]).abs() < 1e-3 && (tp.max[k] - tp0.max[k]).abs() < 1e-3);
        }
        // the simulation (mapped back) is the same cut
        let (a, b) = (&plain["simulation"], &moved["simulation"]);
        assert!((a["removed_mm3"].as_f64().unwrap() - b["removed_mm3"].as_f64().unwrap()).abs() < 1e-6, "{a} vs {b}");
        assert_eq!(b["gouged_cells"], 0);
        let custom = run_with(json!({"xy": "custom", "z": "custom", "at": [1, 2, 3]}));
        assert_eq!(custom["origin"], json!([1.0, 2.0, 3.0]));
    }

    #[test]
    fn parameters_keep_the_curved_faces_meshes() {
        // a.step's fillet and chamfer are curved: setting a rough parameter
        // must not drop their meshes (job files leave them out)
        let mut sp: Spec = serde_json::from_value(json!({
            "step": "testdata/a.step", "stock": {"x": 10, "y": 10, "z": 1},
            "ops": [{"type": "rough", "params": {"tool": 4, "strategy": "adaptive"}}, {"type": "finish", "params": {"tool": 4}}],
            "simulate": {"detail": 250}
        }))
        .unwrap();
        sp.gcode_out = None;
        let model = step::load("testdata/a.step").unwrap();
        let stock = Stock { x: 10.0, y: 10.0, z: 1.0, ..Default::default() }.bounds(model.min, model.max);
        let built = build(&sp.ops[0], &model, stock, stock.1[2], &Settings::default(), &[]).unwrap().remove(0);
        let OpKind::Rough(r) = apply_params(built, &sp.ops[0].params, &Settings::default()).unwrap() else { panic!() };
        assert!(r.faces.iter().any(|f| f.curved && !f.tris.is_empty()));
        let (rep, code) = run(&sp);
        assert_eq!(code, 0, "{rep:#}");
        let sim = &rep["simulation"];
        assert_eq!(sim["gouged_cells"], 0, "{sim}");
        assert_eq!(sim["events"], json!([]), "{sim}");
    }

    #[test]
    fn closest_point_on_a_triangle() {
        let t = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let q = closest_on_tri([0.2, 0.2, 5.0], &t);
        assert!((q[0] - 0.2).abs() < 1e-12 && (q[1] - 0.2).abs() < 1e-12 && q[2] == 0.0);
        assert_eq!(closest_on_tri([-1.0, -1.0, 0.0], &t), [0.0, 0.0, 0.0]);
        let q = closest_on_tri([1.0, 1.0, 0.0], &t);
        assert!((q[0] - 0.5).abs() < 1e-12 && (q[1] - 0.5).abs() < 1e-12);
    }
}
