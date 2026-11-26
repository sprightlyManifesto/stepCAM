//! Simulator tab: plays a program against a Z-map of the stock, with the
//! tool, the path done so far, collision / depth warnings and a comparison
//! of the cut result with the model.

use crate::model::Settings;
use crate::sim::{Event, EventKind, Motion, Program, Shape, Sim, P3};
use crate::view3d::View3d;
use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::time::{Duration, Instant};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Source {
    Step,
    /// The Nest tab's program.
    Nest,
    File,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PathShow {
    All,
    Done,
    Off,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Detail {
    Low,
    Medium,
    High,
    Fine,
}

impl Detail {
    fn cells(self) -> usize {
        match self {
            Detail::Low => 150,
            Detail::Medium => 300,
            Detail::High => 500,
            Detail::Fine => 800,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Detail::Low => "Low",
            Detail::Medium => "Medium",
            Detail::High => "High",
            Detail::Fine => "Fine",
        }
    }
}

/// What the tab simulates.
pub struct Inputs<'a> {
    pub step_code: &'a str,
    pub settings: &'a Settings,
    pub model: Option<&'a crate::step::Model>,
    /// Step milling stock box.
    pub step_stock: Option<(P3, P3)>,
    /// Step milling program zero (model coordinates): the step program is
    /// relative to it, so its moves are mapped back before simulating.
    pub step_origin: P3,
    /// Nest tab: its program, sheet, program zero and copy offsets.
    pub nest_code: &'a str,
    pub nest_stock: Option<(P3, P3)>,
    pub nest_origin: P3,
    pub nest_copies: &'a [crate::stepmill::Placed],
    /// Step milling stock model (triangles), when the stock is one: the
    /// simulation starts from its shape (on every copy, for the nest).
    pub stock_tris: Option<&'a [[P3; 3]]>,
}

/// Comparison with the model: worst gouge, cells gouged, most stock left.
#[derive(Clone, Copy, Default)]
struct Compare {
    gouge: f32,
    gouged: usize,
    left: f32,
    at_size: usize,
    cells: usize,
}

pub struct SimTab {
    pub source: Source,
    file_code: String,
    file_name: String,
    sim: Option<Sim>,
    /// Hash of the program inputs / of everything the sim depends on.
    prog_key: u64,
    sim_key: u64,
    /// Model top surface per Z-map cell (compare).
    part: Option<Vec<f32>>,
    view: View3d,
    playing: bool,
    /// Playback speed (x machine time).
    speed: f64,
    /// Time the sim is heading for.
    target: f64,
    last_frame: Option<Instant>,
    path: PathShow,
    show_tool: bool,
    /// Path kinds shown: rapids, milling, threading.
    show_rapid: bool,
    show_mill: bool,
    show_thread: bool,
    show_stock: bool,
    compare: bool,
    /// At-size band of the comparison (mm).
    tol: f64,
    detail: Detail,
    custom_stock: bool,
    stock: (P3, P3),
    follow: bool,
    last_line: Option<usize>,
    scroll_to: Option<usize>,
    /// Quad draw order for (nx, ny, view direction bucket).
    order: Option<((usize, usize, i32), Vec<u32>)>,
    compare_cache: Option<((u64, u64), Compare)>,
    status: String,
    /// Program zero to draw (step milling only).
    origin: Option<P3>,
}

impl Default for SimTab {
    fn default() -> Self {
        SimTab {
            source: Source::Step,
            file_code: String::new(),
            file_name: String::new(),
            sim: None,
            prog_key: 0,
            sim_key: 0,
            part: None,
            view: View3d::default(),
            playing: false,
            speed: 10.0,
            target: 0.0,
            last_frame: None,
            path: PathShow::All,
            show_tool: true,
            show_rapid: true,
            show_mill: true,
            show_thread: true,
            show_stock: true,
            compare: true,
            tol: 0.02,
            detail: Detail::Medium,
            custom_stock: false,
            stock: ([0.0; 3], [10.0, 10.0, 5.0]),
            follow: true,
            last_line: None,
            scroll_to: None,
            order: None,
            compare_cache: None,
            status: String::new(),
            origin: None,
        }
    }
}

const STOCK: Color32 = Color32::from_rgb(178, 162, 132);
const CUT: Color32 = Color32::from_rgb(190, 198, 212);
const FLOOR: Color32 = Color32::from_rgb(85, 88, 95);
const AT_SIZE: Color32 = Color32::from_rgb(70, 190, 95);
const GOUGE: Color32 = Color32::from_rgb(225, 55, 45);
const LEFT_LO: Color32 = Color32::from_rgb(235, 215, 70);
const LEFT_HI: Color32 = Color32::from_rgb(240, 135, 45);
const FLUTES: Color32 = Color32::from_rgba_premultiplied(215, 175, 60, 235);
const SHANK: Color32 = Color32::from_rgba_premultiplied(140, 142, 150, 235);
const WARN: Color32 = Color32::from_rgb(240, 150, 40);

fn hash_of(h: impl Hash) -> u64 {
    let mut s = DefaultHasher::new();
    h.hash(&mut s);
    s.finish()
}

fn bits(p: P3) -> [u64; 3] {
    p.map(f64::to_bits)
}

/// "1:02:03" / "2:03.4".
fn clock(t: f64) -> String {
    let t = (t.max(0.0) * 10.0).round() / 10.0;
    if t >= 3600.0 {
        let s = t as u64;
        format!("{}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
    } else {
        format!("{}:{:04.1}", (t / 60.0) as u64, t % 60.0)
    }
}

fn mix(a: Color32, b: Color32, t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    let m = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
    Color32::from_rgb(m(a.r(), b.r()), m(a.g(), b.g()), m(a.b(), b.b()))
}

fn shade(c: Color32, k: f32) -> Color32 {
    let m = |x: u8| (x as f32 * k).min(255.0) as u8;
    Color32::from_rgba_unmultiplied(m(c.r()), m(c.g()), m(c.b()), c.a())
}

fn f3(p: P3) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

impl SimTab {
    fn code<'a>(&'a self, inp: &Inputs<'a>) -> &'a str {
        match self.source {
            Source::Step => inp.step_code,
            Source::Nest => inp.nest_code,
            Source::File => &self.file_code,
        }
    }

    /// Stock from the job (step milling) or round the cutting moves.
    fn auto_stock(&self, inp: &Inputs, prog: &Program) -> Option<(P3, P3)> {
        if self.source == Source::Step {
            if let Some(s) = inp.step_stock {
                return Some(s);
            }
        }
        if self.source == Source::Nest {
            if let Some(s) = inp.nest_stock {
                return Some(s);
            }
        }
        let (lo, hi) = prog.feed_bounds?;
        let r = prog.tools.iter().map(|t| t.shape.radius()).fold(0.0, f64::max);
        let m = r + 2.0;
        let top = match self.source {
            _ if lo[2] < -1e-6 => 0.0,
            _ => hi[2],
        };
        let top = top.max(lo[2] + 0.1);
        Some(([lo[0] - m, lo[1] - m, lo[2] - 2.0], [hi[0] + m, hi[1] + m, top]))
    }

    /// Rebuild the program / sim when what it depends on changed.
    fn sync(&mut self, inp: &Inputs) {
        let code = self.code(inp);
        let pk = hash_of((self.source, code, format!("{:?}", inp.settings), bits(inp.step_origin), bits(inp.nest_origin), inp.nest_copies.iter().map(|c| [c.pivot[0].to_bits(), c.pivot[1].to_bits(), c.angle.to_bits(), c.shift[0].to_bits(), c.shift[1].to_bits()]).collect::<Vec<_>>()));
        if pk != self.prog_key || self.sim.is_none() {
            let mut prog = Program::parse(code, inp.settings);
            match self.source {
                Source::Step => prog.translate(inp.step_origin),
                Source::Nest => prog.translate(inp.nest_origin),
                Source::File => {}
            }
            self.prog_key = pk;
            self.rebuild(inp, prog);
            return;
        }
        let stock = self.stock_now(inp);
        let sk = hash_of((pk, stock.map(|(a, b)| (bits(a), bits(b))), self.detail, inp.model.map(|m| bits(m.min)), inp.model.map(|m| m.faces.len()), inp.stock_tris.map(|t| (t.len(), t.first().map(|t| bits(t[0]))))));
        if sk != self.sim_key {
            let prog = self.sim.take().map(|s| s.prog).unwrap_or_default();
            self.rebuild(inp, prog);
        }
    }

    fn stock_now(&self, inp: &Inputs) -> Option<(P3, P3)> {
        if self.custom_stock {
            Some(self.stock)
        } else {
            self.sim.as_ref().and_then(|s| self.auto_stock(inp, &s.prog))
        }
    }

    fn rebuild(&mut self, inp: &Inputs, prog: Program) {
        let keep = self.sim.as_ref().map(|s| s.time()).unwrap_or(0.0);
        let stock = if self.custom_stock { Some(self.stock) } else { self.auto_stock(inp, &prog) };
        self.order = None;
        self.compare_cache = None;
        self.part = None;
        let pk = self.prog_key;
        self.sim_key = hash_of((pk, stock.map(|(a, b)| (bits(a), bits(b))), self.detail, inp.model.map(|m| bits(m.min)), inp.model.map(|m| m.faces.len()), inp.stock_tris.map(|t| (t.len(), t.first().map(|t| bits(t[0]))))));
        let Some((lo, hi)) = stock else {
            self.sim = Some(Sim::new(prog, [0.0; 3], [1.0, 1.0, 0.0], 2));
            return;
        };
        if !self.custom_stock {
            self.stock = (lo, hi);
        }
        let first = self.sim.is_none() || self.sim.as_ref().is_some_and(|s| s.prog.pieces.is_empty());
        let mut sim = Sim::new(prog, lo, hi, self.detail.cells());
        if let (false, Some(tris)) = (self.custom_stock, inp.stock_tris) {
            match self.source {
                Source::Step => sim = sim.with_stock(tris),
                Source::Nest => {
                    let all: Vec<[P3; 3]> = inp.nest_copies.iter().flat_map(|c| tris.iter().map(move |t| t.map(|p| c.apply3(p)))).collect();
                    sim = sim.with_stock(&all);
                }
                Source::File => {}
            }
        }
        if let Some(m) = inp.model {
            match self.source {
                Source::Step => self.part = Some(sim.map.surface_of(crate::sim::top_tris(m))),
                // every copy of the part
                Source::Nest => {
                    let tris: Vec<[P3; 3]> = inp.nest_copies.iter().flat_map(|c| crate::sim::top_tris(m).map(move |t| t.map(|p| c.apply3(p)))).collect();
                    self.part = Some(sim.map.surface_of(tris.iter()));
                }
                Source::File => {}
            }
        }
        self.target = keep.min(sim.prog.total);
        self.sim = Some(sim);
        if first {
            self.view.needs_fit = true;
        }
    }

    fn open_file(&mut self) {
        let Some(path) = rfd::FileDialog::new().add_filter("G-code", &["nc", "ngc", "gcode", "tap", "txt"]).add_filter("All files", &["*"]).pick_file() else { return };
        match std::fs::read_to_string(&path) {
            Ok(s) => {
                self.file_code = s;
                self.file_name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                self.source = Source::File;
                self.target = 0.0;
                self.playing = false;
                self.view.needs_fit = true;
                self.status.clear();
            }
            Err(e) => self.status = format!("Cannot read {}: {e}", path.display()),
        }
    }

    fn jump(&mut self, t: f64) {
        self.target = t;
        self.playing = false;
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, inp: &Inputs) {
        self.sync(inp);
        self.origin = match self.source {
            Source::Step => Some(inp.step_origin),
            Source::Nest => Some(inp.nest_origin),
            Source::File => None,
        };
        let total = self.sim.as_ref().map_or(0.0, |s| s.prog.total);

        // --- playback clock
        let now = Instant::now();
        let dt = self.last_frame.map_or(0.0, |l| (now - l).as_secs_f64().min(0.1));
        self.last_frame = Some(now);
        if !ui.ctx().wants_keyboard_input() {
            let keys = ui.input(|i| {
                (i.key_pressed(egui::Key::Space), i.key_pressed(egui::Key::ArrowRight), i.key_pressed(egui::Key::ArrowLeft), i.key_pressed(egui::Key::Home), i.key_pressed(egui::Key::End))
            });
            if let Some(sim) = &self.sim {
                let t = sim.time();
                let (next, prev) = (sim.prog.next_move(t), sim.prog.prev_move(t));
                let (space, right, left, home, end) = keys;
                if space {
                    self.toggle_play(t, total);
                }
                if right {
                    self.jump(next);
                }
                if left {
                    self.jump(prev);
                }
                if home {
                    self.jump(0.0);
                }
                if end {
                    self.jump(total);
                }
            }
        }
        let mut reached = true;
        if let Some(sim) = &mut self.sim {
            if self.playing {
                self.target = (sim.time() + dt * self.speed).min(total);
            }
            reached = sim.seek(self.target, Duration::from_millis(16));
            if self.playing && reached && sim.time() >= total {
                self.playing = false;
            }
        }
        if self.playing || !reached {
            ui.ctx().request_repaint();
        }

        self.top_bar(ui, inp);
        ui.separator();
        self.transport(ui, total, reached);
        ui.add_space(2.0);

        egui::SidePanel::left("sim_info").resizable(true).default_width(270.0).show_inside(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| self.info(ui, reached));
        });
        egui::CentralPanel::default().show_inside(ui, |ui| self.scene(ui));
    }

    fn toggle_play(&mut self, t: f64, total: f64) {
        if self.playing {
            self.playing = false;
        } else {
            if t >= total {
                self.target = 0.0;
                if let Some(s) = &mut self.sim {
                    s.seek(0.0, Duration::from_millis(16));
                }
            }
            self.playing = true;
        }
    }

    fn top_bar(&mut self, ui: &mut egui::Ui, inp: &Inputs) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Simulate:");
            let before = self.source;
            ui.selectable_value(&mut self.source, Source::Step, "Step milling");
            ui.add_enabled_ui(!inp.nest_code.is_empty(), |ui| {
                ui.selectable_value(&mut self.source, Source::Nest, "Nest").on_disabled_hover_text("Open the Nest tab first (or fix its errors)");
            });
            ui.add_enabled_ui(!self.file_code.is_empty(), |ui| {
                ui.selectable_value(&mut self.source, Source::File, if self.file_name.is_empty() { "File".to_string() } else { self.file_name.clone() });
            });
            if ui.button("📂 Open G-code…").clicked() {
                self.open_file();
            }
            if self.source != before {
                self.target = 0.0;
                self.playing = false;
                self.view.needs_fit = true;
            }
            ui.separator();
            ui.label("Detail:");
            egui::ComboBox::from_id_salt("sim_detail").selected_text(self.detail.name()).show_ui(ui, |ui| {
                for d in [Detail::Low, Detail::Medium, Detail::High, Detail::Fine] {
                    ui.selectable_value(&mut self.detail, d, format!("{} ({} cells)", d.name(), d.cells()));
                }
            });
            ui.separator();
            let was = self.custom_stock;
            ui.checkbox(&mut self.custom_stock, "Custom stock").on_hover_text("Off: the step milling stock, or a box round the cutting moves");
            if self.custom_stock && !was {
                if let Some(s) = self.sim.as_ref().and_then(|s| self.auto_stock(inp, &s.prog)) {
                    self.stock = s;
                }
            }
            if self.custom_stock {
                let (lo, hi) = &mut self.stock;
                for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.label(format!("{a}"));
                    ui.add(egui::DragValue::new(&mut lo[k]).speed(0.1).max_decimals(3));
                    ui.label("…");
                    ui.add(egui::DragValue::new(&mut hi[k]).speed(0.1).max_decimals(3));
                    hi[k] = hi[k].max(lo[k] + 0.01);
                }
            }
            ui.separator();
            if ui.button("⟲ Restart").on_hover_text("Fresh stock, back to the start").clicked() {
                self.sim = None;
                self.target = 0.0;
                self.playing = false;
            }
        });
        if !self.status.is_empty() {
            ui.colored_label(ui.visuals().error_fg_color, &self.status);
        }
    }

    fn transport(&mut self, ui: &mut egui::Ui, total: f64, reached: bool) {
        let Some(t) = self.sim.as_ref().map(|s| s.time()) else { return };
        ui.horizontal(|ui| {
            let big = |s: &str| egui::Button::new(egui::RichText::new(s).size(16.0)).min_size(Vec2::new(30.0, 24.0));
            if ui.add(big("⏮")).on_hover_text("Start (Home)").clicked() {
                self.jump(0.0);
            }
            if ui.add(big("⏪")).on_hover_text("Previous move (←)").clicked() {
                let p = self.sim.as_ref().unwrap().prog.prev_move(t);
                self.jump(p);
            }
            if ui.add(big(if self.playing { "⏸" } else { "▶" })).on_hover_text("Play / pause (Space)").clicked() {
                self.toggle_play(t, total);
            }
            if ui.add(big("⏩")).on_hover_text("Next move (→)").clicked() {
                let n = self.sim.as_ref().unwrap().prog.next_move(t);
                self.jump(n);
            }
            if ui.add(big("⏭")).on_hover_text("End: run the whole program (End)").clicked() {
                self.jump(total);
            }
            ui.separator();
            ui.label("Speed");
            ui.add(egui::Slider::new(&mut self.speed, 0.1..=2000.0).logarithmic(true).max_decimals(1).suffix("×"));
            ui.separator();
            let label = format!("{} / {}", clock(if reached { t } else { self.target }), clock(total));
            let w = (ui.available_width() - 120.0).max(100.0);
            let mut scrub = if reached || self.playing { t } else { self.target };
            ui.spacing_mut().slider_width = w;
            let r = ui.add_enabled(total > 0.0, egui::Slider::new(&mut scrub, 0.0..=total.max(1e-6)).show_value(false));
            if r.changed() {
                self.jump(scrub);
            }
            ui.monospace(label);
        });
    }

    fn info(&mut self, ui: &mut egui::Ui, reached: bool) {
        let Some(sim) = &self.sim else { return };
        let prog = &sim.prog;
        if prog.pieces.is_empty() {
            ui.weak(match self.source {
                Source::Step => "No step milling G-code yet - import a model and add operations (or fix the errors on the Step Milling tab).",
                Source::Nest => "No nested G-code yet - see the Nest tab.",
                Source::File => "The file has no moves.",
            });
            return;
        }
        let t = sim.time();
        ui.heading("Machine");
        egui::Grid::new("sim_state").num_columns(2).striped(true).show(ui, |ui| {
            let cur = sim.current();
            ui.label("Time");
            ui.monospace(format!("{} / {}", clock(t), clock(prog.total)));
            ui.end_row();
            ui.label("Progress");
            ui.add(egui::ProgressBar::new((t / prog.total.max(1e-9)) as f32).show_percentage());
            ui.end_row();
            if let Some(p) = cur {
                ui.label("Line");
                ui.monospace(format!("{} / {}", p.line + 1, prog.lines.len()));
                ui.end_row();
                ui.label("Tool");
                ui.label(p.tool.map_or("none".into(), |k| prog.tools[k].name.clone()));
                ui.end_row();
                ui.label("Move");
                ui.label(match p.motion {
                    Motion::Rapid => "rapid (G0)".to_string(),
                    Motion::Feed => format!("feed F{}", crate::gcode::fmt(p.feed)),
                });
                ui.end_row();
            }
            if let Some(q) = sim.position() {
                for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.label(*a);
                    ui.monospace(format!("{:9.3}", q[k]));
                    ui.end_row();
                }
            }
            ui.label("Removed");
            let rem = sim.removed();
            ui.monospace(format!("{:.0} mm³ ({:.1} %)", rem, 100.0 * rem / sim.stock_volume.max(1e-9)));
            ui.end_row();
        });
        if !reached {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label("simulating…");
            });
        }
        ui.add_space(6.0);
        let m = &sim.map;
        ui.weak(format!("Stock {:.1} × {:.1} × {:.1} mm, grid {} × {} cells of {:.3} mm", m.nx as f64 * m.cell, m.ny as f64 * m.cell, m.top - m.bottom, m.nx, m.ny, m.cell));

        // comparison with the model
        if self.part.is_some() {
            ui.add_space(8.0);
            ui.heading("Compare with model");
            ui.checkbox(&mut self.compare, "Colour by distance to the model");
            ui.horizontal(|ui| {
                ui.label("At size within ±");
                ui.add(egui::DragValue::new(&mut self.tol).speed(0.005).range(0.001..=5.0).max_decimals(3).suffix(" mm"));
            });
            let c = self.compare_now();
            if self.compare {
                for (col, text) in [(AT_SIZE, "at size"), (LEFT_LO, "stock left (small)"), (LEFT_HI, "stock left (2 mm +)"), (GOUGE, "gouge (cut into the part)"), (STOCK, "uncut stock"), (CUT, "cut away (off the part)")] {
                    ui.horizontal(|ui| {
                        let (r, _) = ui.allocate_exact_size(Vec2::new(14.0, 14.0), egui::Sense::hover());
                        ui.painter().rect_filled(r, 2.0, col);
                        ui.label(text);
                    });
                }
            }
            if c.cells > 0 {
                ui.label(format!("{:.0} % of the part's top area at size", 100.0 * c.at_size as f64 / c.cells as f64));
                ui.label(format!("Most stock left: {:.3} mm", c.left.max(0.0)));
                if c.gouged > 0 {
                    ui.colored_label(GOUGE, format!("Gouged: {:.3} mm deep, {} cells", c.gouge, c.gouged));
                } else {
                    ui.label("No gouges");
                }
            }
        }

        // warnings and events
        let sim = self.sim.as_ref().unwrap();
        let prog = &sim.prog;
        if !prog.warnings.is_empty() || !sim.events.is_empty() {
            ui.add_space(8.0);
            ui.heading("Warnings");
        }
        for w in &prog.warnings {
            ui.colored_label(WARN, format!("⚠ {w}"));
        }
        let events: Vec<Event> = sim.events.values().copied().collect();
        let mut jump = None;
        for e in events.iter().take(300) {
            let text = match e.kind {
                EventKind::RapidCut => format!("⚠ line {}: rapid through material ({:.2} mm deep)", e.line + 1, e.depth),
                EventKind::TooDeep => format!("⚠ line {}: {:.2} mm deep, flutes {} mm", e.line + 1, e.depth, crate::gcode::fmt(e.flute)),
            };
            if ui.add(egui::Label::new(egui::RichText::new(text).color(if e.kind == EventKind::RapidCut { GOUGE } else { WARN })).sense(egui::Sense::click())).on_hover_text("Click to go there").clicked() {
                jump = Some(prog.start_of(e.piece + 1));
            }
        }
        if events.len() > 300 {
            ui.weak(format!("… and {} more", events.len() - 300));
        }
        if sim.time() < prog.total && events.is_empty() {
            ui.add_space(8.0);
            ui.weak("Collisions and too-deep cuts are listed here as the simulation reaches them (⏭ runs to the end).");
        }
        if let Some(t) = jump {
            self.jump(t);
        }
    }

    fn compare_now(&mut self) -> Compare {
        let (Some(sim), Some(part)) = (&self.sim, &self.part) else { return Compare::default() };
        let key = (sim.version, self.tol.to_bits());
        if let Some((k, c)) = self.compare_cache {
            if k == key {
                return c;
            }
        }
        let tol = self.tol as f32;
        let mut c = Compare::default();
        for (&z, &p) in sim.map.z.iter().zip(part) {
            if p.is_nan() {
                continue;
            }
            c.cells += 1;
            let d = z - p;
            if d < -tol {
                c.gouged += 1;
                c.gouge = c.gouge.max(-d);
            } else if d <= tol {
                c.at_size += 1;
            }
            c.left = c.left.max(d);
        }
        self.compare_cache = Some((key, c));
        c
    }

    fn scene(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.strong("3D view");
            ui.separator();
            ui.label("Path:");
            ui.selectable_value(&mut self.path, PathShow::All, "All");
            ui.selectable_value(&mut self.path, PathShow::Done, "Done");
            ui.selectable_value(&mut self.path, PathShow::Off, "Off");
            ui.add_enabled_ui(self.path != PathShow::Off, |ui| {
                ui.checkbox(&mut self.show_rapid, egui::RichText::new("Rapids").color(crate::view3d::RAPID.to_opaque()));
                ui.checkbox(&mut self.show_mill, egui::RichText::new("Milling").color(crate::view3d::BORE));
                ui.checkbox(&mut self.show_thread, egui::RichText::new("Threading").color(crate::view3d::THREAD));
            });
            ui.separator();
            ui.checkbox(&mut self.show_tool, "Tool");
            ui.checkbox(&mut self.show_stock, "Stock");
            ui.separator();
            self.view.view_buttons(ui);
            ui.separator();
            ui.weak("Space: play · ←/→: step a move");
        });
        let Some(sim) = &self.sim else { return };
        let m = &sim.map;
        let (mut lo, mut hi) = ([m.x0, m.y0, m.bottom], [m.x0 + m.nx as f64 * m.cell, m.y0 + m.ny as f64 * m.cell, m.top]);
        if let Some((a, b)) = sim.prog.bounds {
            if sim.prog.pieces.len() > 0 && m.nx <= 2 {
                (lo, hi) = (a, b);
            }
        }
        // order of the surface quads: far to near for the current view
        let d = self.view.view_dir();
        let bucket = if d[0].hypot(d[1]) < 1e-3 { 0 } else { (d[1].atan2(d[0]).to_degrees() * 2.0).round() as i32 + 1000 };
        let okey = (m.nx, m.ny, bucket);
        if self.order.as_ref().map_or(true, |(k, _)| *k != okey) {
            self.order = Some((okey, quad_order(m.nx, m.ny, d)));
        }
        let order = &self.order.as_ref().unwrap().1;
        let part = if self.compare { self.part.as_deref() } else { None };
        let tol = self.tol as f32;
        let (path, show_tool, show_stock, origin) = (self.path, self.show_tool, self.show_stock, self.origin);
        let kinds = PathKinds { rapid: self.show_rapid, mill: self.show_mill, thread: self.show_thread };
        let grid_z = m.bottom as f32;
        self.view.scene(ui, f3(lo), f3(hi), grid_z, |v, painter, rect| {
            if show_stock && m.nx > 2 {
                draw_stock(v, painter, rect, sim, order, part, tol);
            }
            if path != PathShow::Off {
                draw_path(v, painter, rect, sim, path == PathShow::All, kinds);
            }
            if let Some(o) = origin {
                v.draw_origin(painter, rect, f3(o));
            }
            if show_tool {
                draw_tool(v, painter, rect, sim);
            }
        });
    }

    /// The program listing for the side panel, following playback.
    pub fn code_panel(&mut self, ui: &mut egui::Ui) {
        let Some(sim) = &self.sim else { return };
        let cur = sim.current().filter(|_| sim.time() > 0.0).map(|p| p.line);
        ui.horizontal(|ui| {
            ui.heading("G-code");
            ui.checkbox(&mut self.follow, "Follow");
            match cur {
                Some(l) => ui.weak(format!("line {}", l + 1)),
                None => ui.weak("click a line to run to it"),
            };
        });
        if cur != self.last_line {
            self.last_line = cur;
            if self.follow {
                self.scroll_to = cur;
            }
        }
        let lines = &sim.prog.lines;
        let row_h = ui.text_style_height(&egui::TextStyle::Monospace);
        let gap = ui.spacing().item_spacing.y;
        let mut area = egui::ScrollArea::both().auto_shrink([false, false]);
        if let Some(l) = self.scroll_to.take() {
            let view_h = ui.available_height();
            area = area.vertical_scroll_offset(((l as f32) * (row_h + gap) - view_h / 3.0).max(0.0));
        }
        let mut clicked = None;
        let digits = lines.len().max(1).to_string().len();
        area.show_rows(ui, row_h, lines.len(), |ui, range| {
            for i in range {
                let text = egui::RichText::new(format!("{:>w$}  {}", i + 1, lines[i], w = digits)).monospace();
                let sel = Some(i) == cur;
                if ui.add(egui::SelectableLabel::new(sel, text)).clicked() {
                    clicked = Some(i);
                }
            }
        });
        if let Some(l) = clicked {
            // run to the end of that line's move (or the next move after it)
            let p = &sim.prog;
            let t = p.end_of_line(l).unwrap_or_else(|| p.start_of(p.pieces.partition_point(|q| q.line < l)));
            self.jump(t);
        }
    }
}

/// Surface quads (i, j) with i < nx-1, j < ny-1, far to near along `dir`.
fn quad_order(nx: usize, ny: usize, dir: [f32; 3]) -> Vec<u32> {
    if nx < 2 || ny < 2 {
        return vec![];
    }
    let (qx, qy) = (nx - 1, ny - 1);
    let mut v: Vec<(f32, u32)> = (0..qx * qy).map(|q| ((q % qx) as f32 * dir[0] + (q / qx) as f32 * dir[1], q as u32)).collect();
    v.sort_unstable_by(|a, b| b.0.total_cmp(&a.0));
    v.into_iter().map(|(_, q)| q).collect()
}

/// Light from the upper left, towards the viewer (view space).
fn lighting(v: &View3d, n: [f32; 3]) -> f32 {
    let nv = v.rot(n);
    let l = [-0.35f32, -0.75, 0.56];
    let len = (l[0] * l[0] + l[1] * l[1] + l[2] * l[2]).sqrt();
    let d = (nv[0] * l[0] + nv[1] * l[1] + nv[2] * l[2]) / len;
    0.32 + 0.68 * d.max(0.0)
}

fn draw_stock(v: &View3d, painter: &egui::Painter, rect: Rect, sim: &Sim, order: &[u32], part: Option<&[f32]>, tol: f32) {
    let m = &sim.map;
    let (nx, ny) = (m.nx, m.ny);
    let c = m.cell as f32;
    let (x0, y0) = (m.x0 as f32 + c / 2.0, m.y0 as f32 + c / 2.0);
    let (top, bottom) = (m.top as f32, m.bottom as f32);
    let z = |i: usize, j: usize| m.z[j * nx + i];
    let base = |idx: usize| -> Color32 {
        let h = m.z[idx];
        if h <= bottom + 1e-4 {
            return FLOOR;
        }
        if let Some(p) = part.map(|p| p[idx]).filter(|p| !p.is_nan()) {
            let d = h - p;
            return if d < -tol {
                GOUGE
            } else if d <= tol {
                AT_SIZE
            } else {
                mix(LEFT_LO, LEFT_HI, (d - tol) / 2.0)
            };
        }
        if h >= top - 1e-4 {
            STOCK
        } else {
            CUT
        }
    };
    let mut mesh = egui::Mesh::default();
    mesh.vertices.reserve(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (il, ir) = (i.saturating_sub(1), (i + 1).min(nx - 1));
            let (jd, ju) = (j.saturating_sub(1), (j + 1).min(ny - 1));
            let gx = (z(ir, j) - z(il, j)) / (c * (ir - il).max(1) as f32);
            let gy = (z(i, ju) - z(i, jd)) / (c * (ju - jd).max(1) as f32);
            let l = (gx * gx + gy * gy + 1.0).sqrt();
            let k = lighting(v, [-gx / l, -gy / l, 1.0 / l]);
            let p = v.project(rect, [x0 + i as f32 * c, y0 + j as f32 * c, z(i, j)]);
            mesh.colored_vertex(p, shade(base(j * nx + i), k));
        }
    }
    // side walls: (vertex indices along the edge, outward normal)
    let sides: [(Vec<(usize, usize)>, [f32; 3]); 4] = [
        ((0..nx).map(|i| (i, 0)).collect(), [0.0, -1.0, 0.0]),
        ((0..nx).map(|i| (i, ny - 1)).collect(), [0.0, 1.0, 0.0]),
        ((0..ny).map(|j| (0, j)).collect(), [-1.0, 0.0, 0.0]),
        ((0..ny).map(|j| (nx - 1, j)).collect(), [1.0, 0.0, 0.0]),
    ];
    let d = v.view_dir();
    let wall = |mesh: &mut egui::Mesh, side: &(Vec<(usize, usize)>, [f32; 3])| {
        let k = lighting(v, side.1);
        for w in side.0.windows(2) {
            let b = mesh.vertices.len() as u32;
            for &(i, j) in &[w[0], w[1]] {
                let h = z(i, j);
                let col = shade(if h >= top - 1e-4 { STOCK } else { CUT }, k * 0.9);
                let (x, y) = (x0 + i as f32 * c, y0 + j as f32 * c);
                mesh.colored_vertex(v.project(rect, [x, y, h]), col);
                mesh.colored_vertex(v.project(rect, [x, y, bottom]), shade(STOCK, k * 0.7));
            }
            mesh.add_triangle(b, b + 1, b + 2);
            mesh.add_triangle(b + 1, b + 3, b + 2);
        }
    };
    // far walls, then the surface, then near walls
    for s in sides.iter().filter(|s| s.1[0] * d[0] + s.1[1] * d[1] > 0.0) {
        wall(&mut mesh, s);
    }
    let qx = nx - 1;
    mesh.indices.reserve(order.len() * 6);
    for &q in order {
        let (i, j) = (q as usize % qx, q as usize / qx);
        let a = (j * nx + i) as u32;
        let (b, cc, dd) = (a + 1, a + nx as u32 + 1, a + nx as u32);
        mesh.indices.extend_from_slice(&[a, b, cc, a, cc, dd]);
    }
    for s in sides.iter().filter(|s| s.1[0] * d[0] + s.1[1] * d[1] <= 0.0) {
        wall(&mut mesh, s);
    }
    if d[2] > 0.0 {
        // looking up from below: the bottom face covers everything
        let k = lighting(v, [0.0, 0.0, -1.0]);
        let b = mesh.vertices.len() as u32;
        let (x1, y1) = (x0 + (nx - 1) as f32 * c, y0 + (ny - 1) as f32 * c);
        for p in [[x0, y0], [x1, y0], [x1, y1], [x0, y1]] {
            mesh.colored_vertex(v.project(rect, [p[0], p[1], bottom]), shade(STOCK, k * 0.8));
        }
        mesh.add_triangle(b, b + 1, b + 2);
        mesh.add_triangle(b, b + 2, b + 3);
    }
    painter.add(egui::Shape::mesh(mesh));
}

/// Which kinds of move the path shows.
#[derive(Clone, Copy)]
struct PathKinds {
    rapid: bool,
    mill: bool,
    thread: bool,
}

/// A piece's kind: rapid, milling (feed) or threading (feed with a thread mill).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PathKind {
    Rapid,
    Mill,
    Thread,
}

fn path_kind(sim: &Sim, p: &crate::sim::Piece) -> PathKind {
    match p.motion {
        Motion::Rapid => PathKind::Rapid,
        Motion::Feed if p.tool.is_some_and(|t| sim.prog.tools[t].thread) => PathKind::Thread,
        Motion::Feed => PathKind::Mill,
    }
}

/// Path done so far (bright) and, with `all`, the rest (faint), for the
/// kinds of move switched on.
fn draw_path(v: &View3d, painter: &egui::Painter, rect: Rect, sim: &Sim, all: bool, kinds: PathKinds) {
    let ps = &sim.prog.pieces;
    let cut = sim.done.min(ps.len());
    let now = sim.position();
    let mut run: Vec<Pos2> = vec![];
    let mut run_style: Option<Stroke> = None;
    let flush = |run: &mut Vec<Pos2>, style: Option<Stroke>| {
        if let (Some(s), true) = (style, run.len() >= 2) {
            painter.add(egui::Shape::line(std::mem::take(run), s));
        }
        run.clear();
    };
    // the same colours as the Step Milling view; what is still to come is faint
    let style = |k: PathKind, done: bool| -> Stroke {
        match (k, done) {
            (PathKind::Rapid, true) => Stroke::new(1.0_f32, crate::view3d::RAPID),
            (PathKind::Mill, true) => Stroke::new(1.3_f32, crate::view3d::BORE),
            (PathKind::Thread, true) => Stroke::new(1.5_f32, crate::view3d::THREAD),
            (PathKind::Rapid, false) => Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(200, 90, 90, 45)),
            (PathKind::Mill, false) => Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(150, 170, 200, 60)),
            (PathKind::Thread, false) => Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(255, 190, 110, 70)),
        }
    };
    let shown = |k: PathKind| match k {
        PathKind::Rapid => kinds.rapid,
        PathKind::Mill => kinds.mill,
        PathKind::Thread => kinds.thread,
    };
    let end = if all { ps.len() } else { (cut + 1).min(ps.len()) };
    let mut last_b: Option<P3> = None;
    for (k, p) in ps.iter().enumerate().take(end) {
        let kind = path_kind(sim, p);
        if !shown(kind) {
            continue;
        }
        // the running piece splits at the tool
        let mut parts = [(p.a, p.b, k < cut); 2];
        let n = if k == cut {
            match now {
                Some(q) if all => {
                    parts = [(p.a, q, true), (q, p.b, false)];
                    2
                }
                Some(q) => {
                    parts[0] = (p.a, q, true);
                    1
                }
                None => 0,
            }
        } else {
            1
        };
        for &(a, b, done) in &parts[..n] {
            let s = style(kind, done);
            if run_style != Some(s) || last_b != Some(a) {
                flush(&mut run, run_style);
                run_style = Some(s);
                run.push(v.project(rect, f3(a)));
            }
            run.push(v.project(rect, f3(b)));
            last_b = Some(b);
        }
    }
    flush(&mut run, run_style);
}

/// The tool at its current position: flutes and a stub of shank.
fn draw_tool(v: &View3d, painter: &egui::Painter, rect: Rect, sim: &Sim) {
    let Some(q) = sim.position() else { return };
    let tool = sim.current().and_then(|p| p.tool).map(|k| &sim.prog.tools[k]);
    let Some(tool) = tool else {
        let p = v.project(rect, f3(q));
        let s = Stroke::new(1.5_f32, WARN);
        painter.line_segment([p - Vec2::new(6.0, 0.0), p + Vec2::new(6.0, 0.0)], s);
        painter.line_segment([p - Vec2::new(0.0, 6.0), p + Vec2::new(0.0, 6.0)], s);
        return;
    };
    let r = tool.shape.radius() as f32;
    let tip = q[2] as f32;
    let ring = |z: f32, rr: f32| -> Vec<Pos2> {
        (0..32)
            .map(|k| {
                let a = k as f32 / 32.0 * std::f32::consts::TAU;
                v.project(rect, [q[0] as f32 + rr * a.cos(), q[1] as f32 + rr * a.sin(), z])
            })
            .collect()
    };
    let flute = (tool.flute as f32).min(60.0).max(r);
    let (cut_rings, shank_from) = match tool.shape {
        Shape::Flat { .. } => (vec![ring(tip, r), ring(tip + flute, r)], tip + flute),
        Shape::Vee { tip: t, .. } => {
            let l = tool.shape.taper_len() as f32;
            (vec![ring(tip, t as f32), ring(tip + l, r)], tip + l)
        }
    };
    let shank_len = (4.0 * r).max(12.0);
    let parts = [(cut_rings, FLUTES), (vec![ring(shank_from, r), ring(shank_from + shank_len, r)], SHANK)];
    for (rings, col) in parts {
        let pts: Vec<Pos2> = rings.iter().flatten().copied().collect();
        let hull = convex_hull(pts);
        if hull.len() >= 3 {
            painter.add(egui::Shape::convex_polygon(hull, col, Stroke::new(1.0_f32, shade(col, 0.6))));
        }
        // top rim for depth
        let mut top = rings.last().unwrap().clone();
        top.push(top[0]);
        painter.add(egui::Shape::line(top, Stroke::new(1.0_f32, shade(col, 0.6))));
    }
    painter.circle_filled(v.project(rect, f3(q)), 2.5, Color32::from_rgb(255, 255, 255));
}

/// Convex hull (monotone chain), counter-clockwise on screen.
fn convex_hull(mut p: Vec<Pos2>) -> Vec<Pos2> {
    p.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let cross = |o: Pos2, a: Pos2, b: Pos2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut h: Vec<Pos2> = Vec::with_capacity(p.len() * 2);
    for pass in 0..2 {
        let start = h.len();
        let it: Box<dyn Iterator<Item = &Pos2>> = if pass == 0 { Box::new(p.iter()) } else { Box::new(p.iter().rev()) };
        for &q in it {
            while h.len() >= start + 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 {
                h.pop();
            }
            h.push(q);
        }
        h.pop();
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quads_are_drawn_far_to_near() {
        // looking along +X (away): larger i is farther, drawn first
        let o = quad_order(4, 3, [1.0, 0.0, -0.5]);
        assert_eq!(o.len(), 3 * 2);
        let i: Vec<u32> = o.iter().map(|q| q % 3).collect();
        assert!(i.windows(2).all(|w| w[0] >= w[1]), "{i:?}");
    }

    #[test]
    fn hull_of_a_square_with_inner_points() {
        let pts = vec![Pos2::new(0.0, 0.0), Pos2::new(1.0, 0.0), Pos2::new(1.0, 1.0), Pos2::new(0.0, 1.0), Pos2::new(0.5, 0.5), Pos2::new(0.2, 0.7)];
        let h = convex_hull(pts);
        assert_eq!(h.len(), 4);
    }

    /// Drive the tab headless for a few frames on a real job: build, play,
    /// run to the end, scrub back, other views - no panics, sane state.
    #[test]
    fn tab_runs_headless_on_a_step_job() {
        use crate::stepmill::*;
        let model = crate::step::parse_model(include_str!("../testdata/a.step")).unwrap();
        let stock = Stock { z: 2.0, ..Default::default() }.bounds(model.min, model.max);
        let op = RoughOp { cutter: crate::model::Cutter::C3x17, start_z: stock.1[2], bottom_z: model.min[2], faces: rough_faces(&model), stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]], ..Default::default() };
        let job = StepJob { ops: vec![StepOp { enabled: true, kind: OpKind::Rough(op) }], ..Default::default() };
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let code = generate(&job, &s).unwrap();
        let inp = Inputs { step_code: &code, settings: &s, model: Some(&model), step_stock: Some(stock), step_origin: [0.0; 3], nest_code: "", nest_stock: None, nest_origin: [0.0; 3], nest_copies: &[], stock_tris: None };
        let ctx = egui::Context::default();
        let mut tab = SimTab::default();
        let frame = |tab: &mut SimTab| {
            let _ = ctx.run(egui::RawInput { screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400.0, 850.0))), ..Default::default() }, |ctx| {
                egui::SidePanel::right("code").show(ctx, |ui| tab.code_panel(ui));
                egui::CentralPanel::default().show(ctx, |ui| tab.ui(ui, &inp));
            });
        };
        frame(&mut tab);
        let total = tab.sim.as_ref().unwrap().prog.total;
        assert!(total > 10.0 && tab.part.is_some());
        tab.playing = true;
        tab.speed = 50.0;
        for _ in 0..3 {
            frame(&mut tab);
        }
        tab.jump(total);
        for _ in 0..200 {
            frame(&mut tab);
            if tab.sim.as_ref().unwrap().time() >= total {
                break;
            }
        }
        let sim = tab.sim.as_ref().unwrap();
        assert_eq!(sim.time(), total);
        assert!(sim.removed() > 100.0);
        let c = tab.compare_now();
        assert!(c.cells > 0 && c.gouged == 0, "{} gouged", c.gouged);
        // scrub back, other path modes, view from below, custom stock rebuild
        tab.jump(total * 0.3);
        tab.path = PathShow::Done;
        tab.view.yaw = 2.0;
        tab.view.pitch = -0.5;
        frame(&mut tab);
        frame(&mut tab);
        assert!((tab.sim.as_ref().unwrap().time() - total * 0.3).abs() < 1e-6);
        tab.custom_stock = true;
        tab.stock.1[2] += 1.0;
        frame(&mut tab);
        frame(&mut tab);
        assert!((tab.sim.as_ref().unwrap().map.top - tab.stock.1[2]).abs() < 1e-9);
        // nothing to simulate: an empty file
        tab.source = Source::File;
        frame(&mut tab);
        assert!(tab.sim.as_ref().unwrap().prog.pieces.is_empty());
    }

    #[test]
    fn thread_mill_feeds_are_threading() {
        // T4 end mill bores, T13 (M3 thread mill) threads
        let code = "T4 M6\nG0 X5 Y5 Z1\nG1 Z-1 F100\nT13 M6\nG0 X5 Y5 Z1\nG1 Z-1 F100\nG0 Z5\n";
        let sim = Sim::new(Program::parse(code, &Settings::default()), [0.0; 3], [10.0, 10.0, 0.0], 50);
        let kinds: Vec<PathKind> = sim.prog.pieces.iter().map(|p| path_kind(&sim, p)).collect();
        // (the first move has no known start, so it is not a piece)
        assert_eq!(kinds, vec![PathKind::Mill, PathKind::Rapid, PathKind::Thread, PathKind::Rapid]);
    }

    #[test]
    fn clock_format() {
        assert_eq!(clock(83.42), "1:23.4");
        assert_eq!(clock(59.97), "1:00.0");
        assert_eq!(clock(3725.0), "1:02:05");
    }
}
