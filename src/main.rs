//! stepCAM - G-code generator for thread milling and 2.5D step milling.

mod gcode;
mod model;
mod step;
mod stepmill;
mod toolpath;
mod view3d;

use eframe::egui;
use model::{Cutter, Job, Settings, ThreadGroup, ThreadSize};
use stepmill::{Entry, FaceOp, MillTab, OpKind, PocketOp, ProfileOp, ProfileSide, RoughOp, StepJob, StepOp, Strategy};
use view3d::{ModelViz, Pick};

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 850.0])
            .with_title("stepCAM"),
        ..Default::default()
    };
    eframe::run_native("stepCAM", options, Box::new(|_cc| Ok(Box::new(StepCamApp::default()))))
}

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    ThreadMilling,
    StepMilling,
    Settings,
}

/// Highlighted model geometry in the step milling view.
#[derive(Default)]
struct Selection {
    edges: Vec<usize>,
    face: Option<usize>,
}

enum OpAction {
    Select(usize),
    Remove(usize),
    MoveUp(usize),
    MoveDown(usize),
}

struct StepCamApp {
    job: Job,
    tab: Tab,
    gcode: String,
    /// Errors from the last (automatic) generation.
    gen_errors: Vec<String>,
    /// Job the current G-code was generated from.
    generated_from: Option<Job>,
    toolpath: toolpath::Toolpath,
    view: view3d::View3d,
    status: String,
    status_is_error: bool,
    job_path: String,
    gcode_path: String,

    // step milling
    /// Model as read from the file, before placement.
    step_raw: Option<step::Model>,
    /// Model in machine coordinates (raw + job placement).
    step_model: Option<step::Model>,
    /// Value of the "Set Z" input in the surface menu.
    set_z_value: f64,
    /// Stock top seen last frame (ops starting there follow stock changes).
    last_stock_top: Option<f64>,
    /// Part bottom seen last frame (rough ops ending there follow the part).
    last_part_bottom: Option<f64>,
    step_gcode: String,
    step_errors: Vec<String>,
    step_generated_from: Option<(StepJob, Settings)>,
    step_toolpath: toolpath::Toolpath,
    step_view: view3d::View3d,
    step_gcode_path: String,
    sel: Selection,
    menu_pick: Option<Pick>,
}

impl Default for StepCamApp {
    fn default() -> Self {
        let mut job = Job::default();
        job.groups.push(ThreadGroup {
            locations: "10, 10\n20, 10\n".into(),
            ..Default::default()
        });
        StepCamApp {
            job,
            tab: Tab::ThreadMilling,
            gcode: String::new(),
            gen_errors: Vec::new(),
            generated_from: None,
            toolpath: Default::default(),
            view: Default::default(),
            status: String::new(),
            status_is_error: false,
            job_path: "job.stepcam.json".into(),
            gcode_path: "output.nc".into(),
            step_raw: None,
            step_model: None,
            set_z_value: 0.0,
            last_stock_top: None,
            last_part_bottom: None,
            step_gcode: String::new(),
            step_errors: Vec::new(),
            step_generated_from: None,
            step_toolpath: Default::default(),
            step_view: view3d::View3d::with_labels(["Milling", ""]),
            step_gcode_path: "step.nc".into(),
            sel: Selection::default(),
            menu_pick: None,
        }
    }
}

enum RowAction {
    Remove(usize),
    Duplicate(usize),
    MoveUp(usize),
    MoveDown(usize),
}

impl StepCamApp {
    fn set_status(&mut self, msg: impl Into<String>, error: bool) {
        self.status = msg.into();
        self.status_is_error = error;
    }

    /// Regenerate the G-code and toolpath if the job changed since last time.
    fn regenerate_if_changed(&mut self) {
        if self.generated_from.as_ref() == Some(&self.job) {
            return;
        }
        let was_empty = self.toolpath.is_empty();
        match gcode::generate(&self.job) {
            Ok(code) => {
                self.toolpath = toolpath::parse(&code);
                self.gcode = code;
                self.gen_errors.clear();
            }
            Err(errs) => {
                self.gcode.clear();
                self.toolpath = Default::default();
                self.gen_errors = errs;
            }
        }
        if was_empty && !self.toolpath.is_empty() {
            self.view.needs_fit = true;
        }
        self.generated_from = Some(self.job.clone());
    }

    /// Regenerate the step milling program if its operations or settings changed.
    /// Stock box for the current model.
    fn stock_box(&self) -> Option<([f64; 3], [f64; 3])> {
        self.step_model.as_ref().map(|m| self.job.step.stock.bounds(m.min, m.max))
    }

    /// Keep operations in step with the stock: facing covers the stock, and
    /// ops that started at the old stock top start at the new one.
    fn sync_stock(&mut self) {
        let Some((smin, smax)) = self.stock_box() else { return };
        let region = [smin[0], smin[1], smax[0], smax[1]];
        let model = self.step_model.as_ref().unwrap();
        let part_bottom = model.min[2];
        let mut faces = None;
        for op in &mut self.job.step.ops {
            match &mut op.kind {
                OpKind::Face(f) => f.region = region,
                OpKind::Rough(r) => {
                    let f = faces.get_or_insert_with(|| stepmill::rough_faces(model));
                    if &r.faces != f {
                        r.faces = f.clone();
                    }
                    // a bottom at the part bottom follows the part
                    if let Some(old) = self.last_part_bottom {
                        if (r.bottom_z - old).abs() < 1e-6 {
                            r.bottom_z = part_bottom;
                        }
                    }
                }
                _ => {}
            }
        }
        self.last_part_bottom = Some(part_bottom);
        if let Some(old) = self.last_stock_top {
            if (old - smax[2]).abs() > 1e-9 {
                for op in &mut self.job.step.ops {
                    let start = match &mut op.kind {
                        OpKind::Face(f) => &mut f.start_z,
                        OpKind::Pocket(p) => &mut p.start_z,
                        OpKind::Profile(p) => &mut p.start_z,
                        OpKind::Rough(r) => &mut r.start_z,
                        OpKind::Tab(_) => continue,
                    };
                    if (*start - old).abs() < 1e-6 {
                        *start = smax[2];
                    }
                }
            }
        }
        self.last_stock_top = Some(smax[2]);
    }

    /// Highest point of the stock and part (falls back to the ops' Start Z
    /// when no model is loaded).
    fn step_top(&self) -> f64 {
        match (&self.step_model, self.stock_box()) {
            (Some(m), Some((_, smax))) => m.max[2].max(smax[2]),
            _ => self
                .job
                .step
                .ops
                .iter()
                .filter_map(|o| match &o.kind {
                    OpKind::Face(f) => Some(f.start_z),
                    OpKind::Pocket(p) => Some(p.start_z),
                    OpKind::Profile(p) => Some(p.start_z),
                    OpKind::Rough(r) => Some(r.start_z),
                    OpKind::Tab(t) => Some(t.top()),
                })
                .fold(0.0, f64::max),
        }
    }

    /// Settings for step milling: the rapid plane sits `rapid_above` over
    /// the highest point of the stock / part.
    fn step_settings(&self) -> Settings {
        let mut s = self.job.settings.clone();
        s.safe_z = self.step_top() + self.job.step.rapid_above;
        s
    }

    fn regenerate_step_if_changed(&mut self) {
        self.sync_stock();
        let settings = self.step_settings();
        if self.step_generated_from.as_ref().map_or(false, |(j, s)| j == &self.job.step && s == &settings) {
            return;
        }
        let was_empty = self.step_toolpath.is_empty();
        match stepmill::generate(&self.job.step, &settings) {
            Ok(code) => {
                self.step_toolpath = toolpath::parse(&code);
                self.step_gcode = code;
                self.step_errors.clear();
            }
            Err(errs) => {
                self.step_gcode.clear();
                self.step_toolpath = Default::default();
                self.step_errors = errs;
            }
        }
        if was_empty && !self.step_toolpath.is_empty() && self.step_model.is_none() {
            self.step_view.needs_fit = true;
        }
        self.step_generated_from = Some((self.job.step.clone(), settings));
    }

    fn is_step_tab(&self) -> bool {
        self.tab == Tab::StepMilling
    }

    fn save_gcode(&mut self) {
        let (code, errors, path) = if self.is_step_tab() {
            self.regenerate_step_if_changed();
            (&self.step_gcode, &self.step_errors, self.step_gcode_path.clone())
        } else {
            self.regenerate_if_changed();
            (&self.gcode, &self.gen_errors, self.gcode_path.clone())
        };
        if !errors.is_empty() {
            self.set_status("Cannot save G-code: fix the errors listed above", true);
            return;
        }
        match std::fs::write(&path, code) {
            Ok(()) => self.set_status(format!("Saved G-code to {path}"), false),
            Err(e) => self.set_status(format!("Error saving {path}: {e}"), true),
        }
    }

    fn load_step_model(&mut self, path: &str) -> bool {
        match step::load(path) {
            Ok(m) => {
                self.set_status(
                    format!("Imported {path}: {} faces, {} edges{}", m.faces.len(), m.edges.len(), if m.unit_scale != 1.0 { format!(" (scaled ×{} to mm)", m.unit_scale) } else { String::new() }),
                    false,
                );
                self.step_model = Some(m.transformed(&self.job.step.placement));
                self.step_raw = Some(m);
                self.sel = Selection::default();
                self.step_view.needs_fit = true;
                true
            }
            Err(e) => {
                self.set_status(format!("Could not import STEP: {e}"), true);
                false
            }
        }
    }

    /// Move the model to a new placement and carry the operations with it.
    fn apply_placement(&mut self, new: step::Placement, what: &str) {
        let Some(raw) = &self.step_raw else { return };
        let old = self.job.step.placement;
        let model = raw.transformed(&new);
        let warnings = refresh_ops(&mut self.job.step.ops, &model, &old, &new);
        self.job.step.placement = new;
        self.step_model = Some(model);
        self.sel = Selection::default();
        self.menu_pick = None;
        self.step_view.needs_fit = true;
        if warnings.is_empty() {
            self.set_status(what, false);
        } else {
            self.set_status(format!("{what} - {}", warnings.join("; ")), true);
        }
    }

    fn menu_action(&mut self, action: MenuAction) {
        let Some(model) = &self.step_model else { return };
        let pl = self.job.step.placement;
        match action {
            MenuAction::SetZ(fi, z) => {
                let dz = z - model.faces[fi].z();
                self.apply_placement(pl.translated([0.0, 0.0, dz]), &format!("Model moved {} mm in Z - surface now at Z{}", gcode::fmt(dz), gcode::fmt(z)));
            }
            MenuAction::LayFlat(fi) => {
                let r = step::rotation_to_down(model.faces[fi].normal);
                let centre = |m: &step::Model| [(m.min[0] + m.max[0]) / 2.0, (m.min[1] + m.max[1]) / 2.0, m.max[2]];
                let before = centre(model);
                let rotated = pl.rotated(&r);
                let after = centre(&self.step_raw.as_ref().unwrap().transformed(&rotated));
                // keep the XY centre where it was, put the new top at Z0
                let moved = rotated.translated([before[0] - after[0], before[1] - after[1], -after[2]]);
                self.apply_placement(moved, "Model laid flat on the surface (top at Z0)");
            }
            MenuAction::ResetPlacement => self.apply_placement(Default::default(), "Model placement reset to the STEP file coordinates"),
        }
    }

    fn import_step(&mut self) {
        let Some(path) = rfd::FileDialog::new().set_title("Import STEP file").add_filter("STEP", &["step", "stp", "STEP", "STP"]).add_filter("All files", &["*"]).pick_file() else {
            return;
        };
        let path = path.to_string_lossy().to_string();
        let new_file = path != self.job.step.step_path;
        let old_placement = self.job.step.placement;
        if new_file {
            self.job.step.placement = Default::default();
        }
        if !self.load_step_model(&path) {
            self.job.step.placement = old_placement;
        } else {
            if new_file && !self.job.step.ops.is_empty() {
                self.job.step.ops.clear();
                self.status.push_str(" - operations cleared (they belonged to the previous model)");
            }
            self.job.step.step_path = path;
        }
    }

    fn save_job(&mut self) {
        let res = serde_json::to_string_pretty(&self.job)
            .map_err(|e| e.to_string())
            .and_then(|s| std::fs::write(&self.job_path, s).map_err(|e| e.to_string()));
        match res {
            Ok(()) => self.set_status(format!("Saved job to {}", self.job_path), false),
            Err(e) => self.set_status(format!("Error saving job: {e}"), true),
        }
    }

    fn load_job(&mut self) {
        let res = std::fs::read_to_string(&self.job_path)
            .map_err(|e| e.to_string())
            .and_then(|s| serde_json::from_str::<Job>(&s).map_err(|e| e.to_string()));
        match res {
            Ok(mut job) => {
                job.normalize();
                self.job = job;
                self.view.needs_fit = true;
                self.set_status(format!("Loaded job from {}", self.job_path), false);
                self.step_model = None;
                self.step_raw = None;
                let sp = self.job.step.step_path.clone();
                if !sp.is_empty() {
                    let msg = self.status.clone();
                    if self.load_step_model(&sp) {
                        self.set_status(format!("{msg}; model {sp}"), false);
                    }
                }
            }
            Err(e) => self.set_status(format!("Error loading job: {e}"), true),
        }
    }

    fn file_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Job file:");
            ui.add(egui::TextEdit::singleline(&mut self.job_path).desired_width(220.0));
            if ui.button("Load").clicked() {
                self.load_job();
            }
            if ui.button("Save").clicked() {
                self.save_job();
            }
            ui.separator();
            ui.label("G-code file:");
            let path = if self.tab == Tab::StepMilling { &mut self.step_gcode_path } else { &mut self.gcode_path };
            ui.add(egui::TextEdit::singleline(path).desired_width(220.0));
            if ui.button("💾 Save G-code").clicked() {
                self.save_gcode();
            }
        });
        let (code, errors) = if self.is_step_tab() { (&self.step_gcode, &self.step_errors) } else { (&self.gcode, &self.gen_errors) };
        if errors.is_empty() {
            ui.label(format!("G-code: {} lines (updates automatically)", code.lines().count()));
        } else {
            ui.colored_label(ui.visuals().error_fg_color, format!("Cannot generate G-code:\n  {}", errors.join("\n  ")));
        }
        if !self.status.is_empty() {
            let color = if self.status_is_error {
                ui.visuals().error_fg_color
            } else {
                ui.visuals().text_color()
            };
            ui.colored_label(color, &self.status);
        }
    }

    fn thread_milling_tab(&mut self, ui: &mut egui::Ui) {
        // 3D view of the generated program, docked at the bottom of the tab
        let holes: Vec<view3d::HoleViz> = self
            .job
            .groups
            .iter()
            .flat_map(|g| {
                let locs = g.parse_locations().unwrap_or_default();
                let (dia, top, bottom) = (g.thread.hole_dia() as f32, g.start_z as f32, (g.start_z - g.hole_depth()) as f32);
                locs.into_iter().map(move |(x, y)| view3d::HoleViz { x: x as f32, y: y as f32, dia, top, bottom })
            })
            .collect();
        egui::TopBottomPanel::bottom("view3d_panel")
            .resizable(true)
            .default_height(420.0)
            .min_height(150.0)
            .show_inside(ui, |ui| {
                ui.add_space(4.0);
                self.view.ui(ui, &self.toolpath, &holes, None);
            });

        ui.horizontal(|ui| {
            ui.heading("Thread Locations");
            if ui.button("➕ Add group").clicked() {
                // new groups copy the previous group's start Z, and use the global down angle
                let start_z = self.job.groups.last().map(|g| g.start_z).unwrap_or(0.0);
                let thread = ThreadGroup::default().thread;
                self.job.groups.push(ThreadGroup {
                    start_z,
                    ramp_angle: self.job.settings.ramp_angle,
                    cutter: Cutter::default_for(&self.job.settings.cutters, thread),
                    ..Default::default()
                });
            }
        });
        ui.horizontal(|ui| {
            let s = &mut self.job.settings;
            ui.label("Spindle speed");
            ui.add(egui::DragValue::new(&mut s.spindle_rpm).range(1.0..=100000.0).speed(10.0).max_decimals(0).suffix(" rpm"))
                .on_hover_text("Spindle speed for milling and threading");
            ui.add_space(12.0);
            ui.label("Rapid plane Z");
            ui.add(egui::DragValue::new(&mut s.safe_z).speed(0.1).max_decimals(3).suffix(" mm"))
                .on_hover_text("Thread milling: absolute Z for rapids between groups and tool changes (step milling sets its own above the stock)");
            ui.add_space(12.0);
            ui.label("Milling feed");
            ui.add(egui::DragValue::new(&mut s.bore_feed).range(1.0..=100000.0).speed(1.0).max_decimals(1).suffix(" mm/min"))
                .on_hover_text("Helical boring feed at the cutter centre");
            ui.add_space(12.0);
            ui.label("Threading feed");
            ui.add(egui::DragValue::new(&mut s.thread_feed).range(1.0..=100000.0).speed(1.0).max_decimals(1).suffix(" mm/min"))
                .on_hover_text("Thread milling feed at the cutting edge (centre feed is reduced automatically)");
            ui.add_space(12.0);
            ui.label("Milling down angle");
            ui.add(egui::DragValue::new(&mut s.ramp_angle).range(0.1..=45.0).speed(0.1).max_decimals(2).suffix("°"))
                .on_hover_text("Default helical descent angle for new groups");
            if ui.button("Apply to all groups").on_hover_text("Set every group's descent angle to this value").clicked() {
                let a = s.ramp_angle;
                for g in &mut self.job.groups {
                    g.ramp_angle = a;
                }
            }
        });
        ui.label(
            "One location per line as \"x, y\" (also accepts \"X10 Y20\"). Hole depth = thread length + 1 pitch; \
             the thread is milled bottom-up and stops one pitch above the bottom of the hole.",
        );
        ui.add_space(4.0);

        let settings = self.job.settings.clone();
        let mut action = None;

        egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
            egui::Grid::new("thread_groups")
                .striped(true)
                .num_columns(10)
                .spacing([12.0, 8.0])
                .min_row_height(24.0)
                .show(ui, |ui| {
                    for h in ["Group", "Thread", "Hole start Z", "Thread length", "Descent angle", "Milling cutter", "Locations (x, y)", "Hole", "Status", ""] {
                        ui.strong(h);
                    }
                    ui.end_row();

                    let n = self.job.groups.len();
                    for (i, g) in self.job.groups.iter_mut().enumerate() {
                        ui.label(format!("{}", i + 1));

                        // thread size
                        let before = g.thread;
                        egui::ComboBox::from_id_salt(("thread", i))
                            .selected_text(g.thread.name())
                            .width(70.0)
                            .show_ui(ui, |ui| {
                                for t in ThreadSize::ALL {
                                    ui.selectable_value(&mut g.thread, t, t.name());
                                }
                            });
                        if g.thread != before && !g.cutter.fits(g.thread) {
                            g.cutter = Cutter::default_for(&settings.cutters, g.thread);
                        }

                        // hole start Z
                        ui.add(egui::DragValue::new(&mut g.start_z).speed(0.1).max_decimals(3).suffix(" mm"))
                            .on_hover_text("All hole geometry is below this Z");

                        // thread length
                        ui.add(
                            egui::DragValue::new(&mut g.length)
                                .range(0.0..=100.0)
                                .speed(0.1)
                                .max_decimals(3)
                                .suffix(" mm"),
                        );

                        // descent angle for the helical milling op
                        ui.add(egui::DragValue::new(&mut g.ramp_angle).range(0.1..=45.0).speed(0.1).max_decimals(2).suffix("°"))
                            .on_hover_text("Helical descent angle for hole milling");

                        // cutter - only cutters smaller than the hole are selectable
                        egui::ComboBox::from_id_salt(("cutter", i))
                            .selected_text(format!("T{} {}", g.cutter.tool, g.cutter.name()))
                            .width(90.0)
                            .show_ui(ui, |ui| {
                                for &c in &settings.cutters {
                                    let ok = c.fits(g.thread);
                                    ui.add_enabled_ui(ok, |ui| {
                                        ui.selectable_value(&mut g.cutter, c, format!("T{} {}", c.tool, c.name()))
                                            .on_disabled_hover_text("cutter must be smaller than the hole diameter");
                                    });
                                }
                            });

                        // locations
                        ui.add(
                            egui::TextEdit::multiline(&mut g.locations)
                                .desired_rows(3)
                                .desired_width(150.0)
                                .code_editor()
                                .hint_text("x, y"),
                        );

                        // info
                        let count = g.parse_locations().map(|v| v.len()).unwrap_or(0);
                        ui.label(format!(
                            "Ø{} × {} deep\nbottom Z{}\npitch {}\n{} hole(s)",
                            gcode::fmt(g.thread.hole_dia()),
                            gcode::fmt(g.hole_depth()),
                            gcode::fmt(g.start_z - g.hole_depth()),
                            gcode::fmt(g.thread.pitch()),
                            count
                        ));

                        // validation
                        let issues = gcode::validate_group(g, &settings);
                        ui.vertical(|ui| {
                            if issues.is_empty() {
                                ui.colored_label(egui::Color32::from_rgb(60, 170, 80), "✔ OK");
                            }
                            for is in &issues {
                                let col = if is.error {
                                    ui.visuals().error_fg_color
                                } else {
                                    ui.visuals().warn_fg_color
                                };
                                ui.colored_label(col, format!("{} {}", if is.error { "✖" } else { "⚠" }, is.msg));
                            }
                        });

                        ui.horizontal(|ui| {
                            if ui.add_enabled(i > 0, egui::Button::new("⬆")).on_hover_text("Move up").clicked() {
                                action = Some(RowAction::MoveUp(i));
                            }
                            if ui.add_enabled(i + 1 < n, egui::Button::new("⬇")).on_hover_text("Move down").clicked() {
                                action = Some(RowAction::MoveDown(i));
                            }
                            if ui.button("⧉").on_hover_text("Duplicate").clicked() {
                                action = Some(RowAction::Duplicate(i));
                            }
                            if ui.button("🗑").on_hover_text("Remove").clicked() {
                                action = Some(RowAction::Remove(i));
                            }
                        });
                        ui.end_row();
                    }
                });
        });

        let groups = &mut self.job.groups;
        match action {
            Some(RowAction::Remove(i)) => {
                groups.remove(i);
            }
            Some(RowAction::Duplicate(i)) => {
                let g = groups[i].clone();
                groups.insert(i + 1, g);
            }
            Some(RowAction::MoveUp(i)) => groups.swap(i, i - 1),
            Some(RowAction::MoveDown(i)) => groups.swap(i, i + 1),
            None => {}
        }
    }

    fn step_milling_tab(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("Step Milling");
            if ui.button("📂 Import STEP…").clicked() {
                self.import_step();
            }
            match &self.step_model {
                Some(m) => {
                    let name = std::path::Path::new(&self.job.step.step_path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                    ui.label(format!(
                        "{name} - {} faces, {} edges - X {:.2}…{:.2}  Y {:.2}…{:.2}  Z {:.2}…{:.2}",
                        m.faces.len(),
                        m.edges.len(),
                        m.min[0],
                        m.max[0],
                        m.min[1],
                        m.max[1],
                        m.min[2],
                        m.max[2]
                    ));
                }
                None => {
                    ui.weak("no model loaded");
                }
            }
        });
        self.stock_row(ui);
        ui.weak("Click to highlight, right-click a flat surface or an edge to add operations. Esc clears the highlight.");

        egui::SidePanel::left("step_ops").resizable(true).default_width(340.0).min_width(260.0).show_inside(ui, |ui| {
            self.ops_list(ui);
        });

        let tabs: Vec<MillTab> = self
            .job
            .step
            .ops
            .iter()
            .filter(|o| o.enabled)
            .filter_map(|o| if let OpKind::Tab(t) = &o.kind { Some(t.clone()) } else { None })
            .collect();
        let Some(model) = self.step_model.take() else {
            self.step_view.ui(ui, &self.step_toolpath, &[], None);
            return;
        };
        let stock = self.job.step.stock.bounds(model.min, model.max);
        let viz = ModelViz { model: &model, sel_edges: &self.sel.edges, sel_face: self.sel.face, tabs: &tabs, stock: Some(stock) };
        let out = self.step_view.ui(ui, &self.step_toolpath, &[], Some(&viz));
        for pick in [out.left_pick.clone(), out.right_pick.clone()].into_iter().flatten() {
            self.sel = match &pick {
                Some(Pick::Face(f)) => Selection { edges: vec![], face: Some(*f) },
                Some(Pick::Edge { edge, .. }) => Selection { edges: model.edge_chain(*edge).0, face: None },
                None => Selection::default(),
            };
        }
        if let Some(p) = out.right_pick {
            self.menu_pick = p;
        }
        let ops = &mut self.job.step.ops;
        let pick = self.menu_pick.clone();
        let set_z = &mut self.set_z_value;
        let placed = !self.job.step.placement.is_identity();
        let stock_top = stock.1[2];
        let mut action = None;
        let n_before = ops.len();
        out.response.context_menu(|ui| {
            action = context_menu(ui, &model, &pick, ops, set_z, placed, stock_top);
        });
        // new ops default to a 3.175 cutter: use the closest one in the tool table
        let table = &self.job.settings.cutters;
        for op in self.job.step.ops.iter_mut().skip(n_before) {
            if let Some(c) = op.cutter_mut() {
                if !table.iter().any(|t| t.id == c.id) {
                    if let Some(&best) = table.iter().min_by(|a, b| (a.dia - c.dia).abs().total_cmp(&(b.dia - c.dia).abs())) {
                        *c = best;
                    }
                }
            }
        }
        self.step_model = Some(model);
        if let Some(a) = action {
            self.menu_action(a);
        }
    }

    fn add_rough_op(&mut self) {
        let (Some(model), Some((_, smax))) = (&self.step_model, self.stock_box()) else { return };
        let mut op = RoughOp {
            start_z: smax[2],
            bottom_z: model.min[2],
            faces: stepmill::rough_faces(model),
            ..Default::default()
        };
        let table = &self.job.settings.cutters;
        if !table.iter().any(|t| t.id == op.cutter.id) {
            if let Some(&best) = table.iter().min_by(|a, b| (a.dia - op.cutter.dia).abs().total_cmp(&(b.dia - op.cutter.dia).abs())) {
                op.cutter = best;
            }
        }
        // roughing goes before the other cutting ops
        let at = self.job.step.ops.iter().position(|o| !matches!(o.kind, OpKind::Tab(_) | OpKind::Rough(_))).unwrap_or(self.job.step.ops.len());
        self.job.step.ops.insert(at, StepOp { enabled: true, kind: OpKind::Rough(op) });
    }

    fn stock_row(&mut self, ui: &mut egui::Ui) {
        let model_box = self.step_model.as_ref().map(|m| (m.min, m.max));
        let rapid_z = self.step_settings().safe_z;
        ui.horizontal(|ui| {
            ui.label("Rapid plane height above stock or part");
            ui.add(egui::DragValue::new(&mut self.job.step.rapid_above).speed(0.1).max_decimals(3).suffix(" mm"))
                .on_hover_text("Measured from the highest point of the stock or the part, whichever is higher");
            ui.weak(format!("= rapid plane Z{}", gcode::fmt(rapid_z)));
        });
        let st = &mut self.job.step.stock;
        ui.horizontal(|ui| {
            ui.label("Stock");
            let hover = if st.offset { "Added to the part's bounding box (X/Y split over both sides, Z on top)" } else { "Absolute stock size" };
            for (a, v) in [("X", &mut st.x), ("Y", &mut st.y), ("Z", &mut st.z)] {
                ui.label(a);
                ui.add(egui::DragValue::new(v).speed(0.1).max_decimals(3).suffix(" mm")).on_hover_text(hover);
            }
            let was = *st;
            let toggled = ui
                .checkbox(&mut st.offset, "Offset")
                .on_hover_text("On: the values are added to the part's bounding box.\nOff: the values are the absolute size of the stock.")
                .changed();
            // switching mode keeps the same stock: convert the values
            if let (true, Some((min, max))) = (toggled, model_box) {
                let part = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
                let sign = if st.offset { -1.0 } else { 1.0 };
                st.x = was.x + sign * part[0];
                st.y = was.y + sign * part[1];
                st.z = was.z + sign * part[2];
            }
            if let Some((min, max)) = model_box {
                let (a, b) = st.bounds(min, max);
                ui.weak(format!(
                    "= {} x {} x {}  (top Z{})",
                    gcode::fmt(b[0] - a[0]),
                    gcode::fmt(b[1] - a[1]),
                    gcode::fmt(b[2] - a[2]),
                    gcode::fmt(b[2])
                ));
                for e in st.issues(min, max) {
                    ui.colored_label(ui.visuals().warn_fg_color, format!("⚠ {e}"));
                }
            }
        });
    }

    fn ops_list(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("Operations");
            let have_model = self.step_model.is_some();
            if ui
                .add_enabled(have_model, egui::Button::new("➕ Rough"))
                .on_hover_text("Rough the part layer by layer: clears the material inside the part (pockets, steps) and profiles all exterior contours")
                .on_disabled_hover_text("Import a STEP model first")
                .clicked()
            {
                self.add_rough_op();
            }
        });
        let s = self.step_settings();
        let ops = &mut self.job.step.ops;
        if ops.is_empty() {
            ui.weak("Right-click a flat surface (Face / Pocket) or an edge (Pocket / Profile / Add mill tab) in the 3D view.");
            return;
        }
        let tabs_top = ops
            .iter()
            .filter(|o| o.enabled)
            .filter_map(|o| if let OpKind::Tab(t) = &o.kind { Some(t.top()) } else { None })
            .fold(f64::NEG_INFINITY, f64::max);
        let n = ops.len();
        let mut action = None;
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (i, op) in ops.iter_mut().enumerate() {
                ui.push_id(i, |ui| {
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut op.enabled, "").on_hover_text("Include in the program");
                            if ui.selectable_label(false, egui::RichText::new(format!("{}. {}", i + 1, op.label())).strong()).on_hover_text("Highlight in the 3D view").clicked() {
                                action = Some(OpAction::Select(i));
                            }
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.small_button("🗑").on_hover_text("Remove").clicked() {
                                    action = Some(OpAction::Remove(i));
                                }
                                if ui.add_enabled(i + 1 < n, egui::Button::new("⬇").small()).clicked() {
                                    action = Some(OpAction::MoveDown(i));
                                }
                                if ui.add_enabled(i > 0, egui::Button::new("⬆").small()).clicked() {
                                    action = Some(OpAction::MoveUp(i));
                                }
                            });
                        });
                        op_params(ui, i, &mut op.kind, &s.cutters);
                        if op.enabled {
                            for e in stepmill::validate(op, &s, tabs_top) {
                                ui.colored_label(ui.visuals().error_fg_color, format!("✖ {e}"));
                            }
                        }
                    });
                });
                ui.add_space(4.0);
            }
        });
        match action {
            Some(OpAction::Select(i)) => {
                self.sel = Selection { edges: ops[i].edges().to_vec(), face: ops[i].face_id() };
            }
            Some(OpAction::Remove(i)) => {
                ops.remove(i);
            }
            Some(OpAction::MoveUp(i)) => ops.swap(i, i - 1),
            Some(OpAction::MoveDown(i)) => ops.swap(i, i + 1),
            None => {}
        }
    }

    fn save_settings(&mut self) {
        let Some(path) = rfd::FileDialog::new().set_title("Save settings").add_filter("JSON", &["json"]).set_file_name("stepcam-settings.json").save_file() else {
            return;
        };
        let res = serde_json::to_string_pretty(&self.job.settings).map_err(|e| e.to_string()).and_then(|j| std::fs::write(&path, j).map_err(|e| e.to_string()));
        match res {
            Ok(()) => self.set_status(format!("Saved settings to {}", path.display()), false),
            Err(e) => self.set_status(format!("Could not save settings: {e}"), true),
        }
    }

    fn load_settings(&mut self) {
        let Some(path) = rfd::FileDialog::new().set_title("Load settings").add_filter("JSON", &["json"]).add_filter("All files", &["*"]).pick_file() else {
            return;
        };
        let res = std::fs::read_to_string(&path).map_err(|e| e.to_string()).and_then(|j| serde_json::from_str::<Settings>(&j).map_err(|e| e.to_string()));
        match res {
            Ok(s) => {
                self.job.settings = s;
                self.job.normalize();
                self.set_status(format!("Loaded settings from {}", path.display()), false);
            }
            Err(e) => self.set_status(format!("Could not load settings: {e}"), true),
        }
    }

    fn settings_tab(&mut self, ui: &mut egui::Ui) {
        let mut action: Option<bool> = None;
        let s = &mut self.job.settings;
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            ui.heading("Machining settings");
            egui::Grid::new("settings").num_columns(2).spacing([16.0, 6.0]).show(ui, |ui| {
                fn row(ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64, suffix: &str) {
                    ui.label(label);
                    ui.add(egui::DragValue::new(v).speed(speed).max_decimals(3).suffix(suffix));
                    ui.end_row();
                }
                row(ui, "Clearance above hole start Z (between holes)", &mut s.clearance, 0.1, " mm");
                row(ui, "Plunge feed", &mut s.plunge_feed, 1.0, " mm/min");
                row(ui, "Boring ring stepover (% of cutter Ø)", &mut s.bore_stepover_pct, 0.5, " %");
                ui.label("Coolant (M8)");
                ui.checkbox(&mut s.coolant, "");
                ui.end_row();
                ui.label("Tool length comp (G43 Hn)");
                ui.checkbox(&mut s.tool_length_comp, "");
                ui.end_row();
            });
            for is in gcode::validate_settings(s) {
                ui.colored_label(ui.visuals().error_fg_color, format!("✖ {}", is.msg));
            }

            ui.add_space(12.0);
            ui.heading("Milling cutters (end mills)");
            ui.weak("Used for hole milling and step milling. Groups and operations follow edits by tool number.");
            let mut remove = None;
            egui::Grid::new("cutters").striped(true).num_columns(5).spacing([16.0, 4.0]).show(ui, |ui| {
                for h in ["Tool", "Ø", "Flute length", "", ""] {
                    ui.strong(h);
                }
                ui.end_row();
                for (k, c) in s.cutters.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label("T");
                        ui.add(egui::DragValue::new(&mut c.tool).range(1..=999).speed(0.1));
                    });
                    ui.add(egui::DragValue::new(&mut c.dia).range(0.01..=100.0).speed(0.01).max_decimals(3).suffix(" mm"));
                    ui.add(egui::DragValue::new(&mut c.flute).range(0.01..=500.0).speed(0.1).max_decimals(3).suffix(" mm"));
                    ui.weak(c.name());
                    if ui.small_button("🗑").on_hover_text("Remove from the tool table").clicked() {
                        remove = Some(k);
                    }
                    ui.end_row();
                }
            });
            if let Some(k) = remove {
                s.cutters.remove(k);
            }
            if ui.button("➕ Add cutter").clicked() {
                let used: Vec<u32> = s.cutters.iter().map(|c| c.tool).chain(s.thread_mills.iter().map(|t| t.tool)).collect();
                let tool = (1..).find(|n| !used.contains(n)).unwrap();
                let last = s.cutters.last().copied().unwrap_or(Cutter::C3x17);
                let id = s.new_cutter_id();
                s.cutters.push(Cutter { id, tool, ..last });
            }

            ui.add_space(12.0);
            ui.heading("Thread mills (single profile)");
            egui::Grid::new("thread_mills").striped(true).num_columns(5).spacing([16.0, 4.0]).show(ui, |ui| {
                for h in ["Tool", "Thread", "Hole Ø", "Cutter Ø", "Reach"] {
                    ui.strong(h);
                }
                ui.end_row();
                for t in ThreadSize::ALL {
                    let tm = &mut s.thread_mills[t.index()];
                    ui.horizontal(|ui| {
                        ui.label("T");
                        ui.add(egui::DragValue::new(&mut tm.tool).range(1..=999).speed(0.1));
                    });
                    ui.label(format!("{} x {}", t.name(), gcode::fmt(t.pitch())));
                    ui.label(format!("{} mm", gcode::fmt(t.hole_dia())));
                    let r = ui.add(egui::DragValue::new(&mut tm.dia).range(0.1..=t.hole_dia()).speed(0.01).max_decimals(3).suffix(" mm"));
                    if tm.dia >= t.hole_dia() {
                        r.on_hover_text("must be smaller than the hole");
                    }
                    ui.add(egui::DragValue::new(&mut tm.reach).range(0.0..=100.0).speed(0.1).max_decimals(3).suffix(" mm"));
                    ui.end_row();
                }
            });
            for e in s.tool_issues() {
                ui.colored_label(ui.visuals().error_fg_color, format!("✖ {e}"));
            }

            ui.add_space(12.0);
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("💾 Save settings…").on_hover_text("Save the machining settings and tool tables as JSON").clicked() {
                    action = Some(true);
                }
                if ui.button("📂 Load settings…").on_hover_text("Load machining settings and tool tables from JSON").clicked() {
                    action = Some(false);
                }
                if ui.button("Reset all settings to defaults").clicked() {
                    *s = model::Settings::default();
                }
            });
        });
        match action {
            Some(true) => self.save_settings(),
            Some(false) => self.load_settings(),
            None => {}
        }
    }
}

impl eframe::App for StepCamApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.job.sync_cutters();
        self.regenerate_if_changed();
        self.regenerate_step_if_changed();
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.sel = Selection::default();
            self.view.highlight_line = None;
            self.step_view.highlight_line = None;
        }
        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.heading("stepCAM");
                ui.separator();
                ui.selectable_value(&mut self.tab, Tab::ThreadMilling, "Thread Milling");
                ui.selectable_value(&mut self.tab, Tab::StepMilling, "Step Milling");
                ui.selectable_value(&mut self.tab, Tab::Settings, "Tools & Settings");
            });
            ui.add_space(2.0);
            self.file_bar(ui);
            ui.add_space(4.0);
        });

        egui::SidePanel::right("gcode")
            .resizable(true)
            .default_width(380.0)
            .show(ctx, |ui| {
                let step = self.tab == Tab::StepMilling;
                let (code, view) = if step { (&self.step_gcode, &mut self.step_view) } else { (&self.gcode, &mut self.view) };
                ui.horizontal(|ui| {
                    ui.heading("G-code");
                    if ui.add_enabled(!code.is_empty(), egui::Button::new("📋 Copy")).clicked() {
                        ui.output_mut(|o| o.copied_text = code.clone());
                    }
                    match view.highlight_line {
                        Some(l) => ui.weak(format!("line {} highlighted", l + 1)),
                        None => ui.weak("click a line to highlight its move"),
                    };
                });
                egui::ScrollArea::both().auto_shrink([false, false]).show(ui, |ui| {
                    let out = egui::TextEdit::multiline(&mut code.as_str())
                        .id_source(if step { "step_gcode_text" } else { "gcode_text" })
                        .code_editor()
                        .desired_width(f32::INFINITY)
                        .show(ui);
                    // follow the text cursor (mouse click or arrow keys)
                    if out.response.has_focus() {
                        if let Some(c) = out.cursor_range {
                            view.highlight_line = Some(c.primary.pcursor.paragraph);
                        }
                    }
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::ThreadMilling => self.thread_milling_tab(ui),
            Tab::StepMilling => self.step_milling_tab(ui),
            Tab::Settings => self.settings_tab(ui),
        });
    }
}

enum MenuAction {
    SetZ(usize, f64),
    LayFlat(usize),
    ResetPlacement,
}

/// XY outline and top Z of an edge chain; None if the chain is not level.
fn chain_outline(model: &step::Model, edges: &[usize]) -> Option<(Vec<[f64; 2]>, f64)> {
    let pl = model.chain_polyline(edges);
    if pl.len() < 2 {
        return None;
    }
    let (lo, hi) = pl.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])));
    if hi - lo > 1e-3 {
        return None;
    }
    Some((pl.iter().map(|p| [p[0], p[1]]).collect(), hi))
}

/// Default (bottom Z, start Z) for an op on an edge chain at height `z`:
/// a rim edge cuts down to the bottom of its walls, a floor edge cuts from
/// the top of its walls, a free wire gets 1 mm deep.
fn edge_depths(model: &step::Model, edges: &[usize], z: f64) -> (f64, f64) {
    let walls: Vec<&step::Face> = edges
        .iter()
        .filter_map(|&e| model.edges.get(e))
        .flat_map(|e| e.faces.iter())
        .filter_map(|&f| model.faces.get(f))
        .filter(|f| !f.is_horizontal())
        .collect();
    let (lo, hi) = walls
        .iter()
        .flat_map(|f| f.loops.iter().flat_map(|l| l.pts.iter()))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])));
    if walls.is_empty() || !lo.is_finite() {
        (z - 1.0, z)
    } else if lo < z - 1e-6 {
        (lo, z)
    } else if hi > z + 1e-6 {
        (z, hi)
    } else {
        (z - 1.0, z)
    }
}

/// Re-derive operation geometry after the model moved from `old` to `new`.
/// Returns warnings for operations that no longer make sense (they are disabled).
fn refresh_ops(ops: &mut [StepOp], model: &step::Model, old: &step::Placement, new: &step::Placement) -> Vec<String> {
    let mut warnings = Vec::new();
    let region = [model.min[0], model.min[1], model.max[0], model.max[1]];
    let xy = |l: &step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
    for (i, op) in ops.iter_mut().enumerate() {
        let face = op.face_id().and_then(|fi| model.faces.get(fi)).filter(|f| f.is_up_horizontal());
        let mut ok = true;
        match &mut op.kind {
            OpKind::Face(f) => match face {
                Some(fc) => {
                    f.start_z += fc.z() - f.z;
                    f.z = fc.z();
                    f.region = region;
                }
                None => ok = false,
            },
            OpKind::Pocket(p) if p.edges.is_empty() => match face {
                Some(fc) => {
                    p.start_z += fc.z() - p.z;
                    p.z = fc.z();
                    p.outer = fc.outer_loop().map(xy).unwrap_or_default();
                    p.islands = fc.loops.iter().filter(|l| !l.outer).map(xy).collect();
                }
                None => ok = false,
            },
            OpKind::Pocket(p) => match chain_outline(model, &p.edges) {
                Some((outline, z)) => {
                    let dz = z - p.edge_z;
                    (p.z, p.start_z, p.edge_z) = (p.z + dz, p.start_z + dz, z);
                    p.outer = outline;
                }
                None => ok = false,
            },
            OpKind::Profile(p) => match chain_outline(model, &p.edges) {
                Some((outline, z)) => {
                    let dz = z - p.edge_z;
                    (p.z, p.start_z, p.edge_z) = (p.z + dz, p.start_z + dz, z);
                    p.path = outline;
                }
                None => ok = false,
            },
            OpKind::Rough(_) => {} // re-synced from the model every frame
            OpKind::Tab(t) => {
                t.center = new.point(old.inv_point(t.center));
                let d = new.dir(old.inv_dir([t.angle.cos(), t.angle.sin(), 0.0]));
                if d[0].hypot(d[1]) > 1e-6 {
                    t.angle = d[1].atan2(d[0]);
                } else {
                    ok = false;
                }
            }
        }
        if !ok {
            op.enabled = false;
            warnings.push(format!(
                "op {} disabled: {}",
                i + 1,
                if op.edges().is_empty() { "its surface no longer faces +Z" } else { "its edges are no longer level" }
            ));
        }
    }
    warnings
}

/// Right-click menu in the step milling view.
fn context_menu(ui: &mut egui::Ui, model: &step::Model, pick: &Option<Pick>, ops: &mut Vec<StepOp>, set_z: &mut f64, placed: bool, stock_top: f64) -> Option<MenuAction> {
    // ops cutting from the part top start at the stock top instead
    let from_top = |start: f64| if start >= model.max[2] - 1e-6 { start.max(stock_top) } else { start };
    let mut action = None;
    // placement entries for any flat surface
    let placement_items = |ui: &mut egui::Ui, fi: usize, action: &mut Option<MenuAction>, set_z: &mut f64| {
        let f = &model.faces[fi];
        ui.separator();
        ui.add_enabled_ui(f.is_horizontal(), |ui| {
            ui.menu_button("Set Z…", |ui| {
                ui.label("Move the model so this surface is at:");
                ui.horizontal(|ui| {
                    ui.label("Z");
                    let r = ui.add(egui::DragValue::new(set_z).speed(0.1).suffix(" mm").max_decimals(4));
                    let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if ui.button("Apply").clicked() || enter {
                        *action = Some(MenuAction::SetZ(fi, *set_z));
                        ui.close_menu();
                    }
                });
            })
        })
        .response
        .on_disabled_hover_text("Only for surfaces normal to Z - lay the model flat first");
        if ui.button("Lay flat on this surface").on_hover_text("Rotate the model so this surface faces down (rests on the table); top ends at Z0").clicked() {
            *action = Some(MenuAction::LayFlat(fi));
            ui.close_menu();
        }
    };
    let reset = |ui: &mut egui::Ui, action: &mut Option<MenuAction>| {
        if placed && ui.button("Reset placement").on_hover_text("Back to the coordinates in the STEP file").clicked() {
            *action = Some(MenuAction::ResetPlacement);
            ui.close_menu();
        }
    };
    match pick {
        Some(Pick::Face(fi)) => {
            let f = &model.faces[*fi];
            if f.is_up_horizontal() {
                ui.label(format!("Flat surface at Z{}", gcode::fmt(f.z())));
                ui.separator();
                if ui.button("Face").on_hover_text("Mill the whole part down to this surface").clicked() {
                    let z = f.z();
                    ops.push(StepOp {
                        enabled: true,
                        kind: OpKind::Face(FaceOp {
                            face_id: *fi,
                            z,
                            region: [model.min[0], model.min[1], model.max[0], model.max[1]],
                            start_z: if stock_top > z + 1e-6 { stock_top } else { z + 1.0 },
                            ..Default::default()
                        }),
                    });
                    ui.close_menu();
                }
                if ui.button("Pocket").on_hover_text("Clear the material inside this surface's outer boundary down to it").clicked() {
                    let xy = |l: &step::Loop| l.pts.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
                    let outer = f.outer_loop().map(xy).unwrap_or_default();
                    let islands = f.loops.iter().filter(|l| !l.outer).map(xy).collect();
                    ops.push(StepOp { enabled: true, kind: OpKind::Pocket(PocketOp { face_id: *fi, z: f.z(), outer, islands, start_z: stock_top, ..Default::default() }) });
                    ui.close_menu();
                }
            } else if f.planar {
                ui.weak("Flat surface not facing +Z - no milling operations");
            } else {
                ui.weak("Curved surface - no operations");
            }
            if f.planar {
                placement_items(ui, *fi, &mut action, set_z);
            }
            reset(ui, &mut action);
        }
        Some(Pick::Edge { edge, point, angle }) => {
            let (chain, closed) = model.edge_chain(*edge);
            ui.label(if closed { format!("Closed loop of {} edge(s)", chain.len()) } else { format!("Open chain of {} edge(s)", chain.len()) });
            ui.separator();
            let outline = chain_outline(model, &chain);
            let level = outline.is_some();
            let pocket_ok = level && closed;
            if ui
                .add_enabled(pocket_ok, egui::Button::new("Pocket"))
                .on_hover_text("Clear the material inside this loop of edges")
                .on_disabled_hover_text(if !closed { "Needs a closed loop of edges" } else { "Edges must be level (constant Z)" })
                .clicked()
            {
                let (outer, z) = outline.clone().unwrap();
                let (bottom, start) = edge_depths(model, &chain, z);
                let start = from_top(start);
                ops.push(StepOp {
                    enabled: true,
                    kind: OpKind::Pocket(PocketOp { face_id: usize::MAX, z: bottom, start_z: start, outer, edges: chain.clone(), edge_z: z, ..Default::default() }),
                });
                ui.close_menu();
            }
            if ui
                .add_enabled(level, egui::Button::new("Profile"))
                .on_hover_text("Cut along these edges: on the centre line, inside or outside the loop")
                .on_disabled_hover_text("Edges must be level (constant Z)")
                .clicked()
            {
                let (path, z) = outline.unwrap();
                let (bottom, start) = edge_depths(model, &chain, z);
                let start = from_top(start);
                ops.push(StepOp {
                    enabled: true,
                    kind: OpKind::Profile(ProfileOp {
                        path,
                        closed,
                        edge_z: z,
                        side: if closed { ProfileSide::Outside } else { ProfileSide::Center },
                        z: bottom,
                        start_z: start,
                        edges: chain.clone(),
                        ..Default::default()
                    }),
                });
                ui.close_menu();
            }
            if ui.button("Add mill tab").on_hover_text("Rectangular tab extruded up from the edge at the clicked point; the cutter lifts over it").clicked() {
                ops.push(StepOp { enabled: true, kind: OpKind::Tab(MillTab { center: *point, angle: *angle, edges: chain, ..Default::default() }) });
                ui.close_menu();
            }
        }
        None => {
            ui.weak("Right-click a flat surface or an edge");
            reset(ui, &mut action);
        }
    }
    action
}

fn num(ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64, suffix: &str, hover: &str) {
    ui.label(label);
    let r = ui.add(egui::DragValue::new(v).speed(speed).max_decimals(3).suffix(suffix));
    if !hover.is_empty() {
        r.on_hover_text(hover);
    }
    ui.end_row();
}

fn cutter_combo(ui: &mut egui::Ui, i: usize, c: &mut Cutter, cutters: &[Cutter]) {
    ui.label("Tool");
    egui::ComboBox::from_id_salt(("step_cutter", i)).selected_text(format!("T{} {}", c.tool, c.name())).show_ui(ui, |ui| {
        for &k in cutters {
            ui.selectable_value(c, k, format!("T{} {}", k.tool, k.name()));
        }
    });
    ui.end_row();
}

/// Parameter editor for one operation.
fn op_params(ui: &mut egui::Ui, i: usize, kind: &mut OpKind, cutters: &[Cutter]) {
    egui::Grid::new(("op_params", i)).num_columns(2).spacing([10.0, 3.0]).show(ui, |ui| match kind {
        OpKind::Face(f) => {
            cutter_combo(ui, i, &mut f.cutter, cutters);
            num(ui, "Stepover", &mut f.stepover_pct, 0.5, " %", "% of cutter diameter");
            num(ui, "Depth per pass", &mut f.depth_per_pass, 0.05, " mm", "");
            num(ui, "Overlap outside part", &mut f.overlap, 0.1, " mm", "How far the cutter edge runs past the part");
            num(ui, "Feed", &mut f.feed, 1.0, " mm/min", "");
            num(ui, "Speed", &mut f.speed, 10.0, " rpm", "");
            num(ui, "Start Z", &mut f.start_z, 0.1, " mm", "Top of the stock");
            ui.label("Surface Z");
            ui.label(gcode::fmt(f.z));
            ui.end_row();
        }
        OpKind::Pocket(p) => {
            cutter_combo(ui, i, &mut p.cutter, cutters);
            ui.label("Strategy");
            egui::ComboBox::from_id_salt(("strategy", i))
                .selected_text(match p.strategy {
                    Strategy::Standard => "Standard",
                    Strategy::Adaptive => "Adaptive clearing",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut p.strategy, Strategy::Standard, "Standard").on_hover_text("Concentric offset passes");
                    ui.selectable_value(&mut p.strategy, Strategy::Adaptive, "Adaptive clearing").on_hover_text("Trochoidal loops along offset paths");
                });
            ui.end_row();
            ui.label("Entry");
            egui::ComboBox::from_id_salt(("entry", i))
                .selected_text(match p.entry {
                    Entry::Helix => "Helix",
                    Entry::Zigzag => "Zigzag",
                    Entry::Plunge => "Plunge",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut p.entry, Entry::Helix, "Helix");
                    ui.selectable_value(&mut p.entry, Entry::Zigzag, "Zigzag");
                    ui.selectable_value(&mut p.entry, Entry::Plunge, "Plunge");
                });
            ui.end_row();
            let so_hover = if p.strategy == Strategy::Adaptive { "Adaptive: advance per trochoid loop, % of cutter diameter" } else { "% of cutter diameter between passes" };
            num(ui, "Stepover", &mut p.stepover_pct, 0.5, " %", so_hover);
            num(ui, "Feed", &mut p.feed, 1.0, " mm/min", "");
            num(ui, "Speed", &mut p.speed, 10.0, " rpm", "");
            num(ui, "Ramp angle", &mut p.ramp_angle, 0.1, "°", "Helix / zigzag entry angle");
            num(ui, "Start Z", &mut p.start_z, 0.1, " mm", "Top of the material");
            num(ui, "Depth per pass", &mut p.depth_per_pass, 0.05, " mm", "");
            num(ui, "Finish pass", &mut p.finish, 0.01, " mm", "Material left on the walls by roughing, removed by the climb-milled finish pass");
            if !p.islands.is_empty() {
                ui.label("Islands");
                ui.checkbox(&mut p.keep_islands, format!("keep {} inner loop(s)", p.islands.len()))
                    .on_hover_text("Off (default): inner loops of the surface are ignored and the whole outer boundary is cleared");
                ui.end_row();
            }
            if p.edges.is_empty() {
                ui.label("Pocket floor Z");
                ui.label(gcode::fmt(p.z));
                ui.end_row();
            } else {
                num(ui, "Pocket floor Z", &mut p.z, 0.1, " mm", "Final depth (pocket made from edges)");
            }
        }
        OpKind::Profile(p) => {
            cutter_combo(ui, i, &mut p.cutter, cutters);
            ui.label("Tool position");
            egui::ComboBox::from_id_salt(("side", i)).selected_text(p.side.name()).show_ui(ui, |ui| {
                ui.selectable_value(&mut p.side, ProfileSide::Center, "on centre line").on_hover_text("Tool centre follows the edges");
                for (side, hover) in [(ProfileSide::Inside, "Tool inside the loop, edge of cut on the edges (CCW, climb)"), (ProfileSide::Outside, "Tool outside the loop, edge of cut on the edges (CW, climb)")] {
                    ui.add_enabled_ui(p.closed, |ui| {
                        ui.selectable_value(&mut p.side, side, side.name()).on_hover_text(hover).on_disabled_hover_text("Only for closed loops of edges");
                    });
                }
            });
            ui.end_row();
            num(ui, "Feed", &mut p.feed, 1.0, " mm/min", "");
            num(ui, "Speed", &mut p.speed, 10.0, " rpm", "");
            num(ui, "Ramp angle", &mut p.ramp_angle, 0.1, "°", "Ramp down along the path at this angle");
            num(ui, "Start Z", &mut p.start_z, 0.1, " mm", "Top of the material");
            num(ui, "Depth per pass", &mut p.depth_per_pass, 0.05, " mm", "");
            num(ui, "Bottom Z", &mut p.z, 0.1, " mm", "Final depth");
            ui.label("Edges");
            ui.label(format!("{} {} at Z{}", p.edges.len(), if p.closed { "(closed loop)" } else { "(open chain)" }, gcode::fmt(p.edge_z)));
            ui.end_row();
        }
        OpKind::Rough(r) => {
            cutter_combo(ui, i, &mut r.cutter, cutters);
            ui.label("Strategy");
            egui::ComboBox::from_id_salt(("rough_strategy", i))
                .selected_text(match r.strategy {
                    Strategy::Standard => "Standard",
                    Strategy::Adaptive => "Adaptive clearing",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut r.strategy, Strategy::Standard, "Standard");
                    ui.selectable_value(&mut r.strategy, Strategy::Adaptive, "Adaptive clearing");
                });
            ui.end_row();
            ui.label("Entry");
            egui::ComboBox::from_id_salt(("rough_entry", i))
                .selected_text(match r.entry {
                    Entry::Helix => "Helix",
                    Entry::Zigzag => "Zigzag",
                    Entry::Plunge => "Plunge",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut r.entry, Entry::Helix, "Helix");
                    ui.selectable_value(&mut r.entry, Entry::Zigzag, "Zigzag");
                    ui.selectable_value(&mut r.entry, Entry::Plunge, "Plunge");
                });
            ui.end_row();
            ui.label("Clear voids");
            ui.checkbox(&mut r.clear_voids, "")
                .on_hover_text("Pockets and steps (anything with a face below) are always cleared.\nTicked: holes that go right through the part are cleared too.\nUnticked: through holes are only profiled.");
            ui.end_row();
            num(ui, "Layer height", &mut r.layer_height, 0.05, " mm", "Depth of each roughing layer (extra layers are added at every flat floor)");
            num(ui, "Stepover", &mut r.stepover_pct, 0.5, " %", "% of cutter diameter");
            num(ui, "Stock to leave", &mut r.allowance, 0.01, " mm", "Left on all walls and floors for finishing ops");
            num(ui, "Feed", &mut r.feed, 1.0, " mm/min", "");
            num(ui, "Speed", &mut r.speed, 10.0, " rpm", "");
            num(ui, "Ramp angle", &mut r.ramp_angle, 0.1, "°", "Helix / zigzag entry angle");
            num(ui, "Start Z", &mut r.start_z, 0.1, " mm", "Top of the stock");
            num(ui, "Bottom Z", &mut r.bottom_z, 0.1, " mm", "Where roughing stops (default: part bottom)");
            num(ui, "Overcut Z", &mut r.overcut_z, 0.05, " mm", "Keep going this far below Bottom Z (e.g. to cut through the part)");
            ui.label("Layers");
            ui.label(format!("{}", stepmill::rough_levels(r).len()));
            ui.end_row();
        }
        OpKind::Tab(t) => {
            num(ui, "Z height", &mut t.z_height, 0.05, " mm", "Extruded up from the edge");
            num(ui, "Width", &mut t.width, 0.1, " mm", "Along the edge");
            num(ui, "Depth", &mut t.depth, 0.1, " mm", "Across the edge");
            ui.label("Edge point");
            ui.label(format!("X{} Y{} Z{}", gcode::fmt(t.center[0]), gcode::fmt(t.center[1]), gcode::fmt(t.center[2])));
            ui.end_row();
            let mut deg = t.angle.to_degrees();
            num(ui, "Direction", &mut deg, 0.5, "°", "Tangent of the edge at the clicked point");
            t.angle = deg.to_radians();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn block() -> step::Model {
        step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap()
    }

    #[test]
    fn set_z_moves_ops_with_model() {
        let raw = block();
        let floor = raw.faces.iter().position(|f| f.is_up_horizontal() && (f.z() + 4.0).abs() < 1e-9).unwrap();
        let mut ops = vec![
            StepOp { enabled: true, kind: OpKind::Pocket(PocketOp { face_id: floor, z: -4.0, start_z: 0.0, ..Default::default() }) },
            StepOp { enabled: true, kind: OpKind::Tab(MillTab { center: [0.0, 20.0, -10.0], angle: FRAC_PI_2, ..Default::default() }) },
        ];
        let old = step::Placement::default();
        let new = old.translated([0.0, 0.0, 10.0]); // bottom of the block to Z0
        let model = raw.transformed(&new);
        assert!(refresh_ops(&mut ops, &model, &old, &new).is_empty());
        let OpKind::Pocket(p) = &ops[0].kind else { panic!() };
        assert!((p.z - 6.0).abs() < 1e-9 && (p.start_z - 10.0).abs() < 1e-9 && p.outer.len() >= 4);
        let OpKind::Tab(t) = &ops[1].kind else { panic!() };
        assert!(t.center[2].abs() < 1e-9 && (t.angle - FRAC_PI_2).abs() < 1e-9);
        assert!((model.min[2]).abs() < 1e-9 && (model.max[2] - 10.0).abs() < 1e-9);
    }

    #[test]
    fn lay_flat_on_side_disables_top_ops() {
        let raw = block();
        let side = raw.faces.iter().position(|f| f.planar && (f.normal[1] + 1.0).abs() < 1e-9).unwrap();
        let top = raw.faces.iter().position(|f| f.is_up_horizontal() && f.z().abs() < 1e-9).unwrap();
        let old = step::Placement::default();
        let new = old.rotated(&step::rotation_to_down(raw.faces[side].normal));
        let model = raw.transformed(&new);
        assert!((model.faces[side].normal[2] + 1.0).abs() < 1e-9);
        // block is now 40 tall, 10 deep
        assert!(((model.max[2] - model.min[2]) - 40.0).abs() < 1e-9);
        let mut ops = vec![StepOp { enabled: true, kind: OpKind::Face(FaceOp { face_id: top, ..Default::default() }) }];
        assert_eq!(refresh_ops(&mut ops, &model, &old, &new).len(), 1);
        assert!(!ops[0].enabled);
    }

    #[test]
    fn edge_ops_get_depth_from_walls() {
        let m = block();
        // pocket rim edge at Z0 (x=15 side) -> pocket floor -4
        let rim = m.edges.iter().position(|e| !e.faces.is_empty() && e.pts.iter().all(|p| (p[0] - 15.0).abs() < 1e-9 && p[2].abs() < 1e-9)).unwrap();
        let (chain, closed) = m.edge_chain(rim);
        assert!(closed);
        let (outline, z) = chain_outline(&m, &chain).unwrap();
        assert_eq!(edge_depths(&m, &chain, z), (-4.0, 0.0));
        assert!(outline.len() >= 4);
        // part outline at Z0 -> bottom of the block
        let out = m.edges.iter().position(|e| !e.faces.is_empty() && e.pts.iter().all(|p| p[1].abs() < 1e-9 && p[2].abs() < 1e-9)).unwrap();
        let (chain, _) = m.edge_chain(out);
        assert_eq!(edge_depths(&m, &chain, 0.0), (-10.0, 0.0));
        // free wire -> 1 mm deep
        let free = m.edges.iter().position(|e| e.faces.is_empty()).unwrap();
        let (chain, closed) = m.edge_chain(free);
        assert!(!closed);
        assert_eq!(edge_depths(&m, &chain, 0.0), (-1.0, 0.0));
    }
}

#[cfg(test)]
mod rapid_tests {
    use super::*;

    #[test]
    fn step_rapid_plane_above_stock_or_part() {
        let mut app = StepCamApp::default();
        app.step_raw = Some(step::parse_model(include_str!("../testdata/pocket_block.step")).unwrap());
        app.step_model = app.step_raw.clone();
        assert_eq!(app.job.step.rapid_above, 10.0);
        // part top Z0, stock = part
        assert!((app.step_settings().safe_z - 10.0).abs() < 1e-9);
        app.job.step.stock = stepmill::Stock { x: 0.0, y: 0.0, z: 3.0, offset: true };
        assert!((app.step_settings().safe_z - 13.0).abs() < 1e-9);
        // absolute stock thinner than the part: the part top counts
        app.job.step.stock = stepmill::Stock { x: 60.0, y: 40.0, z: 5.0, offset: false };
        app.job.step.rapid_above = 4.0;
        assert!((app.step_settings().safe_z - 4.0).abs() < 1e-9);
        // thread milling rapid plane is untouched
        assert!((app.job.settings.safe_z - 10.0).abs() < 1e-9);
    }
}
