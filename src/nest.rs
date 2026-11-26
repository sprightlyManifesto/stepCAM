//! Nest tab: copies of the step milling job laid out on a larger sheet of
//! stock. Each copy sits in a box the size of the programmed stock, turned
//! by its own angle; the program cuts every op on every copy in turn (one
//! tool change per op).

use crate::model::Settings;
use crate::stepmill::{self, Origin, OriginXY, OriginZ, Placed, StepJob, P2, P3};
use eframe::egui::{self, Color32, Pos2, Sense, Stroke, Vec2};
use serde::{Deserialize, Serialize};

/// One copy of the part on the sheet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct NestCopy {
    /// Lower-left corner of the copy's stock box (of its footprint, when it
    /// is turned) from the sheet's lower-left corner.
    pub at: P2,
    /// Turned anticlockwise by this much (degrees) about its stock box's centre.
    pub angle: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Nest {
    /// Sheet size X, Y (mm); None = the programmed stock's size.
    pub size: Option<[f64; 2]>,
    /// The copies. The sheet's lower-left corner is the programmed stock's,
    /// so a copy at [0, 0], not turned, is the part as programmed.
    pub copies: Vec<NestCopy>,
    /// Program zero; None = the step milling origin's modes, on the sheet.
    pub origin: Option<Origin>,
    /// Grid layout helper: columns, rows, gap between the copies' footprints
    /// and the angle every copy is laid out at.
    pub cols: usize,
    pub rows: usize,
    pub gap: [f64; 2],
    pub angle: f64,
}

impl Default for Nest {
    fn default() -> Self {
        Nest { size: None, copies: vec![NestCopy::default()], origin: None, cols: 1, rows: 1, gap: [5.0, 5.0], angle: 0.0 }
    }
}

/// The programmed stock box's corners (XY), turned by `angle` (degrees)
/// about its centre.
fn turned_box(stock: (P3, P3), angle: f64) -> [P2; 4] {
    let (lo, hi) = stock;
    let p = Placed { pivot: [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0], angle: angle.to_radians(), shift: [0.0, 0.0] };
    [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]].map(|q| p.apply(q))
}

/// Lower-left corner and size of the footprint of a box.
fn bounds(c: &[P2]) -> (P2, P2) {
    let lo = c.iter().fold([f64::INFINITY; 2], |a, p| [a[0].min(p[0]), a[1].min(p[1])]);
    let hi = c.iter().fold([f64::NEG_INFINITY; 2], |a, p| [a[0].max(p[0]), a[1].max(p[1])]);
    (lo, [hi[0] - lo[0], hi[1] - lo[1]])
}

/// A copy's footprint size (X, Y) at `angle` (degrees).
fn footprint(stock: (P3, P3), angle: f64) -> P2 {
    bounds(&turned_box(stock, angle)).1
}

impl Nest {
    /// The sheet box for the programmed stock box `stock`.
    pub fn sheet(&self, stock: (P3, P3)) -> (P3, P3) {
        let (lo, hi) = stock;
        let [w, h] = self.size.unwrap_or([hi[0] - lo[0], hi[1] - lo[1]]);
        (lo, [lo[0] + w, lo[1] + h, hi[2]])
    }

    /// Where copy `c` puts the part (model coordinates).
    pub fn placed(&self, c: &NestCopy, stock: (P3, P3)) -> Placed {
        let (lo, hi) = stock;
        let (blo, _) = bounds(&turned_box(stock, c.angle));
        let sheet = self.sheet(stock).0;
        Placed {
            pivot: [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0],
            angle: c.angle.to_radians(),
            shift: [sheet[0] + c.at[0] - blo[0], sheet[1] + c.at[1] - blo[1]],
        }
    }

    pub fn placements(&self, stock: (P3, P3)) -> Vec<Placed> {
        self.copies.iter().map(|c| self.placed(c, stock)).collect()
    }

    /// Copy `c`'s stock box corners on the sheet (model coordinates).
    pub fn corners(&self, c: &NestCopy, stock: (P3, P3)) -> [P2; 4] {
        let p = self.placed(c, stock);
        let (lo, hi) = stock;
        [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]].map(|q| p.apply(q))
    }

    /// Turn copy `i` by `delta` degrees, keeping its footprint's centre.
    pub fn turn(&mut self, i: usize, delta: f64, stock: (P3, P3)) {
        let Some(c) = self.copies.get_mut(i) else { return };
        let before = footprint(stock, c.angle);
        c.angle = (c.angle + delta).rem_euclid(360.0);
        if (c.angle - 360.0).abs() < 1e-9 {
            c.angle = 0.0;
        }
        let after = footprint(stock, c.angle);
        c.at = [c.at[0] + (before[0] - after[0]) / 2.0, c.at[1] + (before[1] - after[1]) / 2.0];
    }

    /// Program zero (model coordinates) on the sheet.
    pub fn origin_at(&self, step_origin: &Origin, stock: (P3, P3)) -> P3 {
        let mut o = self.origin.unwrap_or(*step_origin);
        let (a, b) = self.sheet(stock);
        o.resolve(a, b);
        o.at
    }

    /// Lay the copies out `cols` x `rows` at the grid angle, `gap` apart,
    /// from the sheet's lower-left corner. A sheet too small for them grows
    /// to fit; returns whether it did.
    pub fn lay_out(&mut self, stock: (P3, P3)) -> bool {
        let [w, h] = footprint(stock, self.angle);
        let (cols, rows) = (self.cols.max(1), self.rows.max(1));
        let gap = [self.gap[0].max(0.0), self.gap[1].max(0.0)];
        let angle = self.angle;
        self.copies = (0..rows).flat_map(|r| (0..cols).map(move |c| NestCopy { at: [c as f64 * (w + gap[0]), r as f64 * (h + gap[1])], angle })).collect();
        let need = [cols as f64 * w + (cols - 1) as f64 * gap[0], rows as f64 * h + (rows - 1) as f64 * gap[1]];
        let (a, b) = self.sheet(stock);
        let have = [b[0] - a[0], b[1] - a[1]];
        if need[0] > have[0] + 1e-9 || need[1] > have[1] + 1e-9 {
            self.size = Some([need[0].max(have[0]), need[1].max(have[1])]);
            true
        } else {
            false
        }
    }

    /// As many columns and rows as fit on the sheet with the gap.
    pub fn fill(&mut self, stock: (P3, P3)) {
        let [w, h] = footprint(stock, self.angle);
        let (a, b) = self.sheet(stock);
        let fit = |len: f64, size: f64, gap: f64| (((len + gap.max(0.0)) / (size + gap.max(0.0)) + 1e-9).floor() as usize).max(1);
        self.cols = fit(b[0] - a[0], w, self.gap[0]);
        self.rows = fit(b[1] - a[1], h, self.gap[1]);
        self.lay_out(stock);
    }

    /// A new copy (at the last one's angle): right of the last one, gap
    /// apart, else above the others.
    pub fn next_copy(&self, stock: (P3, P3)) -> NestCopy {
        let (a, b) = self.sheet(stock);
        let Some(last) = self.copies.last() else { return NestCopy::default() };
        let [w, _] = footprint(stock, last.angle);
        let [nw, _] = footprint(stock, last.angle);
        let right = [last.at[0] + w + self.gap[0], last.at[1]];
        if right[0] + nw <= b[0] - a[0] + 1e-9 {
            NestCopy { at: right, angle: last.angle }
        } else {
            let top = self.copies.iter().map(|c| c.at[1] + footprint(stock, c.angle)[1]).fold(0.0, f64::max);
            NestCopy { at: [0.0, top + self.gap[1]], angle: last.angle }
        }
    }

    /// Copies off the sheet or overlapping each other.
    pub fn warnings(&self, stock: (P3, P3)) -> Vec<String> {
        let (a, b) = self.sheet(stock);
        let e = 1e-6;
        let boxes: Vec<[P2; 4]> = self.copies.iter().map(|c| self.corners(c, stock)).collect();
        let mut out = vec![];
        for (i, q) in boxes.iter().enumerate() {
            if q.iter().any(|p| p[0] < a[0] - e || p[1] < a[1] - e || p[0] > b[0] + e || p[1] > b[1] + e) {
                out.push(format!("copy {} is not all on the sheet", i + 1));
            }
        }
        for i in 0..boxes.len() {
            for j in i + 1..boxes.len() {
                if overlap(&boxes[i], &boxes[j]) {
                    out.push(format!("copies {} and {} overlap", i + 1, j + 1));
                }
            }
        }
        out
    }

    /// The nested program.
    pub fn code(&self, job: &StepJob, s: &Settings, stock: (P3, P3)) -> Result<String, Vec<String>> {
        stepmill::generate_at(job, s, &self.placements(stock), self.origin_at(&job.origin, stock))
    }

    /// One copy where it was programmed, sheet and origin as programmed.
    pub fn is_single(&self) -> bool {
        self.size.is_none() && self.origin.is_none() && self.copies == [NestCopy::default()]
    }
}

/// Two convex quadrilaterals overlap by more than a hair (separating axes).
fn overlap(p: &[P2; 4], q: &[P2; 4]) -> bool {
    for poly in [p, q] {
        for k in 0..4 {
            let (a, b) = (poly[k], poly[(k + 1) % 4]);
            let n = [b[1] - a[1], a[0] - b[0]];
            let l = n[0].hypot(n[1]);
            if l < 1e-12 {
                continue;
            }
            let proj = |s: &[P2; 4]| s.iter().map(|v| (v[0] * n[0] + v[1] * n[1]) / l).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)));
            let ((a0, a1), (b0, b1)) = (proj(p), proj(q));
            if a1 <= b0 + 1e-6 || b1 <= a0 + 1e-6 {
                return false;
            }
        }
    }
    true
}

fn inside_quad(q: &[P2; 4], p: P2) -> bool {
    let s: Vec<f64> = (0..4).map(|k| {
        let (a, b) = (q[k], q[(k + 1) % 4]);
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    }).collect();
    s.iter().all(|&v| v >= 0.0) || s.iter().all(|&v| v <= 0.0)
}

/// What the Nest tab asks the app to do.
pub enum Action {
    SaveGcode,
    Simulate,
}

/// What the tab shows besides the nest itself.
pub struct Ctx<'a> {
    /// Programmed stock box (model coordinates).
    pub stock: (P3, P3),
    pub step_origin: &'a Origin,
    /// The part seen from above.
    pub outline: &'a [Vec<P2>],
    /// One copy's toolpath (model coordinates).
    pub toolpath: &'a crate::toolpath::Toolpath,
    /// The nested program's run time (s) and errors.
    pub run_time: f64,
    pub errors: &'a [String],
}

#[derive(Default)]
pub struct NestView {
    /// Copy being dragged.
    drag: Option<usize>,
    show_paths: bool,
}

impl NestView {
    pub fn new() -> Self {
        NestView { drag: None, show_paths: true }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, nest: &mut Nest, cx: &Ctx) -> Option<Action> {
        let mut action = None;
        let (w, h, t) = (cx.stock.1[0] - cx.stock.0[0], cx.stock.1[1] - cx.stock.0[1], cx.stock.1[2] - cx.stock.0[2]);
        egui::SidePanel::left("nest_panel").resizable(true).default_width(360.0).show_inside(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("Sheet");
                ui.weak(format!("Each copy's stock: {} x {} x {} mm (as programmed)", fmt(w), fmt(h), fmt(t)));
                let (a, b) = nest.sheet(cx.stock);
                let mut same = nest.size.is_none();
                if ui.checkbox(&mut same, "Same size as the programmed stock").changed() {
                    nest.size = if same { None } else { Some([b[0] - a[0], b[1] - a[1]]) };
                }
                ui.add_enabled_ui(!same, |ui| {
                    egui::Grid::new("nest_sheet").num_columns(2).show(ui, |ui| {
                        let mut sz = [b[0] - a[0], b[1] - a[1]];
                        for (k, l) in ["X", "Y"].iter().enumerate() {
                            ui.label(*l);
                            if ui.add(egui::DragValue::new(&mut sz[k]).speed(0.5).range(0.1..=100_000.0).max_decimals(3).suffix(" mm")).changed() {
                                nest.size = Some(sz);
                            }
                            ui.end_row();
                        }
                        ui.label("Z");
                        ui.label(format!("{} mm (the programmed stock's)", fmt(t)));
                        ui.end_row();
                    });
                });

                ui.add_space(8.0);
                ui.heading("Grid");
                egui::Grid::new("nest_grid").num_columns(2).show(ui, |ui| {
                    ui.label("Columns");
                    ui.add(egui::DragValue::new(&mut nest.cols).range(1..=500));
                    ui.end_row();
                    ui.label("Rows");
                    ui.add(egui::DragValue::new(&mut nest.rows).range(1..=500));
                    ui.end_row();
                    for (k, l) in ["Gap X", "Gap Y"].iter().enumerate() {
                        ui.label(*l).on_hover_text("Between the copies' footprints");
                        ui.add(egui::DragValue::new(&mut nest.gap[k]).speed(0.1).range(0.0..=10_000.0).max_decimals(3).suffix(" mm"));
                        ui.end_row();
                    }
                    ui.label("Angle").on_hover_text("Every copy of the grid turned this much (anticlockwise)");
                    ui.add(egui::DragValue::new(&mut nest.angle).speed(1.0).range(-360.0..=360.0).max_decimals(3).suffix("°"));
                    ui.end_row();
                });
                ui.horizontal(|ui| {
                    if ui.button("Lay out").on_hover_text("Columns x rows from the sheet's lower-left corner (the sheet grows if they do not fit)").clicked() {
                        nest.lay_out(cx.stock);
                    }
                    if ui.button("Fill sheet").on_hover_text("As many columns and rows as fit on the sheet with the gap").clicked() {
                        nest.fill(cx.stock);
                    }
                });

                ui.add_space(8.0);
                ui.heading(format!("Copies ({})", nest.copies.len()));
                ui.weak("Lower-left corner of each copy's footprint from the sheet's, and its angle. Drag copies in the view; right-click one to turn it 90°.");
                let mut remove = None;
                let mut turn = None;
                egui::Grid::new("nest_copies").num_columns(6).striped(true).show(ui, |ui| {
                    for (i, c) in nest.copies.iter_mut().enumerate() {
                        ui.label(format!("{}", i + 1));
                        ui.add(egui::DragValue::new(&mut c.at[0]).speed(0.1).max_decimals(3).prefix("X "));
                        ui.add(egui::DragValue::new(&mut c.at[1]).speed(0.1).max_decimals(3).prefix("Y "));
                        let mut a = c.angle;
                        if ui.add(egui::DragValue::new(&mut a).speed(1.0).max_decimals(3).suffix("°")).on_hover_text("Turned anticlockwise about its centre").changed() {
                            turn = Some((i, a - c.angle));
                        }
                        if ui.small_button("⟳").on_hover_text("Turn 90° clockwise").clicked() {
                            turn = Some((i, -90.0));
                        }
                        if ui.small_button("🗑").on_hover_text("Remove this copy").clicked() {
                            remove = Some(i);
                        }
                        ui.end_row();
                    }
                });
                if let Some((i, d)) = turn {
                    nest.turn(i, d, cx.stock);
                }
                if let Some(i) = remove {
                    nest.copies.remove(i);
                }
                if ui.button("➕ Add copy").clicked() {
                    let c = nest.next_copy(cx.stock);
                    nest.copies.push(c);
                }

                ui.add_space(8.0);
                ui.heading("G-code origin");
                let mut own = nest.origin.is_some();
                if ui.checkbox(&mut own, "Own origin").on_hover_text("Unticked: the Step Milling tab's origin modes, on the sheet").changed() {
                    nest.origin = own.then_some(*cx.step_origin);
                }
                if let Some(o) = &mut nest.origin {
                    ui.horizontal(|ui| {
                        ui.label("XY");
                        egui::ComboBox::from_id_salt("nest_origin_xy").selected_text(o.xy.name()).show_ui(ui, |ui| {
                            for m in OriginXY::ALL {
                                ui.selectable_value(&mut o.xy, m, m.name());
                            }
                        });
                        ui.label("Z");
                        egui::ComboBox::from_id_salt("nest_origin_z").selected_text(o.z.name()).show_ui(ui, |ui| {
                            for m in OriginZ::ALL {
                                ui.selectable_value(&mut o.z, m, m.name());
                            }
                        });
                    });
                    ui.horizontal(|ui| {
                        for (k, a) in ["X", "Y", "Z"].iter().enumerate() {
                            let mut v = o.at[k];
                            if ui.add(egui::DragValue::new(&mut v).speed(0.1).max_decimals(4).prefix(format!("{a} "))).on_hover_text("Typing a value makes this axis Custom").changed() {
                                let mut set = [None; 3];
                                set[k] = Some(v);
                                o.set(set[0], set[1], set[2]);
                            }
                        }
                    });
                }
                let at = nest.origin_at(cx.step_origin, cx.stock);
                ui.weak(format!("Program zero at X{} Y{} Z{} (model coordinates)", fmt(at[0]), fmt(at[1]), fmt(at[2])));

                ui.add_space(8.0);
                ui.heading("Program");
                for e in cx.errors {
                    ui.colored_label(ui.visuals().error_fg_color, format!("✖ {e}"));
                }
                for wn in nest.warnings(cx.stock) {
                    ui.colored_label(ui.visuals().warn_fg_color, format!("⚠ {wn}"));
                }
                if cx.errors.is_empty() {
                    ui.label(format!("{} cop{}, estimated run time {}", nest.copies.len(), if nest.copies.len() == 1 { "y" } else { "ies" }, crate::estimate::format_time(cx.run_time)));
                }
                ui.horizontal(|ui| {
                    let ok = cx.errors.is_empty() && !nest.copies.is_empty();
                    if ui.add_enabled(ok, egui::Button::new("💾 Save nested G-code…")).clicked() {
                        action = Some(Action::SaveGcode);
                    }
                    if ui.add_enabled(ok, egui::Button::new("▶ Simulate")).on_hover_text("Open the nest in the Simulator tab").clicked() {
                        action = Some(Action::Simulate);
                    }
                });
                ui.checkbox(&mut self.show_paths, "Show toolpaths");
            });
        });
        egui::CentralPanel::default().show_inside(ui, |ui| self.canvas(ui, nest, cx));
        action
    }

    /// Top view of the sheet and the copies; copies can be dragged, and
    /// turned 90° with a right-click.
    fn canvas(&mut self, ui: &mut egui::Ui, nest: &mut Nest, cx: &Ctx) {
        let (rect, resp) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        let dark = ui.visuals().dark_mode;
        painter.rect_filled(rect, 0.0, if dark { Color32::from_gray(28) } else { Color32::from_gray(235) });
        let (sa, sb) = nest.sheet(cx.stock);
        let boxes: Vec<[P2; 4]> = nest.copies.iter().map(|c| nest.corners(c, cx.stock)).collect();
        // everything in view: the sheet and every copy
        let (mut lo, mut hi) = ([sa[0], sa[1]], [sb[0], sb[1]]);
        for p in boxes.iter().flatten() {
            lo = [lo[0].min(p[0]), lo[1].min(p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(p[1])];
        }
        let m = 24.0;
        let scale = ((rect.width() - 2.0 * m) as f64 / (hi[0] - lo[0]).max(1e-6)).min((rect.height() - 2.0 * m) as f64 / (hi[1] - lo[1]).max(1e-6)).max(1e-6);
        let c0 = rect.center();
        let mid = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
        let to = |p: P2| Pos2::new(c0.x + ((p[0] - mid[0]) * scale) as f32, c0.y - ((p[1] - mid[1]) * scale) as f32);
        let from = |q: Pos2| [mid[0] + (q.x - c0.x) as f64 / scale, mid[1] - (q.y - c0.y) as f64 / scale];

        // sheet
        let sheet = [[sa[0], sa[1]], [sb[0], sa[1]], [sb[0], sb[1]], [sa[0], sb[1]]].map(to).to_vec();
        painter.add(egui::Shape::convex_polygon(sheet, if dark { Color32::from_rgb(70, 62, 50) } else { Color32::from_rgb(222, 205, 175) }, Stroke::new(1.5_f32, Color32::from_rgb(150, 120, 80))));

        let under = |q: P2| boxes.iter().rposition(|b| inside_quad(b, q));
        // drag a copy
        if resp.drag_started() {
            self.drag = resp.interact_pointer_pos().and_then(|p| under(from(p)));
        }
        if let Some(i) = self.drag {
            if resp.dragged() {
                let d = resp.drag_delta();
                if let Some(c) = nest.copies.get_mut(i) {
                    c.at[0] += d.x as f64 / scale;
                    c.at[1] -= d.y as f64 / scale;
                }
            }
            if resp.drag_stopped() {
                // to a tenth of a millimetre
                if let Some(c) = nest.copies.get_mut(i) {
                    c.at = [(c.at[0] * 10.0).round() / 10.0, (c.at[1] * 10.0).round() / 10.0];
                }
                self.drag = None;
            }
        }
        // right-click: turn 90° clockwise
        if resp.secondary_clicked() {
            if let Some(i) = resp.interact_pointer_pos().and_then(|p| under(from(p))) {
                nest.turn(i, -90.0, cx.stock);
            }
        }
        let hover = resp.hover_pos().map(from);
        let hovered = hover.and_then(under);

        let warn = nest.warnings(cx.stock);
        for (i, c) in nest.copies.iter().enumerate() {
            let place = nest.placed(c, cx.stock);
            let corners = nest.corners(c, cx.stock);
            let n = i + 1;
            let bad = warn.iter().any(|x| x == &format!("copy {n} is not all on the sheet") || x.starts_with(&format!("copies {n} and ")) || x.ends_with(&format!(" and {n} overlap")));
            let hot = self.drag == Some(i) || hovered == Some(i);
            let fill = if dark { Color32::from_rgba_unmultiplied(120, 140, 170, if hot { 70 } else { 40 }) } else { Color32::from_rgba_unmultiplied(80, 110, 160, if hot { 60 } else { 30 }) };
            let edge = if bad { Color32::from_rgb(220, 70, 60) } else if dark { Color32::from_gray(170) } else { Color32::from_gray(90) };
            painter.add(egui::Shape::convex_polygon(corners.map(to).to_vec(), fill, Stroke::new(if hot { 2.0_f32 } else { 1.0 }, edge)));
            // the part, placed with the copy
            for l in cx.outline {
                let pts: Vec<Pos2> = l.iter().map(|p| to(place.apply(*p))).collect();
                if pts.len() >= 3 {
                    painter.add(egui::Shape::closed_line(pts, Stroke::new(1.5_f32, if dark { Color32::from_rgb(200, 205, 215) } else { Color32::from_rgb(50, 60, 80) })));
                }
            }
            if self.show_paths {
                for p in cx.toolpath.paths.iter().filter(|p| p.kind != crate::toolpath::MoveKind::Rapid) {
                    let pts: Vec<Pos2> = p.pts.iter().map(|q| to(place.apply([q[0] as f64, q[1] as f64]))).collect();
                    if pts.len() >= 2 {
                        painter.add(egui::Shape::line(pts, Stroke::new(0.7_f32, Color32::from_rgba_unmultiplied(60, 170, 90, 110))));
                    }
                }
            }
            // number at the copy's own lower-left corner (shows which way it is turned)
            let label = if c.angle.abs() > 1e-9 { format!("{n} ({}°)", fmt(c.angle)) } else { format!("{n}") };
            painter.text(to(corners[0]) + Vec2::new(4.0, -4.0), egui::Align2::LEFT_BOTTOM, label, egui::FontId::proportional(13.0), edge);
        }

        // program zero
        let at = nest.origin_at(cx.step_origin, cx.stock);
        let p = to([at[0], at[1]]);
        let col = Color32::from_rgb(230, 60, 200);
        painter.line_segment([p + Vec2::new(-8.0, 0.0), p + Vec2::new(8.0, 0.0)], Stroke::new(2.0_f32, col));
        painter.line_segment([p + Vec2::new(0.0, -8.0), p + Vec2::new(0.0, 8.0)], Stroke::new(2.0_f32, col));
        painter.circle_stroke(p, 4.0, Stroke::new(1.5_f32, col));

        painter.text(
            rect.left_bottom() + Vec2::new(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("Sheet {} x {} mm - drag a copy to move it, right-click it to turn it 90°", fmt(sb[0] - sa[0]), fmt(sb[1] - sa[1])),
            egui::FontId::proportional(12.0),
            ui.visuals().weak_text_color(),
        );
        if self.drag.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else if hovered.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        }
    }
}

fn fmt(v: f64) -> String {
    crate::gcode::fmt(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    const STOCK: (P3, P3) = ([10.0, 20.0, -5.0], [40.0, 40.0, 0.0]); // 30 x 20 x 5

    fn at(x: f64, y: f64) -> NestCopy {
        NestCopy { at: [x, y], angle: 0.0 }
    }

    fn close(a: P2, b: P2) -> bool {
        (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9
    }

    #[test]
    fn one_copy_on_the_programmed_stock_by_default() {
        let n = Nest::default();
        assert!(n.is_single());
        assert_eq!(n.sheet(STOCK), STOCK);
        assert!(n.warnings(STOCK).is_empty());
        // and it leaves the part where it is
        let p = n.placed(&n.copies[0], STOCK);
        assert!(close(p.apply([12.0, 33.0]), [12.0, 33.0]));
    }

    #[test]
    fn grid_lay_out_grows_the_sheet_and_fill_counts() {
        let mut n = Nest { cols: 3, rows: 2, gap: [5.0, 4.0], ..Default::default() };
        assert!(n.lay_out(STOCK), "the sheet grows");
        assert_eq!(n.copies, vec![at(0.0, 0.0), at(35.0, 0.0), at(70.0, 0.0), at(0.0, 24.0), at(35.0, 24.0), at(70.0, 24.0)]);
        assert_eq!(n.size, Some([100.0, 44.0]));
        assert!(n.warnings(STOCK).is_empty());
        let mut n = Nest { size: Some([110.0, 70.0]), gap: [5.0, 5.0], ..Default::default() };
        n.fill(STOCK);
        // 3 x 30 + 2 x 5 = 100 <= 110; 3 x 20 + 2 x 5 = 70 <= 70
        assert_eq!((n.cols, n.rows, n.copies.len()), (3, 3, 9));
        // the sheet keeps its size when they fit
        assert_eq!(n.size, Some([110.0, 70.0]));
        // next copy: right of the last if it fits, else a new row
        assert_eq!(n.next_copy(STOCK).at, [0.0, 75.0]);
        // turned 90°, the footprint is 20 x 30
        let mut n = Nest { size: Some([110.0, 70.0]), gap: [5.0, 5.0], angle: 90.0, ..Default::default() };
        n.fill(STOCK);
        assert_eq!((n.cols, n.rows), (4, 2));
        assert!(n.warnings(STOCK).is_empty(), "{:?}", n.warnings(STOCK));
    }

    #[test]
    fn overlaps_and_copies_off_the_sheet_are_warned() {
        let n = Nest { size: Some([50.0, 20.0]), copies: vec![at(0.0, 0.0), at(20.0, 0.0), at(50.0, 0.0)], ..Default::default() };
        let w = n.warnings(STOCK);
        assert!(w.contains(&"copies 1 and 2 overlap".to_string()), "{w:?}");
        assert!(w.contains(&"copy 3 is not all on the sheet".to_string()), "{w:?}");
        assert!(!w.iter().any(|x| x.contains("2 and 3")), "{w:?}");
        // turned 45°, a copy's corners poke out of its old place
        let n = Nest { size: Some([100.0, 100.0]), copies: vec![at(0.0, 0.0), NestCopy { at: [31.0, 0.0], angle: 45.0 }], ..Default::default() };
        assert!(n.warnings(STOCK).is_empty(), "footprints apart: {:?}", n.warnings(STOCK));
        let n = Nest { copies: vec![at(0.0, 0.0), NestCopy { at: [25.0, 0.0], angle: 45.0 }], size: Some([100.0, 100.0]), ..Default::default() };
        assert_eq!(n.warnings(STOCK), vec!["copies 1 and 2 overlap".to_string()]);
    }

    #[test]
    fn a_turned_copy_sits_with_its_footprint_at_its_corner() {
        let mut n = Nest { size: Some([100.0, 100.0]), copies: vec![at(10.0, 5.0)], ..Default::default() };
        n.turn(0, 90.0, STOCK);
        // the 30 x 20 box turned 90° is 20 x 30, about the same centre
        assert_eq!(n.copies[0].angle, 90.0);
        assert!(close(n.copies[0].at, [15.0, 0.0]));
        let c = n.corners(&n.copies[0], STOCK);
        let (lo, size) = bounds(&c);
        assert!(close(lo, [25.0, 20.0]) && close(size, [20.0, 30.0]), "{lo:?} {size:?}");
        // the stock's lower-left corner goes to the footprint's lower right
        assert!(close(c[0], [45.0, 20.0]), "{c:?}");
        // a full turn comes back
        for _ in 0..3 {
            n.turn(0, 90.0, STOCK);
        }
        assert_eq!(n.copies[0].angle, 0.0);
        assert!(close(n.copies[0].at, [10.0, 5.0]));
    }

    #[test]
    fn nested_program_cuts_every_copy_turned_or_not() {
        use stepmill::*;
        let m = crate::step::parse_model(include_str!("../testdata/bracket.step")).unwrap();
        let stock = Stock { x: 6.0, y: 6.0, z: 1.0, ..Default::default() }.bounds(m.min, m.max);
        let rough = RoughOp {
            cutter: crate::model::Cutter::C3x17,
            start_z: stock.1[2],
            bottom_z: m.min[2],
            faces: rough_faces(&m),
            stock_region: [stock.0[0], stock.0[1], stock.1[0], stock.1[1]],
            ..Default::default()
        };
        let mut finish = rough.clone();
        (finish.cutter, finish.start_z, finish.layer_height) = (crate::model::Cutter::C3_175x17, m.max[2], 0.5);
        let job = StepJob {
            ops: vec![StepOp { enabled: true, kind: OpKind::Rough(rough) }, StepOp { enabled: true, kind: OpKind::Finish(finish) }],
            origin: Origin { xy: OriginXY::BottomLeft, z: OriginZ::StockTop, at: [0.0; 3] },
            ..Default::default()
        };
        let s = Settings { safe_z: 25.0, ..Default::default() };
        let mut n = Nest { cols: 2, rows: 2, gap: [4.0, 3.0], ..Default::default() };
        n.lay_out(stock);
        // one copy turned 90°, one 30° (on a bigger sheet)
        n.size = Some([200.0, 160.0]);
        n.turn(1, 90.0, stock);
        // (turned about its centre it pokes below the sheet: back up)
        n.copies[1].at[1] = 0.0;
        n.copies[3].at = [120.0, 60.0];
        n.turn(3, 30.0, stock);
        assert!(n.warnings(stock).is_empty(), "{:?}", n.warnings(stock));
        let one = generate(&job, &s).unwrap();
        let code = n.code(&job, &s, stock).unwrap();
        let changes = |c: &str| c.lines().filter(|l| l.contains("M6")).count();
        assert_eq!(changes(&code), changes(&one), "one tool change per op");
        assert_eq!(code.matches("(copy ").count(), 8, "two ops on four copies");
        // simulated on the sheet against the four parts, each where its copy is
        let sheet = n.sheet(stock);
        let origin = n.origin_at(&job.origin, stock);
        assert_eq!(origin, [sheet.0[0], sheet.0[1], sheet.1[2]]);
        let mut prog = crate::sim::Program::parse(&code, &s);
        prog.translate(origin);
        let mut sim = crate::sim::Sim::new(prog, sheet.0, sheet.1, 500);
        while !sim.seek(f64::INFINITY, std::time::Duration::from_secs(60)) {}
        assert!(!sim.events.values().any(|e| e.kind == crate::sim::EventKind::RapidCut), "{:?}", sim.events);
        let places = n.placements(stock);
        let tris: Vec<[P3; 3]> = places.iter().flat_map(|pl| crate::sim::top_tris(&m).map(move |t| t.map(|p| pl.apply3(p)))).collect();
        let part = sim.map.surface_of(tris.iter());
        let (mut cells, mut at, mut worst) = (0, 0, 0f32);
        let mut worst_at = [0.0; 2];
        for (k, (z, p)) in sim.map.z.iter().zip(&part).enumerate().filter(|(_, (_, p))| !p.is_nan()) {
            cells += 1;
            at += (z - p < 0.3) as usize;
            // (cells on the part's edge straddle its wall: not counted)
            let (i, j) = (k % sim.map.nx, k / sim.map.nx);
            let edge = [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)].iter().any(|(di, dj)| {
                let (a, b) = (i as i64 + di, j as i64 + dj);
                a < 0 || b < 0 || a >= sim.map.nx as i64 || b >= sim.map.ny as i64 || part[b as usize * sim.map.nx + a as usize].is_nan()
            });
            if !edge && p - z > worst {
                worst = p - z;
                let c = sim.map.centre(k % sim.map.nx, k / sim.map.nx);
                worst_at = [c[0], c[1]];
            }
        }
        let which: Vec<usize> = (0..n.copies.len()).filter(|&i| inside_quad(&n.corners(&n.copies[i], stock), worst_at)).collect();
        assert!(worst < 0.07, "gouge {worst} at {worst_at:?} (in copies {which:?}; copies {:?})", n.copies);
        assert!(at as f64 > 0.85 * cells as f64, "every copy cut: {at} of {cells} cells at size");
    }

    #[test]
    fn origin_follows_the_step_modes_on_the_sheet() {
        let step = Origin { xy: OriginXY::TopRight, z: OriginZ::StockTop, at: [0.0; 3] };
        let n = Nest { size: Some([100.0, 50.0]), ..Default::default() };
        assert_eq!(n.origin_at(&step, STOCK), [110.0, 70.0, 0.0]);
        let own = Nest { origin: Some(Origin { xy: OriginXY::BottomLeft, z: OriginZ::StockBase, at: [0.0; 3] }), ..n };
        assert_eq!(own.origin_at(&step, STOCK), [10.0, 20.0, -5.0]);
    }
}
