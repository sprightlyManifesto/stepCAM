//! Simple orthographic 3D viewer for toolpaths and STEP models, drawn with the
//! egui painter.
//!
//! Left drag: orbit · right/middle drag: pan · scroll: zoom at cursor ·
//! double-click: fit · click / right-click: pick (when a model is shown).

use crate::step::Model;
use crate::stepmill::MillTab;
use crate::toolpath::{MoveKind, Toolpath};
use eframe::egui::{self, Color32, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use std::f32::consts::{FRAC_PI_2, PI};

/// Hole outline drawn as a wireframe cylinder.
pub struct HoleViz {
    pub x: f32,
    pub y: f32,
    pub dia: f32,
    pub top: f32,
    pub bottom: f32,
}

/// A STEP model to draw (and pick from), with the current selection.
pub struct ModelViz<'a> {
    pub model: &'a Model,
    pub sel_edges: &'a [usize],
    pub sel_faces: &'a [usize],
    pub tabs: &'a [MillTab],
    /// Stock box (min, max).
    pub stock: Option<([f64; 3], [f64; 3])>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Pick {
    /// Edge index, picked 3D point, XY tangent angle (radians) at that point.
    Edge { edge: usize, point: [f64; 3], angle: f64 },
    Face(usize),
}

pub struct ViewOut {
    pub response: egui::Response,
    /// Set when the user left-clicked: what was under the cursor (None = empty space).
    pub left_pick: Option<Option<Pick>>,
    /// Set when the user right-clicked.
    pub right_pick: Option<Option<Pick>>,
}

pub struct View3d {
    yaw: f32,
    pitch: f32,
    scale: f32,
    target: [f32; 3],
    pub show_rapid: bool,
    pub show_bore: bool,
    pub show_thread: bool,
    pub show_holes: bool,
    pub show_model: bool,
    pub needs_fit: bool,
    /// Legend labels for the two feed move kinds; empty hides the toggle.
    pub labels: [&'static str; 2],
    /// 0-based G-code line whose move is highlighted.
    pub highlight_line: Option<usize>,
}

impl Default for View3d {
    fn default() -> Self {
        let mut v = View3d {
            yaw: 0.0,
            pitch: 0.0,
            scale: 10.0,
            target: [0.0; 3],
            show_rapid: true,
            show_bore: true,
            show_thread: true,
            show_holes: true,
            show_model: true,
            needs_fit: true,
            labels: ["Hole milling", "Threading"],
            highlight_line: None,
        };
        v.iso();
        v
    }
}

/// Highlighted G-code move (cyan).
const HIGHLIGHT: Color32 = Color32::from_rgb(0, 230, 255);
const RAPID: Color32 = Color32::from_rgba_premultiplied(150, 50, 50, 150);
const BORE: Color32 = Color32::from_rgb(70, 150, 255);
const THREAD: Color32 = Color32::from_rgb(255, 165, 40);
const SELECT: Color32 = Color32::from_rgb(255, 220, 0);
const TAB: Color32 = Color32::from_rgb(60, 200, 90);

fn f3(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, p[1] as f32, p[2] as f32]
}

fn seg_dist(p: Pos2, a: Pos2, b: Pos2) -> (f32, f32) {
    let ab = b - a;
    let l2 = ab.length_sq();
    let t = if l2 < 1e-9 { 0.0 } else { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) };
    ((a + ab * t - p).length(), t)
}

impl View3d {
    pub fn with_labels(labels: [&'static str; 2]) -> Self {
        View3d { labels, ..Default::default() }
    }

    fn iso(&mut self) {
        self.yaw = -PI / 6.0;
        self.pitch = PI / 3.5;
    }

    /// Rotate a world-space direction into view space (x right, y depth away from viewer, z up).
    fn rot(&self, d: [f32; 3]) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let x1 = d[0] * cy - d[1] * sy;
        let y1 = d[0] * sy + d[1] * cy;
        [x1, y1 * cp - d[2] * sp, y1 * sp + d[2] * cp]
    }

    /// World point -> view space, relative to the target.
    fn to_view(&self, p: [f32; 3]) -> [f32; 3] {
        self.rot([p[0] - self.target[0], p[1] - self.target[1], p[2] - self.target[2]])
    }

    /// Inverse rotation of a view-space vector back into world space.
    fn view_vec_to_world(&self, v: [f32; 3]) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let y1 = v[1] * cp + v[2] * sp;
        let z1 = -v[1] * sp + v[2] * cp;
        [v[0] * cy + y1 * sy, -v[0] * sy + y1 * cy, z1]
    }

    fn project(&self, rect: Rect, p: [f32; 3]) -> Pos2 {
        let v = self.to_view(p);
        rect.center() + Vec2::new(v[0] * self.scale, -v[2] * self.scale)
    }

    fn fit(&mut self, rect: Rect, min: [f32; 3], max: [f32; 3]) {
        self.target = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0, (min[2] + max[2]) / 2.0];
        let (mut ex, mut ez) = (1e-3f32, 1e-3f32);
        for &x in &[min[0], max[0]] {
            for &y in &[min[1], max[1]] {
                for &z in &[min[2], max[2]] {
                    let v = self.to_view([x, y, z]);
                    ex = ex.max(v[0].abs());
                    ez = ez.max(v[2].abs());
                }
            }
        }
        self.scale = (rect.width() / (2.0 * ex)).min(rect.height() / (2.0 * ez)) * 0.85;
    }

    fn pick(&self, rect: Rect, m: &Model, pos: Pos2) -> Option<Pick> {
        // edges within 6 px win over faces
        let mut best: Option<(f32, usize, [f64; 3], f64)> = None;
        for (ei, e) in m.edges.iter().enumerate() {
            for w in e.pts.windows(2) {
                let (a, b) = (self.project(rect, f3(w[0])), self.project(rect, f3(w[1])));
                let (d, t) = seg_dist(pos, a, b);
                if d < 6.0 && best.map_or(true, |bb| d < bb.0) {
                    let t = t as f64;
                    let p = [w[0][0] + (w[1][0] - w[0][0]) * t, w[0][1] + (w[1][1] - w[0][1]) * t, w[0][2] + (w[1][2] - w[0][2]) * t];
                    let ang = (w[1][1] - w[0][1]).atan2(w[1][0] - w[0][0]);
                    best = Some((d, ei, p, ang));
                }
            }
        }
        if let Some((_, edge, point, angle)) = best {
            return Some(Pick::Edge { edge, point, angle });
        }
        // nearest face triangle under the cursor
        let mut best_face: Option<(f32, usize)> = None;
        for (fi, f) in m.faces.iter().enumerate() {
            for t in &f.tris {
                let p: Vec<Pos2> = t.iter().map(|&q| self.project(rect, f3(q))).collect();
                let den = (p[1].y - p[2].y) * (p[0].x - p[2].x) + (p[2].x - p[1].x) * (p[0].y - p[2].y);
                if den.abs() < 1e-6 {
                    continue;
                }
                let l0 = ((p[1].y - p[2].y) * (pos.x - p[2].x) + (p[2].x - p[1].x) * (pos.y - p[2].y)) / den;
                let l1 = ((p[2].y - p[0].y) * (pos.x - p[2].x) + (p[0].x - p[2].x) * (pos.y - p[2].y)) / den;
                let l2 = 1.0 - l0 - l1;
                if l0 >= 0.0 && l1 >= 0.0 && l2 >= 0.0 {
                    let depth = l0 * self.to_view(f3(t[0]))[1] + l1 * self.to_view(f3(t[1]))[1] + l2 * self.to_view(f3(t[2]))[1];
                    if best_face.map_or(true, |b| depth < b.0) {
                        best_face = Some((depth, fi));
                    }
                }
            }
        }
        best_face.map(|(_, f)| Pick::Face(f))
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, tp: &Toolpath, holes: &[HoleViz], model: Option<&ModelViz>) -> ViewOut {
        // bounds of everything drawable
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        let mut grow = |p: [f32; 3]| {
            for k in 0..3 {
                min[k] = min[k].min(p[k]);
                max[k] = max[k].max(p[k]);
            }
        };
        if !tp.is_empty() {
            grow(tp.min);
            grow(tp.max);
        }
        for h in holes {
            let r = h.dia / 2.0;
            grow([h.x - r, h.y - r, h.bottom]);
            grow([h.x + r, h.y + r, h.top]);
        }
        if let Some(mv) = model {
            grow(f3(mv.model.min));
            grow(f3(mv.model.max));
        }
        let has_bounds = min[0].is_finite();

        ui.horizontal(|ui| {
            ui.strong("3D view");
            ui.separator();
            ui.checkbox(&mut self.show_rapid, egui::RichText::new("Rapids").color(RAPID.to_opaque()));
            if !self.labels[0].is_empty() {
                ui.checkbox(&mut self.show_bore, egui::RichText::new(self.labels[0]).color(BORE));
            }
            if !self.labels[1].is_empty() {
                ui.checkbox(&mut self.show_thread, egui::RichText::new(self.labels[1]).color(THREAD));
            }
            if model.is_some() {
                ui.checkbox(&mut self.show_model, "Model");
            } else {
                ui.checkbox(&mut self.show_holes, "Holes");
            }
            ui.separator();
            if ui.button("Iso").clicked() {
                self.iso();
                self.needs_fit = true;
            }
            if ui.button("Top").clicked() {
                self.yaw = 0.0;
                self.pitch = FRAC_PI_2;
                self.needs_fit = true;
            }
            if ui.button("Front").clicked() {
                self.yaw = 0.0;
                self.pitch = 0.0;
                self.needs_fit = true;
            }
            if ui.button("Side").clicked() {
                self.yaw = -FRAC_PI_2;
                self.pitch = 0.0;
                self.needs_fit = true;
            }
            if ui.button("Fit").clicked() {
                self.needs_fit = true;
            }
            ui.separator();
            ui.weak("drag: orbit · right/middle drag: pan · scroll: zoom · double-click: fit");
        });

        let (rect, resp) = ui.allocate_exact_size(ui.available_size().max(Vec2::splat(50.0)), Sense::click_and_drag());

        // --- interaction
        if resp.dragged_by(egui::PointerButton::Primary) {
            let d = resp.drag_delta();
            self.yaw += d.x * 0.01;
            self.pitch = (self.pitch + d.y * 0.01).clamp(-FRAC_PI_2, FRAC_PI_2);
        }
        if resp.dragged_by(egui::PointerButton::Secondary) || resp.dragged_by(egui::PointerButton::Middle) {
            let d = resp.drag_delta();
            let w = self.view_vec_to_world([d.x / self.scale, 0.0, -d.y / self.scale]);
            for k in 0..3 {
                self.target[k] -= w[k];
            }
        }
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let old = self.scale;
                let new = (old * (scroll * 0.003).exp()).clamp(0.01, 1e5);
                if let Some(m) = resp.hover_pos() {
                    // keep the point under the cursor fixed
                    let d = m - rect.center();
                    let dv = [d.x / new - d.x / old, 0.0, -d.y / new + d.y / old];
                    let w = self.view_vec_to_world(dv);
                    for k in 0..3 {
                        self.target[k] -= w[k];
                    }
                }
                self.scale = new;
            }
        }
        if resp.double_clicked() {
            self.needs_fit = true;
        }
        if self.needs_fit && has_bounds {
            self.fit(rect, min, max);
            self.needs_fit = false;
        }

        let mut left_pick = None;
        let mut right_pick = None;
        if let (Some(mv), true) = (model, self.show_model) {
            if let Some(pos) = resp.interact_pointer_pos() {
                if resp.clicked() && !resp.double_clicked() {
                    left_pick = Some(self.pick(rect, mv.model, pos));
                }
                if resp.secondary_clicked() {
                    right_pick = Some(self.pick(rect, mv.model, pos));
                }
            }
        }

        // --- drawing
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);

        if !has_bounds {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                if model.is_some() || self.labels[1].is_empty() { "Import a STEP file to begin" } else { "No toolpath - add locations to a valid group" },
                egui::FontId::proportional(14.0),
                ui.visuals().weak_text_color(),
            );
            return ViewOut { response: resp, left_pick, right_pick };
        }

        // ground grid on the XY plane
        let grid_z = match model {
            Some(mv) => mv.model.min[2] as f32,
            None => {
                let z = holes.iter().map(|h| h.top).fold(f32::NEG_INFINITY, f32::max);
                if z.is_finite() {
                    z
                } else {
                    0.0
                }
            }
        };
        let span = (max[0] - min[0]).max(max[1] - min[1]).max(1.0);
        let step = nice_step(span / 8.0);
        let gx0 = (min[0] / step).floor() * step - step;
        let gx1 = (max[0] / step).ceil() * step + step;
        let gy0 = (min[1] / step).floor() * step - step;
        let gy1 = (max[1] / step).ceil() * step + step;
        let grid_stroke = Stroke::new(1.0_f32, ui.visuals().widgets.noninteractive.bg_stroke.color.gamma_multiply(0.6));
        let mut x = gx0;
        while x <= gx1 + 1e-3 {
            painter.line_segment([self.project(rect, [x, gy0, grid_z]), self.project(rect, [x, gy1, grid_z])], grid_stroke);
            x += step;
        }
        let mut y = gy0;
        while y <= gy1 + 1e-3 {
            painter.line_segment([self.project(rect, [gx0, y, grid_z]), self.project(rect, [gx1, y, grid_z])], grid_stroke);
            y += step;
        }

        if let (Some(mv), true) = (model, self.show_model) {
            self.draw_model(ui, &painter, rect, mv);
        }

        if self.show_holes && model.is_none() {
            let col = ui.visuals().text_color().gamma_multiply(0.35);
            let stroke = Stroke::new(1.0_f32, col);
            for h in holes {
                let r = h.dia / 2.0;
                let circle = |z: f32| -> Vec<Pos2> {
                    (0..=32)
                        .map(|k| {
                            let a = k as f32 / 32.0 * 2.0 * PI;
                            self.project(rect, [h.x + r * a.cos(), h.y + r * a.sin(), z])
                        })
                        .collect()
                };
                painter.add(Shape::line(circle(h.top), stroke));
                painter.add(Shape::line(circle(h.bottom), stroke));
                for k in 0..4 {
                    let a = k as f32 * FRAC_PI_2 + self.yaw;
                    let (dx, dy) = (r * a.cos(), r * a.sin());
                    painter.line_segment(
                        [self.project(rect, [h.x + dx, h.y - dy, h.top]), self.project(rect, [h.x + dx, h.y - dy, h.bottom])],
                        stroke,
                    );
                }
            }
        }

        for (kind, show, color, width) in [
            (MoveKind::Rapid, self.show_rapid, RAPID, 1.0_f32),
            (MoveKind::Bore, self.show_bore, BORE, 1.2),
            (MoveKind::Thread, self.show_thread, THREAD, 1.5),
        ] {
            if !show {
                continue;
            }
            let stroke = Stroke::new(width, color);
            for p in tp.paths.iter().filter(|p| p.kind == kind) {
                let pts: Vec<Pos2> = p.pts.iter().map(|&q| self.project(rect, q)).collect();
                painter.add(Shape::line(pts, stroke));
            }
        }

        // move belonging to the G-code line under the cursor
        if let Some(pts) = self.highlight_line.and_then(|l| tp.line_move(l)) {
            let pts: Vec<Pos2> = pts.iter().map(|&q| self.project(rect, q)).collect();
            let (a, b) = (pts[0], *pts.last().unwrap());
            painter.add(Shape::line(pts.clone(), Stroke::new(6.0_f32, Color32::from_rgba_unmultiplied(0, 0, 0, 120))));
            painter.add(Shape::line(pts, Stroke::new(3.5_f32, HIGHLIGHT)));
            painter.circle_stroke(a, 4.0, Stroke::new(1.5_f32, HIGHLIGHT));
            painter.circle_filled(b, 4.5, HIGHLIGHT);
        }

        // axis triad in the corner (directions only)
        let origin = rect.left_bottom() + Vec2::new(36.0, -36.0);
        for (axis, color, label) in [
            ([1.0, 0.0, 0.0], Color32::from_rgb(230, 70, 70), "X"),
            ([0.0, 1.0, 0.0], Color32::from_rgb(80, 200, 90), "Y"),
            ([0.0, 0.0, 1.0], Color32::from_rgb(80, 130, 255), "Z"),
        ] {
            let v = self.rot(axis);
            let tip = origin + Vec2::new(v[0], -v[2]) * 24.0;
            painter.line_segment([origin, tip], Stroke::new(2.0_f32, color));
            painter.text(tip, egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(12.0), color);
        }

        let info = if tp.is_empty() {
            format!("X {:.2} … {:.2}\nY {:.2} … {:.2}\nZ {:.2} … {:.2}", min[0], max[0], min[1], max[1], min[2], max[2])
        } else {
            format!(
                "{} moves\nX {:.2} … {:.2}\nY {:.2} … {:.2}\nZ {:.2} … {:.2}",
                tp.moves, tp.min[0], tp.max[0], tp.min[1], tp.max[1], tp.min[2], tp.max[2]
            )
        };
        painter.text(rect.right_top() + Vec2::new(-8.0, 8.0), egui::Align2::RIGHT_TOP, info, egui::FontId::monospace(11.0), ui.visuals().weak_text_color());

        ViewOut { response: resp, left_pick, right_pick }
    }

    fn draw_model(&self, ui: &egui::Ui, painter: &egui::Painter, rect: Rect, mv: &ModelViz) {
        let dark = ui.visuals().dark_mode;
        // flat faces: painter's algorithm, far to near, simple lambert shading
        let mut tris: Vec<(f32, [Pos2; 3], Color32)> = Vec::new();
        for (fi, f) in mv.model.faces.iter().enumerate() {
            let n = self.rot(f3(f.normal));
            // light from the viewer: brightness from how much the face points at us
            let light = 0.35 + 0.65 * n[1].abs().max(n[2].abs() * 0.8).min(1.0);
            let selected = mv.sel_faces.contains(&fi);
            let base = if selected {
                SELECT
            } else if f.is_up_horizontal() {
                Color32::from_rgb(150, 165, 185)
            } else {
                Color32::from_rgb(140, 140, 145)
            };
            let c = Color32::from_rgba_unmultiplied(
                (base.r() as f32 * light) as u8,
                (base.g() as f32 * light) as u8,
                (base.b() as f32 * light) as u8,
                if selected { 210 } else { 150 },
            );
            for t in &f.tris {
                let depth = t.iter().map(|&q| self.to_view(f3(q))[1]).sum::<f32>() / 3.0;
                // curved faces: shade each triangle by its own normal
                let c = if f.planar {
                    c
                } else {
                    let (a, b) = ([t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]], [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]]);
                    let n = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-12);
                    let n = self.rot(f3([n[0] / l, n[1] / l, n[2] / l]));
                    let light = 0.35 + 0.65 * n[1].abs().max(n[2].abs() * 0.8).min(1.0);
                    Color32::from_rgba_unmultiplied((base.r() as f32 * light) as u8, (base.g() as f32 * light) as u8, (base.b() as f32 * light) as u8, c.a())
                };
                tris.push((depth, [self.project(rect, f3(t[0])), self.project(rect, f3(t[1])), self.project(rect, f3(t[2]))], c));
            }
        }
        tris.sort_by(|a, b| b.0.total_cmp(&a.0));
        let mut mesh = egui::Mesh::default();
        for (_, p, c) in tris {
            let base = mesh.vertices.len() as u32;
            for q in p {
                mesh.colored_vertex(q, c);
            }
            mesh.add_triangle(base, base + 1, base + 2);
        }
        painter.add(Shape::mesh(mesh));

        let edge_col = if dark { Color32::from_gray(210) } else { Color32::from_gray(40) };
        for (ei, e) in mv.model.edges.iter().enumerate() {
            if mv.sel_edges.contains(&ei) {
                continue;
            }
            let pts: Vec<Pos2> = e.pts.iter().map(|&q| self.project(rect, f3(q))).collect();
            let col = if e.faces.is_empty() { Color32::from_rgb(200, 120, 255) } else { edge_col };
            painter.add(Shape::line(pts, Stroke::new(1.0_f32, col)));
        }
        for &ei in mv.sel_edges {
            if let Some(e) = mv.model.edges.get(ei) {
                let pts: Vec<Pos2> = e.pts.iter().map(|&q| self.project(rect, f3(q))).collect();
                painter.add(Shape::line(pts, Stroke::new(3.0_f32, SELECT)));
            }
        }
        // stock as a wire box
        if let Some((a, b)) = mv.stock {
            let s = Stroke::new(1.2_f32, Color32::from_rgba_unmultiplied(230, 160, 60, 200));
            let c = [[a[0], a[1]], [b[0], a[1]], [b[0], b[1]], [a[0], b[1]]];
            let p = |i: usize, z: f64| self.project(rect, [c[i][0] as f32, c[i][1] as f32, z as f32]);
            for i in 0..4 {
                let j = (i + 1) % 4;
                painter.line_segment([p(i, a[2]), p(j, a[2])], s);
                painter.line_segment([p(i, b[2]), p(j, b[2])], s);
                painter.line_segment([p(i, a[2]), p(i, b[2])], s);
            }
        }
        // tabs as green boxes
        for t in mv.tabs {
            let c = t.corners();
            let (z0, z1) = (t.center[2] as f32, t.top() as f32);
            let p = |i: usize, z: f32| self.project(rect, [c[i][0] as f32, c[i][1] as f32, z]);
            let s = Stroke::new(2.0_f32, TAB);
            for i in 0..4 {
                let j = (i + 1) % 4;
                painter.line_segment([p(i, z0), p(j, z0)], s);
                painter.line_segment([p(i, z1), p(j, z1)], s);
                painter.line_segment([p(i, z0), p(i, z1)], s);
            }
        }
    }
}

/// Round a step to 1, 2 or 5 x 10^n.
fn nice_step(raw: f32) -> f32 {
    let raw = raw.max(1e-3);
    let mag = 10f32.powf(raw.log10().floor());
    let n = raw / mag;
    let m = if n < 1.5 {
        1.0
    } else if n < 3.5 {
        2.0
    } else if n < 7.5 {
        5.0
    } else {
        10.0
    };
    m * mag
}
