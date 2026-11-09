//! Text engraving on a flat surface: the tool centre follows the outlines of
//! the glyphs of a TrueType / OpenType font, cutting `depth` into the surface.

use crate::model::{Cutter, Engraver, Settings};
use ab_glyph::{Font, FontArc, OutlineCurve};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

pub type P2 = [f64; 2];

/// Built-in fonts (shipped with egui); any other `font` value is a file path.
pub const FONT_SANS: &str = "Sans";
pub const FONT_MONO: &str = "Mono";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HAlign {
    Left,
    Center,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum VAlign {
    Top,
    Center,
    Bottom,
}

/// Engraving can use a tapered bit or an end mill.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum EngraveTool {
    Bit(Engraver),
    EndMill(Cutter),
}

impl Default for EngraveTool {
    fn default() -> Self {
        EngraveTool::Bit(Engraver::DEFAULTS[1])
    }
}

impl EngraveTool {
    pub fn tool_number(self) -> u32 {
        match self {
            EngraveTool::Bit(b) => b.tool,
            EngraveTool::EndMill(c) => c.tool,
        }
    }
    pub fn name(self) -> String {
        match self {
            EngraveTool::Bit(b) => format!("T{} engraver {}", b.tool, b.name()),
            EngraveTool::EndMill(c) => format!("T{} end mill {}", c.tool, c.name()),
        }
    }
    /// Width of the cut at `depth`.
    pub fn width_at(self, depth: f64) -> f64 {
        match self {
            EngraveTool::Bit(b) => b.width_at(depth),
            EngraveTool::EndMill(c) => c.dia,
        }
    }
    /// Deepest cut the tool allows.
    pub fn max_depth(self) -> f64 {
        match self {
            EngraveTool::Bit(b) => b.max_depth,
            EngraveTool::EndMill(c) => c.flute,
        }
    }
    /// Tool change comment.
    pub fn describe(self) -> String {
        match self {
            EngraveTool::Bit(b) => format!("engraver {}deg tip Ø{} max depth {}", crate::gcode::fmt(b.angle), crate::gcode::fmt(b.tip), crate::gcode::fmt(b.max_depth)),
            EngraveTool::EndMill(c) => format!("endmill Ø{} flute {}", crate::gcode::fmt(c.dia), crate::gcode::fmt(c.flute)),
        }
    }
    pub fn in_table(self, s: &Settings) -> bool {
        match self {
            EngraveTool::Bit(b) => s.lookup_engraver(b).is_some(),
            EngraveTool::EndMill(c) => s.lookup(c).is_some(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EngraveOp {
    pub face_id: usize,
    /// Z of the surface (top of the engraving).
    pub z: f64,
    /// Centre of the surface's outer boundary (XY) - the text origin.
    pub origin: P2,
    pub text: String,
    /// Capital letter height (mm).
    pub height: f64,
    /// Engraving depth below the surface (mm).
    pub depth: f64,
    pub depth_per_pass: f64,
    pub tool: EngraveTool,
    /// `FONT_SANS`, `FONT_MONO` or a path to a .ttf / .otf file.
    pub font: String,
    pub h_align: HAlign,
    pub v_align: VAlign,
    /// Offset of the text anchor from the surface centre.
    pub x: f64,
    pub y: f64,
    pub feed: f64,
    pub speed: f64,
}

impl Default for EngraveOp {
    fn default() -> Self {
        EngraveOp {
            face_id: 0,
            z: 0.0,
            origin: [0.0; 2],
            text: "TEXT".into(),
            height: 5.0,
            depth: 0.3,
            depth_per_pass: 0.3,
            tool: EngraveTool::default(),
            font: FONT_SANS.into(),
            h_align: HAlign::Center,
            v_align: VAlign::Center,
            x: 0.0,
            y: 0.0,
            feed: 300.0,
            speed: 12000.0,
        }
    }
}

impl EngraveOp {
    pub fn bottom(&self) -> f64 {
        self.z - self.depth
    }
}

/// Short display name of a font setting.
pub fn font_name(font: &str) -> String {
    match font {
        FONT_SANS => "Sans (built-in)".into(),
        FONT_MONO => "Mono (built-in)".into(),
        p => std::path::Path::new(p).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| p.to_string()),
    }
}

/// Loaded fonts by setting; failures are cached too so a bad path is not
/// re-read every frame.
fn load_font(font: &str) -> Result<Arc<FontArc>, String> {
    static CACHE: OnceLock<Mutex<HashMap<String, Result<Arc<FontArc>, String>>>> = OnceLock::new();
    let mut cache = CACHE.get_or_init(Default::default).lock().unwrap();
    cache
        .entry(font.to_string())
        .or_insert_with(|| {
            let f = match font {
                FONT_SANS => FontArc::try_from_slice(epaint_default_fonts::UBUNTU_LIGHT),
                FONT_MONO => FontArc::try_from_slice(epaint_default_fonts::HACK_REGULAR),
                path => {
                    let data = std::fs::read(path).map_err(|e| format!("cannot read font {}: {e}", font_name(path)))?;
                    FontArc::try_from_vec(data)
                }
            };
            f.map(Arc::new).map_err(|_| format!("{} is not a valid TrueType / OpenType font", font_name(font)))
        })
        .clone()
}

/// Number of line segments for a curve with control polygon length `len`
/// (font units) at `scale` mm per unit: about 0.05 mm per segment.
fn segs(len: f32, scale: f64) -> usize {
    ((len as f64 * scale / 0.05).ceil() as usize).clamp(1, 64)
}

/// Closed outline contours of one glyph in font units (y up).
fn glyph_contours(font: &FontArc, id: ab_glyph::GlyphId, scale: f64) -> Vec<Vec<[f32; 2]>> {
    let Some(outline) = font.outline(id) else { return vec![] };
    let mut contours: Vec<Vec<[f32; 2]>> = vec![];
    let close = |a: [f32; 2], b: ab_glyph::Point| (a[0] - b.x).abs() < 1e-3 && (a[1] - b.y).abs() < 1e-3;
    let dist = |a: ab_glyph::Point, b: ab_glyph::Point| (a.x - b.x).hypot(a.y - b.y);
    for c in &outline.curves {
        let start = match c {
            OutlineCurve::Line(a, _) | OutlineCurve::Quad(a, _, _) | OutlineCurve::Cubic(a, _, _, _) => *a,
        };
        match contours.last() {
            Some(cur) if close(*cur.last().unwrap(), start) => {}
            _ => contours.push(vec![[start.x, start.y]]),
        }
        let cur = contours.last_mut().unwrap();
        match *c {
            OutlineCurve::Line(_, b) => cur.push([b.x, b.y]),
            OutlineCurve::Quad(a, b, e) => {
                let n = segs(dist(a, b) + dist(b, e), scale);
                for i in 1..=n {
                    let t = i as f32 / n as f32;
                    let u = 1.0 - t;
                    cur.push([u * u * a.x + 2.0 * u * t * b.x + t * t * e.x, u * u * a.y + 2.0 * u * t * b.y + t * t * e.y]);
                }
            }
            OutlineCurve::Cubic(a, b, c2, e) => {
                let n = segs(dist(a, b) + dist(b, c2) + dist(c2, e), scale);
                for i in 1..=n {
                    let t = i as f32 / n as f32;
                    let u = 1.0 - t;
                    let (k0, k1, k2, k3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
                    cur.push([k0 * a.x + k1 * b.x + k2 * c2.x + k3 * e.x, k0 * a.y + k1 * b.y + k2 * c2.y + k3 * e.y]);
                }
            }
        }
    }
    // every contour ends where it starts
    for c in &mut contours {
        if c.len() > 1 && c.first() != c.last() {
            let f = c[0];
            c.push(f);
        }
    }
    contours.retain(|c| c.len() >= 3);
    contours
}

/// Tool centre paths for the text in machine XY: closed contours, each
/// ending where it starts. Lines in `text` are stacked downwards.
pub fn text_paths(op: &EngraveOp) -> Result<Vec<Vec<P2>>, String> {
    if op.text.trim().is_empty() {
        return Err("no text to engrave".into());
    }
    if !(op.height > 0.0) {
        return Err("text height must be > 0".into());
    }
    let font = load_font(&op.font)?;
    // scale so that a capital H is `height` tall
    let cap = font
        .outline(font.glyph_id('H'))
        .map(|o| o.bounds.min.y.max(o.bounds.max.y)) // ab_glyph stores y_max in min.y
        .filter(|&h| h > 0.0)
        .unwrap_or_else(|| font.ascent_unscaled() * 0.7) as f64;
    let scale = op.height / cap;
    let line_h = (font.height_unscaled() + font.line_gap_unscaled()) as f64 * scale;

    let lines: Vec<&str> = op.text.lines().collect();
    let n = lines.len();
    // baseline of the first line, relative to the anchor
    let block = op.height + line_h * (n - 1) as f64; // cap top of line 1 to baseline of the last
    let first_base = match op.v_align {
        VAlign::Top => -op.height,
        VAlign::Center => block / 2.0 - op.height,
        VAlign::Bottom => block - op.height,
    };
    let (ax, ay) = (op.origin[0] + op.x, op.origin[1] + op.y);

    let mut out = vec![];
    for (li, line) in lines.iter().enumerate() {
        // lay out the glyphs of the line in font units
        let mut pen = 0.0f32;
        let mut prev = None;
        let mut glyphs = vec![];
        for ch in line.chars() {
            let id = font.glyph_id(ch);
            if let Some(p) = prev {
                pen += font.kern_unscaled(p, id);
            }
            glyphs.push((id, pen));
            pen += font.h_advance_unscaled(id);
            prev = Some(id);
        }
        let width = pen as f64 * scale;
        let x0 = match op.h_align {
            HAlign::Left => 0.0,
            HAlign::Center => -width / 2.0,
            HAlign::Right => -width,
        };
        let base = first_base - line_h * li as f64;
        for (id, gx) in glyphs {
            for c in glyph_contours(&font, id, scale) {
                out.push(c.iter().map(|p| [ax + x0 + (gx + p[0]) as f64 * scale, ay + base + p[1] as f64 * scale]).collect());
            }
        }
    }
    if out.is_empty() {
        return Err(format!("{} has no outlines for this text", font_name(&op.font)));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds(paths: &[Vec<P2>]) -> (P2, P2) {
        paths.iter().flatten().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| {
            ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])
        })
    }

    #[test]
    fn cap_height_and_alignment() {
        let mut op = EngraveOp { text: "HI".into(), height: 10.0, origin: [50.0, 20.0], ..Default::default() };
        for (v, lo_y, hi_y) in [(VAlign::Top, 10.0, 20.0), (VAlign::Center, 15.0, 25.0), (VAlign::Bottom, 20.0, 30.0)] {
            op.v_align = v;
            let (lo, hi) = bounds(&text_paths(&op).unwrap());
            assert!((lo[1] - lo_y).abs() < 0.05 && (hi[1] - hi_y).abs() < 0.05, "{v:?}: {lo:?} {hi:?}");
        }
        op.v_align = VAlign::Bottom;
        op.h_align = HAlign::Left;
        let (lo, _) = bounds(&text_paths(&op).unwrap());
        assert!(lo[0] >= 50.0 && lo[0] < 52.0, "{lo:?}");
        op.h_align = HAlign::Right;
        let (_, hi) = bounds(&text_paths(&op).unwrap());
        assert!(hi[0] <= 50.0 && hi[0] > 48.0, "{hi:?}");
        op.h_align = HAlign::Center;
        op.x = 5.0;
        op.y = -3.0;
        let (lo, hi) = bounds(&text_paths(&op).unwrap());
        assert!(((lo[0] + hi[0]) / 2.0 - 55.0).abs() < 1.0 && (lo[1] - 17.0).abs() < 0.05, "{lo:?} {hi:?}");
    }

    #[test]
    fn contours_are_closed_and_o_has_a_hole() {
        let op = EngraveOp { text: "O".into(), font: FONT_MONO.into(), ..Default::default() };
        let paths = text_paths(&op).unwrap();
        assert_eq!(paths.len(), 2);
        for p in &paths {
            assert_eq!(p.first(), p.last());
        }
    }

    #[test]
    fn multiline_stacks_down_and_errors() {
        let one = EngraveOp { text: "A".into(), v_align: VAlign::Top, ..Default::default() };
        let two = EngraveOp { text: "A\nA".into(), ..one.clone() };
        let (lo1, _) = bounds(&text_paths(&one).unwrap());
        let (lo2, hi2) = bounds(&text_paths(&two).unwrap());
        assert!(lo2[1] < lo1[1] - 4.0 && (hi2[1] - 0.0).abs() < 0.2);
        assert!(text_paths(&EngraveOp { text: "  ".into(), ..Default::default() }).is_err());
        assert!(text_paths(&EngraveOp { font: "/nonexistent.ttf".into(), ..Default::default() }).is_err());
    }
}
