//! In-app help: the user guide (docs/user-guide.md, built into the binary)
//! shown in a window with a contents list and search.
//!
//! Renders the Markdown subset the guide uses: `#` headings, paragraphs,
//! `-` / `1.` lists (nested by indent, with indented continuation
//! paragraphs), pipe tables, ``` code blocks, `**bold**`, `*italic*` and
//! `` `code` ``.

use eframe::egui::{self, text::LayoutJob, Color32, FontId, TextFormat};

pub const GUIDE: &str = include_str!("../docs/user-guide.md");

#[derive(Clone, Debug, PartialEq)]
enum Marker {
    Bullet,
    Number(String),
    /// Continuation paragraph of the item above.
    None,
}

#[derive(Clone, Debug, PartialEq)]
enum Block {
    Heading(usize, String),
    Para(String),
    /// (indent level, marker, text)
    List(Vec<(usize, Marker, String)>),
    /// Header row, body rows.
    Table(Vec<String>, Vec<Vec<String>>),
    /// Fenced code block, lines as written.
    Code(String),
}

fn list_item(line: &str) -> Option<(usize, Marker, String)> {
    let indent = line.len() - line.trim_start().len();
    let t = line.trim_start();
    if let Some(rest) = t.strip_prefix("- ").or_else(|| t.strip_prefix("* ")) {
        return Some((indent / 2, Marker::Bullet, rest.trim().to_string()));
    }
    let digits: String = t.chars().take_while(|c| c.is_ascii_digit()).collect();
    if !digits.is_empty() {
        if let Some(rest) = t[digits.len()..].strip_prefix(". ") {
            return Some((indent / 2, Marker::Number(digits), rest.trim().to_string()));
        }
    }
    None
}

fn cells(line: &str) -> Vec<String> {
    let t = line.trim().trim_start_matches('|').trim_end_matches('|');
    t.split('|').map(|c| c.trim().to_string()).collect()
}

fn parse(md: &str) -> Vec<Block> {
    let mut out: Vec<Block> = vec![];
    let mut para: Vec<String> = vec![];
    let mut blank = false;
    let flush = |para: &mut Vec<String>, out: &mut Vec<Block>| {
        if !para.is_empty() {
            out.push(Block::Para(para.join(" ")));
            para.clear();
        }
    };
    let mut code: Option<Vec<&str>> = None;
    for line in md.lines() {
        let t = line.trim();
        if t.starts_with("```") {
            match code.take() {
                Some(c) => out.push(Block::Code(c.join("\n"))),
                None => {
                    flush(&mut para, &mut out);
                    code = Some(vec![]);
                }
            }
            continue;
        }
        if let Some(c) = &mut code {
            c.push(line);
            continue;
        }
        if t.is_empty() {
            flush(&mut para, &mut out);
            blank = true;
            continue;
        }
        let was_blank = std::mem::replace(&mut blank, false);
        if let Some(h) = t.strip_prefix('#') {
            flush(&mut para, &mut out);
            let level = 1 + h.chars().take_while(|&c| c == '#').count();
            out.push(Block::Heading(level, h.trim_start_matches('#').trim().to_string()));
            continue;
        }
        if t.starts_with('|') {
            flush(&mut para, &mut out);
            let row = cells(t);
            if row.iter().all(|c| !c.is_empty() && c.chars().all(|ch| ch == '-' || ch == ':')) {
                continue; // separator row
            }
            match out.last_mut() {
                Some(Block::Table(_, body)) if !was_blank => body.push(row),
                _ => out.push(Block::Table(row, vec![])),
            }
            continue;
        }
        if let Some(item) = list_item(line) {
            flush(&mut para, &mut out);
            match out.last_mut() {
                Some(Block::List(items)) => items.push(item),
                _ => out.push(Block::List(vec![item])),
            }
            continue;
        }
        let indent = line.len() - line.trim_start().len();
        if let (true, Some(Block::List(items))) = (para.is_empty() && indent >= 2, out.last_mut()) {
            // indented text under a list item: continues it (or a new paragraph of it after a blank line)
            let level = (indent / 2).saturating_sub(1);
            if was_blank {
                items.push((level, Marker::None, t.to_string()));
            } else if let Some(last) = items.last_mut() {
                last.2.push(' ');
                last.2.push_str(t);
            }
            continue;
        }
        para.push(t.to_string());
    }
    flush(&mut para, &mut out);
    out
}

/// Inline `**bold**`, `*italic*`, `` `code` `` into a layout job, with
/// matches of `find` (lower case) highlighted.
fn inline(ui: &egui::Ui, text: &str, size: f32, strong: bool, find: &str) -> LayoutJob {
    let v = ui.visuals();
    let mut job = LayoutJob::default();
    let (mut bold, mut italic, mut code) = (false, false, false);
    let mut buf = String::new();
    let push = |job: &mut LayoutJob, s: &str, bold: bool, italic: bool, code: bool| {
        if s.is_empty() {
            return;
        }
        let fmt = TextFormat {
            font_id: if code { FontId::monospace(size * 0.92) } else { FontId::proportional(size) },
            color: if bold || strong { v.strong_text_color() } else { v.text_color() },
            italics: italic,
            background: if code { v.code_bg_color } else { Color32::TRANSPARENT },
            ..Default::default()
        };
        // highlight search matches (ASCII-safe lower-casing only)
        let low = s.to_lowercase();
        if find.is_empty() || low.len() != s.len() {
            job.append(s, 0.0, fmt);
            return;
        }
        let mut at = 0;
        while let Some(k) = low[at..].find(find) {
            let (a, b) = (at + k, at + k + find.len());
            job.append(&s[at..a], 0.0, fmt.clone());
            job.append(&s[a..b], 0.0, TextFormat { background: Color32::from_rgba_unmultiplied(255, 210, 0, 110), ..fmt.clone() });
            at = b;
        }
        job.append(&s[at..], 0.0, fmt);
    };
    let ch: Vec<char> = text.chars().collect();
    let mut i = 0;
    while i < ch.len() {
        let c = ch[i];
        if c == '`' {
            push(&mut job, &buf, bold, italic, code);
            buf.clear();
            code = !code;
        } else if !code && c == '*' && ch.get(i + 1) == Some(&'*') {
            push(&mut job, &buf, bold, italic, code);
            buf.clear();
            bold = !bold;
            i += 1;
        } else if !code && c == '*' && (italic || ch.get(i + 1).is_some_and(|n| !n.is_whitespace())) {
            push(&mut job, &buf, bold, italic, code);
            buf.clear();
            italic = !italic;
        } else {
            buf.push(c);
        }
        i += 1;
    }
    push(&mut job, &buf, bold, italic, code);
    job.wrap.max_width = ui.available_width();
    job
}

pub struct Help {
    pub open: bool,
    blocks: Vec<Block>,
    search: String,
    /// Heading (block index) to scroll to on the next frame.
    jump: Option<usize>,
}

impl Default for Help {
    fn default() -> Self {
        Help { open: false, blocks: parse(GUIDE), search: String::new(), jump: None }
    }
}

impl Help {
    /// Open the help at the section whose heading starts with `section`.
    pub fn open_at(&mut self, section: &str) {
        self.open = true;
        self.jump = self.blocks.iter().position(|b| matches!(b, Block::Heading(_, h) if h.starts_with(section)));
    }

    /// Text of each section (heading block index -> its text up to the next heading), lower case.
    fn section_text(&self, h: usize) -> String {
        let mut s = String::new();
        for b in &self.blocks[h..] {
            match b {
                Block::Heading(..) if !s.is_empty() => break,
                Block::Heading(_, t) | Block::Para(t) => s += t,
                Block::List(items) => items.iter().for_each(|i| s += &i.2),
                Block::Table(head, body) => head.iter().chain(body.iter().flatten()).for_each(|c| s += c),
                Block::Code(c) => s += c,
            }
            s.push(' ');
        }
        s.to_lowercase()
    }

    pub fn show(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.key_pressed(egui::Key::F1)) {
            self.open = !self.open;
        }
        if !self.open {
            return;
        }
        let mut open = self.open;
        egui::Window::new("❓ stepCAM Help").open(&mut open).default_size([920.0, 680.0]).resizable(true).collapsible(false).show(ctx, |ui| self.contents(ui));
        self.open = open;
    }

    fn contents(&mut self, ui: &mut egui::Ui) {
        let find = self.search.trim().to_lowercase();
        ui.horizontal(|ui| {
            ui.label("🔍");
            ui.add(egui::TextEdit::singleline(&mut self.search).hint_text("Search the guide").desired_width(220.0));
            if !self.search.is_empty() && ui.small_button("✖").clicked() {
                self.search.clear();
            }
        });
        ui.separator();
        // contents: level 2 and 3 headings, filtered by the search
        let heads: Vec<(usize, usize, String)> = self.blocks.iter().enumerate().filter_map(|(i, b)| match b {
            Block::Heading(l, t) if (2..=3).contains(l) => Some((i, *l, t.clone())),
            _ => None,
        }).collect();
        let hits: Vec<(usize, usize, String)> = heads.into_iter().filter(|(i, _, _)| find.is_empty() || self.section_text(*i).contains(&find)).collect();
        let height = ui.available_height();
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                ui.set_width(220.0);
                egui::ScrollArea::vertical().id_salt("help_toc").max_height(height).show(ui, |ui| {
                    if !find.is_empty() {
                        ui.weak(format!("{} section{} match", hits.len(), if hits.len() == 1 { "" } else { "s" }));
                    }
                    for (i, l, t) in &hits {
                        let text = if *l == 2 { egui::RichText::new(t).strong() } else { egui::RichText::new(format!("    {t}")) };
                        if ui.add(egui::Label::new(text).sense(egui::Sense::click())).on_hover_cursor(egui::CursorIcon::PointingHand).clicked() {
                            self.jump = Some(*i);
                        }
                    }
                });
            });
            ui.separator();
            egui::ScrollArea::vertical().id_salt("help_body").max_height(height).auto_shrink([false, false]).show(ui, |ui| {
                ui.set_max_width(760.0);
                self.body(ui, &find);
            });
        });
    }

    fn body(&mut self, ui: &mut egui::Ui, find: &str) {
        let body = 14.0;
        let jump = self.jump.take();
        for (bi, b) in self.blocks.iter().enumerate() {
            match b {
                Block::Heading(l, t) => {
                    let size = match l {
                        1 => 24.0,
                        2 => 20.0,
                        3 => 16.5,
                        _ => 15.0,
                    };
                    ui.add_space(if *l <= 2 { 14.0 } else { 8.0 });
                    let r = ui.label(inline(ui, t, size, true, find));
                    if *l == 2 {
                        ui.separator();
                    }
                    if jump == Some(bi) {
                        r.scroll_to_me(Some(egui::Align::TOP));
                    }
                }
                Block::Para(t) => {
                    ui.add_space(4.0);
                    ui.label(inline(ui, t, body, false, find));
                }
                Block::List(items) => {
                    ui.add_space(2.0);
                    for (level, marker, t) in items {
                        ui.horizontal_top(|ui| {
                            ui.add_space(8.0 + *level as f32 * 18.0);
                            let m = match marker {
                                Marker::Bullet => if *level == 0 { "•" } else { "◦" }.to_string(),
                                Marker::Number(n) => format!("{n}."),
                                Marker::None => String::new(),
                            };
                            let (r, _) = ui.allocate_exact_size(egui::vec2(18.0, body + 2.0), egui::Sense::hover());
                            ui.painter().text(r.left_top(), egui::Align2::LEFT_TOP, m, FontId::proportional(body), ui.visuals().text_color());
                            ui.label(inline(ui, t, body, false, find));
                        });
                    }
                }
                Block::Code(c) => {
                    ui.add_space(4.0);
                    egui::Frame::none().fill(ui.visuals().code_bg_color).inner_margin(8.0).rounding(4.0).show(ui, |ui| {
                        ui.set_min_width(ui.available_width());
                        ui.label(egui::RichText::new(c).monospace().size(body * 0.92));
                    });
                }
                Block::Table(head, rows) => {
                    ui.add_space(4.0);
                    egui::Grid::new(("help_table", bi)).striped(true).spacing([18.0, 4.0]).show(ui, |ui| {
                        for c in head {
                            ui.label(inline(ui, c, body, true, find));
                        }
                        ui.end_row();
                        for row in rows {
                            for c in row {
                                ui.label(inline(ui, c, body, false, find));
                            }
                            ui.end_row();
                        }
                    });
                }
            }
        }
        ui.add_space(20.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_markdown_subset() {
        let md = "# Title\n\nSome *text*\nwrapped.\n\n- one\n  more of one\n  - nested\n\n  second para of one\n- two\n1. first\n\n| A | B |\n|---|---|\n| x | y |\n| z | w |\n\nAfter.\n";
        let b = parse(md);
        assert_eq!(b[0], Block::Heading(1, "Title".into()));
        assert_eq!(b[1], Block::Para("Some *text* wrapped.".into()));
        let Block::List(items) = &b[2] else { panic!("{:?}", b[2]) };
        assert_eq!(items[0], (0, Marker::Bullet, "one more of one".into()));
        assert_eq!(items[1], (1, Marker::Bullet, "nested".into()));
        assert_eq!(items[2], (0, Marker::None, "second para of one".into()));
        assert_eq!(items[3], (0, Marker::Bullet, "two".into()));
        assert_eq!(items[4], (0, Marker::Number("1".into()), "first".into()));
        assert_eq!(b[3], Block::Table(vec!["A".into(), "B".into()], vec![vec!["x".into(), "y".into()], vec!["z".into(), "w".into()]]));
        assert_eq!(b[4], Block::Para("After.".into()));
        let b = parse("Text\n```\nx = 1\n  - not a list\n```\nMore\n");
        assert_eq!(b, vec![Block::Para("Text".into()), Block::Code("x = 1\n  - not a list".into()), Block::Para("More".into())]);
    }

    #[test]
    fn guide_has_a_section_for_every_tab() {
        let h = Help::default();
        for tab in ["Thread Milling tab", "Step Milling tab", "Simulator tab", "Tools & Settings tab", "Getting started"] {
            let mut h2 = Help { blocks: h.blocks.clone(), ..Default::default() };
            h2.open_at(tab);
            assert!(h2.jump.is_some(), "no section {tab}");
        }
        assert!(h.section_text(h.blocks.iter().position(|b| matches!(b, Block::Heading(_, t) if t == "Simulator tab")).unwrap()).contains("material being cut"));
    }

    #[test]
    fn help_window_renders_headless() {
        let ctx = egui::Context::default();
        let mut h = Help::default();
        h.open_at("Simulator tab");
        h.search = "gouge".into();
        for _ in 0..3 {
            let _ = ctx.run(egui::RawInput { screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1400.0, 850.0))), ..Default::default() }, |ctx| h.show(ctx));
        }
        assert!(h.open);
    }
}
