//! Data model: thread sizes, cutters, thread mills, groups and job settings.

use serde::{Deserialize, Serialize};

/// Metric coarse thread sizes supported by stepCAM.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ThreadSize {
    M2,
    M2_5,
    M3,
    M4,
    M5,
    M6,
}

impl ThreadSize {
    pub const ALL: [ThreadSize; 6] = [
        ThreadSize::M2,
        ThreadSize::M2_5,
        ThreadSize::M3,
        ThreadSize::M4,
        ThreadSize::M5,
        ThreadSize::M6,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ThreadSize::M2 => "M2",
            ThreadSize::M2_5 => "M2.5",
            ThreadSize::M3 => "M3",
            ThreadSize::M4 => "M4",
            ThreadSize::M5 => "M5",
            ThreadSize::M6 => "M6",
        }
    }

    /// Nominal (major) diameter in mm.
    pub fn major_dia(self) -> f64 {
        match self {
            ThreadSize::M2 => 2.0,
            ThreadSize::M2_5 => 2.5,
            ThreadSize::M3 => 3.0,
            ThreadSize::M4 => 4.0,
            ThreadSize::M5 => 5.0,
            ThreadSize::M6 => 6.0,
        }
    }

    /// Diameter of the hole that must be milled before threading (mm).
    pub fn hole_dia(self) -> f64 {
        match self {
            ThreadSize::M2 => 1.6,
            ThreadSize::M2_5 => 2.05,
            ThreadSize::M3 => 2.5,
            ThreadSize::M4 => 3.3,
            ThreadSize::M5 => 4.2,
            ThreadSize::M6 => 5.0,
        }
    }

    /// Thread pitch (mm).
    pub fn pitch(self) -> f64 {
        match self {
            ThreadSize::M2 => 0.4,
            ThreadSize::M2_5 => 0.45,
            ThreadSize::M3 => 0.5,
            ThreadSize::M4 => 0.7,
            ThreadSize::M5 => 0.8,
            ThreadSize::M6 => 1.0,
        }
    }

    pub fn index(self) -> usize {
        Self::ALL.iter().position(|&t| t == self).unwrap()
    }
}

/// An end mill from the tool table (used for hole milling and step milling).
/// Groups and operations keep a copy; it is kept in sync with the table by
/// tool number.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "CutterRepr")]
pub struct Cutter {
    /// Stable identity of the tool-table row (0 = unknown, match by tool number).
    pub id: u32,
    /// Tool number in the machine's tool table.
    pub tool: u32,
    /// Cutting diameter (mm).
    pub dia: f64,
    /// Flute (cutting) length (mm).
    pub flute: f64,
}

/// Accepts the current form and the old fixed-list names ("C3x17", ...).
#[derive(Deserialize)]
#[serde(untagged)]
enum CutterRepr {
    Full {
        #[serde(default)]
        id: u32,
        tool: u32,
        dia: f64,
        flute: f64,
    },
    Legacy(String),
}

impl From<CutterRepr> for Cutter {
    fn from(r: CutterRepr) -> Self {
        match r {
            CutterRepr::Full { id, tool, dia, flute } => Cutter { id, tool, dia, flute },
            CutterRepr::Legacy(name) => match name.as_str() {
                "C1x3" => Cutter::C1x3,
                "C1_5x6" => Cutter::C1_5x6,
                "C2x12" => Cutter::C2x12,
                "C3x17" => Cutter::C3x17,
                _ => Cutter::C3_175x17,
            },
        }
    }
}

#[allow(non_upper_case_globals)]
impl Cutter {
    pub const C1x3: Cutter = Cutter { id: 1, tool: 1, dia: 1.0, flute: 3.0 };
    pub const C1_5x6: Cutter = Cutter { id: 2, tool: 2, dia: 1.5, flute: 6.0 };
    pub const C2x12: Cutter = Cutter { id: 3, tool: 3, dia: 2.0, flute: 12.0 };
    pub const C3x17: Cutter = Cutter { id: 4, tool: 4, dia: 3.0, flute: 17.0 };
    pub const C3_175x17: Cutter = Cutter { id: 5, tool: 5, dia: 3.175, flute: 17.0 };

    /// The default tool table.
    pub const DEFAULTS: [Cutter; 5] = [Cutter::C1x3, Cutter::C1_5x6, Cutter::C2x12, Cutter::C3x17, Cutter::C3_175x17];

    pub fn name(self) -> String {
        format!("{} x {}", crate::gcode::fmt(self.dia), crate::gcode::fmt(self.flute))
    }

    pub fn dia(self) -> f64 {
        self.dia
    }

    pub fn flute_len(self) -> f64 {
        self.flute
    }

    pub fn tool_number(self) -> u32 {
        self.tool
    }

    /// The milling cutter must be smaller than the hole diameter.
    pub fn fits(self, thread: ThreadSize) -> bool {
        self.dia() < thread.hole_dia()
    }

    /// Sensible default from `table`: the largest cutter that leaves some room
    /// in the hole (<= 90 % of the hole), else the largest one that fits at all.
    pub fn default_for(table: &[Cutter], thread: ThreadSize) -> Cutter {
        let hole = thread.hole_dia();
        let mut sorted: Vec<Cutter> = table.to_vec();
        sorted.sort_by(|a, b| a.dia.total_cmp(&b.dia));
        sorted
            .iter()
            .rev()
            .copied()
            .find(|c| c.dia() <= hole * 0.9)
            .or_else(|| sorted.iter().rev().copied().find(|c| c.fits(thread)))
            .or_else(|| sorted.first().copied())
            .unwrap_or(Cutter::C1x3)
    }
}

/// Tapered engraving bit (V-bit): `angle` is the included angle of the
/// cone, `tip` the diameter of the flat on the bottom.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Engraver {
    /// Stable identity of the tool-table row (0 = unknown, match by tool number).
    pub id: u32,
    pub tool: u32,
    /// Included angle of the taper (degrees).
    pub angle: f64,
    /// Diameter of the flat tip (mm).
    pub tip: f64,
    /// Deepest cut the taper allows (mm).
    pub max_depth: f64,
}

impl Default for Engraver {
    fn default() -> Self {
        Engraver::DEFAULTS[1]
    }
}

impl Engraver {
    pub const DEFAULTS: [Engraver; 3] = [
        Engraver { id: 1, tool: 21, angle: 30.0, tip: 0.1, max_depth: 2.0 },
        Engraver { id: 2, tool: 22, angle: 60.0, tip: 0.2, max_depth: 2.0 },
        Engraver { id: 3, tool: 23, angle: 90.0, tip: 0.3, max_depth: 2.0 },
    ];

    pub fn name(self) -> String {
        format!("{}° tip Ø{}", crate::gcode::fmt(self.angle), crate::gcode::fmt(self.tip))
    }

    /// Width of the cut at `depth` below the surface.
    pub fn width_at(self, depth: f64) -> f64 {
        self.tip + 2.0 * depth.max(0.0) * (self.angle.to_radians() / 2.0).tan()
    }
}

/// Single-profile thread mill used for one thread size.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThreadMill {
    /// Tool number in the machine's tool table (0 = default T11..T16).
    #[serde(default)]
    pub tool: u32,
    /// Cutting diameter (mm). Must be smaller than the hole diameter.
    pub dia: f64,
    /// Usable reach / neck length (mm).
    pub reach: f64,
}

impl ThreadMill {
    pub fn default_for(thread: ThreadSize) -> ThreadMill {
        let (dia, reach) = match thread {
            ThreadSize::M2 => (1.2, 6.0),
            ThreadSize::M2_5 => (1.6, 7.5),
            ThreadSize::M3 => (1.9, 9.0),
            ThreadSize::M4 => (2.5, 12.0),
            ThreadSize::M5 => (3.2, 15.0),
            ThreadSize::M6 => (4.0, 18.0),
        };
        ThreadMill { tool: Self::default_tool(thread), dia, reach }
    }

    /// Default thread mill numbers are T11..T16 (by thread size).
    pub fn default_tool(thread: ThreadSize) -> u32 {
        11 + thread.index() as u32
    }
}

/// One group of identical threaded holes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThreadGroup {
    pub thread: ThreadSize,
    /// Z where the hole starts; all hole geometry is below this.
    pub start_z: f64,
    /// Helical descent (ramp) angle for the hole milling op, in degrees.
    pub ramp_angle: f64,
    /// Thread length in mm (0..100).
    pub length: f64,
    pub cutter: Cutter,
    /// One "x, y" location per line.
    pub locations: String,
}

impl Default for ThreadGroup {
    fn default() -> Self {
        let thread = ThreadSize::M3;
        ThreadGroup {
            thread,
            start_z: 0.0,
            ramp_angle: 3.0,
            length: 6.0,
            cutter: Cutter::default_for(&Cutter::DEFAULTS, thread),
            locations: String::new(),
        }
    }
}

impl ThreadGroup {
    /// Total hole depth: thread length plus one pitch, so the thread
    /// stops one pitch above the bottom of the hole.
    pub fn hole_depth(&self) -> f64 {
        self.length + self.thread.pitch()
    }

    /// Parse the location list. Accepts "x,y", "x y", "x;y" or "X10 Y20"
    /// per line; blank lines and text after '#' or '(' are ignored.
    pub fn parse_locations(&self) -> Result<Vec<(f64, f64)>, String> {
        let mut out = Vec::new();
        for (n, raw) in self.locations.lines().enumerate() {
            let line = raw.split(['#', '(']).next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let cleaned: String = line
                .chars()
                .map(|c| match c {
                    'x' | 'X' | 'y' | 'Y' | ',' | ';' | '\t' => ' ',
                    other => other,
                })
                .collect();
            let nums: Vec<&str> = cleaned.split_whitespace().collect();
            if nums.len() != 2 {
                return Err(format!("line {}: expected \"x, y\" but got \"{}\"", n + 1, raw.trim()));
            }
            let x: f64 = nums[0]
                .parse()
                .map_err(|_| format!("line {}: bad X value \"{}\"", n + 1, nums[0]))?;
            let y: f64 = nums[1]
                .parse()
                .map_err(|_| format!("line {}: bad Y value \"{}\"", n + 1, nums[1]))?;
            if !x.is_finite() || !y.is_finite() {
                return Err(format!("line {}: value is not finite", n + 1));
            }
            out.push((x, y));
        }
        Ok(out)
    }
}

/// Machining parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Rapid plane Z (absolute) for tool changes and moves between groups.
    pub safe_z: f64,
    /// Clearance height above each group's hole start Z for rapids between holes.
    pub clearance: f64,

    /// Spindle speed for both milling and threading (rpm).
    pub spindle_rpm: f64,
    /// Milling feed at the cutter centre while helically boring (mm/min).
    pub bore_feed: f64,
    /// Default helical descent angle (degrees) given to new groups.
    pub ramp_angle: f64,
    /// Feed for Z moves from clearance to the hole start (mm/min).
    pub plunge_feed: f64,
    /// Radial stepover between concentric rings as % of cutter diameter
    /// (only used when the cutter is too small to clear the hole in one ring).
    pub bore_stepover_pct: f64,

    /// Feed at the cutting edge while thread milling (mm/min). The program
    /// feed at the tool centre is reduced automatically for the arc.
    pub thread_feed: f64,

    pub coolant: bool,
    /// Emit G43 H<n> after each tool change.
    pub tool_length_comp: bool,

    /// Hole / step milling end mills.
    pub cutters: Vec<Cutter>,
    /// Thread mill per thread size (indexed by ThreadSize::index()).
    pub thread_mills: Vec<ThreadMill>,
    /// Tapered engraving bits.
    pub engravers: Vec<Engraver>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            safe_z: 10.0,
            clearance: 2.0,
            spindle_rpm: 10000.0,
            bore_feed: 500.0,
            ramp_angle: 3.0,
            plunge_feed: 150.0,
            bore_stepover_pct: 40.0,
            thread_feed: 100.0,
            coolant: true,
            tool_length_comp: true,
            cutters: Cutter::DEFAULTS.to_vec(),
            thread_mills: ThreadSize::ALL.iter().map(|&t| ThreadMill::default_for(t)).collect(),
            engravers: Engraver::DEFAULTS.to_vec(),
        }
    }
}

impl Settings {
    pub fn thread_mill(&self, t: ThreadSize) -> ThreadMill {
        self.thread_mills
            .get(t.index())
            .cloned()
            .unwrap_or_else(|| ThreadMill::default_for(t))
    }

    /// The tool-table entry a group / operation's cutter refers to
    /// (by row id, or by tool number for copies without one).
    pub fn lookup(&self, c: Cutter) -> Option<Cutter> {
        if c.id != 0 {
            if let Some(t) = self.cutters.iter().find(|t| t.id == c.id) {
                return Some(*t);
            }
        }
        self.cutters.iter().copied().find(|t| t.tool == c.tool)
    }

    /// A row id not used by any table entry.
    pub fn new_cutter_id(&self) -> u32 {
        self.cutters.iter().map(|c| c.id).max().unwrap_or(0) + 1
    }

    /// The engraving-bit table entry `e` refers to (by row id, else tool number).
    pub fn lookup_engraver(&self, e: Engraver) -> Option<Engraver> {
        if e.id != 0 {
            if let Some(t) = self.engravers.iter().find(|t| t.id == e.id) {
                return Some(*t);
            }
        }
        self.engravers.iter().copied().find(|t| t.tool == e.tool)
    }

    pub fn new_engraver_id(&self) -> u32 {
        self.engravers.iter().map(|c| c.id).max().unwrap_or(0) + 1
    }

    /// Tool numbers used by all three tables.
    pub fn used_tool_numbers(&self) -> Vec<u32> {
        self.cutters.iter().map(|c| c.tool).chain(self.thread_mills.iter().map(|t| t.tool)).chain(self.engravers.iter().map(|e| e.tool)).collect()
    }

    /// Fill in missing table entries (older or hand-edited files).
    pub fn normalize(&mut self) {
        while self.thread_mills.len() < ThreadSize::ALL.len() {
            let t = ThreadSize::ALL[self.thread_mills.len()];
            self.thread_mills.push(ThreadMill::default_for(t));
        }
        self.thread_mills.truncate(ThreadSize::ALL.len());
        // give every tool-table row a unique id
        let mut used = std::collections::HashSet::new();
        for k in 0..self.cutters.len() {
            let c = self.cutters[k];
            if c.id == 0 || !used.insert(c.id) {
                let id = (c.tool..).chain(1..).find(|i| *i != 0 && !used.contains(i) && !self.cutters.iter().any(|o| o.id == *i)).unwrap();
                self.cutters[k].id = id;
                used.insert(id);
            }
        }
        let mut used = std::collections::HashSet::new();
        for k in 0..self.engravers.len() {
            let c = self.engravers[k];
            if c.id == 0 || !used.insert(c.id) {
                let id = (1..).find(|i| !used.contains(i) && !self.engravers.iter().any(|o| o.id == *i)).unwrap();
                self.engravers[k].id = id;
                used.insert(id);
            }
        }
        for (t, tm) in ThreadSize::ALL.iter().zip(self.thread_mills.iter_mut()) {
            if tm.tool == 0 {
                tm.tool = ThreadMill::default_tool(*t);
            }
        }
    }

    /// Problems with the tool tables (duplicate tool numbers etc.).
    pub fn tool_issues(&self) -> Vec<String> {
        let mut e = vec![];
        let mut seen: std::collections::BTreeMap<u32, usize> = Default::default();
        for n in self.used_tool_numbers() {
            *seen.entry(n).or_default() += 1;
        }
        for (n, k) in seen {
            if n == 0 {
                e.push("tool number T0 is not allowed".into());
            } else if k > 1 {
                e.push(format!("tool number T{n} is used {k} times"));
            }
        }
        for c in &self.cutters {
            if !(c.dia > 0.0 && c.flute > 0.0) {
                e.push(format!("T{}: diameter and flute length must be > 0", c.tool));
            }
        }
        for b in &self.engravers {
            if !(b.angle > 0.0 && b.angle < 180.0) {
                e.push(format!("T{}: included angle must be between 0 and 180°", b.tool));
            }
            if !(b.tip >= 0.0 && b.max_depth > 0.0) {
                e.push(format!("T{}: tip flat must be >= 0 and max depth > 0", b.tool));
            }
        }
        e
    }
}

/// A complete job: groups plus settings. Saved/loaded as JSON.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Job {
    pub groups: Vec<ThreadGroup>,
    pub settings: Settings,
    /// Step milling tab: STEP file + operations.
    pub step: crate::stepmill::StepJob,
}

impl Job {
    /// Make sure the thread-mill table has one entry per thread size
    /// (e.g. after loading an older/hand-edited file).
    pub fn normalize(&mut self) {
        self.settings.normalize();
        self.sync_cutters();
    }

    /// Copy the tool-table data into every group / operation that uses each
    /// tool number (so editing the table updates the job).
    pub fn sync_cutters(&mut self) {
        let s = &self.settings;
        let sync = |c: &mut Cutter| {
            if let Some(t) = s.lookup(*c) {
                *c = t;
            }
        };
        for g in &mut self.groups {
            sync(&mut g.cutter);
        }
        for op in &mut self.step.ops {
            if let Some(c) = op.cutter_mut() {
                sync(c);
            }
            if let Some(e) = op.engraver_mut() {
                if let Some(t) = s.lookup_engraver(*e) {
                    *e = t;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_cutter_names_still_load() {
        let g: ThreadGroup = serde_json::from_str(r#"{"thread":"M4","cutter":"C2x12"}"#).unwrap();
        assert_eq!(g.cutter, Cutter::C2x12);
        let c: Cutter = serde_json::from_str(r#"{"tool":7,"dia":2.5,"flute":9}"#).unwrap();
        assert_eq!((c.id, c.tool, c.dia, c.flute), (0, 7, 2.5, 9.0));
    }

    #[test]
    fn table_edits_follow_into_job_and_settings_roundtrip() {
        let mut job = Job::default();
        job.groups.push(ThreadGroup { cutter: Cutter::C2x12, ..Default::default() });
        // edit the T3 row: new number and size
        let row = job.settings.cutters.iter_mut().find(|c| c.tool == 3).unwrap();
        row.tool = 8;
        row.dia = 2.2;
        job.sync_cutters();
        assert_eq!((job.groups[0].cutter.tool, job.groups[0].cutter.dia), (8, 2.2));
        // a new row reusing the old number does not steal the group
        let id = job.settings.new_cutter_id();
        job.settings.cutters.push(Cutter { id, tool: 3, dia: 0.8, flute: 2.0 });
        job.sync_cutters();
        assert_eq!(job.groups[0].cutter.tool, 8);
        assert!(job.settings.tool_issues().is_empty());
        // duplicate tool numbers are reported
        job.settings.thread_mills[0].tool = 8;
        assert_eq!(job.settings.tool_issues().len(), 1);
        // settings survive a JSON round trip
        let j = serde_json::to_string(&job.settings).unwrap();
        let mut back: Settings = serde_json::from_str(&j).unwrap();
        back.normalize();
        assert_eq!(back, job.settings);
        // settings files without ids get unique ones
        let mut old: Settings = serde_json::from_str(r#"{"cutters":[{"tool":1,"dia":1,"flute":3},{"tool":2,"dia":2,"flute":6}]}"#).unwrap();
        old.normalize();
        assert!(old.cutters[0].id != 0 && old.cutters[0].id != old.cutters[1].id);
        assert_eq!(old.thread_mills.len(), 6);
        assert_eq!(old.thread_mills[0].tool, 11);
    }
}

#[cfg(test)]
mod engraver_tests {
    use super::*;

    #[test]
    fn groove_width_follows_taper() {
        let b = Engraver { id: 1, tool: 21, angle: 90.0, tip: 0.2, max_depth: 2.0 };
        assert!((b.width_at(0.0) - 0.2).abs() < 1e-9);
        assert!((b.width_at(0.5) - 1.2).abs() < 1e-9);
    }

    #[test]
    fn old_settings_get_default_bits_and_numbers_must_be_unique() {
        let mut s: Settings = serde_json::from_str(r#"{"safe_z": 12.0}"#).unwrap();
        s.normalize();
        assert_eq!(s.engravers, Engraver::DEFAULTS.to_vec());
        assert!(s.tool_issues().is_empty());
        s.engravers[0].tool = 1; // clashes with end mill T1
        assert!(s.tool_issues().iter().any(|e| e.contains("T1 is used 2 times")));
        s.engravers[0].tool = 21;
        s.engravers[1].angle = 180.0;
        assert!(!s.tool_issues().is_empty());
    }

    #[test]
    fn table_edits_follow_into_engrave_ops() {
        use crate::engrave::{EngraveOp, EngraveTool};
        use crate::stepmill::{OpKind, StepOp};
        let mut job = Job::default();
        job.step.ops.push(StepOp { enabled: true, kind: OpKind::Engrave(EngraveOp { tool: EngraveTool::Bit(Engraver::DEFAULTS[0]), ..Default::default() }) });
        job.settings.engravers[0].tip = 0.05;
        job.settings.engravers[0].tool = 30;
        job.sync_cutters();
        let OpKind::Engrave(g) = &job.step.ops[0].kind else { unreachable!() };
        assert_eq!(g.tool, EngraveTool::Bit(job.settings.engravers[0]));
    }
}
