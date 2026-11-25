"""stepCAM from CadQuery: build a step milling job in a CadQuery script.

Everything the Step Milling tab does is available. Faces and edges are
chosen with ordinary CadQuery selectors instead of mouse clicks, and that
includes placing mill tabs::

    import cadquery as cq
    from stepcam import Job

    part = cq.Workplane().rect(40, 30).extrude(8).faces(">Z").workplane().rect(20, 10).cutBlind(-4)

    job = Job(part, stock=(4, 4, 2))                  # 2 mm per side, 2 mm on top
    job.rough(tool=4, stock_to_leave=0.2)
    job.face(">Z")                                    # selector on the part...
    job.pocket(part.faces(">Z[1]"), tool=3)           # ...or a Workplane / Shape
    job.profile(part.faces("<Z"), side="outside")     # a face = its outline edges
    job.tabs(part.faces("<Z").edges("|Y"), width=4, height=1.5)
    job.engrave(">Z", "PART 7", height=4, at=(15, 10), tool=22)
    result = job.generate("part.nc", simulate=True)
    print(result.run_time_text, result.simulation["max_gouge"])

`generate()` exports the part to STEP next to the job file, and runs
`stepcam batch`. That builds each operation exactly as the app's
right-click menu would, then writes the G-code and a `.stepcam.json` job
that `job.open()` (or the app's Load button) opens in stepCAM.

The `stepcam` program is looked up in this order:
  1. the `binary=` argument;
  2. the `STEPCAM` environment variable;
  3. `stepcam` on the PATH;
  4. the repository's own `target/release` / `target/debug` build next to this file.

Parameters are given by name. Friendly names (`stepover`, `stock_to_leave`,
`extra`, `descent_angle`, `align`, ...) are translated. Any field name of the
operation as saved in a job file is also accepted as-is. Unknown names are
reported with the list of valid ones.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence, Union

import cadquery as cq

__all__ = ["Job", "Result", "StepCamError", "find_binary"]

# Friendly parameter names -> field names used by stepCAM (per operation).
_COMMON = {
    "stepover": "stepover_pct",
    "depth_per_pass": "depth_per_pass",
    "ramp": "ramp_angle",
}
_ALIASES = {
    "rough": {**_COMMON, "stock_to_leave": "allowance", "overcut": "overcut_z", "layer": "layer_height"},
    "face": {**_COMMON, "extra": "overlap", "extra_past_outline": "overlap"},
    "pocket": {**_COMMON, "finish_allowance": "finish", "floor_z": "z", "bottom_z": "z", "islands": "keep_islands"},
    "profile": {**_COMMON, "bottom_z": "z"},
    "engrave": {"align": "h_align", "valign": "v_align", "dx": "x", "dy": "y"},
    "thread": {"descent_angle": "ramp_angle", "thread_length": "length", "hole_start_z": "start_z"},
    "tab": {"height": "z_height", "along": "width", "across": "depth"},
}
_ALIASES["thread_alike"] = _ALIASES["thread"]

Selection = Union[str, cq.Workplane, cq.Shape, Iterable[Any]]


class StepCamError(RuntimeError):
    """stepCAM could not build the job; `errors` lists why."""

    def __init__(self, errors: Sequence[str], report: Optional[dict] = None):
        self.errors = list(errors)
        self.report = report or {}
        super().__init__("stepCAM: " + "; ".join(self.errors))


@dataclass
class Result:
    """What `Job.generate()` produced."""

    ok: bool
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    #: Operations in the job, as listed in the app: [{"label": ..., "enabled": ...}].
    ops: list = field(default_factory=list)
    lines: int = 0
    #: Estimated run time (s) and as text ("12 min 05 s").
    run_time: float = 0.0
    run_time_text: str = ""
    gcode_path: Optional[str] = None
    job_path: Optional[str] = None
    step_path: Optional[str] = None
    #: Stock box {"min": [x, y, z], "max": [x, y, z]} (model coordinates).
    stock: dict = field(default_factory=dict)
    #: Program zero in model coordinates: the G-code is relative to it.
    origin: list = field(default_factory=list)
    #: With simulate=: removed_mm3, at_size_pct, max_stock_left, max_gouge,
    #: gouged_cells, events (rapids through material, cuts deeper than the
    #: flutes) and warnings.
    simulation: Optional[dict] = None
    report: dict = field(default_factory=dict)

    @property
    def gcode(self) -> str:
        """The program text."""
        if self.gcode_path:
            return Path(self.gcode_path).read_text()
        return self.report.get("gcode_text", "")


def find_binary(explicit: Optional[str] = None) -> str:
    """Path of the stepcam program (see the module docs for the search order)."""
    exe = "stepcam.exe" if sys.platform == "win32" else "stepcam"
    here = Path(__file__).resolve().parent.parent / "target"
    for c in (explicit, os.environ.get("STEPCAM"), shutil.which("stepcam"), here / "release" / exe, here / "debug" / exe):
        if c and Path(c).is_file():
            return str(c)
    raise FileNotFoundError("stepcam program not found: build it with `cargo build --release`, put it on the PATH, set STEPCAM, or pass binary=")


def _flatten(x: Any) -> list:
    if isinstance(x, cq.Workplane):
        return list(x.vals())
    if isinstance(x, (cq.Shape, cq.Vector)) or not isinstance(x, Iterable) or isinstance(x, (str, bytes)):
        return [x]
    out = []
    for y in x:
        out.extend(_flatten(y))
    return out


def _v(p: cq.Vector) -> list:
    return [round(p.x, 9), round(p.y, 9), round(p.z, 9)]


def _face_pick(f: cq.Face, at: Optional[Sequence[float]] = None) -> dict:
    """A point well inside the face (centroid of its largest mesh triangle), or
    the face's point under `at` (X, Y) for a flat face, with the normal."""
    verts, tris = f.tessellate(0.01, 0.2)
    if not tris:
        raise ValueError("cannot mesh a selected face")

    def area(t):
        a, b, c = (verts[i] for i in t)
        return (b - a).cross(c - a).Length

    a, b, c = (verts[i] for i in max(tris, key=area))
    p = (a + b + c) * (1.0 / 3.0)
    if at is not None:
        if len(at) == 3:
            p = cq.Vector(*at)
        else:
            p = cq.Vector(at[0], at[1], p.z)
    n = f.normalAt(p)
    return {"point": _v(p), "normal": _v(n)}


def _edge_picks(e: cq.Edge, at: Union[float, Sequence[float]] = 0.5) -> list:
    ts = [at] if isinstance(at, (int, float)) else list(at)
    out = []
    for t in ts:
        if not 0.0 <= t <= 1.0:
            raise ValueError(f"edge position {t} must be between 0 and 1")
        out.append({"point": _v(e.positionAt(t)), "tangent": _v(e.tangentAt(t))})
    return out


class Job:
    """A step milling job for a CadQuery part.

    part:           the part (Workplane or Shape), in machine coordinates (mm).
                    Move / rotate it in CadQuery to place it, or use `placement`.
    stock:          (x, y, z). With stock_offset=True (default) these are added
                    to the part's bounding box (X/Y split over both sides, Z on
                    top); with False they are the absolute stock size.
    stock_z_anchor: where the part sits in the stock's height: "bottom"
                    (default, part on the stock base, extra stock on top),
                    "middle" or "top" (part top at the stock top).
    rapid_above:    rapid plane height above the stock or part top (mm).
    settings_file:  a settings JSON saved from the app (Tools & Settings tab).
    settings:       overrides, by field name, e.g. {"spindle_rpm": 12000,
                    "coolant": False, "max_accel": 300}.
    origin_xy:      program X0 Y0: "model" (default, the part's own zero),
                    "center", "bottom_left", "bottom_right", "top_left",
                    "top_right" (of the stock, seen from above), or (x, y).
    origin_z:       program Z0: "model" (default), "stock_top", "stock_base"
                    or a Z value. The G-code is written relative to the origin
                    directly (no G10 / G53 / work offsets).
    placement:      optional {"rot": 3x3, "offset": [x, y, z]} applied to the model.
    binary:         path of the stepcam program (see find_binary()).
    """

    def __init__(
        self,
        part: Union[cq.Workplane, cq.Shape],
        *,
        stock: Sequence[float] = (0.0, 0.0, 0.0),
        stock_offset: bool = True,
        stock_z_anchor: str = "bottom",
        rapid_above: float = 10.0,
        settings_file: Optional[str] = None,
        settings: Optional[dict] = None,
        origin_xy: Union[str, Sequence[float]] = "model",
        origin_z: Union[str, float] = "model",
        placement: Optional[dict] = None,
        binary: Optional[str] = None,
    ):
        self.part = part
        self.stock = {"x": float(stock[0]), "y": float(stock[1]), "z": float(stock[2]), "offset": bool(stock_offset), "z_anchor": stock_z_anchor}
        self.rapid_above = float(rapid_above)
        self.settings_file = settings_file
        self.settings = dict(settings or {})
        self.placement = placement
        self.binary = binary
        at = [0.0, 0.0, 0.0]
        if isinstance(origin_xy, str):
            xy = origin_xy
        else:
            xy, at[0], at[1] = "custom", float(origin_xy[0]), float(origin_xy[1])
        if isinstance(origin_z, str):
            z = origin_z
        else:
            z, at[2] = "custom", float(origin_z)
        self.origin = {"xy": xy, "z": z, "at": at}
        self.ops: list = []
        self.job_path: Optional[str] = None

    # ------------------------------------------------------------ selection
    def _select(self, target: Selection, want: str) -> list:
        if isinstance(target, str):
            if not isinstance(self.part, cq.Workplane):
                raise TypeError("string selectors need the part as a cq.Workplane")
            target = self.part.faces(target) if want == "face" else self.part.edges(target)
        objs = _flatten(target)
        if not objs:
            raise ValueError(f"the selection matched no {want}s")
        return objs

    def _faces(self, target: Selection) -> list:
        objs = self._select(target, "face")
        bad = [type(o).__name__ for o in objs if not isinstance(o, cq.Face)]
        if bad:
            raise TypeError(f"expected faces, got {', '.join(sorted(set(bad)))}")
        return objs

    def _edges(self, target: Selection, faces_ok: bool) -> list:
        out = []
        for o in self._select(target, "edge"):
            if isinstance(o, cq.Edge):
                out.append(o)
            elif isinstance(o, cq.Wire):
                out.extend(o.Edges())
            elif faces_ok and isinstance(o, cq.Face):
                out.extend(o.outerWire().Edges())
            else:
                raise TypeError(f"expected edges{' or faces' if faces_ok else ''}, got {type(o).__name__}")
        return out

    def _add(self, kind: str, picks: list, params: dict, on: Optional[str] = None, enabled: bool = True) -> "Job":
        aliases = _ALIASES.get(kind, {})
        p = {}
        for k, v in params.items():
            if v is None:
                continue
            p[aliases.get(k, k)] = v
        op = {"type": kind, "picks": picks, "params": p, "enabled": enabled}
        if on:
            op["on"] = on
        self.ops.append(op)
        return self

    # ----------------------------------------------------------- operations
    def rough(self, *, enabled: bool = True, **params) -> "Job":
        """Rough the whole part layer by layer (Rough on the Step Milling tab).

        tool, layer_height, stepover (%), stock_to_leave, feed, speed,
        ramp_angle, entry ("helix" / "zigzag" / "plunge"), strategy
        ("standard" / "adaptive"), start_z, bottom_z, overcut_z, clear_voids.
        """
        return self._add("rough", [], params, enabled=enabled)

    def face(self, faces: Selection = ">Z", *, enabled: bool = True, **params) -> "Job":
        """Face down to each flat, up-facing face.

        tool, stepover (%), depth_per_pass, extra (past the outline + tool
        radius), feed, speed, start_z.
        """
        return self._add("face", [_face_pick(f) for f in self._faces(faces)], params, enabled=enabled)

    def pocket(self, target: Selection, *, enabled: bool = True, **params) -> "Job":
        """Pocket down to flat or sloped up-facing faces, or inside closed,
        level loops of edges (one pocket per loop).

        tool, strategy, entry, stepover (%), finish (finish pass amount),
        depth_per_pass, ramp_angle, keep_islands, feed, speed, start_z,
        floor_z (edge pockets).
        """
        objs = self._select(target, "face")
        if all(isinstance(o, cq.Face) for o in objs):
            return self._add("pocket", [_face_pick(f) for f in objs], params, on="face", enabled=enabled)
        edges = self._edges(objs, faces_ok=False)
        return self._add("pocket", [p for e in edges for p in _edge_picks(e)], params, on="edge", enabled=enabled)

    def profile(self, edges: Selection, *, side: str = "outside", enabled: bool = True, **params) -> "Job":
        """Profile along level edge loops / chains (one profile per chain).
        A face selects its outer outline.

        side ("outside" / "inside" / "center"), tool, depth_per_pass,
        ramp_angle, feed, speed, start_z, bottom_z.
        """
        es = self._edges(edges, faces_ok=True)
        return self._add("profile", [p for e in es for p in _edge_picks(e)], {"side": side, **params}, enabled=enabled)

    def engrave(self, face: Selection, text: str, *, at: Optional[Sequence[float]] = None, enabled: bool = True, **params) -> "Job":
        """Engrave text into a flat, up-facing face.

        at: anchor point (X, Y) in machine coordinates (default: well inside
        the face); or give dx / dy from the face centre. height (capitals),
        depth, depth_per_pass, font ("Sans", "Mono" or a .ttf / .otf path),
        tool (an engraving bit or end mill number), align ("left" /
        "center" / "right"), valign ("top" / "center" / "bottom"), angle
        (degrees CCW from +X), feed, speed.
        """
        fs = self._faces(face)
        return self._add("engrave", [_face_pick(f, at) for f in fs], {"text": text, **params}, enabled=enabled)

    def thread(self, holes: Selection, *, enabled: bool = True, **params) -> "Job":
        """Bore and thread mill cylindrical holes (Add to thread milling).

        The size, start Z, length and locations come from the model. Holes of
        different size, top or depth become separate items. Override them
        with thread ("M3"), length, start_z, descent_angle and tool (the
        boring end mill).
        """
        return self._add("thread", [_face_pick(f) for f in self._faces(holes)], params, enabled=enabled)

    def thread_alike(self, hole: Selection, *, enabled: bool = True, **params) -> "Job":
        """Thread mill every hole in the model like the selected one (same Ø,
        depth and blind / through type), skipping holes already threaded."""
        return self._add("thread_alike", [_face_pick(f) for f in self._faces(hole)], params, enabled=enabled)

    def tabs(self, edges: Selection, *, at: Union[float, Sequence[float]] = 0.5, enabled: bool = True, **params) -> "Job":
        """Mill tabs on each selected edge, at fraction(s) `at` along it
        (0.5 = the middle; e.g. at=(0.25, 0.75) for two per edge).

        width (along the edge), depth (across it), height (up from the edge).
        Every cut lifts over the tabs.
        """
        es = self._edges(edges, faces_ok=False)
        return self._add("tab", [p for e in es for p in _edge_picks(e, at)], params, enabled=enabled)

    # -------------------------------------------------------------- output
    def spec(self, step_path: str, gcode: Optional[str] = None, job: Optional[str] = None, simulate: Union[bool, dict] = False) -> dict:
        """The `stepcam batch` request for this job."""
        s: dict = {"step": str(step_path), "stock": self.stock, "rapid_above": self.rapid_above, "ops": self.ops}
        if self.settings_file:
            s["settings_file"] = str(Path(self.settings_file).resolve())
        if self.settings:
            s["settings"] = self.settings
        if self.placement:
            s["placement"] = self.placement
        s["origin"] = self.origin
        if gcode:
            s["gcode_out"] = str(gcode)
        if job:
            s["job_out"] = str(job)
        if simulate:
            s["simulate"] = simulate if isinstance(simulate, dict) else {}
        return s

    def generate(
        self,
        gcode: Optional[str] = "part.nc",
        *,
        job: Optional[str] = None,
        step: Optional[str] = None,
        simulate: Union[bool, dict] = False,
        check: bool = True,
    ) -> Result:
        """Write the G-code (None: keep it in memory, see Result.gcode), the job
        file (default: next to the G-code as <name>.stepcam.json) and the
        part's STEP file (default <name>.step beside the job, which the job
        refers to).

        simulate: True or {"detail": 150..2000 cells, "tolerance": mm} runs
        the program through the simulator and fills Result.simulation.
        check: raise StepCamError when stepCAM reports errors.
        """
        base = Path(gcode or "part.nc").resolve()
        job_path = Path(job).resolve() if job else base.with_suffix(".stepcam.json")
        step_path = Path(step).resolve() if step else job_path.with_name(job_path.name.replace(".stepcam.json", "") + ".step")
        step_path.parent.mkdir(parents=True, exist_ok=True)
        cq.exporters.export(self.part, str(step_path))
        spec = self.spec(str(step_path), str(base) if gcode else None, str(job_path), simulate)
        proc = subprocess.run([find_binary(self.binary), "batch", "-"], input=json.dumps(spec), capture_output=True, text=True)
        try:
            rep = json.loads(proc.stdout)
        except json.JSONDecodeError:
            raise StepCamError([f"stepcam batch failed (exit {proc.returncode}): {proc.stderr.strip() or proc.stdout.strip()}"])
        res = Result(
            ok=bool(rep.get("ok")),
            errors=rep.get("errors", []),
            warnings=rep.get("warnings", []),
            ops=rep.get("ops", []),
            lines=rep.get("lines", 0),
            run_time=rep.get("run_time", 0.0),
            run_time_text=rep.get("run_time_text", ""),
            gcode_path=rep.get("gcode"),
            job_path=rep.get("job"),
            step_path=str(step_path),
            stock=rep.get("stock", {}),
            origin=rep.get("origin", []),
            simulation=rep.get("simulation"),
            report=rep,
        )
        if res.ok:
            self.job_path = res.job_path
        if check and not res.ok:
            raise StepCamError(res.errors, rep)
        return res

    def open(self, job: Optional[str] = None) -> subprocess.Popen:
        """Open the generated job in the stepCAM app (to look at it or simulate it)."""
        path = job or self.job_path
        if not path:
            raise RuntimeError("generate() the job first (or pass job=)")
        return subprocess.Popen([find_binary(self.binary), str(path)])
