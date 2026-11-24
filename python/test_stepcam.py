"""Tests for stepcam.py. Needs CadQuery and a built stepcam (cargo build --release):

    python -m unittest discover -s python
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import cadquery as cq
    import stepcam
    from stepcam import Job, StepCamError

    stepcam.find_binary()
    SKIP = None
except (ImportError, FileNotFoundError) as e:  # pragma: no cover
    SKIP = str(e)


def plate():
    """40 x 30 x 8 plate, 20 x 10 pocket 3 deep, an M3 tap-drill hole each side."""
    return (
        cq.Workplane()
        .rect(40, 30)
        .extrude(8)
        .faces(">Z")
        .workplane()
        .rect(20, 10)
        .cutBlind(-3)
        .faces(">Z")
        .workplane()
        .pushPoints([(-15, 0), (15, 0)])
        .hole(2.5, 5)
    )


@unittest.skipIf(SKIP, f"needs CadQuery and stepcam: {SKIP}")
class StepCamTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.out = Path(self.dir.name)

    def tearDown(self):
        self.dir.cleanup()

    def test_every_operation_from_selectors(self):
        part = plate()
        job = Job(part, stock=(4, 4, 1), settings={"coolant": False})
        job.rough(tool=4, stock_to_leave=0.1, strategy="adaptive")
        job.face(">Z", tool=5, extra=0.5)
        job.pocket(part.faces(">Z[-2]"), tool=3, finish=0.1, entry="zigzag")
        job.pocket(part.faces(">Z[-2]").edges(), tool=2)  # same pocket from its edge loop
        job.thread_alike(part.faces("%CYLINDER").faces(">X"), descent_angle=2.5)
        job.engrave(">Z", "AB", height=3, at=(0, -11), tool=22, align="left")
        job.profile(part.faces("<Z"), side="outside", tool=4, bottom_z=-0.2)
        job.tabs(part.faces("<Z").edges("|X"), at=(0.3, 0.7), width=4, height=1.5)
        res = job.generate(str(self.out / "plate.nc"), simulate={"detail": 200})

        labels = [o["label"] for o in res.ops]
        self.assertEqual(len(labels), 1 + 1 + 2 + 1 + 1 + 1 + 4, labels)
        self.assertTrue(labels[0].startswith("Rough"))
        self.assertIn("Thread M3", " ".join(labels))
        self.assertEqual(sum(l.startswith("Mill tab") for l in labels), 4)
        code = res.gcode
        for t in ("T4 M6", "T5 M6", "T3 M6", "T2 M6", "T22 M6"):
            self.assertIn(t, code)
        self.assertNotIn("M8", code)  # coolant override
        self.assertGreater(res.run_time, 10)
        # the job opens in the app: it names the exported STEP file
        j = json.loads(Path(res.job_path).read_text())
        self.assertEqual(Path(j["step"]["step_path"]), Path(res.step_path))
        self.assertTrue(Path(res.step_path).is_file())
        self.assertEqual(len(j["step"]["ops"]), len(labels))
        sim = res.simulation
        self.assertGreater(sim["removed_mm3"], 1000)
        self.assertFalse([e for e in sim["events"] if e["kind"] == "rapid_through_material"])

    def test_tab_positions_follow_the_edge(self):
        part = cq.Workplane().rect(40, 30).extrude(5)
        job = Job(part)
        job.profile("<Z", side="outside")
        job.tabs(part.faces("<Z").edges(">Y"), at=(0.25, 0.75), width=6)
        res = job.generate(str(self.out / "t.nc"))
        tabs = [o["kind"]["Tab"] for o in json.loads(Path(res.job_path).read_text())["step"]["ops"] if "Tab" in o["kind"]]
        xs = sorted(round(t["center"][0], 6) for t in tabs)
        self.assertEqual(xs, [-10.0, 10.0])
        for t in tabs:
            self.assertAlmostEqual(t["center"][1], 15.0)
            self.assertAlmostEqual(t["width"], 6.0)

    def test_errors_are_reported(self):
        part = plate()
        job = Job(part)
        job.rough(stepovr=40)
        with self.assertRaises(StepCamError) as cm:
            job.generate(str(self.out / "e.nc"))
        self.assertIn("unknown rough parameter", str(cm.exception))
        self.assertIn("stepover_pct", str(cm.exception))  # lists the valid names

        job = Job(part)
        job.face("<Z")  # faces down
        res = job.generate(str(self.out / "e.nc"), check=False)
        self.assertFalse(res.ok)
        self.assertIn("not a flat surface facing up", res.errors[0])

        with self.assertRaises(ValueError):
            Job(part).face(part.faces(">Z").edges(">X").faces())  # empty selection
        with self.assertRaises(TypeError):
            Job(part).face(part.edges(">Z"))  # edges, not faces


if __name__ == "__main__":
    unittest.main()
