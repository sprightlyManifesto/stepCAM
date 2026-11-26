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
        job.rough(tool=4, stock_to_leave=0.1, strategy="adaptive", finish_each_layer=True)
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

    def test_origin_presets_and_custom(self):
        part = cq.Workplane().rect(40, 30).extrude(5)
        job = Job(part, stock=(4, 4, 1), origin_xy="bottom_left", origin_z="stock_top")
        job.profile("<Z", side="outside")
        res = job.generate(str(self.out / "o.nc"))
        self.assertEqual(res.origin, [res.stock["min"][0], res.stock["min"][1], res.stock["max"][2]])
        self.assertEqual(res.origin, [-22.0, -17.0, 6.0])
        # part 0..5 in 7 mm of stock: anchored at the top, 2 mm below it
        job = Job(part, stock=(0, 0, 2), stock_z_anchor="top", origin_z="stock_base")
        job.profile("<Z", side="outside")
        res = job.generate(str(self.out / "a.nc"))
        self.assertEqual((res.stock["min"][2], res.stock["max"][2]), (-2.0, 5.0))
        self.assertEqual(res.origin[2], -2.0)
        job = Job(part, origin_xy=(5, -3), origin_z=2.5)
        job.profile("<Z", side="outside")
        res = job.generate(str(self.out / "c.nc"))
        self.assertEqual(res.origin, [5.0, -3.0, 2.5])
        with self.assertRaises(StepCamError):
            bad = Job(part, origin_xy="middle")
            bad.profile("<Z")
            bad.generate(str(self.out / "b.nc"))

    def test_finish_after_rough_and_the_warning_without(self):
        part = plate()
        job = Job(part, stock=(4, 4, 1))
        job.rough(tool=4)
        job.finish(tool=3, z_step=0.2)
        res = job.generate(str(self.out / "f.nc"))
        self.assertTrue(res.ops[1]["label"].startswith("Finish walls"), res.ops)
        self.assertIn("0.2 mm steps", res.ops[1]["label"])
        self.assertFalse(res.warnings, res.warnings)
        job = Job(part, stock=(4, 4, 1))
        job.finish()
        res = job.generate(str(self.out / "g.nc"))
        self.assertTrue(any("no Rough op before this Finish" in w for w in res.warnings), res.warnings)

    def test_adaptive_defaults_to_a_20_percent_stepover(self):
        part = plate()
        job = Job(part, stock=(4, 4, 1))
        job.rough(strategy="adaptive")
        job.rough(strategy="adaptive", stepover=30)
        job.rough()
        res = job.generate(str(self.out / "s.nc"))
        ops = [o["kind"]["Rough"] for o in json.loads(Path(res.job_path).read_text())["step"]["ops"]]
        self.assertEqual([o["stepover_pct"] for o in ops], [20.0, 30.0, 40.0])

    def test_omit_fills_the_pocket_and_holes(self):
        part = plate()
        job = Job(part, stock=(4, 4, 1))
        job.omit(part.faces(cq.selectors.NearestToPointSelector((0, 0, 5))))
        job.omit(part.faces("%CYLINDER"))
        job.rough()
        res = job.generate(str(self.out / "o.nc"))
        self.assertEqual([o["label"] for o in res.ops[:2]], ["Omit 1 area", "Omit 2 areas"])
        ops = json.loads(Path(res.job_path).read_text())["step"]["ops"]
        fills = [f for o in ops[:2] for f in o["kind"]["Omit"]["fills"]]
        self.assertEqual([(round(f["bottom"], 3), round(f["top"], 3)) for f in fills], [(5.0, 8.0), (3.0, 8.0), (3.0, 8.0)])

    def test_facing_patterns(self):
        part = plate()
        job = Job(part, stock=(4, 4, 2))
        job.face(pattern="square_spiral")
        job.rough(facing="zigzag")
        res = job.generate(str(self.out / "p.nc"))
        ops = json.loads(Path(res.job_path).read_text())["step"]["ops"]
        self.assertEqual(ops[0]["kind"]["Face"]["pattern"], "SquareSpiral")
        self.assertEqual(ops[1]["kind"]["Rough"]["facing"], "Zigzag")

    def test_stock_model(self):
        part = plate()
        # a casting: 1 mm over the plate all round and on top, the pocket cast in
        casting = cq.Workplane().box(42, 32, 9, centered=(True, True, False)).faces(">Z").workplane().rect(16, 6).cutBlind(-2)
        job = Job(part, stock=casting)
        job.rough(tool=4)
        res = job.generate(str(self.out / "c.nc"), simulate={"detail": 200})
        self.assertEqual(res.stock, {"min": [-21.0, -16.0, 0.0], "max": [21.0, 16.0, 9.0]})
        j = json.loads(Path(res.job_path).read_text())
        self.assertTrue(Path(j["step"]["stock_model"]["path"]).is_file())
        sim = res.simulation
        self.assertFalse([e for e in sim["events"] if e["kind"] == "rapid_through_material"])
        # the casting's volume, not the box's
        self.assertLess(sim["stock_mm3"], 42 * 32 * 9 - 16 * 6 * 2 + 50)
        self.assertLess(sim["max_gouge"], 0.05)
        # and a STEP file as the stock, moved
        job = Job(part, stock=j["step"]["stock_model"]["path"], stock_shift=(0, 0, 1))
        job.rough(tool=4)
        res = job.generate(str(self.out / "d.nc"))
        self.assertEqual(res.stock["max"][2], 10.0)

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
