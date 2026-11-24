"""Example: machine a filleted bracket with stepCAM from CadQuery.

Run with a Python that has CadQuery, from the repository root, after
`cargo build --release`:

    python python/example.py

Writes out/bracket.nc, out/bracket.stepcam.json (open it in stepCAM to look
at it or simulate it) and out/bracket.step.
"""

import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stepcam import Job  # noqa: E402

# 50 x 30 x 8 plate with a 30 x 14 pocket 4 deep, two M4 tapped holes and
# a 3 mm chamfer round the top
part = (
    cq.Workplane()
    .rect(50, 30)
    .extrude(8)
    .faces(">Z")
    .workplane()
    .rect(30, 14)
    .cutBlind(-4)
    .faces(">Z")
    .workplane()
    .pushPoints([(-20, 0), (20, 0)])
    .hole(3.3, 6)  # M4 tap drill, 6 deep (blind)
    .faces(">Z")
    .edges("not(<X or >X or <Y or >Y)")
    .edges("|X or |Y")
    .fillet(1)
)

job = Job(part, stock=(6, 6, 1), settings={"spindle_rpm": 12000})
job.rough(tool=4, layer_height=1.0, stock_to_leave=0.15, strategy="adaptive")
job.face(">Z", tool=5)
job.pocket(part.faces(">Z[-2]"), tool=3, finish=0.1)  # the pocket floor (2nd highest flat face)
job.thread_alike(part.faces("%CYLINDER").faces(">X"), descent_angle=3)  # the hole at X20 and every hole like it
job.engrave(">Z", "STEPCAM", height=3, depth=0.2, at=(0, -11), tool=22)
# profile the bottom outline free, with two tabs on each long side
job.profile(part.faces("<Z"), side="outside", tool=4, bottom_z=-0.2)
job.tabs(part.faces("<Z").edges("|X"), at=(0.25, 0.75), width=5, depth=3, height=1.5)

out = Path("out")
out.mkdir(exist_ok=True)
res = job.generate(str(out / "bracket.nc"), simulate={"detail": 300})

print(f"{len(res.ops)} operations, {res.lines} lines, about {res.run_time_text}")
for o in res.ops:
    print("  -", o["label"])
for w in res.warnings:
    print("warning:", w)
sim = res.simulation
print(f"simulated: {sim['removed_mm3']:.0f} mm³ removed, {sim['at_size_pct']:.0f} % at size, max gouge {sim['max_gouge']:.3f} mm")
for e in sim["events"][:10]:
    print("  !", e)
print("job:", res.job_path)
