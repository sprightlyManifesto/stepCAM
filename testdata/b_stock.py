"""Waterjet blank for the part in b.py: its outline seen from above, grown
by ALLOWANCE, with the through holes cut in (shrunk by ALLOWANCE) where
they are big enough for the jet, cut from PLATE mm plate with the part
sitting on its base. Writes b_stock.step beside this file, in b.step's
coordinates, ready to pick as the STEP model stock.

    ~/cq-editor/bin/python testdata/b_stock.py
"""

from pathlib import Path

import cadquery as cq

ALLOWANCE = 1.0  # left on the waterjet edges for machining (mm per side)
PLATE = 11.0  # plate thickness (the part is 10 high: 1 mm to face off)
MIN_HOLE = 4.0  # smallest hole the jet cuts (after the allowance), mm across
STEP = 0.25  # Z spacing of the sections that make the outline

here = Path(__file__).resolve().parent
ns: dict = {}
exec((here / "b.py").read_text(), ns)
part: cq.Workplane = ns["b"]

bb = part.val().BoundingBox()
z0, z1 = bb.zmin, bb.zmax

# the part seen from above: the union of its sections at every height
shadow = None
n = int((z1 - z0) / STEP)
for k in range(n + 1):
    z = min(z0 + 1e-3 + k * STEP, z1 - 1e-3)
    cut = part.val().intersect(cq.Solid.makeBox(bb.xlen + 10, bb.ylen + 10, 1e-3, cq.Vector(bb.xmin - 5, bb.ymin - 5, z)))
    for f in cut.Faces():
        if abs(f.normalAt().z) > 0.99 and abs(f.Center().z - z) < 1e-6:
            flat = f.translate(cq.Vector(0, 0, -z))
            shadow = flat if shadow is None else shadow.fuse(flat).clean()

faces = shadow.Faces()
blank = None
for f in faces:
    outer = cq.Workplane().add(f.outerWire()).toPending().offset2D(ALLOWANCE, kind="arc").extrude(PLATE)
    for w in f.innerWires():
        # shrink the hole (which sign does that depends on the wire's direction)
        area = cq.Face.makeFromWires(w).Area()
        small = []
        for d in (ALLOWANCE, -ALLOWANCE):
            try:
                ws = cq.Workplane().add(w).toPending().offset2D(d, kind="arc").vals()
            except Exception:
                continue
            if ws and sum(cq.Face.makeFromWires(x).Area() for x in ws) < area:
                small = ws
                break
        for hw in small:  # (none: too small to shrink, left solid)
            hf = cq.Face.makeFromWires(hw)
            hb = hf.BoundingBox()
            if min(hb.xlen, hb.ylen) >= MIN_HOLE:
                outer = outer.cut(cq.Workplane().add(hf).wires().toPending().extrude(PLATE))
    blank = outer if blank is None else blank.union(outer)

blank = blank.translate((0, 0, z0))
out = here / "b_stock.step"
cq.exporters.export(blank, str(out))
sb = blank.val().BoundingBox()
print(f"wrote {out}: {sb.xlen:.2f} x {sb.ylen:.2f} x {sb.zlen:.2f}, Z{sb.zmin:g} to Z{sb.zmax:g}, "
      f"{blank.val().Volume():.0f} mm3 (part {part.val().Volume():.0f} mm3, "
      f"box {(bb.xlen + 2 * ALLOWANCE) * (bb.ylen + 2 * ALLOWANCE) * PLATE:.0f} mm3)")
