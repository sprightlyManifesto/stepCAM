#!/usr/bin/env python3
"""Generate a small B-rep STEP test part: block with rectangular pocket,
blind round hole, and a free 2-edge wire. Usage: make_step.py OUT.step"""
import sys

ents = []
def E(s):
    ents.append(s)
    return len(ents)

def pt(x, y, z): return E(f"CARTESIAN_POINT('',({x:.6f},{y:.6f},{z:.6f}))")
def dr(x, y, z): return E(f"DIRECTION('',({x:.6f},{y:.6f},{z:.6f}))")
def ax(o, z, x): return E(f"AXIS2_PLACEMENT_3D('',#{pt(*o)},#{dr(*z)},#{dr(*x)})")

verts = {}
def V(p):
    if p not in verts:
        verts[p] = E(f"VERTEX_POINT('',#{pt(*p)})")
    return verts[p]

edges = {}
def line_edge(a, b):
    key = (a, b)
    if key in edges: return edges[key], True
    if (b, a) in edges: return edges[(b, a)], False
    d = [b[i] - a[i] for i in range(3)]
    L = sum(c * c for c in d) ** 0.5
    vec = E(f"VECTOR('',#{dr(*[c / L for c in d])},{L:.6f})")
    ln = E(f"LINE('',#{pt(*a)},#{vec})")
    e = E(f"EDGE_CURVE('',#{V(a)},#{V(b)},#{ln},.T.)")
    edges[key] = e
    return e, True

def loop_of(pts):
    oes = []
    for i in range(len(pts)):
        e, fwd = line_edge(pts[i], pts[(i + 1) % len(pts)])
        oes.append(E(f"ORIENTED_EDGE('',*,*,#{e},{'.T.' if fwd else '.F.'})"))
    return E("EDGE_LOOP('',(" + ",".join(f"#{o}" for o in oes) + "))")

def oe(e, fwd=True): return E(f"ORIENTED_EDGE('',*,*,#{e},{'.T.' if fwd else '.F.'})")
def loop_e(oes): return E("EDGE_LOOP('',(" + ",".join(f"#{o}" for o in oes) + "))")

faces = []
def plane_face(origin, normal, xdir, loops):  # loops: [(loop_id, outer)]
    pl = E(f"PLANE('',#{ax(origin, normal, xdir)})")
    bounds = [E(f"{'FACE_OUTER_BOUND' if outer else 'FACE_BOUND'}('',#{l},.T.)") for l, outer in loops]
    faces.append(E("ADVANCED_FACE('',(" + ",".join(f"#{b}" for b in bounds) + f"),#{pl},.T.)"))

X0, X1, Y0, Y1, ZT, ZB = 0.0, 60.0, 0.0, 40.0, 0.0, -10.0
PX0, PX1, PY0, PY1, PZ = 15.0, 45.0, 10.0, 30.0, -4.0
HC, HR, HZ = (52.0, 32.0), 4.0, -3.0

def rect(x0, y0, x1, y1, z, ccw=True):
    p = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    return p if ccw else p[::-1]

# hole edges (circles) and seam
top_c = (HC[0] + HR, HC[1], ZT)
bot_c = (HC[0] + HR, HC[1], HZ)
circ_top = E(f"CIRCLE('',#{ax((HC[0], HC[1], ZT), (0, 0, 1), (1, 0, 0))},{HR:.6f})")
circ_bot = E(f"CIRCLE('',#{ax((HC[0], HC[1], HZ), (0, 0, 1), (1, 0, 0))},{HR:.6f})")
e_ct = E(f"EDGE_CURVE('',#{V(top_c)},#{V(top_c)},#{circ_top},.T.)")
e_cb = E(f"EDGE_CURVE('',#{V(bot_c)},#{V(bot_c)},#{circ_bot},.T.)")
seam, _ = line_edge(top_c, bot_c)

# top face: outer box (ccw), pocket rim (cw), hole rim (cw)
plane_face((0, 0, ZT), (0, 0, 1), (1, 0, 0), [
    (loop_of(rect(X0, Y0, X1, Y1, ZT)), True),
    (loop_of(rect(PX0, PY0, PX1, PY1, ZT, ccw=False)), False),
    (loop_e([oe(e_ct, False)]), False),
])
# bottom
plane_face((0, 0, ZB), (0, 0, -1), (1, 0, 0), [(loop_of(rect(X0, Y0, X1, Y1, ZB, ccw=False)), True)])
# box sides
for (a, b, n) in [((X0, Y0), (X1, Y0), (0, -1, 0)), ((X1, Y0), (X1, Y1), (1, 0, 0)),
                  ((X1, Y1), (X0, Y1), (0, 1, 0)), ((X0, Y1), (X0, Y0), (-1, 0, 0))]:
    pts = [(a[0], a[1], ZB), (b[0], b[1], ZB), (b[0], b[1], ZT), (a[0], a[1], ZT)]
    d = (b[0] - a[0], b[1] - a[1], 0)
    L = (d[0] ** 2 + d[1] ** 2) ** 0.5
    plane_face(pts[0], n, (d[0] / L, d[1] / L, 0), [(loop_of(pts), True)])
# pocket floor
plane_face((0, 0, PZ), (0, 0, 1), (1, 0, 0), [(loop_of(rect(PX0, PY0, PX1, PY1, PZ)), True)])
# pocket walls (normals point into the pocket)
for (a, b, n) in [((PX0, PY0), (PX1, PY0), (0, 1, 0)), ((PX1, PY0), (PX1, PY1), (-1, 0, 0)),
                  ((PX1, PY1), (PX0, PY1), (0, -1, 0)), ((PX0, PY1), (PX0, PY0), (1, 0, 0))]:
    pts = [(a[0], a[1], ZT), (b[0], b[1], ZT), (b[0], b[1], PZ), (a[0], a[1], PZ)]
    d = (b[0] - a[0], b[1] - a[1], 0)
    L = (d[0] ** 2 + d[1] ** 2) ** 0.5
    plane_face(pts[0], n, (d[0] / L, d[1] / L, 0), [(loop_of(pts), True)])
# hole wall (cylinder) and floor
cyl = E(f"CYLINDRICAL_SURFACE('',#{ax((HC[0], HC[1], HZ), (0, 0, 1), (1, 0, 0))},{HR:.6f})")
lp = loop_e([oe(e_ct), oe(seam), oe(e_cb, False), oe(seam, False)])
faces.append(E(f"ADVANCED_FACE('',(#{E(f'FACE_OUTER_BOUND({chr(39)}{chr(39)},#{lp},.T.)')}),#{cyl},.F.)"))
plane_face((HC[0], HC[1], HZ), (0, 0, 1), (1, 0, 0), [(loop_e([oe(e_cb)]), True)])

# free wire: two touching line edges (an L) beside the part
line_edge((-10.0, 0.0, 0.0), (-10.0, 20.0, 0.0))
line_edge((-10.0, 20.0, 0.0), (-20.0, 20.0, 0.0))

shell = E("CLOSED_SHELL('',(" + ",".join(f"#{f}" for f in faces) + "))")
E(f"MANIFOLD_SOLID_BREP('block',#{shell})")
E("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")

out = ["ISO-10303-21;", "HEADER;",
       "FILE_DESCRIPTION(('stepCAM test part'),'2;1');",
       "FILE_NAME('pocket_block.step','2026-11-08T00:00:00',(''),(''),'make_step.py','','');",
       "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));", "ENDSEC;", "DATA;"]
out += [f"#{i + 1}={s};" for i, s in enumerate(ents)]
out += ["ENDSEC;", "END-ISO-10303-21;"]
open(sys.argv[1], "w").write("\n".join(out) + "\n")
print(f"wrote {len(ents)} entities, {len(faces)} faces")
