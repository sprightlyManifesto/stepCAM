from cadquery import Workplane as WP
import cadquery as cq

a = WP().circle(20/2).extrude(5).faces(">Z").fillet(3)
a = a.union(WP().circle(10/2).extrude(14).faces(">Z").chamfer(3))

cq.exporters.export(a,"a.step")