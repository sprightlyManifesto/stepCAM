from cadquery import Workplane as WP
import cadquery as cq

b = WP().rect(30,20).extrude(5).edges("|X").fillet(2)
b = b.union(WP().rect(25,12).extrude(5).edges("|Z").fillet(2).translate((0,0,5)))
b = b.cut(WP().rect(20,6).extrude(5).edges("|Z").fillet(2).translate((0,0,5)))
b = b.faces(">Z[-2]").edges("|X").fillet(1)

cq.exporters.export(b,"b.step")