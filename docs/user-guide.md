# stepCAM User Guide

stepCAM turns a part into G-code for a 3-axis CNC mill. It imports a STEP model and machines it from above with facing, pocketing, profiling, roughing, engraving and thread milling (helical boring and thread milling of M2 to M6 holes).

The **Simulator** tab plays the program against a block of stock, so you can check it before it goes near the machine.

All units are millimetres. Programs are climb milled throughout, for a spindle turning clockwise (M3).

## Getting started

A typical step milling session:

1. On **Tools & Settings**, check that the tool table matches the tools in your machine. The tool numbers must match the machine's tool table.
2. On **Step Milling**, click **📂 Import STEP…** and open your part.
3. Place the part. Right-click its bottom face and choose **Lay flat on this surface**, or use **Set Z…** to put a surface at a chosen height.
4. Set the **Stock** size, for example 2 mm extra on every side.
5. Add operations. **➕ Rough** is usually first, and **➕ Finish** after it cleans up the walls. Then right-click surfaces and edges to add facing, pockets, profiles, engraving and thread milling.
6. Open **Simulator**, press ⏭ and check the result. Look for red gouges and for warnings.
7. Type a file name next to **G-code file:** and click **💾 Save G-code**.
8. Click **Save** next to **Job file:** to keep the whole job for later.

The G-code updates by itself whenever you change something, so there is no Generate button.

## The window

- **Tabs** along the top: **Step Milling**, **Simulator** and **Tools & Settings**. There is also a **❓ Help** button, which opens this guide (F1 does the same).
- **File bar** under the tabs:
  - **Job file:** with **Load** and **Save**.
  - **G-code file:** with **💾 Save G-code**.
  - The G-code line count and the **Estimated run time**.
  - Errors that stop the program being generated, and status messages.
- **G-code panel** on the right: the step milling program.
  - **📋 Copy** puts the whole program on the clipboard.
  - Click a line, or move through the lines with the arrow keys, to highlight its move in the 3D view.
- **3D view:** see the next section.

### Moving around the 3D view

- **Left-drag:** orbit.
- **Right-drag or middle-drag:** pan.
- **Scroll:** zoom in or out at the cursor.
- **Double-click:** fit everything in view.
- **Iso / Top / Front / Side / Fit** buttons: standard views.
- The small X / Y / Z arrows in the corner show which way the axes point.

### Toolpath colours

- **Red, faint:** rapid moves (G0).
- **Blue:** milling, including boring holes for threads.
- **Orange:** threading.
- **Cyan:** the move of the G-code line you clicked. A hollow dot marks where it starts, a filled dot where it ends.

The checkboxes above the view switch each kind on or off. **Esc** clears the selection and the highlight.

## Jobs and files

- **Job file** (`.stepcam.json`): the whole job, which includes:
  - step milling operations;
  - stock;
  - model placement;
  - machining settings and tool tables;
  - the path of the STEP file.

  **Load** opens the job and re-imports the STEP model from that path. Keep the STEP file where it was, or re-import it. Thread groups in jobs saved with the old Thread Milling tab become thread milling items in the operations list, with their typed locations.
- **G-code file:** the step milling program. Saving is refused while the program has errors.
- **Settings file:** **💾 Save settings… / 📂 Load settings…** on the Tools & Settings tab store only the machining settings and tool tables. Use this to share one tool table between jobs.

Typed paths are relative to the folder stepCAM was started from.

## Step Milling tab

### Importing a model

**📂 Import STEP…** reads `.step` / `.stp` files:

- **Surfaces:** flat, cylindrical, conical, spherical, toroidal, B-spline and extruded surfaces are shaded. Other surface types show as edges only.
- **Wires:** loose wire edges in the file are shown in purple and can be used for profiles.
- **Units:** inch files are converted to millimetres.

The status bar shows how many faces and edges were read.

### Selecting

- **Click** a surface or an edge to highlight it. Clicking an edge selects the whole loop or chain it belongs to.
- **Ctrl+click** (Cmd on macOS) cylindrical holes to select several. Ctrl+click a selected hole again to drop it.
- **Right-click** opens the menu for whatever is under the cursor. The menu shows what the item is, for example *Flat surface at Z5* or *Cylinder Ø6 (hole)*, and the operations that suit it.
- **Esc** clears the selection.

### Placing the model

Right-click a flat surface:

- **Set Z…** (surfaces facing straight up or down only): type a Z and press **Apply** or Enter. The whole model moves up or down so that the surface sits at that Z.
- **Lay flat on this surface:** turns the model so this surface faces down, as if the part were resting on the table on it. The part keeps its XY centre and its new top ends at Z0.
- **Reset placement:** back to the coordinates in the STEP file.

The placement is saved in the job, and the operations follow the model:

- Depths, boundaries and Start Z move with their surfaces.
- Tabs move and turn with the part.
- An operation whose surface no longer faces up is switched off and listed in the status bar.

### Stock

Set the raw material with the **Stock X / Y / Z** inputs. It is drawn as an orange wire box.

- **Offset** ticked (the default): the values are added to the part's bounding box. X and Y are split over both sides and Z goes on top. All zeros means the stock is exactly the part's box.
- **Offset** unticked: the values are the absolute stock size, centred on the part in XY.
- **Part in stock:** where the part sits in the stock's height.
  - **Bottom** (the default): the part sits on the stock base, and the extra stock is on top, to be faced off.
  - **Middle:** the extra is split evenly above and below.
  - **Top:** the part's top is at the stock top, and the extra is underneath.

  The part stays where it is in model coordinates and the stock box moves round it. With the G-code origin at the stock top or base, program zero moves with it.

Switching Offset converts the numbers so the stock itself doesn't change. The resulting size and stock top are shown, with a warning if the stock is smaller than the part.

New operations that cut from the top start at the stock top, and follow it when you change the stock.

### Rapid plane

**Rapid plane height above stock or part** (default 10 mm) is added to the highest point of the stock or the part, whichever is higher. The resulting Z is shown next to the input. All rapids between cuts and all tool changes happen there.

### G-code origin

The **G-code origin** row sets program zero, the point that is X0 Y0 Z0 in the step milling G-code. Every X, Y and Z in the program is written relative to it. The offset is applied to the numbers directly: no G10, G53 or work-offset codes are used, so set your machine's work zero on the same point of the stock.

- **XY:** **Model 0** (the model's own zero, the default) or a point of the stock seen from above: **Stock centre**, **Stock bottom left**, **Stock bottom right**, **Stock top left** or **Stock top right**. Bottom is −Y and left is −X.
- **Z:** **Model 0** (the default), **Stock top** or **Stock base**.
- **X / Y / Z:** where the origin is, in model coordinates. The stock presets follow the stock when it changes. Typing a value makes that axis **Custom**.

You can also set it from the model by right-clicking:

| Right-click | Menu entry | Sets |
|---|---|---|
| A flat wall facing ±X | **Set origin X** | X at the wall |
| A flat wall facing ±Y | **Set origin Y** | Y at the wall |
| A flat surface facing up or down | **Set origin Z** | Z at the surface |
| A straight edge along X | **Set origin Y** | Y at the edge |
| A straight edge along Y | **Set origin X** | X at the edge |
| A vertical edge | **Set origin X Y = this corner** | X and Y at the corner |
| A vertical hole or boss | **Set origin X Y = centre** | X and Y at its axis |

Setting one axis from the model leaves the other where it was. For example, pick the front wall for Y and the left wall for X.

The origin is drawn in the 3D view as a small X / Y / Z triad with a ringed dot. Right-click the dot for a menu of the presets. The 3D views and the Simulator always show the part, the stock and the toolpaths in model coordinates, so changing the origin moves only the marker and the numbers in the G-code.

### The operations list

Operations run from the top of the list down.

- **Checkbox:** include the operation in the program, or leave it out.
- **Click** an operation to open its settings and highlight it in the 3D view.
- **⬆ / ⬇:** reorder. **🗑:** remove.

Every operation has a **Tool** (from the tool table), **Feed** (mm/min) and **Speed** (rpm). Cuts are climb milled, and the order within each operation keeps non-cutting moves short.

## Step milling operations

### Rough

**➕ Rough** at the top of the operations list adds roughing. It clears the material layer by layer, from **Start Z** (the stock top) down to **Bottom Z** (the part bottom).

- **Where it cuts:** each layer clears everything down to that height except the part. This covers pockets, holes, open steps such as the area above a flange, and a band round the part just wide enough for the cutter to run round its outside walls. The band is the part's outline seen from above, offset outwards by half the tool diameter. Stock further out, such as the corners of a square block round a round part, is left alone.
- **Walls:** the part's walls are profiled at every layer.
- **Order:** each area is cut all the way down before the next: the band round the outside through all its layers, then each pocket, step or hole in turn. A layer is cut only once the cuts above it are done. Going down an area, the tool stays on the floor it has just cleared: it feeds across to the next layer's start, or goes straight down where it stands, instead of lifting out. Moves to another area travel 0.5 mm above the deepest layer cleared all along the way, and go up to the retract height only when no layer is clear.
- **Stock facing:** if the stock is taller than the part, roughing first faces it down to the part top (plus stock to leave) over the same band. The row shows the Z range, or *none*. **Facing** picks its pattern, as for the Face op (see Face below). Except with Spiral, a lap round the band's edge follows the facing.
- **Slopes and curves:** sloped and curved surfaces (fillets, chamfers, domes) are followed. They are roughed down in steps, and the outline steps in and out with them.
- **Layer height:** the spacing of the layers. Extra layers are added at every flat floor and at the top and foot of every sloped or curved surface, so each one is roughed to its depth.
- **Stepover:** % of the cutter Ø between passes.
- **Stock to leave:** left on all walls and floors for finishing operations. The default is 0.
- **Clear voids** (default on): pockets, steps and counterbores are always cleared. When unticked, holes that go right through the part are only profiled round their wall, so the core can drop out.
- **Overcut Z:** carries the last layer this far below Bottom Z, for example to cut the part free.
- **Strategy:** **Standard** (concentric offset passes) or **Adaptive clearing** (trochoidal loops). Choosing adaptive sets the stepover (its advance per loop) to 20 % if it was still at the standard 40 %. Switching back restores 40 %.
  - With adaptive clearing, the band round the outside profile is widened by the loops' diameter (0.8 × tool Ø), so the outside profile is cut with trochoidal loops too instead of a full-width slot. Facing covers the same wider band and finishes with a lap round its edge.
  - **Narrow bands**, such as the one round the outside profile (an area with one island whose loops would be no wider than 1.6 × the standard 0.4 × tool Ø): one pass per layer down the middle of the band, each loop as wide as the band there. This replaces a ring of loops along each side. The advance is scaled so the cut on the longer outer side stays at the stepover.
  - **Wider areas:** rings of loops spaced 1.4 × tool Ø apart from the walls, because one loop as wide as the area would mostly cut air.
  - **Leftovers:** whatever the loops leave (along the middle of wide areas, square corners a band's loop can't reach) is cleared first with standard passes at 40 % stepover.
- **Mixed** strategy: only the very outermost loop, round the band's outer edge against the uncut stock, is cut with adaptive clearing's trochoidal loops. Everything else is cut with standard offset passes at the stepover (default 40 %), first, inside out: the rest of the band round the outline, and the pockets, steps and cleared through holes inside the part. The outer loop's trochoids advance at half the stepover (20 %). The band is widened for the loops as with adaptive clearing. This avoids slotting the outer edge full width at a fraction of adaptive clearing's time.
- **Inside only** (default off): no band round the outside profile, so the part's outside walls are not cut. Use it when the stock is already the part's size, or when the outside is cut by another operation. Everything inside the part's outline seen from above is still roughed: the stock above the part (faced over the outline only), pockets and holes, and fillets and chamfers on the outside edges. Along a fillet or chamfer the cutter runs up to one tool radius past the outline, so a band narrower than the tool is still cleared to the edge. Next to a vertical outside wall nothing is cut. Strips inside the outline narrower than 0.2 mm are left.
- **Finish each layer** (default on): after clearing each layer, a climb-milled pass round the part's walls at tool radius + stock to leave. This cleans up the scallops that adaptive clearing leaves. The outer limit of the band is not a wall and is left out. Untick it to clear only.
- **Entry:** **Helix**, **Zigzag** or **Plunge**. **Ramp angle** is the helix or zigzag angle.

Roughing sees the part from above, so undercuts can't be reached. Hops between cuts stay low (0.5 mm above the layer above) when they only pass over ground already cleared. Otherwise they go up to the retract height.

### Finish

**➕ Finish**, next to **➕ Rough**, adds wall finishing. It goes after the roughing ops in the list. At every layer, from the part top (**Start Z**) down to **Bottom Z**, it runs one climb-milled lap round each of the part's walls at the tool radius. Outside walls are cut clockwise, and pocket and hole walls anticlockwise. Nothing else is cut, so roughing has to clear the stock first.

- **Z step:** the height between laps, 0.1 mm by default. Extra laps are added at every flat floor and at the top and foot of every slope, as for Rough.
- **Stock to leave:** left on the walls. The default 0 cuts them to size.
- **Tool, Feed, Speed.**
- **Warning:** a Finish with no enabled Rough above it in the list shows "⚠ no Rough op before this Finish". It would meet the full stock everywhere except the walls.

Slopes and curves are finished in Z steps, so they are left with cusps up to the step height. Curved walls keep the 0.06 mm margin that roughing also keeps from them to cover the model mesh. Each wall is finished all the way down before the next, and its laps on consecutive layers are joined by a short feed. Moves to another wall go up to the retract height, then come back down to one roughing layer above the lap (or to the Finish's Start Z without a Rough before it).

### Face

Right-click a flat surface that faces up and choose **Face**. Facing mills everything above that surface flat, down to it, in **Depth per pass** steps. It only cuts inside the part's outside outline offset outwards by half the tool diameter, and not out to the whole stock.

- **Pattern:** how the area is covered:
  - **Unidirectional** (default): rows all cut in +X (climb), stepping towards −Y, lifting and coming back between rows.
  - **Zigzag:** rows alternately +X and −X, stepping over at depth without lifting. Quicker, but every other row is conventional milling.
  - **Spiral:** loops following the area's edge, from the outside in, stepping in at depth (climb). Anything the loops leave along the middle of narrow parts is cleared at the end with loops of its own.
  - **Square spiral:** rectangles shrinking from the area's bounding box, outside in (climb), ending with a line down the middle. Where the area isn't rectangular, the rectangles are cut only where they fall inside it.
- **Extra past outline:** goes this much further out (default 0).
- **Stepover:** % of the cutter Ø between rows or loops. The spirals use at most 70 %, so the inner corners of their loops are still covered.
- **Start Z:** the top of the material, which follows the stock top.

Where the outline splits a row or loop, the tool only stays low between the pieces if it passes over ground already faced.

### Pocket

Right-click a flat surface that faces up and choose **Pocket**. The pocket clears the area inside the surface's outer boundary, down to the surface.

- **Islands:** inner loops, such as bosses in the pocket, are ignored by default and the whole area is cleared. Tick **keep N inner loop(s)** to leave them standing.
- **Strategy / Entry / Ramp angle / Stepover:** as for Rough.
- **Finish pass:** the amount left on the walls by the clearing passes. A climb-milled finishing pass then cuts the walls to size at each depth.
- **Depth per pass**, and **Start Z** (the top of the material).

You can also make a pocket from a closed, level loop of edges (right-click an edge, then **Pocket**). Its **Pocket floor Z** can then be edited.

**Sloped surfaces:** a flat surface that is tilted but still faces upwards can be pocketed too. Pocketing goes down in depth-per-pass levels. At each level it clears where the surface is below that level and cuts the walls to size, leaving the slope as steps. The row shows the Z range and the number of levels. Vertical surfaces, surfaces facing down and curved surfaces have no surface operations, but Rough follows them.

### Profile

Right-click an edge (which selects its loop or chain) and choose **Profile** to cut along it. **Tool position** sets where the tool runs:

- **on centre line:** the tool centre follows the edges. This works on any chain, open or closed.
- **inside / outside:** the tool is offset by its radius, so the edge of the cut lies on the edges. Closed loops only. Outside runs clockwise and inside runs anticlockwise, both climb.

Each depth level ramps down along the path at the **Ramp angle**, then makes a full pass at depth. Open chains ramp back and forth.

The default depths come from the model:

- an edge on a rim (outline, pocket or hole top) cuts down to the bottom of the walls next to it;
- an edge on a floor cuts from the top of its walls down to it;
- a loose wire cuts 1 mm deep.

Change **Start Z** / **Bottom Z** if you need different depths.

### Engrave text

Right-click a flat surface that faces up and choose **Engrave text**. The tool centre follows the outlines of the letters.

- **Text:** one line of text per line.
- **Height:** the height of the capital letters.
- **Depth** below the surface, and **Depth per pass**.
- **Font:** built-in **Sans** or **Mono**, or **Browse…** for any `.ttf` / `.otf` file.
- **Tool:** an engraving bit or an end mill. **Line width** shows how wide the groove is at the surface. A tapered bit cuts wider the deeper it goes.
- **Horizontal align / Vertical align:** where the text sits relative to the anchor point. **Top** puts the top of the capitals on the anchor, and **Bottom** puts the baseline of the last line on it.
- **X / Y:** the anchor point, measured from the centre of the surface. It defaults to where you right-clicked.
- **Angle:** the text direction, anticlockwise from +X, turned about the anchor. It defaults to reading left to right in the current view, rounded to the nearest 90°. Turn the view before right-clicking to get the text the right way up.

Plunges run at half the feed.

### Thread milling from the model

Right-click a cylindrical hole and choose **Add to thread milling**. Ctrl+click several holes first to include them all. This adds a thread milling item to the operations list, filled in from the model:

- **Locations:** the hole centres. They are re-taken when the model moves.
- **Hole start Z:** the top of the hole.
- **Thread:** the closest match to the hole Ø, within 0.3 mm. A tap-drill match wins over a major-Ø match, so a Ø5 hole becomes M6.
- **Thread length:**
  - blind holes: the modelled depth minus one pitch, so the bored depth matches the model;
  - through holes: the full depth.

Holes with a different size, top or depth go into separate items.

An item's fields:

- **Thread:** M2, M2.5, M3, M4, M5 or M6.
- **Hole start Z:** the top of the material at the holes. All hole geometry is below it.
- **Thread length:** how deep the thread goes, from 0 to 100 mm.
- **Descent angle:** the helix angle while boring.
- **Milling cutter:** the end mill that bores the hole. Only cutters smaller than the tap-drill hole can be chosen.
- **Locations:** one per line, as `x, y` or `X10 Y20`.

How the holes are cut:

- **Hole depth:** the thread length plus one pitch.
- **Boring:** the hole is bored to the tap-drill Ø with a helix, in rings set by **Boring ring stepover** on Tools & Settings.
- **Threading:** the thread mill goes to the bottom of the thread and climbs out with G3, so the thread stops one pitch above the bottom of the hole.
- **Order:** an item's holes are all bored first, then all threaded. Boring visits the holes nearest-next, and threading runs the same route backwards, starting where boring ended.
- **Speeds and feeds:** the thread milling spindle speed, boring feed and threading feed are set on Tools & Settings.

**Thread mill all alike holes** (same menu) does this for every hole in the model with the same Ø, depth and blind or through type. Holes that already have a thread milling item are skipped.

Only full, vertical holes qualify. Bosses, slot ends, corner fillets and bores of other sizes don't.

**Other operations leave thread-milled holes alone**, wherever the items are in the list. The thread's volume (its major Ø, from the hole top down to the hole bottom) counts as solid:

- Rough treats it as part of the part.
- Pockets treat it as an island.
- Any other cut lifts over it.

Untick the thread item and the hole is cut by the other operations again. Mill tabs don't apply to thread milling.

### Mill tabs

Right-click an edge and choose **Add mill tab**. This adds a rectangular tab standing up from the edge at the clicked point:

- **Along the edge:** the width, along the edge's direction.
- **Across the edge:** the depth.
- **Z height:** how far the tab stands up from the edge.

Every cut lifts over the tab, allowing for the cutter radius, so the part stays attached to the stock. Tabs are drawn as green boxes.

### Omit area

Right-click any face of a pocket, hole or step (its floor, a wall, or a hole's wall) and choose **Omit area**. Every operation after the Omit op in the list then treats that area as filled in and leaves it uncut. Use it for features that are made another way, or later.

- **What is filled:** the cavity round the clicked face, made of the faces reached from it without going below its lowest point or through a flat surface above it. It is filled up to the lowest of those flat surfaces round its top (the pocket's rim), and everything below, such as a hole in a counterbore's floor, goes with it. Clicking a counterbore's floor fills the counterbore; clicking the hole below it fills only the hole, up to the counterbore floor.
- **The Omit op:** the first area makes an **Omit** op at the top of the list, and later ones are added to it. Each area is listed with its Z range and a 🗑 to stop omitting it, or right-click it again and choose **Stop omitting this area**. Move the op down the list to let the ops above it cut the area.
- **In the 3D view** the omitted faces are tinted red-brown.
- **How ops treat it:** roughing and finishing see solid material up to the fill's top; a pocket inside the area is skipped, and an area inside a pocket becomes an island; threaded holes inside it are left out. Any other cut that would reach into the area goes up over it at its top.
- A face with no higher flat surface round it, such as the part's top, cannot be omitted.

## Simulator tab

The Simulator plays a program against the stock and shows the material being cut away.

### What to simulate

- **Simulate:** **Step milling** (the current program) or a G-code file opened with **📂 Open G-code…**. Tools in a file are looked up in the tool table by number.
- **Stock:** the step milling stock, or a box round the cutting moves for a file. Tick **Custom stock** to type the box in yourself.
- **Detail:** how fine the simulated stock is: Low, Medium, High or Fine, with 150 to 800 cells along its longer side. Higher detail shows small features better but redraws more slowly.
- **⟲ Restart:** starts again with fresh stock.

The simulation updates when the program, stock or tools change, and carries on from the same time.

### Playback

| Button | Key | What it does |
|---|---|---|
| ⏮ | Home | Back to the start |
| ⏪ | ← | Previous move |
| ▶ / ⏸ | Space | Play / pause |
| ⏩ | → | Next move |
| ⏭ | End | Run the whole program |

- **Speed:** from 0.1× to 2000× machine time.
- **Time slider:** drag it to scrub through the program.

Times come from the run-time estimate, so the clock matches the estimated run time. Long programs can take a moment to catch up after a jump (*simulating…* appears in the Machine panel).

### What you see

- **Stock:** tan for uncut stock, bright for cut surfaces, dark where the tool has cut right through.
- **Tool:** gold flutes and a grey shank, drawn at its current position.
- **Path:** **All** shows the path done so far in colour and the rest faintly, **Done** shows only what has been cut, and **Off** hides it. **Rapids**, **Milling** and **Threading** switch each kind of move on or off, in the same colours as the Step Milling view (threading is any feed move with a thread mill).
- **G-code panel:** highlights the running line and scrolls with it while **Follow** is ticked. Click a line to run the simulation to the end of that line's move.
- **Machine panel:** time, progress, line number, tool, rapid or feed, X Y Z, and material removed.

### Compare with model

For step milling the cut stock is compared with the model's top surface:

- **green:** at size, within ± the tolerance (default 0.02 mm);
- **yellow to orange:** stock left, from a little to 2 mm or more;
- **red:** a gouge, where the tool cut into the part.

The panel shows how much of the part's top area is at size, the most stock left, and any gouge depth. Untick **Colour by distance to the model** to see plain stock colours.

### Warnings

Problems are listed as the simulation reaches them. Press ⏭ to find them all, and click a warning to jump to it.

- **Rapid through material:** a G0 move cut stock. On a real machine that's a crash.
- **Deeper than the flutes:** the tool met material higher up than its flute length, either under the tool or in the wall beside it. The shank or holder would rub.
- **Unknown tool:** a tool number that isn't in the tool table. It is simulated as a Ø3 end mill.
- **G20 / G91:** inch or incremental programs aren't simulated correctly.

### Limits

- The stock is a height map, so undercuts can't be shown. A thread-milled hole shows as a plain bore at the thread's major Ø.
- Engraving bits are drawn with a 1/8" shank.
- Tool holders are not checked.

## Scripting from CadQuery

`python/stepcam.py` lets a CadQuery script build a step milling job. Faces and edges are chosen with CadQuery selectors instead of mouse clicks. Each operation is made exactly as the right-click menu would make it, so the defaults are the same, and the job opens in the app afterwards.

### Setting up

1. Build stepCAM: `cargo build --release`.
2. Make `stepcam.py` importable: copy it next to your script, add the `python/` folder to `PYTHONPATH`, or use `sys.path.insert`.
3. The module finds the `stepcam` program in this order:
   - the `binary=` argument;
   - the `STEPCAM` environment variable;
   - `stepcam` on the PATH;
   - the repository's own `target/release` build.

### Example

```
import cadquery as cq
from stepcam import Job

part = (cq.Workplane().rect(50, 30).extrude(8)
        .faces(">Z").workplane().rect(30, 14).cutBlind(-4))

job = Job(part, stock=(6, 6, 1))        # 3 mm per side, 1 mm on top
job.rough(tool=4, stock_to_leave=0.15)
job.face(">Z")
job.pocket(part.faces(">Z[-2]"), tool=3)
job.profile(part.faces("<Z"), side="outside", bottom_z=-0.2)
job.tabs(part.faces("<Z").edges("|X"), at=(0.25, 0.75), width=5, height=1.5)
res = job.generate("bracket.nc", simulate=True)
print(res.run_time_text, res.simulation["max_gouge"])
job.open()                               # look at it in stepCAM
```

`python/example.py` is a complete example with threads and engraving.

### The job

`Job(part, stock=(x, y, z), stock_offset=True, rapid_above=10, origin_xy="model", origin_z="model", settings_file=None, settings=None)`

- **part:** a CadQuery Workplane or Shape, in machine coordinates. Move and rotate it in CadQuery to place it on the machine.
- **stock:** with `stock_offset=True`, the values are added to the part's bounding box. With `False`, they are the absolute stock size. This is the same as the Stock inputs on the Step Milling tab.
- **stock_z_anchor:** `"bottom"` (default), `"middle"` or `"top"`, the same as **Part in stock**.
- **origin_xy / origin_z:** the G-code origin.
  - `origin_xy`: `"model"`, `"center"`, `"bottom_left"`, `"bottom_right"`, `"top_left"`, `"top_right"`, or an `(x, y)` pair.
  - `origin_z`: `"model"`, `"stock_top"`, `"stock_base"`, or a Z value.

  `result.origin` gives where it ended up, in model coordinates.
- **settings_file:** a settings file saved from Tools & Settings, holding your tool table.
- **settings:** overrides by name, for example `{"spindle_rpm": 12000, "coolant": False}`.

### Operations

Each method takes a selection: a selector string applied to the part (such as `">Z"`), a Workplane, a Shape, or a list of them. They are added to the job in the order you call them.

- `job.rough(**params)`
- `job.finish(**params)`: wall finishing after roughing. `result.warnings` says if no `rough()` comes before it.
- `job.face(faces=">Z", **params)`: one Face per flat, up-facing face.
- `job.pocket(faces_or_edges, **params)`:
  - faces: flat or sloped faces that face up;
  - edges: closed, level loops, one pocket per loop.
- `job.profile(edges, side="outside", **params)`: one profile per edge chain. A face selects its outline.
- `job.engrave(face, text, at=(x, y), **params)`: `at` is the anchor point in machine coordinates.
- `job.thread(holes, **params)`: the selected cylindrical holes, as **Add to thread milling**.
- `job.thread_alike(hole, **params)`: every hole like the selected one, as **Thread mill all alike holes**.
- `job.omit(faces)`: **Omit area** for each selected face. The ops added after it treat those areas as filled in.
- `job.tabs(edges, at=0.5, **params)`: a mill tab on each selected edge, at the fraction `at` along it. Give a list, such as `at=(0.25, 0.75)`, for several tabs per edge.

Every method also takes `enabled=False`, which adds the operation switched off.

**Parameters** use the field names shown in the app. These are accepted:

- **All operations:** `tool` (a tool number), `feed`, `speed`, `start_z`, `depth_per_pass`, `stepover` (%), `ramp_angle`, `strategy` (`"standard"` / `"adaptive"`), `entry` (`"helix"` / `"zigzag"` / `"plunge"`).
- **Rough:** `layer_height`, `stock_to_leave`, `bottom_z`, `overcut_z`, `clear_voids`.
- **Finish:** `z_step` (default 0.1), `stock_to_leave`, `bottom_z`.
- **Face:** `extra` (past the outline).
- **Pocket:** `finish` (the finish pass amount), `keep_islands`, `floor_z` (pockets made from edges).
- **Profile:** `side` (`"outside"` / `"inside"` / `"center"`), `bottom_z`.
- **Engrave:** `height`, `depth`, `font` (`"Sans"`, `"Mono"` or a font file), `align`, `valign`, `angle`, and `dx` / `dy` from the face centre instead of `at`.
- **Thread:** `thread` (`"M3"`), `length`, `start_z`, `descent_angle`. Here `tool` is the boring end mill.
- **Tabs:** `width` (along the edge), `depth` (across it), `height`.

Field names as saved in a job file are accepted too. A misspelt name is an error that lists the valid ones.

**Selector tips:**
- `">Z"` is the highest face, and `">Z[-2]"` the second highest (`">Z[0]"` is the lowest).
- `"%CYLINDER"` selects cylindrical faces, but fillets are cylinders too. Narrow the selection down, for example `part.faces("%CYLINDER").faces(">X")`.
- `part.faces("<Z").edges("|X")` selects the bottom outline edges that run along X, which suits tabs.

### Results

`job.generate("part.nc", job=None, step=None, simulate=False)` writes three files:
- the G-code;
- `part.stepcam.json`, which you can open with **Load** in the app or with `job.open()`;
- `part.step`, the exported model, which the job refers to.

It returns a result with:
- `ops` (the operation labels), `lines`, `run_time` (s) and `run_time_text`;
- `warnings`, and `gcode` (the program text).

With `simulate=True` (or `{"detail": 300, "tolerance": 0.02}`) the program also runs through the simulator, and `result.simulation` gives:
- `removed_mm3` and `at_size_pct`;
- `max_stock_left` and `max_stock_left_at` (where, as [x, y, z of the cut surface]), `max_gouge`, `max_gouge_at` and `gouged_cells`;
- `events`: rapids through material and cuts deeper than the flutes, each with its line number.

Threads and engraving count as gouges here, because the model has neither.

If stepCAM can't build the job, for example because a selection isn't on a suitable face or a parameter is unknown, `generate()` raises `StepCamError` with the reasons. Pass `check=False` to get a result with `ok=False` and `errors` instead.

### Without Python

The module runs `stepcam batch spec.json`, which reads a JSON description of the job and prints a JSON report. Faces and edges in the description are given as points on the model. The format is described at the top of `src/batch.rs`. `stepcam part.stepcam.json` opens a job in the app.

## Tools & Settings tab

### Machining settings

- **Thread milling: spindle speed, boring feed, threading feed:** for thread milling items. The boring feed is at the cutter centre. The threading feed is at the cutting edge, and the feed at the tool centre is reduced automatically.
- **Thread milling: default descent angle:** the helix angle new thread milling items start with.
- **Clearance above hole start Z (between holes):** how high the tool lifts between holes.
- **Plunge feed.**
- **Boring ring stepover (% of cutter Ø).**
- **Max acceleration (run-time estimate):** default 200 mm/s².
- **Rapid rate G0 (run-time estimate):** default 3000 mm/min.
- **Coolant (M8):** coolant on during cutting.
- **Tool length comp (G43 Hn):** adds `G43 Hn` after each tool change.
- **Face pattern:** the facing pattern new Face ops start with, and new Rough ops' stock facing (Unidirectional by default).
- **Rough strategy:** the strategy new Rough ops start with (Standard by default), with its default stepover: 20 % for adaptive clearing, else 40 %.

Both apply to ops added afterwards. Existing ops keep their own choice. In batch and Python they are the `face_pattern` and `rough_strategy` settings.

### Tool tables

Tool numbers must be unique across all three tables. Groups and operations follow their tool when you edit it.

- **Milling cutters (end mills):** tool number, **Cutter Ø** and **Flute length**. **➕ Add cutter** adds one. The defaults are T1 to T5: 1×3, 1.5×6, 2×12, 3×17 and 3.175×17.
- **Thread mills (single profile):** one for each thread size, with tool number, Ø (it must be smaller than the hole) and **Reach**. The defaults are T11 to T16 for M2 to M6.
- **Engraving bits (tapered):** tool number, **Included angle** (the full angle of the cone), **Tip flat Ø** and **Max depth**. **➕ Add engraving bit** adds one. The defaults are T21 to T23: 30°, 60° and 90°, with 0.1, 0.2 and 0.3 mm tips.

**Reset all settings to defaults** puts everything back.

## The G-code

- **Header:** `G21 G90 G17 G94 G40 G49 G80`, which means metric, absolute coordinates, XY plane, feed per minute, and no compensation or canned cycles.
- **Tool changes:** `M5 M9` stops the spindle and coolant, the tool goes up to the rapid plane, then comes `Tn M6 (description)`, `G43 Hn` if tool length comp is on, `Sxxxx M3`, and `M8` if coolant is on.
- **End of program:** `M5`, `M9`, `M30`.
- **Moves:** only G0, G1, G2 and G3 (arcs use I/J), so nearly any controller can run the program.
- **Coordinates:** relative to the **G-code origin**.
- **Comments** in brackets name each operation.

**Estimated run time:** every move speeds up and slows down at the max acceleration, up to its feed or the rapid rate. Corners slow down as on a typical controller, so smooth paths keep their speed. Tool changes and spindle spin-up are not counted.

## Mouse and keyboard

| Input | Action |
|---|---|
| F1 | Open this help |
| Left-drag | Orbit the 3D view |
| Right-drag or middle-drag | Pan |
| Scroll | Zoom at the cursor |
| Double-click | Fit the view |
| Click | Select a surface or an edge (Step Milling) |
| Ctrl+click | Add or remove a hole from the selection |
| Right-click | Operations menu (Step Milling) |
| Esc | Clear the selection and the highlight |
| ↑ / ↓ in the G-code panel | Highlight the move of each line |
| Space / ← / → / Home / End | Simulator playback |

## Troubleshooting

- **"Cannot generate G-code":** the file bar lists each problem, for example a tool that isn't in the tool table, a cutter too big for the hole, or an empty location list. Fix them and the program reappears.
- **The surface menu says "Only for surfaces normal to Z":** the model isn't sitting flat. Use **Lay flat on this surface** on its bottom face first.
- **A hole won't thread mill:** it must be a full, vertical cylinder of a size within 0.3 mm of an M2 to M6 tap drill or major Ø. Bosses and partial cylinders don't qualify.
- **The simulator shows a red gouge:** click the area's G-code line or use ⏪ / ⏩ to find the move. Check the operation's depths and tool, and the stock to leave.
- **The simulator warns about the flute length:** use a longer tool, or set a smaller Bottom Z range for that tool.
- **A job loads without its model:** the STEP file moved. Import it again. The placement and operations are kept.
