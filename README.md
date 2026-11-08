# stepCAM

Generates G-code to helically bore holes and then thread-mill them (M2–M6).

    cargo run --release

- **Thread Milling** tab: the *Thread Locations* table. Each group has a list of x,y locations,
  a thread size, a thread length (0–100 mm) and a hole-milling cutter.
- **Tools & Settings** tab: Z heights, feeds/speeds, and the editable tool tables (end mills and the thread mill for each size). **Save settings… / Load settings…** store all of it as a JSON file.
- Hole depth = thread length + 1 pitch. Threads are milled bottom-up (G03, climb) from
  Z = top − length, so they stop one pitch above the bottom of the hole.
- Tools (defaults, all editable): cutters T1–T5 (1x3, 1.5x6, 2x12, 3x17, 3.175x17), thread mills T11–T16 (M2…M6). Add, remove and renumber end mills or change their Ø / flute length; groups and operations using a tool follow the edit. Tool numbers must be unique across both tables.
- All boring runs first, then all threading, so there are as few tool changes as possible.

## Step Milling tab

- **Import STEP…** loads a part (B-rep faces built from lines, circles, ellipses and B-splines; loose
  wire edges too). Flat faces are shaded; curved faces show as edges only. Inch files are scaled to mm.
- **Click** highlights a surface or an edge loop/chain; **Esc** clears it. **Right-click** opens the menu:
  - flat surface facing +Z → **Face** (mill the whole part down to it, with overlap past the part) or
    **Pocket** (clear inside its outer boundary; inner loops ignored unless "keep islands" is ticked)
  - any edge → **Add mill tab** (rectangle: width along the edge tangent, depth across it, Z height up
    from the edge). Every cut lifts over the tab, allowing for the cutter radius.
- Pocket strategies: **Standard** (offset passes) or **Adaptive clearing** (trochoidal loops). Entry styles:
  helix, zigzag or plunge. Roughing leaves the *finish pass* amount on the walls, then a climb-milled
  finishing pass cuts them to size at each depth.
- `testdata/pocket_block.step` is a sample part (made by `testdata/make_step.py`).

## G-code ↔ 3D view

Click any line in the G-code listing (or move through it with the arrow keys)
and the move that line produces is highlighted in cyan in the 3D view of the
current tab — a hollow dot marks where the move starts, a filled dot where it
ends. Lines without motion (comments, tool changes, M codes) highlight nothing.
Esc clears the highlight.

## Placing the model (Step Milling)

Right-click any flat surface of the imported model:

* **Set Z…** (surfaces normal to Z) - type a Z (default 0) and press Apply or
  Enter; the whole model moves up or down so that surface sits at that Z.
* **Lay flat on this surface** - rotates the model so the surface faces down
  (as if resting on the table on it), keeps the part's XY centre, and puts the
  new top at Z0. Use Set Z afterwards to put it anywhere else.
* **Reset placement** - back to the STEP file's own coordinates.

The placement is saved in the job. Existing operations follow the model:
Face/Pocket depths, boundaries and Start Z move with their surface, tabs move
and turn with the part. Operations whose surface no longer faces +Z (or tabs
on an edge that became vertical) are disabled and listed in the status bar.

## Edge operations (Step Milling)

Right-clicking an edge (which selects its loop or chain) offers:

* **Pocket** - (closed, level loops) the same pocket operation as on a surface,
  bounded by the edge loop. The floor Z is editable.
* **Profile** - cut along the edges. *Tool position*: **on centre line** (tool
  centre follows the edges, any chain), **inside** or **outside** (tool offset
  by its radius so the cut edge lies on the edges; closed loops only). Climb
  milled: outside runs CW, inside CCW. Each depth level ramps down along the
  path at the ramp angle, then makes a full pass at depth; open chains ramp
  back and forth. Tabs are lifted over as usual.
* **Add mill tab**.

Default depths come from the solid: an edge on a rim (part outline, pocket or
hole top) cuts from that edge down to the bottom of the adjoining walls; an
edge on a floor cuts from the top of its walls down to it; a free wire cuts
1 mm deep. Edit Start Z / Bottom Z in the operation as needed.

## Stock (Step Milling)

The **Stock X / Y / Z** inputs under the Step Milling heading define the raw
material, drawn as an orange wire box in the 3D view:

* **Offset** ticked (default, all 0 = stock is the part's bounding box): the
  values are added to the part's bounding box - X and Y split evenly over both
  sides, Z added on top.
* **Offset** unticked: the values are the absolute stock size, centred on the
  part in XY with its bottom level with the part bottom.

Toggling Offset converts the values so the stock itself does not change. The
resulting size and stock top Z are shown next to the inputs, with a warning if
the stock is smaller than the part.

Facing covers the whole stock (plus the overlap). New operations that cut from
the top of the part start at the stock top, and operations whose Start Z is at
the stock top follow it when the stock changes.

## Rapid plane (Step Milling)

Step milling sets its own rapid plane: **Rapid plane height above stock or
part** (default 10 mm) is added to the highest point of the stock or the part,
whichever is higher, and the resulting Z is shown next to the input. It
follows stock and model changes automatically. The *Rapid plane Z* on the
Thread Milling tab only applies to thread milling.

## Roughing (Step Milling)

**➕ Rough** (top of the Operations list) adds a roughing operation that
works layer by layer from the stock top (Start Z) down to the part bottom
(Bottom Z):

* Inside the part outline (seen from above) all material is cleared:
  pockets, holes and open steps such as the area above a flange. Open edges
  are cut right through, so no fin is left at the outline.
* Outside the part nothing is cleared: each exterior contour is profiled
  once per layer (CW, climb), ramping down along the contour.
* **Layer height** sets the layer spacing; an extra layer is added at every
  flat floor so floors are roughed to their depth.
* **Clear voids** (default on): pockets, steps and counterbores - anything
  with a face below it - are always cleared. Unticked, holes that go right
  through the part are not cleared, only profiled (CCW, climb, at tool
  radius + stock to leave inside the hole wall), so the core can drop out;
  exterior contours are still profiled.
* **Overcut Z** (default 0) carries the last layer this far below Bottom Z,
  e.g. to cut the part free from the stock.
* **Stock to leave** stays on all walls and floors for later Face / Pocket /
  Profile finishing ops.
* Strategy (standard / adaptive), entry (helix / zigzag / plunge), stepover,
  feed, speed and ramp angle work as for pockets. Climb milled.

The part is seen from above: flat up-facing surfaces count at their Z,
sloped or curved surfaces at their highest point (so material is left
rather than gouged), and undercuts are not reachable. The op follows stock,
model placement and part changes automatically.
