# edge-clip — KiCad IPC Plugin

Clips graphic shapes on chosen layers to the board outline (Edge.Cuts), then
commits the result as a single undoable operation.

**Why this exists:** bitmap-traced artwork placed on F.Mask (or any other
layer) often extends far beyond the board edge. Fabs clip it at CAM time, but
automated quoters size the board from the Gerber bounding box — which is the
oversized art, not the routed outline — so the quote comes in wrong. Clipping
the geometry before export fixes the quote and eliminates "did you mean this?"
flags from human CAM reviewers.

---

## Requirements

- KiCad 10.x (IPC API; **not** compatible with SWIG/Action Plugin architecture)
- KiCad API server enabled: **Preferences › Manage Plugins** (or **Preferences › Scripting**)
- Python dependencies are installed automatically by KiCad into a per-plugin
  virtualenv on first load — no manual pip needed.

---

## Installation

Place (or clone) this directory inside your KiCad user plugins folder:

```
~/Documents/KiCad/10.0/plugins/kicad-pcb-edge-clipper/
```

Restart the PCB editor. A **"Clip to Edge.Cuts"** button appears on the right
toolbar.

---

## Usage

1. Open your `.kicad_pcb` file in the PCB editor.
2. Verify the board outline on **Edge.Cuts** is a fully-closed shape (all
   segments/arcs connected end-to-end).
3. Click **Clip to Edge.Cuts** in the toolbar.
4. The plugin prints progress to stdout; KiCad applies all changes as a single
   commit — **Ctrl+Z reverts everything cleanly.**

---

## Configuration

Edit the constants at the top of `ipc_entry.py`:

### `TARGET_LAYERS`

The set of layers whose graphics will be clipped. Default:

```python
TARGET_LAYERS = {
    BoardLayer.BL_F_Mask,
    BoardLayer.BL_B_Mask,
    BoardLayer.BL_F_Cu,
    BoardLayer.BL_B_Cu,
    BoardLayer.BL_F_SilkS,
    BoardLayer.BL_B_SilkS,
    BoardLayer.BL_F_Paste,
    BoardLayer.BL_B_Paste,
    BoardLayer.BL_F_Fab,
    BoardLayer.BL_B_Fab,
}
```

Add or remove `BoardLayer.BL_*` values to match your use case. Full layer
names are listed in `kipy/util/board_layer.py` inside the plugin venv.

### `INSET_MM`

How far to pull the clip boundary inward from Edge.Cuts (default `0.3` mm).
Set to `0` to clip flush to the routed edge.

```python
INSET_MM = 0.3
```

---

## How it works

1. Reads all shapes on **Edge.Cuts** and stitches them into a closed Shapely
   polygon (arcs are discretized to 72 segments per circle).
2. Shrinks the polygon inward by `INSET_MM` using `shapely.buffer(-inset)`.
3. For every graphic shape on each target layer — both board-level shapes and
   shapes inside footprint definitions — computes the Shapely intersection with
   the clip region.
4. Replaces each original shape with the clipped result (one or more polygons,
   or nothing if it was entirely outside).
5. Wraps all changes in a single `begin_commit` / `push_commit` transaction.

---

## Verification

After clipping, generate Gerbers (**File › Plot**) and open them in GerbView.
The **extents** readout should match the true board dimensions. If it's still
oversized, another layer (copper, silk, or a different footprint) also has
overhanging art — run the plugin again with that layer added to `TARGET_LAYERS`.

---

## Dependencies

| Package | Purpose |
|---|---|
| `kicad-python` (`kipy`) | IPC client for the KiCad API |
| `shapely` | Polygon stitching, buffering, and intersection |
