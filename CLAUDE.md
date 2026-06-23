# edge-clip — KiCad plugin to clip graphics to Edge.Cuts

## What this is
A KiCad PCB Editor plugin that clips graphic shapes on a chosen layer (default
**F.Mask**) to the board outline defined on **Edge.Cuts**. It removes geometry
hanging outside the board so the Gerbers are unambiguous and the fab's
auto-quoter sizes from the real board, not the artwork's bounding box.

## Why this exists
A decorative bitmap-traced graphic on F.Mask extends far beyond the actual board
outline (the "Disc Dog Deck" board). Fabs clip mask/copper/silk to the routed
profile at CAM time, but two problems remain:
1. The huge overhang gets flagged by a human CAM reviewer ("did you mean this?").
2. The automated quoter prices off the **Gerber bounding box** — which is the
   oversized art, not the board — so the quote won't drop to the true board size
   until the art is clipped.

Clipping the geometry to Edge.Cuts fixes both, and fixing #2 is the main driver.

## Environment / key decisions
- KiCad **10.0.3**, macOS.
- Build as an **IPC API plugin — NOT a SWIG Action Plugin.** SWIG is deprecated
  in KiCad 9/10 and removed in 11. IPC runs in its own process with a
  KiCad-managed virtualenv, so dependencies (notably Shapely) install cleanly
  instead of fighting KiCad's embedded Python.
- Plugin is developed **directly in** `~/Documents/KiCad/10.0/plugins/edge-clip`
  as a git repo (no symlink). Confirm the version folder name is actually `10.0`.
- The KiCad **API server must be enabled**: Preferences > Plugins.

## Dependencies (requirements.txt)
- `kicad-python` (the `kipy` library) — IPC client
- `shapely` — polygon stitching, buffering (inset), and intersection/clipping

KiCad auto-creates a per-plugin venv inside the plugin folder and installs these
on first load.

## Files to scaffold
- `plugin.json` — manifest; registers a toolbar action ("Clip to Edge.Cuts").
- `requirements.txt` — kicad-python, shapely.
- entry-point script (e.g. `ipc_entry.py`) — connect over socket, get board,
  run the clip; manages its own wx event loop if a dialog is used.
- `.gitignore` — ignore the per-plugin venv KiCad creates inside the plugin
  dir, plus `__pycache__/` and `*.pyc`.

## Clipping logic
1. Connect to the running KiCad over IPC; get the active board.
2. Read Edge.Cuts graphics. They may be separate lines/arcs, not a single closed
   polygon — stitch endpoints into a closed polygon with Shapely. Validate it's
   closed; bail with a clear message if it isn't.
3. Build the clip region. Apply an optional configurable **pullback/inset**
   (default ~0.3 mm) so mask doesn't run flush to the routed edge —
   `shapely` `buffer(-inset)`.
4. Target layer is parameterized; default **F.Mask**. Allow B.Mask, silk, and
   copper graphics too.
5. For each graphic shape on the target layer: compute the Shapely intersection
   with the clip region. Replace the original with the clipped result — note it
   may become multiple polygons, or empty (then delete it).
6. Scope option: whole board vs. current selection only.
7. Make the whole operation a single undoable commit/transaction so Ctrl+Z
   reverts it cleanly.

## Watch-outs
- KiCad internal units are **nanometers**; convert to/from mm for Shapely
  (÷ 1e6 / × 1e6).
- **Verify IPC geometry coverage early** on 10.x — confirm the API can read
  Edge.Cuts shapes and create/replace graphic polygons on F.Mask *before*
  building any UI. This is the main unknown.
- Bitmap-traced art = thousands of tiny polygons. Test performance on the real
  board and batch the replace into one commit.

## Verification
After clipping, generate Gerbers (kicad-cli or the plot dialog) and open them in
GerbView. The bounding-box / extents readout should equal the real board size.
If it's still huge, something else (stray silk or copper on that same footprint)
is also overhanging — the quoter unions all layers.

## First milestone
Load the plugin, confirm the toolbar button appears and the socket connects, and
print the board name + Edge.Cuts shape count. No geometry edits yet. Then add the
clip.
