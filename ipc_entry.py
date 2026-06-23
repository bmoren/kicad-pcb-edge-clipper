#!/usr/bin/env python3
"""
edge-clip: clip graphics on a chosen layer to the board outline (Edge.Cuts)

Called by KiCad IPC API when the "Clip to Edge.Cuts" toolbar action fires.
KiCad sets KICAD_API_SOCKET and KICAD_API_TOKEN in the environment before
launching this script; kipy reads them automatically.
"""
import math
import sys
import traceback

from shapely.geometry import LineString, MultiPolygon, Point, Polygon as SPolygon
from shapely.ops import polygonize, unary_union

from kipy.kicad import KiCad
from kipy.board_types import (
    BoardArc, BoardCircle, BoardPolygon, BoardRectangle, BoardSegment, BoardShape,
    BoardLayer, to_concrete_board_shape,
)
from kipy.proto.board import board_types_pb2

# ── Layers to clip ────────────────────────────────────────────────────────────
# Graphics on these layers will be clipped to the Edge.Cuts outline.
# Add or remove layers here as needed.

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

# ── Other config ──────────────────────────────────────────────────────────────

INSET_MM = 0.3    # pull clip boundary inward from Edge.Cuts (0 = flush)
ARC_SEGS = 72     # segments used to discretize one full circle

# ── Unit helpers ─────────────────────────────────────────────────────────────

NM_PER_MM = 1_000_000

def _mm(nm: int) -> float:
    return nm / NM_PER_MM

def _nm(mm: float) -> int:
    return int(mm * NM_PER_MM)

# ── Arc discretization ───────────────────────────────────────────────────────

def _arc_coords(start, mid, end):
    """Return (x,y) sample points along the circular arc through three mm points."""
    ax, ay = start
    bx, by = mid
    cx, cy = end
    D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-10:
        return [start, end]
    sq = lambda x, y: x*x + y*y
    ux = (sq(ax,ay)*(by-cy) + sq(bx,by)*(cy-ay) + sq(cx,cy)*(ay-by)) / D
    uy = (sq(ax,ay)*(cx-bx) + sq(bx,by)*(ax-cx) + sq(cx,cy)*(bx-ax)) / D
    r  = math.hypot(ax - ux, ay - uy)
    a0 = math.atan2(ay - uy, ax - ux)
    am = math.atan2(by - uy, bx - ux)
    a1 = math.atan2(cy - uy, cx - ux)
    def after(a, ref):
        while a < ref: a += 2 * math.pi
        return a
    am_n = after(am, a0)
    a1_n = after(a1, a0)
    if am_n < a1_n:
        sweep = a1_n - a0
    else:
        sweep = (a1_n - 2*math.pi) - a0
    n = max(4, int(abs(sweep) / (2*math.pi) * ARC_SEGS))
    return [(ux + r*math.cos(a0 + sweep*i/n), uy + r*math.sin(a0 + sweep*i/n))
            for i in range(n + 1)]

# ── Edge.Cuts → Shapely line strings ─────────────────────────────────────────

def _edge_to_lines(shape):
    """Convert one Edge.Cuts BoardShape to Shapely line strings for polygonize."""
    if isinstance(shape, BoardSegment):
        return [LineString([(_mm(shape.start.x), _mm(shape.start.y)),
                            (_mm(shape.end.x),   _mm(shape.end.y))])]
    if isinstance(shape, BoardArc):
        pts = _arc_coords(
            (_mm(shape.start.x), _mm(shape.start.y)),
            (_mm(shape.mid.x),   _mm(shape.mid.y)),
            (_mm(shape.end.x),   _mm(shape.end.y)),
        )
        return [LineString(pts)]
    if isinstance(shape, BoardCircle):
        cx = _mm(shape.center.x);  cy = _mm(shape.center.y)
        r  = math.hypot(_mm(shape.radius_point.x)-cx, _mm(shape.radius_point.y)-cy)
        return [Point(cx, cy).buffer(r, resolution=ARC_SEGS).exterior]
    if isinstance(shape, BoardRectangle):
        x1, y1 = _mm(shape.top_left.x),     _mm(shape.top_left.y)
        x2, y2 = _mm(shape.bottom_right.x), _mm(shape.bottom_right.y)
        return [LineString([(x1,y1),(x2,y1),(x2,y2),(x1,y2),(x1,y1)])]
    if isinstance(shape, BoardPolygon):
        lines = []
        for pwh in shape.polygons:
            pts = [(_mm(n.point.x), _mm(n.point.y)) for n in pwh.outline.nodes if n.has_point]
            if pts: lines.append(LineString(pts + [pts[0]]))
        return lines
    return []

# ── Any BoardShape → filled Shapely geometry ──────────────────────────────────

def _shape_to_shapely(shape):
    """Convert a BoardShape to a filled Shapely geometry, or None for open shapes."""
    if isinstance(shape, BoardPolygon):
        parts = []
        for pwh in shape.polygons:
            ext = [(_mm(n.point.x), _mm(n.point.y)) for n in pwh.outline.nodes if n.has_point]
            if len(ext) < 3: continue
            holes = [[(_mm(n.point.x), _mm(n.point.y)) for n in h.nodes if n.has_point]
                     for h in pwh.holes]
            parts.append(SPolygon(ext, [h for h in holes if len(h) >= 3]))
        return unary_union(parts) if parts else None
    if isinstance(shape, BoardRectangle):
        x1, y1 = _mm(shape.top_left.x),     _mm(shape.top_left.y)
        x2, y2 = _mm(shape.bottom_right.x), _mm(shape.bottom_right.y)
        return SPolygon([(x1,y1),(x2,y1),(x2,y2),(x1,y2)])
    if isinstance(shape, BoardCircle):
        cx = _mm(shape.center.x);  cy = _mm(shape.center.y)
        r  = math.hypot(_mm(shape.radius_point.x)-cx, _mm(shape.radius_point.y)-cy)
        return Point(cx, cy).buffer(r, resolution=ARC_SEGS)
    # Segments and arcs are open — leave untouched
    return None

# ── Shapely polygon(s) → BoardPolygon list ────────────────────────────────────

def _make_board_polygon(s_poly, layer, template_proto=None):
    """Build a BoardGraphicShape proto for one Shapely Polygon and wrap it."""
    p = board_types_pb2.BoardGraphicShape()
    p.layer = layer
    if template_proto is not None:
        p.shape.attributes.CopyFrom(template_proto.shape.attributes)
    p.shape.polygon.SetInParent()
    pwh = p.shape.polygon.polygons.add()
    pwh.outline.closed = True
    for x, y in s_poly.exterior.coords[:-1]:
        node = pwh.outline.nodes.add()
        node.point.x_nm = _nm(x);  node.point.y_nm = _nm(y)
    for ring in s_poly.interiors:
        hole = pwh.holes.add()
        hole.closed = True
        for x, y in ring.coords[:-1]:
            node = hole.nodes.add()
            node.point.x_nm = _nm(x);  node.point.y_nm = _nm(y)
    return BoardPolygon(proto=p)

def _geom_to_board_polygons(geom, layer, template_proto=None):
    """Convert any Shapely geometry to a list of BoardPolygon objects."""
    if geom is None or geom.is_empty: return []
    if isinstance(geom, SPolygon):       geoms = [geom]
    elif isinstance(geom, MultiPolygon): geoms = list(geom.geoms)
    else: geoms = [g for g in polygonize(geom) if isinstance(g, SPolygon)]
    return [_make_board_polygon(g, layer, template_proto) for g in geoms if not g.is_empty]

# ── Clipping helpers ──────────────────────────────────────────────────────────

def _clip_shape(shape, clip_region):
    """
    Clip one BoardShape against clip_region.
    Returns (new_polygons, was_consumed) where was_consumed=True means the
    original shape should be removed/replaced.
    """
    concrete = to_concrete_board_shape(shape)
    if concrete is None: return [], False
    geom = _shape_to_shapely(concrete)
    if geom is None: return [], False          # open shape — leave as-is
    clipped = geom.intersection(clip_region)
    new_polys = _geom_to_board_polygons(clipped, concrete.layer, concrete._proto)
    return new_polys, True


def _clip_footprint_shapes(fp, clip_region):
    """
    Clip all target-layer shapes inside a footprint definition in-place.
    Returns True if any shapes were modified.
    """
    old_items = list(fp.definition.items)
    new_items = []
    modified  = False

    for item in old_items:
        if not isinstance(item, BoardShape) or item.layer not in TARGET_LAYERS:
            new_items.append(item)
            continue
        new_polys, consumed = _clip_shape(item, clip_region)
        if not consumed:
            new_items.append(item)
        else:
            new_items.extend(new_polys)
            modified = True

    if modified:
        fp.definition.items = new_items
    return modified

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    kicad = KiCad()
    board = kicad.get_board()
    print(f"[edge-clip] Board: {board.name}")

    # ── 1. Build clip region from Edge.Cuts ──────────────────────────────────
    all_shapes  = board.get_shapes()
    edge_shapes = [s for s in all_shapes if s.layer == BoardLayer.BL_Edge_Cuts]
    print(f"[edge-clip] Edge.Cuts shapes: {len(edge_shapes)}")

    lines = []
    for s in edge_shapes:
        lines.extend(_edge_to_lines(s))

    if not lines:
        print("[edge-clip] ERROR: No Edge.Cuts shapes found.", file=sys.stderr)
        sys.exit(1)

    board_polys = list(polygonize(unary_union(lines)))
    if not board_polys:
        print("[edge-clip] ERROR: Edge.Cuts do not form a closed polygon.", file=sys.stderr)
        sys.exit(1)

    board_poly  = max(board_polys, key=lambda p: p.area)
    clip_region = board_poly.buffer(-INSET_MM) if INSET_MM > 0 else board_poly

    if clip_region.is_empty:
        print(f"[edge-clip] ERROR: Clip region empty after {INSET_MM}mm inset.", file=sys.stderr)
        sys.exit(1)

    # ── 2. Clip board-level shapes ───────────────────────────────────────────
    board_target = [s for s in all_shapes if s.layer in TARGET_LAYERS]
    print(f"[edge-clip] Board-level target shapes: {len(board_target)}")

    board_to_delete = []
    board_to_create = []
    for shape in board_target:
        new_polys, consumed = _clip_shape(shape, clip_region)
        if consumed:
            board_to_delete.append(shape)
            board_to_create.extend(new_polys)

    # ── 3. Clip footprint shapes ─────────────────────────────────────────────
    footprints    = board.get_footprints()
    fps_to_update = []
    total_fp_shapes = 0

    for fp in footprints:
        fp_target = [s for s in fp.definition.shapes if s.layer in TARGET_LAYERS]
        if not fp_target:
            continue
        total_fp_shapes += len(fp_target)
        if _clip_footprint_shapes(fp, clip_region):
            fps_to_update.append(fp)

    print(f"[edge-clip] Footprint target shapes: {total_fp_shapes} across {len(fps_to_update)} footprint(s)")

    if not board_to_delete and not board_to_create and not fps_to_update:
        print("[edge-clip] Nothing to clip — all shapes already inside the boundary.")
        return

    # ── 4. Apply everything as one undoable commit ───────────────────────────
    print(f"[edge-clip] Applying: "
          f"{len(board_to_delete)} board shapes → {len(board_to_create)} clipped, "
          f"{len(fps_to_update)} footprint(s) updated")

    commit = board.begin_commit()
    try:
        if board_to_delete:
            board.remove_items(board_to_delete)
        if board_to_create:
            board.create_items(board_to_create)
        if fps_to_update:
            board.update_items(fps_to_update)
        board.push_commit(commit, "Clip graphics to Edge.Cuts")
        print("[edge-clip] Done. Use Ctrl+Z to undo.")
    except Exception:
        board.drop_commit(commit)
        raise


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[edge-clip] ERROR: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)
