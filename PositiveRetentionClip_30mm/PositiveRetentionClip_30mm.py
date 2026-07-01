# Units: mm throughout.

from build123d import *
from build123d import WorkplaneList  # not in __all__, needed for hole placement
from build123d.topology import Compound
import math
try:
    from ocp_vscode import show
    _has_ocp = True
except ImportError:
    _has_ocp = False

# ---- Boolean operation helpers ----
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse as BFuse, BRepAlgoAPI_Cut as BCut, BRepAlgoAPI_Common as BCommon

def fuse_solids(solid1, solid2):
    """Fuse two solids with fuzzy tolerance. Falls back to Compound if they do not intersect."""
    # v16.99: guard against None inputs (first feature skipped/failed → part.part is None)
    if solid1 is None: return solid2
    if solid2 is None: return solid1
    from OCP.TopAbs import TopAbs_SOLID
    try:
        fuse_op = BFuse(solid1.wrapped, solid2.wrapped)
        fuse_op.SetFuzzyValue(0.01)
        fuse_op.Build()
        result_shape = fuse_op.Shape()
        if not result_shape.IsNull():
            from OCP.TopAbs import TopAbs_COMPOUND
            from OCP.TopExp import TopExp_Explorer
            from OCP.TopoDS import TopoDS
            if result_shape.ShapeType() == TopAbs_SOLID:
                return Solid(result_shape)
            if result_shape.ShapeType() == TopAbs_COMPOUND:
                from OCP.TopAbs import TopAbs_SOLID as _TS
                _exp = TopExp_Explorer(result_shape, _TS)
                _solids = []
                while _exp.More():
                    _solids.append(Solid(TopoDS.Solid_s(_exp.Current())))
                    _exp.Next()
                if len(_solids) == 1:
                    return _solids[0]
                if len(_solids) > 1:
                    return Compound(_solids)
    except:
        pass
    # Non-touching bodies: OCC returns a Compound — accumulate explicitly.
    existing = list(solid1.solids()) if isinstance(solid1, Compound) else [solid1]
    new_s = list(solid2.solids()) if isinstance(solid2, Compound) else [solid2]
    return Compound(existing + new_s)

def cut_solids(shape, tool):
    """Cut tool from shape. When tool extends beyond shape (through-all cuts), BCut returns a
    Compound with the cut body AND the tool remainder. _extract_cut_result discards solids
    outside the original bounding box so stray geometry is not returned."""
    if shape is None: return None
    if tool is None: return shape

    def _extract_cut_result(raw_shape, original_solid):
        from OCP.TopAbs import TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        if raw_shape.IsNull(): return original_solid
        exp = TopExp_Explorer(raw_shape, TopAbs_SOLID)
        solids = []
        while exp.More():
            s = Solid(TopoDS.Solid_s(exp.Current()))
            if not s.wrapped.IsNull() and len(list(s.faces())) > 0:
                solids.append(s)
            exp.Next()
        if not solids: return original_solid
        if len(solids) == 1: return solids[0]
        # Multiple solids: tool extended beyond the shape. Keep only solids inside original bbox.
        try:
            obb = original_solid.bounding_box()
            tol = 0.5
            kept = []
            for s in solids:
                try:
                    sbb = s.bounding_box()
                    if (sbb.min.X >= obb.min.X - tol and sbb.max.X <= obb.max.X + tol and
                        sbb.min.Y >= obb.min.Y - tol and sbb.max.Y <= obb.max.Y + tol and
                        sbb.min.Z >= obb.min.Z - tol and sbb.max.Z <= obb.max.Z + tol):
                        kept.append(s)
                except:
                    kept.append(s)
            if len(kept) == 1: return kept[0]
            if len(kept) > 1: return Compound(kept)
        except:
            pass
        return max(solids, key=lambda s: len(list(s.faces())))

    try:
        if isinstance(shape, Compound):
            result_solids = []
            for solid in shape.solids():
                try:
                    sbb = solid.bounding_box()
                    tbb = tool.bounding_box()
                    overlap = (
                        not (sbb.max.X < tbb.min.X or sbb.min.X > tbb.max.X) and
                        not (sbb.max.Y < tbb.min.Y or sbb.min.Y > tbb.max.Y) and
                        not (sbb.max.Z < tbb.min.Z or sbb.min.Z > tbb.max.Z)
                    )
                    if overlap:
                        cut_op = BCut(solid.wrapped, tool.wrapped)
                        cut_op.SetFuzzyValue(0.01)
                        cut_op.Build()
                        result_solids.append(_extract_cut_result(cut_op.Shape(), solid))
                    else:
                        result_solids.append(solid)
                except:
                    result_solids.append(solid)
            return result_solids[0] if len(result_solids) == 1 else Compound(result_solids)
        else:
            sbb = shape.bounding_box()
            tbb = tool.bounding_box()
            overlap = (
                not (sbb.max.X < tbb.min.X or sbb.min.X > tbb.max.X) and
                not (sbb.max.Y < tbb.min.Y or sbb.min.Y > tbb.max.Y) and
                not (sbb.max.Z < tbb.min.Z or sbb.min.Z > tbb.max.Z)
            )
            if overlap:
                cut_op = BCut(shape.wrapped, tool.wrapped)
                cut_op.SetFuzzyValue(0.01)
                cut_op.Build()
                return _extract_cut_result(cut_op.Shape(), shape)
            return shape
    except:
        return shape

def intersect_solids(solid1, solid2):
    """Intersect two solids (keep only overlapping volume). Used to clip a
    mirrored cut tool to its companion body so the subtract stays bounded.
    Returns solid1 unchanged if either input is None or BCommon fails."""
    if solid1 is None or solid2 is None:
        return solid1
    try:
        common_op = BCommon(solid1.wrapped, solid2.wrapped)
        common_op.SetFuzzyValue(0.01)
        common_op.Build()
        result = common_op.Shape()
        if result is None or result.IsNull():
            return solid1
        from OCP.TopAbs import TopAbs_SOLID
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        exp = TopExp_Explorer(result, TopAbs_SOLID)
        solids = []
        while exp.More():
            solids.append(Solid(TopoDS.Solid_s(exp.Current())))
            exp.Next()
        if len(solids) == 1:
            return solids[0]
        if len(solids) > 1:
            return Compound(solids)
        return solid1
    except Exception:
        return solid1

# -- Edge selection helpers --
# Use these to select edges for fillet/chamfer operations.
# Find edge coordinates using the diagnostic pattern at the bottom of this file.

def get_edge_by_endpoints(solid, p1, p2, tol=0.05):
    for e in solid.edges():
        verts = e.vertices()
        if len(verts) != 2:
            continue
        pts = [(v.X, v.Y, v.Z) for v in verts]
        if (all(abs(pts[0][i]-p1[i])<tol for i in range(3)) and
            all(abs(pts[1][i]-p2[i])<tol for i in range(3))) or \
           (all(abs(pts[0][i]-p2[i])<tol for i in range(3)) and
            all(abs(pts[1][i]-p1[i])<tol for i in range(3))):
            return e
    return None

def get_vertical_edge(solid, x, y, z0, z1, tol=0.01):
    for e in solid.edges():
        verts = e.vertices()
        if len(verts) != 2:
            continue
        xs = [v.X for v in verts]
        ys = [v.Y for v in verts]
        zs = sorted([v.Z for v in verts])
        if (abs(xs[0]-x)<tol and abs(xs[1]-x)<tol and
            abs(ys[0]-y)<tol and abs(ys[1]-y)<tol and
            abs(zs[0]-z0)<tol and abs(zs[1]-z1)<tol):
            return e
    return None

# All dimensions below are raw numbers.

# 'Sketch1': 4 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(80.0, 0.0, -0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((43.0, -0.0), (0.0, 0.0))
        Line((0.0, 0.0), (0.0, 13.0))
        Line((0.0, 13.0), (43.0, 13.0))
        Line((43.0, 13.0), (43.0, -0.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 6 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((72.0, 3.0), (73.0, 4.0), -1.0)
        Line((73.0, 4.0), (73.0, 46.5553))
        Line((73.0, 46.5553), (7.0, 46.5553))
        Line((7.0, 46.5553), (7.0, 4.0))
        RadiusArc((7.0, 4.0), (8.0, 3.0), -1.0)
        Line((8.0, 3.0), (72.0, 3.0))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch3': 24 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((-31.0, 26.5176), (-50.1709, 26.5176))
        Line((-50.1709, 26.5176), (-50.1709, 13.0))
        Line((-50.1709, 13.0), (-50.1709, -7.1447))
        Line((-50.1709, -7.1447), (-35.1385, -7.1447))
        Line((-35.1385, -7.1447), (-35.1385, 0.0))
        Line((-35.1385, 0.0), (-42.0, 0.0))
        RadiusArc((-42.0, 0.0), (-43.0, 1.0), 1.0)
        Line((-43.0, 1.0), (-43.0, 12.0))
        RadiusArc((-43.0, 12.0), (-42.0, 13.0), 1.0)
        Line((-42.0, 13.0), (-32.0, 13.0))
        RadiusArc((-32.0, 13.0), (-31.0, 12.0), 1.0)
        Line((-31.0, 12.0), (-31.0, 4.0))
        RadiusArc((-31.0, 4.0), (-30.0, 3.0), -1.0)
        Line((-30.0, 3.0), (-4.0, 3.0))
        RadiusArc((-4.0, 3.0), (-3.0, 4.0), -1.0)
        Line((-3.0, 4.0), (-3.0, 9.0))
        RadiusArc((-3.0, 9.0), (-2.0, 10.0), 1.0)
        Line((-2.0, 10.0), (-1.0, 10.0))
        RadiusArc((-1.0, 10.0), (-0.0, 9.0), 1.0)
        Line((-0.0, 9.0), (2.6783, 9.0))
        Line((2.6783, 9.0), (2.6783, 26.5176))
        Line((2.6783, 26.5176), (-0.0, 26.5176))
        Line((-0.0, 26.5176), (-3.0, 26.5176))
        Line((-3.0, 26.5176), (-31.0, 26.5176))
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch4': 4 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        Line((-41.0, 11.2294), (-32.9962, 11.2294))
        Line((-32.9962, 11.2294), (-32.9962, 2.9962))
        Line((-32.9962, 2.9962), (-41.0, 3.0))
        Line((-41.0, 3.0), (-41.0, 11.2294))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch5': 10 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -0.0, -1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with BuildLine():
        RadiusArc((-1.0, 0.0), (0.0, 1.0), -1.0)
        Line((0.0, 1.0), (0.0, 3.0536))
        Line((0.0, 3.0536), (2.1339, 3.0536))
        Line((2.1339, 3.0536), (2.1339, -2.5464))
        Line((2.1339, -2.5464), (-83.243, -2.5464))
        Line((-83.243, -2.5464), (-83.243, 3.483))
        Line((-83.243, 3.483), (-80.0, 3.483))
        Line((-80.0, 3.483), (-80.0, 1.0))
        RadiusArc((-80.0, 1.0), (-79.0, -0.0), -1.0)
        Line((-79.0, -0.0), (-1.0, 0.0))
    _inc_edges_sk_Sketch5_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_5 = Wire.combine(_inc_edges_sk_Sketch5_5)[0]
_wire_sk_Sketch5_5 = _wire_sk_Sketch5_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch5_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch5_5.wrapped, True)
_face_sk_Sketch5_5 = Face(_mkf_sk_Sketch5_5.Face())

# 'Sketch6': 10 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch6_6:
    with BuildLine():
        RadiusArc((-80.0, -9.0), (-79.0, -10.0), -1.0)
        Line((-79.0, -10.0), (-1.0, -10.0))
        RadiusArc((-1.0, -10.0), (0.0, -9.0), -1.0)
        Line((0.0, -9.0), (-0.0, -7.7633))
        Line((-0.0, -7.7633), (1.0707, -7.7633))
        Line((1.0707, -7.7633), (1.0708, -15.4535))
        Line((1.0708, -15.4535), (-80.4554, -15.4538))
        Line((-80.4554, -15.4538), (-80.4555, -8.4394))
        Line((-80.4555, -8.4394), (-80.0, -8.4394))
        Line((-80.0, -8.4394), (-80.0, -9.0))
    _inc_edges_sk_Sketch6_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_6 = Wire.combine(_inc_edges_sk_Sketch6_6)[0]
_wire_sk_Sketch6_6 = _wire_sk_Sketch6_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch6_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch6_6.wrapped, True)
_face_sk_Sketch6_6 = Face(_mkf_sk_Sketch6_6.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(1.0, 0.0, 0.0) * -80.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -80.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -55.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -55.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(-1.0, 0.0, 0.0) * -94.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -94.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(-1.0, 0.0, 0.0) * -89.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -89.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch5_5
    _vec = Vector(-0.0, -0.0, -1.0) * -19.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -19.000000 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch6_6
    _vec = Vector(-0.0, -1.0, -0.0) * -6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -6.000000 mm
    
    
    try:
        # OCP-confirmed indices: [38, 39, 40, 41]
        # OCP-confirmed indices: [38, 39, 40, 41]
        fillet([part.edges()[38], part.edges()[39], part.edges()[40], part.edges()[41]], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet1 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    
   
    try:
        # OCP-confirmed indices: [52, 55, 56, 58]
        # OCP-confirmed indices: [52, 55, 56, 58]
        fillet([part.edges()[52], part.edges()[55], part.edges()[56], part.edges()[58]], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet2 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    
   
    try:
        # OCP-confirmed indices: [92, 93, 94, 95]
        # OCP-confirmed indices: [92, 93, 94, 95]
        fillet([part.edges()[92], part.edges()[93], part.edges()[94], part.edges()[95]], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet3 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    
    
    try:
        # OCP-confirmed indices: [52, 54, 57, 58]
        # OCP-confirmed indices: [52, 54, 57, 58]
        fillet([part.edges()[52], part.edges()[54], part.edges()[57], part.edges()[58]], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet4 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)


