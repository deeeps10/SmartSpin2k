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

# 'Sketch1': 6 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -9.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        RadiusArc((56.0, 15.0), (71.0, 0.0), -15.0)
        Line((71.0, 0.0), (170.0, 0.0))
        Line((170.0, 0.0), (170.0, 65.0))
        Line((170.0, 65.0), (71.0, 65.0))
        RadiusArc((71.0, 65.0), (56.0, 50.0), -15.0)
        Line((56.0, 50.0), (56.0, 15.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch4': 6 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 1.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch4_2:
    with BuildLine():
        RadiusArc((-58.1, 50.0), (-71.0, 62.9), -12.9)
        Line((-71.0, 62.9), (-167.9, 62.9))
        Line((-167.9, 62.9), (-167.9, 2.1))
        Line((-167.9, 2.1), (-71.0, 2.1))
        RadiusArc((-71.0, 2.1), (-58.1, 15.0), -12.9)
        Line((-58.1, 15.0), (-58.1, 50.0))
    _inc_edges_sk_Sketch4_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_2 = Wire.combine(_inc_edges_sk_Sketch4_2)[0]
_wire_sk_Sketch4_2 = _wire_sk_Sketch4_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch4_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch4_2.wrapped, True)
_face_sk_Sketch4_2 = Face(_mkf_sk_Sketch4_2.Face())

# 'Sketch5': 18 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, -6.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch5_3:
    with BuildLine():
        RadiusArc((-62.9, 11.35), (-60.9, 13.35), -2.0)
        Line((-60.9, 13.35), (-60.9, 52.9452))
        RadiusArc((-60.9, 52.9452), (-62.9, 54.9452), -2.0)
        Line((-62.9, 54.9452), (-74.0548, 54.9452))
        RadiusArc((-74.0548, 54.9452), (-76.0547, 56.9452), 2.0)
        Line((-76.0547, 56.9452), (-76.0547, 60.1))
        Line((-76.0547, 60.1), (-159.9452, 60.1))
        Line((-159.9452, 60.1), (-159.9452, 56.9452))
        RadiusArc((-159.9452, 56.9452), (-161.9452, 54.9452), 2.0)
        Line((-161.9452, 54.9452), (-165.1, 54.9452))
        Line((-165.1, 54.9452), (-165.1, 11.35))
        Line((-165.1, 11.35), (-161.9452, 11.35))
        RadiusArc((-161.9452, 11.35), (-159.9452, 9.35), 2.0)
        Line((-159.9452, 9.35), (-159.9452, 4.9))
        Line((-159.9452, 4.9), (-76.0547, 4.9))
        Line((-76.0547, 4.9), (-76.0547, 9.35))
        RadiusArc((-76.0547, 9.35), (-74.0548, 11.35), 2.0)
        Line((-74.0548, 11.35), (-62.9, 11.35))
    _inc_edges_sk_Sketch5_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_3 = Wire.combine(_inc_edges_sk_Sketch5_3)[0]
_wire_sk_Sketch5_3 = _wire_sk_Sketch5_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch5_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch5_3.wrapped, True)
_face_sk_Sketch5_3 = Face(_mkf_sk_Sketch5_3.Face())

# 'Sketch6': circle on inclined plane
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch6_4:
    with Locations((-162.9999, 58.0)):
        Circle(radius=2.35)

# 'Sketch6': circle on inclined plane
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch6_5:
    with Locations((-73.0, 58.0)):
        Circle(radius=2.35)

# 'Sketch6': circle on inclined plane
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch6_6:
    with Locations((-73.0, 7.0)):
        Circle(radius=2.35)

# 'Sketch6': circle on inclined plane
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 0.0, 3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch6_7:
    with Locations((-163.0, 7.0)):
        Circle(radius=2.35)

# 'Sketch7': 20 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 0.0, 3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch7_8:
    with BuildLine():
        Line((-109.95, 43.55), (-107.95, 43.55))
        Line((-107.95, 43.55), (-107.95, 48.55))
        RadiusArc((-107.95, 48.55), (-114.95, 55.55), -7.0)
        Line((-114.95, 55.55), (-147.25, 55.55))
        RadiusArc((-147.25, 55.55), (-154.25, 48.55), -7.0)
        Line((-154.25, 48.55), (-154.25, 16.25))
        RadiusArc((-154.25, 16.25), (-147.25, 9.25), -7.0)
        Line((-147.25, 9.25), (-114.95, 9.25))
        RadiusArc((-114.95, 9.25), (-107.95, 16.25), -7.0)
        Line((-107.95, 16.25), (-107.95, 21.25))
        Line((-107.95, 21.25), (-109.95, 21.25))
        Line((-109.95, 21.25), (-109.95, 16.25))
        RadiusArc((-109.95, 16.25), (-114.95, 11.25), 5.0)
        Line((-114.95, 11.25), (-147.25, 11.25))
        RadiusArc((-147.25, 11.25), (-152.25, 16.25), 5.0)
        Line((-152.25, 16.25), (-152.25, 48.55))
        RadiusArc((-152.25, 48.55), (-147.25, 53.55), 5.0)
        Line((-147.25, 53.55), (-114.95, 53.55))
        RadiusArc((-114.95, 53.55), (-109.95, 48.55), 5.0)
        Line((-109.95, 48.55), (-109.95, 43.55))
    _inc_edges_sk_Sketch7_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_8 = Wire.combine(_inc_edges_sk_Sketch7_8)[0]
_wire_sk_Sketch7_8 = _wire_sk_Sketch7_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch7_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch7_8.wrapped, True)
_face_sk_Sketch7_8 = Face(_mkf_sk_Sketch7_8.Face())

# Path wire for Sweep1
with BuildLine() as _bl_Sweep1:
    Line((-167.9, 2.1, 3.0), (-71.0, 2.1, 3.0))
    ThreePointArc((-71.0, 2.1, 3.0), (-61.8783, 5.8783, 3.0), (-58.1, 15.0, 3.0))
    Line((-58.1, 15.0, 3.0), (-58.1, 50.0, 3.0))
    ThreePointArc((-58.1, 50.0, 3.0), (-61.8783, 59.1217, 3.0), (-71.0, 62.9, 3.0))
    Line((-71.0, 62.9, 3.0), (-167.9, 62.9, 3.0))
path_Sweep1 = _bl_Sweep1.wires()[0]

# Profile plane from sketch (origin at sketch_origin)
_plane_Sweep1 = Plane(origin=Vector(-167.9, 0.0, 0.0), x_dir=Vector(0.0, -1.0, 0.0), z_dir=Vector(-1.0, 0.0, 0.0))

# 'Sketch9': 6 segments -> sweep profile
with BuildSketch(_plane_Sweep1) as sk_Sketch9_8:
    with BuildLine():
        Line((-1.8135, 3.4679), (-1.8135, 2.2))
        Line((-1.8135, 2.2), (-2.1, 2.2))
        RadiusArc((-2.1, 2.2), (-2.2816, 2.7075), -0.8)
        RadiusArc((-2.2816, 2.7075), (-2.9, 3.0), -0.8)
        Line((-2.9, 3.0), (-2.9, 3.4679))
        Line((-2.9, 3.4679), (-1.8135, 3.4679))
    make_face()
# 'Sketch11': 7 segments → Line/RadiusArc profile
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch11_10:
    with BuildLine():
        Line((167.1, -3.0), (167.1, -3.4819))
        Line((167.1, -3.4819), (168.3494, -3.4819))
        Line((168.3494, -3.4819), (168.3494, -2.2784))
        Line((168.3494, -2.2784), (167.8962, -2.2784))
        RadiusArc((167.8962, -2.2784), (167.7184, -2.7075), 0.8)
        RadiusArc((167.7184, -2.7075), (167.4061, -2.9391), 0.7999)
        RadiusArc((167.4061, -2.9391), (167.1, -3.0), 0.8)
    _inc_edges_sk_Sketch11_10 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_10 = Wire.combine(_inc_edges_sk_Sketch11_10)[0]
_wire_sk_Sketch11_10 = _wire_sk_Sketch11_10.moved(_inclined_plane_10.location)
_mkf_sk_Sketch11_10 = BRepBuilderAPI_MakeFace(_inclined_plane_10.wrapped, _wire_sk_Sketch11_10.wrapped, True)
_face_sk_Sketch11_10 = Face(_mkf_sk_Sketch11_10.Face())

# 'Sketch12': circle on inclined plane
_inclined_plane_11 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_11) as sk_Sketch12_11:
    with Locations((73.0, 58.0)):
        Circle(radius=2.35)

# 'Sketch12': circle on inclined plane
_inclined_plane_12 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_12) as sk_Sketch12_12:
    with Locations((163.0, 7.0)):
        Circle(radius=2.35)

# 'Sketch12': circle on inclined plane
_inclined_plane_13 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_13) as sk_Sketch12_13:
    with Locations((73.0, 7.0)):
        Circle(radius=2.35)

# 'Sketch13': circle on inclined plane
_inclined_plane_14 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_14) as sk_Sketch13_14:
    with Locations((162.9999, 58.0)):
        Circle(radius=2.35)

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -10.000000373 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch4_2
    _vec = Vector(0.0, 0.0, 1.0) * 2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 2.000000104 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch5_3
    _vec = Vector(0.0, 0.0, 1.0) * 30.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 30.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    extrude(sk_Sketch6_4.sketch, amount=-50.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -50.000000 mm
    
    # -- Extrude4_p1 --
    extrude(sk_Sketch6_5.sketch, amount=-50.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -50.000000 mm
    
    # -- Extrude4_p2 --
    extrude(sk_Sketch6_6.sketch, amount=-50.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -50.000000 mm
    
    # -- Extrude4_p3 --
    extrude(sk_Sketch6_7.sketch, amount=-50.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -50.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch7_8
    _vec = Vector(0.0, 0.0, 1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Fillet1 ---
    # -- Fillet1 --
    # Fillet radius=1.0mm  (expr: 0.99999879 mm)  |  9 edge(s)
    # Edge indices exported from Fusion — valid for current body state.
    # If features were added before this fillet, re-run the edge diagnostic
    # below and update the indices.
    # edge 0 vertices: [-109.95, 43.55, 3.0] → [-109.95, 48.55, 3.0]
    # edge 1 vertices: [-109.95, 48.55, 3.0] → [-114.95, 53.55, 3.0]
    # edge 2 vertices: [-114.95, 53.55, 3.0] → [-147.25, 53.55, 3.0]
    # edge 3 vertices: [-147.25, 53.55, 3.0] → [-152.25, 48.55, 3.0]
    # edge 4 vertices: [-152.25, 48.55, 3.0] → [-152.25, 16.25, 3.0]
    # edge 5 vertices: [-152.25, 16.25, 3.0] → [-147.25, 11.25, 3.0]
    # edge 6 vertices: [-147.25, 11.25, 3.0] → [-114.95, 11.25, 3.0]
    # edge 7 vertices: [-114.95, 11.25, 3.0] → [-109.95, 16.25, 3.0]
    # edge 8 vertices: [-109.95, 16.25, 3.0] → [-109.95, 21.25, 3.0]
    try:
        # OCP-confirmed indices: [146, 150, 154, 158, 161]
        # OCP-confirmed indices: [146, 150, 154, 158, 161]
        fillet([part.edges()[146], part.edges()[150], part.edges()[154], part.edges()[158], part.edges()[161]], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet1 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    
    # --- FEATURE: Sweep1 ---
    # -- Sweep1 --
    try:
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeFace
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE, TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.gp import gp_Pln, gp_Ax3, gp_Dir, gp_Pnt
        import numpy as _np
        _profile_face = sk_Sketch9_8.sketch.faces()[0]
        _occ_wire = None
        _wire_exp = TopExp_Explorer(_profile_face.wrapped, TopAbs_WIRE)
        if _wire_exp.More():
            _occ_wire = TopoDS.Wire_s(_wire_exp.Current())
        _path_wire = path_Sweep1
        def _make_pipe_solid(_wire, reverse=False):
            _w = _wire.Reversed() if reverse else _wire
            _pipe = BRepOffsetAPI_MakePipeShell(_path_wire.wrapped)
            _pipe.Add(_w)
            _pipe.Build()
            if not _pipe.IsDone(): return None
            if _pipe.MakeSolid(): return Solid(_pipe.Shape())
            return None
        def _fit_plane_cap(wire):
            _pts = []
            _ee = TopExp_Explorer(wire, TopAbs_EDGE)
            while _ee.More():
                _c = BRepAdaptor_Curve(TopoDS.Edge_s(_ee.Current()))
                _t = (_c.FirstParameter() + _c.LastParameter()) / 2.0
                _p = _c.Value(_t)
                _pts.append([_p.X(), _p.Y(), _p.Z()])
                _ee.Next()
            if len(_pts) < 3: return None
            _pts = _np.array(_pts)
            _cen = _pts.mean(axis=0)
            _, _, _vh = _np.linalg.svd(_pts - _cen)
            _n = _vh[-1]; _n /= _np.linalg.norm(_n)
            _x = _pts[0] - _cen; _x -= _np.dot(_x, _n) * _n
            if _np.linalg.norm(_x) < 1e-6: _x = _pts[1] - _cen; _x -= _np.dot(_x, _n) * _n
            _x /= _np.linalg.norm(_x)
            _ax = gp_Ax3(gp_Pnt(*_cen.tolist()), gp_Dir(*_n.tolist()), gp_Dir(*_x.tolist()))
            _mf = BRepBuilderAPI_MakeFace(gp_Pln(_ax), wire)
            return _mf.Face() if _mf.IsDone() else None
        # Attempt A: wire as-is
        _solid = _make_pipe_solid(_occ_wire) if _occ_wire else None
        if _solid is None and _occ_wire:
            # Attempt B: reversed wire
            _solid = _make_pipe_solid(_occ_wire, reverse=True)
        if _solid is None:
            # Attempt C: Solid.sweep() + cap free boundary wires
            _sweep_shell = Solid.sweep(sk_Sketch9_8.sketch.faces()[0], path_Sweep1)
            _sa = ShapeAnalysis_FreeBounds(_sweep_shell.wrapped)
            _cw_exp = TopExp_Explorer(_sa.GetClosedWires(), TopAbs_WIRE)
            _caps = []
            while _cw_exp.More():
                _w = TopoDS.Wire_s(_cw_exp.Current())
                _mf = BRepBuilderAPI_MakeFace(_w, True)
                if _mf.IsDone(): _caps.append(_mf.Face())
                else:
                    _fc = _fit_plane_cap(_w)
                    if _fc is not None: _caps.append(_fc)
                _cw_exp.Next()
            _sew = BRepBuilderAPI_Sewing(0.1)
            _sew.Add(_sweep_shell.wrapped)
            for _fc in _caps: _sew.Add(_fc)
            _sew.Perform()
            _mk = BRepBuilderAPI_MakeSolid()
            _exp = TopExp_Explorer(_sew.SewedShape(), TopAbs_SHELL)
            while _exp.More(): _mk.Add(TopoDS.Shell_s(_exp.Current())); _exp.Next()
            _mk.Build()
            if _mk.IsDone():
                _fix = ShapeFix_Solid(_mk.Solid())
                _fix.Perform()
                _solid = Solid(_fix.Shape())
            else:
                _solid = _sweep_shell
                print('WARNING: Sweep1 sweep — all solid attempts failed, result is Shell')
        # v17.95: final Shell→Solid coercion — add() rejects Shell in empty BuildPart
        from OCP.TopAbs import TopAbs_SHELL as _TS_SHELL, TopAbs_SOLID as _TS_SOLID
        if hasattr(_solid, 'wrapped') and _solid.wrapped.ShapeType() != _TS_SOLID:
            try:
                from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid as _MkSol2
                _mk2 = _MkSol2()
                _exp2 = TopExp_Explorer(_solid.wrapped, _TS_SHELL)
                while _exp2.More(): _mk2.Add(TopoDS.Shell_s(_exp2.Current())); _exp2.Next()
                _mk2.Build()
                if _mk2.IsDone(): _solid = Solid(_mk2.Shape())
            except Exception as _coerce_err:
                print('WARNING: Sweep1 Shell→Solid coercion failed:', _coerce_err)
        add(_solid, mode=Mode.SUBTRACT)
    except Exception as _sweep_err:
        print('WARNING: Sweep1 sweep failed:', _sweep_err)
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch11_10
    _vec = Vector(-0.0, -1.0, -0.0) * -70.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -70.000000 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7_p0 --
    extrude(sk_Sketch12_11.sketch, amount=2.0, taper=-45.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000477 mm
    # Fusion taper angle expression: 45.00000 deg
    
    # -- Extrude7_p1 --
    extrude(sk_Sketch12_12.sketch, amount=2.0, taper=-45.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000477 mm
    # Fusion taper angle expression: 45.00000 deg
    
    # -- Extrude7_p2 --
    extrude(sk_Sketch12_13.sketch, amount=2.0, taper=-45.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000477 mm
    # Fusion taper angle expression: 45.00000 deg
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    extrude(sk_Sketch13_14.sketch, amount=2.0, taper=-45.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000477 mm
    # Fusion taper angle expression: 45 deg
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)

# -- Edge coordinate diagnostic --
# Uncomment the block below to find edge coordinates for fillet/chamfer.
# Set tx, ty, tz to a point near the edge you're looking for.
# Run the script — matching edges will print their vertex coordinates.
#
# tx, ty, tz = 0, 0, 0  # <-- set target coordinates here
# for e in part.part.edges():
#     verts = e.vertices()
#     for v in verts:
#         if abs(v.X - tx) < 1 and abs(v.Y - ty) < 1 and abs(v.Z - tz) < 1:
#             print([(round(v2.X,3), round(v2.Y,3), round(v2.Z,3)) for v2 in verts])
#             break
