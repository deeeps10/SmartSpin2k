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

# All dimensions below are raw numbers.

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(1900.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch3:
    with BuildLine():
        Line((114.0, 214.0), (138.0, 214.0))
        RadiusArc((138.0, 214.0), (114.0, 238.0), -24.0)
        Line((114.0, 238.0), (114.0, 214.0))
    _inc_edges_sk_Sketch3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3 = Wire.combine(_inc_edges_sk_Sketch3)[0]
_wire_sk_Sketch3 = _wire_sk_Sketch3.moved(_inclined_plane_1.location)
_mkf_sk_Sketch3 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch3.wrapped, True)
_face_sk_Sketch3 = Face(_mkf_sk_Sketch3.Face())

# 'Sketch7': 6 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(1496.1699, 88.0, 81.3902),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(0.707107, -0.0, 0.707107),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch7_2:
    with BuildLine():
        RadiusArc((50.0, -49.9999), (43.3013, -24.9999), -50.0004)
        RadiusArc((43.3013, -24.9999), (25.0, -6.6987), -49.9994)
        RadiusArc((25.0, -6.6987), (0.0, 0.0), -50.0002)
        Line((0.0, 0.0), (-88.0, 0.0))
        Line((-88.0, 0.0), (-88.0, -49.9999))
        Line((-88.0, -49.9999), (50.0, -49.9999))
    _inc_edges_sk_Sketch7_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_2 = Wire.combine(_inc_edges_sk_Sketch7_2)[0]
_wire_sk_Sketch7_2 = _wire_sk_Sketch7_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch7_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch7_2.wrapped, True)
_face_sk_Sketch7_2 = Face(_mkf_sk_Sketch7_2.Face())

# 'Sketch8': 8 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, -200.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch8_3:
    with BuildLine():
        Line((1670.3246, -223.2103), (1646.5675, -230.7125))
        Line((1646.5675, -230.7125), (1638.7925, -233.167))
        Line((1638.7925, -233.167), (1638.7925, -253.7186))
        Line((1638.7925, -253.7186), (1699.4904, -253.7186))
        Line((1699.4904, -253.7186), (1699.4904, -214.0))
        Line((1699.4904, -214.0), (1690.1614, -216.946))
        Line((1690.1614, -216.946), (1672.6271, -222.4831))
        Line((1672.6271, -222.4831), (1670.3246, -223.2103))
    _inc_edges_sk_Sketch8_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_3 = Wire.combine(_inc_edges_sk_Sketch8_3)[0]
_wire_sk_Sketch8_3 = _wire_sk_Sketch8_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch8_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch8_3.wrapped, True)
_face_sk_Sketch8_3 = Face(_mkf_sk_Sketch8_3.Face())

# Path wire for Sweep1
with BuildLine() as _bl_Sweep1:
    ThreePointArc((1699.4904, 114.0, 238.0), (1687.5852, 112.5748, 238.0), (1676.4069, 108.238, 238.0))
    ThreePointArc((1676.4069, 108.238, 238.0), (1670.2526, 103.743, 238.0), (1665.4483, 97.8271, 238.0))
    ThreePointArc((1665.4483, 97.8271, 238.0), (1663.4404, 93.0923, 238.0), (1662.7208, 88.0, 238.0))
path_Sweep1 = _bl_Sweep1.wires()[0]

# Profile plane from sketch (origin at sketch_origin)
_plane_Sweep1 = Plane(origin=Vector(1699.4904, 0.0, 0.0), x_dir=Vector(0.0, -1.0, 0.0), z_dir=Vector(-1.0, 0.0, 0.0))

# 'Sketch4': 3 segments -> sweep profile
with BuildSketch(_plane_Sweep1) as sk_Sketch4_3:
    with BuildLine():
        Line((-114.0, 214.0), (-114.0, 238.0))
        RadiusArc((-114.0, 238.0), (-138.0, 214.0), -24.0)
        Line((-138.0, 214.0), (-114.0, 214.0))
    make_face()
# 'Sketch6': 4 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch6_5:
    with BuildLine():
        Line((-1662.7208, -225.6114), (-1662.7208, -238.0))
        RadiusArc((-1662.7208, -238.0), (-1645.7503, -230.9706), -23.9999)
        Line((-1645.7503, -230.9706), (-1646.5675, -230.7125))
        Line((-1646.5675, -230.7125), (-1662.7208, -225.6114))
    _inc_edges_sk_Sketch6_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_5 = Wire.combine(_inc_edges_sk_Sketch6_5)[0]
_wire_sk_Sketch6_5 = _wire_sk_Sketch6_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch6_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch6_5.wrapped, True)
_face_sk_Sketch6_5 = Face(_mkf_sk_Sketch6_5.Face())

# 'Sketch9': 7 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 238.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-0.0, -0.0, -1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch9_6:
    with BuildLine():
        Line((-114.0, -1900.0), (-0.0, -1900.0))
        Line((-0.0, -1900.0), (-0.0, -1662.7208))
        Line((-0.0, -1662.7208), (-88.0, -1662.7208))
        RadiusArc((-88.0, -1662.7208), (-97.8271, -1665.4483), -19.7814)
        RadiusArc((-97.8271, -1665.4483), (-108.238, -1676.4069), -29.6329)
        RadiusArc((-108.238, -1676.4069), (-114.0, -1699.4904), -47.9069)
        Line((-114.0, -1699.4904), (-114.0, -1900.0))
    _inc_edges_sk_Sketch9_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch9_6 = Wire.combine(_inc_edges_sk_Sketch9_6)[0]
_wire_sk_Sketch9_6 = _wire_sk_Sketch9_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch9_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch9_6.wrapped, True)
_face_sk_Sketch9_6 = Face(_mkf_sk_Sketch9_6.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, -0.0, 0.0),
    z_dir=Vector(0.0, -1.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch11_7:
    with BuildLine():
        Line((-1436.186, 108.0373), (-1436.186, -232.0067))
        Line((-1436.186, -232.0067), (-1513.8409, -232.0067))
        Line((-1513.8409, -232.0067), (-1654.6916, 108.0373))
        Line((-1654.6916, 108.0373), (-1436.186, 108.0373))
    _inc_edges_sk_Sketch11_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_7 = Wire.combine(_inc_edges_sk_Sketch11_7)[0]
_wire_sk_Sketch11_7 = _wire_sk_Sketch11_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch11_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch11_7.wrapped, True)
_face_sk_Sketch11_7 = Face(_mkf_sk_Sketch11_7.Face())

# 'Sketch12': 6 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch12_8:
    with BuildLine():
        Line((-1573.4903, -88.0), (-1609.9411, 0.0))
        Line((-1609.9411, 0.0), (-1709.9411, -100.0))
        Line((-1709.9411, -100.0), (-1900.0, -100.0))
        Line((-1900.0, -100.0), (-1900.0, -214.0))
        Line((-1900.0, -214.0), (-1699.4903, -214.0))
        Line((-1699.4903, -214.0), (-1573.4903, -88.0))
    _inc_edges_sk_Sketch12_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch12_8 = Wire.combine(_inc_edges_sk_Sketch12_8)[0]
_wire_sk_Sketch12_8 = _wire_sk_Sketch12_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch12_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch12_8.wrapped, True)
_face_sk_Sketch12_8 = Face(_mkf_sk_Sketch12_8.Face())

# 'Sketch13': 7 segments → Line/RadiusArc profile
_inclined_plane_9 = Plane(
    origin=Vector(99.9999, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch13_9:
    with BuildLine():
        RadiusArc((138.0, 88.0), (88.0, 138.0), -50.0)
        Line((88.0, 138.0), (0.0, 138.0))
        Line((0.0, 138.0), (0.0, 8.8873))
        Line((0.0, 8.8873), (0.0, 0.0))
        Line((0.0, 0.0), (133.5768, 0.0))
        Line((133.5768, 0.0), (138.0, 0.0))
        Line((138.0, 0.0), (138.0, 88.0))
    _inc_edges_sk_Sketch13_9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_9 = Wire.combine(_inc_edges_sk_Sketch13_9)[0]
_wire_sk_Sketch13_9 = _wire_sk_Sketch13_9.moved(_inclined_plane_9.location)
_mkf_sk_Sketch13_9 = BRepBuilderAPI_MakeFace(_inclined_plane_9.wrapped, _wire_sk_Sketch13_9.wrapped, True)
_face_sk_Sketch13_9 = Face(_mkf_sk_Sketch13_9.Face())

# 'Sketch14': 5 segments → Line/RadiusArc profile
_inclined_plane_10 = Plane(
    origin=Vector(99.9999, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch14_10:
    with BuildLine():
        Line((0.0, 138.0), (-88.0, 138.0))
        RadiusArc((-88.0, 138.0), (-138.0, 88.0), -50.0)
        Line((-138.0, 88.0), (-138.0, 0.0))
        Line((-138.0, 0.0), (0.0, 0.0))
        Line((0.0, 0.0), (0.0, 138.0))
    _inc_edges_sk_Sketch14_10 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch14_10 = Wire.combine(_inc_edges_sk_Sketch14_10)[0]
_wire_sk_Sketch14_10 = _wire_sk_Sketch14_10.moved(_inclined_plane_10.location)
_mkf_sk_Sketch14_10 = BRepBuilderAPI_MakeFace(_inclined_plane_10.wrapped, _wire_sk_Sketch14_10.wrapped, True)
_face_sk_Sketch14_10 = Face(_mkf_sk_Sketch14_10.Face())

# Path wire for Sweep2
with BuildLine() as _bl_Sweep2:
    ThreePointArc((99.9999, 88.0, 138.0), (29.2893, 88.0, 108.7106), (0.0, 88.0, 38.0))
    Line((0.0, 88.0, 38.0), (0.0, 88.0, 0.0))
path_Sweep2 = _bl_Sweep2.wires()[0]

# Profile plane from sketch (origin at sketch_origin)
_plane_Sweep2 = Plane(origin=Vector(99.9999, 0.0, 0.0), x_dir=Vector(0.0, -1.0, 0.0), z_dir=Vector(-1.0, 0.0, 0.0))

# 'Sketch15': 7 segments -> sweep profile
with BuildSketch(_plane_Sweep2) as sk_Sketch15_10:
    with BuildLine():
        Line((-88.0, 138.0), (15.4242, 138.0001))
        Line((15.4242, 138.0001), (15.4241, 217.7556))
        Line((15.4241, 217.7556), (-246.5496, 217.7554))
        Line((-246.5496, 217.7554), (-246.5495, 63.7472))
        Line((-246.5495, 63.7472), (-138.0, 63.7473))
        Line((-138.0, 63.7473), (-138.0, 88.0))
        RadiusArc((-138.0, 88.0), (-88.0, 138.0), 50.0)
    make_face()
# 'Sketch18': 4 segments → Line/RadiusArc profile
_inclined_plane_12 = Plane(
    origin=Vector(1900.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_12) as sk_Sketch18_12:
    with BuildLine():
        Line((124.0, 114.0), (14.0, 114.0))
        Line((14.0, 114.0), (14.0, 224.0))
        Line((14.0, 224.0), (124.0, 224.0))
        Line((124.0, 224.0), (124.0, 114.0))
    _inc_edges_sk_Sketch18_12 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18_12 = Wire.combine(_inc_edges_sk_Sketch18_12)[0]
_wire_sk_Sketch18_12 = _wire_sk_Sketch18_12.moved(_inclined_plane_12.location)
_mkf_sk_Sketch18_12 = BRepBuilderAPI_MakeFace(_inclined_plane_12.wrapped, _wire_sk_Sketch18_12.wrapped, True)
_face_sk_Sketch18_12 = Face(_mkf_sk_Sketch18_12.Face())

_solid_sk_Sketch18_12 = extrude(_face_sk_Sketch18_12, amount=-10.0, dir=Vector(1.0, 0.0, 0.0), taper=45.0).solid()

# 'Sketch20': 11 segments → Line/RadiusArc profile
_inclined_plane_13 = Plane(
    origin=Vector(0.0, 24.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_13) as sk_Sketch20_13:
    with BuildLine():
        Line((1900.0, -224.0), (1902.4109, -224.0))
        Line((1902.4109, -224.0), (1902.4109, -114.0))
        Line((1902.4109, -114.0), (1900.0, -114.0))
        Line((1900.0, -114.0), (1899.0, -115.0))
        Line((1899.0, -115.0), (1890.0, -124.0))
        Line((1890.0, -124.0), (1700.0, -124.0))
        Line((1700.0, -124.0), (1690.0, -114.0))
        Line((1690.0, -114.0), (1562.7208, -114.0))
        Line((1562.7208, -114.0), (1662.7208, -214.0))
        Line((1662.7208, -214.0), (1890.0, -214.0))
        Line((1890.0, -214.0), (1900.0, -224.0))
    _inc_edges_sk_Sketch20_13 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_13 = Wire.combine(_inc_edges_sk_Sketch20_13)[0]
_wire_sk_Sketch20_13 = _wire_sk_Sketch20_13.moved(_inclined_plane_13.location)
_mkf_sk_Sketch20_13 = BRepBuilderAPI_MakeFace(_inclined_plane_13.wrapped, _wire_sk_Sketch20_13.wrapped, True)
_face_sk_Sketch20_13 = Face(_mkf_sk_Sketch20_13.Face())

# 'Sketch21': 4 segments → Line/RadiusArc profile
_inclined_plane_14 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_14) as sk_Sketch21_14:
    with BuildLine():
        Line((-1773.0, -146.5), (-1818.0, -146.5))
        Line((-1818.0, -146.5), (-1818.0, -191.5))
        Line((-1818.0, -191.5), (-1773.0, -191.5))
        Line((-1773.0, -191.5), (-1773.0, -146.5))
    _inc_edges_sk_Sketch21_14 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_14 = Wire.combine(_inc_edges_sk_Sketch21_14)[0]
_wire_sk_Sketch21_14 = _wire_sk_Sketch21_14.moved(_inclined_plane_14.location)
_mkf_sk_Sketch21_14 = BRepBuilderAPI_MakeFace(_inclined_plane_14.wrapped, _wire_sk_Sketch21_14.wrapped, True)
_face_sk_Sketch21_14 = Face(_mkf_sk_Sketch21_14.Face())

# 'Sketch22': 4 segments → Line/RadiusArc profile
_inclined_plane_15 = Plane(
    origin=Vector(0.0, 0.0, 238.0),
    x_dir=Vector(0.0, 1.0, -0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_15) as sk_Sketch22_15:
    with BuildLine():
        Line((46.5, -1773.0), (91.5, -1773.0))
        Line((91.5, -1773.0), (91.5, -1818.0))
        Line((91.5, -1818.0), (46.5, -1818.0))
        Line((46.5, -1818.0), (46.5, -1773.0))
    _inc_edges_sk_Sketch22_15 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch22_15 = Wire.combine(_inc_edges_sk_Sketch22_15)[0]
_wire_sk_Sketch22_15 = _wire_sk_Sketch22_15.moved(_inclined_plane_15.location)
_mkf_sk_Sketch22_15 = BRepBuilderAPI_MakeFace(_inclined_plane_15.wrapped, _wire_sk_Sketch22_15.wrapped, True)
_face_sk_Sketch22_15 = Face(_mkf_sk_Sketch22_15.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch3
    _vec = Vector(1.0, 0.0, 0.0) * -200.5096
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -200.509643555 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch7_2
    _vec = Vector(0.707107, -0.0, 0.707107) * 237.5385
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 237.5385478136 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch8_3
    _vec = Vector(-0.0, 1.0, 0.0) * 2300.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2300.000000 mm
    
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
        _profile_face = sk_Sketch4_3.sketch.faces()[0]
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
            _sweep_shell = Solid.sweep(sk_Sketch4_3.sketch.faces()[0], path_Sweep1)
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
        add(_solid, mode=Mode.ADD)
    except Exception as _sweep_err:
        print('WARNING: Sweep1 sweep failed:', _sweep_err)
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch6_5
    _vec = Vector(-0.0, -1.0, -0.0) * -89.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -89.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch9_6
    _vec = Vector(-0.0, -0.0, -1.0) * 24.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 23.9999948925 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch11_7
    _vec = Vector(0.0, -1.0, 0.0) * -175.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -175.000000 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    _face = _face_sk_Sketch12_8
    _vec = Vector(-0.0, -1.0, -0.0) * -138.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -138.000001907 mm
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch13_9
    _vec = Vector(1.0, 0.0, 0.0) * 1509.9412
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 1509.941196442 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10 --
    _face = _face_sk_Sketch14_10
    _vec = Vector(-1.0, 0.0, 0.0) * 99.9999
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 99.999904633 mm
    
    # --- FEATURE: Sweep2 ---
    # -- Sweep2 --
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
        _profile_face = sk_Sketch15_10.sketch.faces()[0]
        _occ_wire = None
        _wire_exp = TopExp_Explorer(_profile_face.wrapped, TopAbs_WIRE)
        if _wire_exp.More():
            _occ_wire = TopoDS.Wire_s(_wire_exp.Current())
        _path_wire = path_Sweep2
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
            _sweep_shell = Solid.sweep(sk_Sketch15_10.sketch.faces()[0], path_Sweep2)
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
                print('WARNING: Sweep2 sweep — all solid attempts failed, result is Shell')
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
                print('WARNING: Sweep2 Shell→Solid coercion failed:', _coerce_err)
        add(_solid, mode=Mode.SUBTRACT)
    except Exception as _sweep_err:
        print('WARNING: Sweep2 sweep failed:', _sweep_err)
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch18_12
    _solid = _solid_sk_Sketch18_12
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -10.000000 mm
    # Fusion taper angle expression: -45 deg
    
    # --- FEATURE: Extrude12 ---
    # -- Extrude12 --
    _face = _face_sk_Sketch20_13
    _vec = Vector(-0.0, 1.0, 0.0) * 90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 89.999995232 mm
    
    # --- FEATURE: Extrude13 ---
    # -- Extrude13 --
    _face = _face_sk_Sketch21_14
    _vec = Vector(-0.0, -1.0, -0.0) * -210.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -210.000000 mm
    
    # --- FEATURE: Extrude14 ---
    # -- Extrude14 --
    _face = _face_sk_Sketch22_15
    _vec = Vector(0.0, 0.0, 1.0) * -190.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -190.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
