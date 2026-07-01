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

# 'Sketch1': 8 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -14.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        RadiusArc((-0.5, -1.5), (1.5, 0.5), -2.0)
        Line((1.5, 0.5), (1.5, 51.2377))
        RadiusArc((1.5, 51.2377), (-0.5, 53.2377), -2.0)
        Line((-0.5, 53.2377), (-34.5, 53.2377))
        RadiusArc((-34.5, 53.2377), (-36.5, 51.2377), -2.0)
        Line((-36.5, 51.2377), (-36.5, 0.5))
        RadiusArc((-36.5, 0.5), (-34.5, -1.5), -2.0)
        Line((-34.5, -1.5), (-0.5, -1.5))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 4 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, -10.4),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((23.65, 39.57), (23.65, 51.77))
        Line((23.65, 51.77), (14.55, 51.77))
        Line((14.55, 51.77), (14.55, 39.57))
        Line((14.55, 39.57), (23.65, 39.57))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch3': circle on inclined plane
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, -3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with Locations((26.67, 44.831)):
        Circle(radius=2.0)

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, -13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        # Arc split: sweep=271.95deg >= 150 — emitted as two half-arcs
        RadiusArc((28.8244, 38.665), (26.67, 43.994), -3.1)
        RadiusArc((26.67, 43.994), (24.5156, 38.665), -3.1)
        Line((24.5156, 38.665), (28.8244, 38.665))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch5': 8 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, -11.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with BuildLine():
        Line((21.705, 38.665), (21.705, 35.665))
        Line((21.705, 35.665), (26.205, 35.665))
        Line((26.205, 35.665), (26.205, 21.665))
        Line((26.205, 21.665), (29.205, 21.665))
        Line((29.205, 21.665), (29.205, 38.665))
        Line((29.205, 38.665), (28.8244, 38.665))
        Line((28.8244, 38.665), (24.5156, 38.665))
        Line((24.5156, 38.665), (21.705, 38.665))
    _inc_edges_sk_Sketch5_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_5 = Wire.combine(_inc_edges_sk_Sketch5_5)[0]
_wire_sk_Sketch5_5 = _wire_sk_Sketch5_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch5_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch5_5.wrapped, True)
_face_sk_Sketch5_5 = Face(_mkf_sk_Sketch5_5.Face())

# 'Sketch6': 6 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch6_6:
    with BuildLine():
        Line((8.095, 51.09), (8.095, 42.99))
        Line((8.095, 42.99), (8.4885, 42.99))
        # Spline from NurbsCurve3D, 15 adaptive samples
        Spline((8.4885, 42.99), (8.7652, 43.3051), (9.0789, 43.5772), (9.4393, 43.8177), (9.8139, 44.0056), (10.2079, 44.1368), (10.6329, 44.221), (11.0508, 44.2511), (11.4651, 44.2214), (11.89, 44.1364), (12.2875, 44.0042), (12.6589, 43.818), (13.0189, 43.5769), (13.3353, 43.3025), (13.6094, 42.99))
        Line((13.6094, 42.99), (14.55, 42.99))
        Line((14.55, 42.99), (14.55, 51.09))
        Line((14.55, 51.09), (8.095, 51.09))
    _inc_edges_sk_Sketch6_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_6 = Wire.combine(_inc_edges_sk_Sketch6_6)[0]
_wire_sk_Sketch6_6 = _wire_sk_Sketch6_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch6_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch6_6.wrapped, True)
_face_sk_Sketch6_6 = Face(_mkf_sk_Sketch6_6.Face())

# 'Sketch7': 5 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 0.0, -13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch7_7:
    with BuildLine():
        # Spline from NurbsCurve3D, 57 adaptive samples
        Spline((13.895, 42.552), (13.7427, 42.801), (13.5718, 43.0372), (13.3884, 43.2504), (13.1891, 43.4407), (12.9632, 43.6204), (12.7199, 43.7834), (12.469, 43.9229), (12.218, 44.0334), (11.9486, 44.1221), (11.6643, 44.1897), (11.3768, 44.2339), (11.0978, 44.2522), (10.8211, 44.243), (10.5337, 44.2076), (10.247, 44.1484), (9.9729, 44.0675), (9.7186, 43.9655), (9.4654, 43.8347), (9.2176, 43.6792), (8.9852, 43.5055), (8.7781, 43.3203), (8.5899, 43.115), (8.4132, 42.8845), (8.2547, 42.6388), (8.121, 42.3876), (8.016, 42.1345), (7.9316, 41.8609), (7.8687, 41.575), (7.8299, 41.2883), (7.8173, 41.0123), (7.8323, 40.7325), (7.873, 40.4442), (7.937, 40.1593), (8.0222, 39.8893), (8.1298, 39.6368), (8.2664, 39.3839), (8.4264, 39.1385), (8.603, 38.9106), (8.7907, 38.7094), (9.0022, 38.5231), (9.2367, 38.3497), (9.4844, 38.1959), (9.7355, 38.0684), (9.992, 37.9683), (10.2692, 37.8883), (10.5562, 37.8303), (10.8415, 37.7969), (11.1158, 37.7902), (11.3984, 37.8108), (11.687, 37.8566), (11.9697, 37.9254), (12.235, 38.0148), (12.4871, 38.1285), (12.7392, 38.2706), (12.9819, 38.4345), (13.205, 38.6136))
        Line((13.205, 38.6136), (13.205, 38.665))
        Line((13.205, 38.665), (13.2619, 38.665))
        # Spline from NurbsCurve3D, 8 adaptive samples
        Spline((13.2619, 38.665), (13.3716, 38.773), (13.4754, 38.8836), (13.5714, 38.996), (13.6584, 39.1098), (13.741, 39.2311), (13.82, 39.3598), (13.895, 39.4939))
        Line((13.895, 39.4939), (13.895, 42.552))
    _inc_edges_sk_Sketch7_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_7 = Wire.combine(_inc_edges_sk_Sketch7_7)[0]
_wire_sk_Sketch7_7 = _wire_sk_Sketch7_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch7_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch7_7.wrapped, True)
_face_sk_Sketch7_7 = Face(_mkf_sk_Sketch7_7.Face())

# 'Sketch8': 10 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 0.0, -6.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch8_8:
    with BuildLine():
        Line((13.205, 38.6163), (13.205, 17.565))
        Line((13.205, 17.565), (29.205, 17.565))
        Line((29.205, 17.565), (29.205, 21.665))
        Line((29.205, 21.665), (26.205, 21.665))
        Line((26.205, 21.665), (26.205, 35.665))
        Line((26.205, 35.665), (21.705, 35.665))
        Line((21.705, 35.665), (21.705, 38.665))
        Line((21.705, 38.665), (13.251, 38.665))
        Line((13.251, 38.665), (13.205, 38.665))
        Line((13.205, 38.665), (13.205, 38.6163))
    _inc_edges_sk_Sketch8_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_8 = Wire.combine(_inc_edges_sk_Sketch8_8)[0]
_wire_sk_Sketch8_8 = _wire_sk_Sketch8_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch8_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch8_8.wrapped, True)
_face_sk_Sketch8_8 = Face(_mkf_sk_Sketch8_8.Face())

# 'Sketch9': 4 segments → Line/RadiusArc profile
_inclined_plane_9 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch9_9:
    with BuildLine():
        Line((6.425, 36.595), (6.425, 23.095))
        Line((6.425, 23.095), (12.625, 23.095))
        Line((12.625, 23.095), (12.625, 36.595))
        Line((12.625, 36.595), (6.425, 36.595))
    _inc_edges_sk_Sketch9_9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch9_9 = Wire.combine(_inc_edges_sk_Sketch9_9)[0]
_wire_sk_Sketch9_9 = _wire_sk_Sketch9_9.moved(_inclined_plane_9.location)
_mkf_sk_Sketch9_9 = BRepBuilderAPI_MakeFace(_inclined_plane_9.wrapped, _wire_sk_Sketch9_9.wrapped, True)
_face_sk_Sketch9_9 = Face(_mkf_sk_Sketch9_9.Face())

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 0.0, -7.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch10_10:
    with BuildLine():
        Line((6.425, 20.975), (6.425, 10.775))
        Line((6.425, 10.775), (12.625, 10.775))
        Line((12.625, 10.775), (12.625, 20.975))
        Line((12.625, 20.975), (6.425, 20.975))
    _inc_edges_sk_Sketch10_10 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_10 = Wire.combine(_inc_edges_sk_Sketch10_10)[0]
_wire_sk_Sketch10_10 = _wire_sk_Sketch10_10.moved(_inclined_plane_10.location)
_mkf_sk_Sketch10_10 = BRepBuilderAPI_MakeFace(_inclined_plane_10.wrapped, _wire_sk_Sketch10_10.wrapped, True)
_face_sk_Sketch10_10 = Face(_mkf_sk_Sketch10_10.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_11 = Plane(
    origin=Vector(0.0, 0.0, -3.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_11) as sk_Sketch11_11:
    with BuildLine():
        Line((22.88, 15.73), (22.88, 7.13))
        Line((22.88, 7.13), (25.38, 7.13))
        Line((25.38, 7.13), (25.38, 15.73))
        Line((25.38, 15.73), (22.88, 15.73))
    _inc_edges_sk_Sketch11_11 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_11 = Wire.combine(_inc_edges_sk_Sketch11_11)[0]
_wire_sk_Sketch11_11 = _wire_sk_Sketch11_11.moved(_inclined_plane_11.location)
_mkf_sk_Sketch11_11 = BRepBuilderAPI_MakeFace(_inclined_plane_11.wrapped, _wire_sk_Sketch11_11.wrapped, True)
_face_sk_Sketch11_11 = Face(_mkf_sk_Sketch11_11.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -14.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -13.999999762 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * 14.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 14.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    extrude(sk_Sketch3_3.sketch, amount=8.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 8.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(0.0, 0.0, 1.0) * 51.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 51.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_5
    _vec = Vector(0.0, 0.0, 1.0) * 18.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 18.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch6_6
    _vec = Vector(0.0, 0.0, 1.0) * 13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 13.000000 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch7_7
    _vec = Vector(0.0, 0.0, 1.0) * 16.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 16.000000 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    _face = _face_sk_Sketch8_8
    _vec = Vector(0.0, 0.0, 1.0) * 20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 20.000000 mm
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch9_9
    _vec = Vector(0.0, 0.0, 1.0) * 19.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 19.000000 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10 --
    _face = _face_sk_Sketch10_10
    _vec = Vector(0.0, 0.0, 1.0) * 17.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 17.000000 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch11_11
    _vec = Vector(0.0, 0.0, 1.0) * 31.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 31.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
