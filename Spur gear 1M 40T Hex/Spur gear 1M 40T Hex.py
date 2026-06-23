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

# 'Sketch18': 160 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(11.3, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch18:
    with BuildLine():
        RadiusArc((2.2136, 20.5085), (0.0, 19.617), -7.9998)
        Line((0.0, 19.617), (0.0484, 19.0025))
        RadiusArc((0.0484, 19.0025), (2.3741, 18.4682), -7.9999)
        Line((2.3741, 18.4682), (2.5145, 17.5819))
        RadiusArc((2.5145, 17.5819), (0.4677, 16.355), -8.0)
        Line((0.4677, 16.355), (0.6116, 15.7557))
        RadiusArc((0.6116, 15.7557), (2.9923, 15.5918), -8.0)
        Line((2.9923, 15.5918), (3.2696, 14.7384))
        RadiusArc((3.2696, 14.7384), (1.4399, 13.2064), -8.0)
        Line((1.4399, 13.2064), (1.6757, 12.637))
        RadiusArc((1.6757, 12.637), (4.0528, 12.8475), -8.0001)
        Line((4.0528, 12.8475), (4.4602, 12.048))
        RadiusArc((4.4602, 12.048), (2.8927, 10.2487), -8.0)
        Line((2.8927, 10.2487), (3.2147, 9.7231))
        RadiusArc((3.2147, 9.7231), (5.5295, 10.3029), -8.0)
        Line((5.5295, 10.3029), (6.057, 9.577))
        RadiusArc((6.057, 9.577), (4.7902, 7.5546), -8.0)
        Line((4.7902, 7.5546), (5.1905, 7.0859))
        RadiusArc((5.1905, 7.0859), (7.3862, 8.0207), -8.0)
        Line((7.3862, 8.0207), (8.0207, 7.3862))
        RadiusArc((8.0207, 7.3862), (7.0859, 5.1905), -8.0)
        Line((7.0859, 5.1905), (7.5546, 4.7902))
        RadiusArc((7.5546, 4.7902), (9.577, 6.057), -8.0)
        Line((9.577, 6.057), (10.3029, 5.5295))
        RadiusArc((10.3029, 5.5295), (9.7231, 3.2147), -8.0)
        Line((9.7231, 3.2147), (10.2487, 2.8927))
        RadiusArc((10.2487, 2.8927), (12.048, 4.4602), -8.0)
        Line((12.048, 4.4602), (12.8475, 4.0528))
        RadiusArc((12.8475, 4.0528), (12.637, 1.6757), -8.0001)
        Line((12.637, 1.6757), (13.2064, 1.4399))
        RadiusArc((13.2064, 1.4399), (14.7384, 3.2696), -8.0)
        Line((14.7384, 3.2696), (15.5918, 2.9923))
        RadiusArc((15.5918, 2.9923), (15.7557, 0.6116), -8.0)
        Line((15.7557, 0.6116), (16.355, 0.4677))
        RadiusArc((16.355, 0.4677), (17.5819, 2.5145), -8.0)
        Line((17.5819, 2.5145), (18.4682, 2.3741))
        RadiusArc((18.4682, 2.3741), (19.0025, 0.0484), -7.9999)
        Line((19.0025, 0.0484), (19.617, -0.0))
        RadiusArc((19.617, -0.0), (20.5085, 2.2136), -7.9998)
        Line((20.5085, 2.2136), (21.4059, 2.2136))
        RadiusArc((21.4059, 2.2136), (22.2974, 0.0), -7.9999)
        Line((22.2974, 0.0), (22.9119, 0.0484))
        RadiusArc((22.9119, 0.0484), (23.4462, 2.3741), -7.9999)
        Line((23.4462, 2.3741), (24.3325, 2.5145))
        RadiusArc((24.3325, 2.5145), (25.5593, 0.4677), -8.0001)
        Line((25.5593, 0.4677), (26.1587, 0.6116))
        RadiusArc((26.1587, 0.6116), (26.3226, 2.9923), -7.9999)
        Line((26.3226, 2.9923), (27.176, 3.2696))
        RadiusArc((27.176, 3.2696), (28.708, 1.4399), -8.0)
        Line((28.708, 1.4399), (29.2774, 1.6757))
        RadiusArc((29.2774, 1.6757), (29.0668, 4.0528), -8.0002)
        Line((29.0668, 4.0528), (29.8664, 4.4602))
        RadiusArc((29.8664, 4.4602), (31.6657, 2.8927), -8.0)
        Line((31.6657, 2.8927), (32.1912, 3.2147))
        RadiusArc((32.1912, 3.2147), (31.6114, 5.5295), -7.9999)
        Line((31.6114, 5.5295), (32.3374, 6.057))
        RadiusArc((32.3374, 6.057), (34.3598, 4.7902), -7.9999)
        Line((34.3598, 4.7902), (34.8285, 5.1905))
        RadiusArc((34.8285, 5.1905), (33.8937, 7.3862), -8.0)
        Line((33.8937, 7.3862), (34.5282, 8.0207))
        RadiusArc((34.5282, 8.0207), (36.7238, 7.0859), -8.0)
        Line((36.7238, 7.0859), (37.1241, 7.5546))
        RadiusArc((37.1241, 7.5546), (35.8574, 9.577), -8.0)
        Line((35.8574, 9.577), (36.3848, 10.3029))
        RadiusArc((36.3848, 10.3029), (38.6997, 9.7231), -8.0)
        Line((38.6997, 9.7231), (39.0217, 10.2487))
        RadiusArc((39.0217, 10.2487), (37.4542, 12.048), -8.0)
        Line((37.4542, 12.048), (37.8616, 12.8475))
        RadiusArc((37.8616, 12.8475), (40.2386, 12.637), -8.0001)
        Line((40.2386, 12.637), (40.4745, 13.2064))
        RadiusArc((40.4745, 13.2064), (38.6448, 14.7384), -8.0)
        Line((38.6448, 14.7384), (38.9221, 15.5918))
        RadiusArc((38.9221, 15.5918), (41.3028, 15.7557), -8.0)
        Line((41.3028, 15.7557), (41.4467, 16.355))
        RadiusArc((41.4467, 16.355), (39.3999, 17.5819), -7.9999)
        Line((39.3999, 17.5819), (39.5402, 18.4682))
        RadiusArc((39.5402, 18.4682), (41.866, 19.0025), -8.0)
        Line((41.866, 19.0025), (41.9144, 19.617))
        RadiusArc((41.9144, 19.617), (39.7008, 20.5085), -7.9999)
        Line((39.7008, 20.5085), (39.7008, 21.4059))
        RadiusArc((39.7008, 21.4059), (41.9144, 22.2974), -8.0)
        Line((41.9144, 22.2974), (41.866, 22.9119))
        RadiusArc((41.866, 22.9119), (39.5402, 23.4462), -7.9999)
        Line((39.5402, 23.4462), (39.3999, 24.3325))
        RadiusArc((39.3999, 24.3325), (41.4467, 25.5593), -7.9999)
        Line((41.4467, 25.5593), (41.3028, 26.1587))
        RadiusArc((41.3028, 26.1587), (38.9221, 26.3226), -7.9999)
        Line((38.9221, 26.3226), (38.6448, 27.176))
        RadiusArc((38.6448, 27.176), (40.4745, 28.708), -7.9999)
        Line((40.4745, 28.708), (40.2386, 29.2774))
        RadiusArc((40.2386, 29.2774), (37.8616, 29.0668), -8.0002)
        Line((37.8616, 29.0668), (37.4542, 29.8664))
        RadiusArc((37.4542, 29.8664), (39.0217, 31.6657), -8.0)
        Line((39.0217, 31.6657), (38.6997, 32.1912))
        RadiusArc((38.6997, 32.1912), (36.3848, 31.6114), -7.9999)
        Line((36.3848, 31.6114), (35.8574, 32.3374))
        RadiusArc((35.8574, 32.3374), (37.1241, 34.3598), -7.9999)
        Line((37.1241, 34.3598), (36.7238, 34.8285))
        RadiusArc((36.7238, 34.8285), (34.5282, 33.8937), -7.9999)
        Line((34.5282, 33.8937), (33.8937, 34.5282))
        RadiusArc((33.8937, 34.5282), (34.8285, 36.7238), -7.9999)
        Line((34.8285, 36.7238), (34.3598, 37.1241))
        RadiusArc((34.3598, 37.1241), (32.3374, 35.8574), -7.9999)
        Line((32.3374, 35.8574), (31.6114, 36.3848))
        RadiusArc((31.6114, 36.3848), (32.1912, 38.6997), -7.9999)
        Line((32.1912, 38.6997), (31.6657, 39.0217))
        RadiusArc((31.6657, 39.0217), (29.8664, 37.4542), -8.0)
        Line((29.8664, 37.4542), (29.0668, 37.8616))
        RadiusArc((29.0668, 37.8616), (29.2774, 40.2386), -8.0002)
        Line((29.2774, 40.2386), (28.708, 40.4745))
        RadiusArc((28.708, 40.4745), (27.176, 38.6448), -7.9999)
        Line((27.176, 38.6448), (26.3226, 38.9221))
        RadiusArc((26.3226, 38.9221), (26.1587, 41.3028), -7.9999)
        Line((26.1587, 41.3028), (25.5593, 41.4467))
        RadiusArc((25.5593, 41.4467), (24.3325, 39.3999), -7.9999)
        Line((24.3325, 39.3999), (23.4462, 39.5402))
        RadiusArc((23.4462, 39.5402), (22.9119, 41.866), -7.9999)
        Line((22.9119, 41.866), (22.2974, 41.9144))
        RadiusArc((22.2974, 41.9144), (21.4059, 39.7008), -8.0)
        Line((21.4059, 39.7008), (20.5085, 39.7008))
        RadiusArc((20.5085, 39.7008), (19.617, 41.9144), -7.9999)
        Line((19.617, 41.9144), (19.0025, 41.866))
        RadiusArc((19.0025, 41.866), (18.4682, 39.5402), -8.0)
        Line((18.4682, 39.5402), (17.5819, 39.3999))
        RadiusArc((17.5819, 39.3999), (16.355, 41.4467), -7.9999)
        Line((16.355, 41.4467), (15.7557, 41.3028))
        RadiusArc((15.7557, 41.3028), (15.5918, 38.9221), -8.0)
        Line((15.5918, 38.9221), (14.7384, 38.6448))
        RadiusArc((14.7384, 38.6448), (13.2064, 40.4745), -8.0)
        Line((13.2064, 40.4745), (12.637, 40.2386))
        RadiusArc((12.637, 40.2386), (12.8475, 37.8616), -8.0001)
        Line((12.8475, 37.8616), (12.048, 37.4542))
        RadiusArc((12.048, 37.4542), (10.2487, 39.0217), -8.0)
        Line((10.2487, 39.0217), (9.7231, 38.6997))
        RadiusArc((9.7231, 38.6997), (10.3029, 36.3848), -8.0)
        Line((10.3029, 36.3848), (9.577, 35.8574))
        RadiusArc((9.577, 35.8574), (7.5546, 37.1241), -8.0)
        Line((7.5546, 37.1241), (7.0859, 36.7238))
        RadiusArc((7.0859, 36.7238), (8.0207, 34.5282), -8.0)
        Line((8.0207, 34.5282), (7.3862, 33.8937))
        RadiusArc((7.3862, 33.8937), (5.1905, 34.8285), -8.0)
        Line((5.1905, 34.8285), (4.7902, 34.3598))
        RadiusArc((4.7902, 34.3598), (6.057, 32.3374), -7.9999)
        Line((6.057, 32.3374), (5.5295, 31.6114))
        RadiusArc((5.5295, 31.6114), (3.2147, 32.1912), -7.9999)
        Line((3.2147, 32.1912), (2.8927, 31.6657))
        RadiusArc((2.8927, 31.6657), (4.4602, 29.8664), -8.0)
        Line((4.4602, 29.8664), (4.0528, 29.0668))
        RadiusArc((4.0528, 29.0668), (1.6757, 29.2774), -8.0002)
        Line((1.6757, 29.2774), (1.4399, 28.708))
        RadiusArc((1.4399, 28.708), (3.2696, 27.176), -8.0)
        Line((3.2696, 27.176), (2.9923, 26.3226))
        RadiusArc((2.9923, 26.3226), (0.6116, 26.1587), -7.9999)
        Line((0.6116, 26.1587), (0.4677, 25.5593))
        RadiusArc((0.4677, 25.5593), (2.5145, 24.3325), -8.0001)
        Line((2.5145, 24.3325), (2.3741, 23.4462))
        RadiusArc((2.3741, 23.4462), (0.0484, 22.9119), -7.9999)
        Line((0.0484, 22.9119), (0.0, 22.2974))
        RadiusArc((0.0, 22.2974), (2.2136, 21.4059), -7.9999)
        Line((2.2136, 21.4059), (2.2136, 20.5085))
    _inc_edges_sk_Sketch18 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18 = Wire.combine(_inc_edges_sk_Sketch18)[0]
_wire_sk_Sketch18 = _wire_sk_Sketch18.moved(_inclined_plane_1.location)
_mkf_sk_Sketch18 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch18.wrapped, True)
_face_sk_Sketch18 = Face(_mkf_sk_Sketch18.Face())

# 'Sketch19': circle on inclined plane
_inclined_plane_2 = Plane(
    origin=Vector(2.3, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch19_2:
    with Locations((-20.9572, 20.9572)):
        Circle(radius=6.0)

# 'Sketch20': 14 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(5.8, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch20_3:
    with BuildLine():
        Line((16.9072, 26.9424), (15.4072, 24.3443))
        Line((15.4072, 24.3443), (13.4516, 20.9572))
        Line((13.4516, 20.9572), (15.4072, 17.5701))
        Line((15.4072, 17.5701), (16.9072, 14.972))
        Line((16.9072, 14.972), (17.2044, 14.4572))
        Line((17.2044, 14.4572), (24.71, 14.4572))
        Line((24.71, 14.4572), (25.0072, 14.972))
        Line((25.0072, 14.972), (26.5072, 17.5701))
        Line((26.5072, 17.5701), (28.4627, 20.9572))
        Line((28.4627, 20.9572), (26.5072, 24.3443))
        Line((26.5072, 24.3443), (25.0072, 26.9424))
        Line((25.0072, 26.9424), (24.71, 27.4572))
        Line((24.71, 27.4572), (17.2044, 27.4572))
        Line((17.2044, 27.4572), (16.9072, 26.9424))
    _inc_edges_sk_Sketch20_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_3 = Wire.combine(_inc_edges_sk_Sketch20_3)[0]
_wire_sk_Sketch20_3 = _wire_sk_Sketch20_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch20_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch20_3.wrapped, True)
_face_sk_Sketch20_3 = Face(_mkf_sk_Sketch20_3.Face())

# 'Sketch21': 12 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(6.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch21_4:
    with BuildLine():
        Line((25.0072, 26.9424), (25.0072, 26.5072))
        Line((25.0072, 26.5072), (16.9072, 26.5072))
        Line((16.9072, 26.5072), (16.9072, 26.9424))
        Line((16.9072, 26.9424), (15.4072, 24.3443))
        Line((15.4072, 24.3443), (15.4072, 17.5701))
        Line((15.4072, 17.5701), (16.9072, 14.972))
        Line((16.9072, 14.972), (16.9072, 15.4072))
        Line((16.9072, 15.4072), (25.0072, 15.4072))
        Line((25.0072, 15.4072), (25.0072, 14.972))
        Line((25.0072, 14.972), (26.5072, 17.5701))
        Line((26.5072, 17.5701), (26.5072, 24.3443))
        Line((26.5072, 24.3443), (25.0072, 26.9424))
    _inc_edges_sk_Sketch21_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_4 = Wire.combine(_inc_edges_sk_Sketch21_4)[0]
_wire_sk_Sketch21_4 = _wire_sk_Sketch21_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch21_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch21_4.wrapped, True)
_face_sk_Sketch21_4 = Face(_mkf_sk_Sketch21_4.Face())

# 'Sketch24': 4 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(6.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch24_5:
    with BuildLine():
        Line((25.0072, 16.9072), (16.9072, 16.9072))
        Line((16.9072, 16.9072), (16.9072, 25.0072))
        Line((16.9072, 25.0072), (25.0072, 25.0072))
        Line((25.0072, 25.0072), (25.0072, 16.9072))
    _inc_edges_sk_Sketch24_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch24_5 = Wire.combine(_inc_edges_sk_Sketch24_5)[0]
_wire_sk_Sketch24_5 = _wire_sk_Sketch24_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch24_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch24_5.wrapped, True)
_face_sk_Sketch24_5 = Face(_mkf_sk_Sketch24_5.Face())

# 'Sketch22': circle on inclined plane
_inclined_plane_6 = Plane(
    origin=Vector(2.3, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch22_6:
    with Locations((20.9572, 20.9572)):
        Circle(radius=4.05)

# 'Sketch25': circle on inclined plane
_inclined_plane_7 = Plane(
    origin=Vector(2.3, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch25_7:
    with Locations((20.9572, 20.9572)):
        Circle(radius=4.0)

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude13 ---
    # -- Extrude13 --
    _face = _face_sk_Sketch18
    _vec = Vector(1.0, 0.0, 0.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -8.999999911 mm
    
    # --- FEATURE: Extrude14 ---
    # -- Extrude14 --
    extrude(sk_Sketch19_2.sketch, amount=2.3, mode=Mode.ADD)
    # Fusion depth expression: 2.300000042 mm
    
    # --- FEATURE: Extrude15 ---
    # -- Extrude15 --
    _face = _face_sk_Sketch20_3
    _vec = Vector(1.0, 0.0, 0.0) * 5.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 5.499999523 mm
    
    # --- FEATURE: Extrude16 ---
    # -- Extrude16 --
    _face = _face_sk_Sketch21_4
    _vec = Vector(1.0, 0.0, 0.0) * -0.3
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.299999714 mm
    
    # --- FEATURE: Extrude19 ---
    # -- Extrude19 --
    _face = _face_sk_Sketch24_5
    _vec = Vector(1.0, 0.0, 0.0) * -0.3
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.299999714 mm
    
    # --- FEATURE: Extrude17 ---
    # -- Extrude17 --
    extrude(sk_Sketch22_6.sketch, amount=20.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 20.000000 mm
    
    # --- FEATURE: Extrude20 ---
    # -- Extrude20 --
    extrude(sk_Sketch25_7.sketch, amount=-11.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -11.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
