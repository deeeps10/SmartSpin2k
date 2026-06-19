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

# 'Sketch2': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch2:
    with BuildLine():
        Line((0.6, 41.5034), (0.6, 41.4))
        Line((0.6, 41.4), (-9.0, 41.4))
        Line((-9.0, 41.4), (-9.0, 38.0))
        RadiusArc((-9.0, 38.0), (-6.0, 35.0102), -2.9846)
        Line((-6.0, 35.0102), (-6.0, -50.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -50.0), (-0.0, -56.0), -6.0)
        RadiusArc((-0.0, -56.0), (6.0, -50.0), -6.0)
        Line((6.0, -50.0), (6.0, 35.0102))
        RadiusArc((6.0, 35.0102), (9.0, 38.0), -2.9864)
        Line((9.0, 38.0), (9.0, 50.0))
        Line((9.0, 50.0), (-9.0, 50.0))
        Line((-9.0, 50.0), (-9.0, 46.6))
        Line((-9.0, 46.6), (0.6, 46.6))
        Line((0.6, 46.6), (0.6, 46.4966))
        # Arc split: sweep=212.69deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 46.4966), (3.9341, 44.0), 2.6018)
        RadiusArc((3.9341, 44.0), (0.6, 41.5034), 2.6018)
    _inc_edges_sk_Sketch2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2 = Wire.combine(_inc_edges_sk_Sketch2)[0]
_wire_sk_Sketch2 = _wire_sk_Sketch2.moved(_inclined_plane_1.location)
_mkf_sk_Sketch2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch2.wrapped, True)
_face_sk_Sketch2 = Face(_mkf_sk_Sketch2.Face())

# 'Sketch3': 11 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch3_2:
    with BuildLine():
        Line((-4.3974, -16.119), (3.2681, -16.119))
        Line((3.2681, -16.119), (3.2681, -14.0257))
        Line((3.2681, -14.0257), (4.2019, -14.0257))
        Line((4.2019, -14.0257), (4.2019, -19.4147))
        Line((4.2019, -19.4147), (3.2681, -19.4147))
        Line((3.2681, -19.4147), (3.2681, -17.2237))
        Line((3.2681, -17.2237), (-3.3477, -17.2237))
        Line((-3.3477, -17.2237), (-1.9623, -19.1645))
        Line((-1.9623, -19.1645), (-2.9998, -19.1645))
        Line((-2.9998, -19.1645), (-4.3974, -17.1321))
        Line((-4.3974, -17.1321), (-4.3974, -16.119))
    _inc_edges_sk_Sketch3_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_2 = Wire.combine(_inc_edges_sk_Sketch3_2)[0]
_wire_sk_Sketch3_2 = _wire_sk_Sketch3_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch3_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch3_2.wrapped, True)
_face_sk_Sketch3_2 = Face(_mkf_sk_Sketch3_2.Face())

# 'Sketch3': 22 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        RadiusArc((-4.4462, -9.3269), (-4.4814, -10.6721), -3.7475)
        RadiusArc((-4.4814, -10.6721), (-3.9315, -11.921), -2.3808)
        RadiusArc((-3.9315, -11.921), (-3.1066, -12.5464), -2.7299)
        RadiusArc((-3.1066, -12.5464), (-2.148, -12.8839), -4.6596)
        RadiusArc((-2.148, -12.8839), (-1.2034, -13.0382), -7.5216)
        RadiusArc((-1.2034, -13.0382), (0.1834, -13.0863), -11.93)
        RadiusArc((0.1834, -13.0863), (1.6954, -12.9275), -7.9587)
        RadiusArc((1.6954, -12.9275), (2.8753, -12.5304), -4.4058)
        RadiusArc((2.8753, -12.5304), (3.7167, -11.8954), -2.6602)
        RadiusArc((3.7167, -11.8954), (4.1433, -11.1857), -2.3226)
        RadiusArc((4.1433, -11.1857), (4.3189, -10.3118), -2.9834)
        RadiusArc((4.3189, -10.3118), (4.2785, -9.5531), -3.4072)
        RadiusArc((4.2785, -9.5531), (4.0767, -8.8907), -2.5665)
        RadiusArc((4.0767, -8.8907), (3.4713, -8.09), -2.4113)
        RadiusArc((3.4713, -8.09), (2.6939, -7.601), -3.3155)
        RadiusArc((2.6939, -7.601), (1.4596, -7.2363), -5.4305)
        RadiusArc((1.4596, -7.2363), (0.1819, -7.1181), -9.1444)
        RadiusArc((0.1819, -7.1181), (-1.193, -7.1661), -10.779)
        RadiusArc((-1.193, -7.1661), (-2.3434, -7.3751), -6.4595)
        RadiusArc((-2.3434, -7.3751), (-3.255, -7.7447), -3.8744)
        RadiusArc((-3.255, -7.7447), (-4.0292, -8.4085), -2.3999)
        RadiusArc((-4.0292, -8.4085), (-4.4462, -9.3269), -2.4631)
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch3': 23 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        RadiusArc((-2.5448, -0.6472), (-3.4087, -1.049), -3.8725)
        RadiusArc((-3.4087, -1.049), (-4.0292, -1.6196), -2.4586)
        RadiusArc((-4.0292, -1.6196), (-4.4462, -2.538), -2.3342)
        RadiusArc((-4.4462, -2.538), (-4.5207, -3.4964), -3.3935)
        RadiusArc((-4.5207, -3.4964), (-4.3488, -4.4075), -3.2736)
        RadiusArc((-4.3488, -4.4075), (-3.8186, -5.2546), -2.328)
        RadiusArc((-3.8186, -5.2546), (-3.1066, -5.7575), -2.5589)
        RadiusArc((-3.1066, -5.7575), (-2.148, -6.095), -4.2047)
        RadiusArc((-2.148, -6.095), (-0.9426, -6.2717), -7.2146)
        RadiusArc((-0.9426, -6.2717), (0.4585, -6.2875), -11.7415)
        RadiusArc((0.4585, -6.2875), (1.4665, -6.1816), -8.8049)
        RadiusArc((1.4665, -6.1816), (2.5189, -5.9003), -5.6899)
        RadiusArc((2.5189, -5.9003), (3.4759, -5.3444), -3.4662)
        RadiusArc((3.4759, -5.3444), (4.078, -4.5518), -2.3611)
        RadiusArc((4.078, -4.5518), (4.3189, -3.5229), -2.6613)
        RadiusArc((4.3189, -3.5229), (4.2432, -2.5889), -3.4928)
        RadiusArc((4.2432, -2.5889), (3.8194, -1.6725), -2.47)
        RadiusArc((3.8194, -1.6725), (3.0329, -0.9877), -2.572)
        RadiusArc((3.0329, -0.9877), (1.9073, -0.5419), -4.014)
        RadiusArc((1.9073, -0.5419), (0.9757, -0.3798), -6.5892)
        RadiusArc((0.9757, -0.3798), (-0.1008, -0.3258), -9.7647)
        RadiusArc((-0.1008, -0.3258), (-1.4422, -0.4061), -10.8696)
        RadiusArc((-1.4422, -0.4061), (-2.5448, -0.6472), -6.4614)
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# 'Sketch4': 28 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch4_5:
    with BuildLine():
        # Near-straight arc (sagitta=0.008707mm) replaced with Line
        Line((-2.6824, -1.8174), (-3.0065, -1.9975))
        RadiusArc((-3.0065, -1.9975), (-3.3967, -2.3961), -1.4659)
        RadiusArc((-3.3967, -2.3961), (-3.6011, -2.9187), -1.4249)
        RadiusArc((-3.6011, -2.9187), (-3.6308, -3.4244), -2.0257)
        RadiusArc((-3.6308, -3.4244), (-3.5429, -3.9038), -1.8919)
        RadiusArc((-3.5429, -3.9038), (-3.3379, -4.3021), -1.4459)
        RadiusArc((-3.3379, -4.3021), (-3.0156, -4.6193), -1.4494)
        RadiusArc((-3.0156, -4.6193), (-2.572, -4.8576), -1.93)
        RadiusArc((-2.572, -4.8576), (-1.9816, -5.0306), -3.4875)
        RadiusArc((-1.9816, -5.0306), (-1.2401, -5.1407), -6.3444)
        # Near-straight arc (sagitta=0.009245mm) replaced with Line
        Line((-1.2401, -5.1407), (-0.3476, -5.1879))
        # Near-straight arc (sagitta=0.007961mm) replaced with Line
        Line((-0.3476, -5.1879), (0.5922, -5.1719))
        # Near-straight arc (sagitta=0.009298mm) replaced with Line
        Line((0.5922, -5.1719), (1.394, -5.0922))
        RadiusArc((1.394, -5.0922), (2.0561, -4.9487), -5.2177)
        RadiusArc((2.0561, -4.9487), (2.5785, -4.7413), -2.9852)
        RadiusArc((2.5785, -4.7413), (3.0497, -4.3836), -1.7706)
        RadiusArc((3.0497, -4.3836), (3.3325, -3.9086), -1.3922)
        RadiusArc((3.3325, -3.9086), (3.423, -3.4441), -1.652)
        RadiusArc((3.423, -3.4441), (3.3921, -2.9488), -2.0134)
        RadiusArc((3.3921, -2.9488), (3.1803, -2.4287), -1.4657)
        RadiusArc((3.1803, -2.4287), (2.7759, -2.0238), -1.4788)
        RadiusArc((2.7759, -2.0238), (2.1764, -1.7316), -2.278)
        RadiusArc((2.1764, -1.7316), (1.5492, -1.571), -4.2271)
        RadiusArc((1.5492, -1.571), (0.5798, -1.4606), -7.2757)
        # Near-straight arc (sagitta=0.008603mm) replaced with Line
        Line((0.5798, -1.4606), (-0.3445, -1.4445))
        RadiusArc((-0.3445, -1.4445), (-1.426, -1.5114), -11.3711)
        RadiusArc((-1.426, -1.5114), (-2.1271, -1.6338), -6.2074)
        RadiusArc((-2.1271, -1.6338), (-2.6824, -1.8174), -3.4645)
    _inc_edges_sk_Sketch4_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_5 = Wire.combine(_inc_edges_sk_Sketch4_5)[0]
_wire_sk_Sketch4_5 = _wire_sk_Sketch4_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch4_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch4_5.wrapped, True)
_face_sk_Sketch4_5 = Face(_mkf_sk_Sketch4_5.Face())

# 'Sketch4': 28 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch4_6:
    with BuildLine():
        RadiusArc((-2.6824, -8.6063), (-3.0994, -8.8563), -1.9075)
        # Near-straight arc (sagitta=0.009824mm) replaced with Line
        Line((-3.0994, -8.8563), (-3.3335, -9.0954))
        RadiusArc((-3.3335, -9.0954), (-3.5751, -9.5932), -1.3796)
        RadiusArc((-3.5751, -9.5932), (-3.6345, -10.0807), -1.8389)
        RadiusArc((-3.6345, -10.0807), (-3.5759, -10.5805), -2.0876)
        RadiusArc((-3.5759, -10.5805), (-3.3379, -11.0911), -1.4944)
        RadiusArc((-3.3379, -11.0911), (-2.9168, -11.4749), -1.4752)
        RadiusArc((-2.9168, -11.4749), (-2.2956, -11.7409), -2.308)
        RadiusArc((-2.2956, -11.7409), (-1.6297, -11.8825), -4.7385)
        RadiusArc((-1.6297, -11.8825), (-0.8127, -11.9611), -8.2881)
        # Near-straight arc (sagitta=0.004863mm) replaced with Line
        Line((-0.8127, -11.9611), (-0.1008, -11.9788))
        # Near-straight arc (sagitta=0.008269mm) replaced with Line
        Line((-0.1008, -11.9788), (0.8058, -11.9469))
        # Near-straight arc (sagitta=0.009733mm) replaced with Line
        Line((0.8058, -11.9469), (1.5727, -11.8512))
        RadiusArc((1.5727, -11.8512), (2.1998, -11.6918), -4.5186)
        RadiusArc((2.1998, -11.6918), (2.6878, -11.4681), -2.6037)
        RadiusArc((2.6878, -11.4681), (3.1214, -11.087), -1.633)
        RadiusArc((3.1214, -11.087), (3.3325, -10.6976), -1.3918)
        RadiusArc((3.3325, -10.6976), (3.423, -10.2331), -1.6523)
        RadiusArc((3.423, -10.2331), (3.3921, -9.7378), -2.0136)
        RadiusArc((3.3921, -9.7378), (3.2381, -9.3124), -1.4836)
        RadiusArc((3.2381, -9.3124), (2.9607, -8.9609), -1.4042)
        RadiusArc((2.9607, -8.9609), (2.4405, -8.6249), -1.8303)
        RadiusArc((2.4405, -8.6249), (1.5492, -8.36), -3.551)
        RadiusArc((1.5492, -8.36), (0.5798, -8.2496), -7.2685)
        # Near-straight arc (sagitta=0.008603mm) replaced with Line
        Line((0.5798, -8.2496), (-0.3445, -8.2334))
        # Near-straight arc (sagitta=0.008259mm) replaced with Line
        Line((-0.3445, -8.2334), (-1.2279, -8.2793))
        # Near-straight arc (sagitta=0.009756mm) replaced with Line
        Line((-1.2279, -8.2793), (-1.9655, -8.3864))
        RadiusArc((-1.9655, -8.3864), (-2.6824, -8.6063), -3.8244)
    _inc_edges_sk_Sketch4_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_6 = Wire.combine(_inc_edges_sk_Sketch4_6)[0]
_wire_sk_Sketch4_6 = _wire_sk_Sketch4_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch4_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch4_6.wrapped, True)
_face_sk_Sketch4_6 = Face(_mkf_sk_Sketch4_6.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0001, -49.9999, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6249, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    _face = _face_sk_Sketch3_2
    _vec = Vector(0.0, 0.0, 1.0) * 6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 6.000000 mm
    
    # -- Extrude4_p1 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(0.0, 0.0, 1.0) * 6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 6.000000 mm
    
    # -- Extrude4_p2 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(0.0, 0.0, 1.0) * 6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 6.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5_p0 --
    _face = _face_sk_Sketch4_5
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.500000119 mm
    
    # -- Extrude5_p1 --
    _face = _face_sk_Sketch4_6
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
