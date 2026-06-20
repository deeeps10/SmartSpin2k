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

# 'Sketch17': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch17:
    with BuildLine():
        Line((0.6, 56.4), (-9.0, 56.4))
        Line((-9.0, 56.4), (-9.0, 53.0))
        RadiusArc((-9.0, 53.0), (-6.0, 50.0102), -2.9819)
        Line((-6.0, 50.0102), (-6.0, -65.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -65.0), (-0.0, -71.0), -6.0)
        RadiusArc((-0.0, -71.0), (6.0, -65.0), -6.0)
        Line((6.0, -65.0), (6.0, 50.0102))
        RadiusArc((6.0, 50.0102), (9.0, 53.0), -2.9819)
        Line((9.0, 53.0), (9.0, 65.0))
        Line((9.0, 65.0), (-9.0, 65.0))
        Line((-9.0, 65.0), (-9.0, 61.6))
        Line((-9.0, 61.6), (0.6, 61.6))
        Line((0.6, 61.6), (0.6, 61.4966))
        # Arc split: sweep=211.57deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 61.4966), (3.9002, 59.0), 2.5944)
        RadiusArc((3.9002, 59.0), (0.6, 56.5034), 2.5944)
        Line((0.6, 56.5034), (0.6, 56.4))
    _inc_edges_sk_Sketch17 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch17 = Wire.combine(_inc_edges_sk_Sketch17)[0]
_wire_sk_Sketch17 = _wire_sk_Sketch17.moved(_inclined_plane_1.location)
_mkf_sk_Sketch17 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch17.wrapped, True)
_face_sk_Sketch17 = Face(_mkf_sk_Sketch17.Face())

# 'Sketch18': 18 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch18_2:
    with BuildLine():
        RadiusArc((-3.255, -0.9558), (-4.1235, -1.755), -2.3543)
        RadiusArc((-4.1235, -1.755), (-4.5206, -3.0927), -2.5343)
        RadiusArc((-4.5206, -3.0927), (-4.4029, -4.2402), -3.768)
        RadiusArc((-4.4029, -4.2402), (-3.4209, -5.5774), -2.3456)
        RadiusArc((-3.4209, -5.5774), (-2.148, -6.095), -4.3586)
        RadiusArc((-2.148, -6.095), (-0.9426, -6.2717), -8.0344)
        RadiusArc((-0.9426, -6.2717), (0.4585, -6.2875), -11.9291)
        RadiusArc((0.4585, -6.2875), (2.5189, -5.9003), -5.8951)
        RadiusArc((2.5189, -5.9003), (3.7167, -5.1065), -2.807)
        RadiusArc((3.7167, -5.1065), (4.2436, -4.0669), -2.4245)
        RadiusArc((4.2436, -4.0669), (4.2785, -2.7641), -3.4226)
        RadiusArc((4.2785, -2.7641), (3.9153, -1.8092), -2.4438)
        RadiusArc((3.9153, -1.8092), (3.1887, -1.0856), -2.6127)
        RadiusArc((3.1887, -1.0856), (2.1175, -0.5993), -4.1289)
        RadiusArc((2.1175, -0.5993), (0.4555, -0.3393), -7.7652)
        RadiusArc((0.4555, -0.3393), (-1.193, -0.3772), -10.9148)
        RadiusArc((-1.193, -0.3772), (-2.5448, -0.6472), -6.0195)
        RadiusArc((-2.5448, -0.6472), (-3.255, -0.9558), -3.7491)
    _inc_edges_sk_Sketch18_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18_2 = Wire.combine(_inc_edges_sk_Sketch18_2)[0]
_wire_sk_Sketch18_2 = _wire_sk_Sketch18_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch18_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch18_2.wrapped, True)
_face_sk_Sketch18_2 = Face(_mkf_sk_Sketch18_2.Face())

# 'Sketch18': 68 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch18_3:
    with BuildLine():
        Line((-0.6501, -10.4408), (-0.6501, -11.039))
        Line((-0.6501, -11.039), (0.302, -11.039))
        Line((0.302, -11.039), (0.302, -10.4164))
        # Near-straight arc (sagitta=0.007224mm) replaced with Line
        Line((0.302, -10.4164), (0.33, -9.9069))
        # Near-straight arc (sagitta=0.008208mm) replaced with Line
        Line((0.33, -9.9069), (0.414, -9.4655))
        RadiusArc((0.414, -9.4655), (0.5977, -9.0096), 2.0883)
        RadiusArc((0.5977, -9.0096), (0.8654, -8.6627), 1.5149)
        RadiusArc((0.8654, -8.6627), (1.2032, -8.4346), 1.2259)
        RadiusArc((1.2032, -8.4346), (1.6094, -8.3266), 1.3007)
        RadiusArc((1.6094, -8.3266), (2.0975, -8.3359), 2.0099)
        RadiusArc((2.0975, -8.3359), (2.5376, -8.4515), 1.7445)
        RadiusArc((2.5376, -8.4515), (2.8896, -8.6723), 1.2996)
        RadiusArc((2.8896, -8.6723), (3.1074, -8.9225), 1.3158)
        RadiusArc((3.1074, -8.9225), (3.2983, -9.3092), 1.568)
        # Near-straight arc (sagitta=0.008744mm) replaced with Line
        Line((3.2983, -9.3092), (3.3855, -9.6757))
        # Near-straight arc (sagitta=0.007912mm) replaced with Line
        Line((3.3855, -9.6757), (3.4146, -10.093))
        RadiusArc((3.4146, -10.093), (3.3061, -10.8731), 2.7051)
        RadiusArc((3.3061, -10.8731), (2.7364, -11.6608), 1.521)
        RadiusArc((2.7364, -11.6608), (1.8888, -11.9666), 1.8575)
        Line((1.8888, -11.9666), (1.9926, -13.1019))
        RadiusArc((1.9926, -13.1019), (2.825, -12.8836), -2.946)
        RadiusArc((2.825, -12.8836), (3.5679, -12.3735), -2.2135)
        RadiusArc((3.5679, -12.3735), (4.0622, -11.5998), -2.5619)
        RadiusArc((4.0622, -11.5998), (4.282, -10.752), -3.5812)
        RadiusArc((4.282, -10.752), (4.321, -9.9087), -5.3317)
        RadiusArc((4.321, -9.9087), (4.2194, -9.0735), -4.2017)
        RadiusArc((4.2194, -9.0735), (3.9727, -8.3821), -2.8273)
        RadiusArc((3.9727, -8.3821), (3.5814, -7.835), -2.2375)
        RadiusArc((3.5814, -7.835), (2.9444, -7.391), -2.0587)
        RadiusArc((2.9444, -7.391), (2.1356, -7.1892), -2.6433)
        RadiusArc((2.1356, -7.1892), (1.3097, -7.2354), -2.3035)
        RadiusArc((1.3097, -7.2354), (0.5343, -7.6436), -1.8035)
        RadiusArc((0.5343, -7.6436), (0.1474, -8.1183), -1.9626)
        # Near-straight arc (sagitta=0.002612mm) replaced with Line
        Line((0.1474, -8.1183), (0.0494, -8.3071))
        RadiusArc((0.0494, -8.3071), (-0.0997, -8.7302), -2.4949)
        # Near-straight arc (sagitta=0.009269mm) replaced with Line
        Line((-0.0997, -8.7302), (-0.1862, -9.2141))
        Line((-0.1862, -9.2141), (-0.2106, -9.2141))
        RadiusArc((-0.2106, -9.2141), (-0.454, -8.4849), -3.2258)
        RadiusArc((-0.454, -8.4849), (-0.7779, -8.0048), -2.1333)
        RadiusArc((-0.7779, -8.0048), (-1.2875, -7.6185), -1.7493)
        RadiusArc((-1.2875, -7.6185), (-1.8316, -7.4464), -1.8719)
        RadiusArc((-1.8316, -7.4464), (-2.3903, -7.4169), -2.5757)
        RadiusArc((-2.3903, -7.4169), (-3.0328, -7.5284), -2.5298)
        RadiusArc((-3.0328, -7.5284), (-3.665, -7.8726), -1.936)
        RadiusArc((-3.665, -7.8726), (-4.2584, -8.673), -2.0909)
        RadiusArc((-4.2584, -8.673), (-4.4829, -9.4928), -2.95)
        RadiusArc((-4.4829, -9.4928), (-4.5151, -10.4499), -4.6295)
        RadiusArc((-4.5151, -10.4499), (-4.3572, -11.2823), -3.5391)
        RadiusArc((-4.3572, -11.2823), (-3.9336, -12.0795), -2.6531)
        RadiusArc((-3.9336, -12.0795), (-3.28, -12.6455), -2.2537)
        RadiusArc((-3.28, -12.6455), (-2.2858, -12.9553), -2.4788)
        Line((-2.2858, -12.9553), (-2.2003, -11.8507))
        RadiusArc((-2.2003, -11.8507), (-2.6934, -11.732), 1.9457)
        RadiusArc((-2.6934, -11.732), (-3.0298, -11.5414), 1.4307)
        RadiusArc((-3.0298, -11.5414), (-3.3427, -11.2043), 1.3818)
        RadiusArc((-3.3427, -11.2043), (-3.5628, -10.6906), 1.5611)
        RadiusArc((-3.5628, -10.6906), (-3.6206, -10.0491), 2.2393)
        RadiusArc((-3.6206, -10.0491), (-3.5584, -9.5774), 2.3127)
        RadiusArc((-3.5584, -9.5774), (-3.3665, -9.1268), 1.5145)
        RadiusArc((-3.3665, -9.1268), (-3.0511, -8.8026), 1.2075)
        RadiusArc((-3.0511, -8.8026), (-2.7137, -8.6312), 1.2966)
        RadiusArc((-2.7137, -8.6312), (-2.3119, -8.55), 1.6147)
        RadiusArc((-2.3119, -8.55), (-1.8669, -8.5627), 1.9916)
        RadiusArc((-1.8669, -8.5627), (-1.407, -8.7219), 1.3521)
        RadiusArc((-1.407, -8.7219), (-1.0976, -8.9761), 1.2466)
        RadiusArc((-1.0976, -8.9761), (-0.8618, -9.3343), 1.4879)
        RadiusArc((-0.8618, -9.3343), (-0.7131, -9.7832), 1.9417)
        RadiusArc((-0.7131, -9.7832), (-0.6501, -10.4408), 2.9921)
    _inc_edges_sk_Sketch18_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18_3 = Wire.combine(_inc_edges_sk_Sketch18_3)[0]
_wire_sk_Sketch18_3 = _wire_sk_Sketch18_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch18_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch18_3.wrapped, True)
_face_sk_Sketch18_3 = Face(_mkf_sk_Sketch18_3.Face())

# 'Sketch18': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch18_4:
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
    _inc_edges_sk_Sketch18_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18_4 = Wire.combine(_inc_edges_sk_Sketch18_4)[0]
_wire_sk_Sketch18_4 = _wire_sk_Sketch18_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch18_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch18_4.wrapped, True)
_face_sk_Sketch18_4 = Face(_mkf_sk_Sketch18_4.Face())

# 'Sketch21': 22 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch21_5:
    with BuildLine():
        RadiusArc((-3.5027, -4.011), (-2.9168, -4.6859), -1.4285)
        RadiusArc((-2.9168, -4.6859), (-1.9816, -5.0306), -2.558)
        RadiusArc((-1.9816, -5.0306), (-0.8127, -5.1722), -6.9509)
        # Near-straight arc (sagitta=0.008344mm) replaced with Line
        Line((-0.8127, -5.1722), (0.1389, -5.1879))
        RadiusArc((0.1389, -5.1879), (1.2067, -5.1181), -10.6491)
        RadiusArc((1.2067, -5.1181), (2.1998, -4.9028), -5.4499)
        RadiusArc((2.1998, -4.9028), (2.9706, -4.4646), -2.2929)
        RadiusArc((2.9706, -4.4646), (3.3325, -3.9086), -1.4102)
        RadiusArc((3.3325, -3.9086), (3.4268, -3.3163), -1.6999)
        RadiusArc((3.4268, -3.3163), (3.3305, -2.7269), -1.795)
        RadiusArc((3.3305, -2.7269), (3.0416, -2.2529), -1.3905)
        RadiusArc((3.0416, -2.2529), (2.5602, -1.8942), -1.6814)
        RadiusArc((2.5602, -1.8942), (1.7184, -1.6051), -3.1185)
        RadiusArc((1.7184, -1.6051), (0.9923, -1.4928), -6.189)
        RadiusArc((0.9923, -1.4928), (-0.1008, -1.4426), -10.3133)
        RadiusArc((-0.1008, -1.4426), (-1.426, -1.5114), -12.2877)
        RadiusArc((-1.426, -1.5114), (-2.1271, -1.6338), -6.2074)
        RadiusArc((-2.1271, -1.6338), (-2.6824, -1.8174), -3.4645)
        RadiusArc((-2.6824, -1.8174), (-3.1849, -2.142), -1.849)
        RadiusArc((-3.1849, -2.142), (-3.5007, -2.5902), -1.3697)
        RadiusArc((-3.5007, -2.5902), (-3.6308, -3.1625), -1.6003)
        RadiusArc((-3.6308, -3.1625), (-3.5027, -4.011), -2.0585)
    _inc_edges_sk_Sketch21_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_5 = Wire.combine(_inc_edges_sk_Sketch21_5)[0]
_wire_sk_Sketch21_5 = _wire_sk_Sketch21_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch21_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch21_5.wrapped, True)
_face_sk_Sketch21_5 = Face(_mkf_sk_Sketch21_5.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude14 ---
    # -- Extrude14 --
    _face = _face_sk_Sketch17
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(-0.0001, -65.0, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6251, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude15 ---
    # -- Extrude15_p0 --
    _face = _face_sk_Sketch18_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude15_p1 --
    _face = _face_sk_Sketch18_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude15_p2 --
    _face = _face_sk_Sketch18_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # --- FEATURE: Extrude16 ---
    # -- Extrude16 --
    _face = _face_sk_Sketch21_5
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
