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

# 'Sketch1': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((0.6, 43.9), (-9.0, 43.9))
        Line((-9.0, 43.9), (-9.0, 40.5))
        RadiusArc((-9.0, 40.5), (-6.0, 37.5102), -2.9864)
        Line((-6.0, 37.5102), (-6.0, -52.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -52.5), (-0.0, -58.5), -6.0)
        RadiusArc((-0.0, -58.5), (6.0, -52.5), -6.0)
        Line((6.0, -52.5), (6.0, 37.5102))
        RadiusArc((6.0, 37.5102), (9.0, 40.5), -2.9864)
        Line((9.0, 40.5), (9.0, 52.5))
        Line((9.0, 52.5), (-9.0, 52.5))
        Line((-9.0, 52.5), (-9.0, 49.1))
        Line((-9.0, 49.1), (0.6, 49.1))
        Line((0.6, 49.1), (0.6, 48.9966))
        # Arc split: sweep=211.59deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 48.9966), (3.9006, 46.5), 2.5945)
        RadiusArc((3.9006, 46.5), (0.6, 44.0034), 2.5945)
        Line((0.6, 44.0034), (0.6, 43.9))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 59 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((2.2794, -6.2884), (2.7765, -6.1457), -2.9534)
        # Near-straight arc (sagitta=0.006883mm) replaced with Line
        Line((2.7765, -6.1457), (3.1055, -5.9879))
        RadiusArc((3.1055, -5.9879), (3.4857, -5.7099), -2.1952)
        RadiusArc((3.4857, -5.7099), (3.799, -5.3547), -2.2078)
        RadiusArc((3.799, -5.3547), (4.0416, -4.9279), -2.4538)
        # Near-straight arc (sagitta=0.006577mm) replaced with Line
        Line((4.0416, -4.9279), (4.1746, -4.5644))
        RadiusArc((4.1746, -4.5644), (4.2866, -4.0222), -3.4781)
        RadiusArc((4.2866, -4.0222), (4.3239, -3.4139), -4.607)
        # Near-straight arc (sagitta=0.008033mm) replaced with Line
        Line((4.3239, -3.4139), (4.2927, -2.8653))
        RadiusArc((4.2927, -2.8653), (4.1538, -2.2116), -3.6027)
        # Near-straight arc (sagitta=0.009532mm) replaced with Line
        Line((4.1538, -2.2116), (3.9767, -1.7797))
        RadiusArc((3.9767, -1.7797), (3.6434, -1.2816), -2.5557)
        RadiusArc((3.6434, -1.2816), (3.207, -0.8799), -2.5243)
        RadiusArc((3.207, -0.8799), (2.5445, -0.5384), -2.5247)
        RadiusArc((2.5445, -0.5384), (1.9217, -0.3947), -3.0706)
        RadiusArc((1.9217, -0.3947), (1.243, -0.3656), -4.0195)
        RadiusArc((1.243, -0.3656), (0.6532, -0.446), -3.1571)
        RadiusArc((0.6532, -0.446), (0.1277, -0.6337), -2.578)
        RadiusArc((0.1277, -0.6337), (-0.3336, -0.9285), -2.4039)
        RadiusArc((-0.3336, -0.9285), (-0.7255, -1.3262), -2.5281)
        RadiusArc((-0.7255, -1.3262), (-1.0167, -1.8016), -2.4189)
        RadiusArc((-1.0167, -1.8016), (-1.2021, -2.3503), -2.676)
        RadiusArc((-1.2021, -2.3503), (-1.2815, -2.9723), -3.3671)
        RadiusArc((-1.2815, -2.9723), (-1.2628, -3.5501), -3.95)
        RadiusArc((-1.2628, -3.5501), (-1.1652, -4.0546), -3.1222)
        RadiusArc((-1.1652, -4.0546), (-0.9894, -4.5097), -2.6503)
        RadiusArc((-0.9894, -4.5097), (-0.7355, -4.9153), -2.4822)
        Line((-0.7355, -4.9153), (-3.4637, -4.7505))
        Line((-3.4637, -4.7505), (-3.4637, -0.8628))
        Line((-3.4637, -0.8628), (-4.3974, -0.8628))
        Line((-4.3974, -0.8628), (-4.3974, -5.7514))
        Line((-4.3974, -5.7514), (0.2287, -6.0382))
        Line((0.2287, -6.0382), (0.2287, -4.9641))
        # Near-straight arc (sagitta=0.004183mm) replaced with Line
        Line((0.2287, -4.9641), (0.0266, -4.7078))
        # Near-straight arc (sagitta=0.004519mm) replaced with Line
        Line((0.0266, -4.7078), (-0.132, -4.4514))
        # Near-straight arc (sagitta=0.006952mm) replaced with Line
        Line((-0.132, -4.4514), (-0.2654, -4.1436))
        # Near-straight arc (sagitta=0.00608mm) replaced with Line
        Line((-0.2654, -4.1436), (-0.3477, -3.8229))
        # Near-straight arc (sagitta=0.008062mm) replaced with Line
        Line((-0.3477, -3.8229), (-0.387, -3.4242))
        RadiusArc((-0.387, -3.4242), (-0.3521, -2.9343), 2.644)
        RadiusArc((-0.3521, -2.9343), (-0.2453, -2.5535), 1.9467)
        RadiusArc((-0.2453, -2.5535), (-0.0674, -2.2226), 1.6522)
        RadiusArc((-0.0674, -2.2226), (0.1809, -1.9422), 1.6318)
        RadiusArc((0.1809, -1.9422), (0.481, -1.7274), 1.6354)
        RadiusArc((0.481, -1.7274), (0.8254, -1.5842), 1.6637)
        RadiusArc((0.8254, -1.5842), (1.3181, -1.5058), 1.9989)
        # Near-straight arc (sagitta=0.008939mm) replaced with Line
        Line((1.3181, -1.5058), (1.7825, -1.5236))
        # Near-straight arc (sagitta=0.00992mm) replaced with Line
        Line((1.7825, -1.5236), (2.2094, -1.6126))
        # Near-straight arc (sagitta=0.006084mm) replaced with Line
        Line((2.2094, -1.6126), (2.4924, -1.726))
        RadiusArc((2.4924, -1.726), (2.8198, -1.9396), 1.7013)
        RadiusArc((2.8198, -1.9396), (3.0851, -2.2197), 1.6262)
        RadiusArc((3.0851, -2.2197), (3.2749, -2.5549), 1.5927)
        RadiusArc((3.2749, -2.5549), (3.3888, -2.9448), 1.8408)
        RadiusArc((3.3888, -2.9448), (3.4268, -3.3895), 2.416)
        # Near-straight arc (sagitta=0.005952mm) replaced with Line
        Line((3.4268, -3.3895), (3.4041, -3.7541))
        RadiusArc((3.4041, -3.7541), (3.2851, -4.2283), 2.0354)
        RadiusArc((3.2851, -4.2283), (3.064, -4.6151), 1.527)
        RadiusArc((3.064, -4.6151), (2.7408, -4.9144), 1.4542)
        RadiusArc((2.7408, -4.9144), (2.1512, -5.1777), 1.8703)
        Line((2.1512, -5.1777), (2.2794, -6.2884))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 28 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((-4.4814, -10.6721), (-4.285, -11.3562), -2.8366)
        RadiusArc((-4.285, -11.3562), (-3.9315, -11.921), -2.2893)
        RadiusArc((-3.9315, -11.921), (-3.4209, -12.3664), -2.3587)
        RadiusArc((-3.4209, -12.3664), (-2.7528, -12.7007), -3.3361)
        RadiusArc((-2.7528, -12.7007), (-1.6955, -12.9739), -5.1981)
        RadiusArc((-1.6955, -12.9739), (-0.6719, -13.0768), -8.6923)
        RadiusArc((-0.6719, -13.0768), (0.4585, -13.0764), -12.2991)
        RadiusArc((0.4585, -13.0764), (1.4665, -12.9705), -8.8352)
        RadiusArc((1.4665, -12.9705), (2.3269, -12.7587), -5.8712)
        RadiusArc((2.3269, -12.7587), (3.0397, -12.441), -4.0085)
        RadiusArc((3.0397, -12.441), (3.7167, -11.8954), -2.78)
        RadiusArc((3.7167, -11.8954), (4.1433, -11.1857), -2.3044)
        RadiusArc((4.1433, -11.1857), (4.3189, -10.3118), -2.7724)
        RadiusArc((4.3189, -10.3118), (4.2785, -9.5531), -3.5868)
        RadiusArc((4.2785, -9.5531), (4.0767, -8.8907), -2.6617)
        RadiusArc((4.0767, -8.8907), (3.5974, -8.2074), -2.346)
        RadiusArc((3.5974, -8.2074), (2.8679, -7.6854), -2.837)
        RadiusArc((2.8679, -7.6854), (1.9073, -7.3308), -4.245)
        RadiusArc((1.9073, -7.3308), (0.9757, -7.1687), -6.589)
        RadiusArc((0.9757, -7.1687), (-0.1008, -7.1147), -9.7702)
        # Near-straight arc (sagitta=0.007492mm) replaced with Line
        Line((-0.1008, -7.1147), (-0.9343, -7.1436))
        RadiusArc((-0.9343, -7.1436), (-2.1324, -7.3204), -7.9696)
        RadiusArc((-2.1324, -7.3204), (-2.919, -7.5776), -4.8834)
        RadiusArc((-2.919, -7.5776), (-3.5527, -7.9379), -3.3056)
        RadiusArc((-3.5527, -7.9379), (-4.0292, -8.4085), -2.3776)
        RadiusArc((-4.0292, -8.4085), (-4.4015, -9.1561), -2.3019)
        RadiusArc((-4.4015, -9.1561), (-4.5206, -9.8817), -3.01)
        RadiusArc((-4.5206, -9.8817), (-4.4814, -10.6721), -3.9817)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch2': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch2_4:
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
    _inc_edges_sk_Sketch2_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_4 = Wire.combine(_inc_edges_sk_Sketch2_4)[0]
_wire_sk_Sketch2_4 = _wire_sk_Sketch2_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch2_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch2_4.wrapped, True)
_face_sk_Sketch2_4 = Face(_mkf_sk_Sketch2_4.Face())

# 'Sketch3': 31 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        RadiusArc((2.0561, -11.7376), (2.5785, -11.5303), -2.9746)
        RadiusArc((2.5785, -11.5303), (2.9706, -11.2535), -1.8241)
        RadiusArc((2.9706, -11.2535), (3.242, -10.9016), -1.4169)
        RadiusArc((3.242, -10.9016), (3.3928, -10.4747), -1.4632)
        RadiusArc((3.3928, -10.4747), (3.4229, -9.9781), -1.9465)
        RadiusArc((3.4229, -9.9781), (3.3305, -9.5159), -1.6938)
        RadiusArc((3.3305, -9.5159), (3.1148, -9.1274), -1.3891)
        RadiusArc((3.1148, -9.1274), (2.7759, -8.8128), -1.5271)
        RadiusArc((2.7759, -8.8128), (2.3126, -8.5707), -2.1806)
        RadiusArc((2.3126, -8.5707), (1.7184, -8.3941), -3.7082)
        RadiusArc((1.7184, -8.3941), (0.9923, -8.2817), -6.1912)
        # Near-straight arc (sagitta=0.009276mm) replaced with Line
        Line((0.9923, -8.2817), (0.1343, -8.2335))
        # Near-straight arc (sagitta=0.004366mm) replaced with Line
        Line((0.1343, -8.2335), (-0.579, -8.2392))
        # Near-straight arc (sagitta=0.008589mm) replaced with Line
        Line((-0.579, -8.2392), (-1.426, -8.3004))
        RadiusArc((-1.426, -8.3004), (-2.1271, -8.4227), -6.2069)
        RadiusArc((-2.1271, -8.4227), (-2.6824, -8.6063), -3.4644)
        # Near-straight arc (sagitta=0.008719mm) replaced with Line
        Line((-2.6824, -8.6063), (-3.0065, -8.7865))
        RadiusArc((-3.0065, -8.7865), (-3.3335, -9.0954), -1.4877)
        RadiusArc((-3.3335, -9.0954), (-3.5416, -9.4837), -1.3644)
        # Near-straight arc (sagitta=0.009528mm) replaced with Line
        Line((-3.5416, -9.4837), (-3.6196, -9.827))
        # Near-straight arc (sagitta=0.008638mm) replaced with Line
        Line((-3.6196, -9.827), (-3.6308, -10.2133))
        RadiusArc((-3.6308, -10.2133), (-3.5429, -10.6927), -1.8905)
        RadiusArc((-3.5429, -10.6927), (-3.3379, -11.0911), -1.4464)
        # Near-straight arc (sagitta=0.009833mm) replaced with Line
        Line((-3.3379, -11.0911), (-3.1072, -11.3366))
        RadiusArc((-3.1072, -11.3366), (-2.696, -11.5934), -1.7554)
        RadiusArc((-2.696, -11.5934), (-1.9816, -11.8195), -3.1411)
        RadiusArc((-1.9816, -11.8195), (-1.2401, -11.9297), -6.2987)
        # Near-straight arc (sagitta=0.009245mm) replaced with Line
        Line((-1.2401, -11.9297), (-0.3476, -11.9769))
        # Near-straight arc (sagitta=0.007878mm) replaced with Line
        Line((-0.3476, -11.9769), (0.5922, -11.9609))
        # Near-straight arc (sagitta=0.009255mm) replaced with Line
        Line((0.5922, -11.9609), (1.394, -11.8811))
        RadiusArc((1.394, -11.8811), (2.0561, -11.7376), -5.2184)
    _inc_edges_sk_Sketch3_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_5 = Wire.combine(_inc_edges_sk_Sketch3_5)[0]
_wire_sk_Sketch3_5 = _wire_sk_Sketch3_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch3_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch3_5.wrapped, True)
_face_sk_Sketch3_5 = Face(_mkf_sk_Sketch3_5.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(-0.0021, -52.4983, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6223, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * 5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 5.000000 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * 5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 5.000000 mm
    
    # -- Extrude2_p2 --
    _face = _face_sk_Sketch2_4
    _vec = Vector(0.0, 0.0, 1.0) * 5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 5.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_5
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.50000012 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
