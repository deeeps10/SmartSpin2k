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
        Line((0.6, 66.5034), (0.6, 66.4))
        Line((0.6, 66.4), (-9.0, 66.4))
        Line((-9.0, 66.4), (-9.0, 63.0))
        RadiusArc((-9.0, 63.0), (-6.0, 60.0102), -2.9779)
        Line((-6.0, 60.0102), (-6.0, -75.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -75.0), (-0.0, -81.0), -6.0)
        RadiusArc((-0.0, -81.0), (6.0, -75.0), -6.0)
        Line((6.0, -75.0), (6.0, 60.0102))
        RadiusArc((6.0, 60.0102), (9.0, 63.0), -2.9819)
        Line((9.0, 63.0), (9.0, 75.0))
        Line((9.0, 75.0), (-9.0, 75.0))
        Line((-9.0, 75.0), (-9.0, 71.6))
        Line((-9.0, 71.6), (0.6, 71.6))
        Line((0.6, 71.6), (0.6, 71.4966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 71.4966), (3.9, 69.0), 2.5944)
        RadiusArc((3.9, 69.0), (0.6, 66.5034), 2.5944)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 19 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-4.4814, -3.8832), (-4.1279, -4.8646), -2.7125)
        RadiusArc((-4.1279, -4.8646), (-3.4209, -5.5774), -2.2857)
        RadiusArc((-3.4209, -5.5774), (-2.3595, -6.0403), -3.5622)
        RadiusArc((-2.3595, -6.0403), (-0.9426, -6.2717), -6.7024)
        RadiusArc((-0.9426, -6.2717), (0.4585, -6.2875), -11.7415)
        RadiusArc((0.4585, -6.2875), (2.1256, -6.0326), -7.9555)
        RadiusArc((2.1256, -6.0326), (3.4759, -5.3444), -3.8987)
        RadiusArc((3.4759, -5.3444), (4.1433, -4.3967), -2.3555)
        RadiusArc((4.1433, -4.3967), (4.3189, -3.1339), -2.9186)
        RadiusArc((4.3189, -3.1339), (4.001, -1.9523), -2.8615)
        RadiusArc((4.001, -1.9523), (3.3351, -1.1901), -2.3928)
        RadiusArc((3.3351, -1.1901), (1.688, -0.4912), -3.7156)
        RadiusArc((1.688, -0.4912), (0.1819, -0.3291), -7.7673)
        RadiusArc((0.1819, -0.3291), (-1.6818, -0.4415), -10.9965)
        RadiusArc((-1.6818, -0.4415), (-2.919, -0.7886), -5.6102)
        RadiusArc((-2.919, -0.7886), (-3.8108, -1.3701), -3.0857)
        RadiusArc((-3.8108, -1.3701), (-4.2824, -2.0469), -2.2548)
        RadiusArc((-4.2824, -2.0469), (-4.5057, -2.9007), -2.657)
        RadiusArc((-4.5057, -2.9007), (-4.4814, -3.8832), -3.8805)
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 44 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((2.1512, -11.9666), (2.7408, -11.7034), -1.8675)
        RadiusArc((2.7408, -11.7034), (3.149, -11.2848), -1.444)
        RadiusArc((3.149, -11.2848), (3.3758, -10.7108), -1.7552)
        RadiusArc((3.3758, -10.7108), (3.4244, -10.0621), -2.8099)
        RadiusArc((3.4244, -10.0621), (3.2749, -9.3438), -1.964)
        RadiusArc((3.2749, -9.3438), (2.9616, -8.8615), -1.583)
        RadiusArc((2.9616, -8.8615), (2.6632, -8.6128), -1.6629)
        RadiusArc((2.6632, -8.6128), (2.2094, -8.4015), -1.8596)
        RadiusArc((2.2094, -8.4015), (1.6668, -8.3014), -2.5252)
        RadiusArc((1.6668, -8.3014), (1.1127, -8.3127), -2.5243)
        RadiusArc((1.1127, -8.3127), (0.6476, -8.4358), -1.8056)
        RadiusArc((0.6476, -8.4358), (0.3254, -8.6148), -1.6178)
        RadiusArc((0.3254, -8.6148), (-0.1185, -9.0895), -1.6343)
        RadiusArc((-0.1185, -9.0895), (-0.3521, -9.7232), -1.8581)
        RadiusArc((-0.3521, -9.7232), (-0.3571, -10.5565), -2.5975)
        RadiusArc((-0.3571, -10.5565), (-0.1585, -11.1891), -2.0838)
        RadiusArc((-0.1585, -11.1891), (0.2287, -11.753), -2.8891)
        Line((0.2287, -11.753), (0.2287, -12.8272))
        Line((0.2287, -12.8272), (-4.3974, -12.5403))
        Line((-4.3974, -12.5403), (-4.3974, -7.6517))
        Line((-4.3974, -7.6517), (-3.4637, -7.6517))
        Line((-3.4637, -7.6517), (-3.4637, -11.5395))
        Line((-3.4637, -11.5395), (-0.7355, -11.7042))
        RadiusArc((-0.7355, -11.7042), (-1.087, -11.0773), 2.52)
        RadiusArc((-1.087, -11.0773), (-1.2457, -10.4698), 2.9368)
        RadiusArc((-1.2457, -10.4698), (-1.2716, -9.5988), 3.9161)
        RadiusArc((-1.2716, -9.5988), (-1.1226, -8.8557), 3.0261)
        RadiusArc((-1.1226, -8.8557), (-0.8083, -8.2271), 2.4664)
        RadiusArc((-0.8083, -8.2271), (-0.3336, -7.7174), 2.5137)
        RadiusArc((-0.3336, -7.7174), (0.5158, -7.2718), 2.4617)
        RadiusArc((0.5158, -7.2718), (1.243, -7.1546), 3.0917)
        RadiusArc((1.243, -7.1546), (2.0852, -7.2087), 3.8416)
        RadiusArc((2.0852, -7.2087), (2.825, -7.4424), 2.8049)
        RadiusArc((2.825, -7.4424), (3.5427, -7.96), 2.4886)
        RadiusArc((3.5427, -7.96), (4.0427, -8.707), 2.5874)
        RadiusArc((4.0427, -8.707), (4.2684, -9.4824), 3.289)
        RadiusArc((4.2684, -9.4824), (4.3239, -10.2029), 4.5721)
        RadiusArc((4.3239, -10.2029), (4.2399, -11.0905), 4.1834)
        RadiusArc((4.2399, -11.0905), (4.0416, -11.7168), 2.9575)
        RadiusArc((4.0416, -11.7168), (3.727, -12.2397), 2.4171)
        RadiusArc((3.727, -12.2397), (3.3039, -12.6475), 2.1696)
        RadiusArc((3.3039, -12.6475), (2.8904, -12.8868), 2.3089)
        RadiusArc((2.8904, -12.8868), (2.2794, -13.0774), 2.902)
        Line((2.2794, -13.0774), (2.1512, -11.9666))
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch2': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
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

# 'Sketch3': 22 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        RadiusArc((-3.5429, -3.9038), (-3.1072, -4.5476), -1.4282)
        RadiusArc((-3.1072, -4.5476), (-2.2956, -4.952), -1.9763)
        RadiusArc((-2.2956, -4.952), (-1.2401, -5.1407), -5.222)
        # Near-straight arc (sagitta=0.009245mm) replaced with Line
        Line((-1.2401, -5.1407), (-0.3476, -5.1879))
        RadiusArc((-0.3476, -5.1879), (0.8058, -5.158), -13.3723)
        RadiusArc((0.8058, -5.158), (1.7425, -5.0284), -7.3928)
        RadiusArc((1.7425, -5.0284), (2.461, -4.7991), -3.7764)
        RadiusArc((2.461, -4.7991), (2.9706, -4.4646), -1.9464)
        RadiusArc((2.9706, -4.4646), (3.291, -4.013), -1.4125)
        RadiusArc((3.291, -4.013), (3.4229, -3.1892), -1.665)
        RadiusArc((3.4229, -3.1892), (3.3305, -2.7269), -1.6937)
        RadiusArc((3.3305, -2.7269), (2.9607, -2.172), -1.3958)
        RadiusArc((2.9607, -2.172), (2.3126, -1.7818), -1.9013)
        RadiusArc((2.3126, -1.7818), (1.5492, -1.571), -3.8726)
        RadiusArc((1.5492, -1.571), (0.5798, -1.4606), -7.2757)
        RadiusArc((0.5798, -1.4606), (-0.8044, -1.4598), -12.8763)
        RadiusArc((-0.8044, -1.4598), (-2.1271, -1.6338), -8.1035)
        RadiusArc((-2.1271, -1.6338), (-2.7984, -1.8729), -3.2907)
        RadiusArc((-2.7984, -1.8729), (-3.2629, -2.2218), -1.6777)
        RadiusArc((-3.2629, -2.2218), (-3.5416, -2.6947), -1.3611)
        RadiusArc((-3.5416, -2.6947), (-3.6345, -3.2918), -1.7339)
        RadiusArc((-3.6345, -3.2918), (-3.5429, -3.9038), -2.0222)
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
    _bore_ax = _gAx2(_gPnt(0.0, -75.0, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.625, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude2_p2 --
    _face = _face_sk_Sketch2_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_5
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
