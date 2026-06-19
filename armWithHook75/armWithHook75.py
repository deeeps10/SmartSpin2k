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
        Line((0.6, 29.0034), (0.6, 28.9))
        Line((0.6, 28.9), (-9.0, 28.9))
        Line((-9.0, 28.9), (-9.0, 25.5))
        RadiusArc((-9.0, 25.5), (-6.0, 22.5102), -2.9864)
        Line((-6.0, 22.5102), (-6.0, -37.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -37.5), (0.0, -43.5), -6.0)
        RadiusArc((0.0, -43.5), (6.0, -37.5), -6.0)
        Line((6.0, -37.5), (6.0, 22.5102))
        RadiusArc((6.0, 22.5102), (9.0, 25.5), -2.9846)
        Line((9.0, 25.5), (9.0, 37.5))
        Line((9.0, 37.5), (-9.0, 37.5))
        Line((-9.0, 37.5), (-9.0, 34.1))
        Line((-9.0, 34.1), (0.6, 34.1))
        Line((0.6, 34.1), (0.6, 33.9966))
        # Arc split: sweep=211.72deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 33.9966), (3.9047, 31.5), 2.5954)
        RadiusArc((3.9047, 31.5), (0.6, 29.0034), 2.5954)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 33 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((2.0865, -5.1777), (2.9993, -4.6151), -1.5967)
        RadiusArc((2.9993, -4.6151), (3.3394, -3.7541), -1.8324)
        RadiusArc((3.3394, -3.7541), (3.3526, -3.1603), -2.8258)
        RadiusArc((3.3526, -3.1603), (3.2102, -2.5549), -1.9197)
        RadiusArc((3.2102, -2.5549), (2.897, -2.0725), -1.583)
        RadiusArc((2.897, -2.0725), (2.0433, -1.5837), -1.8374)
        RadiusArc((2.0433, -1.5837), (1.1493, -1.5125), -2.7702)
        RadiusArc((1.1493, -1.5125), (0.3371, -1.7744), -1.7208)
        RadiusArc((0.3371, -1.7744), (-0.3434, -2.644), -1.6549)
        RadiusArc((-0.3434, -2.644), (-0.4517, -3.4242), -2.4624)
        RadiusArc((-0.4517, -3.4242), (-0.3469, -4.0915), -2.3508)
        RadiusArc((-0.3469, -4.0915), (-0.1384, -4.554), -2.1161)
        RadiusArc((-0.1384, -4.554), (0.164, -4.9641), -3.0411)
        Line((0.164, -4.9641), (0.164, -6.0382))
        Line((0.164, -6.0382), (-4.4621, -5.7514))
        Line((-4.4621, -5.7514), (-4.4621, -0.8628))
        Line((-4.4621, -0.8628), (-3.5284, -0.8628))
        Line((-3.5284, -0.8628), (-3.5284, -4.7505))
        Line((-3.5284, -4.7505), (-0.8002, -4.9153))
        RadiusArc((-0.8002, -4.9153), (-1.3104, -3.6809), 2.752)
        RadiusArc((-1.3104, -3.6809), (-1.1873, -2.0667), 3.3367)
        RadiusArc((-1.1873, -2.0667), (-0.289, -0.8447), 2.4789)
        RadiusArc((-0.289, -0.8447), (0.8754, -0.3924), 2.6445)
        RadiusArc((0.8754, -0.3924), (2.0205, -0.4198), 3.8191)
        RadiusArc((2.0205, -0.4198), (2.8928, -0.7217), 2.7275)
        RadiusArc((2.8928, -0.7217), (3.6725, -1.3978), 2.5081)
        RadiusArc((3.6725, -1.3978), (4.1724, -2.5273), 2.9432)
        RadiusArc((4.1724, -2.5273), (4.2382, -3.8763), 4.7663)
        RadiusArc((4.2382, -3.8763), (3.9769, -4.9279), 3.1432)
        RadiusArc((3.9769, -4.9279), (3.6623, -5.4507), 2.4092)
        RadiusArc((3.6623, -5.4507), (3.0408, -5.9879), 2.1858)
        RadiusArc((3.0408, -5.9879), (2.2147, -6.2884), 2.7895)
        Line((2.2147, -6.2884), (2.0865, -5.1777))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 51 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((3.6643, -10.192), (3.8212, -10.198))
        Line((3.8212, -10.198), (3.9788, -10.2017))
        Line((3.9788, -10.2017), (4.1372, -10.2029))
        Line((4.1372, -10.2029), (4.1372, -11.3502))
        RadiusArc((4.1372, -11.3502), (-3.5284, -8.317), 13.6791)
        Line((-3.5284, -8.317), (-3.5284, -12.9371))
        Line((-3.5284, -12.9371), (-4.4621, -12.9371))
        Line((-4.4621, -12.9371), (-4.4621, -7.255))
        Line((-4.4621, -7.255), (-3.571, -7.255))
        Line((-3.571, -7.255), (-3.3064, -7.4273))
        Line((-3.3064, -7.4273), (-3.0495, -7.5928))
        Line((-3.0495, -7.5928), (-2.8003, -7.7513))
        Line((-2.8003, -7.7513), (-2.5589, -7.903))
        Line((-2.5589, -7.903), (-2.3253, -8.0478))
        Line((-2.3253, -8.0478), (-2.0995, -8.1856))
        Line((-2.0995, -8.1856), (-1.8814, -8.3166))
        Line((-1.8814, -8.3166), (-1.671, -8.4407))
        Line((-1.671, -8.4407), (-1.4684, -8.5579))
        Line((-1.4684, -8.5579), (-1.2736, -8.6682))
        Line((-1.2736, -8.6682), (-1.0865, -8.7717))
        Line((-1.0865, -8.7717), (-0.9071, -8.8682))
        Line((-0.9071, -8.8682), (-0.7356, -8.9579))
        Line((-0.7356, -8.9579), (-0.5718, -9.0406))
        Line((-0.5718, -9.0406), (-0.4157, -9.1165))
        Line((-0.4157, -9.1165), (-0.2637, -9.1877))
        Line((-0.2637, -9.1877), (-0.1119, -9.2565))
        Line((-0.1119, -9.2565), (0.0396, -9.3229))
        Line((0.0396, -9.3229), (0.1908, -9.3868))
        Line((0.1908, -9.3868), (0.3417, -9.4484))
        Line((0.3417, -9.4484), (0.4924, -9.5076))
        Line((0.4924, -9.5076), (0.6428, -9.5643))
        Line((0.6428, -9.5643), (0.793, -9.6186))
        Line((0.793, -9.6186), (0.9428, -9.6705))
        Line((0.9428, -9.6705), (1.0924, -9.72))
        Line((1.0924, -9.72), (1.2417, -9.7671))
        Line((1.2417, -9.7671), (1.3908, -9.8117))
        Line((1.3908, -9.8117), (1.5396, -9.854))
        Line((1.5396, -9.854), (1.6881, -9.8938))
        Line((1.6881, -9.8938), (1.8363, -9.9312))
        Line((1.8363, -9.9312), (1.9848, -9.9663))
        Line((1.9848, -9.9663), (2.1339, -9.9988))
        Line((2.1339, -9.9988), (2.2838, -10.029))
        Line((2.2838, -10.029), (2.4344, -10.0568))
        Line((2.4344, -10.0568), (2.5857, -10.0821))
        Line((2.5857, -10.0821), (2.7376, -10.1051))
        Line((2.7376, -10.1051), (2.8903, -10.1256))
        Line((2.8903, -10.1256), (3.0437, -10.1437))
        Line((3.0437, -10.1437), (3.1978, -10.1594))
        Line((3.1978, -10.1594), (3.3526, -10.1727))
        Line((3.3526, -10.1727), (3.5081, -10.1835))
        Line((3.5081, -10.1835), (3.6643, -10.192))
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

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
    _bore_ax = _gAx2(_gPnt(0.0001, -37.5007, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6244, 9.0)
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
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
