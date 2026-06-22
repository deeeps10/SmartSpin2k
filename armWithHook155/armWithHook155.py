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
        Line((0.6, 69.0034), (0.6, 68.9))
        Line((0.6, 68.9), (-9.0, 68.9))
        Line((-9.0, 68.9), (-9.0, 65.5))
        RadiusArc((-9.0, 65.5), (-6.0, 62.5102), -2.9779)
        Line((-6.0, 62.5102), (-6.0, -77.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -77.5), (-0.0, -83.5), -6.0)
        RadiusArc((-0.0, -83.5), (6.0, -77.5), -6.0)
        Line((6.0, -77.5), (6.0, 62.5102))
        RadiusArc((6.0, 62.5102), (9.0, 65.5), -2.9709)
        Line((9.0, 65.5), (9.0, 77.5))
        Line((9.0, 77.5), (-9.0, 77.5))
        Line((-9.0, 77.5), (-9.0, 74.1))
        Line((-9.0, 74.1), (0.6, 74.1))
        Line((0.6, 74.1), (0.6, 73.9966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 73.9966), (3.9, 71.5), 2.5944)
        RadiusArc((3.9, 71.5), (0.6, 69.0034), 2.5944)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 11 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((-4.4621, -16.119), (3.2034, -16.119))
        Line((3.2034, -16.119), (3.2034, -14.0257))
        Line((3.2034, -14.0257), (4.1372, -14.0257))
        Line((4.1372, -14.0257), (4.1372, -19.4147))
        Line((4.1372, -19.4147), (3.2034, -19.4147))
        Line((3.2034, -19.4147), (3.2034, -17.2237))
        Line((3.2034, -17.2237), (-3.4124, -17.2237))
        Line((-3.4124, -17.2237), (-2.027, -19.1645))
        Line((-2.027, -19.1645), (-3.0645, -19.1645))
        Line((-3.0645, -19.1645), (-4.4621, -17.1321))
        Line((-4.4621, -17.1321), (-4.4621, -16.119))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 46 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((2.0865, -11.9666), (2.2147, -13.0774))
        RadiusArc((2.2147, -13.0774), (2.9353, -12.8342), -2.8464)
        RadiusArc((2.9353, -12.8342), (3.421, -12.4988), -2.2235)
        RadiusArc((3.421, -12.4988), (3.802, -12.0431), -2.2216)
        RadiusArc((3.802, -12.0431), (4.1099, -11.3534), -2.605)
        RadiusArc((4.1099, -11.3534), (4.2499, -10.5152), -3.6386)
        RadiusArc((4.2499, -10.5152), (4.228, -9.6542), -5.0019)
        RadiusArc((4.228, -9.6542), (4.0891, -9.0005), -3.6039)
        RadiusArc((4.0891, -9.0005), (3.7593, -8.3085), -2.7854)
        RadiusArc((3.7593, -8.3085), (3.2594, -7.7587), -2.5181)
        RadiusArc((3.2594, -7.7587), (2.7603, -7.4424), -2.474)
        RadiusArc((2.7603, -7.4424), (2.0205, -7.2087), -2.8054)
        RadiusArc((2.0205, -7.2087), (1.3359, -7.1512), -3.7895)
        RadiusArc((1.3359, -7.1512), (0.73, -7.2048), -3.3744)
        RadiusArc((0.73, -7.2048), (0.063, -7.4226), -2.6513)
        RadiusArc((0.063, -7.4226), (-0.5036, -7.8078), -2.4152)
        RadiusArc((-0.5036, -7.8078), (-0.873, -8.2271), -2.4947)
        RadiusArc((-0.873, -8.2271), (-1.1873, -8.8557), -2.4662)
        RadiusArc((-1.1873, -8.8557), (-1.3362, -9.5988), -3.0241)
        RadiusArc((-1.3362, -9.5988), (-1.3397, -10.2052), -4.0175)
        RadiusArc((-1.3397, -10.2052), (-1.2616, -10.7221), -3.2977)
        RadiusArc((-1.2616, -10.7221), (-1.1054, -11.1895), -2.7426)
        RadiusArc((-1.1054, -11.1895), (-0.8002, -11.7042), -2.4996)
        Line((-0.8002, -11.7042), (-3.5284, -11.5395))
        Line((-3.5284, -11.5395), (-3.5284, -7.6517))
        Line((-3.5284, -7.6517), (-4.4621, -7.6517))
        Line((-4.4621, -7.6517), (-4.4621, -12.5403))
        Line((-4.4621, -12.5403), (0.164, -12.8272))
        Line((0.164, -12.8272), (0.164, -11.753))
        RadiusArc((0.164, -11.753), (-0.1684, -11.2916), 2.9762)
        RadiusArc((-0.1684, -11.2916), (-0.3469, -10.8804), 2.0588)
        RadiusArc((-0.3469, -10.8804), (-0.4499, -10.2717), 2.3334)
        # Near-straight arc (sagitta=0.009152mm) replaced with Line
        Line((-0.4499, -10.2717), (-0.4323, -9.8263))
        RadiusArc((-0.4323, -9.8263), (-0.2299, -9.1707), 1.9593)
        RadiusArc((-0.2299, -9.1707), (0.1162, -8.7312), 1.6178)
        RadiusArc((0.1162, -8.7312), (0.583, -8.4358), 1.6294)
        RadiusArc((0.583, -8.4358), (1.1493, -8.3015), 1.8361)
        RadiusArc((1.1493, -8.3015), (1.8298, -8.3281), 2.7919)
        RadiusArc((1.8298, -8.3281), (2.3369, -8.4727), 2.193)
        RadiusArc((2.3369, -8.4727), (2.8281, -8.793), 1.7226)
        RadiusArc((2.8281, -8.793), (3.1248, -9.1694), 1.5895)
        RadiusArc((3.1248, -9.1694), (3.3597, -10.0621), 1.8354)
        RadiusArc((3.3597, -10.0621), (3.3111, -10.7108), 2.8098)
        RadiusArc((3.3111, -10.7108), (3.0843, -11.2848), 1.7549)
        RadiusArc((3.0843, -11.2848), (2.6761, -11.7034), 1.4437)
        RadiusArc((2.6761, -11.7034), (2.0865, -11.9666), 1.8684)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch2': 46 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch2_4:
    with BuildLine():
        Line((2.0865, -5.1777), (2.2147, -6.2885))
        RadiusArc((2.2147, -6.2885), (2.9353, -6.0453), -2.8464)
        RadiusArc((2.9353, -6.0453), (3.421, -5.7099), -2.2235)
        RadiusArc((3.421, -5.7099), (3.802, -5.2542), -2.2216)
        RadiusArc((3.802, -5.2542), (4.1099, -4.5645), -2.605)
        RadiusArc((4.1099, -4.5645), (4.2499, -3.7263), -3.6386)
        RadiusArc((4.2499, -3.7263), (4.228, -2.8653), -5.0019)
        RadiusArc((4.228, -2.8653), (4.0891, -2.2115), -3.6039)
        RadiusArc((4.0891, -2.2115), (3.7593, -1.5195), -2.7854)
        RadiusArc((3.7593, -1.5195), (3.2594, -0.9698), -2.5181)
        RadiusArc((3.2594, -0.9698), (2.7603, -0.6535), -2.474)
        RadiusArc((2.7603, -0.6535), (2.0205, -0.4198), -2.8054)
        RadiusArc((2.0205, -0.4198), (1.3359, -0.3623), -3.7895)
        RadiusArc((1.3359, -0.3623), (0.73, -0.4159), -3.3744)
        RadiusArc((0.73, -0.4159), (0.063, -0.6337), -2.6513)
        RadiusArc((0.063, -0.6337), (-0.5036, -1.0189), -2.4152)
        RadiusArc((-0.5036, -1.0189), (-0.873, -1.4382), -2.4947)
        RadiusArc((-0.873, -1.4382), (-1.1873, -2.0667), -2.4662)
        RadiusArc((-1.1873, -2.0667), (-1.3362, -2.8099), -3.0241)
        RadiusArc((-1.3362, -2.8099), (-1.3397, -3.4163), -4.0175)
        RadiusArc((-1.3397, -3.4163), (-1.2616, -3.9332), -3.2977)
        RadiusArc((-1.2616, -3.9332), (-1.1054, -4.4006), -2.7426)
        RadiusArc((-1.1054, -4.4006), (-0.8002, -4.9153), -2.4996)
        Line((-0.8002, -4.9153), (-3.5284, -4.7506))
        Line((-3.5284, -4.7506), (-3.5284, -0.8628))
        Line((-3.5284, -0.8628), (-4.4621, -0.8628))
        Line((-4.4621, -0.8628), (-4.4621, -5.7514))
        Line((-4.4621, -5.7514), (0.164, -6.0383))
        Line((0.164, -6.0383), (0.164, -4.9641))
        RadiusArc((0.164, -4.9641), (-0.1684, -4.5027), 2.9762)
        RadiusArc((-0.1684, -4.5027), (-0.3469, -4.0915), 2.0588)
        RadiusArc((-0.3469, -4.0915), (-0.4499, -3.4828), 2.3334)
        # Near-straight arc (sagitta=0.009156mm) replaced with Line
        Line((-0.4499, -3.4828), (-0.4323, -3.0373))
        RadiusArc((-0.4323, -3.0373), (-0.2299, -2.3818), 1.9593)
        RadiusArc((-0.2299, -2.3818), (0.1162, -1.9422), 1.6178)
        RadiusArc((0.1162, -1.9422), (0.583, -1.6468), 1.6294)
        RadiusArc((0.583, -1.6468), (1.1493, -1.5125), 1.8361)
        RadiusArc((1.1493, -1.5125), (1.8298, -1.5392), 2.7919)
        RadiusArc((1.8298, -1.5392), (2.3369, -1.6838), 2.193)
        RadiusArc((2.3369, -1.6838), (2.8281, -2.0041), 1.7226)
        RadiusArc((2.8281, -2.0041), (3.1248, -2.3805), 1.5895)
        RadiusArc((3.1248, -2.3805), (3.3597, -3.2732), 1.8354)
        RadiusArc((3.3597, -3.2732), (3.3111, -3.9219), 2.8098)
        RadiusArc((3.3111, -3.9219), (3.0843, -4.4959), 1.7549)
        RadiusArc((3.0843, -4.4959), (2.6761, -4.9145), 1.4437)
        RadiusArc((2.6761, -4.9145), (2.0865, -5.1777), 1.8684)
    _inc_edges_sk_Sketch2_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_4 = Wire.combine(_inc_edges_sk_Sketch2_4)[0]
_wire_sk_Sketch2_4 = _wire_sk_Sketch2_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch2_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch2_4.wrapped, True)
_face_sk_Sketch2_4 = Face(_mkf_sk_Sketch2_4.Face())

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
    _bore_ax = _gAx2(_gPnt(-0.0003, -77.5001, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6253, 9.0)
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
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
