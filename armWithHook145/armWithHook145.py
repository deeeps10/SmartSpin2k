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
        Line((0.6, 64.0034), (0.6, 63.9))
        Line((0.6, 63.9), (-9.0, 63.9))
        Line((-9.0, 63.9), (-9.0, 60.5))
        RadiusArc((-9.0, 60.5), (-6.0, 57.5102), -2.9846)
        Line((-6.0, 57.5102), (-6.0, -72.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -72.5), (-0.0, -78.5), -6.0)
        RadiusArc((-0.0, -78.5), (6.0, -72.5), -6.0)
        Line((6.0, -72.5), (6.0, 57.5102))
        RadiusArc((6.0, 57.5102), (9.0, 60.5), -2.9846)
        Line((9.0, 60.5), (9.0, 72.5))
        Line((9.0, 72.5), (-9.0, 72.5))
        Line((-9.0, 72.5), (-9.0, 69.1))
        Line((-9.0, 69.1), (0.6, 69.1))
        Line((0.6, 69.1), (0.6, 68.9966))
        # Arc split: sweep=211.59deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 68.9966), (3.9006, 66.5), 2.5945)
        RadiusArc((3.9006, 66.5), (0.6, 64.0034), 2.5945)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 44 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((3.1421, -5.9256), (3.6623, -5.4507), -2.1731)
        RadiusArc((3.6623, -5.4507), (3.9769, -4.9279), -2.4092)
        RadiusArc((3.9769, -4.9279), (4.2219, -4.0222), -3.0769)
        RadiusArc((4.2219, -4.0222), (4.2453, -3.0426), -4.7669)
        RadiusArc((4.2453, -3.0426), (3.978, -1.9181), -3.6172)
        RadiusArc((3.978, -1.9181), (3.5787, -1.2816), -2.5985)
        RadiusArc((3.5787, -1.2816), (2.8928, -0.7217), -2.5064)
        RadiusArc((2.8928, -0.7217), (2.0205, -0.4198), -2.7275)
        RadiusArc((2.0205, -0.4198), (1.1784, -0.3656), -3.8414)
        RadiusArc((1.1784, -0.3656), (0.4511, -0.4829), -3.0954)
        RadiusArc((0.4511, -0.4829), (-0.0583, -0.6973), -2.4985)
        RadiusArc((-0.0583, -0.6973), (-0.6049, -1.116), -2.4378)
        RadiusArc((-0.6049, -1.116), (-1.0186, -1.6758), -2.4327)
        RadiusArc((-1.0186, -1.6758), (-1.2667, -2.3503), -2.6092)
        RadiusArc((-1.2667, -2.3503), (-1.3495, -3.1392), -3.4499)
        # Near-straight arc (sagitta=0.009988mm) replaced with Line
        Line((-1.3495, -3.1392), (-1.3104, -3.6809))
        RadiusArc((-1.3104, -3.6809), (-1.1517, -4.2884), -2.9325)
        RadiusArc((-1.1517, -4.2884), (-0.8002, -4.9153), -2.5166)
        Line((-0.8002, -4.9153), (-3.5284, -4.7505))
        Line((-3.5284, -4.7505), (-3.5284, -0.8628))
        Line((-3.5284, -0.8628), (-4.4621, -0.8628))
        Line((-4.4621, -0.8628), (-4.4621, -5.7514))
        Line((-4.4621, -5.7514), (0.164, -6.0382))
        Line((0.164, -6.0382), (0.164, -4.9641))
        # Near-straight arc (sagitta=0.006015mm) replaced with Line
        Line((0.164, -4.9641), (-0.0732, -4.6565))
        # Near-straight arc (sagitta=0.008996mm) replaced with Line
        Line((-0.0732, -4.6565), (-0.2711, -4.2976))
        RadiusArc((-0.2711, -4.2976), (-0.4018, -3.8777), 2.0907)
        RadiusArc((-0.4018, -3.8777), (-0.4524, -3.3651), 2.478)
        # Near-straight arc (sagitta=0.00915mm) replaced with Line
        Line((-0.4524, -3.3651), (-0.4168, -2.9343))
        RadiusArc((-0.4168, -2.9343), (-0.2299, -2.3818), 1.885)
        RadiusArc((-0.2299, -2.3818), (0.1162, -1.9422), 1.6189)
        RadiusArc((0.1162, -1.9422), (0.4982, -1.6849), 1.6305)
        RadiusArc((0.4982, -1.6849), (1.1493, -1.5125), 1.7901)
        RadiusArc((1.1493, -1.5125), (1.8298, -1.5392), 2.7943)
        RadiusArc((1.8298, -1.5392), (2.4277, -1.726), 2.1502)
        RadiusArc((2.4277, -1.726), (2.961, -2.1444), 1.6836)
        RadiusArc((2.961, -2.1444), (3.2458, -2.6473), 1.5937)
        RadiusArc((3.2458, -2.6473), (3.3597, -3.2732), 2.0456)
        # Near-straight arc (sagitta=0.009945mm) replaced with Line
        Line((3.3597, -3.2732), (3.3394, -3.7541))
        RadiusArc((3.3394, -3.7541), (3.158, -4.3669), 1.9579)
        RadiusArc((3.158, -4.3669), (2.7952, -4.8244), 1.4521)
        RadiusArc((2.7952, -4.8244), (2.0865, -5.1777), 1.7531)
        Line((2.0865, -5.1777), (2.2147, -6.2884))
        RadiusArc((2.2147, -6.2884), (3.1421, -5.9256), -2.7421)
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 11 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((1.3236, -6.9926), (2.1903, -6.9926))
        Line((2.1903, -6.9926), (2.1903, -8.201))
        Line((2.1903, -8.201), (4.1372, -8.201))
        Line((4.1372, -8.201), (4.1372, -9.2385))
        Line((4.1372, -9.2385), (2.1903, -9.2385))
        Line((2.1903, -9.2385), (2.1903, -13.291))
        Line((2.1903, -13.291), (1.3359, -13.291))
        Line((1.3359, -13.291), (-4.4621, -9.3545))
        Line((-4.4621, -9.3545), (-4.4621, -8.201))
        Line((-4.4621, -8.201), (1.3236, -8.201))
        Line((1.3236, -8.201), (1.3236, -6.9926))
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
    _inc_edges_sk_Sketch2_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_4 = Wire.combine(_inc_edges_sk_Sketch2_4)[0]
_wire_sk_Sketch2_4 = _wire_sk_Sketch2_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch2_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch2_4.wrapped, True)
_face_sk_Sketch2_4 = Face(_mkf_sk_Sketch2_4.Face())

# 'Sketch3': 7 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        Line((-2.4969, -9.6475), (0.7499, -11.8507))
        Line((0.7499, -11.8507), (1.2015, -12.1803))
        Line((1.2015, -12.1803), (1.3236, -12.2779))
        Line((1.3236, -12.2779), (1.3236, -9.2385))
        Line((1.3236, -9.2385), (-3.2232, -9.2385))
        # Near-straight arc (sagitta=0.005218mm) replaced with Line
        Line((-3.2232, -9.2385), (-2.8622, -9.4302))
        # Near-straight arc (sagitta=0.003126mm) replaced with Line
        Line((-2.8622, -9.4302), (-2.4969, -9.6475))
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
    _bore_ax = _gAx2(_gPnt(-0.0003, -72.5002, -4.5), _gDir(-0.0, -0.0, 1.0))
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
