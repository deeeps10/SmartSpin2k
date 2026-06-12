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
        Line((0.6, 4.0034), (0.6, 3.9))
        Line((0.6, 3.9), (-9.0, 3.9))
        Line((-9.0, 3.9), (-9.0, 0.5))
        RadiusArc((-9.0, 0.5), (-6.0, -2.4897), -2.9546)
        Line((-6.0, -2.4897), (-6.0, -12.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -12.5), (-0.0, -18.5), -6.0)
        RadiusArc((-0.0, -18.5), (6.0, -12.5), -6.0)
        Line((6.0, -12.5), (6.0, -2.4897))
        RadiusArc((6.0, -2.4897), (9.0, 0.5), -2.9546)
        Line((9.0, 0.5), (9.0, 12.5))
        Line((9.0, 12.5), (-9.0, 12.5))
        Line((-9.0, 12.5), (-9.0, 9.1))
        Line((-9.0, 9.1), (0.6, 9.1))
        Line((0.6, 9.1), (0.6, 8.9966))
        # Arc split: sweep=211.65deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 8.9966), (3.9027, 6.5), 2.595)
        RadiusArc((3.9027, 6.5), (0.6, 4.0034), 2.595)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 29 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((1.4216, -10.1181), (2.5024, -11.2493), -8.1502)
        RadiusArc((2.5024, -11.2493), (3.2681, -11.7408), -2.9694)
        Line((3.2681, -11.7408), (3.2681, -7.255))
        Line((3.2681, -7.255), (4.2019, -7.255))
        Line((4.2019, -7.255), (4.2019, -12.9492))
        Line((4.2019, -12.9492), (3.4268, -12.9492))
        RadiusArc((3.4268, -12.9492), (2.623, -12.5215), 5.7145)
        Line((2.623, -12.5215), (0.0439, -9.8818))
        RadiusArc((0.0439, -9.8818), (-0.9207, -8.9368), -4.945)
        RadiusArc((-0.9207, -8.9368), (-2.0636, -8.5194), -1.9032)
        RadiusArc((-2.0636, -8.5194), (-2.7981, -8.6364), -1.653)
        RadiusArc((-2.7981, -8.6364), (-3.1783, -8.8799), -1.234)
        RadiusArc((-3.1783, -8.8799), (-3.579, -9.6305), -1.4542)
        RadiusArc((-3.579, -9.6305), (-3.6071, -10.3577), -2.4174)
        RadiusArc((-3.6071, -10.3577), (-3.2408, -11.2434), -1.6148)
        RadiusArc((-3.2408, -11.2434), (-2.7463, -11.6217), -1.4323)
        # Near-straight arc (sagitta=0.005319mm) replaced with Line
        Line((-2.7463, -11.6217), (-2.4996, -11.7128))
        Line((-2.4996, -11.7128), (-2.1594, -11.7128))
        Line((-2.1594, -11.7128), (-2.1594, -12.8439))
        Line((-2.1594, -12.8439), (-2.5949, -12.8439))
        RadiusArc((-2.5949, -12.8439), (-3.9152, -12.0246), 2.3232)
        RadiusArc((-3.9152, -12.0246), (-4.4578, -10.8284), 2.6692)
        RadiusArc((-4.4578, -10.8284), (-4.4574, -9.287), 4.3962)
        RadiusArc((-4.4574, -9.287), (-3.7395, -7.9142), 2.28)
        RadiusArc((-3.7395, -7.9142), (-2.1698, -7.3893), 2.4355)
        RadiusArc((-2.1698, -7.3893), (-0.9772, -7.7186), 2.5356)
        RadiusArc((-0.9772, -7.7186), (-0.1862, -8.2986), 4.1911)
        RadiusArc((-0.1862, -8.2986), (0.7143, -9.267), 13.3617)
        # Near-straight arc (sagitta=0.003222mm) replaced with Line
        Line((0.7143, -9.267), (1.4216, -10.1181))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 26 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((2.2794, -6.2884), (2.1512, -5.1777))
        RadiusArc((2.1512, -5.1777), (3.149, -4.4959), -1.5757)
        RadiusArc((3.149, -4.4959), (3.4268, -3.3895), -2.145)
        RadiusArc((3.4268, -3.3895), (3.0851, -2.2197), -1.7816)
        RadiusArc((3.0851, -2.2197), (2.4924, -1.726), -1.6628)
        RadiusArc((2.4924, -1.726), (1.214, -1.5125), -2.537)
        RadiusArc((1.214, -1.5125), (0.2518, -1.8818), -1.7861)
        RadiusArc((0.2518, -1.8818), (-0.2787, -2.644), -1.6399)
        RadiusArc((-0.2787, -2.644), (-0.3571, -3.7676), -2.3531)
        RadiusArc((-0.3571, -3.7676), (0.2287, -4.9641), -2.234)
        Line((0.2287, -4.9641), (0.2287, -6.0382))
        Line((0.2287, -6.0382), (-4.3974, -5.7514))
        Line((-4.3974, -5.7514), (-4.3974, -0.8628))
        Line((-4.3974, -0.8628), (-3.4637, -0.8628))
        Line((-3.4637, -0.8628), (-3.4637, -4.7505))
        Line((-3.4637, -4.7505), (-0.7355, -4.9153))
        RadiusArc((-0.7355, -4.9153), (-1.2815, -2.9723), 3.058)
        RadiusArc((-1.2815, -2.9723), (-0.6362, -1.2188), 2.6522)
        RadiusArc((-0.6362, -1.2188), (1.243, -0.3656), 2.5209)
        RadiusArc((1.243, -0.3656), (3.0848, -0.7972), 3.0557)
        RadiusArc((3.0848, -0.7972), (4.0427, -1.9181), 2.5361)
        RadiusArc((4.0427, -1.9181), (4.2866, -4.0222), 4.2511)
        RadiusArc((4.2866, -4.0222), (3.9297, -5.1495), 2.9741)
        RadiusArc((3.9297, -5.1495), (3.3969, -5.7866), 2.2339)
        RadiusArc((3.3969, -5.7866), (2.7765, -6.1457), 2.341)
        RadiusArc((2.7765, -6.1457), (2.2794, -6.2884), 2.8914)
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
    _bore_ax = _gAx2(_gPnt(-0.0004, -12.5024, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6226, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50000012 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50000012 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
