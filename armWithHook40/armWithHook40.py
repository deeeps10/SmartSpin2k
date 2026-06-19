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
        Line((0.6, 11.4), (-9.0, 11.4))
        Line((-9.0, 11.4), (-9.0, 8.0))
        RadiusArc((-9.0, 8.0), (-6.0, 5.0103), -2.9863)
        Line((-6.0, 5.0103), (-6.0, -20.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -20.0), (-0.0, -26.0), -6.0)
        RadiusArc((-0.0, -26.0), (6.0, -20.0), -6.0)
        Line((6.0, -20.0), (6.0, 5.0103))
        RadiusArc((6.0, 5.0103), (9.0, 8.0), -2.9707)
        Line((9.0, 8.0), (9.0, 20.0))
        Line((9.0, 20.0), (-9.0, 20.0))
        Line((-9.0, 20.0), (-9.0, 16.6))
        Line((-9.0, 16.6), (0.6, 16.6))
        Line((0.6, 16.6), (0.6, 16.4966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 16.4966), (3.9, 14.0), 2.5944)
        RadiusArc((3.9, 14.0), (0.6, 11.5034), 2.5944)
        Line((0.6, 11.5034), (0.6, 11.4))
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
        Line((-4.3974, -8.201), (1.3883, -8.201))
        Line((1.3883, -8.201), (1.3883, -6.9926))
        Line((1.3883, -6.9926), (2.255, -6.9926))
        Line((2.255, -6.9926), (2.255, -8.201))
        Line((2.255, -8.201), (4.2019, -8.201))
        Line((4.2019, -8.201), (4.2019, -9.2385))
        Line((4.2019, -9.2385), (2.255, -9.2385))
        Line((2.255, -9.2385), (2.255, -13.291))
        Line((2.255, -13.291), (1.4005, -13.291))
        Line((1.4005, -13.291), (-4.3974, -9.3545))
        Line((-4.3974, -9.3545), (-4.3974, -8.201))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch3': 6 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((0.8146, -11.8507), (1.2663, -12.1803))
        Line((1.2663, -12.1803), (1.3883, -12.2779))
        Line((1.3883, -12.2779), (1.3883, -9.2385))
        Line((1.3883, -9.2385), (-3.1585, -9.2385))
        Line((-3.1585, -9.2385), (-2.4322, -9.6475))
        Line((-2.4322, -9.6475), (0.8146, -11.8507))
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch4': 13 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        RadiusArc((-3.2687, -5.6707), (-1.4544, -6.2203), -4.5981)
        RadiusArc((-1.4544, -6.2203), (0.9809, -6.2477), -10.5701)
        RadiusArc((0.9809, -6.2477), (2.8753, -5.7414), -5.887)
        RadiusArc((2.8753, -5.7414), (4.0028, -4.7003), -2.4794)
        RadiusArc((4.0028, -4.7003), (4.2785, -2.7641), -3.2272)
        RadiusArc((4.2785, -2.7641), (3.3351, -1.1901), -2.4367)
        RadiusArc((3.3351, -1.1901), (2.1175, -0.5993), -4.016)
        RadiusArc((2.1175, -0.5993), (-0.1008, -0.3258), -8.8406)
        RadiusArc((-0.1008, -0.3258), (-2.1324, -0.5315), -7.9667)
        RadiusArc((-2.1324, -0.5315), (-3.4087, -1.049), -3.8827)
        RadiusArc((-3.4087, -1.049), (-4.3469, -2.2035), -2.298)
        RadiusArc((-4.3469, -2.2035), (-4.4029, -4.2402), -3.5536)
        RadiusArc((-4.4029, -4.2402), (-3.2687, -5.6707), -2.3515)
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch5': 12 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with BuildLine():
        RadiusArc((-3.6011, -2.9187), (-3.4551, -4.1131), -2.0083)
        RadiusArc((-3.4551, -4.1131), (-2.696, -4.8045), -1.5263)
        RadiusArc((-2.696, -4.8045), (-1.8103, -5.064), -3.865)
        RadiusArc((-1.8103, -5.064), (0.5922, -5.1719), -12.2121)
        RadiusArc((0.5922, -5.1719), (2.6878, -4.6792), -4.7261)
        RadiusArc((2.6878, -4.6792), (3.4229, -3.1892), -1.5353)
        RadiusArc((3.4229, -3.1892), (2.8722, -2.0956), -1.4669)
        RadiusArc((2.8722, -2.0956), (2.032, -1.6854), -2.6061)
        RadiusArc((2.032, -1.6854), (-0.1008, -1.4426), -7.2377)
        RadiusArc((-0.1008, -1.4426), (-2.2796, -1.674), -8.7909)
        RadiusArc((-2.2796, -1.674), (-3.0065, -1.9975), -2.198)
        RadiusArc((-3.0065, -1.9975), (-3.6011, -2.9187), -1.44)
    _inc_edges_sk_Sketch5_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_5 = Wire.combine(_inc_edges_sk_Sketch5_5)[0]
_wire_sk_Sketch5_5 = _wire_sk_Sketch5_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch5_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch5_5.wrapped, True)
_face_sk_Sketch5_5 = Face(_mkf_sk_Sketch5_5.Face())

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
    _bore_ax = _gAx2(_gPnt(-0.0001, -20.0, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6251, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50000012 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.50000012 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_5
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
