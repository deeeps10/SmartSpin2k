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
        Line((0.6, 61.4), (-9.0, 61.4))
        Line((-9.0, 61.4), (-9.0, 58.0))
        RadiusArc((-9.0, 58.0), (-6.0, 55.0102), -2.9846)
        Line((-6.0, 55.0102), (-6.0, -70.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -70.0), (-0.0, -76.0), -6.0)
        RadiusArc((-0.0, -76.0), (6.0, -70.0), -6.0)
        Line((6.0, -70.0), (6.0, 55.0102))
        RadiusArc((6.0, 55.0102), (9.0, 58.0), -2.9709)
        Line((9.0, 58.0), (9.0, 70.0))
        Line((9.0, 70.0), (-9.0, 70.0))
        Line((-9.0, 70.0), (-9.0, 66.6))
        Line((-9.0, 66.6), (0.6, 66.6))
        Line((0.6, 66.6), (0.6, 66.4966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 66.4966), (3.9, 64.0), 2.5944)
        RadiusArc((3.9, 64.0), (0.6, 61.5034), 2.5944)
        Line((0.6, 61.5034), (0.6, 61.4))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 22 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-4.4029, -4.2402), (-3.8186, -5.2546), -2.2913)
        RadiusArc((-3.8186, -5.2546), (-2.7528, -5.9117), -3.1971)
        RadiusArc((-2.7528, -5.9117), (-1.2034, -6.2493), -6.7023)
        RadiusArc((-1.2034, -6.2493), (-0.1008, -6.3007), -11.4332)
        RadiusArc((-0.1008, -6.3007), (1.2283, -6.2179), -9.3942)
        RadiusArc((1.2283, -6.2179), (2.5189, -5.9003), -6.2901)
        RadiusArc((2.5189, -5.9003), (3.4759, -5.3444), -3.0905)
        RadiusArc((3.4759, -5.3444), (4.078, -4.5518), -2.3158)
        RadiusArc((4.078, -4.5518), (4.3038, -3.7108), -2.7681)
        RadiusArc((4.3038, -3.7108), (4.2785, -2.7641), -3.4456)
        RadiusArc((4.2785, -2.7641), (3.9153, -1.8092), -2.4438)
        RadiusArc((3.9153, -1.8092), (3.1887, -1.0856), -2.6127)
        RadiusArc((3.1887, -1.0856), (2.1175, -0.5993), -4.1289)
        RadiusArc((2.1175, -0.5993), (0.9757, -0.3798), -6.8106)
        RadiusArc((0.9757, -0.3798), (-0.3882, -0.329), -11.1305)
        RadiusArc((-0.3882, -0.329), (-1.6818, -0.4415), -8.8407)
        RadiusArc((-1.6818, -0.4415), (-2.7367, -0.7147), -5.2347)
        RadiusArc((-2.7367, -0.7147), (-3.5527, -1.149), -3.1922)
        RadiusArc((-3.5527, -1.149), (-4.0292, -1.6196), -2.3186)
        RadiusArc((-4.0292, -1.6196), (-4.4015, -2.3672), -2.3815)
        RadiusArc((-4.4015, -2.3672), (-4.5256, -3.2918), -3.3959)
        RadiusArc((-4.5256, -3.2918), (-4.4029, -4.2402), -3.2774)
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
        Line((1.3883, -6.9926), (2.255, -6.9926))
        Line((2.255, -6.9926), (2.255, -8.201))
        Line((2.255, -8.201), (4.2019, -8.201))
        Line((4.2019, -8.201), (4.2019, -9.2385))
        Line((4.2019, -9.2385), (2.255, -9.2385))
        Line((2.255, -9.2385), (2.255, -13.291))
        Line((2.255, -13.291), (1.4005, -13.291))
        Line((1.4005, -13.291), (-4.3974, -9.3545))
        Line((-4.3974, -9.3545), (-4.3974, -8.201))
        Line((-4.3974, -8.201), (1.3883, -8.201))
        Line((1.3883, -8.201), (1.3883, -6.9926))
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

# 'Sketch3': 6 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        Line((0.8146, -11.8507), (1.2663, -12.1803))
        Line((1.2663, -12.1803), (1.3883, -12.2779))
        Line((1.3883, -12.2779), (1.3883, -9.2385))
        Line((1.3883, -9.2385), (-3.1585, -9.2385))
        RadiusArc((-3.1585, -9.2385), (-2.4322, -9.6475), 8.4181)
        Line((-2.4322, -9.6475), (0.8146, -11.8507))
    _inc_edges_sk_Sketch3_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_5 = Wire.combine(_inc_edges_sk_Sketch3_5)[0]
_wire_sk_Sketch3_5 = _wire_sk_Sketch3_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch3_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch3_5.wrapped, True)
_face_sk_Sketch3_5 = Face(_mkf_sk_Sketch3_5.Face())

# 'Sketch3': 22 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch3_6:
    with BuildLine():
        RadiusArc((-2.696, -4.8045), (-1.9816, -5.0306), -3.1466)
        RadiusArc((-1.9816, -5.0306), (-1.0311, -5.1584), -6.643)
        # Near-straight arc (sagitta=0.008868mm) replaced with Line
        Line((-1.0311, -5.1584), (-0.1008, -5.1899))
        RadiusArc((-0.1008, -5.1899), (1.0106, -5.1401), -11.9711)
        RadiusArc((1.0106, -5.1401), (2.0561, -4.9487), -6.2197)
        RadiusArc((2.0561, -4.9487), (2.5785, -4.7413), -2.9852)
        RadiusArc((2.5785, -4.7413), (3.0497, -4.3836), -1.7706)
        RadiusArc((3.0497, -4.3836), (3.3325, -3.9086), -1.3922)
        RadiusArc((3.3325, -3.9086), (3.4229, -3.1892), -1.7451)
        RadiusArc((3.4229, -3.1892), (3.2881, -2.6229), -1.6549)
        RadiusArc((3.2881, -2.6229), (2.9607, -2.172), -1.3938)
        RadiusArc((2.9607, -2.172), (2.4405, -1.836), -1.8312)
        RadiusArc((2.4405, -1.836), (1.5492, -1.571), -3.5494)
        RadiusArc((1.5492, -1.571), (0.5798, -1.4606), -7.2757)
        RadiusArc((0.5798, -1.4606), (-0.579, -1.4502), -12.7423)
        RadiusArc((-0.579, -1.4502), (-1.7947, -1.565), -9.6397)
        RadiusArc((-1.7947, -1.565), (-2.7984, -1.8729), -4.0139)
        RadiusArc((-2.7984, -1.8729), (-3.3335, -2.3064), -1.6394)
        RadiusArc((-3.3335, -2.3064), (-3.6011, -2.9187), -1.3989)
        RadiusArc((-3.6011, -2.9187), (-3.6016, -3.6742), -2.0784)
        RadiusArc((-3.6016, -3.6742), (-3.3379, -4.3021), -1.5528)
        RadiusArc((-3.3379, -4.3021), (-2.696, -4.8045), -1.5405)
    _inc_edges_sk_Sketch3_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_6 = Wire.combine(_inc_edges_sk_Sketch3_6)[0]
_wire_sk_Sketch3_6 = _wire_sk_Sketch3_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch3_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch3_6.wrapped, True)
_face_sk_Sketch3_6 = Face(_mkf_sk_Sketch3_6.Face())

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
    _bore_ax = _gAx2(_gPnt(0.0, -70.0001, -4.5), _gDir(-0.0, -0.0, 1.0))
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
    # -- Extrude3_p0 --
    _face = _face_sk_Sketch3_5
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude3_p1 --
    _face = _face_sk_Sketch3_6
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
