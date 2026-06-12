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
    origin=Vector(0.0, 0.0, -45.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((6.0, 15.0341), (6.0, 14.0))
        Line((6.0, 14.0), (-90.0, 14.0))
        Line((-90.0, 14.0), (-90.0, -20.0))
        RadiusArc((-90.0, -20.0), (-60.0, -49.8975), -29.7073)
        Line((-60.0, -49.8975), (-60.0, -100.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -100.0), (-0.0, -159.9996), -60.0)
        RadiusArc((-0.0, -159.9996), (60.0, -100.0), -60.0)
        Line((60.0, -100.0), (60.0, -49.8975))
        RadiusArc((60.0, -49.8975), (90.0, -20.0), -29.8888)
        Line((90.0, -20.0), (90.0, 100.0))
        Line((90.0, 100.0), (-90.0, 100.0))
        Line((-90.0, 100.0), (-90.0, 66.0))
        Line((-90.0, 66.0), (6.0, 66.0))
        Line((6.0, 66.0), (6.0, 64.9659))
        # Arc split: sweep=211.72deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 64.9659), (39.0464, 40.0), 25.9538)
        RadiusArc((39.0464, 40.0), (6.0, 15.0341), 25.9538)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 14 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (-21.5428,-85.1876)->(18.202,-118.9127) gap=52.125mm [LONG — verify geometry]
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-11.2606, -76.4154), (-35.4845, -77.6511), -24.9931)
        RadiusArc((-35.4845, -77.6511), (-43.0479, -87.4333), -20.4329)
        RadiusArc((-43.0479, -87.4333), (-44.8219, -106.877), -42.6698)
        RadiusArc((-44.8219, -106.877), (-39.1522, -120.246), -28.1103)
        RadiusArc((-39.1522, -120.246), (-22.7353, -129.004), -24.4307)
        Line((-22.7353, -129.004), (-21.6975, -117.774))
        RadiusArc((-21.6975, -117.774), (-26.8396, -116.4917), 18.792)
        RadiusArc((-26.8396, -116.4917), (-35.1384, -107.652), 14.508)
        RadiusArc((-35.1384, -107.652), (-34.1231, -91.7775), 20.8186)
        RadiusArc((-34.1231, -91.7775), (-28.6848, -86.6776), 12.4408)
        RadiusArc((-28.6848, -86.6776), (-21.5428, -85.1876), 17.2149)
        # auto-repair bridge: gap=?mm
        Line((-21.5428, -85.1876), (18.202, -118.9127))
        RadiusArc((18.202, -118.9127), (23.682, -111.3215), -26.2493)
        Line((23.682, -111.3215), (-11.2606, -76.4154))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 7 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((34.2679, -129.492), (42.0187, -129.492))
        Line((42.0187, -129.492), (42.0187, -72.55))
        Line((42.0187, -72.55), (32.6811, -72.55))
        Line((32.6811, -72.55), (32.6811, -117.408))
        RadiusArc((32.6811, -117.408), (23.682, -111.3215), 32.9133)
        RadiusArc((23.682, -111.3215), (18.1423, -118.856), 26.2493)
        RadiusArc((18.1423, -118.856), (34.2679, -129.492), -54.7539)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch3': 15 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        RadiusArc((-1.0083, -63.0067), (21.256, -60.3264), -84.9758)
        RadiusArc((21.256, -60.3264), (30.397, -56.5207), -41.3238)
        RadiusArc((30.397, -56.5207), (41.4326, -43.9674), -24.5368)
        RadiusArc((41.4326, -43.9674), (43.2391, -33.2843), -30.6846)
        RadiusArc((43.2391, -33.2843), (42.4318, -25.8891), -22.1476)
        RadiusArc((42.4318, -25.8891), (25.1083, -7.3441), -26.968)
        RadiusArc((25.1083, -7.3441), (4.5554, -3.3926), -68.1201)
        RadiusArc((4.5554, -3.3926), (-16.8179, -4.4148), -88.0127)
        RadiusArc((-16.8179, -4.4148), (-29.1901, -7.8864), -61.9466)
        RadiusArc((-29.1901, -7.8864), (-39.2496, -14.9132), -26.653)
        RadiusArc((-39.2496, -14.9132), (-45.0574, -29.0071), -23.9748)
        RadiusArc((-45.0574, -29.0071), (-42.8503, -45.6729), -34.2499)
        RadiusArc((-42.8503, -45.6729), (-31.0661, -57.5745), -23.4583)
        RadiusArc((-31.0661, -57.5745), (-23.5948, -60.4031), -40.5849)
        RadiusArc((-23.5948, -60.4031), (-1.0083, -63.0067), -92.5448)
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# 'Sketch4': 13 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch4_5:
    with BuildLine():
        RadiusArc((13.9404, -50.9221), (25.7845, -47.4133), -36.0274)
        RadiusArc((25.7845, -47.4133), (33.3253, -39.0862), -14.5955)
        RadiusArc((33.3253, -39.0862), (33.3049, -27.2694), -17.5893)
        RadiusArc((33.3049, -27.2694), (24.4049, -18.3598), -15.5423)
        RadiusArc((24.4049, -18.3598), (11.8619, -15.1483), -44.0623)
        RadiusArc((11.8619, -15.1483), (-1.0083, -14.4257), -110.9319)
        RadiusArc((-1.0083, -14.4257), (-17.9474, -15.6496), -100.2496)
        RadiusArc((-17.9474, -15.6496), (-26.8237, -18.174), -40.1859)
        RadiusArc((-26.8237, -18.174), (-36.3084, -34.2435), -15.1001)
        RadiusArc((-36.3084, -34.2435), (-26.9598, -48.0449), -15.298)
        RadiusArc((-26.9598, -48.0449), (-21.4331, -49.9326), -31.4453)
        RadiusArc((-21.4331, -49.9326), (-10.3111, -51.5843), -63.4586)
        RadiusArc((-10.3111, -51.5843), (13.9404, -50.9221), -113.1081)
    _inc_edges_sk_Sketch4_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_5 = Wire.combine(_inc_edges_sk_Sketch4_5)[0]
_wire_sk_Sketch4_5 = _wire_sk_Sketch4_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch4_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch4_5.wrapped, True)
_face_sk_Sketch4_5 = Face(_mkf_sk_Sketch4_5.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0058, -99.9859, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.265, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_5
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
