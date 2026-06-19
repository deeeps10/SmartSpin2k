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

# 'Sketch1': circle on inclined plane
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with Locations((-34.5, -34.4239)):
        Circle(radius=34.5)

# 'Sketch2': 5 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-33.0482, -68.4945), (-34.5, -67.3716), -1.5)
        RadiusArc((-34.5, -67.3716), (-35.9518, -68.4945), -1.5)
        RadiusArc((-35.9518, -68.4945), (-36.4642, -68.8679), 0.5)
        RadiusArc((-36.4642, -68.8679), (-32.5358, -68.8679), -2.0463)
        RadiusArc((-32.5358, -68.8679), (-33.0482, -68.4945), 0.5)
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 7 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((-35.854, -0.5697), (-35.301, -1.1926), -1.4999)
        RadiusArc((-35.301, -1.1926), (-34.5, -1.4187), -1.4574)
        RadiusArc((-34.5, -1.4187), (-33.6452, -1.157), -1.4575)
        RadiusArc((-33.6452, -1.157), (-33.1459, -0.5697), -1.4999)
        RadiusArc((-33.1459, -0.5697), (-32.1759, -0.0023), 1.0)
        RadiusArc((-32.1759, -0.0023), (-36.8241, -0.0023), -2.9932)
        RadiusArc((-36.8241, -0.0023), (-35.854, -0.5697), 1.0)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch3': 66 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 25.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        RadiusArc((4.6473, -31.4565), (5.1673, -33.0104), -2.0)
        Line((5.1673, -33.0104), (5.4161, -33.2381))
        RadiusArc((5.4161, -33.2381), (5.5999, -35.4458), 1.5)
        RadiusArc((5.5999, -35.4458), (4.9604, -36.1077), -5.3683)
        RadiusArc((4.9604, -36.1077), (4.6473, -37.3953), -2.0)
        Line((4.6473, -37.3953), (5.1265, -40.5247))
        Line((5.1265, -40.5247), (5.9327, -43.5862))
        Line((5.9327, -43.5862), (7.0571, -46.5457))
        Line((7.0571, -46.5457), (8.4871, -49.3701))
        Line((8.4871, -49.3701), (10.2068, -52.0282))
        Line((10.2068, -52.0282), (12.197, -54.4903))
        Line((12.197, -54.4903), (14.4356, -56.7289))
        Line((14.4356, -56.7289), (16.8977, -58.7191))
        Line((16.8977, -58.7191), (19.5558, -60.4388))
        Line((19.5558, -60.4388), (22.3802, -61.8688))
        Line((22.3802, -61.8688), (25.3397, -62.9932))
        Line((25.3397, -62.9932), (28.4012, -63.7994))
        Line((28.4012, -63.7994), (31.5306, -64.2786))
        RadiusArc((31.5306, -64.2786), (33.3122, -63.5098), -2.0)
        RadiusArc((33.3122, -63.5098), (35.6878, -63.5098), 1.5)
        RadiusArc((35.6878, -63.5098), (37.4694, -64.2786), -2.0)
        Line((37.4694, -64.2786), (40.5988, -63.7994))
        Line((40.5988, -63.7994), (43.6603, -62.9932))
        Line((43.6603, -62.9932), (46.6198, -61.8688))
        Line((46.6198, -61.8688), (49.4442, -60.4388))
        Line((49.4442, -60.4388), (52.1023, -58.7191))
        Line((52.1023, -58.7191), (54.5644, -56.7289))
        Line((54.5644, -56.7289), (56.803, -54.4903))
        Line((56.803, -54.4903), (58.7932, -52.0282))
        Line((58.7932, -52.0282), (60.5129, -49.3701))
        Line((60.5129, -49.3701), (61.9429, -46.5457))
        Line((61.9429, -46.5457), (63.0673, -43.5862))
        Line((63.0673, -43.5862), (63.8735, -40.5247))
        Line((63.8735, -40.5247), (64.3527, -37.3953))
        RadiusArc((64.3527, -37.3953), (63.5839, -35.6137), -2.0)
        RadiusArc((63.5839, -35.6137), (63.5839, -33.2381), 1.5)
        RadiusArc((63.5839, -33.2381), (64.3527, -31.4565), -2.0)
        Line((64.3527, -31.4565), (63.8735, -28.3271))
        Line((63.8735, -28.3271), (63.0673, -25.2656))
        Line((63.0673, -25.2656), (61.9429, -22.3061))
        Line((61.9429, -22.3061), (60.5129, -19.4817))
        Line((60.5129, -19.4817), (58.7932, -16.8236))
        Line((58.7932, -16.8236), (56.803, -14.3615))
        Line((56.803, -14.3615), (54.5644, -12.1229))
        Line((54.5644, -12.1229), (52.1023, -10.1327))
        Line((52.1023, -10.1327), (49.4442, -8.413))
        Line((49.4442, -8.413), (46.6198, -6.983))
        Line((46.6198, -6.983), (43.6603, -5.8586))
        Line((43.6603, -5.8586), (40.5988, -5.0524))
        Line((40.5988, -5.0524), (37.4694, -4.5732))
        RadiusArc((37.4694, -4.5732), (35.6878, -5.342), -2.0)
        RadiusArc((35.6878, -5.342), (33.4801, -5.5258), 1.5)
        RadiusArc((33.4801, -5.5258), (31.5306, -4.5732), -2.2341)
        Line((31.5306, -4.5732), (28.4012, -5.0524))
        Line((28.4012, -5.0524), (25.3397, -5.8586))
        Line((25.3397, -5.8586), (22.3802, -6.983))
        Line((22.3802, -6.983), (19.5558, -8.413))
        Line((19.5558, -8.413), (16.8977, -10.1327))
        Line((16.8977, -10.1327), (14.4356, -12.1229))
        Line((14.4356, -12.1229), (12.197, -14.3615))
        Line((12.197, -14.3615), (10.2068, -16.8236))
        Line((10.2068, -16.8236), (8.4871, -19.4817))
        Line((8.4871, -19.4817), (7.0571, -22.3061))
        Line((7.0571, -22.3061), (5.9327, -25.2656))
        Line((5.9327, -25.2656), (5.1265, -28.3271))
        Line((5.1265, -28.3271), (4.6473, -31.4565))
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    extrude(sk_Sketch1.sketch, amount=-25.0)
    # Fusion depth expression: -25.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(-0.0, -1.0, -0.0) * -33.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -33.000000 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(-0.0, -1.0, -0.0) * -33.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -33.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(-0.0, 1.0, 0.0) * -24.6
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -24.600000009 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
