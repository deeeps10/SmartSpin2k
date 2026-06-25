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

# 'Sketch8': 44 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(12.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch8:
    with BuildLine():
        RadiusArc((12.7773, 4.7667), (10.7098, 5.8774), -2.2)
        RadiusArc((10.7098, 5.8774), (10.7098, 7.0295), -4.249)
        RadiusArc((10.7098, 7.0295), (12.7773, 8.1403), -2.2)
        Line((12.7773, 8.1403), (12.6928, 8.4282))
        RadiusArc((12.6928, 8.4282), (10.3529, 8.2449), -2.2)
        RadiusArc((10.3529, 8.2449), (9.7301, 9.2141), -4.249)
        RadiusArc((9.7301, 9.2141), (10.8689, 11.2663), -2.2)
        Line((10.8689, 11.2663), (10.6421, 11.4628))
        RadiusArc((10.6421, 11.4628), (8.7728, 10.0435), -2.2)
        RadiusArc((8.7728, 10.0435), (7.7248, 10.5221), -4.249)
        RadiusArc((7.7248, 10.5221), (7.5733, 12.8643), -2.2)
        Line((7.5733, 12.8643), (7.2763, 12.907))
        RadiusArc((7.2763, 12.907), (6.4711, 10.7024), -2.2)
        RadiusArc((6.4711, 10.7024), (5.3307, 10.5384), -4.249)
        RadiusArc((5.3307, 10.5384), (3.937, 12.4268), -2.2)
        Line((3.937, 12.4268), (3.664, 12.3022))
        RadiusArc((3.664, 12.3022), (4.1785, 10.0122), -2.2)
        RadiusArc((4.1785, 10.0122), (3.3078, 9.2578), -4.249)
        RadiusArc((3.3078, 9.2578), (1.1144, 10.0929), -2.2)
        Line((1.1144, 10.0929), (0.9522, 9.8405))
        RadiusArc((0.9522, 9.8405), (2.623, 8.1922), -2.2)
        RadiusArc((2.623, 8.1922), (2.2985, 7.0868), -4.249)
        RadiusArc((2.2985, 7.0868), (0.0017, 6.6035), -2.2)
        Line((0.0017, 6.6035), (0.0017, 6.3034))
        RadiusArc((0.0017, 6.3034), (2.2985, 5.8202), -2.2)
        RadiusArc((2.2985, 5.8202), (2.623, 4.7147), -4.249)
        RadiusArc((2.623, 4.7147), (0.9522, 3.0665), -2.2)
        Line((0.9522, 3.0665), (1.1144, 2.814))
        RadiusArc((1.1144, 2.814), (3.3078, 3.6492), -2.2)
        RadiusArc((3.3078, 3.6492), (4.1785, 2.8947), -4.249)
        RadiusArc((4.1785, 2.8947), (3.664, 0.6048), -2.2)
        Line((3.664, 0.6048), (3.937, 0.4801))
        RadiusArc((3.937, 0.4801), (5.3307, 2.3685), -2.2)
        RadiusArc((5.3307, 2.3685), (6.4711, 2.2046), -4.249)
        RadiusArc((6.4711, 2.2046), (7.2763, 0.0), -2.2)
        Line((7.2763, 0.0), (7.5733, 0.0427))
        RadiusArc((7.5733, 0.0427), (7.7248, 2.3848), -2.2)
        RadiusArc((7.7248, 2.3848), (8.7728, 2.8634), -4.249)
        RadiusArc((8.7728, 2.8634), (10.6421, 1.4442), -2.2)
        Line((10.6421, 1.4442), (10.8689, 1.6407))
        RadiusArc((10.8689, 1.6407), (9.7301, 3.6929), -2.2)
        RadiusArc((9.7301, 3.6929), (10.3529, 4.6621), -4.249)
        RadiusArc((10.3529, 4.6621), (12.6928, 4.4787), -2.2)
        Line((12.6928, 4.4787), (12.7773, 4.7667))
    _inc_edges_sk_Sketch8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8 = Wire.combine(_inc_edges_sk_Sketch8)[0]
_wire_sk_Sketch8 = _wire_sk_Sketch8.moved(_inclined_plane_1.location)
_mkf_sk_Sketch8 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch8.wrapped, True)
_face_sk_Sketch8 = Face(_mkf_sk_Sketch8.Face())

# 'Sketch9': 2 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, -0.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch9_2:
    with BuildLine():
        # Arc split: sweep=307.79deg >= 150 — emitted as two half-arcs
        RadiusArc((-8.745, 7.5535), (-4.0, 6.4535), 2.5)
        RadiusArc((-4.0, 6.4535), (-8.745, 5.3535), 2.5)
        Line((-8.745, 5.3535), (-8.745, 7.5535))
    _inc_edges_sk_Sketch9_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch9_2 = Wire.combine(_inc_edges_sk_Sketch9_2)[0]
_wire_sk_Sketch9_2 = _wire_sk_Sketch9_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch9_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch9_2.wrapped, True)
_face_sk_Sketch9_2 = Face(_mkf_sk_Sketch9_2.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    _face = _face_sk_Sketch8
    _vec = Vector(1.0, 0.0, 0.0) * -12.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -12.000000477 mm
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch9_2
    _vec = Vector(-1.0, -0.0, 0.0) * -14.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -14.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
