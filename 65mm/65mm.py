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

# 'Sketch4': 8 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch4:
    with BuildLine():
        RadiusArc((-33.0482, -68.4945), (-32.5358, -68.8679), -0.5)
        # Arc split: sweep=172.87deg >= 150 — emitted as two half-arcs
        RadiusArc((-32.5358, -68.8679), (-0.0005, -34.6042), -34.5)
        RadiusArc((-0.0005, -34.6042), (-32.1759, -0.0023), -34.5)
        RadiusArc((-32.1759, -0.0023), (-33.1459, -0.5697), -1.0)
        RadiusArc((-33.1459, -0.5697), (-35.8541, -0.5697), 1.5)
        RadiusArc((-35.8541, -0.5697), (-36.8241, -0.0023), -1.0)
        # Arc split: sweep=172.87deg >= 150 — emitted as two half-arcs
        RadiusArc((-36.8241, -0.0023), (-68.9995, -34.6042), -34.5)
        RadiusArc((-68.9995, -34.6042), (-36.4642, -68.8679), -34.5)
        RadiusArc((-36.4642, -68.8679), (-35.9518, -68.4945), -0.5)
        # Arc split: sweep=150.88deg >= 150 — emitted as two half-arcs
        RadiusArc((-35.9518, -68.4945), (-34.5, -67.3716), 1.5)
        RadiusArc((-34.5, -67.3716), (-33.0482, -68.4945), 1.5)
    _inc_edges_sk_Sketch4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4 = Wire.combine(_inc_edges_sk_Sketch4)[0]
_wire_sk_Sketch4 = _wire_sk_Sketch4.moved(_inclined_plane_1.location)
_mkf_sk_Sketch4 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch4.wrapped, True)
_face_sk_Sketch4 = Face(_mkf_sk_Sketch4.Face())

# 'Sketch5': 16 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 25.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 1.0, -0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch5_2:
    with BuildLine():
        RadiusArc((-33.3087, 2.8374), (-31.5381, 2.0611), -2.0)
        RadiusArc((-31.5381, 2.0611), (-2.1352, 31.464), -32.5)
        RadiusArc((-2.1352, 31.464), (-2.9115, 33.2346), -2.0)
        RadiusArc((-2.9115, 33.2346), (-2.9115, 35.6172), 1.5)
        RadiusArc((-2.9115, 35.6172), (-2.1352, 37.3878), -2.0)
        RadiusArc((-2.1352, 37.3878), (-31.5381, 66.7907), -32.5)
        RadiusArc((-31.5381, 66.7907), (-33.3087, 66.0144), -2.0)
        RadiusArc((-33.3087, 66.0144), (-35.6913, 66.0144), 1.5)
        RadiusArc((-35.6913, 66.0144), (-37.4619, 66.7907), -2.0)
        RadiusArc((-37.4619, 66.7907), (-66.8648, 37.3878), -32.5)
        RadiusArc((-66.8648, 37.3878), (-66.0885, 35.6172), -2.0)
        RadiusArc((-66.0885, 35.6172), (-66.0885, 33.2346), 1.5)
        RadiusArc((-66.0885, 33.2346), (-66.8648, 31.464), -2.0)
        RadiusArc((-66.8648, 31.464), (-37.4619, 2.0611), -32.5)
        RadiusArc((-37.4619, 2.0611), (-35.6913, 2.8374), -2.0)
        RadiusArc((-35.6913, 2.8374), (-33.3087, 2.8374), 1.5)
    _inc_edges_sk_Sketch5_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_2 = Wire.combine(_inc_edges_sk_Sketch5_2)[0]
_wire_sk_Sketch5_2 = _wire_sk_Sketch5_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch5_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch5_2.wrapped, True)
_face_sk_Sketch5_2 = Face(_mkf_sk_Sketch5_2.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch4
    _vec = Vector(-0.0, -1.0, -0.0) * -25.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -25.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch5_2
    _vec = Vector(0.0, 1.0, -0.0) * -24.6
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -24.600000009 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
