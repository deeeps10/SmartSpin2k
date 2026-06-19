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

# 'Sketch2': 8 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch2:
    with BuildLine():
        RadiusArc((53.4, -7.8), (56.8, -4.4), -3.4)
        Line((56.8, -4.4), (56.8, -1.6))
        RadiusArc((56.8, -1.6), (53.4, 1.8), -3.4)
        Line((53.4, 1.8), (36.6, 1.8))
        RadiusArc((36.6, 1.8), (33.2, -1.6), -3.4)
        Line((33.2, -1.6), (33.2, -4.4))
        RadiusArc((33.2, -4.4), (36.6, -7.8), -3.4)
        Line((36.6, -7.8), (53.4, -7.8))
    _inc_edges_sk_Sketch2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2 = Wire.combine(_inc_edges_sk_Sketch2)[0]
_wire_sk_Sketch2 = _wire_sk_Sketch2.moved(_inclined_plane_1.location)
_mkf_sk_Sketch2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch2.wrapped, True)
_face_sk_Sketch2 = Face(_mkf_sk_Sketch2.Face())
# v16.993: inner loop 0 bore face (segment-based, inclined plane)
with BuildSketch(_inclined_plane_1) as sk_Sketch2_bore0_2:
    with BuildLine():
        Line((55.0, -1.6), (55.0, -4.4))
        RadiusArc((55.0, -4.4), (53.4, -6.0), 1.6)
        Line((53.4, -6.0), (36.6, -6.0))
        RadiusArc((36.6, -6.0), (35.0, -4.4), 1.6)
        Line((35.0, -4.4), (35.0, -1.6))
        RadiusArc((35.0, -1.6), (36.6, -0.0), 1.6)
        Line((36.6, -0.0), (53.4, 0.0))
        RadiusArc((53.4, 0.0), (55.0, -1.6), 1.6)
    _inc_bore_edges_sk_Sketch2_bore0_2 = list(BuildSketch._get_context().pending_edges)
_bore_wire_sk_Sketch2_bore0_2 = Wire.combine(_inc_bore_edges_sk_Sketch2_bore0_2)[0]
_bore_wire_sk_Sketch2_bore0_2 = _bore_wire_sk_Sketch2_bore0_2.moved(_inclined_plane_1.location)
_bore_mkf_sk_Sketch2_bore0_2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _bore_wire_sk_Sketch2_bore0_2.wrapped, True)
_bore_face_sk_Sketch2_bore0_2 = Face(_bore_mkf_sk_Sketch2_bore0_2.Face())


# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2
    _vec = Vector(-0.0, -1.0, -0.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.993: subtract segment-based bore(s) — inclined plane
    _bore_solid_sk_Sketch2_bore0_2 = Solid.extrude(_bore_face_sk_Sketch2_bore0_2, Vector(-0.0, -1.0, -0.0) * -5.0)
    part.part = cut_solids(part.part, _bore_solid_sk_Sketch2_bore0_2)
    # Fusion depth expression: -5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)

# -- Volume Display --
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
if part.part is not None:
    _vprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.part.wrapped, _vprops)
    print(f"\n  Volume of final part: {abs(_vprops.Mass()):.2f} mm³")
