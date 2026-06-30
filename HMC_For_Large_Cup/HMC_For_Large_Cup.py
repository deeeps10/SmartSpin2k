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

# 'Sketch3': 8 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch3:
    with BuildLine():
        RadiusArc((-32.96, -68.3233), (-32.4555, -68.6704), -0.5)
        # Arc split: sweep=172.93deg >= 150 — emitted as two half-arcs
        RadiusArc((-32.4555, -68.6704), (-0.0005, -34.5022), -34.4)
        RadiusArc((-0.0005, -34.5022), (-32.1025, -0.0022), -34.4)
        RadiusArc((-32.1025, -0.0022), (-33.0616, -0.5485), -1.0)
        RadiusArc((-33.0616, -0.5485), (-35.7384, -0.5485), 1.5)
        RadiusArc((-35.7384, -0.5485), (-36.6975, -0.0022), -1.0)
        # Arc split: sweep=172.97deg >= 150 — emitted as two half-arcs
        RadiusArc((-36.6975, -0.0022), (-68.7995, -34.5132), -34.4)
        RadiusArc((-68.7995, -34.5132), (-36.3225, -68.6717), -34.4)
        RadiusArc((-36.3225, -68.6717), (-35.8182, -68.3242), -0.5)
        RadiusArc((-35.8182, -68.3242), (-32.96, -68.3233), 1.5)
    _inc_edges_sk_Sketch3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3 = Wire.combine(_inc_edges_sk_Sketch3)[0]
_wire_sk_Sketch3 = _wire_sk_Sketch3.moved(_inclined_plane_1.location)
_mkf_sk_Sketch3 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch3.wrapped, True)
_face_sk_Sketch3 = Face(_mkf_sk_Sketch3.Face())

# 'Sketch4': 12 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.5, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch4_2:
    with BuildLine():
        RadiusArc((5.3079, -48.8134), (5.3079, -19.8374), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((5.3079, -19.8374), (7.9862, -19.0755), -2.0)
        RadiusArc((7.9862, -19.0755), (7.307, -16.375), -2.0)
        RadiusArc((7.307, -16.375), (32.4009, -1.887), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((32.4009, -1.887), (34.4001, -3.8254), -2.0)
        RadiusArc((34.4001, -3.8254), (36.3991, -1.887), -2.0)
        RadiusArc((36.3991, -1.887), (61.493, -16.375), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((61.493, -16.375), (60.8138, -19.0754), -2.0)
        RadiusArc((60.8138, -19.0754), (63.4921, -19.8374), -2.0)
        RadiusArc((63.4921, -19.8374), (63.4921, -48.8134), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((63.4921, -48.8134), (60.8138, -49.5753), -2.0)
        RadiusArc((60.8138, -49.5753), (61.493, -52.2759), -2.0)
        RadiusArc((61.493, -52.2759), (36.3991, -66.7639), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((36.3991, -66.7639), (34.4, -64.8254), -2.0)
        RadiusArc((34.4, -64.8254), (32.4009, -66.7639), -2.0)
        RadiusArc((32.4009, -66.7639), (7.307, -52.2759), 32.5)
        # Arc split: sweep=176.47deg >= 150 — emitted as two half-arcs
        RadiusArc((7.307, -52.2759), (7.9863, -49.5754), -2.0)
        RadiusArc((7.9863, -49.5754), (5.3079, -48.8134), -2.0)
    _inc_edges_sk_Sketch4_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_2 = Wire.combine(_inc_edges_sk_Sketch4_2)[0]
_wire_sk_Sketch4_2 = _wire_sk_Sketch4_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch4_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch4_2.wrapped, True)
_face_sk_Sketch4_2 = Face(_mkf_sk_Sketch4_2.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3
    _vec = Vector(-0.0, -1.0, -0.0) * -25.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -25.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_2
    _vec = Vector(-0.0, 1.0, 0.0) * 48.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 48.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
