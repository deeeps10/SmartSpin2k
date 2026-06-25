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

# 'Sketch4': circle on inclined plane
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch4:
    with Locations((198.1193, -56.8088)):
        Circle(radius=32.75)

# 'Sketch5': 2 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch5_2:
    with BuildLine():
        # Arc split: sweep=302.65deg >= 150 — emitted as two half-arcs
        RadiusArc((209.8673, -78.288), (198.1187, -32.3244), -24.4833)
        RadiusArc((198.1187, -32.3244), (186.3701, -78.288), -24.4833)
        Line((186.3701, -78.288), (209.8673, -78.288))
    _inc_edges_sk_Sketch5_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_2 = Wire.combine(_inc_edges_sk_Sketch5_2)[0]
_wire_sk_Sketch5_2 = _wire_sk_Sketch5_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch5_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch5_2.wrapped, True)
_face_sk_Sketch5_2 = Face(_mkf_sk_Sketch5_2.Face())

# 'Sketch6': 2 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch6_3:
    with BuildLine():
        # Arc split: sweep=299.53deg >= 150 — emitted as two half-arcs
        RadiusArc((209.2394, -75.888), (198.1184, -34.7253), -22.0836)
        RadiusArc((198.1184, -34.7253), (186.9977, -75.888), -22.0836)
        Line((186.9977, -75.888), (209.2394, -75.888))
    _inc_edges_sk_Sketch6_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_3 = Wire.combine(_inc_edges_sk_Sketch6_3)[0]
_wire_sk_Sketch6_3 = _wire_sk_Sketch6_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch6_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch6_3.wrapped, True)
_face_sk_Sketch6_3 = Face(_mkf_sk_Sketch6_3.Face())

# 'Sketch7': 8 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 16.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch7_4:
    with BuildLine():
        RadiusArc((-195.8957, -87.4783), (-167.4539, -59.089), -30.75)
        # Arc split: sweep=175.81deg >= 150 — emitted as two half-arcs
        RadiusArc((-167.4539, -59.089), (-169.6193, -56.8383), 2.25)
        RadiusArc((-169.6193, -56.8383), (-167.4493, -54.592), 2.25)
        RadiusArc((-167.4493, -54.592), (-195.8459, -26.143), -30.75)
        # Arc split: sweep=175.81deg >= 150 — emitted as two half-arcs
        RadiusArc((-195.8459, -26.143), (-198.0962, -28.3088), 2.25)
        RadiusArc((-198.0962, -28.3088), (-200.3429, -26.1393), 2.25)
        RadiusArc((-200.3429, -26.1393), (-228.7856, -54.5423), -30.75)
        # Arc split: sweep=175.81deg >= 150 — emitted as two half-arcs
        RadiusArc((-228.7856, -54.5423), (-226.6193, -56.7921), 2.25)
        RadiusArc((-226.6193, -56.7921), (-228.7883, -59.0392), 2.25)
        RadiusArc((-228.7883, -59.0392), (-200.3926, -87.4747), -30.75)
        # Arc split: sweep=175.81deg >= 150 — emitted as two half-arcs
        RadiusArc((-200.3926, -87.4747), (-198.1424, -85.3088), 2.25)
        RadiusArc((-198.1424, -85.3088), (-195.8957, -87.4783), 2.25)
    _inc_edges_sk_Sketch7_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_4 = Wire.combine(_inc_edges_sk_Sketch7_4)[0]
_wire_sk_Sketch7_4 = _wire_sk_Sketch7_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch7_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch7_4.wrapped, True)
_face_sk_Sketch7_4 = Face(_mkf_sk_Sketch7_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    extrude(sk_Sketch4.sketch, amount=-17.0)
    # Fusion depth expression: -16.999999285 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_2
    _vec = Vector(0.0, 0.0, -1.0) * 13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 13.000000715 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch6_3
    _vec = Vector(0.0, 0.0, -1.0) * -13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -13.000000715 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch7_4
    _vec = Vector(0.0, 0.0, 1.0) * 50.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 50.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
