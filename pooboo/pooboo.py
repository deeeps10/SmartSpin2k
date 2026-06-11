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
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, -1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch2:
    with BuildLine():
        RadiusArc((330.479, 684.9555), (325.3577, 688.6793), -5.0)
        # Arc split: sweep=172.87deg >= 150 — emitted as two half-arcs
        RadiusArc((325.3577, 688.6793), (0.0047, 346.042), -345.0)
        RadiusArc((0.0047, 346.042), (321.7587, 0.0227), -345.0)
        RadiusArc((321.7587, 0.0227), (331.4594, 5.6975), -10.0)
        RadiusArc((331.4594, 5.6975), (358.5406, 5.6975), 15.0)
        RadiusArc((358.5406, 5.6975), (368.2413, 0.0227), -10.0)
        # Arc split: sweep=172.87deg >= 150 — emitted as two half-arcs
        RadiusArc((368.2413, 0.0227), (689.9953, 346.042), -345.0)
        RadiusArc((689.9953, 346.042), (364.6423, 688.6794), -345.0)
        RadiusArc((364.6423, 688.6794), (359.5182, 684.9446), -5.0001)
        # Arc split: sweep=150.92deg >= 150 — emitted as two half-arcs
        RadiusArc((359.5182, 684.9446), (344.9944, 673.7158), 15.0)
        RadiusArc((344.9944, 673.7158), (330.479, 684.9555), 15.0)
    _inc_edges_sk_Sketch2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2 = Wire.combine(_inc_edges_sk_Sketch2)[0]
_wire_sk_Sketch2 = _wire_sk_Sketch2.moved(_inclined_plane_1.location)
_mkf_sk_Sketch2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch2.wrapped, True)
_face_sk_Sketch2 = Face(_mkf_sk_Sketch2.Face())

# 'Sketch3': circle on inclined plane
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 10.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 1.0, -0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch3_2:
    with Locations((-345.0, 344.259)):
        Circle(radius=330.0)

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        RadiusArc((107.3145, -115.3363), (27.9044, -252.8787), -330.0)
        RadiusArc((27.9044, -252.8787), (107.3145, -115.3363), -119.2)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        RadiusArc((424.4101, -23.956), (265.5899, -23.956), -330.0)
        RadiusArc((265.5899, -23.956), (424.4101, -23.956), -119.2)
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch4_5:
    with BuildLine():
        RadiusArc((582.6855, -115.3363), (662.0956, -252.8787), -119.2)
        RadiusArc((662.0956, -252.8787), (582.6855, -115.3363), -330.0004)
    _inc_edges_sk_Sketch4_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_5 = Wire.combine(_inc_edges_sk_Sketch4_5)[0]
_wire_sk_Sketch4_5 = _wire_sk_Sketch4_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch4_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch4_5.wrapped, True)
_face_sk_Sketch4_5 = Face(_mkf_sk_Sketch4_5.Face())

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch4_6:
    with BuildLine():
        RadiusArc((662.0956, -435.6393), (582.6855, -573.1817), -119.2)
        RadiusArc((582.6855, -573.1817), (662.0956, -435.6393), -330.0003)
    _inc_edges_sk_Sketch4_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_6 = Wire.combine(_inc_edges_sk_Sketch4_6)[0]
_wire_sk_Sketch4_6 = _wire_sk_Sketch4_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch4_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch4_6.wrapped, True)
_face_sk_Sketch4_6 = Face(_mkf_sk_Sketch4_6.Face())

# 'Sketch4': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch4_7:
    with BuildLine():
        RadiusArc((424.4101, -664.562), (265.5899, -664.562), -119.2001)
        RadiusArc((265.5899, -664.562), (341.0848, -674.2358), -329.9992)
        RadiusArc((341.0848, -674.2358), (348.9152, -674.2358), -329.9992)
        RadiusArc((348.9152, -674.2358), (424.4101, -664.562), -329.9992)
    _inc_edges_sk_Sketch4_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_7 = Wire.combine(_inc_edges_sk_Sketch4_7)[0]
_wire_sk_Sketch4_7 = _wire_sk_Sketch4_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch4_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch4_7.wrapped, True)
_face_sk_Sketch4_7 = Face(_mkf_sk_Sketch4_7.Face())

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 250.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch4_8:
    with BuildLine():
        RadiusArc((107.3145, -573.1817), (27.9044, -435.6393), -119.2)
        RadiusArc((27.9044, -435.6393), (107.3145, -573.1817), -330.0)
    _inc_edges_sk_Sketch4_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_8 = Wire.combine(_inc_edges_sk_Sketch4_8)[0]
_wire_sk_Sketch4_8 = _wire_sk_Sketch4_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch4_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch4_8.wrapped, True)
_face_sk_Sketch4_8 = Face(_mkf_sk_Sketch4_8.Face())

# -- Build main body (Extrude1 + Extrude3 subtract) --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    _face = _face_sk_Sketch2
    _vec = Vector(0.0, -1.0, 0.0) * -250.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)

    # --- FEATURE: Extrude3 (subtract) ---
    extrude(sk_Sketch3_2.sketch, amount=1000.0, mode=Mode.SUBTRACT)

# --- FEATURE: Extrude4 — kept as separate bodies (not merged into main) ---
_extrude_vec = Vector(0.0, -1.0, 0.0) * 250.0
_e4_p0 = Solid.extrude(_face_sk_Sketch4_3, _extrude_vec)
_e4_p1 = Solid.extrude(_face_sk_Sketch4_4, _extrude_vec)
_e4_p2 = Solid.extrude(_face_sk_Sketch4_5, _extrude_vec)
_e4_p3 = Solid.extrude(_face_sk_Sketch4_6, _extrude_vec)
_e4_p4 = Solid.extrude(_face_sk_Sketch4_7, _extrude_vec)
_e4_p5 = Solid.extrude(_face_sk_Sketch4_8, _extrude_vec)

# -- Export (all bodies combined) --
_all_bodies = Compound([part.part, _e4_p0, _e4_p1, _e4_p2, _e4_p3, _e4_p4, _e4_p5])
export_step(_all_bodies, 'fusion_features.step')
export_stl(_all_bodies,  'fusion_features.stl')
if _has_ocp: show(part, _e4_p0, _e4_p1, _e4_p2, _e4_p3, _e4_p4, _e4_p5)
