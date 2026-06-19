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
        Line((0.6, 14.0034), (0.6, 13.9))
        Line((0.6, 13.9), (-9.0, 13.9))
        Line((-9.0, 13.9), (-9.0, 10.5))
        RadiusArc((-9.0, 10.5), (-6.0, 7.5103), -2.9707)
        Line((-6.0, 7.5103), (-6.0, -22.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -22.5), (0.0, -28.5), -6.0)
        RadiusArc((0.0, -28.5), (6.0, -22.5), -6.0)
        Line((6.0, -22.5), (6.0, 7.5103))
        RadiusArc((6.0, 7.5103), (9.0, 10.5), -2.9818)
        Line((9.0, 10.5), (9.0, 22.5))
        Line((9.0, 22.5), (-9.0, 22.5))
        Line((-9.0, 22.5), (-9.0, 19.1))
        Line((-9.0, 19.1), (0.6, 19.1))
        Line((0.6, 19.1), (0.6, 18.9966))
        # Arc split: sweep=211.65deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 18.9966), (3.9027, 16.5), 2.595)
        RadiusArc((3.9027, 16.5), (0.6, 14.0034), 2.595)
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
        Line((1.3359, -13.291), (2.1903, -13.291))
        Line((2.1903, -13.291), (2.1903, -9.2385))
        Line((2.1903, -9.2385), (4.1372, -9.2385))
        Line((4.1372, -9.2385), (4.1372, -8.201))
        Line((4.1372, -8.201), (2.1903, -8.201))
        Line((2.1903, -8.201), (2.1903, -6.9926))
        Line((2.1903, -6.9926), (1.3236, -6.9926))
        Line((1.3236, -6.9926), (1.3236, -8.201))
        Line((1.3236, -8.201), (-4.4621, -8.201))
        Line((-4.4621, -8.201), (-4.4621, -9.3545))
        Line((-4.4621, -9.3545), (1.3359, -13.291))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch4': 25 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        Line((2.0865, -5.1777), (2.2147, -6.2884))
        RadiusArc((2.2147, -6.2884), (3.6623, -5.4507), -2.4954)
        RadiusArc((3.6623, -5.4507), (4.2499, -3.7263), -3.0065)
        RadiusArc((4.2499, -3.7263), (3.912, -1.7797), -3.5336)
        RadiusArc((3.912, -1.7797), (2.6226, -0.5923), -2.5052)
        RadiusArc((2.6226, -0.5923), (1.0249, -0.3757), -3.6948)
        RadiusArc((1.0249, -0.3757), (-0.289, -0.8447), -2.541)
        RadiusArc((-0.289, -0.8447), (-1.2304, -2.2062), -2.5024)
        RadiusArc((-1.2304, -2.2062), (-1.3275, -3.5501), -3.7567)
        RadiusArc((-1.3275, -3.5501), (-0.8002, -4.9153), -2.6322)
        Line((-0.8002, -4.9153), (-3.5414, -4.7497))
        Line((-3.5414, -4.7497), (-3.5414, -0.862))
        Line((-3.5414, -0.862), (-4.4752, -0.862))
        Line((-4.4752, -0.862), (-4.4752, -5.7506))
        Line((-4.4752, -5.7506), (0.164, -6.0383))
        Line((0.164, -6.0383), (0.164, -4.9641))
        RadiusArc((0.164, -4.9641), (-0.1967, -4.4514), 2.9032)
        RadiusArc((-0.1967, -4.4514), (-0.4218, -3.7676), 2.0856)
        RadiusArc((-0.4218, -3.7676), (-0.2722, -2.4661), 2.2644)
        RadiusArc((-0.2722, -2.4661), (0.8537, -1.5595), 1.647)
        RadiusArc((0.8537, -1.5595), (1.6021, -1.5125), 2.3256)
        RadiusArc((1.6021, -1.5125), (2.5985, -1.8239), 2.0442)
        RadiusArc((2.5985, -1.8239), (3.3028, -2.8422), 1.6275)
        RadiusArc((3.3028, -2.8422), (3.158, -4.3669), 2.5142)
        RadiusArc((3.158, -4.3669), (2.0865, -5.1777), 1.6511)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch5': 6 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch5_4:
    with BuildLine():
        Line((1.2015, -12.1803), (1.3236, -12.2779))
        Line((1.3236, -12.2779), (1.3236, -9.2385))
        Line((1.3236, -9.2385), (-3.2232, -9.2385))
        Line((-3.2232, -9.2385), (-2.4969, -9.6475))
        Line((-2.4969, -9.6475), (0.7499, -11.8507))
        Line((0.7499, -11.8507), (1.2015, -12.1803))
    _inc_edges_sk_Sketch5_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_4 = Wire.combine(_inc_edges_sk_Sketch5_4)[0]
_wire_sk_Sketch5_4 = _wire_sk_Sketch5_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch5_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch5_4.wrapped, True)
_face_sk_Sketch5_4 = Face(_mkf_sk_Sketch5_4.Face())

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
    _bore_ax = _gAx2(_gPnt(0.0, -22.5, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.625, 9.0)
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
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 0.500000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch5_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.50000012 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
