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

# 'Sketch9': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch9:
    with BuildLine():
        Line((0.6, 49.0034), (0.6, 48.9))
        Line((0.6, 48.9), (-9.0, 48.9))
        Line((-9.0, 48.9), (-9.0, 45.5))
        RadiusArc((-9.0, 45.5), (-6.0, 42.5102), -2.9889)
        Line((-6.0, 42.5102), (-6.0, -57.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -57.5), (-0.0, -63.5), -6.0)
        RadiusArc((-0.0, -63.5), (6.0, -57.5), -6.0)
        Line((6.0, -57.5), (6.0, 42.5102))
        RadiusArc((6.0, 42.5102), (9.0, 45.5), -2.9819)
        Line((9.0, 45.5), (9.0, 57.5))
        Line((9.0, 57.5), (-9.0, 57.5))
        Line((-9.0, 57.5), (-9.0, 54.1))
        Line((-9.0, 54.1), (0.6, 54.1))
        Line((0.6, 54.1), (0.6, 53.9966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 53.9966), (3.9, 51.5), 2.5944)
        RadiusArc((3.9, 51.5), (0.6, 49.0034), 2.5944)
    _inc_edges_sk_Sketch9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch9 = Wire.combine(_inc_edges_sk_Sketch9)[0]
_wire_sk_Sketch9 = _wire_sk_Sketch9.moved(_inclined_plane_1.location)
_mkf_sk_Sketch9 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch9.wrapped, True)
_face_sk_Sketch9 = Face(_mkf_sk_Sketch9.Face())

# 'Sketch10': 54 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch10_2:
    with BuildLine():
        RadiusArc((2.2147, -6.2884), (2.8256, -6.0979), -2.8955)
        # Near-straight arc (sagitta=0.006972mm) replaced with Line
        Line((2.8256, -6.0979), (3.1421, -5.9256))
        RadiusArc((3.1421, -5.9256), (3.586, -5.542), -2.1698)
        RadiusArc((3.586, -5.542), (3.9769, -4.9279), -2.3656)
        RadiusArc((3.9769, -4.9279), (4.1752, -4.3016), -2.9537)
        RadiusArc((4.1752, -4.3016), (4.2592, -3.4139), -4.1823)
        # Near-straight arc (sagitta=0.008045mm) replaced with Line
        Line((4.2592, -3.4139), (4.228, -2.8653))
        RadiusArc((4.228, -2.8653), (4.0891, -2.2116), -3.6035)
        RadiusArc((4.0891, -2.2116), (3.8391, -1.6468), -2.8215)
        RadiusArc((3.8391, -1.6468), (3.3713, -1.0668), -2.5236)
        # Near-straight arc (sagitta=0.009935mm) replaced with Line
        Line((3.3713, -1.0668), (3.0202, -0.7972))
        # Near-straight arc (sagitta=0.009895mm) replaced with Line
        Line((3.0202, -0.7972), (2.6226, -0.5923))
        # Near-straight arc (sagitta=0.009554mm) replaced with Line
        Line((2.6226, -0.5923), (2.1787, -0.4522))
        RadiusArc((2.1787, -0.4522), (1.5147, -0.3659), -3.5155)
        RadiusArc((1.5147, -0.3659), (0.73, -0.4159), -3.6386)
        RadiusArc((0.73, -0.4159), (0.1884, -0.5767), -2.6828)
        RadiusArc((0.1884, -0.5767), (-0.3983, -0.9285), -2.4134)
        RadiusArc((-0.3983, -0.9285), (-0.9491, -1.5547), -2.5003)
        RadiusArc((-0.9491, -1.5547), (-1.1873, -2.0667), -2.4982)
        RadiusArc((-1.1873, -2.0667), (-1.3197, -2.6521), -2.961)
        RadiusArc((-1.3197, -2.6521), (-1.3397, -3.4162), -3.8755)
        RadiusArc((-1.3397, -3.4162), (-1.1932, -4.173), -3.1849)
        RadiusArc((-1.1932, -4.173), (-0.8002, -4.9153), -2.5492)
        Line((-0.8002, -4.9153), (-3.5284, -4.7505))
        Line((-3.5284, -4.7505), (-3.5284, -0.8628))
        Line((-3.5284, -0.8628), (-4.4621, -0.8628))
        Line((-4.4621, -0.8628), (-4.4621, -5.7514))
        Line((-4.4621, -5.7514), (0.164, -6.0382))
        Line((0.164, -6.0382), (0.164, -4.9641))
        # Near-straight arc (sagitta=0.006015mm) replaced with Line
        Line((0.164, -4.9641), (-0.0732, -4.6565))
        # Near-straight arc (sagitta=0.006607mm) replaced with Line
        Line((-0.0732, -4.6565), (-0.248, -4.3489))
        RadiusArc((-0.248, -4.3489), (-0.39, -3.932), 2.0379)
        # Near-straight arc (sagitta=0.008165mm) replaced with Line
        Line((-0.39, -3.932), (-0.4467, -3.5408))
        RadiusArc((-0.4467, -3.5408), (-0.4168, -2.9343), 2.6667)
        RadiusArc((-0.4168, -2.9343), (-0.2722, -2.4661), 1.9144)
        # Near-straight arc (sagitta=0.006086mm) replaced with Line
        Line((-0.2722, -2.4661), (-0.132, -2.2226))
        RadiusArc((-0.132, -2.2226), (0.1162, -1.9422), 1.6317)
        RadiusArc((0.1162, -1.9422), (0.4163, -1.7274), 1.6338)
        RadiusArc((0.4163, -1.7274), (0.8537, -1.5595), 1.681)
        # Near-straight arc (sagitta=0.009812mm) replaced with Line
        Line((0.8537, -1.5595), (1.2534, -1.5058))
        RadiusArc((1.2534, -1.5058), (1.8298, -1.5392), 2.9933)
        RadiusArc((1.8298, -1.5392), (2.2426, -1.6459), 2.2395)
        # Near-straight arc (sagitta=0.006188mm) replaced with Line
        Line((2.2426, -1.6459), (2.5149, -1.7727))
        RadiusArc((2.5149, -1.7727), (2.8281, -2.0041), 1.6762)
        RadiusArc((2.8281, -2.0041), (3.0749, -2.2983), 1.5953)
        RadiusArc((3.0749, -2.2983), (3.2458, -2.6473), 1.6255)
        # Near-straight arc (sagitta=0.006196mm) replaced with Line
        Line((3.2458, -2.6473), (3.3241, -2.9448))
        # Near-straight arc (sagitta=0.005799mm) replaced with Line
        Line((3.3241, -2.9448), (3.3597, -3.2732))
        # Near-straight arc (sagitta=0.009945mm) replaced with Line
        Line((3.3597, -3.2732), (3.3394, -3.7541))
        RadiusArc((3.3394, -3.7541), (3.158, -4.3669), 1.9579)
        RadiusArc((3.158, -4.3669), (2.9029, -4.7246), 1.4578)
        RadiusArc((2.9029, -4.7246), (2.5457, -4.9948), 1.5202)
        RadiusArc((2.5457, -4.9948), (2.0865, -5.1777), 2.0155)
        Line((2.0865, -5.1777), (2.2147, -6.2884))
    _inc_edges_sk_Sketch10_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_2 = Wire.combine(_inc_edges_sk_Sketch10_2)[0]
_wire_sk_Sketch10_2 = _wire_sk_Sketch10_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch10_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch10_2.wrapped, True)
_face_sk_Sketch10_2 = Face(_mkf_sk_Sketch10_2.Face())

# 'Sketch10': 11 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch10_3:
    with BuildLine():
        Line((-4.4621, -9.3301), (3.2034, -9.3301))
        Line((3.2034, -9.3301), (3.2034, -7.2367))
        Line((3.2034, -7.2367), (4.1372, -7.2367))
        Line((4.1372, -7.2367), (4.1372, -12.6257))
        Line((4.1372, -12.6257), (3.2034, -12.6257))
        Line((3.2034, -12.6257), (3.2034, -10.4348))
        Line((3.2034, -10.4348), (-3.4124, -10.4348))
        Line((-3.4124, -10.4348), (-2.027, -12.3755))
        Line((-2.027, -12.3755), (-3.0645, -12.3755))
        Line((-3.0645, -12.3755), (-4.4621, -10.3432))
        Line((-4.4621, -10.3432), (-4.4621, -9.3301))
    _inc_edges_sk_Sketch10_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_3 = Wire.combine(_inc_edges_sk_Sketch10_3)[0]
_wire_sk_Sketch10_3 = _wire_sk_Sketch10_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch10_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch10_3.wrapped, True)
_face_sk_Sketch10_3 = Face(_mkf_sk_Sketch10_3.Face())

# 'Sketch10': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch10_4:
    with BuildLine():
        Line((-4.4621, -15.213), (3.2034, -15.213))
        Line((3.2034, -15.213), (3.2034, -13.1197))
        Line((3.2034, -13.1197), (4.1372, -13.1197))
        Line((4.1372, -13.1197), (4.1372, -18.5087))
        Line((4.1372, -18.5087), (3.2034, -18.5087))
        Line((3.2034, -18.5087), (3.2034, -16.3177))
        Line((3.2034, -16.3177), (-3.4124, -16.3177))
        Line((-3.4124, -16.3177), (-2.027, -18.2585))
        Line((-2.027, -18.2585), (-3.0645, -18.2585))
        Line((-3.0645, -18.2585), (-4.4621, -16.2261))
        Line((-4.4621, -16.2261), (-4.4621, -15.213))
    _inc_edges_sk_Sketch10_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_4 = Wire.combine(_inc_edges_sk_Sketch10_4)[0]
_wire_sk_Sketch10_4 = _wire_sk_Sketch10_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch10_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch10_4.wrapped, True)
_face_sk_Sketch10_4 = Face(_mkf_sk_Sketch10_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch9
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(-0.0001, -57.5, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6249, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8_p0 --
    _face = _face_sk_Sketch10_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude8_p1 --
    _face = _face_sk_Sketch10_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude8_p2 --
    _face = _face_sk_Sketch10_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
