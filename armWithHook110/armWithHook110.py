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

# 'Sketch5': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch5:
    with BuildLine():
        Line((0.6, 46.5034), (0.6, 46.4))
        Line((0.6, 46.4), (-9.0, 46.4))
        Line((-9.0, 46.4), (-9.0, 43.0))
        RadiusArc((-9.0, 43.0), (-6.0, 40.0102), -2.9846)
        Line((-6.0, 40.0102), (-6.0, -55.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -55.0), (-0.0, -61.0), -6.0)
        RadiusArc((-0.0, -61.0), (6.0, -55.0), -6.0)
        Line((6.0, -55.0), (6.0, 40.0102))
        RadiusArc((6.0, 40.0102), (9.0, 43.0), -2.9819)
        Line((9.0, 43.0), (9.0, 55.0))
        Line((9.0, 55.0), (-9.0, 55.0))
        Line((-9.0, 55.0), (-9.0, 51.6))
        Line((-9.0, 51.6), (0.6, 51.6))
        Line((0.6, 51.6), (0.6, 51.4966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 51.4966), (3.9, 49.0), 2.5944)
        RadiusArc((3.9, 49.0), (0.6, 46.5034), 2.5944)
    _inc_edges_sk_Sketch5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5 = Wire.combine(_inc_edges_sk_Sketch5)[0]
_wire_sk_Sketch5 = _wire_sk_Sketch5.moved(_inclined_plane_1.location)
_mkf_sk_Sketch5 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch5.wrapped, True)
_face_sk_Sketch5 = Face(_mkf_sk_Sketch5.Face())

# 'Sketch7': 11 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch7_2:
    with BuildLine():
        Line((-4.3974, -15.213), (3.2681, -15.213))
        Line((3.2681, -15.213), (3.2681, -13.1197))
        Line((3.2681, -13.1197), (4.2019, -13.1197))
        Line((4.2019, -13.1197), (4.2019, -18.5087))
        Line((4.2019, -18.5087), (3.2681, -18.5087))
        Line((3.2681, -18.5087), (3.2681, -16.3177))
        Line((3.2681, -16.3177), (-3.3477, -16.3177))
        Line((-3.3477, -16.3177), (-1.9623, -18.2585))
        Line((-1.9623, -18.2585), (-2.9998, -18.2585))
        Line((-2.9998, -18.2585), (-4.3974, -16.2261))
        Line((-4.3974, -16.2261), (-4.3974, -15.213))
    _inc_edges_sk_Sketch7_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_2 = Wire.combine(_inc_edges_sk_Sketch7_2)[0]
_wire_sk_Sketch7_2 = _wire_sk_Sketch7_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch7_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch7_2.wrapped, True)
_face_sk_Sketch7_2 = Face(_mkf_sk_Sketch7_2.Face())

# 'Sketch7': 11 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch7_3:
    with BuildLine():
        Line((-4.3974, -9.3301), (3.2681, -9.3301))
        Line((3.2681, -9.3301), (3.2681, -7.2367))
        Line((3.2681, -7.2367), (4.2019, -7.2367))
        Line((4.2019, -7.2367), (4.2019, -12.6257))
        Line((4.2019, -12.6257), (3.2681, -12.6257))
        Line((3.2681, -12.6257), (3.2681, -10.4348))
        Line((3.2681, -10.4348), (-3.3477, -10.4348))
        Line((-3.3477, -10.4348), (-1.9623, -12.3755))
        Line((-1.9623, -12.3755), (-2.9998, -12.3755))
        Line((-2.9998, -12.3755), (-4.3974, -10.3432))
        Line((-4.3974, -10.3432), (-4.3974, -9.3301))
    _inc_edges_sk_Sketch7_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_3 = Wire.combine(_inc_edges_sk_Sketch7_3)[0]
_wire_sk_Sketch7_3 = _wire_sk_Sketch7_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch7_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch7_3.wrapped, True)
_face_sk_Sketch7_3 = Face(_mkf_sk_Sketch7_3.Face())

# 'Sketch7': 34 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch7_4:
    with BuildLine():
        RadiusArc((-4.0346, -5.002), (-3.4209, -5.5774), -2.3128)
        RadiusArc((-3.4209, -5.5774), (-2.7528, -5.9117), -3.3339)
        RadiusArc((-2.7528, -5.9117), (-2.148, -6.095), -4.8377)
        # Near-straight arc (sagitta=0.009257mm) replaced with Line
        Line((-2.148, -6.095), (-1.4544, -6.2203))
        # Near-straight arc (sagitta=0.008309mm) replaced with Line
        Line((-1.4544, -6.2203), (-0.6719, -6.2878))
        RadiusArc((-0.6719, -6.2878), (0.4585, -6.2875), -12.2545)
        # Near-straight arc (sagitta=0.008206mm) replaced with Line
        Line((0.4585, -6.2875), (1.2283, -6.2179))
        RadiusArc((1.2283, -6.2179), (2.1256, -6.0326), -6.4978)
        RadiusArc((2.1256, -6.0326), (2.8753, -5.7414), -4.3924)
        RadiusArc((2.8753, -5.7414), (3.4759, -5.3444), -3.0998)
        RadiusArc((3.4759, -5.3444), (3.9175, -4.8423), -2.391)
        RadiusArc((3.9175, -4.8423), (4.1985, -4.2351), -2.3486)
        RadiusArc((4.1985, -4.2351), (4.3038, -3.7108), -2.8258)
        RadiusArc((4.3038, -3.7108), (4.3189, -3.1339), -3.6083)
        RadiusArc((4.3189, -3.1339), (4.2432, -2.5889), -3.138)
        RadiusArc((4.2432, -2.5889), (4.0767, -2.1018), -2.5585)
        RadiusArc((4.0767, -2.1018), (3.7134, -1.5423), -2.3386)
        RadiusArc((3.7134, -1.5423), (3.3351, -1.1901), -2.549)
        RadiusArc((3.3351, -1.1901), (2.8679, -0.8965), -3.153)
        RadiusArc((2.8679, -0.8965), (2.1175, -0.5993), -4.1206)
        RadiusArc((2.1175, -0.5993), (1.2222, -0.4102), -5.9801)
        # Near-straight arc (sagitta=0.008653mm) replaced with Line
        Line((1.2222, -0.4102), (0.4555, -0.3393))
        # Near-straight arc (sagitta=0.00775mm) replaced with Line
        Line((0.4555, -0.3393), (-0.3882, -0.329))
        # Near-straight arc (sagitta=0.007728mm) replaced with Line
        Line((-0.3882, -0.329), (-1.193, -0.3772))
        # Near-straight arc (sagitta=0.008578mm) replaced with Line
        Line((-1.193, -0.3772), (-1.9119, -0.4833))
        RadiusArc((-1.9119, -0.4833), (-2.7367, -0.7147), -5.4176)
        RadiusArc((-2.7367, -0.7147), (-3.4087, -1.049), -3.6332)
        RadiusArc((-3.4087, -1.049), (-3.925, -1.4913), -2.4956)
        RadiusArc((-3.925, -1.4913), (-4.2079, -1.8974), -2.2405)
        RadiusArc((-4.2079, -1.8974), (-4.4015, -2.3672), -2.4033)
        RadiusArc((-4.4015, -2.3672), (-4.5206, -3.0927), -3.01)
        RadiusArc((-4.5206, -3.0927), (-4.506, -3.6935), -4.0761)
        RadiusArc((-4.506, -3.6935), (-4.4029, -4.2402), -3.171)
        RadiusArc((-4.4029, -4.2402), (-4.0346, -5.002), -2.4455)
    _inc_edges_sk_Sketch7_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_4 = Wire.combine(_inc_edges_sk_Sketch7_4)[0]
_wire_sk_Sketch7_4 = _wire_sk_Sketch7_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch7_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch7_4.wrapped, True)
_face_sk_Sketch7_4 = Face(_mkf_sk_Sketch7_4.Face())

# 'Sketch8': 34 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch8_5:
    with BuildLine():
        # Near-straight arc (sagitta=0.008696mm) replaced with Line
        Line((3.4117, -3.5673), (3.4229, -3.1892))
        # Near-straight arc (sagitta=0.009258mm) replaced with Line
        Line((3.4229, -3.1892), (3.3652, -2.8356))
        RadiusArc((3.3652, -2.8356), (3.1803, -2.4287), -1.423)
        RadiusArc((3.1803, -2.4287), (2.9607, -2.172), -1.427)
        RadiusArc((2.9607, -2.172), (2.5602, -1.8942), -1.7705)
        RadiusArc((2.5602, -1.8942), (2.032, -1.6854), -2.8579)
        RadiusArc((2.032, -1.6854), (1.3718, -1.5409), -4.8146)
        # Near-straight arc (sagitta=0.00565mm) replaced with Line
        Line((1.3718, -1.5409), (0.7902, -1.4747))
        # Near-straight arc (sagitta=0.008962mm) replaced with Line
        Line((0.7902, -1.4747), (-0.1008, -1.4426))
        # Near-straight arc (sagitta=0.004483mm) replaced with Line
        Line((-0.1008, -1.4426), (-0.8044, -1.4598))
        # Near-straight arc (sagitta=0.005042mm) replaced with Line
        Line((-0.8044, -1.4598), (-1.426, -1.5114))
        RadiusArc((-1.426, -1.5114), (-2.1271, -1.6338), -6.2074)
        # Near-straight arc (sagitta=0.006943mm) replaced with Line
        Line((-2.1271, -1.6338), (-2.5572, -1.7658))
        RadiusArc((-2.5572, -1.7658), (-3.0065, -1.9975), -2.186)
        # Near-straight arc (sagitta=0.009608mm) replaced with Line
        Line((-3.0065, -1.9975), (-3.2629, -2.2218))
        RadiusArc((-3.2629, -2.2218), (-3.5007, -2.5902), -1.355)
        RadiusArc((-3.5007, -2.5902), (-3.6196, -3.0381), -1.557)
        # Near-straight arc (sagitta=0.008638mm) replaced with Line
        Line((-3.6196, -3.0381), (-3.6308, -3.4244))
        RadiusArc((-3.6308, -3.4244), (-3.5429, -3.9038), -1.8919)
        # Near-straight arc (sagitta=0.009805mm) replaced with Line
        Line((-3.5429, -3.9038), (-3.4002, -4.2102))
        RadiusArc((-3.4002, -4.2102), (-3.1914, -4.4709), -1.3963)
        RadiusArc((-3.1914, -4.4709), (-2.8106, -4.7475), -1.6164)
        RadiusArc((-2.8106, -4.7475), (-2.2956, -4.952), -2.5479)
        # Near-straight arc (sagitta=0.006879mm) replaced with Line
        Line((-2.2956, -4.952), (-1.8103, -5.064))
        RadiusArc((-1.8103, -5.064), (-1.0311, -5.1584), -7.2775)
        # Near-straight arc (sagitta=0.008868mm) replaced with Line
        Line((-1.0311, -5.1584), (-0.1008, -5.1899))
        # Near-straight arc (sagitta=0.00827mm) replaced with Line
        Line((-0.1008, -5.1899), (0.8058, -5.158))
        # Near-straight arc (sagitta=0.009682mm) replaced with Line
        Line((0.8058, -5.158), (1.5727, -5.0623))
        # Near-straight arc (sagitta=0.006468mm) replaced with Line
        Line((1.5727, -5.0623), (2.0561, -4.9487))
        # Near-straight arc (sagitta=0.007451mm) replaced with Line
        Line((2.0561, -4.9487), (2.461, -4.7991))
        # Near-straight arc (sagitta=0.008607mm) replaced with Line
        Line((2.461, -4.7991), (2.7896, -4.6123))
        RadiusArc((2.7896, -4.6123), (3.1214, -4.298), -1.5594)
        # Near-straight arc (sagitta=0.009999mm) replaced with Line
        Line((3.1214, -4.298), (3.291, -4.013))
        RadiusArc((3.291, -4.013), (3.4117, -3.5673), -1.5417)
    _inc_edges_sk_Sketch8_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_5 = Wire.combine(_inc_edges_sk_Sketch8_5)[0]
_wire_sk_Sketch8_5 = _wire_sk_Sketch8_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch8_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch8_5.wrapped, True)
_face_sk_Sketch8_5 = Face(_mkf_sk_Sketch8_5.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch5
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0014, -55.0006, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6265, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5_p0 --
    _face = _face_sk_Sketch7_2
    _vec = Vector(0.0, 0.0, 1.0) * 2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000 mm
    
    # -- Extrude5_p1 --
    _face = _face_sk_Sketch7_3
    _vec = Vector(0.0, 0.0, 1.0) * 2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000 mm
    
    # -- Extrude5_p2 --
    _face = _face_sk_Sketch7_4
    _vec = Vector(0.0, 0.0, 1.0) * 2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch8_5
    _vec = Vector(0.0, 0.0, 1.0) * 0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
