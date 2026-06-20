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

# 'Sketch15': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch15:
    with BuildLine():
        Line((0.6, 54.0034), (0.6, 53.9))
        Line((0.6, 53.9), (-9.0, 53.9))
        Line((-9.0, 53.9), (-9.0, 50.5))
        RadiusArc((-9.0, 50.5), (-6.0, 47.5102), -2.9709)
        Line((-6.0, 47.5102), (-6.0, -62.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -62.5), (0.0, -68.5), -6.0)
        RadiusArc((0.0, -68.5), (6.0, -62.5), -6.0)
        Line((6.0, -62.5), (6.0, 47.5102))
        RadiusArc((6.0, 47.5102), (9.0, 50.5), -2.9709)
        Line((9.0, 50.5), (9.0, 62.5))
        Line((9.0, 62.5), (-9.0, 62.5))
        Line((-9.0, 62.5), (-9.0, 59.1))
        Line((-9.0, 59.1), (0.6, 59.1))
        Line((0.6, 59.1), (0.6, 58.9966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 58.9966), (3.9, 56.5), 2.5944)
        RadiusArc((3.9, 56.5), (0.6, 54.0034), 2.5944)
    _inc_edges_sk_Sketch15 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch15 = Wire.combine(_inc_edges_sk_Sketch15)[0]
_wire_sk_Sketch15 = _wire_sk_Sketch15.moved(_inclined_plane_1.location)
_mkf_sk_Sketch15 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch15.wrapped, True)
_face_sk_Sketch15 = Face(_mkf_sk_Sketch15.Face())

# 'Sketch16': 38 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch16_2:
    with BuildLine():
        RadiusArc((2.2794, -6.2884), (3.727, -5.4507), -2.277)
        RadiusArc((3.727, -5.4507), (4.1746, -4.5644), -2.692)
        RadiusArc((4.1746, -4.5644), (4.3216, -3.5722), -4.1866)
        RadiusArc((4.3216, -3.5722), (4.2371, -2.5273), -4.192)
        RadiusArc((4.2371, -2.5273), (3.8239, -1.5195), -2.763)
        RadiusArc((3.8239, -1.5195), (2.9575, -0.7217), -2.4893)
        RadiusArc((2.9575, -0.7217), (1.4005, -0.3623), -3.4652)
        RadiusArc((1.4005, -0.3623), (0.0063, -0.6973), -2.6593)
        RadiusArc((0.0063, -0.6973), (-0.7255, -1.3262), -2.4911)
        RadiusArc((-0.7255, -1.3262), (-1.2021, -2.3503), -2.633)
        RadiusArc((-1.2021, -2.3503), (-1.275, -3.4162), -3.8371)
        RadiusArc((-1.275, -3.4162), (-0.7355, -4.9153), -2.6197)
        Line((-0.7355, -4.9153), (-3.4637, -4.7505))
        Line((-3.4637, -4.7505), (-3.4637, -0.8628))
        Line((-3.4637, -0.8628), (-4.3974, -0.8628))
        Line((-4.3974, -0.8628), (-4.3974, -5.7514))
        Line((-4.3974, -5.7514), (0.2287, -6.0382))
        Line((0.2287, -6.0382), (0.2287, -4.9641))
        RadiusArc((0.2287, -4.9641), (-0.0738, -4.554), 3.0411)
        # Near-straight arc (sagitta=0.00928mm) replaced with Line
        Line((-0.0738, -4.554), (-0.2473, -4.1951))
        RadiusArc((-0.2473, -4.1951), (-0.3571, -3.7676), 2.2463)
        RadiusArc((-0.3571, -3.7676), (-0.3854, -3.2527), 2.5765)
        RadiusArc((-0.3854, -3.2527), (-0.3076, -2.7377), 2.3161)
        RadiusArc((-0.3076, -2.7377), (-0.0674, -2.2226), 1.7267)
        RadiusArc((-0.0674, -2.2226), (0.2518, -1.8818), 1.6417)
        RadiusArc((0.2518, -1.8818), (0.6476, -1.6468), 1.6212)
        RadiusArc((0.6476, -1.6468), (1.1127, -1.5237), 1.8055)
        RadiusArc((1.1127, -1.5237), (1.6668, -1.5125), 2.5271)
        RadiusArc((1.6668, -1.5125), (2.2094, -1.6126), 2.5253)
        RadiusArc((2.2094, -1.6126), (2.7433, -1.8795), 1.8371)
        RadiusArc((2.7433, -1.8795), (3.1396, -2.2983), 1.6334)
        RadiusArc((3.1396, -2.2983), (3.3675, -2.8422), 1.6684)
        RadiusArc((3.3675, -2.8422), (3.4244, -3.2732), 2.2406)
        RadiusArc((3.4244, -3.2732), (3.3758, -3.9219), 2.81)
        RadiusArc((3.3758, -3.9219), (3.2227, -4.3669), 1.8119)
        RadiusArc((3.2227, -4.3669), (2.8599, -4.8244), 1.4519)
        RadiusArc((2.8599, -4.8244), (2.1512, -5.1777), 1.7531)
        Line((2.1512, -5.1777), (2.2794, -6.2884))
    _inc_edges_sk_Sketch16_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16_2 = Wire.combine(_inc_edges_sk_Sketch16_2)[0]
_wire_sk_Sketch16_2 = _wire_sk_Sketch16_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch16_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch16_2.wrapped, True)
_face_sk_Sketch16_2 = Face(_mkf_sk_Sketch16_2.Face())

# 'Sketch16': 50 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch16_3:
    with BuildLine():
        # Near-straight arc (sagitta=0.001992mm) replaced with Line
        Line((1.0786, -9.7003), (0.6025, -9.1372))
        # Near-straight arc (sagitta=0.003808mm) replaced with Line
        Line((0.6025, -9.1372), (0.116, -8.5981))
        # Near-straight arc (sagitta=0.005205mm) replaced with Line
        Line((0.116, -8.5981), (-0.2522, -8.2394))
        # Near-straight arc (sagitta=0.009223mm) replaced with Line
        Line((-0.2522, -8.2394), (-0.7136, -7.8803))
        RadiusArc((-0.7136, -7.8803), (-1.175, -7.6182), -3.4463)
        RadiusArc((-1.175, -7.6182), (-1.6377, -7.4544), -2.6059)
        RadiusArc((-1.6377, -7.4544), (-2.1698, -7.3893), -2.2475)
        RadiusArc((-2.1698, -7.3893), (-2.7353, -7.439), -3.1536)
        RadiusArc((-2.7353, -7.439), (-3.446, -7.6999), -2.2439)
        RadiusArc((-3.446, -7.6999), (-3.9913, -8.184), -1.9254)
        RadiusArc((-3.9913, -8.184), (-4.3048, -8.7433), -2.1943)
        RadiusArc((-4.3048, -8.7433), (-4.482, -9.4363), -2.9842)
        RadiusArc((-4.482, -9.4363), (-4.5229, -10.2428), -4.5233)
        RadiusArc((-4.5229, -10.2428), (-4.4578, -10.8284), -3.824)
        RadiusArc((-4.4578, -10.8284), (-4.2543, -11.4743), -2.9814)
        RadiusArc((-4.2543, -11.4743), (-3.9152, -12.0246), -2.5512)
        RadiusArc((-3.9152, -12.0246), (-3.4616, -12.457), -2.4119)
        RadiusArc((-3.4616, -12.457), (-2.7937, -12.7904), -2.303)
        RadiusArc((-2.7937, -12.7904), (-2.2735, -12.9004), -2.7502)
        Line((-2.2735, -12.9004), (-2.1698, -11.7774))
        RadiusArc((-2.1698, -11.7774), (-2.6722, -11.6539), 1.994)
        RadiusArc((-2.6722, -11.6539), (-3.0155, -11.4592), 1.4838)
        RadiusArc((-3.0155, -11.4592), (-3.3358, -11.1178), 1.4321)
        RadiusArc((-3.3358, -11.1178), (-3.5613, -10.6088), 1.5705)
        RadiusArc((-3.5613, -10.6088), (-3.6206, -9.9898), 2.0862)
        RadiusArc((-3.6206, -9.9898), (-3.5113, -9.3912), 2.0577)
        RadiusArc((-3.5113, -9.3912), (-3.2821, -8.9902), 1.3754)
        RadiusArc((-3.2821, -8.9902), (-3.0009, -8.7416), 1.2395)
        # Near-straight arc (sagitta=0.009198mm) replaced with Line
        Line((-3.0009, -8.7416), (-2.7248, -8.6087))
        RadiusArc((-2.7248, -8.6087), (-2.2261, -8.5202), 1.6533)
        RadiusArc((-2.2261, -8.5202), (-1.7955, -8.5451), 2.1861)
        RadiusArc((-1.7955, -8.5451), (-1.3059, -8.6994), 1.8019)
        RadiusArc((-1.3059, -8.6994), (-0.8671, -8.9763), 2.4829)
        RadiusArc((-0.8671, -8.9763), (-0.3998, -9.3912), 3.858)
        # Near-straight arc (sagitta=0.005892mm) replaced with Line
        Line((-0.3998, -9.3912), (0.0542, -9.8939))
        # Near-straight arc (sagitta=0.004282mm) replaced with Line
        Line((0.0542, -9.8939), (0.5108, -10.4614))
        # Near-straight arc (sagitta=0.005971mm) replaced with Line
        Line((0.5108, -10.4614), (1.1191, -11.1806))
        RadiusArc((1.1191, -11.1806), (1.8139, -11.8853), -8.9331)
        # Near-straight arc (sagitta=0.008532mm) replaced with Line
        Line((1.8139, -11.8853), (2.3917, -12.3634))
        RadiusArc((2.3917, -12.3634), (2.9694, -12.7265), -5.1575)
        # Near-straight arc (sagitta=0.00556mm) replaced with Line
        Line((2.9694, -12.7265), (3.4268, -12.9492))
        Line((3.4268, -12.9492), (4.2019, -12.9492))
        Line((4.2019, -12.9492), (4.2019, -7.255))
        Line((4.2019, -7.255), (3.2681, -7.255))
        Line((3.2681, -7.255), (3.2681, -11.7408))
        RadiusArc((3.2681, -11.7408), (2.7434, -11.4376), 2.6198)
        # Near-straight arc (sagitta=0.006611mm) replaced with Line
        Line((2.7434, -11.4376), (2.3801, -11.1427))
        # Near-straight arc (sagitta=0.007284mm) replaced with Line
        Line((2.3801, -11.1427), (1.9254, -10.6929))
        # Near-straight arc (sagitta=0.004516mm) replaced with Line
        Line((1.9254, -10.6929), (1.4966, -10.2079))
        # Near-straight arc (sagitta=0.001358mm) replaced with Line
        Line((1.4966, -10.2079), (1.0786, -9.7003))
    _inc_edges_sk_Sketch16_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16_3 = Wire.combine(_inc_edges_sk_Sketch16_3)[0]
_wire_sk_Sketch16_3 = _wire_sk_Sketch16_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch16_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch16_3.wrapped, True)
_face_sk_Sketch16_3 = Face(_mkf_sk_Sketch16_3.Face())

# 'Sketch16': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch16_4:
    with BuildLine():
        Line((-4.3974, -16.119), (3.2681, -16.119))
        Line((3.2681, -16.119), (3.2681, -14.0257))
        Line((3.2681, -14.0257), (4.2019, -14.0257))
        Line((4.2019, -14.0257), (4.2019, -19.4147))
        Line((4.2019, -19.4147), (3.2681, -19.4147))
        Line((3.2681, -19.4147), (3.2681, -17.2237))
        Line((3.2681, -17.2237), (-3.3477, -17.2237))
        Line((-3.3477, -17.2237), (-1.9623, -19.1645))
        Line((-1.9623, -19.1645), (-2.9998, -19.1645))
        Line((-2.9998, -19.1645), (-4.3974, -17.1321))
        Line((-4.3974, -17.1321), (-4.3974, -16.119))
    _inc_edges_sk_Sketch16_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16_4 = Wire.combine(_inc_edges_sk_Sketch16_4)[0]
_wire_sk_Sketch16_4 = _wire_sk_Sketch16_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch16_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch16_4.wrapped, True)
_face_sk_Sketch16_4 = Face(_mkf_sk_Sketch16_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude12 ---
    # -- Extrude12 --
    _face = _face_sk_Sketch15
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0001, -62.5002, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6252, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude13 ---
    # -- Extrude13_p0 --
    _face = _face_sk_Sketch16_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude13_p1 --
    _face = _face_sk_Sketch16_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude13_p2 --
    _face = _face_sk_Sketch16_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
