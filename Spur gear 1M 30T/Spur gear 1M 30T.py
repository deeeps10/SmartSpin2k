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

# 'Sketch7': 120 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(26.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch7:
    with BuildLine():
        Line((-29.6814, 16.4664), (-29.6814, 15.5336))
        RadiusArc((-29.6814, 15.5336), (-31.8806, 14.6189), -6.0)
        Line((-31.8806, 14.6189), (-31.8194, 14.0368))
        RadiusArc((-31.8194, 14.0368), (-29.4781, 13.5993), -6.0)
        Line((-29.4781, 13.5993), (-29.2841, 12.6869))
        RadiusArc((-29.2841, 12.6869), (-31.2451, 11.3349), -6.0)
        Line((-31.2451, 11.3349), (-31.0642, 10.7782))
        RadiusArc((-31.0642, 10.7782), (-28.6831, 10.8371), -6.0)
        Line((-28.6831, 10.8371), (-28.3037, 9.9849))
        RadiusArc((-28.3037, 9.9849), (-29.9407, 8.2548), -6.0)
        Line((-29.9407, 8.2548), (-29.648, 7.7479))
        RadiusArc((-29.648, 7.7479), (-27.3312, 8.3005), -6.0)
        Line((-27.3312, 8.3005), (-26.7829, 7.5459))
        RadiusArc((-26.7829, 7.5459), (-28.0244, 5.5132), -6.0001)
        Line((-28.0244, 5.5132), (-27.6328, 5.0782))
        RadiusArc((-27.6328, 5.0782), (-25.4815, 6.1005), -6.0)
        Line((-25.4815, 6.1005), (-24.7883, 5.4763))
        RadiusArc((-24.7883, 5.4763), (-25.58, 3.2299), -6.0)
        Line((-25.58, 3.2299), (-25.1065, 2.8859))
        RadiusArc((-25.1065, 2.8859), (-23.2147, 4.3331), -6.0)
        Line((-23.2147, 4.3331), (-22.4069, 3.8667))
        RadiusArc((-22.4069, 3.8667), (-22.7143, 1.5048), -6.0)
        Line((-22.7143, 1.5048), (-22.1796, 1.2667))
        RadiusArc((-22.1796, 1.2667), (-20.6301, 3.0756), -6.0)
        Line((-20.6301, 3.0756), (-19.7429, 2.7873))
        RadiusArc((-19.7429, 2.7873), (-19.5526, 0.4131), -6.0)
        Line((-19.5526, 0.4131), (-18.9801, 0.2914))
        RadiusArc((-18.9801, 0.2914), (-17.8405, 2.3829), -6.0)
        Line((-17.8405, 2.3829), (-16.9128, 2.2854))
        RadiusArc((-16.9128, 2.2854), (-16.2329, 0.0027), -6.0)
        Line((-16.2329, 0.0027), (-15.6476, 0.0027))
        RadiusArc((-15.6476, 0.0027), (-14.9678, 2.2854), -6.0)
        Line((-14.9678, 2.2854), (-14.0401, 2.3829))
        RadiusArc((-14.0401, 2.3829), (-12.9005, 0.2914), -6.0)
        Line((-12.9005, 0.2914), (-12.328, 0.4131))
        RadiusArc((-12.328, 0.4131), (-12.1376, 2.7873), -6.0)
        Line((-12.1376, 2.7873), (-11.2505, 3.0756))
        RadiusArc((-11.2505, 3.0756), (-9.7009, 1.2667), -6.0)
        Line((-9.7009, 1.2667), (-9.1662, 1.5048))
        RadiusArc((-9.1662, 1.5048), (-9.4736, 3.8667), -6.0)
        Line((-9.4736, 3.8667), (-8.6658, 4.3331))
        RadiusArc((-8.6658, 4.3331), (-6.7741, 2.8859), -6.0)
        Line((-6.7741, 2.8859), (-6.3005, 3.2299))
        RadiusArc((-6.3005, 3.2299), (-7.0923, 5.4763), -6.0)
        Line((-7.0923, 5.4763), (-6.3991, 6.1005))
        RadiusArc((-6.3991, 6.1005), (-4.2478, 5.0782), -6.0)
        Line((-4.2478, 5.0782), (-3.8561, 5.5132))
        RadiusArc((-3.8561, 5.5132), (-5.0976, 7.5459), -6.0)
        Line((-5.0976, 7.5459), (-4.5494, 8.3005))
        RadiusArc((-4.5494, 8.3005), (-2.2325, 7.7479), -6.0)
        Line((-2.2325, 7.7479), (-1.9399, 8.2548))
        RadiusArc((-1.9399, 8.2548), (-3.5769, 9.9849), -6.0)
        Line((-3.5769, 9.9849), (-3.1975, 10.8371))
        RadiusArc((-3.1975, 10.8371), (-0.8164, 10.7782), -6.0)
        Line((-0.8164, 10.7782), (-0.6355, 11.3349))
        RadiusArc((-0.6355, 11.3349), (-2.5964, 12.6869), -6.0)
        Line((-2.5964, 12.6869), (-2.4025, 13.5993))
        RadiusArc((-2.4025, 13.5993), (-0.0612, 14.0368), -6.0)
        Line((-0.0612, 14.0368), (0.0, 14.6189))
        RadiusArc((0.0, 14.6189), (-2.1992, 15.5336), -6.0)
        Line((-2.1992, 15.5336), (-2.1992, 16.4664))
        RadiusArc((-2.1992, 16.4664), (-0.0, 17.3811), -5.9999)
        Line((-0.0, 17.3811), (-0.0612, 17.9632))
        RadiusArc((-0.0612, 17.9632), (-2.4025, 18.4007), -6.0)
        Line((-2.4025, 18.4007), (-2.5964, 19.3131))
        RadiusArc((-2.5964, 19.3131), (-0.6355, 20.6651), -6.0)
        Line((-0.6355, 20.6651), (-0.8164, 21.2218))
        RadiusArc((-0.8164, 21.2218), (-3.1975, 21.1629), -6.0001)
        Line((-3.1975, 21.1629), (-3.5769, 22.0151))
        RadiusArc((-3.5769, 22.0151), (-1.9399, 23.7452), -6.0)
        Line((-1.9399, 23.7452), (-2.2325, 24.2521))
        RadiusArc((-2.2325, 24.2521), (-4.5494, 23.6995), -5.9999)
        Line((-4.5494, 23.6995), (-5.0976, 24.4541))
        RadiusArc((-5.0976, 24.4541), (-3.8561, 26.4868), -6.0001)
        Line((-3.8561, 26.4868), (-4.2478, 26.9218))
        RadiusArc((-4.2478, 26.9218), (-6.3991, 25.8995), -6.0)
        Line((-6.3991, 25.8995), (-7.0923, 26.5237))
        RadiusArc((-7.0923, 26.5237), (-6.3005, 28.7701), -6.0)
        Line((-6.3005, 28.7701), (-6.7741, 29.1141))
        RadiusArc((-6.7741, 29.1141), (-8.6658, 27.6669), -6.0)
        Line((-8.6658, 27.6669), (-9.4736, 28.1333))
        RadiusArc((-9.4736, 28.1333), (-9.1662, 30.4953), -6.0)
        Line((-9.1662, 30.4953), (-9.7009, 30.7333))
        RadiusArc((-9.7009, 30.7333), (-11.2505, 28.9244), -5.9999)
        Line((-11.2505, 28.9244), (-12.1376, 29.2127))
        RadiusArc((-12.1376, 29.2127), (-12.328, 31.5869), -6.0)
        Line((-12.328, 31.5869), (-12.9005, 31.7086))
        RadiusArc((-12.9005, 31.7086), (-14.0401, 29.6171), -6.0)
        Line((-14.0401, 29.6171), (-14.9678, 29.7146))
        RadiusArc((-14.9678, 29.7146), (-15.6476, 31.9973), -6.0001)
        Line((-15.6476, 31.9973), (-16.2329, 31.9973))
        RadiusArc((-16.2329, 31.9973), (-16.9128, 29.7146), -6.0)
        Line((-16.9128, 29.7146), (-17.8405, 29.6171))
        RadiusArc((-17.8405, 29.6171), (-18.9801, 31.7086), -6.0)
        Line((-18.9801, 31.7086), (-19.5526, 31.5869))
        RadiusArc((-19.5526, 31.5869), (-19.7429, 29.2127), -6.0)
        Line((-19.7429, 29.2127), (-20.6301, 28.9244))
        RadiusArc((-20.6301, 28.9244), (-22.1796, 30.7333), -6.0)
        Line((-22.1796, 30.7333), (-22.7143, 30.4953))
        RadiusArc((-22.7143, 30.4953), (-22.4069, 28.1333), -6.0)
        Line((-22.4069, 28.1333), (-23.2147, 27.6669))
        RadiusArc((-23.2147, 27.6669), (-25.1065, 29.1141), -6.0)
        Line((-25.1065, 29.1141), (-25.58, 28.7701))
        RadiusArc((-25.58, 28.7701), (-24.7883, 26.5237), -6.0)
        Line((-24.7883, 26.5237), (-25.4815, 25.8995))
        RadiusArc((-25.4815, 25.8995), (-27.6328, 26.9218), -6.0)
        Line((-27.6328, 26.9218), (-28.0244, 26.4868))
        RadiusArc((-28.0244, 26.4868), (-26.7829, 24.4541), -6.0001)
        Line((-26.7829, 24.4541), (-27.3312, 23.6995))
        RadiusArc((-27.3312, 23.6995), (-29.648, 24.2521), -5.9999)
        Line((-29.648, 24.2521), (-29.9407, 23.7452))
        RadiusArc((-29.9407, 23.7452), (-28.3037, 22.0151), -6.0)
        Line((-28.3037, 22.0151), (-28.6831, 21.1629))
        RadiusArc((-28.6831, 21.1629), (-31.0642, 21.2218), -6.0001)
        Line((-31.0642, 21.2218), (-31.2451, 20.6651))
        RadiusArc((-31.2451, 20.6651), (-29.2841, 19.3131), -6.0)
        Line((-29.2841, 19.3131), (-29.4781, 18.4007))
        RadiusArc((-29.4781, 18.4007), (-31.8194, 17.9632), -6.0)
        Line((-31.8194, 17.9632), (-31.8806, 17.3811))
        RadiusArc((-31.8806, 17.3811), (-29.6814, 16.4664), -5.9999)
    _inc_edges_sk_Sketch7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7 = Wire.combine(_inc_edges_sk_Sketch7)[0]
_wire_sk_Sketch7 = _wire_sk_Sketch7.moved(_inclined_plane_1.location)
_mkf_sk_Sketch7 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch7.wrapped, True)
_face_sk_Sketch7 = Face(_mkf_sk_Sketch7.Face())

# 'Sketch8': circle on inclined plane
_inclined_plane_2 = Plane(
    origin=Vector(26.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, -0.0, -0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch8_2:
    with Locations((-15.9403, 16.0)):
        Circle(radius=4.5)

# 'Sketch9': circle on inclined plane
_inclined_plane_3 = Plane(
    origin=Vector(25.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, -0.0, -0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch9_3:
    with Locations((-15.9403, 16.0)):
        Circle(radius=3.85)

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 18.9101, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch10_4:
    with BuildLine():
        Line((13.0, -13.55), (0.0, -13.55))
        Line((0.0, -13.55), (0.0, -18.45))
        Line((0.0, -18.45), (13.0, -18.45))
        Line((13.0, -18.45), (13.0, -13.55))
    _inc_edges_sk_Sketch10_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_4 = Wire.combine(_inc_edges_sk_Sketch10_4)[0]
_wire_sk_Sketch10_4 = _wire_sk_Sketch10_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch10_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch10_4.wrapped, True)
_face_sk_Sketch10_4 = Face(_mkf_sk_Sketch10_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch7
    _vec = Vector(-1.0, 0.0, 0.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -8.999998569 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    extrude(sk_Sketch8_2.sketch, amount=1.0, mode=Mode.ADD)
    # Fusion depth expression: 1.000001431 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    extrude(sk_Sketch9_3.sketch, amount=25.0, mode=Mode.ADD)
    # Fusion depth expression: 25.000000 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    _face = _face_sk_Sketch10_4
    _vec = Vector(-0.0, 1.0, 0.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 3.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
