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
    origin=Vector(0.0, 0.0, -45.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((6.0, 165.034), (6.0, 164.0))
        Line((6.0, 164.0), (-90.0, 164.0))
        Line((-90.0, 164.0), (-90.0, 130.0))
        RadiusArc((-90.0, 130.0), (-60.0, 100.102), -29.8459)
        Line((-60.0, 100.102), (-60.0, -250.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -250.0), (0.0, -309.9999), -60.0)
        RadiusArc((0.0, -309.9999), (60.0, -250.0), -60.0)
        Line((60.0, -250.0), (60.0, 100.102))
        RadiusArc((60.0, 100.102), (90.0, 130.0), -29.7792)
        Line((90.0, 130.0), (90.0, 250.0))
        Line((90.0, 250.0), (-90.0, 250.0))
        Line((-90.0, 250.0), (-90.0, 216.0))
        Line((-90.0, 216.0), (6.0, 216.0))
        Line((6.0, 216.0), (6.0, 214.966))
        # Arc split: sweep=211.59deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 214.966), (39.0069, 190.0), 25.9454)
        RadiusArc((39.0069, 190.0), (6.0, 165.034), 25.9454)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 26 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((21.5121, -119.666), (22.7937, -130.774))
        RadiusArc((22.7937, -130.774), (35.7028, -124.173), -23.1916)
        RadiusArc((35.7028, -124.173), (42.3993, -110.905), -26.0301)
        RadiusArc((42.3993, -110.905), (43.2043, -100.144), -47.1051)
        RadiusArc((43.2043, -100.144), (42.3711, -93.1621), -56.4003)
        RadiusArc((42.3711, -93.1621), (35.4271, -79.5998), -26.4596)
        RadiusArc((35.4271, -79.5998), (25.4451, -73.2736), -25.0008)
        RadiusArc((25.4451, -73.2736), (3.8243, -73.1538), -34.8655)
        RadiusArc((3.8243, -73.1538), (-8.8437, -83.4366), -24.6419)
        RadiusArc((-8.8437, -83.4366), (-12.8148, -97.6122), -30.5653)
        RadiusArc((-12.8148, -97.6122), (-7.3552, -117.042), -29.3172)
        Line((-7.3552, -117.042), (-34.6368, -115.395))
        Line((-34.6368, -115.395), (-34.6368, -76.5172))
        Line((-34.6368, -76.5172), (-43.9743, -76.5172))
        Line((-43.9743, -76.5172), (-43.9743, -125.403))
        Line((-43.9743, -125.403), (2.2873, -128.272))
        Line((2.2873, -128.272), (2.2873, -117.53))
        RadiusArc((2.2873, -117.53), (-3.8629, -102.0337), 22.7529)
        RadiusArc((-3.8629, -102.0337), (-2.0749, -92.55), 22.1202)
        RadiusArc((-2.0749, -92.55), (2.5179, -86.7075), 16.254)
        RadiusArc((2.5179, -86.7075), (12.1399, -83.0147), 16.9188)
        RadiusArc((12.1399, -83.0147), (30.8505, -90.0859), 21.2918)
        RadiusArc((30.8505, -90.0859), (34.173, -99.4925), 19.0306)
        RadiusArc((34.173, -99.4925), (33.7576, -107.108), 27.3715)
        RadiusArc((33.7576, -107.108), (28.5817, -116.1467), 15.0211)
        RadiusArc((28.5817, -116.1467), (21.5121, -119.666), 18.0803)
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch4': 15 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        RadiusArc((-45.2567, -31.7396), (-44.0286, -42.4016), -36.2637)
        RadiusArc((-44.0286, -42.4016), (-35.6331, -54.7728), -23.0039)
        RadiusArc((-35.6331, -54.7728), (-21.4803, -60.9496), -40.6427)
        RadiusArc((-21.4803, -60.9496), (1.8343, -62.9736), -102.0076)
        RadiusArc((1.8343, -62.9736), (21.256, -60.3264), -71.9327)
        RadiusArc((21.256, -60.3264), (34.7586, -53.4442), -34.7533)
        RadiusArc((34.7586, -53.4442), (41.9846, -42.3511), -23.4397)
        RadiusArc((41.9846, -42.3511), (43.0373, -29.4582), -34.2145)
        RadiusArc((43.0373, -29.4582), (35.974, -14.1847), -24.2971)
        RadiusArc((35.974, -14.1847), (23.1871, -6.6348), -35.9148)
        RadiusArc((23.1871, -6.6348), (4.5554, -3.3926), -72.7099)
        RadiusArc((4.5554, -3.3926), (-14.4218, -4.0611), -102.6842)
        RadiusArc((-14.4218, -4.0611), (-30.9178, -8.6899), -48.9675)
        RadiusArc((-30.9178, -8.6899), (-42.079, -18.974), -23.8389)
        RadiusArc((-42.079, -18.974), (-45.2567, -31.7396), -29.5276)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch3': 16 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        RadiusArc((5.7977, -14.6063), (-16.1491, -15.3627), -112.2372)
        RadiusArc((-16.1491, -15.3627), (-26.8237, -18.174), -38.2284)
        RadiusArc((-26.8237, -18.174), (-35.4161, -26.9473), -14.4054)
        RadiusArc((-35.4161, -26.9473), (-36.3084, -34.2435), -19.6369)
        RadiusArc((-36.3084, -34.2435), (-26.9598, -48.0449), -15.1018)
        RadiusArc((-26.9598, -48.0449), (-16.297, -50.9355), -36.6987)
        RadiusArc((-16.297, -50.9355), (-3.4756, -51.8793), -95.0638)
        RadiusArc((-3.4756, -51.8793), (10.106, -51.4006), -124.139)
        RadiusArc((10.106, -51.4006), (19.037, -49.9054), -65.0257)
        RadiusArc((19.037, -49.9054), (26.8779, -46.7918), -31.3041)
        RadiusArc((26.8779, -46.7918), (32.9105, -40.13), -15.5508)
        RadiusArc((32.9105, -40.13), (34.2294, -31.8919), -17.4725)
        RadiusArc((34.2294, -31.8919), (33.3049, -27.2694), -16.1018)
        RadiusArc((33.3049, -27.2694), (28.7216, -20.956), -14.2131)
        RadiusArc((28.7216, -20.956), (21.7639, -17.316), -22.8619)
        RadiusArc((21.7639, -17.316), (5.7977, -14.6063), -59.1782)
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0001, -249.9998, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.2499, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * 15.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 15.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * 25.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 25.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(0.0, 0.0, 1.0) * 5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
