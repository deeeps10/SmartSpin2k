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
        Line((0.6, 6.4), (-9.0, 6.4))
        Line((-9.0, 6.4), (-9.0, 3.0))
        RadiusArc((-9.0, 3.0), (-6.0, 0.0102), -2.9818)
        Line((-6.0, 0.0102), (-6.0, -15.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -15.0), (0.0, -21.0), -6.0)
        RadiusArc((0.0, -21.0), (6.0, -15.0), -6.0)
        Line((6.0, -15.0), (6.0, 0.0102))
        RadiusArc((6.0, 0.0102), (9.0, 3.0), -2.9845)
        Line((9.0, 3.0), (9.0, 15.0))
        Line((9.0, 15.0), (-9.0, 15.0))
        Line((-9.0, 15.0), (-9.0, 11.6))
        Line((-9.0, 11.6), (0.6, 11.6))
        Line((0.6, 11.6), (0.6, 11.4966))
        # Arc split: sweep=211.59deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 11.4966), (3.9006, 9.0), 2.5945)
        RadiusArc((3.9006, 9.0), (0.6, 6.5034), 2.5945)
        Line((0.6, 6.5034), (0.6, 6.4))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 44 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-0.6519, -10.3222), (-0.7621, -9.5928), -2.6748)
        RadiusArc((-0.7621, -9.5928), (-1.0976, -8.9761), -1.6843)
        RadiusArc((-1.0976, -8.9761), (-1.7836, -8.5782), -1.2481)
        RadiusArc((-1.7836, -8.5782), (-2.7137, -8.6312), -1.875)
        RadiusArc((-2.7137, -8.6312), (-3.4784, -9.3364), -1.2436)
        RadiusArc((-3.4784, -9.3364), (-3.6207, -10.2496), -2.1694)
        RadiusArc((-3.6207, -10.2496), (-3.4221, -11.0699), -1.895)
        RadiusArc((-3.4221, -11.0699), (-3.0298, -11.5414), -1.4349)
        RadiusArc((-3.0298, -11.5414), (-2.2003, -11.8507), -1.5586)
        Line((-2.2003, -11.8507), (-2.2858, -12.9553))
        RadiusArc((-2.2858, -12.9553), (-3.7675, -12.2693), 2.3066)
        RadiusArc((-3.7675, -12.2693), (-4.3125, -11.4077), 2.5548)
        RadiusArc((-4.3125, -11.4077), (-4.5151, -10.4499), 3.1774)
        RadiusArc((-4.5151, -10.4499), (-4.4829, -9.4928), 4.7305)
        RadiusArc((-4.4829, -9.4928), (-4.0019, -8.229), 2.5689)
        RadiusArc((-4.0019, -8.229), (-2.6599, -7.4423), 1.9627)
        RadiusArc((-2.6599, -7.4423), (-1.4579, -7.5447), 2.4629)
        RadiusArc((-1.4579, -7.5447), (-0.5006, -8.3956), 1.7979)
        RadiusArc((-0.5006, -8.3956), (-0.2106, -9.2141), 3.1513)
        Line((-0.2106, -9.2141), (-0.1862, -9.2141))
        RadiusArc((-0.1862, -9.2141), (-0.0329, -8.5111), 3.2714)
        RadiusArc((-0.0329, -8.5111), (0.261, -7.9447), 2.1783)
        RadiusArc((0.261, -7.9447), (0.9401, -7.3691), 1.8692)
        RadiusArc((0.9401, -7.3691), (1.8278, -7.1757), 1.926)
        RadiusArc((1.8278, -7.1757), (2.9444, -7.391), 2.7798)
        RadiusArc((2.9444, -7.391), (3.8334, -8.1458), 2.1037)
        RadiusArc((3.8334, -8.1458), (4.321, -9.9087), 3.3042)
        RadiusArc((4.321, -9.9087), (4.1564, -11.336), 4.1462)
        RadiusArc((4.1564, -11.336), (3.6539, -12.2781), 2.6323)
        RadiusArc((3.6539, -12.2781), (2.5676, -12.9788), 2.2736)
        RadiusArc((2.5676, -12.9788), (1.9926, -13.1019), 3.3039)
        Line((1.9926, -13.1019), (1.8888, -11.9666))
        RadiusArc((1.8888, -11.9666), (2.8653, -11.5614), -1.9418)
        RadiusArc((2.8653, -11.5614), (3.3535, -10.6972), -1.5958)
        RadiusArc((3.3535, -10.6972), (3.3255, -9.3961), -2.7623)
        RadiusArc((3.3255, -9.3961), (2.9494, -8.7291), -1.5425)
        RadiusArc((2.9494, -8.7291), (2.1926, -8.3507), -1.3401)
        RadiusArc((2.1926, -8.3507), (1.2032, -8.4346), -1.7269)
        RadiusArc((1.2032, -8.4346), (0.4437, -9.3658), -1.4017)
        RadiusArc((0.4437, -9.3658), (0.302, -10.4164), -3.8428)
        Line((0.302, -10.4164), (0.302, -11.039))
        Line((0.302, -11.039), (-0.6501, -11.039))
        Line((-0.6501, -11.039), (-0.6501, -10.4408))
        Line((-0.6501, -10.4408), (-0.6519, -10.3222))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 15 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((-4.1279, -4.8646), (-3.6959, -5.3697), -2.2558)
        RadiusArc((-3.6959, -5.3697), (-2.5611, -5.9792), -3.0893)
        RadiusArc((-2.5611, -5.9792), (-0.3913, -6.2975), -6.9509)
        RadiusArc((-0.3913, -6.2975), (1.6954, -6.1385), -10.3735)
        RadiusArc((1.6954, -6.1385), (3.1949, -5.5561), -4.03)
        RadiusArc((3.1949, -5.5561), (4.1985, -4.2351), -2.3669)
        RadiusArc((4.1985, -4.2351), (4.3037, -2.9458), -3.3589)
        RadiusArc((4.3037, -2.9458), (3.8194, -1.6725), -2.489)
        RadiusArc((3.8194, -1.6725), (2.5108, -0.7344), -3.0025)
        RadiusArc((2.5108, -0.7344), (0.7201, -0.3562), -5.9834)
        RadiusArc((0.7201, -0.3562), (-1.6818, -0.4415), -10.9395)
        RadiusArc((-1.6818, -0.4415), (-3.6867, -1.256), -4.2762)
        RadiusArc((-3.6867, -1.256), (-4.3469, -2.2035), -2.2695)
        RadiusArc((-4.3469, -2.2035), (-4.506, -3.6935), -3.3769)
        RadiusArc((-4.506, -3.6935), (-4.1279, -4.8646), -2.658)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch4': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        RadiusArc((-1.6149, -1.5363), (-3.1849, -2.142), -3.2762)
        RadiusArc((-3.1849, -2.142), (-3.5429, -3.9038), -1.6878)
        RadiusArc((-3.5429, -3.9038), (-2.696, -4.8045), -1.5488)
        RadiusArc((-2.696, -4.8045), (-1.4396, -5.1191), -3.8593)
        RadiusArc((-1.4396, -5.1191), (0.1389, -5.1879), -11.1712)
        RadiusArc((0.1389, -5.1879), (2.0561, -4.9487), -7.9984)
        RadiusArc((2.0561, -4.9487), (3.0497, -4.3836), -2.3094)
        RadiusArc((3.0497, -4.3836), (3.4268, -3.3163), -1.5082)
        RadiusArc((3.4268, -3.3163), (2.5602, -1.8942), -1.5045)
        RadiusArc((2.5602, -1.8942), (0.5798, -1.4606), -4.9916)
        RadiusArc((0.5798, -1.4606), (-1.6149, -1.5363), -11.6106)
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

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
    _bore_ax = _gAx2(_gPnt(0.0004, -14.9999, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6254, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50000012 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50000012 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
