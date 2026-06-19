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
        Line((0.6, 26.4), (-9.0, 26.4))
        Line((-9.0, 26.4), (-9.0, 23.0))
        RadiusArc((-9.0, 23.0), (-6.0, 20.0102), -2.9819)
        Line((-6.0, 20.0102), (-6.0, -35.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -35.0), (0.0, -41.0), -6.0)
        RadiusArc((0.0, -41.0), (6.0, -35.0), -6.0)
        Line((6.0, -35.0), (6.0, 20.0102))
        RadiusArc((6.0, 20.0102), (9.0, 23.0), -2.9779)
        Line((9.0, 23.0), (9.0, 35.0))
        Line((9.0, 35.0), (-9.0, 35.0))
        Line((-9.0, 35.0), (-9.0, 31.6))
        Line((-9.0, 31.6), (0.6, 31.6))
        Line((0.6, 31.6), (0.6, 31.4966))
        # Arc split: sweep=211.59deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 31.4966), (3.9006, 29.0), 2.5945)
        RadiusArc((3.9006, 29.0), (0.6, 26.5034), 2.5945)
        Line((0.6, 26.5034), (0.6, 26.4))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 51 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((3.729, -10.192), (3.8859, -10.198))
        Line((3.8859, -10.198), (4.0436, -10.2017))
        Line((4.0436, -10.2017), (4.2019, -10.2029))
        Line((4.2019, -10.2029), (4.2019, -11.3502))
        RadiusArc((4.2019, -11.3502), (-3.4637, -8.317), 13.679)
        Line((-3.4637, -8.317), (-3.4637, -12.9371))
        Line((-3.4637, -12.9371), (-4.3974, -12.9371))
        Line((-4.3974, -12.9371), (-4.3974, -7.255))
        Line((-4.3974, -7.255), (-3.5064, -7.255))
        Line((-3.5064, -7.255), (-3.2417, -7.4273))
        Line((-3.2417, -7.4273), (-2.9848, -7.5928))
        Line((-2.9848, -7.5928), (-2.7356, -7.7513))
        Line((-2.7356, -7.7513), (-2.4943, -7.903))
        Line((-2.4943, -7.903), (-2.2606, -8.0478))
        Line((-2.2606, -8.0478), (-2.0348, -8.1856))
        Line((-2.0348, -8.1856), (-1.8167, -8.3166))
        Line((-1.8167, -8.3166), (-1.6063, -8.4407))
        Line((-1.6063, -8.4407), (-1.4037, -8.5579))
        Line((-1.4037, -8.5579), (-1.2089, -8.6682))
        Line((-1.2089, -8.6682), (-1.0218, -8.7717))
        Line((-1.0218, -8.7717), (-0.8425, -8.8682))
        Line((-0.8425, -8.8682), (-0.6709, -8.9579))
        Line((-0.6709, -8.9579), (-0.5071, -9.0406))
        Line((-0.5071, -9.0406), (-0.351, -9.1165))
        Line((-0.351, -9.1165), (-0.199, -9.1877))
        Line((-0.199, -9.1877), (-0.0472, -9.2565))
        Line((-0.0472, -9.2565), (0.1042, -9.3229))
        Line((0.1042, -9.3229), (0.2555, -9.3868))
        Line((0.2555, -9.3868), (0.4064, -9.4484))
        Line((0.4064, -9.4484), (0.5571, -9.5076))
        Line((0.5571, -9.5076), (0.7075, -9.5643))
        Line((0.7075, -9.5643), (0.8577, -9.6186))
        Line((0.8577, -9.6186), (1.0075, -9.6705))
        Line((1.0075, -9.6705), (1.1571, -9.72))
        Line((1.1571, -9.72), (1.3064, -9.7671))
        Line((1.3064, -9.7671), (1.4555, -9.8117))
        Line((1.4555, -9.8117), (1.6043, -9.854))
        Line((1.6043, -9.854), (1.7528, -9.8938))
        Line((1.7528, -9.8938), (1.901, -9.9312))
        Line((1.901, -9.9312), (2.0495, -9.9663))
        Line((2.0495, -9.9663), (2.1986, -9.9988))
        Line((2.1986, -9.9988), (2.3485, -10.029))
        Line((2.3485, -10.029), (2.4991, -10.0568))
        Line((2.4991, -10.0568), (2.6503, -10.0821))
        Line((2.6503, -10.0821), (2.8023, -10.1051))
        Line((2.8023, -10.1051), (2.955, -10.1256))
        Line((2.955, -10.1256), (3.1084, -10.1437))
        Line((3.1084, -10.1437), (3.2625, -10.1594))
        Line((3.2625, -10.1594), (3.4173, -10.1727))
        Line((3.4173, -10.1727), (3.5728, -10.1835))
        Line((3.5728, -10.1835), (3.729, -10.192))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch4': 8 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        RadiusArc((-2.148, -6.095), (0.9809, -6.2477), -10.4124)
        RadiusArc((0.9809, -6.2477), (3.0397, -5.6521), -5.0093)
        RadiusArc((3.0397, -5.6521), (4.2788, -3.8921), -2.413)
        RadiusArc((4.2788, -3.8921), (3.9153, -1.8092), -2.9175)
        RadiusArc((3.9153, -1.8092), (2.5108, -0.7344), -3.0855)
        RadiusArc((2.5108, -0.7344), (1.2222, -0.4102), -5.9793)
        RadiusArc((1.2222, -0.4102), (-2.1324, -0.5315), -9.9992)
        # Arc split: sweep=166.96deg >= 150 — emitted as two half-arcs
        RadiusArc((-2.1324, -0.5315), (-4.5658, -3.3568), -2.7936)
        RadiusArc((-4.5658, -3.3568), (-2.148, -6.095), -2.7936)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch5': 20 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch5_4:
    with BuildLine():
        RadiusArc((2.0561, -4.9487), (2.7896, -4.6123), -2.722)
        RadiusArc((2.7896, -4.6123), (3.1855, -4.2077), -1.5334)
        RadiusArc((3.1855, -4.2077), (3.4117, -3.5673), -1.457)
        RadiusArc((3.4117, -3.5673), (3.3652, -2.8356), -1.9784)
        RadiusArc((3.3652, -2.8356), (3.0416, -2.2529), -1.4111)
        RadiusArc((3.0416, -2.2529), (2.5602, -1.8942), -1.6814)
        RadiusArc((2.5602, -1.8942), (1.8793, -1.6433), -2.9847)
        RadiusArc((1.8793, -1.6433), (0.9923, -1.4928), -5.684)
        RadiusArc((0.9923, -1.4928), (-0.1008, -1.4426), -10.3133)
        RadiusArc((-0.1008, -1.4426), (-1.426, -1.5114), -12.2877)
        RadiusArc((-1.426, -1.5114), (-2.423, -1.718), -5.6417)
        RadiusArc((-2.423, -1.718), (-3.0994, -2.0673), -2.1002)
        RadiusArc((-3.0994, -2.0673), (-3.5007, -2.5902), -1.3735)
        RadiusArc((-3.5007, -2.5902), (-3.6345, -3.2918), -1.6477)
        RadiusArc((-3.6345, -3.2918), (-3.2683, -4.389), -1.6138)
        RadiusArc((-3.2683, -4.389), (-2.572, -4.8576), -1.6424)
        RadiusArc((-2.572, -4.8576), (-1.6297, -5.0936), -4.0674)
        RadiusArc((-1.6297, -5.0936), (-0.3476, -5.1879), -9.0962)
        # Near-straight arc (sagitta=0.007961mm) replaced with Line
        Line((-0.3476, -5.1879), (0.5922, -5.1719))
        RadiusArc((0.5922, -5.1719), (2.0561, -4.9487), -7.3685)
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
    _bore_ax = _gAx2(_gPnt(0.0014, -34.9994, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6265, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * 2.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.500000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch5_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
