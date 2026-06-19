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

# 'Sketch3': 18 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch3:
    with BuildLine():
        Line((0.687, 8.9733), (0.6, 9.0034))
        Line((0.6, 9.0034), (0.6, 8.9))
        Line((0.6, 8.9), (-9.0, 8.9))
        Line((-9.0, 8.9), (-9.0, 5.5))
        RadiusArc((-9.0, 5.5), (-6.247, 2.5196), -2.9897)
        Line((-6.247, 2.5196), (-6.0, 2.5103))
        Line((-6.0, 2.5103), (-6.0, -17.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -17.5), (-0.0, -23.5), -6.0)
        RadiusArc((-0.0, -23.5), (6.0, -17.5), -6.0)
        Line((6.0, -17.5), (6.0, 2.5103))
        Line((6.0, 2.5103), (6.247, 2.5196))
        RadiusArc((6.247, 2.5196), (9.0, 5.5), -2.9898)
        Line((9.0, 5.5), (9.0, 17.5))
        Line((9.0, 17.5), (-9.0, 17.5))
        Line((-9.0, 17.5), (-9.0, 14.1))
        Line((-9.0, 14.1), (0.6, 14.1))
        Line((0.6, 14.1), (0.6, 13.9966))
        Line((0.6, 13.9966), (0.687, 14.0267))
        # Arc split: sweep=207.27deg >= 150 — emitted as two half-arcs
        RadiusArc((0.687, 14.0267), (3.9, 11.5), 2.6)
        RadiusArc((3.9, 11.5), (0.687, 8.9733), 2.6)
    _inc_edges_sk_Sketch3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3 = Wire.combine(_inc_edges_sk_Sketch3)[0]
_wire_sk_Sketch3 = _wire_sk_Sketch3.moved(_inclined_plane_1.location)
_mkf_sk_Sketch3 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch3.wrapped, True)
_face_sk_Sketch3 = Face(_mkf_sk_Sketch3.Face())

# 'Sketch4': 22 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch4_2:
    with BuildLine():
        RadiusArc((2.2793, -6.2881), (3.0002, -6.0456), -2.734)
        RadiusArc((3.0002, -6.0456), (3.8177, -5.3645), -2.1963)
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((3.8177, -5.3645), (4.0493, -4.8855), (4.2142, -4.3797), (4.3096, -3.8563), (4.3336, -3.3248), (4.2859, -2.7949), (4.1673, -2.2762), (3.9799, -1.7783))
        RadiusArc((3.9799, -1.7783), (2.6908, -0.5906), -2.5063)
        RadiusArc((2.6908, -0.5906), (0.0138, -0.6995), -3.3315)
        RadiusArc((0.0138, -0.6995), (-1.1156, -2.0692), -2.4643)
        RadiusArc((-1.1156, -2.0692), (-0.7539, -4.9142), -3.2303)
        Line((-0.7539, -4.9142), (-3.4637, -4.7505))
        Line((-3.4637, -4.7505), (-3.4637, -0.8628))
        Line((-3.4637, -0.8628), (-4.3974, -0.8628))
        Line((-4.3974, -0.8628), (-4.3974, -5.7514))
        Line((-4.3974, -5.7514), (0.2287, -6.0382))
        Line((0.2287, -6.0382), (0.2287, -4.9639))
        # Near-straight arc (sagitta=0.008488mm) replaced with Line
        Line((0.2287, -4.9639), (-0.0421, -4.6053))
        RadiusArc((-0.0421, -4.6053), (-0.3341, -3.8904), 2.2027)
        RadiusArc((-0.3341, -3.8904), (-0.3523, -2.9344), 2.4978)
        # Near-straight arc (sagitta=0.005724mm) replaced with Line
        Line((-0.3523, -2.9344), (-0.2787, -2.6438))
        RadiusArc((-0.2787, -2.6438), (0.735, -1.613), 1.6415)
        RadiusArc((0.735, -1.613), (2.748, -1.8717), 2.3951)
        RadiusArc((2.748, -1.8717), (3.3105, -2.6473), 1.5995)
        # Spline from NurbsCurve3D, 14 adaptive samples
        Spline((3.3105, -2.6473), (3.3723, -2.8634), (3.4097, -3.085), (3.4235, -3.534), (3.4036, -3.758), (3.363, -3.9791), (3.2981, -4.1943), (3.2058, -4.3992), (3.0846, -4.5884), (2.9355, -4.7565), (2.7622, -4.8996), (2.5702, -5.0165), (2.365, -5.1083), (2.1512, -5.1777))
        Line((2.1512, -5.1777), (2.2793, -6.2881))
    _inc_edges_sk_Sketch4_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_2 = Wire.combine(_inc_edges_sk_Sketch4_2)[0]
_wire_sk_Sketch4_2 = _wire_sk_Sketch4_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch4_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch4_2.wrapped, True)
_face_sk_Sketch4_2 = Face(_mkf_sk_Sketch4_2.Face())

# 'Sketch4': 14 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        # Spline from NurbsCurve3D, 28 adaptive samples
        Spline((1.9926, -13.1019), (2.3134, -13.0465), (2.6271, -12.9594), (2.9285, -12.8362), (3.2107, -12.6741), (3.4664, -12.4728), (3.6893, -12.2355), (3.8762, -11.9691), (4.0273, -11.6807), (4.1436, -11.3766), (4.2278, -11.0621), (4.2835, -10.7412), (4.3144, -10.417), (4.3145, -9.7659), (4.2844, -9.4417), (4.2298, -9.1206), (4.1466, -8.8058), (4.0304, -8.5017), (3.8776, -8.2143), (3.6865, -7.9508), (3.4584, -7.7186), (3.1969, -7.525), (2.908, -7.3751), (2.6001, -7.2698), (2.2809, -7.2058), (1.9564, -7.178), (1.6309, -7.1837), (1.3097, -7.2354))
        RadiusArc((1.3097, -7.2354), (0.5298, -7.639), -1.7845)
        RadiusArc((0.5298, -7.639), (-0.1787, -9.2141), -2.3591)
        Line((-0.1787, -9.2141), (-0.2106, -9.2141))
        # Spline from NurbsCurve3D, 34 adaptive samples
        Spline((-0.2106, -9.2141), (-0.3116, -8.8347), (-0.4606, -8.4717), (-0.667, -8.1381), (-0.7935, -7.9879), (-0.9352, -7.852), (-1.0912, -7.7328), (-1.2599, -7.6325), (-1.4389, -7.5519), (-1.6256, -7.4911), (-2.0121, -7.4238), (-2.4046, -7.4175), (-2.794, -7.4658), (-3.1699, -7.5783), (-3.3479, -7.6611), (-3.5164, -7.7619), (-3.6735, -7.8797), (-3.8177, -8.0129), (-4.0645, -8.3179), (-4.2534, -8.6618), (-4.3866, -9.031), (-4.471, -9.4144), (-4.5148, -9.8046), (-4.5253, -10.1972), (-4.5029, -10.5892), (-4.4407, -10.9769), (-4.3324, -11.3542), (-4.1731, -11.7129), (-3.9614, -12.0434), (-3.6998, -12.3358), (-3.3911, -12.578), (-3.0444, -12.7616), (-2.6721, -12.8857), (-2.2858, -12.9553))
        Line((-2.2858, -12.9553), (-2.2002, -11.8504))
        RadiusArc((-2.2002, -11.8504), (-2.6934, -11.7318), 1.8513)
        RadiusArc((-2.6934, -11.7318), (-3.298, -11.2678), 1.3801)
        # Spline from NurbsCurve3D, 27 adaptive samples
        Spline((-3.298, -11.2678), (-3.4296, -11.0555), (-3.5249, -10.8245), (-3.5857, -10.5822), (-3.6163, -10.3341), (-3.6216, -10.0842), (-3.6046, -9.8348), (-3.5612, -9.5887), (-3.4846, -9.3509), (-3.3685, -9.1298), (-3.2105, -8.9366), (-3.0168, -8.7791), (-2.7961, -8.6623), (-2.5578, -8.5873), (-2.3108, -8.5499), (-2.0609, -8.5442), (-1.8127, -8.5722), (-1.5728, -8.6417), (-1.351, -8.7564), (-1.157, -8.9136), (-0.9958, -9.1043), (-0.869, -9.3195), (-0.7753, -9.5512), (-0.7111, -9.7927), (-0.6717, -10.0396), (-0.6506, -10.5389), (-0.6501, -11.039))
        Line((-0.6501, -11.039), (0.302, -11.039))
        # Spline from NurbsCurve3D, 8 adaptive samples
        Spline((0.302, -11.039), (0.3007, -10.6185), (0.3068, -10.2383), (0.3333, -9.8837), (0.3933, -9.5424), (0.4992, -9.2144), (0.6442, -8.9347), (0.8062, -8.7228))
        RadiusArc((0.8062, -8.7228), (1.4387, -8.3554), 1.2234)
        # Spline from NurbsCurve3D, 24 adaptive samples
        Spline((1.4387, -8.3554), (1.6427, -8.3233), (1.8492, -8.3176), (2.2593, -8.364), (2.458, -8.4205), (2.6469, -8.5038), (2.8206, -8.6154), (2.9735, -8.7542), (3.1026, -8.9154), (3.2078, -9.0931), (3.2891, -9.283), (3.3479, -9.481), (3.408, -9.8894), (3.4081, -10.3025), (3.3503, -10.7114), (3.2937, -10.9102), (3.2152, -11.1013), (3.1126, -11.2806), (2.9857, -11.4436), (2.8361, -11.586), (2.6674, -11.7054), (2.4844, -11.8014), (2.2915, -11.8752), (1.8888, -11.9666))
        Line((1.8888, -11.9666), (1.9926, -13.1019))
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch3
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0, -17.5, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.625, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.00 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3_p0 --
    _face = _face_sk_Sketch4_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50 mm
    
    # -- Extrude3_p1 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.50 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
