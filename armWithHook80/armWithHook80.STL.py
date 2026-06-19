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

# 'Sketch3': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch3:
    with BuildLine():
        Line((0.6, 31.4), (-9.0, 31.4))
        Line((-9.0, 31.4), (-9.0, 28.0))
        RadiusArc((-9.0, 28.0), (-6.0, 25.0102), -2.9819)
        Line((-6.0, 25.0102), (-6.0, -40.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -40.0), (0.0, -46.0), -6.0)
        RadiusArc((0.0, -46.0), (6.0, -40.0), -6.0)
        Line((6.0, -40.0), (6.0, 25.0102))
        RadiusArc((6.0, 25.0102), (9.0, 28.0), -2.9846)
        Line((9.0, 28.0), (9.0, 40.0))
        Line((9.0, 40.0), (-9.0, 40.0))
        Line((-9.0, 40.0), (-9.0, 36.6))
        Line((-9.0, 36.6), (0.6, 36.6))
        Line((0.6, 36.6), (0.6, 36.4966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 36.4966), (3.9, 34.0), 2.5944)
        RadiusArc((3.9, 34.0), (0.6, 31.5034), 2.5944)
        Line((0.6, 31.5034), (0.6, 31.4))
    _inc_edges_sk_Sketch3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3 = Wire.combine(_inc_edges_sk_Sketch3)[0]
_wire_sk_Sketch3 = _wire_sk_Sketch3.moved(_inclined_plane_1.location)
_mkf_sk_Sketch3 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch3.wrapped, True)
_face_sk_Sketch3 = Face(_mkf_sk_Sketch3.Face())

# 'Sketch4': 242 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch4_2:
    with BuildLine():
        Line((4.2973, -9.5591), (4.3121, -9.7332))
        Line((4.3121, -9.7332), (4.321, -9.9132))
        Line((4.321, -9.9132), (4.3239, -10.0991))
        Line((4.3239, -10.0991), (4.321, -10.2803))
        Line((4.321, -10.2803), (4.3123, -10.4562))
        Line((4.3123, -10.4562), (4.2978, -10.6268))
        Line((4.2978, -10.6268), (4.2775, -10.792))
        Line((4.2775, -10.792), (4.2513, -10.9518))
        Line((4.2513, -10.9518), (4.2194, -11.1063))
        Line((4.2194, -11.1063), (4.1817, -11.2555))
        Line((4.1817, -11.2555), (4.1382, -11.3993))
        Line((4.1382, -11.3993), (4.0888, -11.5378))
        Line((4.0888, -11.5378), (4.0337, -11.671))
        Line((4.0337, -11.671), (3.9727, -11.7988))
        Line((3.9727, -11.7988), (3.906, -11.9212))
        Line((3.906, -11.9212), (3.8334, -12.0384))
        Line((3.8334, -12.0384), (3.755, -12.1501))
        Line((3.755, -12.1501), (3.6709, -12.2566))
        Line((3.6709, -12.2566), (3.5814, -12.3569))
        Line((3.5814, -12.3569), (3.487, -12.4502))
        Line((3.487, -12.4502), (3.3877, -12.5367))
        Line((3.3877, -12.5367), (3.2835, -12.6162))
        Line((3.2835, -12.6162), (3.1745, -12.6889))
        Line((3.1745, -12.6889), (3.0606, -12.7546))
        Line((3.0606, -12.7546), (2.9417, -12.8134))
        Line((2.9417, -12.8134), (2.8181, -12.8652))
        Line((2.8181, -12.8652), (2.6895, -12.9102))
        Line((2.6895, -12.9102), (2.556, -12.9483))
        Line((2.556, -12.9483), (2.4177, -12.9794))
        Line((2.4177, -12.9794), (2.2745, -13.0036))
        Line((2.2745, -13.0036), (2.1264, -13.0209))
        Line((2.1264, -13.0209), (1.9734, -13.0312))
        Line((1.9734, -13.0312), (1.8155, -13.0347))
        Line((1.8155, -13.0347), (1.7044, -13.0326))
        Line((1.7044, -13.0326), (1.5957, -13.0261))
        Line((1.5957, -13.0261), (1.4894, -13.0154))
        Line((1.4894, -13.0154), (1.3854, -13.0004))
        Line((1.3854, -13.0004), (1.2839, -12.9811))
        Line((1.2839, -12.9811), (1.1847, -12.9576))
        Line((1.1847, -12.9576), (1.0879, -12.9297))
        Line((1.0879, -12.9297), (0.9935, -12.8976))
        Line((0.9935, -12.8976), (0.9015, -12.8611))
        Line((0.9015, -12.8611), (0.8119, -12.8204))
        Line((0.8119, -12.8204), (0.7247, -12.7754))
        Line((0.7247, -12.7754), (0.6398, -12.7262))
        Line((0.6398, -12.7262), (0.5574, -12.6726))
        Line((0.5574, -12.6726), (0.4773, -12.6147))
        Line((0.4773, -12.6147), (0.3996, -12.5526))
        Line((0.3996, -12.5526), (0.3252, -12.4871))
        Line((0.3252, -12.4871), (0.2547, -12.4192))
        Line((0.2547, -12.4192), (0.1882, -12.349))
        Line((0.1882, -12.349), (0.1258, -12.2763))
        Line((0.1258, -12.2763), (0.0673, -12.2013))
        Line((0.0673, -12.2013), (0.0129, -12.1239))
        Line((0.0129, -12.1239), (-0.0375, -12.0441))
        Line((-0.0375, -12.0441), (-0.0839, -11.9619))
        Line((-0.0839, -11.9619), (-0.1262, -11.8773))
        Line((-0.1262, -11.8773), (-0.1646, -11.7903))
        Line((-0.1646, -11.7903), (-0.1989, -11.7009))
        Line((-0.1989, -11.7009), (-0.2292, -11.6092))
        Line((-0.2292, -11.6092), (-0.2556, -11.5151))
        Line((-0.2556, -11.5151), (-0.2779, -11.4186))
        Line((-0.2779, -11.4186), (-0.2961, -11.3197))
        Line((-0.2961, -11.3197), (-0.3205, -11.3197))
        Line((-0.3205, -11.3197), (-0.3442, -11.4119))
        Line((-0.3442, -11.4119), (-0.3713, -11.5016))
        Line((-0.3713, -11.5016), (-0.4018, -11.5886))
        Line((-0.4018, -11.5886), (-0.4358, -11.673))
        Line((-0.4358, -11.673), (-0.4731, -11.7547))
        Line((-0.4731, -11.7547), (-0.5139, -11.8338))
        Line((-0.5139, -11.8338), (-0.5581, -11.9103))
        Line((-0.5581, -11.9103), (-0.6057, -11.9842))
        Line((-0.6057, -11.9842), (-0.6567, -12.0554))
        Line((-0.6567, -12.0554), (-0.7111, -12.124))
        Line((-0.7111, -12.124), (-0.769, -12.19))
        Line((-0.769, -12.19), (-0.8303, -12.2533))
        Line((-0.8303, -12.2533), (-0.8949, -12.314))
        Line((-0.8949, -12.314), (-0.9631, -12.3721))
        Line((-0.9631, -12.3721), (-1.0346, -12.4275))
        Line((-1.0346, -12.4275), (-1.1087, -12.4798))
        Line((-1.1087, -12.4798), (-1.1844, -12.5285))
        Line((-1.1844, -12.5285), (-1.2619, -12.5736))
        Line((-1.2619, -12.5736), (-1.341, -12.6151))
        Line((-1.341, -12.6151), (-1.4218, -12.6529))
        Line((-1.4218, -12.6529), (-1.5043, -12.6872))
        Line((-1.5043, -12.6872), (-1.5885, -12.7179))
        Line((-1.5885, -12.7179), (-1.6743, -12.7449))
        Line((-1.6743, -12.7449), (-1.7619, -12.7684))
        Line((-1.7619, -12.7684), (-1.8511, -12.7882))
        Line((-1.8511, -12.7882), (-1.942, -12.8044))
        Line((-1.942, -12.8044), (-2.0346, -12.8171))
        Line((-2.0346, -12.8171), (-2.1288, -12.8261))
        Line((-2.1288, -12.8261), (-2.2248, -12.8315))
        Line((-2.2248, -12.8315), (-2.3224, -12.8333))
        Line((-2.3224, -12.8333), (-2.4518, -12.83))
        Line((-2.4518, -12.83), (-2.5778, -12.8202))
        Line((-2.5778, -12.8202), (-2.7005, -12.8039))
        Line((-2.7005, -12.8039), (-2.82, -12.781))
        Line((-2.82, -12.781), (-2.936, -12.7516))
        Line((-2.936, -12.7516), (-3.0489, -12.7156))
        Line((-3.0489, -12.7156), (-3.1584, -12.6731))
        Line((-3.1584, -12.6731), (-3.2645, -12.6241))
        Line((-3.2645, -12.6241), (-3.3674, -12.5686))
        Line((-3.3674, -12.5686), (-3.467, -12.5065))
        Line((-3.467, -12.5065), (-3.5633, -12.4378))
        Line((-3.5633, -12.4378), (-3.6562, -12.3626))
        Line((-3.6562, -12.3626), (-3.7459, -12.2809))
        Line((-3.7459, -12.2809), (-3.8322, -12.1927))
        Line((-3.8322, -12.1927), (-3.9152, -12.0979))
        Line((-3.9152, -12.0979), (-3.9939, -11.9976))
        Line((-3.9939, -11.9976), (-4.0671, -11.8928))
        Line((-4.0671, -11.8928), (-4.135, -11.7836))
        Line((-4.135, -11.7836), (-4.1973, -11.6698))
        Line((-4.1973, -11.6698), (-4.2543, -11.5517))
        Line((-4.2543, -11.5517), (-4.3059, -11.429))
        Line((-4.3059, -11.429), (-4.352, -11.3018))
        Line((-4.352, -11.3018), (-4.3927, -11.1702))
        Line((-4.3927, -11.1702), (-4.4279, -11.0341))
        Line((-4.4279, -11.0341), (-4.4578, -10.8935))
        Line((-4.4578, -10.8935), (-4.4822, -10.7484))
        Line((-4.4822, -10.7484), (-4.5012, -10.5989))
        Line((-4.5012, -10.5989), (-4.5147, -10.4449))
        Line((-4.5147, -10.4449), (-4.5229, -10.2864))
        Line((-4.5229, -10.2864), (-4.5256, -10.1235))
        Line((-4.5256, -10.1235), (-4.5229, -9.9566))
        Line((-4.5229, -9.9566), (-4.515, -9.7945))
        Line((-4.515, -9.7945), (-4.5017, -9.6371))
        Line((-4.5017, -9.6371), (-4.4831, -9.4844))
        Line((-4.4831, -9.4844), (-4.4591, -9.3365))
        Line((-4.4591, -9.3365), (-4.4299, -9.1934))
        Line((-4.4299, -9.1934), (-4.3953, -9.055))
        Line((-4.3953, -9.055), (-4.3555, -8.9213))
        Line((-4.3555, -8.9213), (-4.3103, -8.7924))
        Line((-4.3103, -8.7924), (-4.2597, -8.6682))
        Line((-4.2597, -8.6682), (-4.2039, -8.5488))
        Line((-4.2039, -8.5488), (-4.1428, -8.4342))
        Line((-4.1428, -8.4342), (-4.0763, -8.3243))
        Line((-4.0763, -8.3243), (-4.0045, -8.2191))
        Line((-4.0045, -8.2191), (-3.9275, -8.1186))
        Line((-3.9275, -8.1186), (-3.8458, -8.0239))
        Line((-3.8458, -8.0239), (-3.7605, -7.9356))
        Line((-3.7605, -7.9356), (-3.6714, -7.8539))
        Line((-3.6714, -7.8539), (-3.5786, -7.7787))
        Line((-3.5786, -7.7787), (-3.482, -7.71))
        Line((-3.482, -7.71), (-3.3816, -7.6479))
        Line((-3.3816, -7.6479), (-3.2775, -7.5924))
        Line((-3.2775, -7.5924), (-3.1697, -7.5433))
        Line((-3.1697, -7.5433), (-3.0582, -7.5008))
        Line((-3.0582, -7.5008), (-2.9429, -7.4649))
        Line((-2.9429, -7.4649), (-2.8238, -7.4355))
        Line((-2.8238, -7.4355), (-2.701, -7.4126))
        Line((-2.701, -7.4126), (-2.5745, -7.3962))
        Line((-2.5745, -7.3962), (-2.4442, -7.3864))
        Line((-2.4442, -7.3864), (-2.3101, -7.3832))
        Line((-2.3101, -7.3832), (-2.2125, -7.385))
        Line((-2.2125, -7.385), (-2.1166, -7.3904))
        Line((-2.1166, -7.3904), (-2.0223, -7.3995))
        Line((-2.0223, -7.3995), (-1.9298, -7.4123))
        Line((-1.9298, -7.4123), (-1.8389, -7.4286))
        Line((-1.8389, -7.4286), (-1.7497, -7.4486))
        Line((-1.7497, -7.4486), (-1.6621, -7.4722))
        Line((-1.6621, -7.4722), (-1.5763, -7.4995))
        Line((-1.5763, -7.4995), (-1.4921, -7.5304))
        Line((-1.4921, -7.5304), (-1.4096, -7.5649))
        Line((-1.4096, -7.5649), (-1.3288, -7.6031))
        Line((-1.3288, -7.6031), (-1.2497, -7.6449))
        Line((-1.2497, -7.6449), (-1.1722, -7.6903))
        Line((-1.1722, -7.6903), (-1.0964, -7.7394))
        Line((-1.0964, -7.7394), (-1.0223, -7.7921))
        Line((-1.0223, -7.7921), (-0.9509, -7.848))
        Line((-0.9509, -7.848), (-0.8831, -7.9065))
        Line((-0.8831, -7.9065), (-0.819, -7.9676))
        Line((-0.819, -7.9676), (-0.7585, -8.0315))
        Line((-0.7585, -8.0315), (-0.7016, -8.0979))
        Line((-0.7016, -8.0979), (-0.6484, -8.1671))
        Line((-0.6484, -8.1671), (-0.5988, -8.2389))
        Line((-0.5988, -8.2389), (-0.5528, -8.3133))
        Line((-0.5528, -8.3133), (-0.5105, -8.3904))
        Line((-0.5105, -8.3904), (-0.4717, -8.4702))
        Line((-0.4717, -8.4702), (-0.4367, -8.5526))
        Line((-0.4367, -8.5526), (-0.4052, -8.6377))
        Line((-0.4052, -8.6377), (-0.3775, -8.7255))
        Line((-0.3775, -8.7255), (-0.3533, -8.8159))
        Line((-0.3533, -8.8159), (-0.3328, -8.9089))
        Line((-0.3328, -8.9089), (-0.3083, -8.9089))
        Line((-0.3083, -8.9089), (-0.2885, -8.8007))
        Line((-0.2885, -8.8007), (-0.2648, -8.6957))
        Line((-0.2648, -8.6957), (-0.2374, -8.594))
        Line((-0.2374, -8.594), (-0.2061, -8.4956))
        Line((-0.2061, -8.4956), (-0.171, -8.4004))
        Line((-0.171, -8.4004), (-0.1321, -8.3084))
        Line((-0.1321, -8.3084), (-0.0893, -8.2197))
        Line((-0.0893, -8.2197), (-0.0427, -8.1343))
        Line((-0.0427, -8.1343), (0.0077, -8.0521))
        Line((0.0077, -8.0521), (0.0619, -7.9732))
        Line((0.0619, -7.9732), (0.12, -7.8975))
        Line((0.12, -7.8975), (0.1818, -7.8251))
        Line((0.1818, -7.8251), (0.2476, -7.7559))
        Line((0.2476, -7.7559), (0.3171, -7.69))
        Line((0.3171, -7.69), (0.3905, -7.6273))
        Line((0.3905, -7.6273), (0.467, -7.5683))
        Line((0.467, -7.5683), (0.5461, -7.5134))
        Line((0.5461, -7.5134), (0.6276, -7.4625))
        Line((0.6276, -7.4625), (0.7117, -7.4158))
        Line((0.7117, -7.4158), (0.7984, -7.373))
        Line((0.7984, -7.373), (0.8875, -7.3344))
        Line((0.8875, -7.3344), (0.9792, -7.2998))
        Line((0.9792, -7.2998), (1.0734, -7.2693))
        Line((1.0734, -7.2693), (1.1701, -7.2428))
        Line((1.1701, -7.2428), (1.2693, -7.2205))
        Line((1.2693, -7.2205), (1.3711, -7.2022))
        Line((1.3711, -7.2022), (1.4754, -7.1879))
        Line((1.4754, -7.1879), (1.5822, -7.1777))
        Line((1.5822, -7.1777), (1.6915, -7.1716))
        Line((1.6915, -7.1716), (1.8034, -7.1696))
        Line((1.8034, -7.1696), (1.9597, -7.173))
        Line((1.9597, -7.173), (2.1114, -7.183))
        Line((2.1114, -7.183), (2.2584, -7.1999))
        Line((2.2584, -7.1999), (2.4008, -7.2234))
        Line((2.4008, -7.2234), (2.5384, -7.2537))
        Line((2.5384, -7.2537), (2.6715, -7.2907))
        Line((2.6715, -7.2907), (2.7998, -7.3344))
        Line((2.7998, -7.3344), (2.9235, -7.3849))
        Line((2.9235, -7.3849), (3.0425, -7.442))
        Line((3.0425, -7.442), (3.1569, -7.506))
        Line((3.1569, -7.506), (3.2666, -7.5766))
        Line((3.2666, -7.5766), (3.3716, -7.654))
        Line((3.3716, -7.654), (3.472, -7.738))
        Line((3.472, -7.738), (3.5677, -7.8289))
        Line((3.5677, -7.8289), (3.6587, -7.9264))
        Line((3.6587, -7.9264), (3.7445, -8.0302))
        Line((3.7445, -8.0302), (3.8243, -8.1399))
        Line((3.8243, -8.1399), (3.8982, -8.2555))
        Line((3.8982, -8.2555), (3.9662, -8.3769))
        Line((3.9662, -8.3769), (4.0283, -8.5041))
        Line((4.0283, -8.5041), (4.0844, -8.6373))
        Line((4.0844, -8.6373), (4.1347, -8.7762))
        Line((4.1347, -8.7762), (4.1791, -8.9211))
        Line((4.1791, -8.9211), (4.2175, -9.0718))
        Line((4.2175, -9.0718), (4.25, -9.2284))
        Line((4.25, -9.2284), (4.2766, -9.3908))
        Line((4.2766, -9.3908), (4.2973, -9.5591))
    _inc_edges_sk_Sketch4_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_2 = Wire.combine(_inc_edges_sk_Sketch4_2)[0]
_wire_sk_Sketch4_2 = _wire_sk_Sketch4_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch4_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch4_2.wrapped, True)
_face_sk_Sketch4_2 = Face(_mkf_sk_Sketch4_2.Face())

# 'Sketch4': 18 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        # Near-straight arc (sagitta=0.003434mm) replaced with Line
        Line((-0.3882, -0.329), (-0.9343, -0.3547))
        RadiusArc((-0.9343, -0.3547), (-2.5448, -0.6472), -7.4255)
        RadiusArc((-2.5448, -0.6472), (-3.6867, -1.256), -3.6215)
        RadiusArc((-3.6867, -1.256), (-4.4015, -2.3672), -2.2917)
        RadiusArc((-4.4015, -2.3672), (-4.285, -4.5673), -3.3497)
        RadiusArc((-4.285, -4.5673), (-3.4209, -5.5774), -2.291)
        RadiusArc((-3.4209, -5.5774), (-2.148, -6.095), -3.6863)
        RadiusArc((-2.148, -6.095), (-0.6719, -6.2878), -7.4805)
        RadiusArc((-0.6719, -6.2878), (1.2283, -6.2179), -9.9619)
        RadiusArc((1.2283, -6.2179), (2.8753, -5.7414), -4.6953)
        RadiusArc((2.8753, -5.7414), (4.078, -4.5518), -2.4259)
        RadiusArc((4.078, -4.5518), (4.3239, -3.3284), -3.0776)
        RadiusArc((4.3239, -3.3284), (4.1978, -2.4201), -2.9895)
        RadiusArc((4.1978, -2.4201), (3.7134, -1.5423), -2.3566)
        RadiusArc((3.7134, -1.5423), (3.0329, -0.9877), -2.8399)
        RadiusArc((3.0329, -0.9877), (2.1175, -0.5993), -4.2458)
        RadiusArc((2.1175, -0.5993), (0.9757, -0.3798), -6.8106)
        RadiusArc((0.9757, -0.3798), (-0.3882, -0.329), -11.1305)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch5': 22 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch5_4:
    with BuildLine():
        # Near-straight arc (sagitta=0.009688mm) replaced with Line
        Line((-1.2921, -11.4427), (-1.0731, -11.235))
        # Near-straight arc (sagitta=0.009143mm) replaced with Line
        Line((-1.0731, -11.235), (-0.9078, -10.9752))
        RadiusArc((-0.9078, -10.9752), (-0.7786, -10.5815), -1.5772)
        RadiusArc((-0.7786, -10.5815), (-0.7355, -10.1112), -2.3048)
        RadiusArc((-0.7355, -10.1112), (-0.7752, -9.6356), -2.7817)
        RadiusArc((-0.7752, -9.6356), (-0.9275, -9.173), -1.726)
        RadiusArc((-0.9275, -9.173), (-1.1421, -8.878), -1.2014)
        RadiusArc((-1.1421, -8.878), (-1.449, -8.6706), -1.0413)
        RadiusArc((-1.449, -8.6706), (-1.8542, -8.5533), -1.4938)
        # Near-straight arc (sagitta=0.008049mm) replaced with Line
        Line((-1.8542, -8.5533), (-2.2491, -8.5245))
        RadiusArc((-2.2491, -8.5245), (-2.9234, -8.6382), -1.9298)
        RadiusArc((-2.9234, -8.6382), (-3.3901, -8.9793), -1.1307)
        RadiusArc((-3.3901, -8.9793), (-3.604, -9.3844), -1.2716)
        RadiusArc((-3.604, -9.3844), (-3.7013, -9.9174), -2.0678)
        RadiusArc((-3.7013, -9.9174), (-3.6931, -10.4187), -3.0989)
        RadiusArc((-3.6931, -10.4187), (-3.5759, -10.9207), -2.001)
        RadiusArc((-3.5759, -10.9207), (-3.3888, -11.2487), -1.3246)
        RadiusArc((-3.3888, -11.2487), (-3.0575, -11.5299), -1.1052)
        RadiusArc((-3.0575, -11.5299), (-2.6109, -11.6814), -1.3348)
        RadiusArc((-2.6109, -11.6814), (-2.1522, -11.7084), -2.1411)
        RadiusArc((-2.1522, -11.7084), (-1.7145, -11.6434), -1.991)
        RadiusArc((-1.7145, -11.6434), (-1.2921, -11.4427), -1.353)
    _inc_edges_sk_Sketch5_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_4 = Wire.combine(_inc_edges_sk_Sketch5_4)[0]
_wire_sk_Sketch5_4 = _wire_sk_Sketch5_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch5_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch5_4.wrapped, True)
_face_sk_Sketch5_4 = Face(_mkf_sk_Sketch5_4.Face())

# 'Sketch5': 20 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with BuildLine():
        RadiusArc((0.2125, -10.9112), (0.4685, -11.3653), -1.5656)
        RadiusArc((0.4685, -11.3653), (0.8519, -11.6907), -1.4152)
        RadiusArc((0.8519, -11.6907), (1.2517, -11.8473), -1.4806)
        # Near-straight arc (sagitta=0.009261mm) replaced with Line
        Line((1.2517, -11.8473), (1.6237, -11.8975))
        RadiusArc((1.6237, -11.8975), (2.166, -11.8673), -2.9353)
        RadiusArc((2.166, -11.8673), (2.8607, -11.6095), -1.7701)
        RadiusArc((2.8607, -11.6095), (3.2159, -11.247), -1.2805)
        RadiusArc((3.2159, -11.247), (3.429, -10.7394), -1.5935)
        RadiusArc((3.429, -10.7394), (3.5001, -10.0869), -2.6597)
        RadiusArc((3.5001, -10.0869), (3.4522, -9.5396), -3.0494)
        RadiusArc((3.4522, -9.5396), (3.2687, -9.0172), -1.8276)
        RadiusArc((3.2687, -9.0172), (2.946, -8.641), -1.2501)
        RadiusArc((2.946, -8.641), (2.2696, -8.3597), -1.4651)
        RadiusArc((2.2696, -8.3597), (1.6996, -8.3109), -2.8967)
        RadiusArc((1.6996, -8.3109), (1.0391, -8.4146), -2.0907)
        RadiusArc((1.0391, -8.4146), (0.6094, -8.6685), -1.3182)
        RadiusArc((0.6094, -8.6685), (0.3066, -9.0682), -1.3513)
        RadiusArc((0.3066, -9.0682), (0.1172, -9.6919), -1.8551)
        RadiusArc((0.1172, -9.6919), (0.0903, -10.2333), -3.0289)
        RadiusArc((0.0903, -10.2333), (0.2125, -10.9112), -2.3598)
    _inc_edges_sk_Sketch5_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_5 = Wire.combine(_inc_edges_sk_Sketch5_5)[0]
_wire_sk_Sketch5_5 = _wire_sk_Sketch5_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch5_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch5_5.wrapped, True)
_face_sk_Sketch5_5 = Face(_mkf_sk_Sketch5_5.Face())

# 'Sketch5': 25 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch5_6:
    with BuildLine():
        RadiusArc((1.0106, -5.1401), (1.7425, -5.0284), -6.7924)
        RadiusArc((1.7425, -5.0284), (2.461, -4.7991), -3.7764)
        RadiusArc((2.461, -4.7991), (2.8839, -4.5408), -2.009)
        RadiusArc((2.8839, -4.5408), (3.242, -4.1127), -1.4582)
        RadiusArc((3.242, -4.1127), (3.3928, -3.6858), -1.463)
        RadiusArc((3.3928, -3.6858), (3.4229, -3.1892), -1.9459)
        RadiusArc((3.4229, -3.1892), (3.2881, -2.6229), -1.6549)
        RadiusArc((3.2881, -2.6229), (2.9607, -2.172), -1.3938)
        RadiusArc((2.9607, -2.172), (2.4405, -1.836), -1.8312)
        RadiusArc((2.4405, -1.836), (1.7184, -1.6051), -3.3978)
        RadiusArc((1.7184, -1.6051), (0.9923, -1.4928), -6.189)
        # Near-straight arc (sagitta=0.00929mm) replaced with Line
        Line((0.9923, -1.4928), (0.1343, -1.4446))
        # Near-straight arc (sagitta=0.004374mm) replaced with Line
        Line((0.1343, -1.4446), (-0.579, -1.4502))
        # Near-straight arc (sagitta=0.008589mm) replaced with Line
        Line((-0.579, -1.4502), (-1.426, -1.5114))
        RadiusArc((-1.426, -1.5114), (-2.1271, -1.6338), -6.2074)
        RadiusArc((-2.1271, -1.6338), (-2.7984, -1.8729), -3.2907)
        RadiusArc((-2.7984, -1.8729), (-3.2629, -2.2218), -1.6777)
        RadiusArc((-3.2629, -2.2218), (-3.5416, -2.6947), -1.3611)
        RadiusArc((-3.5416, -2.6947), (-3.6308, -3.1625), -1.6809)
        RadiusArc((-3.6308, -3.1625), (-3.5759, -3.7916), -2.1792)
        RadiusArc((-3.5759, -3.7916), (-3.2683, -4.389), -1.4772)
        RadiusArc((-3.2683, -4.389), (-2.696, -4.8045), -1.5933)
        RadiusArc((-2.696, -4.8045), (-1.8103, -5.064), -3.313)
        RadiusArc((-1.8103, -5.064), (-0.5849, -5.182), -7.9579)
        RadiusArc((-0.5849, -5.182), (1.0106, -5.1401), -13.2971)
    _inc_edges_sk_Sketch5_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_6 = Wire.combine(_inc_edges_sk_Sketch5_6)[0]
_wire_sk_Sketch5_6 = _wire_sk_Sketch5_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch5_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch5_6.wrapped, True)
_face_sk_Sketch5_6 = Face(_mkf_sk_Sketch5_6.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0004, -40.0007, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6258, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    _face = _face_sk_Sketch4_2
    _vec = Vector(0.0, 0.0, 1.0) * 4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 4.000000 mm
    
    # -- Extrude4_p1 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * 4.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 4.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5_p0 --
    _face = _face_sk_Sketch5_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude5_p1 --
    _face = _face_sk_Sketch5_5
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude5_p2 --
    _face = _face_sk_Sketch5_6
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
