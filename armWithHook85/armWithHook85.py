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
        Line((0.6, 33.9), (-9.0, 33.9))
        Line((-9.0, 33.9), (-9.0, 30.5))
        RadiusArc((-9.0, 30.5), (-6.0, 27.5102), -2.9878)
        Line((-6.0, 27.5102), (-6.0, -42.5))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -42.5), (-0.0, -48.5), -6.0)
        RadiusArc((-0.0, -48.5), (6.0, -42.5), -6.0)
        Line((6.0, -42.5), (6.0, 27.5102))
        RadiusArc((6.0, 27.5102), (9.0, 30.5), -2.9846)
        Line((9.0, 30.5), (9.0, 42.5))
        Line((9.0, 42.5), (-9.0, 42.5))
        Line((-9.0, 42.5), (-9.0, 39.1))
        Line((-9.0, 39.1), (0.6, 39.1))
        Line((0.6, 39.1), (0.6, 38.9966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 38.9966), (3.9, 36.5), 2.5944)
        RadiusArc((3.9, 36.5), (0.6, 34.0034), 2.5944)
        Line((0.6, 34.0034), (0.6, 33.9))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 242 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
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
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 40 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((3.2068, -5.9256), (3.799, -5.3547), -2.2114)
        RadiusArc((3.799, -5.3547), (4.135, -4.6897), -2.644)
        RadiusArc((4.135, -4.6897), (4.3029, -3.8763), -3.7221)
        RadiusArc((4.3029, -3.8763), (4.2684, -2.6935), -4.5505)
        RadiusArc((4.2684, -2.6935), (3.9038, -1.6468), -2.8895)
        RadiusArc((3.9038, -1.6468), (3.3241, -0.9698), -2.5226)
        RadiusArc((3.3241, -0.9698), (2.5445, -0.5384), -2.5732)
        RadiusArc((2.5445, -0.5384), (1.4005, -0.3623), -3.7046)
        RadiusArc((1.4005, -0.3623), (0.3824, -0.5264), -2.8793)
        RadiusArc((0.3824, -0.5264), (-0.3336, -0.9285), -2.4169)
        RadiusArc((-0.3336, -0.9285), (-0.8083, -1.4382), -2.4671)
        RadiusArc((-0.8083, -1.4382), (-1.1226, -2.0667), -2.524)
        RadiusArc((-1.1226, -2.0667), (-1.2815, -2.9723), -3.3733)
        RadiusArc((-1.2815, -2.9723), (-1.2457, -3.6809), -3.6388)
        RadiusArc((-1.2457, -3.6809), (-1.087, -4.2884), -2.8141)
        RadiusArc((-1.087, -4.2884), (-0.7355, -4.9153), -2.4947)
        Line((-0.7355, -4.9153), (-3.4637, -4.7505))
        Line((-3.4637, -4.7505), (-3.4637, -0.8628))
        Line((-3.4637, -0.8628), (-4.3974, -0.8628))
        Line((-4.3974, -0.8628), (-4.3974, -5.7514))
        Line((-4.3974, -5.7514), (0.2287, -6.0382))
        Line((0.2287, -6.0382), (0.2287, -4.9641))
        RadiusArc((0.2287, -4.9641), (-0.0738, -4.554), 2.7709)
        RadiusArc((-0.0738, -4.554), (-0.2654, -4.1436), 2.0135)
        RadiusArc((-0.2654, -4.1436), (-0.3652, -3.7117), 2.3371)
        # Near-straight arc (sagitta=0.005663mm) replaced with Line
        Line((-0.3652, -3.7117), (-0.3877, -3.3651))
        RadiusArc((-0.3877, -3.3651), (-0.3076, -2.7377), 2.2133)
        RadiusArc((-0.3076, -2.7377), (-0.0117, -2.1476), 1.6449)
        RadiusArc((-0.0117, -2.1476), (0.5629, -1.6849), 1.6366)
        RadiusArc((0.5629, -1.6849), (1.1127, -1.5237), 1.8652)
        RadiusArc((1.1127, -1.5237), (1.7825, -1.5236), 2.8868)
        RadiusArc((1.7825, -1.5236), (2.4924, -1.726), 2.0306)
        RadiusArc((2.4924, -1.726), (3.0851, -2.2197), 1.6355)
        RadiusArc((3.0851, -2.2197), (3.3414, -2.743), 1.6878)
        RadiusArc((3.3414, -2.743), (3.4268, -3.3895), 2.4232)
        RadiusArc((3.4268, -3.3895), (3.2851, -4.2283), 2.135)
        RadiusArc((3.2851, -4.2283), (2.7408, -4.9144), 1.4653)
        RadiusArc((2.7408, -4.9144), (2.1512, -5.1777), 2.0214)
        Line((2.1512, -5.1777), (2.2794, -6.2884))
        RadiusArc((2.2794, -6.2884), (3.2068, -5.9256), -2.4818)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch3': 17 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        RadiusArc((1.724, -11.8995), (2.5449, -11.7706), -2.5068)
        RadiusArc((2.5449, -11.7706), (2.9949, -11.5048), -1.4201)
        RadiusArc((2.9949, -11.5048), (3.3738, -10.9247), -1.3527)
        RadiusArc((3.3738, -10.9247), (3.5001, -10.0869), -2.3494)
        # Near-straight arc (sagitta=0.007928mm) replaced with Line
        Line((3.5001, -10.0869), (3.4695, -9.641))
        RadiusArc((3.4695, -9.641), (3.3451, -9.1751), -2.0564)
        RadiusArc((3.3451, -9.1751), (3.0101, -8.6937), -1.3492)
        RadiusArc((3.0101, -8.6937), (2.5572, -8.4359), -1.2963)
        RadiusArc((2.5572, -8.4359), (1.8226, -8.3128), -2.259)
        RadiusArc((1.8226, -8.3128), (1.0391, -8.4146), -2.313)
        RadiusArc((1.0391, -8.4146), (0.5501, -8.7256), -1.3075)
        RadiusArc((0.5501, -8.7256), (0.2038, -9.315), -1.4654)
        RadiusArc((0.2038, -9.315), (0.0901, -10.0105), -2.3123)
        RadiusArc((0.0901, -10.0105), (0.1368, -10.6402), -2.8895)
        RadiusArc((0.1368, -10.6402), (0.3676, -11.2269), -1.7856)
        RadiusArc((0.3676, -11.2269), (0.9261, -11.7303), -1.426)
        RadiusArc((0.9261, -11.7303), (1.724, -11.8995), -1.6509)
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# 'Sketch3': 19 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        RadiusArc((-1.4893, -11.5598), (-1.1763, -11.3461), -1.211)
        RadiusArc((-1.1763, -11.3461), (-0.9439, -11.0447), -1.2508)
        RadiusArc((-0.9439, -11.0447), (-0.7786, -10.5815), -1.5179)
        RadiusArc((-0.7786, -10.5815), (-0.7419, -9.9111), -2.4372)
        RadiusArc((-0.7419, -9.9111), (-0.8133, -9.4683), -2.3076)
        RadiusArc((-0.8133, -9.4683), (-0.964, -9.1074), -1.5144)
        RadiusArc((-0.964, -9.1074), (-1.2531, -8.7842), -1.114)
        RadiusArc((-1.2531, -8.7842), (-1.7653, -8.5695), -1.2301)
        RadiusArc((-1.7653, -8.5695), (-2.2491, -8.5245), -2.2592)
        RadiusArc((-2.2491, -8.5245), (-2.7743, -8.5884), -2.0511)
        RadiusArc((-2.7743, -8.5884), (-3.1827, -8.7803), -1.265)
        RadiusArc((-3.1827, -8.7803), (-3.5457, -9.2351), -1.1149)
        RadiusArc((-3.5457, -9.2351), (-3.7013, -9.9174), -1.8307)
        RadiusArc((-3.7013, -9.9174), (-3.6931, -10.4187), -3.0989)
        RadiusArc((-3.6931, -10.4187), (-3.545, -10.9928), -1.9455)
        RadiusArc((-3.545, -10.9928), (-3.1807, -11.4506), -1.214)
        RadiusArc((-3.1807, -11.4506), (-2.6109, -11.6814), -1.2513)
        RadiusArc((-2.6109, -11.6814), (-1.9678, -11.6936), -2.1986)
        RadiusArc((-1.9678, -11.6936), (-1.4893, -11.5598), -1.6525)
    _inc_edges_sk_Sketch3_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_5 = Wire.combine(_inc_edges_sk_Sketch3_5)[0]
_wire_sk_Sketch3_5 = _wire_sk_Sketch3_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch3_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch3_5.wrapped, True)
_face_sk_Sketch3_5 = Face(_mkf_sk_Sketch3_5.Face())

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
    _bore_ax = _gAx2(_gPnt(-0.0003, -42.4999, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6253, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -0.500000119 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3_p0 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    
    # -- Extrude3_p1 --
    _face = _face_sk_Sketch3_5
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
