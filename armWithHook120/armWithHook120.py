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

# 'Sketch11': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -4.5),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch11:
    with BuildLine():
        Line((0.6, 51.5034), (0.6, 51.4))
        Line((0.6, 51.4), (-9.0, 51.4))
        Line((-9.0, 51.4), (-9.0, 48.0))
        RadiusArc((-9.0, 48.0), (-6.0, 45.0102), -2.9819)
        Line((-6.0, 45.0102), (-6.0, -60.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-6.0, -60.0), (-0.0, -66.0), -6.0)
        RadiusArc((-0.0, -66.0), (6.0, -60.0), -6.0)
        Line((6.0, -60.0), (6.0, 45.0102))
        RadiusArc((6.0, 45.0102), (9.0, 48.0), -2.9779)
        Line((9.0, 48.0), (9.0, 60.0))
        Line((9.0, 60.0), (-9.0, 60.0))
        Line((-9.0, 60.0), (-9.0, 56.6))
        Line((-9.0, 56.6), (0.6, 56.6))
        Line((0.6, 56.6), (0.6, 56.4966))
        # Arc split: sweep=211.61deg >= 150 — emitted as two half-arcs
        RadiusArc((0.6, 56.4966), (3.9014, 54.0), 2.5947)
        RadiusArc((3.9014, 54.0), (0.6, 51.5034), 2.5947)
    _inc_edges_sk_Sketch11 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11 = Wire.combine(_inc_edges_sk_Sketch11)[0]
_wire_sk_Sketch11 = _wire_sk_Sketch11.moved(_inclined_plane_1.location)
_mkf_sk_Sketch11 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch11.wrapped, True)
_face_sk_Sketch11 = Face(_mkf_sk_Sketch11.Face())

# 'Sketch13': 11 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch13_2:
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
    _inc_edges_sk_Sketch13_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_2 = Wire.combine(_inc_edges_sk_Sketch13_2)[0]
_wire_sk_Sketch13_2 = _wire_sk_Sketch13_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch13_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch13_2.wrapped, True)
_face_sk_Sketch13_2 = Face(_mkf_sk_Sketch13_2.Face())

# 'Sketch13': 58 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch13_3:
    with BuildLine():
        # Near-straight arc (sagitta=0.002985mm) replaced with Line
        Line((1.2097, -9.8594), (0.6025, -9.1372))
        # Near-straight arc (sagitta=0.003808mm) replaced with Line
        Line((0.6025, -9.1372), (0.116, -8.5981))
        # Near-straight arc (sagitta=0.005205mm) replaced with Line
        Line((0.116, -8.5981), (-0.2522, -8.2394))
        # Near-straight arc (sagitta=0.003004mm) replaced with Line
        Line((-0.2522, -8.2394), (-0.5158, -8.0223))
        # Near-straight arc (sagitta=0.004987mm) replaced with Line
        Line((-0.5158, -8.0223), (-0.8454, -7.7955))
        # Near-straight arc (sagitta=0.005359mm) replaced with Line
        Line((-0.8454, -7.7955), (-1.175, -7.6182))
        RadiusArc((-1.175, -7.6182), (-1.6377, -7.4544), -2.6059)
        RadiusArc((-1.6377, -7.4544), (-2.1698, -7.3893), -2.2475)
        RadiusArc((-2.1698, -7.3893), (-2.8652, -7.4669), -3.0668)
        RadiusArc((-2.8652, -7.4669), (-3.446, -7.6999), -2.1603)
        RadiusArc((-3.446, -7.6999), (-3.8282, -7.9981), -1.9082)
        RadiusArc((-3.8282, -7.9981), (-4.1958, -8.5036), -2.0617)
        RadiusArc((-4.1958, -8.5036), (-4.392, -9.0045), -2.5286)
        RadiusArc((-4.392, -9.0045), (-4.5011, -9.5909), -3.4177)
        RadiusArc((-4.5011, -9.5909), (-4.5229, -10.2428), -4.7813)
        RadiusArc((-4.5229, -10.2428), (-4.4578, -10.8284), -3.824)
        RadiusArc((-4.4578, -10.8284), (-4.3059, -11.3528), -3.0281)
        RadiusArc((-4.3059, -11.3528), (-4.0671, -11.816), -2.6109)
        RadiusArc((-4.0671, -11.816), (-3.6542, -12.3009), -2.5102)
        RadiusArc((-3.6542, -12.3009), (-3.0313, -12.7017), -2.2807)
        RadiusArc((-3.0313, -12.7017), (-2.2735, -12.9004), -2.5953)
        Line((-2.2735, -12.9004), (-2.1698, -11.7774))
        RadiusArc((-2.1698, -11.7774), (-2.6722, -11.6539), 1.994)
        # Near-straight arc (sagitta=0.00842mm) replaced with Line
        Line((-2.6722, -11.6539), (-2.9523, -11.5049))
        RadiusArc((-2.9523, -11.5049), (-3.29, -11.1817), 1.4265)
        RadiusArc((-3.29, -11.1817), (-3.485, -10.8401), 1.5336)
        RadiusArc((-3.485, -10.8401), (-3.5952, -10.4436), 1.7268)
        RadiusArc((-3.5952, -10.4436), (-3.6206, -9.9898), 2.2632)
        # Near-straight arc (sagitta=0.007368mm) replaced with Line
        Line((-3.6206, -9.9898), (-3.579, -9.6305))
        # Near-straight arc (sagitta=0.008156mm) replaced with Line
        Line((-3.579, -9.6305), (-3.4817, -9.3172))
        RadiusArc((-3.4817, -9.3172), (-3.2821, -8.9902), 1.3418)
        RadiusArc((-3.2821, -8.9902), (-3.0009, -8.7416), 1.2395)
        # Near-straight arc (sagitta=0.009198mm) replaced with Line
        Line((-3.0009, -8.7416), (-2.7248, -8.6087))
        RadiusArc((-2.7248, -8.6087), (-2.3163, -8.5257), 1.612)
        RadiusArc((-2.3163, -8.5257), (-1.861, -8.5355), 2.2088)
        RadiusArc((-1.861, -8.5355), (-1.4822, -8.6255), 1.8551)
        RadiusArc((-1.4822, -8.6255), (-1.1381, -8.7922), 1.7545)
        # Near-straight arc (sagitta=0.00793mm) replaced with Line
        Line((-1.1381, -8.7922), (-0.761, -9.0591))
        # Near-straight arc (sagitta=0.005537mm) replaced with Line
        Line((-0.761, -9.0591), (-0.4504, -9.3397))
        # Near-straight arc (sagitta=0.002206mm) replaced with Line
        Line((-0.4504, -9.3397), (-0.198, -9.6062))
        # Near-straight arc (sagitta=0.00253mm) replaced with Line
        Line((-0.198, -9.6062), (0.1046, -9.954))
        # Near-straight arc (sagitta=0.001656mm) replaced with Line
        Line((0.1046, -9.954), (0.3569, -10.2669))
        # Near-straight arc (sagitta=0.001303mm) replaced with Line
        Line((0.3569, -10.2669), (0.6163, -10.5914))
        # Near-straight arc (sagitta=0.001271mm) replaced with Line
        Line((0.6163, -10.5914), (0.8899, -10.9179))
        # Near-straight arc (sagitta=0.001853mm) replaced with Line
        Line((0.8899, -10.9179), (1.2372, -11.3122))
        # Near-straight arc (sagitta=0.004281mm) replaced with Line
        Line((1.2372, -11.3122), (1.6133, -11.6979))
        # Near-straight arc (sagitta=0.005743mm) replaced with Line
        Line((1.6133, -11.6979), (2.0941, -12.1294))
        # Near-straight arc (sagitta=0.005247mm) replaced with Line
        Line((2.0941, -12.1294), (2.4698, -12.4189))
        # Near-straight arc (sagitta=0.005748mm) replaced with Line
        Line((2.4698, -12.4189), (2.8824, -12.6783))
        # Near-straight arc (sagitta=0.00804mm) replaced with Line
        Line((2.8824, -12.6783), (3.4268, -12.9492))
        Line((3.4268, -12.9492), (4.2019, -12.9492))
        Line((4.2019, -12.9492), (4.2019, -7.255))
        Line((4.2019, -7.255), (3.2681, -7.255))
        Line((3.2681, -7.255), (3.2681, -11.7408))
        RadiusArc((3.2681, -11.7408), (2.6836, -11.3936), 2.6684)
        # Near-straight arc (sagitta=0.008889mm) replaced with Line
        Line((2.6836, -11.3936), (2.2552, -11.027))
        # Near-straight arc (sagitta=0.005068mm) replaced with Line
        Line((2.2552, -11.027), (1.8565, -10.6185))
        # Near-straight arc (sagitta=0.00887mm) replaced with Line
        Line((1.8565, -10.6185), (1.2097, -9.8594))
    _inc_edges_sk_Sketch13_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_3 = Wire.combine(_inc_edges_sk_Sketch13_3)[0]
_wire_sk_Sketch13_3 = _wire_sk_Sketch13_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch13_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch13_3.wrapped, True)
_face_sk_Sketch13_3 = Face(_mkf_sk_Sketch13_3.Face())

# 'Sketch13': 29 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 4.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch13_4:
    with BuildLine():
        RadiusArc((-2.7528, -5.9117), (-1.9267, -6.1432), -5.0142)
        # Near-straight arc (sagitta=0.00398mm) replaced with Line
        Line((-1.9267, -6.1432), (-1.4544, -6.2203))
        RadiusArc((-1.4544, -6.2203), (-0.3913, -6.2975), -10.31)
        RadiusArc((-0.3913, -6.2975), (0.7243, -6.2709), -11.0697)
        RadiusArc((0.7243, -6.2709), (1.6954, -6.1385), -7.4358)
        RadiusArc((1.6954, -6.1385), (2.7017, -5.8242), -4.6816)
        RadiusArc((2.7017, -5.8242), (3.4759, -5.3444), -3.002)
        RadiusArc((3.4759, -5.3444), (4.0028, -4.7003), -2.3212)
        RadiusArc((4.0028, -4.7003), (4.2436, -4.0669), -2.5007)
        RadiusArc((4.2436, -4.0669), (4.3239, -3.3284), -3.3325)
        RadiusArc((4.3239, -3.3284), (4.2785, -2.7641), -3.3226)
        RadiusArc((4.2785, -2.7641), (4.0767, -2.1018), -2.5676)
        RadiusArc((4.0767, -2.1018), (3.7134, -1.5423), -2.3416)
        RadiusArc((3.7134, -1.5423), (3.1887, -1.0856), -2.6993)
        RadiusArc((3.1887, -1.0856), (2.6939, -0.8121), -3.4973)
        RadiusArc((2.6939, -0.8121), (1.9073, -0.5419), -4.7801)
        RadiusArc((1.9073, -0.5419), (0.9757, -0.3798), -7.0364)
        # Near-straight arc (sagitta=0.008104mm) replaced with Line
        Line((0.9757, -0.3798), (0.1819, -0.3291))
        RadiusArc((0.1819, -0.3291), (-0.9343, -0.3547), -11.4831)
        RadiusArc((-0.9343, -0.3547), (-1.9119, -0.4833), -7.7045)
        RadiusArc((-1.9119, -0.4833), (-3.0918, -0.869), -4.4188)
        RadiusArc((-3.0918, -0.869), (-3.6867, -1.256), -2.7789)
        RadiusArc((-3.6867, -1.256), (-4.1235, -1.755), -2.2652)
        RadiusArc((-4.1235, -1.755), (-4.4462, -2.538), -2.4978)
        RadiusArc((-4.4462, -2.538), (-4.5256, -3.2918), -3.5011)
        RadiusArc((-4.5256, -3.2918), (-4.4029, -4.2402), -3.2774)
        RadiusArc((-4.4029, -4.2402), (-3.9315, -5.132), -2.3072)
        RadiusArc((-3.9315, -5.132), (-3.4209, -5.5774), -2.4255)
        RadiusArc((-3.4209, -5.5774), (-2.7528, -5.9117), -3.5491)
    _inc_edges_sk_Sketch13_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_4 = Wire.combine(_inc_edges_sk_Sketch13_4)[0]
_wire_sk_Sketch13_4 = _wire_sk_Sketch13_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch13_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch13_4.wrapped, True)
_face_sk_Sketch13_4 = Face(_mkf_sk_Sketch13_4.Face())

# 'Sketch14': 21 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 4.5),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch14_5:
    with BuildLine():
        RadiusArc((-1.6149, -1.5363), (-2.6824, -1.8174), -4.6539)
        RadiusArc((-2.6824, -1.8174), (-3.2629, -2.2218), -1.7953)
        RadiusArc((-3.2629, -2.2218), (-3.6011, -2.9187), -1.3869)
        RadiusArc((-3.6011, -2.9187), (-3.6199, -3.5518), -2.0658)
        RadiusArc((-3.6199, -3.5518), (-3.4002, -4.2102), -1.6571)
        RadiusArc((-3.4002, -4.2102), (-3.0156, -4.6193), -1.4249)
        RadiusArc((-3.0156, -4.6193), (-2.2956, -4.952), -2.1209)
        RadiusArc((-2.2956, -4.952), (-1.4396, -5.1191), -4.9776)
        RadiusArc((-1.4396, -5.1191), (-0.3476, -5.1879), -9.9349)
        RadiusArc((-0.3476, -5.1879), (0.8058, -5.158), -13.3723)
        RadiusArc((0.8058, -5.158), (1.7425, -5.0284), -7.3928)
        RadiusArc((1.7425, -5.0284), (2.5785, -4.7413), -3.6022)
        RadiusArc((2.5785, -4.7413), (3.1214, -4.298), -1.7277)
        RadiusArc((3.1214, -4.298), (3.3928, -3.6858), -1.4139)
        RadiusArc((3.3928, -3.6858), (3.4114, -3.0667), -1.9615)
        RadiusArc((3.4114, -3.0667), (3.1803, -2.4287), -1.5221)
        RadiusArc((3.1803, -2.4287), (2.6719, -1.9567), -1.5115)
        RadiusArc((2.6719, -1.9567), (1.8793, -1.6433), -2.7202)
        RadiusArc((1.8793, -1.6433), (0.9923, -1.4928), -5.684)
        RadiusArc((0.9923, -1.4928), (-0.3445, -1.4445), -10.711)
        RadiusArc((-0.3445, -1.4445), (-1.6149, -1.5363), -10.9087)
    _inc_edges_sk_Sketch14_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch14_5 = Wire.combine(_inc_edges_sk_Sketch14_5)[0]
_wire_sk_Sketch14_5 = _wire_sk_Sketch14_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch14_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch14_5.wrapped, True)
_face_sk_Sketch14_5 = Face(_mkf_sk_Sketch14_5.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch11
    _vec = Vector(0.0, 0.0, -1.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0002, -60.0002, -4.5), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 2.6247, 9.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10_p0 --
    _face = _face_sk_Sketch13_2
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 3.000000 mm
    
    # -- Extrude10_p1 --
    _face = _face_sk_Sketch13_3
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 3.000000 mm
    
    # -- Extrude10_p2 --
    _face = _face_sk_Sketch13_4
    _vec = Vector(0.0, 0.0, 1.0) * 3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 3.000000 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch14_5
    _vec = Vector(0.0, 0.0, 1.0) * -0.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.500000119 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
