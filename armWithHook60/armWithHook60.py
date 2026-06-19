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
        Line((6.0, 214.0), (-90.0, 214.0))
        Line((-90.0, 214.0), (-90.0, 180.0))
        RadiusArc((-90.0, 180.0), (-60.0, 150.102), -29.8459)
        Line((-60.0, 150.102), (-60.0, -300.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -300.0), (-0.0, -360.0), -60.0)
        RadiusArc((-0.0, -360.0), (60.0, -300.0), -60.0)
        Line((60.0, -300.0), (60.0, 150.102))
        RadiusArc((60.0, 150.102), (90.0, 180.0), -29.8459)
        Line((90.0, 180.0), (90.0, 300.0))
        Line((90.0, 300.0), (-90.0, 300.0))
        Line((-90.0, 300.0), (-90.0, 266.0))
        Line((-90.0, 266.0), (6.0, 266.0))
        Line((6.0, 266.0), (6.0, 264.966))
        # Arc split: sweep=211.65deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 264.966), (39.0272, 240.0), 25.9498)
        RadiusArc((39.0272, 240.0), (6.0, 215.034), 25.9498)
        Line((6.0, 215.034), (6.0, 214.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 25 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((-25.8484, -84.6344), (-30.7002, -86.9063), -17.7426)
        RadiusArc((-30.7002, -86.9063), (-35.5986, -93.5143), -12.4439)
        RadiusArc((-35.5986, -93.5143), (-35.9947, -102.384), -18.4564)
        RadiusArc((-35.9947, -102.384), (-31.6353, -110.298), -15.4907)
        RadiusArc((-31.6353, -110.298), (-20.8772, -116.215), -27.8405)
        RadiusArc((-20.8772, -116.215), (-2.2287, -118.507), -74.552)
        RadiusArc((-2.2287, -118.507), (-8.454, -113.434), 20.1889)
        RadiusArc((-8.454, -113.434), (-12.7559, -104.669), 23.0305)
        RadiusArc((-12.7559, -104.669), (-13.2826, -93.0702), 32.4429)
        RadiusArc((-13.2826, -93.0702), (-9.5764, -82.8217), 23.8067)
        RadiusArc((-9.5764, -82.8217), (1.9308, -73.745), 24.0493)
        RadiusArc((1.9308, -73.745), (15.6721, -71.79), 38.1073)
        RadiusArc((15.6721, -71.79), (25.3368, -73.3653), 34.9495)
        RadiusArc((25.3368, -73.3653), (39.0056, -83.4793), 24.521)
        RadiusArc((39.0056, -83.4793), (43.2391, -99.5258), 31.0292)
        RadiusArc((43.2391, -99.5258), (35.1257, -119.569), 26.3628)
        RadiusArc((35.1257, -119.569), (18.1194, -127.756), 40.4113)
        RadiusArc((18.1194, -127.756), (-10.1695, -128.862), 97.0952)
        RadiusArc((-10.1695, -128.862), (-28.0934, -124.314), 53.3358)
        RadiusArc((-28.0934, -124.314), (-38.8226, -116.691), 30.428)
        RadiusArc((-38.8226, -116.691), (-45.0433, -102.49), 25.9646)
        RadiusArc((-45.0433, -102.49), (-44.5554, -91.4603), 35.0596)
        RadiusArc((-44.5554, -91.4603), (-41.4415, -83.6374), 22.6386)
        RadiusArc((-41.4415, -83.6374), (-27.7402, -74.1368), 24.4176)
        Line((-27.7402, -74.1368), (-25.8484, -84.6344))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch3': 49 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((34.2766, -101.893), (34.3611, -100.905))
        Line((34.3611, -100.905), (34.3893, -99.892))
        Line((34.3893, -99.892), (34.3657, -98.8495))
        Line((34.3657, -98.8495), (34.2944, -97.8372))
        RadiusArc((34.2944, -97.8372), (24.1377, -84.5514), -15.1591)
        Line((24.1377, -84.5514), (23.1947, -84.2101))
        Line((23.1947, -84.2101), (22.2162, -83.909))
        Line((22.2162, -83.909), (21.2021, -83.6479))
        Line((21.2021, -83.6479), (20.1524, -83.4271))
        Line((20.1524, -83.4271), (19.0674, -83.2465))
        Line((19.0674, -83.2465), (17.9466, -83.1059))
        Line((17.9466, -83.1059), (16.7905, -83.0055))
        Line((16.7905, -83.0055), (15.5988, -82.9453))
        Line((15.5988, -82.9453), (14.3716, -82.9253))
        Line((14.3716, -82.9253), (13.1456, -82.9456))
        Line((13.1456, -82.9456), (11.9576, -83.0066))
        Line((11.9576, -83.0066), (10.8075, -83.1084))
        Line((10.8075, -83.1084), (9.6954, -83.2507))
        Line((9.6954, -83.2507), (8.6212, -83.4338))
        Line((8.6212, -83.4338), (7.5851, -83.6577))
        Line((7.5851, -83.6577), (6.5869, -83.9221))
        Line((6.5869, -83.9221), (5.6268, -84.2273))
        RadiusArc((5.6268, -84.2273), (-4.8944, -97.131), -15.1061)
        Line((-4.8944, -97.131), (-5.007, -98.1415))
        Line((-5.007, -98.1415), (-5.0746, -99.184))
        Line((-5.0746, -99.184), (-5.0971, -100.258))
        Line((-5.0971, -100.258), (-5.0772, -101.27))
        RadiusArc((-5.0772, -101.27), (9.7079, -117.324), -15.4694)
        Line((9.7079, -117.324), (10.7124, -117.387))
        Line((10.7124, -117.387), (11.7471, -117.408))
        Line((11.7471, -117.408), (13.0576, -117.386))
        Line((13.0576, -117.386), (14.3359, -117.321))
        Line((14.3359, -117.321), (15.5823, -117.211))
        Line((15.5823, -117.211), (16.7966, -117.059))
        Line((16.7966, -117.059), (17.979, -116.862))
        Line((17.979, -116.862), (19.1293, -116.622))
        Line((19.1293, -116.622), (20.2477, -116.338))
        Line((20.2477, -116.338), (21.3341, -116.011))
        Line((21.3341, -116.011), (22.3885, -115.639))
        Line((22.3885, -115.639), (23.4108, -115.225))
        Line((23.4108, -115.225), (24.4011, -114.766))
        Line((24.4011, -114.766), (25.3595, -114.264))
        Line((25.3595, -114.264), (26.2859, -113.718))
        Line((26.2859, -113.718), (27.1802, -113.129))
        Line((27.1802, -113.129), (28.0426, -112.496))
        Line((28.0426, -112.496), (28.8606, -111.828))
        Line((28.8606, -111.828), (29.6222, -111.136))
        Line((29.6222, -111.136), (30.3275, -110.419))
        RadiusArc((30.3275, -110.419), (34.2766, -101.893), -14.7213)
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch4': 120 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        Line((42.785, -27.6414), (43.0373, -29.4582))
        Line((43.0373, -29.4582), (43.1886, -31.3391))
        Line((43.1886, -31.3391), (43.2391, -33.2843))
        Line((43.2391, -33.2843), (43.1889, -35.2289))
        Line((43.1889, -35.2289), (43.0383, -37.108))
        Line((43.0383, -37.108), (42.7875, -38.9214))
        Line((42.7875, -38.9214), (42.4362, -40.6689))
        Line((42.4362, -40.6689), (41.9846, -42.3511))
        Line((41.9846, -42.3511), (41.4326, -43.9674))
        Line((41.4326, -43.9674), (40.7802, -45.5182))
        Line((40.7802, -45.5182), (40.0276, -47.0033))
        Line((40.0276, -47.0033), (39.1745, -48.4229))
        Line((39.1745, -48.4229), (38.2211, -49.7766))
        Line((38.2211, -49.7766), (37.1674, -51.0649))
        Line((37.1674, -51.0649), (36.0132, -52.2874))
        Line((36.0132, -52.2874), (34.7586, -53.4442))
        Line((34.7586, -53.4442), (33.4038, -54.5355))
        Line((33.4038, -54.5355), (31.9485, -55.5611))
        Line((31.9485, -55.5611), (30.397, -56.5207))
        Line((30.397, -56.5207), (28.7534, -57.4142))
        Line((28.7534, -57.4142), (27.0174, -58.2416))
        Line((27.0174, -58.2416), (25.1891, -59.0027))
        Line((25.1891, -59.0027), (23.2687, -59.6976))
        Line((23.2687, -59.6976), (21.256, -60.3264))
        Line((21.256, -60.3264), (19.1512, -60.8888))
        Line((19.1512, -60.8888), (16.9539, -61.3853))
        Line((16.9539, -61.3853), (14.6646, -61.8155))
        Line((14.6646, -61.8155), (12.283, -62.1794))
        Line((12.283, -62.1794), (9.8091, -62.4773))
        Line((9.8091, -62.4773), (7.243, -62.7089))
        Line((7.243, -62.7089), (4.5848, -62.8745))
        Line((4.5848, -62.8745), (1.8343, -62.9736))
        Line((1.8343, -62.9736), (-1.0083, -63.0067))
        Line((-1.0083, -63.0067), (-3.913, -62.9747))
        Line((-3.913, -62.9747), (-6.7188, -62.8783))
        Line((-6.7188, -62.8783), (-9.4258, -62.7174))
        Line((-9.4258, -62.7174), (-12.0341, -62.4925))
        Line((-12.0341, -62.4925), (-14.5438, -62.2032))
        Line((-14.5438, -62.2032), (-16.9547, -61.8497))
        Line((-16.9547, -61.8497), (-19.2668, -61.4317))
        Line((-19.2668, -61.4317), (-21.4803, -60.9496))
        Line((-21.4803, -60.9496), (-23.5948, -60.4031))
        Line((-23.5948, -60.4031), (-25.6108, -59.7923))
        Line((-25.6108, -59.7923), (-27.5279, -59.1173))
        Line((-27.5279, -59.1173), (-29.3465, -58.378))
        Line((-29.3465, -58.378), (-31.0661, -57.5745))
        Line((-31.0661, -57.5745), (-32.6871, -56.7065))
        Line((-32.6871, -56.7065), (-34.2094, -55.7744))
        Line((-34.2094, -55.7744), (-35.6331, -54.7728))
        Line((-35.6331, -54.7728), (-36.9588, -53.6966))
        Line((-36.9588, -53.6966), (-38.1862, -52.5458))
        Line((-38.1862, -52.5458), (-39.3153, -51.3203))
        Line((-39.3153, -51.3203), (-40.3464, -50.0204))
        Line((-40.3464, -50.0204), (-41.2791, -48.6458))
        Line((-41.2791, -48.6458), (-42.1138, -47.1967))
        Line((-42.1138, -47.1967), (-42.8503, -45.6729))
        Line((-42.8503, -45.6729), (-43.4885, -44.0746))
        Line((-43.4885, -44.0746), (-44.0286, -42.4016))
        Line((-44.0286, -42.4016), (-44.4704, -40.654))
        Line((-44.4704, -40.654), (-44.8141, -38.8319))
        Line((-44.8141, -38.8319), (-45.0597, -36.9351))
        Line((-45.0597, -36.9351), (-45.2069, -34.9638))
        Line((-45.2069, -34.9638), (-45.256, -32.9179))
        Line((-45.256, -32.9179), (-45.2063, -30.9271))
        Line((-45.2063, -30.9271), (-45.0574, -29.0071))
        Line((-45.0574, -29.0071), (-44.8093, -27.1581))
        Line((-44.8093, -27.1581), (-44.4617, -25.3796))
        Line((-44.4617, -25.3796), (-44.015, -23.672))
        Line((-44.015, -23.672), (-43.4689, -22.0352))
        Line((-43.4689, -22.0352), (-42.8236, -20.4692))
        Line((-42.8236, -20.4692), (-42.079, -18.974))
        Line((-42.079, -18.974), (-41.2352, -17.5496))
        Line((-41.2352, -17.5496), (-40.2919, -16.196))
        Line((-40.2919, -16.196), (-39.2496, -14.9132))
        Line((-39.2496, -14.9132), (-38.1078, -13.7012))
        Line((-38.1078, -13.7012), (-36.8668, -12.56))
        Line((-36.8668, -12.56), (-35.5266, -11.4896))
        Line((-35.5266, -11.4896), (-34.0869, -10.49))
        Line((-34.0869, -10.49), (-32.5502, -9.5578))
        Line((-32.5502, -9.5578), (-30.9178, -8.6899))
        Line((-30.9178, -8.6899), (-29.1901, -7.8864))
        Line((-29.1901, -7.8864), (-27.3668, -7.1471))
        Line((-27.3668, -7.1471), (-25.448, -6.472))
        Line((-25.448, -6.472), (-23.4337, -5.8612))
        Line((-23.4337, -5.8612), (-21.324, -5.3148))
        Line((-21.324, -5.3148), (-19.1187, -4.8326))
        Line((-19.1187, -4.8326), (-16.8179, -4.4148))
        Line((-16.8179, -4.4148), (-14.4218, -4.0611))
        Line((-14.4218, -4.0611), (-11.9301, -3.7718))
        Line((-11.9301, -3.7718), (-9.3428, -3.5469))
        Line((-9.3428, -3.5469), (-6.6602, -3.3862))
        Line((-6.6602, -3.3862), (-3.882, -3.2898))
        Line((-3.882, -3.2898), (-1.0083, -3.2576))
        Line((-1.0083, -3.2576), (1.8187, -3.2913))
        Line((1.8187, -3.2913), (4.5554, -3.3926))
        Line((4.5554, -3.3926), (7.2014, -3.5616))
        Line((7.2014, -3.5616), (9.7569, -3.7979))
        Line((9.7569, -3.7979), (12.2218, -4.1019))
        Line((12.2218, -4.1019), (14.5961, -4.4734))
        Line((14.5961, -4.4734), (16.8797, -4.9124))
        Line((16.8797, -4.9124), (19.0727, -5.419))
        Line((19.0727, -5.419), (21.1752, -5.993))
        Line((21.1752, -5.993), (23.1871, -6.6348))
        Line((23.1871, -6.6348), (25.1083, -7.3441))
        Line((25.1083, -7.3441), (26.9389, -8.1207))
        Line((26.9389, -8.1207), (28.679, -8.965))
        Line((28.679, -8.965), (30.3285, -9.8769))
        Line((30.3285, -9.8769), (31.8874, -10.8563))
        Line((31.8874, -10.8563), (33.3505, -11.9014))
        Line((33.3505, -11.9014), (34.7127, -13.0109))
        Line((34.7127, -13.0109), (35.974, -14.1847))
        Line((35.974, -14.1847), (37.1344, -15.4227))
        Line((37.1344, -15.4227), (38.1938, -16.725))
        Line((38.1938, -16.725), (39.1525, -18.0917))
        Line((39.1525, -18.0917), (40.0101, -19.5226))
        Line((40.0101, -19.5226), (40.7669, -21.0178))
        Line((40.7669, -21.0178), (41.4227, -22.5774))
        Line((41.4227, -22.5774), (41.9778, -24.201))
        Line((41.9778, -24.201), (42.4318, -25.8891))
        Line((42.4318, -25.8891), (42.785, -27.6414))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch5': 120 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with BuildLine():
        Line((34.1171, -35.6729), (34.2302, -34.4414))
        Line((34.2302, -34.4414), (34.2679, -33.1628))
        Line((34.2679, -33.1628), (34.2294, -31.8919))
        Line((34.2294, -31.8919), (34.1138, -30.6671))
        Line((34.1138, -30.6671), (33.9212, -29.4884))
        Line((33.9212, -29.4884), (33.6516, -28.3559))
        Line((33.6516, -28.3559), (33.3049, -27.2694))
        Line((33.3049, -27.2694), (32.8813, -26.2292))
        Line((32.8813, -26.2292), (32.3805, -25.235))
        Line((32.3805, -25.235), (31.8028, -24.287))
        Line((31.8028, -24.287), (31.1481, -23.385))
        Line((31.1481, -23.385), (30.4163, -22.5291))
        Line((30.4163, -22.5291), (29.6074, -21.7195))
        Line((29.6074, -21.7195), (28.7216, -20.956))
        Line((28.7216, -20.956), (27.7586, -20.2385))
        Line((27.7586, -20.2385), (26.7188, -19.5671))
        Line((26.7188, -19.5671), (25.6018, -18.9418))
        Line((25.6018, -18.9418), (24.4049, -18.3598))
        Line((24.4049, -18.3598), (23.1256, -17.8178))
        Line((23.1256, -17.8178), (21.7639, -17.316))
        Line((21.7639, -17.316), (20.3197, -16.8544))
        Line((20.3197, -16.8544), (18.793, -16.433))
        Line((18.793, -16.433), (17.184, -16.0515))
        Line((17.184, -16.0515), (15.4924, -15.7103))
        Line((15.4924, -15.7103), (13.7184, -15.4092))
        Line((13.7184, -15.4092), (11.8619, -15.1483))
        Line((11.8619, -15.1483), (9.9229, -14.9275))
        Line((9.9229, -14.9275), (7.9016, -14.7469))
        Line((7.9016, -14.7469), (5.7977, -14.6063))
        Line((5.7977, -14.6063), (3.6114, -14.5059))
        Line((3.6114, -14.5059), (1.3426, -14.4458))
        Line((1.3426, -14.4458), (-1.0083, -14.4257))
        Line((-1.0083, -14.4257), (-3.4448, -14.4447))
        Line((-3.4448, -14.4447), (-5.7899, -14.5021))
        Line((-5.7899, -14.5021), (-8.0441, -14.5978))
        Line((-8.0441, -14.5978), (-10.2071, -14.7316))
        Line((-10.2071, -14.7316), (-12.2789, -14.9037))
        Line((-12.2789, -14.9037), (-14.2595, -15.1141))
        Line((-14.2595, -15.1141), (-16.1491, -15.3627))
        Line((-16.1491, -15.3627), (-17.9474, -15.6496))
        Line((-17.9474, -15.6496), (-19.6547, -15.9747))
        Line((-19.6547, -15.9747), (-21.2708, -16.338))
        Line((-21.2708, -16.338), (-22.7957, -16.7397))
        Line((-22.7957, -16.7397), (-24.2296, -17.1796))
        Line((-24.2296, -17.1796), (-25.5722, -17.6576))
        Line((-25.5722, -17.6576), (-26.8237, -18.174))
        Line((-26.8237, -18.174), (-27.984, -18.7286))
        Line((-27.984, -18.7286), (-29.0617, -19.3271))
        Line((-29.0617, -19.3271), (-30.065, -19.9753))
        Line((-30.065, -19.9753), (-30.994, -20.6731))
        Line((-30.994, -20.6731), (-31.8488, -21.4204))
        Line((-31.8488, -21.4204), (-32.6291, -22.2176))
        Line((-32.6291, -22.2176), (-33.3351, -23.0643))
        Line((-33.3351, -23.0643), (-33.9668, -23.9606))
        Line((-33.9668, -23.9606), (-34.5242, -24.9066))
        Line((-34.5242, -24.9066), (-35.0073, -25.9021))
        Line((-35.0073, -25.9021), (-35.4161, -26.9473))
        Line((-35.4161, -26.9473), (-35.7506, -28.0423))
        Line((-35.7506, -28.0423), (-36.0106, -29.1867))
        Line((-36.0106, -29.1867), (-36.1964, -30.3809))
        Line((-36.1964, -30.3809), (-36.308, -31.6246))
        Line((-36.308, -31.6246), (-36.3451, -32.9179))
        Line((-36.3451, -32.9179), (-36.3084, -34.2435))
        Line((-36.3084, -34.2435), (-36.1986, -35.5182))
        Line((-36.1986, -35.5182), (-36.0155, -36.7422))
        Line((-36.0155, -36.7422), (-35.7591, -37.9156))
        Line((-35.7591, -37.9156), (-35.4295, -39.0382))
        Line((-35.4295, -39.0382), (-35.0267, -40.11))
        Line((-35.0267, -40.11), (-34.5506, -41.1313))
        Line((-34.5506, -41.1313), (-34.0015, -42.1016))
        Line((-34.0015, -42.1016), (-33.3789, -43.0214))
        Line((-33.3789, -43.0214), (-32.6831, -43.8904))
        Line((-32.6831, -43.8904), (-31.9141, -44.7087))
        Line((-31.9141, -44.7087), (-31.0718, -45.4762))
        Line((-31.0718, -45.4762), (-30.1562, -46.1931))
        Line((-30.1562, -46.1931), (-29.1675, -46.8591))
        Line((-29.1675, -46.8591), (-28.1056, -47.4745))
        Line((-28.1056, -47.4745), (-26.9598, -48.0449))
        Line((-26.9598, -48.0449), (-25.7198, -48.5757))
        Line((-25.7198, -48.5757), (-24.3854, -49.0674))
        Line((-24.3854, -49.0674), (-22.9564, -49.5197))
        Line((-22.9564, -49.5197), (-21.4331, -49.9326))
        Line((-21.4331, -49.9326), (-19.8155, -50.3062))
        Line((-19.8155, -50.3062), (-18.1035, -50.6404))
        Line((-18.1035, -50.6404), (-16.297, -50.9355))
        Line((-16.297, -50.9355), (-14.3961, -51.1911))
        Line((-14.3961, -51.1911), (-12.4008, -51.4073))
        Line((-12.4008, -51.4073), (-10.3111, -51.5843))
        Line((-10.3111, -51.5843), (-8.127, -51.722))
        Line((-8.127, -51.722), (-5.8485, -51.8204))
        Line((-5.8485, -51.8204), (-3.4756, -51.8793))
        Line((-3.4756, -51.8793), (-1.0083, -51.899))
        Line((-1.0083, -51.899), (1.389, -51.879))
        Line((1.389, -51.879), (3.6993, -51.8193))
        Line((3.6993, -51.8193), (5.9222, -51.7195))
        Line((5.9222, -51.7195), (8.0579, -51.58))
        Line((8.0579, -51.58), (10.106, -51.4006))
        Line((10.106, -51.4006), (12.067, -51.1813))
        Line((12.067, -51.1813), (13.9404, -50.9221))
        Line((13.9404, -50.9221), (15.7268, -50.623))
        Line((15.7268, -50.623), (17.4255, -50.2841))
        Line((17.4255, -50.2841), (19.037, -49.9054))
        Line((19.037, -49.9054), (20.5612, -49.4867))
        Line((20.5612, -49.4867), (21.9981, -49.0282))
        Line((21.9981, -49.0282), (23.3475, -48.5298))
        Line((23.3475, -48.5298), (24.6097, -47.9915))
        Line((24.6097, -47.9915), (25.7845, -47.4133))
        Line((25.7845, -47.4133), (26.8779, -46.7918))
        Line((26.8779, -46.7918), (27.896, -46.1234))
        Line((27.896, -46.1234), (28.8385, -45.408))
        Line((28.8385, -45.408), (29.7057, -44.6457))
        Line((29.7057, -44.6457), (30.4974, -43.8364))
        Line((30.4974, -43.8364), (31.2138, -42.9802))
        Line((31.2138, -42.9802), (31.8549, -42.077))
        Line((31.8549, -42.077), (32.4203, -41.127))
        Line((32.4203, -41.127), (32.9105, -40.13))
        Line((32.9105, -40.13), (33.3253, -39.0862))
        Line((33.3253, -39.0862), (33.6646, -37.9953))
        Line((33.6646, -37.9953), (33.9285, -36.8576))
        Line((33.9285, -36.8576), (34.1171, -35.6729))
    _inc_edges_sk_Sketch5_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_5 = Wire.combine(_inc_edges_sk_Sketch5_5)[0]
_wire_sk_Sketch5_5 = _wire_sk_Sketch5_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch5_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch5_5.wrapped, True)
_face_sk_Sketch5_5 = Face(_mkf_sk_Sketch5_5.Face())

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
    _bore_ax = _gAx2(_gPnt(-0.0002, -300.0111, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.2392, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(0.0, 0.0, 1.0) * 40.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 40.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_5
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
