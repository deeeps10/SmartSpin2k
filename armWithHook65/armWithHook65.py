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
        Line((6.0, 239.0), (-90.0, 239.0))
        Line((-90.0, 239.0), (-90.0, 205.0))
        RadiusArc((-90.0, 205.0), (-60.0, 175.102), -29.8459)
        Line((-60.0, 175.102), (-60.0, -325.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -325.0), (-0.0, -385.0), -60.0)
        RadiusArc((-0.0, -385.0), (60.0, -325.0), -60.0)
        Line((60.0, -325.0), (60.0, 175.102))
        RadiusArc((60.0, 175.102), (90.0, 205.0), -29.8459)
        Line((90.0, 205.0), (90.0, 325.0))
        Line((90.0, 325.0), (-90.0, 325.0))
        Line((-90.0, 325.0), (-90.0, 291.0))
        Line((-90.0, 291.0), (6.0, 291.0))
        Line((6.0, 291.0), (6.0, 289.966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 289.966), (39.0002, 265.0), 25.944)
        RadiusArc((39.0002, 265.0), (6.0, 240.034), 25.944)
        Line((6.0, 240.034), (6.0, 239.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch3': 182 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch3_2:
    with BuildLine():
        Line((42.9242, -94.5505), (43.0991, -96.1591))
        Line((43.0991, -96.1591), (43.204, -97.8174))
        Line((43.204, -97.8174), (43.2391, -99.5258))
        Line((43.2391, -99.5258), (43.1911, -101.432))
        Line((43.1911, -101.432), (43.047, -103.278))
        Line((43.047, -103.278), (42.807, -105.063))
        Line((42.807, -105.063), (42.471, -106.787))
        Line((42.471, -106.787), (42.0389, -108.45))
        Line((42.0389, -108.45), (41.5108, -110.053))
        Line((41.5108, -110.053), (40.8867, -111.595))
        Line((40.8867, -111.595), (40.1666, -113.076))
        Line((40.1666, -113.076), (39.3504, -114.496))
        Line((39.3504, -114.496), (38.4383, -115.855))
        Line((38.4383, -115.855), (37.4301, -117.154))
        Line((37.4301, -117.154), (36.326, -118.392))
        Line((36.326, -118.392), (35.1257, -119.569))
        Line((35.1257, -119.569), (33.8295, -120.686))
        Line((33.8295, -120.686), (32.4373, -121.741))
        Line((32.4373, -121.741), (30.9532, -122.732))
        Line((30.9532, -122.732), (29.3819, -123.655))
        Line((29.3819, -123.655), (27.7231, -124.51))
        Line((27.7231, -124.51), (25.977, -125.296))
        Line((25.977, -125.296), (24.1437, -126.013))
        Line((24.1437, -126.013), (22.2229, -126.663))
        Line((22.2229, -126.663), (20.2148, -127.244))
        Line((20.2148, -127.244), (18.1194, -127.756))
        Line((18.1194, -127.756), (15.9366, -128.201))
        Line((15.9366, -128.201), (13.6665, -128.577))
        Line((13.6665, -128.577), (11.3091, -128.884))
        Line((11.3091, -128.884), (8.8641, -129.123))
        Line((8.8641, -129.123), (6.3321, -129.294))
        Line((6.3321, -129.294), (3.7125, -129.397))
        Line((3.7125, -129.397), (1.0057, -129.431))
        Line((1.0057, -129.431), (-1.9263, -129.395))
        Line((-1.9263, -129.395), (-4.7662, -129.289))
        Line((-4.7662, -129.289), (-7.514, -129.111))
        Line((-7.514, -129.111), (-10.1695, -128.862))
        Line((-10.1695, -128.862), (-12.7328, -128.543))
        Line((-12.7328, -128.543), (-15.2039, -128.152))
        Line((-15.2039, -128.152), (-17.5827, -127.69))
        Line((-17.5827, -127.69), (-19.8692, -127.157))
        Line((-19.8692, -127.157), (-22.0636, -126.553))
        Line((-22.0636, -126.553), (-24.1658, -125.878))
        Line((-24.1658, -125.878), (-26.1757, -125.132))
        Line((-26.1757, -125.132), (-28.0934, -124.314))
        Line((-28.0934, -124.314), (-29.9188, -123.426))
        Line((-29.9188, -123.426), (-31.6521, -122.467))
        Line((-31.6521, -122.467), (-33.2932, -121.436))
        Line((-33.2932, -121.436), (-34.8351, -120.34))
        Line((-34.8351, -120.34), (-36.2706, -119.184))
        Line((-36.2706, -119.184), (-37.5998, -117.967))
        Line((-37.5998, -117.967), (-38.8226, -116.691))
        Line((-38.8226, -116.691), (-39.9391, -115.354))
        Line((-39.9391, -115.354), (-40.9494, -113.956))
        Line((-40.9494, -113.956), (-41.8532, -112.499))
        Line((-41.8532, -112.499), (-42.6508, -110.981))
        Line((-42.6508, -110.981), (-43.342, -109.404))
        Line((-43.342, -109.404), (-43.9268, -107.766))
        Line((-43.9268, -107.766), (-44.4052, -106.067))
        Line((-44.4052, -106.067), (-44.7775, -104.309))
        Line((-44.7775, -104.309), (-45.0433, -102.49))
        Line((-45.0433, -102.49), (-45.2028, -100.611))
        Line((-45.2028, -100.611), (-45.256, -98.6716))
        Line((-45.256, -98.6716), (-45.1781, -96.1398))
        Line((-45.1781, -96.1398), (-44.9446, -93.736))
        Line((-44.9446, -93.736), (-44.5554, -91.4603))
        Line((-44.5554, -91.4603), (-44.0105, -89.3124))
        Line((-44.0105, -89.3124), (-43.3098, -87.2928))
        Line((-43.3098, -87.2928), (-42.4535, -85.401))
        Line((-42.4535, -85.401), (-41.4415, -83.6374))
        Line((-41.4415, -83.6374), (-40.2737, -82.0016))
        Line((-40.2737, -82.0016), (-38.9503, -80.4941))
        Line((-38.9503, -80.4941), (-37.4712, -79.1144))
        Line((-37.4712, -79.1144), (-35.8363, -77.8629))
        Line((-35.8363, -77.8629), (-34.0459, -76.7393))
        Line((-34.0459, -76.7393), (-32.0996, -75.7437))
        Line((-32.0996, -75.7437), (-29.9977, -74.8763))
        Line((-29.9977, -74.8763), (-27.7402, -74.1368))
        Line((-27.7402, -74.1368), (-25.8484, -84.6344))
        Line((-25.8484, -84.6344), (-27.2012, -85.0998))
        Line((-27.2012, -85.0998), (-28.4608, -85.6335))
        Line((-28.4608, -85.6335), (-29.6272, -86.2358))
        Line((-29.6272, -86.2358), (-30.7002, -86.9063))
        Line((-30.7002, -86.9063), (-31.6798, -87.6453))
        Line((-31.6798, -87.6453), (-32.5662, -88.4525))
        Line((-32.5662, -88.4525), (-33.3594, -89.3282))
        Line((-33.3594, -89.3282), (-34.0591, -90.2721))
        Line((-34.0591, -90.2721), (-34.6655, -91.2845))
        Line((-34.6655, -91.2845), (-35.1788, -92.3651))
        Line((-35.1788, -92.3651), (-35.5986, -93.5143))
        Line((-35.5986, -93.5143), (-35.9251, -94.7316))
        Line((-35.9251, -94.7316), (-36.1584, -96.0175))
        Line((-36.1584, -96.0175), (-36.2984, -97.3715))
        Line((-36.2984, -97.3715), (-36.3451, -98.7941))
        Line((-36.3451, -98.7941), (-36.3062, -100.028))
        Line((-36.3062, -100.028), (-36.1894, -101.225))
        Line((-36.1894, -101.225), (-35.9947, -102.384))
        Line((-35.9947, -102.384), (-35.7224, -103.505))
        Line((-35.7224, -103.505), (-35.372, -104.588))
        Line((-35.372, -104.588), (-34.9438, -105.634))
        Line((-34.9438, -105.634), (-34.4379, -106.642))
        Line((-34.4379, -106.642), (-33.8539, -107.613))
        Line((-33.8539, -107.613), (-33.1923, -108.546))
        Line((-33.1923, -108.546), (-32.4527, -109.44))
        Line((-32.4527, -109.44), (-31.6353, -110.298))
        Line((-31.6353, -110.298), (-30.7401, -111.117))
        Line((-30.7401, -111.117), (-29.7668, -111.899))
        Line((-29.7668, -111.899), (-28.716, -112.643))
        Line((-28.716, -112.643), (-27.5871, -113.35))
        Line((-27.5871, -113.35), (-26.3846, -114.014))
        Line((-26.3846, -114.014), (-25.1123, -114.633))
        Line((-25.1123, -114.633), (-23.7703, -115.206))
        Line((-23.7703, -115.206), (-22.3586, -115.733))
        Line((-22.3586, -115.733), (-20.8772, -116.215))
        Line((-20.8772, -116.215), (-19.326, -116.65))
        Line((-19.326, -116.65), (-17.7052, -117.04))
        Line((-17.7052, -117.04), (-16.0146, -117.384))
        Line((-16.0146, -117.384), (-14.2543, -117.682))
        Line((-14.2543, -117.682), (-12.4243, -117.934))
        Line((-12.4243, -117.934), (-10.5247, -118.14))
        Line((-10.5247, -118.14), (-8.5553, -118.3))
        Line((-8.5553, -118.3), (-6.5161, -118.415))
        Line((-6.5161, -118.415), (-4.4074, -118.484))
        Line((-4.4074, -118.484), (-2.2287, -118.507))
        RadiusArc((-2.2287, -118.507), (-13.5805, -97.634), 22.3197)
        Line((-13.5805, -97.634), (-13.5474, -96.069))
        Line((-13.5474, -96.069), (-13.4482, -94.5477))
        Line((-13.4482, -94.5477), (-13.2826, -93.0702))
        Line((-13.2826, -93.0702), (-13.051, -91.6362))
        Line((-13.051, -91.6362), (-12.7531, -90.246))
        Line((-12.7531, -90.246), (-12.3892, -88.8995))
        Line((-12.3892, -88.8995), (-11.9589, -87.5966))
        Line((-11.9589, -87.5966), (-11.4626, -86.3374))
        Line((-11.4626, -86.3374), (-10.9, -85.1218))
        Line((-10.9, -85.1218), (-10.2713, -83.9499))
        Line((-10.2713, -83.9499), (-9.5764, -82.8217))
        Line((-9.5764, -82.8217), (-8.8152, -81.7372))
        Line((-8.8152, -81.7372), (-7.988, -80.6964))
        Line((-7.988, -80.6964), (-7.0944, -79.6991))
        Line((-7.0944, -79.6991), (-6.1348, -78.7456))
        Line((-6.1348, -78.7456), (-5.1192, -77.8448))
        Line((-5.1192, -77.8448), (-4.0581, -77.0062))
        Line((-4.0581, -77.0062), (-2.9515, -76.2297))
        Line((-2.9515, -76.2297), (-1.7992, -75.5154))
        Line((-1.7992, -75.5154), (-0.6013, -74.8631))
        Line((-0.6013, -74.8631), (0.6418, -74.2729))
        Line((0.6418, -74.2729), (1.9308, -73.745))
        Line((1.9308, -73.745), (3.2654, -73.2791))
        Line((3.2654, -73.2791), (4.6454, -72.8754))
        Line((4.6454, -72.8754), (6.071, -72.5336))
        Line((6.071, -72.5336), (7.5423, -72.254))
        Line((7.5423, -72.254), (9.0591, -72.0367))
        Line((9.0591, -72.0367), (10.6215, -71.8814))
        Line((10.6215, -71.8814), (12.2293, -71.7882))
        Line((12.2293, -71.7882), (13.8829, -71.7572))
        Line((13.8829, -71.7572), (15.6721, -71.79))
        Line((15.6721, -71.79), (17.4104, -71.8884))
        Line((17.4104, -71.8884), (19.0976, -72.0525))
        Line((19.0976, -72.0525), (20.7339, -72.2823))
        Line((20.7339, -72.2823), (22.3192, -72.5777))
        Line((22.3192, -72.5777), (23.8535, -72.9387))
        Line((23.8535, -72.9387), (25.3368, -73.3653))
        Line((25.3368, -73.3653), (26.769, -73.8576))
        Line((26.769, -73.8576), (28.1502, -74.4156))
        Line((28.1502, -74.4156), (29.4804, -75.0391))
        Line((29.4804, -75.0391), (30.7597, -75.7283))
        Line((30.7597, -75.7283), (31.9881, -76.4832))
        Line((31.9881, -76.4832), (33.1653, -77.3038))
        Line((33.1653, -77.3038), (34.2915, -78.1898))
        Line((34.2915, -78.1898), (35.3668, -79.1417))
        Line((35.3668, -79.1417), (36.3814, -80.1512))
        Line((36.3814, -80.1512), (37.326, -81.2106))
        Line((37.326, -81.2106), (38.2008, -82.3201))
        Line((38.2008, -82.3201), (39.0056, -83.4793))
        Line((39.0056, -83.4793), (39.7403, -84.6886))
        Line((39.7403, -84.6886), (40.4051, -85.9477))
        Line((40.4051, -85.9477), (40.9999, -87.2566))
        Line((40.9999, -87.2566), (41.5247, -88.6156))
        Line((41.5247, -88.6156), (41.9795, -90.0244))
        Line((41.9795, -90.0244), (42.3643, -91.4833))
        Line((42.3643, -91.4833), (42.6793, -92.9919))
        Line((42.6793, -92.9919), (42.9242, -94.5505))
    _inc_edges_sk_Sketch3_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_2 = Wire.combine(_inc_edges_sk_Sketch3_2)[0]
_wire_sk_Sketch3_2 = _wire_sk_Sketch3_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch3_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch3_2.wrapped, True)
_face_sk_Sketch3_2 = Face(_mkf_sk_Sketch3_2.Face())

# 'Sketch4': 155 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        Line((42.9266, -28.6525), (43.1001, -30.4259))
        Line((43.1001, -30.4259), (43.2043, -32.2548))
        Line((43.2043, -32.2548), (43.2391, -34.1393))
        Line((43.2391, -34.1393), (43.2158, -35.7217))
        Line((43.2158, -35.7217), (43.1458, -37.263))
        Line((43.1458, -37.263), (43.0292, -38.763))
        Line((43.0292, -38.763), (42.8658, -40.2217))
        Line((42.8658, -40.2217), (42.6559, -41.6393))
        Line((42.6559, -41.6393), (42.3993, -43.0156))
        Line((42.3993, -43.0156), (42.0959, -44.3506))
        Line((42.0959, -44.3506), (41.7461, -45.6445))
        Line((41.7461, -45.6445), (41.3495, -46.8971))
        Line((41.3495, -46.8971), (40.9062, -48.1085))
        Line((40.9062, -48.1085), (40.4164, -49.2786))
        Line((40.4164, -49.2786), (39.8799, -50.4076))
        Line((39.8799, -50.4076), (39.2966, -51.4952))
        Line((39.2966, -51.4952), (38.6668, -52.5417))
        Line((38.6668, -52.5417), (37.9903, -53.5469))
        Line((37.9903, -53.5469), (37.2696, -54.5073))
        Line((37.2696, -54.5073), (36.5071, -55.4195))
        Line((36.5071, -55.4195), (35.7028, -56.2834))
        Line((35.7028, -56.2834), (34.8567, -57.099))
        Line((34.8567, -57.099), (33.969, -57.8664))
        Line((33.969, -57.8664), (33.0394, -58.5855))
        Line((33.0394, -58.5855), (32.068, -59.2563))
        Line((32.068, -59.2563), (31.055, -59.8787))
        Line((31.055, -59.8787), (30.0002, -60.4529))
        Line((30.0002, -60.4529), (28.9035, -60.9789))
        Line((28.9035, -60.9789), (27.765, -61.4566))
        Line((27.765, -61.4566), (26.5849, -61.886))
        Line((26.5849, -61.886), (25.363, -62.267))
        Line((25.363, -62.267), (24.0993, -62.5998))
        Line((24.0993, -62.5998), (22.7937, -62.8844))
        Line((22.7937, -62.8844), (21.5121, -51.7766))
        Line((21.5121, -51.7766), (23.1563, -51.2642))
        Line((23.1563, -51.2642), (24.6869, -50.6548))
        Line((24.6869, -50.6548), (26.1043, -49.9481))
        Line((26.1043, -49.9481), (27.4081, -49.1444))
        Line((27.4081, -49.1444), (28.5986, -48.2437))
        Line((28.5986, -48.2437), (29.6758, -47.2458))
        Line((29.6758, -47.2458), (30.6396, -46.1507))
        Line((30.6396, -46.1507), (31.49, -44.9586))
        Line((31.49, -44.9586), (32.227, -43.6693))
        Line((32.227, -43.6693), (32.8506, -42.283))
        Line((32.8506, -42.283), (33.3607, -40.7996))
        Line((33.3607, -40.7996), (33.7576, -39.2189))
        Line((33.7576, -39.2189), (34.0411, -37.5412))
        Line((34.0411, -37.5412), (34.2111, -35.7664))
        Line((34.2111, -35.7664), (34.2679, -33.8945))
        Line((34.2679, -33.8945), (34.2441, -32.7318))
        Line((34.2441, -32.7318), (34.173, -31.6031))
        Line((34.173, -31.6031), (34.0543, -30.5087))
        Line((34.0543, -30.5087), (33.8881, -29.4484))
        Line((33.8881, -29.4484), (33.6746, -28.4224))
        Line((33.6746, -28.4224), (33.4135, -27.4304))
        Line((33.4135, -27.4304), (33.105, -26.4728))
        RadiusArc((33.105, -26.4728), (24.9236, -17.2603), -16.1994)
        Line((24.9236, -17.2603), (24.0161, -16.8376))
        Line((24.0161, -16.8376), (23.073, -16.4595))
        Line((23.073, -16.4595), (22.0944, -16.1258))
        Line((22.0944, -16.1258), (21.0802, -15.8366))
        Line((21.0802, -15.8366), (20.0305, -15.592))
        Line((20.0305, -15.592), (18.9453, -15.3918))
        Line((18.9453, -15.3918), (17.8246, -15.2361))
        Line((17.8246, -15.2361), (16.6684, -15.1248))
        Line((16.6684, -15.1248), (15.4765, -15.0581))
        Line((15.4765, -15.0581), (14.2491, -15.0359))
        Line((14.2491, -15.0359), (13.1807, -15.0583))
        Line((13.1807, -15.0583), (12.1399, -15.1254))
        Line((12.1399, -15.1254), (11.1269, -15.2373))
        RadiusArc((11.1269, -15.2373), (-3.3206, -28.3441), -16.558)
        Line((-3.3206, -28.3441), (-3.5208, -29.3431))
        Line((-3.5208, -29.3431), (-3.6764, -30.3731))
        Line((-3.6764, -30.3731), (-3.7877, -31.4345))
        Line((-3.7877, -31.4345), (-3.8544, -32.5269))
        Line((-3.8544, -32.5269), (-3.8767, -33.6505))
        RadiusArc((-3.8767, -33.6505), (2.2873, -49.6408), -22.567)
        Line((2.2873, -49.6408), (2.2873, -60.3824))
        Line((2.2873, -60.3824), (-43.9743, -57.514))
        Line((-43.9743, -57.514), (-43.9743, -8.6279))
        Line((-43.9743, -8.6279), (-34.6368, -8.6279))
        Line((-34.6368, -8.6279), (-34.6368, -47.5052))
        Line((-34.6368, -47.5052), (-7.3552, -49.1531))
        Line((-7.3552, -49.1531), (-8.063, -48.1854))
        Line((-8.063, -48.1854), (-8.7222, -47.1869))
        Line((-8.7222, -47.1869), (-9.3326, -46.1574))
        Line((-9.3326, -46.1574), (-9.894, -45.097))
        Line((-9.894, -45.097), (-10.4066, -44.0057))
        Line((-10.4066, -44.0057), (-10.8705, -42.8835))
        Line((-10.8705, -42.8835), (-11.2856, -41.7303))
        Line((-11.2856, -41.7303), (-11.6516, -40.5463))
        Line((-11.6516, -40.5463), (-11.969, -39.3314))
        Line((-11.969, -39.3314), (-12.2375, -38.0855))
        Line((-12.2375, -38.0855), (-12.4573, -36.8088))
        Line((-12.4573, -36.8088), (-12.6282, -35.5009))
        Line((-12.6282, -35.5009), (-12.7502, -34.1624))
        Line((-12.7502, -34.1624), (-12.8235, -32.793))
        Line((-12.8235, -32.793), (-12.8479, -31.3925))
        Line((-12.8479, -31.3925), (-12.8148, -29.7227))
        Line((-12.8148, -29.7227), (-12.7155, -28.0989))
        Line((-12.7155, -28.0989), (-12.55, -26.521))
        Line((-12.55, -26.521), (-12.3184, -24.9889))
        Line((-12.3184, -24.9889), (-12.0206, -23.5025))
        Line((-12.0206, -23.5025), (-11.6565, -22.0621))
        Line((-11.6565, -22.0621), (-11.2263, -20.6674))
        Line((-11.2263, -20.6674), (-10.73, -19.3185))
        Line((-10.73, -19.3185), (-10.1674, -18.0156))
        Line((-10.1674, -18.0156), (-9.5387, -16.7584))
        Line((-9.5387, -16.7584), (-8.8437, -15.5472))
        Line((-8.8437, -15.5472), (-8.0826, -14.3817))
        Line((-8.0826, -14.3817), (-7.2554, -13.2622))
        Line((-7.2554, -13.2622), (-6.3618, -12.1884))
        Line((-6.3618, -12.1884), (-5.4022, -11.1604))
        Line((-5.4022, -11.1604), (-4.3893, -10.1889))
        Line((-4.3893, -10.1889), (-3.3363, -9.2845))
        Line((-3.3363, -9.2845), (-2.2432, -8.447))
        Line((-2.2432, -8.447), (-1.1099, -7.6765))
        Line((-1.1099, -7.6765), (0.0633, -6.973))
        Line((0.0633, -6.973), (1.2769, -6.3365))
        Line((1.2769, -6.3365), (2.5305, -5.7669))
        Line((2.5305, -5.7669), (3.8243, -5.2644))
        Line((3.8243, -5.2644), (5.1584, -4.8289))
        Line((5.1584, -4.8289), (6.5324, -4.4605))
        Line((6.5324, -4.4605), (7.9468, -4.1589))
        Line((7.9468, -4.1589), (9.4011, -3.9244))
        Line((9.4011, -3.9244), (10.8957, -3.757))
        Line((10.8957, -3.757), (12.4304, -3.6565))
        Line((12.4304, -3.6565), (14.0053, -3.623))
        Line((14.0053, -3.623), (15.7942, -3.6589))
        Line((15.7942, -3.6589), (17.5314, -3.7668))
        Line((17.5314, -3.7668), (19.2172, -3.9465))
        Line((19.2172, -3.9465), (20.8516, -4.1982))
        Line((20.8516, -4.1982), (22.4342, -4.5216))
        Line((22.4342, -4.5216), (23.9655, -4.917))
        Line((23.9655, -4.917), (25.4451, -5.3842))
        Line((25.4451, -5.3842), (26.8732, -5.9235))
        Line((26.8732, -5.9235), (28.2498, -6.5344))
        Line((28.2498, -6.5344), (29.5749, -7.2174))
        Line((29.5749, -7.2174), (30.8484, -7.9723))
        Line((30.8484, -7.9723), (32.0703, -8.799))
        Line((32.0703, -8.799), (33.2408, -9.6976))
        Line((33.2408, -9.6976), (34.3597, -10.668))
        Line((34.3597, -10.668), (35.4271, -11.7104))
        Line((35.4271, -11.7104), (36.434, -12.8163))
        Line((36.434, -12.8163), (37.3714, -13.978))
        Line((37.3714, -13.978), (38.2394, -15.1952))
        Line((38.2394, -15.1952), (39.0379, -16.468))
        Line((39.0379, -16.468), (39.767, -17.7965))
        Line((39.767, -17.7965), (40.4268, -19.1805))
        Line((40.4268, -19.1805), (41.017, -20.6201))
        Line((41.017, -20.6201), (41.5378, -22.1155))
        Line((41.5378, -22.1155), (41.9891, -23.6662))
        Line((41.9891, -23.6662), (42.3711, -25.2728))
        Line((42.3711, -25.2728), (42.6836, -26.9348))
        Line((42.6836, -26.9348), (42.9266, -28.6525))
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch5': 49 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch5_4:
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
    _vec = Vector(0.0, 0.0, -1.0) * -90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.001, -325.0001, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.249, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch3_2
    _vec = Vector(0.0, 0.0, 1.0) * 50.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 50.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * 70.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 70.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch5_4
    _vec = Vector(0.0, 0.0, 1.0) * 5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
