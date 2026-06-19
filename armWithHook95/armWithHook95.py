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
        Line((6.0, 389.0), (-90.0, 389.0))
        Line((-90.0, 389.0), (-90.0, 355.0))
        RadiusArc((-90.0, 355.0), (-60.0, 325.102), -29.8458)
        Line((-60.0, 325.102), (-60.0, -475.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -475.0), (0.0, -535.0003), -60.0)
        RadiusArc((0.0, -535.0003), (60.0, -475.0), -60.0)
        Line((60.0, -475.0), (60.0, 325.102))
        RadiusArc((60.0, 325.102), (90.0, 355.0), -29.8458)
        Line((90.0, 355.0), (90.0, 475.0))
        Line((90.0, 475.0), (-90.0, 475.0))
        Line((-90.0, 475.0), (-90.0, 441.0))
        Line((-90.0, 441.0), (6.0, 441.0))
        Line((6.0, 441.0), (6.0, 439.966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 439.966), (39.0002, 415.0), 25.944)
        RadiusArc((39.0002, 415.0), (6.0, 390.034), 25.944)
        Line((6.0, 390.034), (6.0, 389.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 155 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
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
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 197 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((42.7631, -97.6039), (43.0275, -99.4449))
        Line((43.0275, -99.4449), (43.1862, -101.347))
        Line((43.1862, -101.347), (43.2391, -103.31))
        Line((43.2391, -103.31), (43.2202, -104.635))
        Line((43.2202, -104.635), (43.1636, -105.925))
        Line((43.1636, -105.925), (43.0695, -107.178))
        Line((43.0695, -107.178), (42.9375, -108.397))
        Line((42.9375, -108.397), (42.7678, -109.579))
        Line((42.7678, -109.579), (42.5604, -110.727))
        Line((42.5604, -110.727), (42.3154, -111.838))
        Line((42.3154, -111.838), (42.0325, -112.915))
        Line((42.0325, -112.915), (41.712, -113.955))
        Line((41.712, -113.955), (41.3539, -114.96))
        Line((41.3539, -114.96), (40.9579, -115.93))
        Line((40.9579, -115.93), (40.5244, -116.864))
        Line((40.5244, -116.864), (40.0531, -117.762))
        Line((40.0531, -117.762), (39.5441, -118.625))
        Line((39.5441, -118.625), (38.9973, -119.453))
        Line((38.9973, -119.453), (38.4087, -120.247))
        Line((38.4087, -120.247), (37.7734, -121.011))
        Line((37.7734, -121.011), (37.092, -121.744))
        Line((37.092, -121.744), (36.3641, -122.446))
        Line((36.3641, -122.446), (35.5898, -123.118))
        Line((35.5898, -123.118), (34.7691, -123.759))
        Line((34.7691, -123.759), (33.902, -124.369))
        Line((33.902, -124.369), (32.9886, -124.949))
        Line((32.9886, -124.949), (32.0288, -125.498))
        Line((32.0288, -125.498), (31.0226, -126.017))
        Line((31.0226, -126.017), (29.9699, -126.505))
        Line((29.9699, -126.505), (28.871, -126.962))
        Line((28.871, -126.962), (27.7257, -127.388))
        Line((27.7257, -127.388), (26.534, -127.784))
        Line((26.534, -127.784), (25.2957, -128.149))
        Line((25.2957, -128.149), (23.6479, -117.653))
        Line((23.6479, -117.653), (25.0323, -117.178))
        Line((25.0323, -117.178), (26.3213, -116.633))
        Line((26.3213, -116.633), (27.5148, -116.017))
        Line((27.5148, -116.017), (28.6128, -115.331))
        Line((28.6128, -115.331), (29.6153, -114.574))
        Line((29.6153, -114.574), (30.5225, -113.746))
        Line((30.5225, -113.746), (31.3341, -112.849))
        Line((31.3341, -112.849), (32.0502, -111.88))
        Line((32.0502, -111.88), (32.6707, -110.841))
        Line((32.6707, -110.841), (33.1958, -109.732))
        Line((33.1958, -109.732), (33.6255, -108.552))
        Line((33.6255, -108.552), (33.9597, -107.301))
        Line((33.9597, -107.301), (34.1985, -105.98))
        Line((34.1985, -105.98), (34.3416, -104.589))
        Line((34.3416, -104.589), (34.3893, -103.127))
        Line((34.3893, -103.127), (34.3503, -101.884))
        Line((34.3503, -101.884), (34.2331, -100.68))
        Line((34.2331, -100.68), (34.0378, -99.5135))
        Line((34.0378, -99.5135), (33.7643, -98.3852))
        Line((33.7643, -98.3852), (33.4129, -97.2948))
        Line((33.4129, -97.2948), (32.9832, -96.2422))
        Line((32.9832, -96.2422), (32.4754, -95.2278))
        Line((32.4754, -95.2278), (31.8895, -94.2513))
        Line((31.8895, -94.2513), (31.2256, -93.3128))
        Line((31.2256, -93.3128), (30.4834, -92.4123))
        Line((30.4834, -92.4123), (29.6631, -91.5497))
        Line((29.6631, -91.5497), (28.7648, -90.7251))
        Line((28.7648, -90.7251), (27.7882, -89.9385))
        Line((27.7882, -89.9385), (26.7336, -89.1898))
        Line((26.7336, -89.1898), (25.6009, -88.4792))
        Line((25.6009, -88.4792), (24.3958, -87.8094))
        Line((24.3958, -87.8094), (23.1238, -87.1837))
        Line((23.1238, -87.1837), (21.7851, -86.6019))
        Line((21.7851, -86.6019), (20.3798, -86.064))
        Line((20.3798, -86.064), (18.9078, -85.5701))
        Line((18.9078, -85.5701), (17.3689, -85.1201))
        Line((17.3689, -85.1201), (15.7634, -84.7141))
        Line((15.7634, -84.7141), (14.0912, -84.352))
        Line((14.0912, -84.352), (12.3521, -84.0338))
        Line((12.3521, -84.0338), (10.5464, -83.7595))
        Line((10.5464, -83.7595), (8.674, -83.5292))
        Line((8.674, -83.5292), (6.7349, -83.3429))
        Line((6.7349, -83.3429), (4.729, -83.2005))
        Line((4.729, -83.2005), (2.6564, -83.1021))
        Line((2.6564, -83.1021), (0.517, -83.0476))
        RadiusArc((0.517, -83.0476), (12.6625, -104.408), 23.1073)
        Line((12.6625, -104.408), (12.6273, -105.934))
        Line((12.6273, -105.934), (12.5214, -107.418))
        Line((12.5214, -107.418), (12.3451, -108.861))
        Line((12.3451, -108.861), (12.0984, -110.263))
        Line((12.0984, -110.263), (11.781, -111.624))
        Line((11.781, -111.624), (11.3931, -112.943))
        Line((11.3931, -112.943), (10.9348, -114.221))
        Line((10.9348, -114.221), (10.4059, -115.458))
        Line((10.4059, -115.458), (9.8064, -116.654))
        Line((9.8064, -116.654), (9.1365, -117.809))
        Line((9.1365, -117.809), (8.396, -118.922))
        Line((8.396, -118.922), (7.585, -119.994))
        Line((7.585, -119.994), (6.7035, -121.025))
        Line((6.7035, -121.025), (5.7515, -122.014))
        Line((5.7515, -122.014), (4.729, -122.963))
        Line((4.729, -122.963), (3.6481, -123.859))
        Line((3.6481, -123.859), (2.5209, -124.694))
        Line((2.5209, -124.694), (1.3477, -125.467))
        Line((1.3477, -125.467), (0.1283, -126.178))
        Line((0.1283, -126.178), (-1.1369, -126.828))
        Line((-1.1369, -126.828), (-2.4484, -127.415))
        Line((-2.4484, -127.415), (-3.806, -127.941))
        Line((-3.806, -127.941), (-5.2098, -128.405))
        Line((-5.2098, -128.405), (-6.6597, -128.806))
        Line((-6.6597, -128.806), (-8.1557, -129.147))
        Line((-8.1557, -129.147), (-9.6977, -129.425))
        Line((-9.6977, -129.425), (-11.286, -129.641))
        Line((-11.286, -129.641), (-12.9202, -129.796))
        Line((-12.9202, -129.796), (-14.6007, -129.889))
        Line((-14.6007, -129.889), (-16.3272, -129.92))
        Line((-16.3272, -129.92), (-18.1, -129.886))
        Line((-18.1, -129.886), (-19.8213, -129.785))
        Line((-19.8213, -129.785), (-21.4915, -129.617))
        Line((-21.4915, -129.617), (-23.1105, -129.381))
        Line((-23.1105, -129.381), (-24.678, -129.079))
        Line((-24.678, -129.079), (-26.1945, -128.709))
        Line((-26.1945, -128.709), (-27.6596, -128.271))
        Line((-27.6596, -128.271), (-29.0733, -127.767))
        Line((-29.0733, -127.767), (-30.4359, -127.195))
        Line((-30.4359, -127.195), (-31.7473, -126.556))
        Line((-31.7473, -126.556), (-33.0074, -125.85))
        Line((-33.0074, -125.85), (-34.2162, -125.076))
        Line((-34.2162, -125.076), (-35.3737, -124.235))
        Line((-35.3737, -124.235), (-36.4799, -123.327))
        Line((-36.4799, -123.327), (-37.5349, -122.352))
        Line((-37.5349, -122.352), (-38.53, -121.316))
        Line((-38.53, -121.316), (-39.4566, -120.228))
        Line((-39.4566, -120.228), (-40.3145, -119.088))
        Line((-40.3145, -119.088), (-41.1037, -117.894))
        Line((-41.1037, -117.894), (-41.8243, -116.648))
        Line((-41.8243, -116.648), (-42.4763, -115.35))
        Line((-42.4763, -115.35), (-43.0597, -113.999))
        Line((-43.0597, -113.999), (-43.5745, -112.595))
        Line((-43.5745, -112.595), (-44.0205, -111.139))
        Line((-44.0205, -111.139), (-44.398, -109.63))
        Line((-44.398, -109.63), (-44.7069, -108.068))
        Line((-44.7069, -108.068), (-44.9472, -106.454))
        Line((-44.9472, -106.454), (-45.1187, -104.787))
        Line((-45.1187, -104.787), (-45.2217, -103.068))
        Line((-45.2217, -103.068), (-45.256, -101.296))
        Line((-45.256, -101.296), (-45.2087, -99.4147))
        Line((-45.2087, -99.4147), (-45.0671, -97.5951))
        Line((-45.0671, -97.5951), (-44.8312, -95.8374))
        Line((-44.8312, -95.8374), (-44.5007, -94.1415))
        Line((-44.5007, -94.1415), (-44.0759, -92.5075))
        Line((-44.0759, -92.5075), (-43.5568, -90.9354))
        Line((-43.5568, -90.9354), (-42.9431, -89.425))
        Line((-42.9431, -89.425), (-42.2351, -87.9767))
        Line((-42.2351, -87.9767), (-41.4328, -86.59))
        Line((-41.4328, -86.59), (-40.536, -85.2652))
        Line((-40.536, -85.2652), (-39.5448, -84.0024))
        Line((-39.5448, -84.0024), (-38.4592, -82.8012))
        Line((-38.4592, -82.8012), (-37.2792, -81.662))
        Line((-37.2792, -81.662), (-36.0048, -80.5847))
        Line((-36.0048, -80.5847), (-34.6359, -79.5691))
        Line((-34.6359, -79.5691), (-33.1726, -78.6172))
        Line((-33.1726, -78.6172), (-31.6144, -77.731))
        Line((-31.6144, -77.731), (-29.9611, -76.9104))
        Line((-29.9611, -76.9104), (-28.2129, -76.1554))
        Line((-28.2129, -76.1554), (-26.3699, -75.4662))
        Line((-26.3699, -75.4662), (-24.4319, -74.8425))
        Line((-24.4319, -74.8425), (-22.3988, -74.2845))
        Line((-22.3988, -74.2845), (-20.271, -73.7921))
        Line((-20.271, -73.7921), (-18.0481, -73.3653))
        Line((-18.0481, -73.3653), (-15.7304, -73.0043))
        Line((-15.7304, -73.0043), (-13.3177, -72.7089))
        Line((-13.3177, -72.7089), (-10.8101, -72.4791))
        Line((-10.8101, -72.4791), (-8.2074, -72.3151))
        Line((-8.2074, -72.3151), (-5.5099, -72.2165))
        Line((-5.5099, -72.2165), (-2.7174, -72.1837))
        Line((-2.7174, -72.1837), (0.1907, -72.2197))
        Line((0.1907, -72.2197), (3.008, -72.3276))
        Line((3.008, -72.3276), (5.734, -72.5072))
        Line((5.734, -72.5072), (8.369, -72.7588))
        Line((8.369, -72.7588), (10.9128, -73.0823))
        Line((10.9128, -73.0823), (13.3655, -73.4778))
        Line((13.3655, -73.4778), (15.7269, -73.945))
        Line((15.7269, -73.945), (17.9974, -74.4841))
        Line((17.9974, -74.4841), (20.1767, -75.0952))
        Line((20.1767, -75.0952), (22.2647, -75.7782))
        Line((22.2647, -75.7782), (24.2618, -76.5329))
        Line((24.2618, -76.5329), (26.1676, -77.3596))
        Line((26.1676, -77.3596), (27.9823, -78.2582))
        Line((27.9823, -78.2582), (29.706, -79.2288))
        Line((29.706, -79.2288), (31.3383, -80.2711))
        Line((31.3383, -80.2711), (32.8723, -81.3799))
        Line((32.8723, -81.3799), (34.3004, -82.5496))
        Line((34.3004, -82.5496), (35.6226, -83.7804))
        Line((35.6226, -83.7804), (36.8391, -85.0722))
        Line((36.8391, -85.0722), (37.9498, -86.425))
        Line((37.9498, -86.425), (38.9548, -87.8389))
        Line((38.9548, -87.8389), (39.854, -89.3138))
        Line((39.854, -89.3138), (40.6474, -90.8498))
        Line((40.6474, -90.8498), (41.335, -92.4467))
        Line((41.335, -92.4467), (41.9168, -94.1048))
        Line((41.9168, -94.1048), (42.3927, -95.8238))
        Line((42.3927, -95.8238), (42.7631, -97.6039))
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch3': 46 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        Line((3.8992, -103.81), (3.9722, -102.811))
        Line((3.9722, -102.811), (3.9964, -101.784))
        RadiusArc((3.9964, -101.784), (-13.3366, -84.1466), -16.8735)
        Line((-13.3366, -84.1466), (-14.7018, -84.1676))
        Line((-14.7018, -84.1676), (-16.0312, -84.2311))
        Line((-16.0312, -84.2311), (-17.3242, -84.337))
        Line((-17.3242, -84.337), (-18.5814, -84.485))
        Line((-18.5814, -84.485), (-19.8024, -84.6754))
        Line((-19.8024, -84.6754), (-20.9872, -84.9081))
        Line((-20.9872, -84.9081), (-22.1361, -85.1831))
        Line((-22.1361, -85.1831), (-23.2489, -85.5005))
        Line((-23.2489, -85.5005), (-24.3256, -85.8601))
        Line((-24.3256, -85.8601), (-25.3662, -86.2622))
        Line((-25.3662, -86.2622), (-26.3707, -86.7064))
        Line((-26.3707, -86.7064), (-27.3392, -87.193))
        Line((-27.3392, -87.193), (-28.2715, -87.7219))
        Line((-28.2715, -87.7219), (-29.1678, -88.2932))
        Line((-29.1678, -88.2932), (-30.0279, -88.9067))
        Line((-30.0279, -88.9067), (-30.8421, -89.5557))
        Line((-30.8421, -89.5557), (-31.6002, -90.2335))
        RadiusArc((-31.6002, -90.2335), (-36.0924, -98.5934), -14.6715)
        Line((-36.0924, -98.5934), (-36.2328, -99.5877))
        Line((-36.2328, -99.5877), (-36.317, -100.611))
        Line((-36.317, -100.611), (-36.3451, -101.662))
        Line((-36.3451, -101.662), (-36.3211, -102.705))
        Line((-36.3211, -102.705), (-36.2491, -103.718))
        RadiusArc((-36.2491, -103.718), (-26.0123, -117.103), -15.404)
        Line((-26.0123, -117.103), (-25.0717, -117.449))
        Line((-25.0717, -117.449), (-24.0974, -117.754))
        Line((-24.0974, -117.754), (-23.0891, -118.018))
        Line((-23.0891, -118.018), (-22.0468, -118.242))
        Line((-22.0468, -118.242), (-20.9708, -118.425))
        Line((-20.9708, -118.425), (-19.8607, -118.568))
        Line((-19.8607, -118.568), (-18.7169, -118.669))
        Line((-18.7169, -118.669), (-17.5391, -118.73))
        Line((-17.5391, -118.73), (-16.3272, -118.751))
        Line((-16.3272, -118.751), (-15.0916, -118.73))
        Line((-15.0916, -118.73), (-13.891, -118.669))
        Line((-13.891, -118.669), (-12.7252, -118.568))
        Line((-12.7252, -118.568), (-11.5945, -118.425))
        Line((-11.5945, -118.425), (-10.4988, -118.242))
        Line((-10.4988, -118.242), (-9.438, -118.018))
        Line((-9.438, -118.018), (-8.4123, -117.754))
        Line((-8.4123, -117.754), (-7.4214, -117.449))
        Line((-7.4214, -117.449), (-6.4658, -117.103))
        RadiusArc((-6.4658, -117.103), (3.8992, -103.81), -15.4228)
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
    _bore_ax = _gAx2(_gPnt(-0.0078, -474.9971, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.2582, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_4
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
