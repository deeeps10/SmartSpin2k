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
        Line((6.0, 589.0), (-90.0, 589.0))
        Line((-90.0, 589.0), (-90.0, 555.0))
        RadiusArc((-90.0, 555.0), (-60.0, 525.102), -29.8188)
        Line((-60.0, 525.102), (-60.0, -675.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -675.0), (-0.0, -735.0003), -60.0)
        RadiusArc((-0.0, -735.0003), (60.0, -675.0), -60.0)
        Line((60.0, -675.0), (60.0, 525.102))
        RadiusArc((60.0, 525.102), (90.0, 555.0), -29.8188)
        Line((90.0, 555.0), (90.0, 675.0))
        Line((90.0, 675.0), (-90.0, 675.0))
        Line((-90.0, 675.0), (-90.0, 641.0))
        Line((-90.0, 641.0), (6.0, 641.0))
        Line((6.0, 641.0), (6.0, 639.966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 639.966), (39.0002, 615.0), 25.944)
        RadiusArc((39.0002, 615.0), (6.0, 590.034), 25.944)
        Line((6.0, 590.034), (6.0, 589.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 44 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((22.7937, -62.8844), (28.9035, -60.9789), -28.9474)
        RadiusArc((28.9035, -60.9789), (34.8567, -57.099), -22.698)
        RadiusArc((34.8567, -57.099), (39.2966, -51.4952), -22.4525)
        RadiusArc((39.2966, -51.4952), (42.0959, -44.3506), -27.1964)
        RadiusArc((42.0959, -44.3506), (43.2391, -34.1393), -39.8934)
        RadiusArc((43.2391, -34.1393), (41.9891, -23.6662), -43.3029)
        RadiusArc((41.9891, -23.6662), (38.2394, -15.1952), -28.7939)
        RadiusArc((38.2394, -15.1952), (28.2498, -6.5344), -25.0665)
        RadiusArc((28.2498, -6.5344), (19.2172, -3.9465), -28.6021)
        RadiusArc((19.2172, -3.9465), (9.4011, -3.9244), -39.1768)
        RadiusArc((9.4011, -3.9244), (0.0633, -6.973), -26.9936)
        RadiusArc((0.0633, -6.973), (-8.0826, -14.3817), -24.6066)
        RadiusArc((-8.0826, -14.3817), (-12.3184, -24.9889), -25.5001)
        RadiusArc((-12.3184, -24.9889), (-12.7502, -34.1624), -37.175)
        RadiusArc((-12.7502, -34.1624), (-11.2856, -41.7303), -31.8299)
        RadiusArc((-11.2856, -41.7303), (-7.3552, -49.1531), -25.4914)
        Line((-7.3552, -49.1531), (-34.6368, -47.5052))
        Line((-34.6368, -47.5052), (-34.6368, -8.6279))
        Line((-34.6368, -8.6279), (-43.9743, -8.6279))
        Line((-43.9743, -8.6279), (-43.9743, -57.514))
        Line((-43.9743, -57.514), (2.2873, -60.3824))
        Line((2.2873, -60.3824), (2.2873, -49.6408))
        RadiusArc((2.2873, -49.6408), (-0.0856, -46.5651), 29.5488)
        RadiusArc((-0.0856, -46.5651), (-2.0638, -42.9765), 21.7471)
        RadiusArc((-2.0638, -42.9765), (-3.1216, -39.8569), 21.7316)
        RadiusArc((-3.1216, -39.8569), (-3.7769, -35.9833), 24.2513)
        RadiusArc((-3.7769, -35.9833), (-3.8767, -33.6505), 27.1124)
        RadiusArc((-3.8767, -33.6505), (-3.3206, -28.3441), 23.1793)
        RadiusArc((-3.3206, -28.3441), (-1.6522, -23.8177), 17.4478)
        RadiusArc((-1.6522, -23.8177), (1.1281, -20.0714), 16.1627)
        RadiusArc((1.1281, -20.0714), (5.6294, -16.8486), 16.2692)
        RadiusArc((5.6294, -16.8486), (11.1269, -15.2373), 18.6517)
        RadiusArc((11.1269, -15.2373), (16.6684, -15.1248), 28.592)
        RadiusArc((16.6684, -15.1248), (21.0802, -15.8366), 24.6267)
        RadiusArc((21.0802, -15.8366), (24.9236, -17.2603), 19.1338)
        RadiusArc((24.9236, -17.2603), (28.9282, -20.0407), 16.7704)
        RadiusArc((28.9282, -20.0407), (31.8947, -23.8045), 15.7875)
        RadiusArc((31.8947, -23.8045), (33.4135, -27.4304), 17.2539)
        RadiusArc((33.4135, -27.4304), (34.2441, -32.7318), 22.9581)
        RadiusArc((34.2441, -32.7318), (33.7576, -39.2189), 26.0446)
        RadiusArc((33.7576, -39.2189), (31.49, -44.9586), 16.5064)
        RadiusArc((31.49, -44.9586), (27.4081, -49.1444), 14.6165)
        RadiusArc((27.4081, -49.1444), (21.5121, -51.7766), 20.2144)
        Line((21.5121, -51.7766), (22.7937, -62.8844))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 66 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        RadiusArc((28.2498, -128.836), (33.8069, -125.473), -23.4054)
        RadiusArc((33.8069, -125.473), (38.8153, -119.603), -22.2578)
        RadiusArc((38.8153, -119.603), (41.5639, -113.36), -27.3344)
        RadiusArc((41.5639, -113.36), (43.0035, -105.943), -36.6456)
        RadiusArc((43.0035, -105.943), (43.21, -99.0869), -52.0817)
        RadiusArc((43.21, -99.0869), (42.7747, -93.9032), -48.6882)
        RadiusArc((42.7747, -93.9032), (40.8881, -86.4142), -35.4293)
        RadiusArc((40.8881, -86.4142), (36.7087, -79.3253), -24.6911)
        RadiusArc((36.7087, -79.3253), (30.6252, -74.4817), -20.7991)
        RadiusArc((30.6252, -74.4817), (22.823, -72.0598), -23.019)
        RadiusArc((22.823, -72.0598), (15.0941, -71.972), -30.5209)
        RadiusArc((15.0941, -71.972), (9.4011, -73.6908), -19.1028)
        RadiusArc((9.4011, -73.6908), (5.3433, -76.436), -17.872)
        RadiusArc((5.3433, -76.436), (2.6097, -79.4473), -19.3033)
        RadiusArc((2.6097, -79.4473), (0.4941, -83.071), -20.6289)
        RadiusArc((0.4941, -83.071), (-0.9966, -87.3024), -24.9488)
        RadiusArc((-0.9966, -87.3024), (-1.8625, -92.1413), -32.6349)
        Line((-1.8625, -92.1413), (-2.1063, -92.1413))
        RadiusArc((-2.1063, -92.1413), (-3.3197, -87.7513), -34.6769)
        RadiusArc((-3.3197, -87.7513), (-5.5014, -83.1), -25.649)
        RadiusArc((-5.5014, -83.1), (-9.7961, -78.1508), -19.3978)
        RadiusArc((-9.7961, -78.1508), (-17.3409, -74.6487), -17.4413)
        RadiusArc((-17.3409, -74.6487), (-25.2716, -74.2642), -25.2335)
        RadiusArc((-25.2716, -74.2642), (-33.6783, -76.7183), -22.7125)
        RadiusArc((-33.6783, -76.7183), (-40.7404, -83.3228), -19.3528)
        RadiusArc((-40.7404, -83.3228), (-43.5458, -89.2572), -24.058)
        RadiusArc((-43.5458, -89.2572), (-45.0156, -96.4734), -33.3667)
        RadiusArc((-45.0156, -96.4734), (-45.0192, -105.982), -47.7113)
        RadiusArc((-45.0192, -105.982), (-43.1247, -114.077), -33.3356)
        RadiusArc((-43.1247, -114.077), (-38.5258, -121.772), -25.9368)
        RadiusArc((-38.5258, -121.772), (-32.8003, -126.455), -22.1892)
        RadiusArc((-32.8003, -126.455), (-22.8577, -129.553), -24.7877)
        Line((-22.8577, -129.553), (-22.0027, -118.507))
        RadiusArc((-22.0027, -118.507), (-27.6614, -117.006), 17.1499)
        RadiusArc((-27.6614, -117.006), (-31.9904, -113.87), 13.6788)
        RadiusArc((-31.9904, -113.87), (-34.5688, -109.989), 14.826)
        RadiusArc((-34.5688, -109.989), (-36.0747, -104.334), 19.2172)
        RadiusArc((-36.0747, -104.334), (-36.1525, -99.478), 28.0945)
        RadiusArc((-36.1525, -99.478), (-35.3529, -94.9355), 19.8224)
        RadiusArc((-35.3529, -94.9355), (-33.2208, -90.6393), 13.1174)
        RadiusArc((-33.2208, -90.6393), (-28.5637, -86.8893), 12.7074)
        RadiusArc((-28.5637, -86.8893), (-24.8033, -85.7167), 15.2105)
        RadiusArc((-24.8033, -85.7167), (-22.238, -85.4462), 19.4289)
        RadiusArc((-22.238, -85.4462), (-16.2489, -86.2241), 17.326)
        RadiusArc((-16.2489, -86.2241), (-11.5411, -89.1644), 12.3646)
        RadiusArc((-11.5411, -89.1644), (-8.6182, -93.3435), 14.5784)
        RadiusArc((-8.6182, -93.3435), (-6.9385, -98.8376), 19.9129)
        RadiusArc((-6.9385, -98.8376), (-6.501, -104.408), 31.7038)
        Line((-6.501, -104.408), (-6.501, -110.39))
        Line((-6.501, -110.39), (3.0199, -110.39))
        Line((3.0199, -110.39), (3.0199, -104.164))
        RadiusArc((3.0199, -104.164), (3.4573, -97.9018), 43.6689)
        RadiusArc((3.4573, -97.9018), (4.7694, -92.704), 27.1584)
        RadiusArc((4.7694, -92.704), (7.4951, -87.8748), 17.5261)
        RadiusArc((7.4951, -87.8748), (12.7898, -84.0341), 12.7641)
        RadiusArc((12.7898, -84.0341), (17.9112, -83.17), 13.9754)
        RadiusArc((17.9112, -83.17), (23.7213, -83.9268), 21.9959)
        RadiusArc((23.7213, -83.9268), (30.0568, -87.9004), 13.5316)
        RadiusArc((30.0568, -87.9004), (32.9825, -93.0922), 15.1228)
        RadiusArc((32.9825, -93.0922), (34.1455, -100.93), 22.5068)
        RadiusArc((34.1455, -100.93), (33.5352, -106.972), 28.5775)
        RadiusArc((33.5352, -106.972), (30.8228, -113.243), 17.4706)
        RadiusArc((30.8228, -113.243), (25.9402, -117.474), 14.1592)
        RadiusArc((25.9402, -117.474), (18.8878, -119.666), 20.122)
        Line((18.8878, -119.666), (19.9255, -131.019))
        RadiusArc((19.9255, -131.019), (28.2498, -128.836), -32.4377)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# 'Sketch2': 11 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 45.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch2_4:
    with BuildLine():
        Line((-43.9743, -161.19), (32.6811, -161.19))
        Line((32.6811, -161.19), (32.6811, -140.257))
        Line((32.6811, -140.257), (42.0187, -140.257))
        Line((42.0187, -140.257), (42.0187, -194.147))
        Line((42.0187, -194.147), (32.6811, -194.147))
        Line((32.6811, -194.147), (32.6811, -172.237))
        Line((32.6811, -172.237), (-33.4767, -172.237))
        Line((-33.4767, -172.237), (-19.6231, -191.645))
        Line((-19.6231, -191.645), (-29.9983, -191.645))
        Line((-29.9983, -191.645), (-43.9743, -171.321))
        Line((-43.9743, -171.321), (-43.9743, -161.19))
    _inc_edges_sk_Sketch2_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_4 = Wire.combine(_inc_edges_sk_Sketch2_4)[0]
_wire_sk_Sketch2_4 = _wire_sk_Sketch2_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch2_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch2_4.wrapped, True)
_face_sk_Sketch2_4 = Face(_mkf_sk_Sketch2_4.Face())

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
    _bore_ax = _gAx2(_gPnt(0.0019, -674.9976, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.2472, 90.0)
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
    
    # -- Extrude2_p2 --
    _face = _face_sk_Sketch2_4
    _vec = Vector(0.0, 0.0, 1.0) * -5.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
