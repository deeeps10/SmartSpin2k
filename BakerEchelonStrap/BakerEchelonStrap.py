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

# 'Sketch4': 96 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4:
    with BuildLine():
        # Spline from EllipticalArc3D, 29 adaptive samples
        Spline((-41.0, 41.6411), (-41.2666, 41.7572), (-41.5419, 41.8506), (-41.8241, 41.9207), (-42.1112, 41.9669), (-42.4011, 41.9889), (-42.6918, 41.9867), (-42.9814, 41.9601), (-43.2677, 41.9094), (-43.5488, 41.835), (-43.8226, 41.7373), (-44.0873, 41.617), (-44.3411, 41.4751), (-44.5821, 41.3124), (-44.8086, 41.1301), (-45.0191, 40.9296), (-45.2122, 40.7121), (-45.3863, 40.4793), (-45.5404, 40.2327), (-45.6733, 39.9741), (-45.7841, 39.7053), (-45.8721, 39.4282), (-45.9366, 39.1447), (-45.9771, 38.8567), (-45.9935, 38.5664), (-45.9855, 38.2758), (-45.9532, 37.9868), (-45.8969, 37.7016), (-45.8169, 37.422))
        RadiusArc((-45.8169, 37.422), (-43.5699, 35.175), -3.5024)
        RadiusArc((-43.5699, 35.175), (-41.0, 35.4886), -3.1975)
        Line((-41.0, 35.4886), (-41.0, 0.646))
        # Spline from EllipticalArc3D, 29 adaptive samples
        Spline((-41.0, 0.646), (-41.4675, 0.8302), (-41.9567, 0.9454), (-42.4573, 0.9891), (-42.959, 0.9604), (-43.4514, 0.8599), (-43.9242, 0.6898), (-44.3677, 0.4534), (-44.7726, 0.1558), (-45.1306, -0.1968), (-45.4342, -0.5973), (-45.6772, -1.0372), (-45.8544, -1.5074), (-45.9622, -1.9982), (-45.9984, -2.4994), (-45.9622, -3.0006), (-45.8544, -3.4915), (-45.6772, -3.9617), (-45.4342, -4.4016), (-45.1306, -4.802), (-44.7726, -5.1547), (-44.3677, -5.4523), (-43.9242, -5.6886), (-43.4514, -5.8588), (-42.959, -5.9592), (-42.4573, -5.9879), (-41.9567, -5.9442), (-41.4675, -5.8291), (-41.0, -5.6449))
        Line((-41.0, -5.6449), (-41.0, -14.0))
        Line((-41.0, -14.0), (94.0, -14.0))
        Line((94.0, -14.0), (94.0, -12.0))
        Line((94.0, -12.0), (-33.3661, -12.0))
        RadiusArc((-33.3661, -12.0), (-37.0, -7.0627), -3.4449)
        Line((-37.0, -7.0627), (-37.0, 45.079))
        Line((-37.0, 45.079), (-36.4991, 44.9999))
        RadiusArc((-36.4991, 44.9999), (-33.1715, 49.5829), -3.5001)
        Line((-33.1715, 49.5829), (-33.3841, 50.0))
        Line((-33.3841, 50.0), (94.173, 50.0))
        Line((94.173, 50.0), (94.173, 52.0))
        Line((94.173, 52.0), (91.5067, 52.0))
        Line((91.5067, 52.0), (92.006, 52.014))
        Line((92.006, 52.014), (91.253, 52.75))
        Line((91.253, 52.75), (91.257, 52.0))
        Line((91.257, 52.0), (84.5067, 52.0))
        Line((84.5067, 52.0), (85.006, 52.014))
        Line((85.006, 52.014), (84.253, 52.75))
        Line((84.253, 52.75), (84.257, 52.0))
        Line((84.257, 52.0), (77.5067, 52.0))
        Line((77.5067, 52.0), (78.006, 52.014))
        Line((78.006, 52.014), (77.253, 52.75))
        Line((77.253, 52.75), (77.257, 52.0))
        Line((77.257, 52.0), (70.5067, 52.0))
        Line((70.5067, 52.0), (71.006, 52.014))
        Line((71.006, 52.014), (70.253, 52.75))
        Line((70.253, 52.75), (70.257, 52.0))
        Line((70.257, 52.0), (63.5067, 52.0))
        Line((63.5067, 52.0), (64.006, 52.014))
        Line((64.006, 52.014), (63.253, 52.75))
        Line((63.253, 52.75), (63.257, 52.0))
        Line((63.257, 52.0), (56.5067, 52.0))
        Line((56.5067, 52.0), (57.006, 52.014))
        Line((57.006, 52.014), (56.253, 52.75))
        Line((56.253, 52.75), (56.257, 52.0))
        Line((56.257, 52.0), (49.5067, 52.0))
        Line((49.5067, 52.0), (50.006, 52.014))
        Line((50.006, 52.014), (49.253, 52.75))
        Line((49.253, 52.75), (49.257, 52.0))
        Line((49.257, 52.0), (42.5067, 52.0))
        Line((42.5067, 52.0), (43.006, 52.014))
        Line((43.006, 52.014), (42.253, 52.75))
        Line((42.253, 52.75), (42.257, 52.0))
        Line((42.257, 52.0), (35.5067, 52.0))
        Line((35.5067, 52.0), (36.006, 52.014))
        Line((36.006, 52.014), (35.253, 52.75))
        Line((35.253, 52.75), (35.257, 52.0))
        Line((35.257, 52.0), (28.5067, 52.0))
        Line((28.5067, 52.0), (29.006, 52.014))
        Line((29.006, 52.014), (28.253, 52.75))
        Line((28.253, 52.75), (28.257, 52.0))
        Line((28.257, 52.0), (22.5067, 52.0))
        Line((22.5067, 52.0), (23.006, 52.014))
        Line((23.006, 52.014), (22.253, 52.75))
        Line((22.253, 52.75), (22.257, 52.0))
        Line((22.257, 52.0), (16.5067, 52.0))
        Line((16.5067, 52.0), (17.006, 52.014))
        Line((17.006, 52.014), (16.253, 52.75))
        Line((16.253, 52.75), (16.257, 52.0))
        Line((16.257, 52.0), (10.5067, 52.0))
        Line((10.5067, 52.0), (11.006, 52.014))
        Line((11.006, 52.014), (10.253, 52.75))
        Line((10.253, 52.75), (10.257, 52.0))
        Line((10.257, 52.0), (4.5067, 52.0))
        Line((4.5067, 52.0), (5.006, 52.014))
        Line((5.006, 52.014), (4.253, 52.75))
        Line((4.253, 52.75), (4.257, 52.0))
        Line((4.257, 52.0), (-1.4933, 52.0))
        Line((-1.4933, 52.0), (-0.994, 52.014))
        Line((-0.994, 52.014), (-1.747, 52.75))
        Line((-1.747, 52.75), (-1.743, 52.0))
        Line((-1.743, 52.0), (-7.5183, 52.0))
        Line((-7.5183, 52.0), (-6.994, 52.014))
        Line((-6.994, 52.014), (-7.747, 52.75))
        Line((-7.747, 52.75), (-7.743, 52.0))
        Line((-7.743, 52.0), (-13.5183, 52.0))
        Line((-13.5183, 52.0), (-12.994, 52.014))
        Line((-12.994, 52.014), (-13.747, 52.75))
        Line((-13.747, 52.75), (-13.743, 52.0))
        Line((-13.743, 52.0), (-19.5183, 52.0))
        Line((-19.5183, 52.0), (-18.994, 52.014))
        Line((-18.994, 52.014), (-19.747, 52.75))
        Line((-19.747, 52.75), (-19.743, 52.0))
        Line((-19.743, 52.0), (-25.5183, 52.0))
        Line((-25.5183, 52.0), (-24.994, 52.014))
        Line((-24.994, 52.014), (-25.747, 52.75))
        Line((-25.747, 52.75), (-25.743, 52.0))
        Line((-25.743, 52.0), (-40.827, 52.0))
        Line((-40.827, 52.0), (-40.827, 50.0))
        Line((-40.827, 50.0), (-41.0, 50.0))
        Line((-41.0, 50.0), (-41.0, 41.6411))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_2:
    with BuildLine():
        Line((-22.746, -14.997), (-22.753, -14.247))
        Line((-22.753, -14.247), (-21.997, -14.256))
        Line((-21.997, -14.256), (-22.746, -14.997))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_3:
    with BuildLine():
        Line((-15.997, -14.256), (-16.746, -14.997))
        Line((-16.746, -14.997), (-16.753, -14.247))
        Line((-16.753, -14.247), (-15.997, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_4:
    with BuildLine():
        Line((-9.997, -14.256), (-10.746, -14.997))
        Line((-10.746, -14.997), (-10.753, -14.247))
        Line((-10.753, -14.247), (-9.997, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_5:
    with BuildLine():
        Line((-3.997, -14.256), (-4.746, -14.997))
        Line((-4.746, -14.997), (-4.753, -14.247))
        Line((-4.753, -14.247), (-3.997, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_6:
    with BuildLine():
        Line((2.003, -14.256), (1.254, -14.997))
        Line((1.254, -14.997), (1.247, -14.247))
        Line((1.247, -14.247), (2.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_7:
    with BuildLine():
        Line((8.003, -14.256), (7.254, -14.997))
        Line((7.254, -14.997), (7.247, -14.247))
        Line((7.247, -14.247), (8.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_8:
    with BuildLine():
        Line((14.003, -14.256), (13.254, -14.997))
        Line((13.254, -14.997), (13.247, -14.247))
        Line((13.247, -14.247), (14.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_9:
    with BuildLine():
        Line((20.003, -14.256), (19.254, -14.997))
        Line((19.254, -14.997), (19.247, -14.247))
        Line((19.247, -14.247), (20.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_10:
    with BuildLine():
        Line((26.003, -14.256), (25.254, -14.997))
        Line((25.254, -14.997), (25.247, -14.247))
        Line((25.247, -14.247), (26.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_11:
    with BuildLine():
        Line((32.003, -14.256), (31.254, -14.997))
        Line((31.254, -14.997), (31.247, -14.247))
        Line((31.247, -14.247), (32.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_12:
    with BuildLine():
        Line((38.003, -14.256), (37.254, -14.997))
        Line((37.254, -14.997), (37.247, -14.247))
        Line((37.247, -14.247), (38.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_13:
    with BuildLine():
        Line((44.003, -14.256), (43.254, -14.997))
        Line((43.254, -14.997), (43.247, -14.247))
        Line((43.247, -14.247), (44.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_14:
    with BuildLine():
        Line((50.003, -14.256), (49.254, -14.997))
        Line((49.254, -14.997), (49.247, -14.247))
        Line((49.247, -14.247), (50.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_15:
    with BuildLine():
        Line((56.003, -14.256), (55.254, -14.997))
        Line((55.254, -14.997), (55.247, -14.247))
        Line((55.247, -14.247), (56.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_16:
    with BuildLine():
        Line((62.003, -14.256), (61.254, -14.997))
        Line((61.254, -14.997), (61.247, -14.247))
        Line((61.247, -14.247), (62.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_17:
    with BuildLine():
        Line((68.003, -14.256), (67.254, -14.997))
        Line((67.254, -14.997), (67.247, -14.247))
        Line((67.247, -14.247), (68.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_18:
    with BuildLine():
        Line((74.003, -14.256), (73.254, -14.997))
        Line((73.254, -14.997), (73.247, -14.247))
        Line((73.247, -14.247), (74.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_19:
    with BuildLine():
        Line((80.003, -14.256), (79.254, -14.997))
        Line((79.254, -14.997), (79.247, -14.247))
        Line((79.247, -14.247), (80.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_20:
    with BuildLine():
        Line((86.003, -14.256), (85.254, -14.997))
        Line((85.254, -14.997), (85.247, -14.247))
        Line((85.247, -14.247), (86.003, -14.256))
    make_face()

# 'Sketch4': 3 segments → Line/RadiusArc profile
with BuildSketch(Plane.XY) as sk_Sketch4_21:
    with BuildLine():
        Line((92.003, -14.256), (91.254, -14.997))
        Line((91.254, -14.997), (91.247, -14.247))
        Line((91.247, -14.247), (92.003, -14.256))
    make_face()

# 'Sketch3': 36 segments → Line/RadiusArc profile
_inclined_plane_22 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_22) as sk_Sketch3_22:
    with BuildLine():
        Line((45.6556, -4.0), (51.247, -4.0))
        Line((51.247, -4.0), (51.256, -4.754))
        Line((51.256, -4.754), (51.997, -4.004))
        Line((51.997, -4.004), (51.5684, -4.0))
        Line((51.5684, -4.0), (56.247, -4.0))
        Line((56.247, -4.0), (56.256, -4.754))
        Line((56.256, -4.754), (56.997, -4.004))
        Line((56.997, -4.004), (56.5684, -4.0))
        Line((56.5684, -4.0), (61.247, -4.0))
        Line((61.247, -4.0), (61.256, -4.754))
        Line((61.256, -4.754), (61.997, -4.004))
        Line((61.997, -4.004), (61.5684, -4.0))
        Line((61.5684, -4.0), (66.247, -4.0))
        Line((66.247, -4.0), (66.256, -4.754))
        Line((66.256, -4.754), (66.997, -4.004))
        Line((66.997, -4.004), (66.5684, -4.0))
        Line((66.5684, -4.0), (71.247, -4.0))
        Line((71.247, -4.0), (71.256, -4.754))
        Line((71.256, -4.754), (71.997, -4.004))
        Line((71.997, -4.004), (71.5684, -4.0))
        Line((71.5684, -4.0), (76.247, -4.0))
        Line((76.247, -4.0), (76.256, -4.754))
        Line((76.256, -4.754), (76.997, -4.004))
        Line((76.997, -4.004), (76.5684, -4.0))
        Line((76.5684, -4.0), (81.247, -4.0))
        Line((81.247, -4.0), (81.256, -4.754))
        Line((81.256, -4.754), (81.997, -4.004))
        Line((81.997, -4.004), (81.5684, -4.0))
        Line((81.5684, -4.0), (86.247, -4.0))
        Line((86.247, -4.0), (86.256, -4.754))
        Line((86.256, -4.754), (86.997, -4.004))
        Line((86.997, -4.004), (86.5684, -4.0))
        Line((86.5684, -4.0), (91.0, -4.0))
        Line((91.0, -4.0), (91.0, -2.0))
        Line((91.0, -2.0), (45.9591, -2.0))
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((45.9591, -2.0), (45.9888, -2.2916), (45.9939, -2.5847), (45.9744, -2.8771), (45.9305, -3.1669), (45.8623, -3.452), (45.7705, -3.7303), (45.6556, -4.0))
    _inc_edges_sk_Sketch3_22 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_22 = Wire.combine(_inc_edges_sk_Sketch3_22)[0]
_wire_sk_Sketch3_22 = _wire_sk_Sketch3_22.moved(_inclined_plane_22.location)
_mkf_sk_Sketch3_22 = BRepBuilderAPI_MakeFace(_inclined_plane_22.wrapped, _wire_sk_Sketch3_22.wrapped, True)
_face_sk_Sketch3_22 = Face(_mkf_sk_Sketch3_22.Face())

# 'Sketch3': 12 segments → Line/RadiusArc profile
_inclined_plane_23 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_23) as sk_Sketch3_23:
    with BuildLine():
        Line((73.0, 40.0), (68.0, 40.0))
        Line((68.0, 40.0), (63.0, 40.0))
        Line((63.0, 40.0), (58.0, 40.0))
        Line((58.0, 40.0), (53.0, 40.0))
        Line((53.0, 40.0), (45.6562, 40.0))
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((45.6562, 40.0), (45.7699, 39.73), (45.8604, 39.4515), (45.9272, 39.1663), (45.9696, 38.8764), (45.9875, 38.5841), (45.9807, 38.2912), (45.9492, 38.0))
        Line((45.9492, 38.0), (91.0, 38.0))
        Line((91.0, 38.0), (91.0, 40.0))
        Line((91.0, 40.0), (88.0, 40.0))
        Line((88.0, 40.0), (83.0, 40.0))
        Line((83.0, 40.0), (78.0, 40.0))
        Line((78.0, 40.0), (73.0, 40.0))
    _inc_edges_sk_Sketch3_23 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_23 = Wire.combine(_inc_edges_sk_Sketch3_23)[0]
_wire_sk_Sketch3_23 = _wire_sk_Sketch3_23.moved(_inclined_plane_23.location)
_mkf_sk_Sketch3_23 = BRepBuilderAPI_MakeFace(_inclined_plane_23.wrapped, _wire_sk_Sketch3_23.wrapped, True)
_face_sk_Sketch3_23 = Face(_mkf_sk_Sketch3_23.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_24 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_24) as sk_Sketch3_24:
    with BuildLine():
        Line((78.0, 40.0), (77.022, 40.75))
        Line((77.022, 40.75), (77.0, 40.008))
        Line((77.0, 40.008), (78.0, 40.0))
    _inc_edges_sk_Sketch3_24 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_24 = Wire.combine(_inc_edges_sk_Sketch3_24)[0]
_wire_sk_Sketch3_24 = _wire_sk_Sketch3_24.moved(_inclined_plane_24.location)
_mkf_sk_Sketch3_24 = BRepBuilderAPI_MakeFace(_inclined_plane_24.wrapped, _wire_sk_Sketch3_24.wrapped, True)
_face_sk_Sketch3_24 = Face(_mkf_sk_Sketch3_24.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_25 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_25) as sk_Sketch3_25:
    with BuildLine():
        Line((88.0, 40.0), (87.022, 40.75))
        Line((87.022, 40.75), (87.0, 40.008))
        Line((87.0, 40.008), (88.0, 40.0))
    _inc_edges_sk_Sketch3_25 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_25 = Wire.combine(_inc_edges_sk_Sketch3_25)[0]
_wire_sk_Sketch3_25 = _wire_sk_Sketch3_25.moved(_inclined_plane_25.location)
_mkf_sk_Sketch3_25 = BRepBuilderAPI_MakeFace(_inclined_plane_25.wrapped, _wire_sk_Sketch3_25.wrapped, True)
_face_sk_Sketch3_25 = Face(_mkf_sk_Sketch3_25.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_26 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_26) as sk_Sketch3_26:
    with BuildLine():
        Line((83.0, 40.0), (82.022, 40.75))
        Line((82.022, 40.75), (82.0, 40.008))
        Line((82.0, 40.008), (83.0, 40.0))
    _inc_edges_sk_Sketch3_26 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_26 = Wire.combine(_inc_edges_sk_Sketch3_26)[0]
_wire_sk_Sketch3_26 = _wire_sk_Sketch3_26.moved(_inclined_plane_26.location)
_mkf_sk_Sketch3_26 = BRepBuilderAPI_MakeFace(_inclined_plane_26.wrapped, _wire_sk_Sketch3_26.wrapped, True)
_face_sk_Sketch3_26 = Face(_mkf_sk_Sketch3_26.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_27 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_27) as sk_Sketch3_27:
    with BuildLine():
        Line((73.0, 40.0), (72.022, 40.75))
        Line((72.022, 40.75), (72.0, 40.008))
        Line((72.0, 40.008), (73.0, 40.0))
    _inc_edges_sk_Sketch3_27 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_27 = Wire.combine(_inc_edges_sk_Sketch3_27)[0]
_wire_sk_Sketch3_27 = _wire_sk_Sketch3_27.moved(_inclined_plane_27.location)
_mkf_sk_Sketch3_27 = BRepBuilderAPI_MakeFace(_inclined_plane_27.wrapped, _wire_sk_Sketch3_27.wrapped, True)
_face_sk_Sketch3_27 = Face(_mkf_sk_Sketch3_27.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_28 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_28) as sk_Sketch3_28:
    with BuildLine():
        Line((68.0, 40.0), (67.022, 40.75))
        Line((67.022, 40.75), (67.0, 40.008))
        Line((67.0, 40.008), (68.0, 40.0))
    _inc_edges_sk_Sketch3_28 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_28 = Wire.combine(_inc_edges_sk_Sketch3_28)[0]
_wire_sk_Sketch3_28 = _wire_sk_Sketch3_28.moved(_inclined_plane_28.location)
_mkf_sk_Sketch3_28 = BRepBuilderAPI_MakeFace(_inclined_plane_28.wrapped, _wire_sk_Sketch3_28.wrapped, True)
_face_sk_Sketch3_28 = Face(_mkf_sk_Sketch3_28.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_29 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_29) as sk_Sketch3_29:
    with BuildLine():
        Line((63.0, 40.0), (62.022, 40.75))
        Line((62.022, 40.75), (62.0, 40.008))
        Line((62.0, 40.008), (63.0, 40.0))
    _inc_edges_sk_Sketch3_29 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_29 = Wire.combine(_inc_edges_sk_Sketch3_29)[0]
_wire_sk_Sketch3_29 = _wire_sk_Sketch3_29.moved(_inclined_plane_29.location)
_mkf_sk_Sketch3_29 = BRepBuilderAPI_MakeFace(_inclined_plane_29.wrapped, _wire_sk_Sketch3_29.wrapped, True)
_face_sk_Sketch3_29 = Face(_mkf_sk_Sketch3_29.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_30 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_30) as sk_Sketch3_30:
    with BuildLine():
        Line((58.0, 40.0), (57.022, 40.75))
        Line((57.022, 40.75), (57.0, 40.008))
        Line((57.0, 40.008), (58.0, 40.0))
    _inc_edges_sk_Sketch3_30 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_30 = Wire.combine(_inc_edges_sk_Sketch3_30)[0]
_wire_sk_Sketch3_30 = _wire_sk_Sketch3_30.moved(_inclined_plane_30.location)
_mkf_sk_Sketch3_30 = BRepBuilderAPI_MakeFace(_inclined_plane_30.wrapped, _wire_sk_Sketch3_30.wrapped, True)
_face_sk_Sketch3_30 = Face(_mkf_sk_Sketch3_30.Face())

# 'Sketch3': 3 segments → Line/RadiusArc profile
_inclined_plane_31 = Plane(
    origin=Vector(0.0, 0.0, 5.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_31) as sk_Sketch3_31:
    with BuildLine():
        Line((53.0, 40.0), (52.022, 40.75))
        Line((52.022, 40.75), (52.0, 40.008))
        Line((52.0, 40.008), (53.0, 40.0))
    _inc_edges_sk_Sketch3_31 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_31 = Wire.combine(_inc_edges_sk_Sketch3_31)[0]
_wire_sk_Sketch3_31 = _wire_sk_Sketch3_31.moved(_inclined_plane_31.location)
_mkf_sk_Sketch3_31 = BRepBuilderAPI_MakeFace(_inclined_plane_31.wrapped, _wire_sk_Sketch3_31.wrapped, True)
_face_sk_Sketch3_31 = Face(_mkf_sk_Sketch3_31.Face())

# 'Sketch5': 18 segments → Line/RadiusArc profile
_inclined_plane_32 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_32) as sk_Sketch5_32:
    with BuildLine():
        Line((-35.0, -82.0), (65.0, -82.0))
        Line((65.0, -82.0), (65.0, -46.0))
        Line((65.0, -46.0), (-35.0, -46.0))
        Line((-35.0, -46.0), (-35.0, -37.0))
        Line((-35.0, -37.0), (-65.0, -37.0))
        Line((-65.0, -37.0), (-65.0, -48.0))
        Line((-65.0, -48.0), (58.3745, -48.0))
        RadiusArc((58.3745, -48.0), (58.1735, -50.5811), -3.1975)
        RadiusArc((58.1735, -50.5811), (60.4194, -52.8292), -3.5024)
        Line((60.4194, -52.8292), (61.0, -52.921))
        Line((61.0, -52.921), (61.0, -75.079))
        Line((61.0, -75.079), (60.4194, -75.1708))
        RadiusArc((60.4194, -75.1708), (58.1734, -77.4189), -3.5024)
        RadiusArc((58.1734, -77.4189), (58.3744, -80.0), -3.1975)
        Line((58.3744, -80.0), (-65.0, -80.0))
        Line((-65.0, -80.0), (-65.0, -91.0))
        Line((-65.0, -91.0), (-35.0, -91.0))
        Line((-35.0, -91.0), (-35.0, -82.0))
    _inc_edges_sk_Sketch5_32 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_32 = Wire.combine(_inc_edges_sk_Sketch5_32)[0]
_wire_sk_Sketch5_32 = _wire_sk_Sketch5_32.moved(_inclined_plane_32.location)
_mkf_sk_Sketch5_32 = BRepBuilderAPI_MakeFace(_inclined_plane_32.wrapped, _wire_sk_Sketch5_32.wrapped, True)
_face_sk_Sketch5_32 = Face(_mkf_sk_Sketch5_32.Face())

# 'Sketch6': 4 segments → Line/RadiusArc profile
_inclined_plane_33 = Plane(
    origin=Vector(0.0, -46.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_33) as sk_Sketch6_33:
    with BuildLine():
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-51.0, -65.0), (-41.0, -75.0), -10.0)
        RadiusArc((-41.0, -75.0), (-31.0, -65.0), -10.0)
        Line((-31.0, -65.0), (-31.0, -20.0))
        Line((-31.0, -20.0), (-51.0, -20.0))
        Line((-51.0, -20.0), (-51.0, -65.0))
    _inc_edges_sk_Sketch6_33 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_33 = Wire.combine(_inc_edges_sk_Sketch6_33)[0]
_wire_sk_Sketch6_33 = _wire_sk_Sketch6_33.moved(_inclined_plane_33.location)
_mkf_sk_Sketch6_33 = BRepBuilderAPI_MakeFace(_inclined_plane_33.wrapped, _wire_sk_Sketch6_33.wrapped, True)
_face_sk_Sketch6_33 = Face(_mkf_sk_Sketch6_33.Face())

# 'Sketch7': 4 segments → Line/RadiusArc profile
_inclined_plane_34 = Plane(
    origin=Vector(0.0, -82.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_34) as sk_Sketch7_34:
    with BuildLine():
        Line((51.0, -20.0), (31.0, -20.0))
        Line((31.0, -20.0), (31.0, -65.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((31.0, -65.0), (41.0, -75.0), -10.0)
        RadiusArc((41.0, -75.0), (51.0, -65.0), -10.0)
        Line((51.0, -65.0), (51.0, -20.0))
    _inc_edges_sk_Sketch7_34 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_34 = Wire.combine(_inc_edges_sk_Sketch7_34)[0]
_wire_sk_Sketch7_34 = _wire_sk_Sketch7_34.moved(_inclined_plane_34.location)
_mkf_sk_Sketch7_34 = BRepBuilderAPI_MakeFace(_inclined_plane_34.wrapped, _wire_sk_Sketch7_34.wrapped, True)
_face_sk_Sketch7_34 = Face(_mkf_sk_Sketch7_34.Face())

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_35 = Plane(
    origin=Vector(65.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_35) as sk_Sketch8_35:
    with BuildLine():
        Line((-87.003, 3.998), (-87.003, 17.002))
        Line((-87.003, 17.002), (-83.998, 17.002))
        Line((-83.998, 17.002), (-83.998, 3.998))
        Line((-83.998, 3.998), (-87.003, 3.998))
    _inc_edges_sk_Sketch8_35 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_35 = Wire.combine(_inc_edges_sk_Sketch8_35)[0]
_wire_sk_Sketch8_35 = _wire_sk_Sketch8_35.moved(_inclined_plane_35.location)
_mkf_sk_Sketch8_35 = BRepBuilderAPI_MakeFace(_inclined_plane_35.wrapped, _wire_sk_Sketch8_35.wrapped, True)
_face_sk_Sketch8_35 = Face(_mkf_sk_Sketch8_35.Face())

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_36 = Plane(
    origin=Vector(65.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_36) as sk_Sketch8_36:
    with BuildLine():
        Line((-40.998, 16.002), (-40.998, 2.998))
        Line((-40.998, 2.998), (-44.003, 2.998))
        Line((-44.003, 2.998), (-44.003, 16.002))
        Line((-44.003, 16.002), (-40.998, 16.002))
    _inc_edges_sk_Sketch8_36 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_36 = Wire.combine(_inc_edges_sk_Sketch8_36)[0]
_wire_sk_Sketch8_36 = _wire_sk_Sketch8_36.moved(_inclined_plane_36.location)
_mkf_sk_Sketch8_36 = BRepBuilderAPI_MakeFace(_inclined_plane_36.wrapped, _wire_sk_Sketch8_36.wrapped, True)
_face_sk_Sketch8_36 = Face(_mkf_sk_Sketch8_36.Face())

# 'Sketch9': 6 segments → Line/RadiusArc profile
_inclined_plane_37 = Plane(
    origin=Vector(-41.0, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_37) as sk_Sketch9_37:
    with BuildLine():
        Line((-50.0, 18.0), (-50.0, 15.0))
        Line((-50.0, 15.0), (-0.9905, 15.0))
        Line((-0.9905, 15.0), (2.0, 15.0))
        Line((2.0, 15.0), (14.0, 15.0))
        Line((14.0, 15.0), (14.0, 18.0))
        Line((14.0, 18.0), (-50.0, 18.0))
    _inc_edges_sk_Sketch9_37 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch9_37 = Wire.combine(_inc_edges_sk_Sketch9_37)[0]
_wire_sk_Sketch9_37 = _wire_sk_Sketch9_37.moved(_inclined_plane_37.location)
_mkf_sk_Sketch9_37 = BRepBuilderAPI_MakeFace(_inclined_plane_37.wrapped, _wire_sk_Sketch9_37.wrapped, True)
_face_sk_Sketch9_37 = Face(_mkf_sk_Sketch9_37.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude3 ---
    # -- Extrude3_p0 --
    extrude(sk_Sketch4.sketch, amount=18.0)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p1 --
    extrude(sk_Sketch4_2.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p2 --
    extrude(sk_Sketch4_3.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p3 --
    extrude(sk_Sketch4_4.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p4 --
    extrude(sk_Sketch4_5.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p5 --
    extrude(sk_Sketch4_6.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p6 --
    extrude(sk_Sketch4_7.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p7 --
    extrude(sk_Sketch4_8.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p8 --
    extrude(sk_Sketch4_9.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p9 --
    extrude(sk_Sketch4_10.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p10 --
    extrude(sk_Sketch4_11.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p11 --
    extrude(sk_Sketch4_12.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p12 --
    extrude(sk_Sketch4_13.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p13 --
    extrude(sk_Sketch4_14.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p14 --
    extrude(sk_Sketch4_15.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p15 --
    extrude(sk_Sketch4_16.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p16 --
    extrude(sk_Sketch4_17.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p17 --
    extrude(sk_Sketch4_18.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p18 --
    extrude(sk_Sketch4_19.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p19 --
    extrude(sk_Sketch4_20.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # -- Extrude3_p20 --
    extrude(sk_Sketch4_21.sketch, amount=18.0, mode=Mode.ADD)
    # Fusion depth expression: 18.000000715 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    _face = _face_sk_Sketch3_22
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p1 --
    _face = _face_sk_Sketch3_23
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p2 --
    _face = _face_sk_Sketch3_24
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p3 --
    _face = _face_sk_Sketch3_25
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p4 --
    _face = _face_sk_Sketch3_26
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p5 --
    _face = _face_sk_Sketch3_27
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p6 --
    _face = _face_sk_Sketch3_28
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p7 --
    _face = _face_sk_Sketch3_29
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p8 --
    _face = _face_sk_Sketch3_30
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # -- Extrude4_p9 --
    _face = _face_sk_Sketch3_31
    _vec = Vector(0.0, 0.0, -1.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -10.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_32
    _vec = Vector(0.0, 0.0, -1.0) * -20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -20.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch6_33
    _vec = Vector(-0.0, 1.0, 0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -2.000002861 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7 --
    _face = _face_sk_Sketch7_34
    _vec = Vector(-0.0, -1.0, -0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -1.999998093 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8_p0 --
    _face = _face_sk_Sketch8_35
    _vec = Vector(1.0, 0.0, 0.0) * -30.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -30.000000 mm
    
    # -- Extrude8_p1 --
    _face = _face_sk_Sketch8_36
    _vec = Vector(1.0, 0.0, 0.0) * -30.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -30.000000 mm
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch9_37
    _vec = Vector(-1.0, 0.0, 0.0) * 18.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 18.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
