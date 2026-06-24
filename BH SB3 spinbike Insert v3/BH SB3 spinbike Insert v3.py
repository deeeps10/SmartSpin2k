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

# 'Sketch1': circle on inclined plane
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with Locations((298.1193, -56.8088)):
        Circle(radius=36.0)

# 'Sketch2': 2 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        # Spline from EllipticalArc3D, 113 adaptive samples
        Spline((309.867, -78.288), (310.8666, -77.7103), (311.8379, -77.086), (312.7786, -76.4167), (313.6868, -75.7038), (314.5603, -74.9489), (315.3972, -74.1536), (316.1958, -73.3197), (316.9541, -72.4491), (317.6706, -71.5438), (318.3435, -70.6056), (318.9715, -69.6368), (319.5531, -68.6395), (320.0871, -67.6158), (320.5722, -66.5681), (321.0073, -65.4987), (321.3916, -64.41), (321.7241, -63.3044), (322.0041, -62.1843), (322.231, -61.0522), (322.4043, -59.9108), (322.5236, -58.7624), (322.5885, -57.6097), (322.5991, -56.4552), (322.5553, -55.3015), (322.457, -54.1511), (322.3047, -53.0066), (322.0986, -51.8706), (321.8391, -50.7456), (321.5269, -49.6341), (321.1627, -48.5385), (320.7472, -47.4613), (320.2813, -46.4049), (319.7662, -45.3716), (319.203, -44.3638), (318.5928, -43.3836), (317.9371, -42.4333), (317.2374, -41.515), (316.4951, -40.6307), (315.712, -39.7823), (314.8897, -38.9719), (314.0302, -38.2011), (313.1353, -37.4716), (312.2069, -36.7852), (311.2473, -36.1433), (310.2584, -35.5473), (309.2426, -34.9987), (308.202, -34.4985), (307.139, -34.048), (306.0559, -33.6481), (304.9552, -33.2996), (303.8392, -33.0035), (302.7106, -32.7603), (301.5717, -32.5706), (300.4252, -32.4348), (299.2735, -32.3532), (298.1193, -32.3259), (296.9651, -32.3532), (295.8134, -32.4348), (294.6669, -32.5706), (293.528, -32.7603), (292.3994, -33.0035), (291.2835, -33.2996), (290.1828, -33.6481), (289.0997, -34.048), (288.0367, -34.4985), (286.9961, -34.9987), (285.9802, -35.5473), (284.9914, -36.1433), (284.0317, -36.7852), (283.1034, -37.4716), (282.2084, -38.2011), (281.3489, -38.9719), (280.5266, -39.7823), (279.7435, -40.6307), (279.0012, -41.515), (278.3015, -42.4333), (277.6458, -43.3836), (277.0357, -44.3638), (276.4724, -45.3716), (275.9573, -46.4049), (275.4915, -47.4613), (275.076, -48.5385), (274.7117, -49.6341), (274.3995, -50.7456), (274.1401, -51.8706), (273.9339, -53.0066), (273.7816, -54.1511), (273.6834, -55.3015), (273.6395, -56.4552), (273.6501, -57.6097), (273.7151, -58.7624), (273.8343, -59.9108), (274.0076, -61.0522), (274.2345, -62.1843), (274.5145, -63.3044), (274.847, -64.41), (275.2313, -65.4987), (275.6665, -66.5681), (276.1516, -67.6158), (276.6855, -68.6395), (277.2671, -69.6368), (277.8951, -70.6056), (278.5681, -71.5438), (279.2845, -72.4491), (280.0429, -73.3197), (280.8414, -74.1536), (281.6784, -74.9489), (282.5519, -75.7038), (283.46, -76.4167), (284.4008, -77.086), (285.372, -77.7103), (286.3716, -78.288))
        Line((286.3716, -78.288), (309.867, -78.288))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch4': 14 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 16.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch4_3:
    with BuildLine():
        RadiusArc((-301.4434, -23.9767), (-318.4765, -30.8361), -33.0001)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-318.4765, -30.8361), (-318.4428, -38.2831), 5.5)
        RadiusArc((-318.4428, -38.2831), (-325.861, -38.9372), 5.5)
        RadiusArc((-325.861, -38.9372), (-331.1181, -56.531), -32.9996)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-331.1181, -56.531), (-325.2748, -61.1477), 5.5)
        RadiusArc((-325.2748, -61.1477), (-329.3886, -67.3554), 5.5)
        RadiusArc((-329.3886, -67.3554), (-318.9109, -82.4351), -33.0)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-318.9109, -82.4351), (-311.6582, -80.7452), 5.5)
        RadiusArc((-311.6582, -80.7452), (-309.3697, -87.8318), 5.5)
        RadiusArc((-309.3697, -87.8318), (-291.0472, -89.0421), -33.0)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-291.0472, -89.0421), (-287.8464, -82.318), 5.5)
        RadiusArc((-287.8464, -82.318), (-280.879, -84.9473), 5.5)
        RadiusArc((-280.879, -84.9473), (-268.5089, -71.3768), -33.0)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-268.5089, -71.3768), (-271.7704, -64.682), 5.5)
        RadiusArc((-271.7704, -64.682), (-265.3706, -60.8739), 5.5)
        RadiusArc((-265.3706, -60.8739), (-268.2678, -42.7415), -32.9998)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-268.2678, -42.7415), (-275.5356, -41.1172), 5.5)
        RadiusArc((-275.5356, -41.1172), (-274.5226, -33.7394), 5.5)
        RadiusArc((-274.5226, -33.7394), (-290.5055, -24.6992), -33.0001)
        # Arc split: sweep=170.44deg >= 150 — emitted as two half-arcs
        RadiusArc((-290.5055, -24.6992), (-296.3067, -29.3686), 5.5)
        RadiusArc((-296.3067, -29.3686), (-301.4434, -23.9767), 5.5)
    _inc_edges_sk_Sketch4_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_3 = Wire.combine(_inc_edges_sk_Sketch4_3)[0]
_wire_sk_Sketch4_3 = _wire_sk_Sketch4_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch4_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch4_3.wrapped, True)
_face_sk_Sketch4_3 = Face(_mkf_sk_Sketch4_3.Face())

# 'Sketch5': 2 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch5_4:
    with BuildLine():
        # Arc split: sweep=299.53deg >= 150 — emitted as two half-arcs
        RadiusArc((309.2394, -75.888), (298.1184, -34.7214), -22.0854)
        RadiusArc((298.1184, -34.7214), (286.9977, -75.888), -22.0854)
        Line((286.9977, -75.888), (309.2394, -75.888))
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
    extrude(sk_Sketch1.sketch, amount=-27.0)
    # Fusion depth expression: -26.999999285 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, -1.0) * 13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 13.000000715 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_3
    _vec = Vector(0.0, 0.0, 1.0) * 55.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 55.000000 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch5_4
    _vec = Vector(0.0, 0.0, -1.0) * -13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -13.000000715 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
