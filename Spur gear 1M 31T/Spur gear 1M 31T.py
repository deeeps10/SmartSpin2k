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

# -- Edge selection helpers --
# Use these to select edges for fillet/chamfer operations.
# Find edge coordinates using the diagnostic pattern at the bottom of this file.

def get_edge_by_endpoints(solid, p1, p2, tol=0.05):
    for e in solid.edges():
        verts = e.vertices()
        if len(verts) != 2:
            continue
        pts = [(v.X, v.Y, v.Z) for v in verts]
        if (all(abs(pts[0][i]-p1[i])<tol for i in range(3)) and
            all(abs(pts[1][i]-p2[i])<tol for i in range(3))) or \
           (all(abs(pts[0][i]-p2[i])<tol for i in range(3)) and
            all(abs(pts[1][i]-p1[i])<tol for i in range(3))):
            return e
    return None

def get_vertical_edge(solid, x, y, z0, z1, tol=0.01):
    for e in solid.edges():
        verts = e.vertices()
        if len(verts) != 2:
            continue
        xs = [v.X for v in verts]
        ys = [v.Y for v in verts]
        zs = sorted([v.Z for v in verts])
        if (abs(xs[0]-x)<tol and abs(xs[1]-x)<tol and
            abs(ys[0]-y)<tol and abs(ys[1]-y)<tol and
            abs(zs[0]-z0)<tol and abs(zs[1]-z1)<tol):
            return e
    return None

# All dimensions below are raw numbers.

# 'Sketch11': 124 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(9.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch11:
    with BuildLine():
        Line((32.8829, 14.5289), (32.9425, 15.1153))
        RadiusArc((32.9425, 15.1153), (30.7414, 16.027), -6.2)
        Line((30.7414, 16.027), (30.7414, 16.9553))
        RadiusArc((30.7414, 16.9553), (32.9425, 17.8669), -6.1999)
        Line((32.9425, 17.8669), (32.8829, 18.4533))
        RadiusArc((32.8829, 18.4533), (30.5433, 18.9032), -6.2)
        Line((30.5433, 18.9032), (30.3565, 19.8126))
        RadiusArc((30.3565, 19.8126), (32.329, 21.1486), -6.1999)
        Line((32.329, 21.1486), (32.1526, 21.711))
        RadiusArc((32.1526, 21.711), (29.7703, 21.6808), -6.2)
        Line((29.7703, 21.6808), (29.4042, 22.5339))
        RadiusArc((29.4042, 22.5339), (31.0674, 24.2396), -6.2001)
        Line((31.0674, 24.2396), (30.7814, 24.755))
        RadiusArc((30.7814, 24.755), (28.454, 24.2458), -6.2)
        Line((28.454, 24.2458), (27.9237, 25.0078))
        RadiusArc((27.9237, 25.0078), (29.2095, 27.0134), -6.2)
        Line((29.2095, 27.0134), (28.8255, 27.4607))
        RadiusArc((28.8255, 27.4607), (26.6483, 26.4934), -6.2)
        Line((26.6483, 26.4934), (25.9755, 27.133))
        RadiusArc((25.9755, 27.133), (26.8312, 29.3564), -6.2)
        Line((26.8312, 29.3564), (26.3651, 29.7172))
        RadiusArc((26.3651, 29.7172), (24.4271, 28.3315), -6.2001)
        Line((24.4271, 28.3315), (23.6393, 28.8226))
        RadiusArc((23.6393, 28.8226), (24.0299, 31.1727), -6.1999)
        Line((24.0299, 31.1727), (23.5008, 31.4323))
        RadiusArc((23.5008, 31.4323), (21.8814, 29.6849), -6.2)
        Line((21.8814, 29.6849), (21.0109, 30.0073))
        RadiusArc((21.0109, 30.0073), (20.9204, 32.388), -6.1999)
        Line((20.9204, 32.388), (20.3498, 32.5357))
        RadiusArc((20.3498, 32.5357), (19.1154, 30.498), -6.1999)
        Line((19.1154, 30.498), (18.1977, 30.6386))
        RadiusArc((18.1977, 30.6386), (17.6299, 32.9524), -6.2)
        Line((17.6299, 32.9524), (17.0412, 32.9822))
        RadiusArc((17.0412, 32.9822), (16.2422, 30.7378), -6.2)
        Line((16.2422, 30.7378), (15.3151, 30.6908))
        RadiusArc((15.3151, 30.6908), (14.2932, 32.8429), -6.2)
        Line((14.2932, 32.8429), (13.7105, 32.7536))
        RadiusArc((13.7105, 32.7536), (13.3797, 30.3943), -6.2)
        Line((13.3797, 30.3943), (12.481, 30.1616))
        RadiusArc((12.481, 30.1616), (11.0467, 32.0639), -6.2)
        Line((11.0467, 32.0639), (10.494, 31.8592))
        RadiusArc((10.494, 31.8592), (10.6449, 29.4816), -6.2)
        Line((10.6449, 29.4816), (9.8114, 29.0727))
        RadiusArc((9.8114, 29.0727), (8.0236, 30.6474), -6.2)
        Line((8.0236, 30.6474), (7.5234, 30.3356))
        RadiusArc((7.5234, 30.3356), (8.1498, 28.037), -6.2)
        Line((8.1498, 28.037), (7.4157, 27.4688))
        RadiusArc((7.4157, 27.4688), (5.3475, 28.6513), -6.2001)
        Line((5.3475, 28.6513), (4.9203, 28.2452))
        RadiusArc((4.9203, 28.2452), (5.9965, 26.1198), -6.2)
        Line((5.9965, 26.1198), (5.3919, 25.4154))
        RadiusArc((5.3919, 25.4154), (3.1279, 26.1574), -6.1999)
        Line((3.1279, 26.1574), (2.7912, 25.6737))
        RadiusArc((2.7912, 25.6737), (4.2733, 23.8084), -6.2)
        Line((4.2733, 23.8084), (3.8228, 22.9967))
        RadiusArc((3.8228, 22.9967), (1.4558, 23.2678), -6.2)
        Line((1.4558, 23.2678), (1.2234, 22.7261))
        RadiusArc((1.2234, 22.7261), (3.0506, 21.1973), -6.2)
        Line((3.0506, 21.1973), (2.7727, 20.3116))
        RadiusArc((2.7727, 20.3116), (0.3997, 20.1007), -6.2)
        Line((0.3997, 20.1007), (0.281, 19.5233))
        RadiusArc((0.281, 19.5233), (2.3786, 18.3937), -6.2)
        Line((2.3786, 18.3937), (2.2847, 17.4701))
        RadiusArc((2.2847, 17.4701), (0.0026, 16.7858), -6.2)
        Line((0.0026, 16.7858), (0.0026, 16.1964))
        RadiusArc((0.0026, 16.1964), (2.2847, 15.5121), -6.2)
        Line((2.2847, 15.5121), (2.3786, 14.5886))
        RadiusArc((2.3786, 14.5886), (0.281, 13.4589), -6.2)
        Line((0.281, 13.4589), (0.3997, 12.8815))
        RadiusArc((0.3997, 12.8815), (2.7727, 12.6706), -6.2)
        Line((2.7727, 12.6706), (3.0506, 11.7849))
        RadiusArc((3.0506, 11.7849), (1.2234, 10.2561), -6.2)
        Line((1.2234, 10.2561), (1.4558, 9.7145))
        RadiusArc((1.4558, 9.7145), (3.8228, 9.9856), -6.2)
        Line((3.8228, 9.9856), (4.2733, 9.1739))
        RadiusArc((4.2733, 9.1739), (2.7912, 7.3086), -6.2)
        Line((2.7912, 7.3086), (3.1279, 6.8248))
        RadiusArc((3.1279, 6.8248), (5.3919, 7.5668), -6.2)
        Line((5.3919, 7.5668), (5.9965, 6.8625))
        RadiusArc((5.9965, 6.8625), (4.9203, 4.737), -6.2)
        Line((4.9203, 4.737), (5.3475, 4.3309))
        RadiusArc((5.3475, 4.3309), (7.4157, 5.5134), -6.2)
        Line((7.4157, 5.5134), (8.1498, 4.9452))
        RadiusArc((8.1498, 4.9452), (7.5234, 2.6466), -6.2)
        Line((7.5234, 2.6466), (8.0236, 2.3348))
        RadiusArc((8.0236, 2.3348), (9.8114, 3.9095), -6.2)
        Line((9.8114, 3.9095), (10.6449, 3.5007))
        RadiusArc((10.6449, 3.5007), (10.494, 1.123), -6.2)
        Line((10.494, 1.123), (11.0467, 0.9183))
        RadiusArc((11.0467, 0.9183), (12.481, 2.8207), -6.2)
        Line((12.481, 2.8207), (13.3797, 2.588))
        RadiusArc((13.3797, 2.588), (13.7105, 0.2286), -6.2)
        Line((13.7105, 0.2286), (14.2932, 0.1394))
        RadiusArc((14.2932, 0.1394), (15.3151, 2.2915), -6.2)
        Line((15.3151, 2.2915), (16.2422, 2.2445))
        RadiusArc((16.2422, 2.2445), (17.0412, 0.0), -6.2)
        Line((17.0412, 0.0), (17.6299, 0.0299))
        RadiusArc((17.6299, 0.0299), (18.1977, 2.3436), -6.2)
        Line((18.1977, 2.3436), (19.1154, 2.4842))
        RadiusArc((19.1154, 2.4842), (20.3498, 0.4465), -6.2)
        Line((20.3498, 0.4465), (20.9204, 0.5943))
        RadiusArc((20.9204, 0.5943), (21.0109, 2.975), -6.1999)
        Line((21.0109, 2.975), (21.8814, 3.2974))
        RadiusArc((21.8814, 3.2974), (23.5008, 1.5499), -6.2)
        Line((23.5008, 1.5499), (24.0299, 1.8095))
        RadiusArc((24.0299, 1.8095), (23.6393, 4.1597), -6.1999)
        Line((23.6393, 4.1597), (24.4271, 4.6507))
        RadiusArc((24.4271, 4.6507), (26.3651, 3.265), -6.2)
        Line((26.3651, 3.265), (26.8312, 3.6258))
        RadiusArc((26.8312, 3.6258), (25.9755, 5.8492), -6.2)
        Line((25.9755, 5.8492), (26.6483, 6.4888))
        RadiusArc((26.6483, 6.4888), (28.8255, 5.5216), -6.2)
        Line((28.8255, 5.5216), (29.2095, 5.9688))
        RadiusArc((29.2095, 5.9688), (27.9237, 7.9745), -6.2)
        Line((27.9237, 7.9745), (28.454, 8.7364))
        RadiusArc((28.454, 8.7364), (30.7814, 8.2272), -6.2)
        Line((30.7814, 8.2272), (31.0674, 8.7426))
        RadiusArc((31.0674, 8.7426), (29.4042, 10.4484), -6.2)
        Line((29.4042, 10.4484), (29.7703, 11.3015))
        RadiusArc((29.7703, 11.3015), (32.1526, 11.2712), -6.2)
        Line((32.1526, 11.2712), (32.329, 11.8336))
        RadiusArc((32.329, 11.8336), (30.3565, 13.1697), -6.2)
        Line((30.3565, 13.1697), (30.5433, 14.079))
        RadiusArc((30.5433, 14.079), (32.8829, 14.5289), -6.2)
    _inc_edges_sk_Sketch11 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11 = Wire.combine(_inc_edges_sk_Sketch11)[0]
_wire_sk_Sketch11 = _wire_sk_Sketch11.moved(_inclined_plane_1.location)
_mkf_sk_Sketch11 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch11.wrapped, True)
_face_sk_Sketch11 = Face(_mkf_sk_Sketch11.Face())

# 'Sketch12': circle on inclined plane
_inclined_plane_2 = Plane(
    origin=Vector(9.0, -0.0, -0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch12_2:
    with Locations((16.5, 16.4911)):
        Circle(radius=4.5)

# 'Sketch13': circle on inclined plane
_inclined_plane_3 = Plane(
    origin=Vector(10.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch13_3:
    with Locations((16.5, 16.4911)):
        Circle(radius=3.85)

# 'Sketch14': 4 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 19.4698, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch14_4:
    with BuildLine():
        Line((38.9393, -18.9411), (38.9393, -14.0411))
        Line((38.9393, -14.0411), (22.0, -14.0411))
        Line((22.0, -14.0411), (22.0, -18.9411))
        Line((22.0, -18.9411), (38.9393, -18.9411))
    _inc_edges_sk_Sketch14_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch14_4 = Wire.combine(_inc_edges_sk_Sketch14_4)[0]
_wire_sk_Sketch14_4 = _wire_sk_Sketch14_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch14_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch14_4.wrapped, True)
_face_sk_Sketch14_4 = Face(_mkf_sk_Sketch14_4.Face())

_inclined_plane_5 = Plane(
    origin=Vector(0.0, 16.5, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
# 'Sketch17': 5 segments → revolve profile
with BuildSketch(_inclined_plane_5) as sk_Sketch17_4:
    with BuildLine():
        Line((34.5, -20.3411), (34.5, -21.1184))
        Line((34.5, -21.1184), (36.2658, -21.1184))
        Line((36.2658, -21.1184), (36.2658, -19.8437))
        Line((36.2658, -19.8437), (35.0, -19.8437))
        RadiusArc((35.0, -19.8437), (34.5, -20.3411), 0.5)
    make_face()
# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch11
    _vec = Vector(1.0, 0.0, 0.0) * -9.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -9.000000358 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10 --
    extrude(sk_Sketch12_2.sketch, amount=1.0, mode=Mode.ADD)
    # Fusion depth expression: 0.999999642 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    extrude(sk_Sketch13_3.sketch, amount=25.0, mode=Mode.ADD)
    # Fusion depth expression: 24.999998829 mm
    
    # --- FEATURE: Extrude12 ---
    # -- Extrude12 --
    _face = _face_sk_Sketch14_4
    _vec = Vector(-0.0, 1.0, 0.0) * 2.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.500000 mm
    
    # --- FEATURE: Fillet3 ---
    # -- Fillet3 --
    # Fillet radius=0.5mm  (expr: 0.5 mm)  |  1 edge(s)
    # Edge indices exported from Fusion — valid for current body state.
    # If features were added before this fillet, re-run the edge diagnostic
    # below and update the indices.
    # edge 0 vertices: [35.0, 19.4698, 14.0411] → [35.0, 19.4698, 18.9411]
    try:
        # OCP-confirmed indices: [388]
        # OCP-confirmed indices: [388]
        fillet(part.edges()[388], radius=0.5)
    except Exception as _fe:
        print('WARNING: Fillet3 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    
    # --- FEATURE: Revolve1 ---
    # -- Revolve1 --
    _custom_axis = Axis(
        Vector(35.0, 16.5, 16.4911),
        Vector(-1.0, 0.0, 0.0),
    )
    revolve(sk_Sketch17_4.sketch.faces(), axis=_custom_axis, mode=Mode.SUBTRACT)
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)

