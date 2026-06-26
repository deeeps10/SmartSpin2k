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

# 'Sketch18': 10 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 46.1, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch18:
    with BuildLine():
        RadiusArc((147.1, 31.7), (146.8071, 32.4071), -1.0002)
        RadiusArc((146.8071, 32.4071), (146.1, 32.7), -1.0001)
        Line((146.1, 32.7), (42.1, 32.7))
        RadiusArc((42.1, 32.7), (41.1, 31.7), -1.0)
        Line((41.1, 31.7), (41.1, 24.7))
        RadiusArc((41.1, 24.7), (42.1, 23.7), -1.0)
        Line((42.1, 23.7), (146.1, 23.7))
        RadiusArc((146.1, 23.7), (146.8071, 23.9929), -1.0)
        RadiusArc((146.8071, 23.9929), (147.1, 24.7), -1.0)
        Line((147.1, 24.7), (147.1, 31.7))
    _inc_edges_sk_Sketch18 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18 = Wire.combine(_inc_edges_sk_Sketch18)[0]
_wire_sk_Sketch18 = _wire_sk_Sketch18.moved(_inclined_plane_1.location)
_mkf_sk_Sketch18 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch18.wrapped, True)
_face_sk_Sketch18 = Face(_mkf_sk_Sketch18.Face())

# 'Sketch19': 12 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, -26.7),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch19_2:
    with BuildLine():
        RadiusArc((-56.1, 42.6), (-57.1, 41.6), -1.0)
        Line((-57.1, 41.6), (-57.1, 28.2))
        RadiusArc((-57.1, 28.2), (-56.1, 27.2), -1.0)
        Line((-56.1, 27.2), (-42.1, 27.2))
        RadiusArc((-42.1, 27.2), (-41.1, 26.2), 1.0)
        Line((-41.1, 26.2), (-34.2276, 26.2))
        Line((-34.2276, 26.2), (-34.2276, 27.2))
        Line((-34.2276, 27.2), (-34.2276, 42.6))
        Line((-34.2276, 42.6), (-34.2276, 43.6))
        Line((-34.2276, 43.6), (-41.1, 43.6))
        RadiusArc((-41.1, 43.6), (-42.1, 42.6), 1.0)
        Line((-42.1, 42.6), (-56.1, 42.6))
    _inc_edges_sk_Sketch19_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch19_2 = Wire.combine(_inc_edges_sk_Sketch19_2)[0]
_wire_sk_Sketch19_2 = _wire_sk_Sketch19_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch19_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch19_2.wrapped, True)
_face_sk_Sketch19_2 = Face(_mkf_sk_Sketch19_2.Face())

# 'Sketch20': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (46.1,-31.7)->(46.1,-24.7) gap=7.0mm [LONG — verify geometry]
_inclined_plane_3 = Plane(
    origin=Vector(147.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch20_3:
    with BuildLine():
        Line((46.1, -32.7), (45.1, -32.7))
        RadiusArc((45.1, -32.7), (46.1, -31.7), -1.0)
        # auto-repair bridge: gap=?mm
        Line((46.1, -31.7), (46.1, -24.7))
        Line((46.1, -24.7), (46.1, -23.7))
        Line((46.1, -23.7), (46.1, -32.7))
    _inc_edges_sk_Sketch20_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_3 = Wire.combine(_inc_edges_sk_Sketch20_3)[0]
_wire_sk_Sketch20_3 = _wire_sk_Sketch20_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch20_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch20_3.wrapped, True)
_face_sk_Sketch20_3 = Face(_mkf_sk_Sketch20_3.Face())

# 'Sketch20': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (45.1,-23.7)->(24.7,-23.7) gap=20.4mm [LONG — verify geometry]
_inclined_plane_4 = Plane(
    origin=Vector(147.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch20_4:
    with BuildLine():
        Line((46.1, -23.7), (46.1, -24.7))
        RadiusArc((46.1, -24.7), (45.1, -23.7), -1.0)
        # auto-repair bridge: gap=?mm
        Line((45.1, -23.7), (24.7, -23.7))
        Line((24.7, -23.7), (23.7, -23.7))
        Line((23.7, -23.7), (46.1, -23.7))
    _inc_edges_sk_Sketch20_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_4 = Wire.combine(_inc_edges_sk_Sketch20_4)[0]
_wire_sk_Sketch20_4 = _wire_sk_Sketch20_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch20_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch20_4.wrapped, True)
_face_sk_Sketch20_4 = Face(_mkf_sk_Sketch20_4.Face())

# 'Sketch20': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (24.7,-32.7)->(45.1,-32.7) gap=20.4mm [LONG — verify geometry]
_inclined_plane_5 = Plane(
    origin=Vector(147.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch20_5:
    with BuildLine():
        Line((23.7, -32.7), (23.7, -31.7))
        RadiusArc((23.7, -31.7), (24.7, -32.7), -1.0)
        # auto-repair bridge: gap=?mm
        Line((24.7, -32.7), (45.1, -32.7))
        Line((45.1, -32.7), (46.1, -32.7))
        Line((46.1, -32.7), (23.7, -32.7))
    _inc_edges_sk_Sketch20_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_5 = Wire.combine(_inc_edges_sk_Sketch20_5)[0]
_wire_sk_Sketch20_5 = _wire_sk_Sketch20_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch20_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch20_5.wrapped, True)
_face_sk_Sketch20_5 = Face(_mkf_sk_Sketch20_5.Face())

# 'Sketch20': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (23.7,-24.7)->(23.7,-31.7) gap=7.0mm [LONG — verify geometry]
_inclined_plane_6 = Plane(
    origin=Vector(147.1, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch20_6:
    with BuildLine():
        Line((23.7, -23.7), (24.7, -23.7))
        RadiusArc((24.7, -23.7), (23.7, -24.7), -1.0)
        # auto-repair bridge: gap=?mm
        Line((23.7, -24.7), (23.7, -31.7))
        Line((23.7, -31.7), (23.7, -32.7))
        Line((23.7, -32.7), (23.7, -23.7))
    _inc_edges_sk_Sketch20_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch20_6 = Wire.combine(_inc_edges_sk_Sketch20_6)[0]
_wire_sk_Sketch20_6 = _wire_sk_Sketch20_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch20_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch20_6.wrapped, True)
_face_sk_Sketch20_6 = Face(_mkf_sk_Sketch20_6.Face())

# 'Sketch21': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (-41.1,24.7)->(-41.1,45.1) gap=20.4mm [LONG — verify geometry]
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 0.0, -32.7),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch21_7:
    with BuildLine():
        Line((-41.1, 23.7), (-42.1, 23.7))
        RadiusArc((-42.1, 23.7), (-41.1, 24.7), -1.0)
        # auto-repair bridge: gap=?mm
        Line((-41.1, 24.7), (-41.1, 45.1))
        Line((-41.1, 45.1), (-41.1, 46.1))
        Line((-41.1, 46.1), (-41.1, 23.7))
    _inc_edges_sk_Sketch21_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_7 = Wire.combine(_inc_edges_sk_Sketch21_7)[0]
_wire_sk_Sketch21_7 = _wire_sk_Sketch21_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch21_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch21_7.wrapped, True)
_face_sk_Sketch21_7 = Face(_mkf_sk_Sketch21_7.Face())

# 'Sketch21': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (-42.1,46.1)->(-146.1,46.1) gap=104.0mm [LONG — verify geometry]
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 0.0, -32.7),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch21_8:
    with BuildLine():
        Line((-41.1, 46.1), (-41.1, 45.1))
        RadiusArc((-41.1, 45.1), (-42.1, 46.1), -1.0)
        # auto-repair bridge: gap=?mm
        Line((-42.1, 46.1), (-146.1, 46.1))
        Line((-146.1, 46.1), (-147.1, 46.1))
        Line((-147.1, 46.1), (-41.1, 46.1))
    _inc_edges_sk_Sketch21_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_8 = Wire.combine(_inc_edges_sk_Sketch21_8)[0]
_wire_sk_Sketch21_8 = _wire_sk_Sketch21_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch21_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch21_8.wrapped, True)
_face_sk_Sketch21_8 = Face(_mkf_sk_Sketch21_8.Face())

# 'Sketch21': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (-147.1,45.1)->(-147.1,24.7) gap=20.4mm [LONG — verify geometry]
_inclined_plane_9 = Plane(
    origin=Vector(0.0, 0.0, -32.7),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch21_9:
    with BuildLine():
        Line((-147.1, 46.1), (-146.1, 46.1))
        RadiusArc((-146.1, 46.1), (-147.1, 45.1), -1.0)
        # auto-repair bridge: gap=?mm
        Line((-147.1, 45.1), (-147.1, 24.7))
        Line((-147.1, 24.7), (-147.1, 23.7))
        Line((-147.1, 23.7), (-147.1, 46.1))
    _inc_edges_sk_Sketch21_9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_9 = Wire.combine(_inc_edges_sk_Sketch21_9)[0]
_wire_sk_Sketch21_9 = _wire_sk_Sketch21_9.moved(_inclined_plane_9.location)
_mkf_sk_Sketch21_9 = BRepBuilderAPI_MakeFace(_inclined_plane_9.wrapped, _wire_sk_Sketch21_9.wrapped, True)
_face_sk_Sketch21_9 = Face(_mkf_sk_Sketch21_9.Face())

# 'Sketch21': 5 segments → Line/RadiusArc profile
# auto-repair bridge inserted: (-146.1,23.7)->(-42.1,23.7) gap=104.0mm [LONG — verify geometry]
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 0.0, -32.7),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch21_10:
    with BuildLine():
        Line((-147.1, 23.7), (-147.1, 24.7))
        RadiusArc((-147.1, 24.7), (-146.1, 23.7), -1.008)
        # auto-repair bridge: gap=?mm
        Line((-146.1, 23.7), (-42.1, 23.7))
        Line((-42.1, 23.7), (-41.1, 23.7))
        Line((-41.1, 23.7), (-147.1, 23.7))
    _inc_edges_sk_Sketch21_10 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch21_10 = Wire.combine(_inc_edges_sk_Sketch21_10)[0]
_wire_sk_Sketch21_10 = _wire_sk_Sketch21_10.moved(_inclined_plane_10.location)
_mkf_sk_Sketch21_10 = BRepBuilderAPI_MakeFace(_inclined_plane_10.wrapped, _wire_sk_Sketch21_10.wrapped, True)
_face_sk_Sketch21_10 = Face(_mkf_sk_Sketch21_10.Face())

# Path wire for Sweep2
with BuildLine() as _bl_Sweep2:
    Line((39.1, 27.2, -32.7), (56.1, 27.2, -32.7))
    ThreePointArc((56.1, 27.2, -32.7), (56.8071, 27.4929, -32.7), (57.1, 28.2, -32.7))
    Line((57.1, 28.2, -32.7), (57.1, 41.6, -32.7))
    ThreePointArc((57.1, 41.6, -32.7), (56.8071, 42.3071, -32.7), (56.1, 42.6, -32.7))
    Line((56.1, 42.6, -32.7), (36.6538, 42.5997, -32.7))
path_Sweep2 = _bl_Sweep2.wires()[0]

# Profile plane from sketch (origin at sketch_origin)
_plane_Sweep2 = Plane(origin=Vector(39.1, 0.0, 0.0), x_dir=Vector(0.0, -1.0, 0.0), z_dir=Vector(-1.0, 0.0, 0.0))

# 'Sketch23': 3 segments -> sweep profile
with BuildSketch(_plane_Sweep2) as sk_Sketch23_10:
    with BuildLine():
        Line((-26.2, -32.7), (-27.2, -32.7))
        Line((-27.2, -32.7), (-27.2, -31.7))
        RadiusArc((-27.2, -31.7), (-26.2, -32.7), -1.0)
    make_face()
# 'Sketch24': 3 segments → Line/RadiusArc profile
_inclined_plane_12 = Plane(
    origin=Vector(42.6, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_12) as sk_Sketch24_12:
    with BuildLine():
        Line((-28.1172, -29.8), (-27.2, -30.7172))
        Line((-27.2, -30.7172), (-27.2, -28.8828))
        Line((-27.2, -28.8828), (-28.1172, -29.8))
    _inc_edges_sk_Sketch24_12 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch24_12 = Wire.combine(_inc_edges_sk_Sketch24_12)[0]
_wire_sk_Sketch24_12 = _wire_sk_Sketch24_12.moved(_inclined_plane_12.location)
_mkf_sk_Sketch24_12 = BRepBuilderAPI_MakeFace(_inclined_plane_12.wrapped, _wire_sk_Sketch24_12.wrapped, True)
_face_sk_Sketch24_12 = Face(_mkf_sk_Sketch24_12.Face())

# 'Sketch24': 3 segments → Line/RadiusArc profile
_inclined_plane_13 = Plane(
    origin=Vector(42.6, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_13) as sk_Sketch24_13:
    with BuildLine():
        Line((-41.6828, -29.8), (-42.6, -28.8828))
        Line((-42.6, -28.8828), (-42.6, -30.7172))
        Line((-42.6, -30.7172), (-41.6828, -29.8))
    _inc_edges_sk_Sketch24_13 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch24_13 = Wire.combine(_inc_edges_sk_Sketch24_13)[0]
_wire_sk_Sketch24_13 = _wire_sk_Sketch24_13.moved(_inclined_plane_13.location)
_mkf_sk_Sketch24_13 = BRepBuilderAPI_MakeFace(_inclined_plane_13.wrapped, _wire_sk_Sketch24_13.wrapped, True)
_face_sk_Sketch24_13 = Face(_mkf_sk_Sketch24_13.Face())

# 'Sketch25': circle on inclined plane
_inclined_plane_14 = Plane(
    origin=Vector(0.0, 0.0, -23.7),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_14) as sk_Sketch25_14:
    with Locations((48.9, 34.9)):
        Circle(radius=2.6)

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude12 ---
    # -- Extrude12 --
    _face = _face_sk_Sketch18
    _vec = Vector(-0.0, 1.0, 0.0) * -22.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -22.400000095 mm
    
    # --- FEATURE: Extrude13 ---
    # -- Extrude13 --
    _face = _face_sk_Sketch19_2
    _vec = Vector(0.0, 0.0, -1.0) * 13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 13.000000 mm
    
    # --- FEATURE: Extrude14 ---
    # -- Extrude14_p0 --
    _face = _face_sk_Sketch20_3
    _vec = Vector(1.0, 0.0, 0.0) * -110.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -110.000000 mm
    
    # -- Extrude14_p1 --
    _face = _face_sk_Sketch20_4
    _vec = Vector(1.0, 0.0, 0.0) * -110.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -110.000000 mm
    
    # -- Extrude14_p2 --
    _face = _face_sk_Sketch20_5
    _vec = Vector(1.0, 0.0, 0.0) * -110.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -110.000000 mm
    
    # -- Extrude14_p3 --
    _face = _face_sk_Sketch20_6
    _vec = Vector(1.0, 0.0, 0.0) * -110.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -110.000000 mm
    
    # --- FEATURE: Extrude15 ---
    # -- Extrude15_p0 --
    _face = _face_sk_Sketch21_7
    _vec = Vector(0.0, 0.0, -1.0) * -20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -20.000000 mm
    
    # -- Extrude15_p1 --
    _face = _face_sk_Sketch21_8
    _vec = Vector(0.0, 0.0, -1.0) * -20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -20.000000 mm
    
    # -- Extrude15_p2 --
    _face = _face_sk_Sketch21_9
    _vec = Vector(0.0, 0.0, -1.0) * -20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -20.000000 mm
    
    # -- Extrude15_p3 --
    _face = _face_sk_Sketch21_10
    _vec = Vector(0.0, 0.0, -1.0) * -20.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -20.000000 mm
    
    # --- FEATURE: Sweep2 ---
    # -- Sweep2 --
    try:
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeFace
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE, TopAbs_EDGE
        from OCP.TopoDS import TopoDS
        from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
        from OCP.BRepAdaptor import BRepAdaptor_Curve
        from OCP.gp import gp_Pln, gp_Ax3, gp_Dir, gp_Pnt
        import numpy as _np
        _profile_face = sk_Sketch23_10.sketch.faces()[0]
        _occ_wire = None
        _wire_exp = TopExp_Explorer(_profile_face.wrapped, TopAbs_WIRE)
        if _wire_exp.More():
            _occ_wire = TopoDS.Wire_s(_wire_exp.Current())
        _path_wire = path_Sweep2
        def _make_pipe_solid(_wire, reverse=False):
            _w = _wire.Reversed() if reverse else _wire
            _pipe = BRepOffsetAPI_MakePipeShell(_path_wire.wrapped)
            _pipe.Add(_w)
            _pipe.Build()
            if not _pipe.IsDone(): return None
            if _pipe.MakeSolid(): return Solid(_pipe.Shape())
            return None
        def _fit_plane_cap(wire):
            _pts = []
            _ee = TopExp_Explorer(wire, TopAbs_EDGE)
            while _ee.More():
                _c = BRepAdaptor_Curve(TopoDS.Edge_s(_ee.Current()))
                _t = (_c.FirstParameter() + _c.LastParameter()) / 2.0
                _p = _c.Value(_t)
                _pts.append([_p.X(), _p.Y(), _p.Z()])
                _ee.Next()
            if len(_pts) < 3: return None
            _pts = _np.array(_pts)
            _cen = _pts.mean(axis=0)
            _, _, _vh = _np.linalg.svd(_pts - _cen)
            _n = _vh[-1]; _n /= _np.linalg.norm(_n)
            _x = _pts[0] - _cen; _x -= _np.dot(_x, _n) * _n
            if _np.linalg.norm(_x) < 1e-6: _x = _pts[1] - _cen; _x -= _np.dot(_x, _n) * _n
            _x /= _np.linalg.norm(_x)
            _ax = gp_Ax3(gp_Pnt(*_cen.tolist()), gp_Dir(*_n.tolist()), gp_Dir(*_x.tolist()))
            _mf = BRepBuilderAPI_MakeFace(gp_Pln(_ax), wire)
            return _mf.Face() if _mf.IsDone() else None
        # Attempt A: wire as-is
        _solid = _make_pipe_solid(_occ_wire) if _occ_wire else None
        if _solid is None and _occ_wire:
            # Attempt B: reversed wire
            _solid = _make_pipe_solid(_occ_wire, reverse=True)
        if _solid is None:
            # Attempt C: Solid.sweep() + cap free boundary wires
            _sweep_shell = Solid.sweep(sk_Sketch23_10.sketch.faces()[0], path_Sweep2)
            _sa = ShapeAnalysis_FreeBounds(_sweep_shell.wrapped)
            _cw_exp = TopExp_Explorer(_sa.GetClosedWires(), TopAbs_WIRE)
            _caps = []
            while _cw_exp.More():
                _w = TopoDS.Wire_s(_cw_exp.Current())
                _mf = BRepBuilderAPI_MakeFace(_w, True)
                if _mf.IsDone(): _caps.append(_mf.Face())
                else:
                    _fc = _fit_plane_cap(_w)
                    if _fc is not None: _caps.append(_fc)
                _cw_exp.Next()
            _sew = BRepBuilderAPI_Sewing(0.1)
            _sew.Add(_sweep_shell.wrapped)
            for _fc in _caps: _sew.Add(_fc)
            _sew.Perform()
            _mk = BRepBuilderAPI_MakeSolid()
            _exp = TopExp_Explorer(_sew.SewedShape(), TopAbs_SHELL)
            while _exp.More(): _mk.Add(TopoDS.Shell_s(_exp.Current())); _exp.Next()
            _mk.Build()
            if _mk.IsDone():
                _fix = ShapeFix_Solid(_mk.Solid())
                _fix.Perform()
                _solid = Solid(_fix.Shape())
            else:
                _solid = _sweep_shell
                print('WARNING: Sweep2 sweep — all solid attempts failed, result is Shell')
        # v17.95: final Shell→Solid coercion — add() rejects Shell in empty BuildPart
        from OCP.TopAbs import TopAbs_SHELL as _TS_SHELL, TopAbs_SOLID as _TS_SOLID
        if hasattr(_solid, 'wrapped') and _solid.wrapped.ShapeType() != _TS_SOLID:
            try:
                from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid as _MkSol2
                _mk2 = _MkSol2()
                _exp2 = TopExp_Explorer(_solid.wrapped, _TS_SHELL)
                while _exp2.More(): _mk2.Add(TopoDS.Shell_s(_exp2.Current())); _exp2.Next()
                _mk2.Build()
                if _mk2.IsDone(): _solid = Solid(_mk2.Shape())
            except Exception as _coerce_err:
                print('WARNING: Sweep2 Shell→Solid coercion failed:', _coerce_err)
        add(_solid, mode=Mode.SUBTRACT)
    except Exception as _sweep_err:
        print('WARNING: Sweep2 sweep failed:', _sweep_err)
    
    # --- FEATURE: Extrude16 ---
    # -- Extrude16_p0 --
    _face = _face_sk_Sketch24_12
    _vec = Vector(-1.0, 0.0, 0.0) * -15.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -15.500001907 mm
    
    # -- Extrude16_p1 --
    _face = _face_sk_Sketch24_13
    _vec = Vector(-1.0, 0.0, 0.0) * -15.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -15.500001907 mm
    
    # --- FEATURE: Extrude17 ---
    # -- Extrude17 --
    extrude(sk_Sketch25_14.sketch, amount=-5.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -5.000000 mm
    
    # --- FEATURE: Fillet5 ---
    # -- Fillet5 --
    # Fillet radius=1.0mm  (expr: 1 mm)  |  1 edge(s)
    # Edge indices exported from Fusion — valid for current body state.
    # If features were added before this fillet, re-run the edge diagnostic
    # below and update the indices.
    # edge 0 vertices: [41.1, 43.6, -26.7] → [41.1, 26.2, -26.7]
    try:
        # OCP-confirmed indices: [17]
        # OCP-confirmed indices: [17]
        fillet(part.edges()[17], radius=1.0)
    except Exception as _fe:
        print('WARNING: Fillet5 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)

# -- Volume --
import struct as _struct
def _stl_volume(path):
    with open(path, 'rb') as _f:
        _f.read(80)
        _n = _struct.unpack('<I', _f.read(4))[0]
        _vol = 0.0
        for _ in range(_n):
            _f.read(12)
            _p1 = _struct.unpack('<3f', _f.read(12))
            _p2 = _struct.unpack('<3f', _f.read(12))
            _p3 = _struct.unpack('<3f', _f.read(12))
            _f.read(2)
            _vol += (_p1[0]*(_p2[1]*_p3[2]-_p3[1]*_p2[2])
                    -_p1[1]*(_p2[0]*_p3[2]-_p3[0]*_p2[2])
                    +_p1[2]*(_p2[0]*_p3[1]-_p3[0]*_p2[1])) / 6.0
        return abs(_vol), _n
_vol_mm3, _tris = _stl_volume('fusion_features.stl')
print('========================================')
print('  STL Volume')
print('========================================')
print(f'  File      : fusion_features.stl')
print(f'  Triangles : {_tris:,}')
print(f'  Volume    : {_vol_mm3:,.2f} mm³')
print(f'  Volume    : {_vol_mm3/1000:.4f} cm³')
print('========================================')
