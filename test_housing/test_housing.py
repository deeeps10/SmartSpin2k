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

# 'Sketch1': 4 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((7.2998, -8.2393), (18.093, 2.5539), -10.7932)
        RadiusArc((18.093, 2.5539), (7.2998, 13.347), -10.7932)
        Line((7.2998, 13.347), (-7.3002, 13.347))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-7.3002, 13.347), (-18.0934, 2.5539), -10.7932)
        RadiusArc((-18.0934, 2.5539), (-7.3002, -8.2393), -10.7932)
        Line((-7.3002, -8.2393), (7.2998, -8.2393))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 4 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 8.4),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((7.3002, 13.347), (-7.2998, 13.347))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-7.2998, 13.347), (-18.093, 2.5539), -10.7932)
        RadiusArc((-18.093, 2.5539), (-7.2998, -8.2393), -10.7932)
        Line((-7.2998, -8.2393), (7.3002, -8.2393))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((7.3002, -8.2393), (18.0934, 2.5539), -10.7932)
        RadiusArc((18.0934, 2.5539), (7.3002, 13.347), -10.7932)
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

_solid_sk_Sketch2_2 = extrude(_face_sk_Sketch2_2, amount=4.4, dir=Vector(0.0, 0.0, 1.0), taper=30.0).solid()

# 'Sketch3': 6 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 13.2),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((-5.0256, 7.4985), (-12.1667, 2.5539))
        Line((-12.1667, 2.5539), (-5.0256, -2.3908))
        Line((-5.0256, -2.3908), (-5.0256, -1.6773))
        Line((-5.0256, -1.6773), (-11.1363, 2.5539))
        Line((-11.1363, 2.5539), (-5.0256, 6.7851))
        Line((-5.0256, 6.7851), (-5.0256, 7.4985))
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

_solid_sk_Sketch3_3 = extrude(_face_sk_Sketch3_3, amount=-0.4, dir=Vector(0.0, 0.0, 1.0), taper=-26.57).solid()

# 'Sketch4': 6 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 0.0, 13.2),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        Line((5.0259, -1.6773), (5.0259, -2.3908))
        Line((5.0259, -2.3908), (12.1671, 2.5539))
        Line((12.1671, 2.5539), (5.0259, 7.4985))
        Line((5.0259, 7.4985), (5.0259, 6.7851))
        Line((5.0259, 6.7851), (11.1367, 2.5539))
        Line((11.1367, 2.5539), (5.0259, -1.6773))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

_solid_sk_Sketch4_4 = extrude(_face_sk_Sketch4_4, amount=-0.4, dir=Vector(0.0, 0.0, 1.0), taper=-26.57).solid()

# 'Sketch7': 4 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 0.0, 8.2),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch7_5:
    with BuildLine():
        Line((-1.1998, -9.3039), (-13.3998, -9.3039))
        Line((-13.3998, -9.3039), (-13.3998, 4.1961))
        Line((-13.3998, 4.1961), (-1.1998, 4.1961))
        Line((-1.1998, 4.1961), (-1.1998, -9.3039))
    _inc_edges_sk_Sketch7_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_5 = Wire.combine(_inc_edges_sk_Sketch7_5)[0]
_wire_sk_Sketch7_5 = _wire_sk_Sketch7_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch7_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch7_5.wrapped, True)
_face_sk_Sketch7_5 = Face(_mkf_sk_Sketch7_5.Face())

# 'Sketch7': 4 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 8.2),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch7_6:
    with BuildLine():
        Line((13.4002, -9.3039), (1.2002, -9.3039))
        Line((1.2002, -9.3039), (1.2002, 4.1961))
        Line((1.2002, 4.1961), (13.4002, 4.1961))
        Line((13.4002, 4.1961), (13.4002, -9.3039))
    _inc_edges_sk_Sketch7_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch7_6 = Wire.combine(_inc_edges_sk_Sketch7_6)[0]
_wire_sk_Sketch7_6 = _wire_sk_Sketch7_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch7_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch7_6.wrapped, True)
_face_sk_Sketch7_6 = Face(_mkf_sk_Sketch7_6.Face())

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 18.5039, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch8_7:
    with BuildLine():
        Line((7.5502, 0.0), (7.0502, 0.0))
        Line((7.0502, 0.0), (7.0502, -0.7137))
        # Arc split: sweep=347.52deg >= 150 — emitted as two half-arcs
        RadiusArc((7.0502, -0.7137), (7.3002, -5.3), -2.3)
        RadiusArc((7.3002, -5.3), (7.5502, -0.7137), -2.3)
        Line((7.5502, -0.7137), (7.5502, 0.0))
    _inc_edges_sk_Sketch8_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_7 = Wire.combine(_inc_edges_sk_Sketch8_7)[0]
_wire_sk_Sketch8_7 = _wire_sk_Sketch8_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch8_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch8_7.wrapped, True)
_face_sk_Sketch8_7 = Face(_mkf_sk_Sketch8_7.Face())

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 18.5039, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch8_8:
    with BuildLine():
        Line((-7.5498, 0.0), (-7.5498, -0.7137))
        # Arc split: sweep=347.52deg >= 150 — emitted as two half-arcs
        RadiusArc((-7.5498, -0.7137), (-7.2998, -5.3), -2.3)
        RadiusArc((-7.2998, -5.3), (-7.0498, -0.7137), -2.3)
        Line((-7.0498, -0.7137), (-7.0498, -0.0))
        Line((-7.0498, -0.0), (-7.5498, 0.0))
    _inc_edges_sk_Sketch8_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_8 = Wire.combine(_inc_edges_sk_Sketch8_8)[0]
_wire_sk_Sketch8_8 = _wire_sk_Sketch8_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch8_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch8_8.wrapped, True)
_face_sk_Sketch8_8 = Face(_mkf_sk_Sketch8_8.Face())

# 'Sketch9': circle on inclined plane
_inclined_plane_9 = Plane(
    origin=Vector(0.0, 18.5039, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch9_9:
    with Locations((7.3002, -3.0)):
        Circle(radius=1.5)

# 'Sketch9': circle on inclined plane
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 18.5039, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch9_10:
    with Locations((-7.2998, -3.0)):
        Circle(radius=1.5)

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_11 = Plane(
    origin=Vector(0.0, 0.0, 7.8),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_11) as sk_Sketch10_11:
    with BuildLine():
        Line((10.2998, -4.1961), (11.0998, -4.1961))
        Line((11.0998, -4.1961), (11.0998, 9.3039))
        Line((11.0998, 9.3039), (10.2998, 9.3039))
        Line((10.2998, 9.3039), (10.2998, -4.1961))
    _inc_edges_sk_Sketch10_11 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_11 = Wire.combine(_inc_edges_sk_Sketch10_11)[0]
_wire_sk_Sketch10_11 = _wire_sk_Sketch10_11.moved(_inclined_plane_11.location)
_mkf_sk_Sketch10_11 = BRepBuilderAPI_MakeFace(_inclined_plane_11.wrapped, _wire_sk_Sketch10_11.wrapped, True)
_face_sk_Sketch10_11 = Face(_mkf_sk_Sketch10_11.Face())

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_12 = Plane(
    origin=Vector(0.0, 0.0, 7.8),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_12) as sk_Sketch10_12:
    with BuildLine():
        Line((3.4998, -4.1961), (4.2998, -4.1961))
        Line((4.2998, -4.1961), (4.2998, 9.3039))
        Line((4.2998, 9.3039), (3.4998, 9.3039))
        Line((3.4998, 9.3039), (3.4998, -4.1961))
    _inc_edges_sk_Sketch10_12 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_12 = Wire.combine(_inc_edges_sk_Sketch10_12)[0]
_wire_sk_Sketch10_12 = _wire_sk_Sketch10_12.moved(_inclined_plane_12.location)
_mkf_sk_Sketch10_12 = BRepBuilderAPI_MakeFace(_inclined_plane_12.wrapped, _wire_sk_Sketch10_12.wrapped, True)
_face_sk_Sketch10_12 = Face(_mkf_sk_Sketch10_12.Face())

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_13 = Plane(
    origin=Vector(0.0, 0.0, 7.8),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_13) as sk_Sketch10_13:
    with BuildLine():
        Line((-4.3002, -4.1961), (-3.5002, -4.1961))
        Line((-3.5002, -4.1961), (-3.5002, 9.3039))
        Line((-3.5002, 9.3039), (-4.3002, 9.3039))
        Line((-4.3002, 9.3039), (-4.3002, -4.1961))
    _inc_edges_sk_Sketch10_13 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_13 = Wire.combine(_inc_edges_sk_Sketch10_13)[0]
_wire_sk_Sketch10_13 = _wire_sk_Sketch10_13.moved(_inclined_plane_13.location)
_mkf_sk_Sketch10_13 = BRepBuilderAPI_MakeFace(_inclined_plane_13.wrapped, _wire_sk_Sketch10_13.wrapped, True)
_face_sk_Sketch10_13 = Face(_mkf_sk_Sketch10_13.Face())

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_14 = Plane(
    origin=Vector(0.0, 0.0, 7.8),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_14) as sk_Sketch10_14:
    with BuildLine():
        Line((-11.1002, -4.1961), (-10.3002, -4.1961))
        Line((-10.3002, -4.1961), (-10.3002, 9.3039))
        Line((-10.3002, 9.3039), (-11.1002, 9.3039))
        Line((-11.1002, 9.3039), (-11.1002, -4.1961))
    _inc_edges_sk_Sketch10_14 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_14 = Wire.combine(_inc_edges_sk_Sketch10_14)[0]
_wire_sk_Sketch10_14 = _wire_sk_Sketch10_14.moved(_inclined_plane_14.location)
_mkf_sk_Sketch10_14 = BRepBuilderAPI_MakeFace(_inclined_plane_14.wrapped, _wire_sk_Sketch10_14.wrapped, True)
_face_sk_Sketch10_14 = Face(_mkf_sk_Sketch10_14.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_15 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_15) as sk_Sketch11_15:
    with BuildLine():
        Line((-13.4002, 5.5539), (-11.1002, 5.5539))
        Line((-11.1002, 5.5539), (-11.1002, 6.3539))
        Line((-11.1002, 6.3539), (-13.4002, 6.3539))
        Line((-13.4002, 6.3539), (-13.4002, 5.5539))
    _inc_edges_sk_Sketch11_15 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_15 = Wire.combine(_inc_edges_sk_Sketch11_15)[0]
_wire_sk_Sketch11_15 = _wire_sk_Sketch11_15.moved(_inclined_plane_15.location)
_mkf_sk_Sketch11_15 = BRepBuilderAPI_MakeFace(_inclined_plane_15.wrapped, _wire_sk_Sketch11_15.wrapped, True)
_face_sk_Sketch11_15 = Face(_mkf_sk_Sketch11_15.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_16 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_16) as sk_Sketch11_16:
    with BuildLine():
        Line((-13.4002, -1.2461), (-11.1002, -1.2461))
        Line((-11.1002, -1.2461), (-11.1002, -0.4461))
        Line((-11.1002, -0.4461), (-13.4002, -0.4461))
        Line((-13.4002, -0.4461), (-13.4002, -1.2461))
    _inc_edges_sk_Sketch11_16 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_16 = Wire.combine(_inc_edges_sk_Sketch11_16)[0]
_wire_sk_Sketch11_16 = _wire_sk_Sketch11_16.moved(_inclined_plane_16.location)
_mkf_sk_Sketch11_16 = BRepBuilderAPI_MakeFace(_inclined_plane_16.wrapped, _wire_sk_Sketch11_16.wrapped, True)
_face_sk_Sketch11_16 = Face(_mkf_sk_Sketch11_16.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_17 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_17) as sk_Sketch11_17:
    with BuildLine():
        Line((-10.3002, -1.2461), (-4.3002, -1.2461))
        Line((-4.3002, -1.2461), (-4.3002, -0.4461))
        Line((-4.3002, -0.4461), (-10.3002, -0.4461))
        Line((-10.3002, -0.4461), (-10.3002, -1.2461))
    _inc_edges_sk_Sketch11_17 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_17 = Wire.combine(_inc_edges_sk_Sketch11_17)[0]
_wire_sk_Sketch11_17 = _wire_sk_Sketch11_17.moved(_inclined_plane_17.location)
_mkf_sk_Sketch11_17 = BRepBuilderAPI_MakeFace(_inclined_plane_17.wrapped, _wire_sk_Sketch11_17.wrapped, True)
_face_sk_Sketch11_17 = Face(_mkf_sk_Sketch11_17.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_18 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_18) as sk_Sketch11_18:
    with BuildLine():
        Line((-10.3002, 5.5539), (-4.3002, 5.5539))
        Line((-4.3002, 5.5539), (-4.3002, 6.3539))
        Line((-4.3002, 6.3539), (-10.3002, 6.3539))
        Line((-10.3002, 6.3539), (-10.3002, 5.5539))
    _inc_edges_sk_Sketch11_18 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_18 = Wire.combine(_inc_edges_sk_Sketch11_18)[0]
_wire_sk_Sketch11_18 = _wire_sk_Sketch11_18.moved(_inclined_plane_18.location)
_mkf_sk_Sketch11_18 = BRepBuilderAPI_MakeFace(_inclined_plane_18.wrapped, _wire_sk_Sketch11_18.wrapped, True)
_face_sk_Sketch11_18 = Face(_mkf_sk_Sketch11_18.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_19 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_19) as sk_Sketch11_19:
    with BuildLine():
        Line((-3.5002, 5.5539), (-1.2002, 5.5539))
        Line((-1.2002, 5.5539), (-1.2002, 6.3539))
        Line((-1.2002, 6.3539), (-3.5002, 6.3539))
        Line((-3.5002, 6.3539), (-3.5002, 5.5539))
    _inc_edges_sk_Sketch11_19 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_19 = Wire.combine(_inc_edges_sk_Sketch11_19)[0]
_wire_sk_Sketch11_19 = _wire_sk_Sketch11_19.moved(_inclined_plane_19.location)
_mkf_sk_Sketch11_19 = BRepBuilderAPI_MakeFace(_inclined_plane_19.wrapped, _wire_sk_Sketch11_19.wrapped, True)
_face_sk_Sketch11_19 = Face(_mkf_sk_Sketch11_19.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_20 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_20) as sk_Sketch11_20:
    with BuildLine():
        Line((-3.5002, -1.2461), (-1.2002, -1.2461))
        Line((-1.2002, -1.2461), (-1.2002, -0.4461))
        Line((-1.2002, -0.4461), (-3.5002, -0.4461))
        Line((-3.5002, -0.4461), (-3.5002, -1.2461))
    _inc_edges_sk_Sketch11_20 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_20 = Wire.combine(_inc_edges_sk_Sketch11_20)[0]
_wire_sk_Sketch11_20 = _wire_sk_Sketch11_20.moved(_inclined_plane_20.location)
_mkf_sk_Sketch11_20 = BRepBuilderAPI_MakeFace(_inclined_plane_20.wrapped, _wire_sk_Sketch11_20.wrapped, True)
_face_sk_Sketch11_20 = Face(_mkf_sk_Sketch11_20.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_21 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_21) as sk_Sketch11_21:
    with BuildLine():
        Line((1.1998, 5.5539), (3.4998, 5.5539))
        Line((3.4998, 5.5539), (3.4998, 6.3539))
        Line((3.4998, 6.3539), (1.1998, 6.3539))
        Line((1.1998, 6.3539), (1.1998, 5.5539))
    _inc_edges_sk_Sketch11_21 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_21 = Wire.combine(_inc_edges_sk_Sketch11_21)[0]
_wire_sk_Sketch11_21 = _wire_sk_Sketch11_21.moved(_inclined_plane_21.location)
_mkf_sk_Sketch11_21 = BRepBuilderAPI_MakeFace(_inclined_plane_21.wrapped, _wire_sk_Sketch11_21.wrapped, True)
_face_sk_Sketch11_21 = Face(_mkf_sk_Sketch11_21.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_22 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_22) as sk_Sketch11_22:
    with BuildLine():
        Line((1.1998, -1.2461), (3.4998, -1.2461))
        Line((3.4998, -1.2461), (3.4998, -0.4461))
        Line((3.4998, -0.4461), (1.1998, -0.4461))
        Line((1.1998, -0.4461), (1.1998, -1.2461))
    _inc_edges_sk_Sketch11_22 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_22 = Wire.combine(_inc_edges_sk_Sketch11_22)[0]
_wire_sk_Sketch11_22 = _wire_sk_Sketch11_22.moved(_inclined_plane_22.location)
_mkf_sk_Sketch11_22 = BRepBuilderAPI_MakeFace(_inclined_plane_22.wrapped, _wire_sk_Sketch11_22.wrapped, True)
_face_sk_Sketch11_22 = Face(_mkf_sk_Sketch11_22.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_23 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_23) as sk_Sketch11_23:
    with BuildLine():
        Line((4.2998, -1.2461), (10.2998, -1.2461))
        Line((10.2998, -1.2461), (10.2998, -0.4461))
        Line((10.2998, -0.4461), (4.2998, -0.4461))
        Line((4.2998, -0.4461), (4.2998, -1.2461))
    _inc_edges_sk_Sketch11_23 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_23 = Wire.combine(_inc_edges_sk_Sketch11_23)[0]
_wire_sk_Sketch11_23 = _wire_sk_Sketch11_23.moved(_inclined_plane_23.location)
_mkf_sk_Sketch11_23 = BRepBuilderAPI_MakeFace(_inclined_plane_23.wrapped, _wire_sk_Sketch11_23.wrapped, True)
_face_sk_Sketch11_23 = Face(_mkf_sk_Sketch11_23.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_24 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_24) as sk_Sketch11_24:
    with BuildLine():
        Line((4.2998, 5.5539), (10.2998, 5.5539))
        Line((10.2998, 5.5539), (10.2998, 6.3539))
        Line((10.2998, 6.3539), (4.2998, 6.3539))
        Line((4.2998, 6.3539), (4.2998, 5.5539))
    _inc_edges_sk_Sketch11_24 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_24 = Wire.combine(_inc_edges_sk_Sketch11_24)[0]
_wire_sk_Sketch11_24 = _wire_sk_Sketch11_24.moved(_inclined_plane_24.location)
_mkf_sk_Sketch11_24 = BRepBuilderAPI_MakeFace(_inclined_plane_24.wrapped, _wire_sk_Sketch11_24.wrapped, True)
_face_sk_Sketch11_24 = Face(_mkf_sk_Sketch11_24.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_25 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_25) as sk_Sketch11_25:
    with BuildLine():
        Line((11.0998, 5.5539), (13.3998, 5.5539))
        Line((13.3998, 5.5539), (13.3998, 6.3539))
        Line((13.3998, 6.3539), (11.0998, 6.3539))
        Line((11.0998, 6.3539), (11.0998, 5.5539))
    _inc_edges_sk_Sketch11_25 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_25 = Wire.combine(_inc_edges_sk_Sketch11_25)[0]
_wire_sk_Sketch11_25 = _wire_sk_Sketch11_25.moved(_inclined_plane_25.location)
_mkf_sk_Sketch11_25 = BRepBuilderAPI_MakeFace(_inclined_plane_25.wrapped, _wire_sk_Sketch11_25.wrapped, True)
_face_sk_Sketch11_25 = Face(_mkf_sk_Sketch11_25.Face())

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_26 = Plane(
    origin=Vector(0.0, 0.0, 8.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_26) as sk_Sketch11_26:
    with BuildLine():
        Line((11.0998, -1.2461), (13.3998, -1.2461))
        Line((13.3998, -1.2461), (13.3998, -0.4461))
        Line((13.3998, -0.4461), (11.0998, -0.4461))
        Line((11.0998, -0.4461), (11.0998, -1.2461))
    _inc_edges_sk_Sketch11_26 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_26 = Wire.combine(_inc_edges_sk_Sketch11_26)[0]
_wire_sk_Sketch11_26 = _wire_sk_Sketch11_26.moved(_inclined_plane_26.location)
_mkf_sk_Sketch11_26 = BRepBuilderAPI_MakeFace(_inclined_plane_26.wrapped, _wire_sk_Sketch11_26.wrapped, True)
_face_sk_Sketch11_26 = Face(_mkf_sk_Sketch11_26.Face())

# 'Sketch12': 5 segments → Line/RadiusArc profile
_inclined_plane_27 = Plane(
    origin=Vector(-7.5498, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_27) as sk_Sketch12_27:
    with BuildLine():
        Line((-18.462, 0.7137), (-24.0395, 0.7137))
        Line((-24.0395, 0.7137), (-24.0395, -2.9211))
        Line((-24.0395, -2.9211), (-17.5039, -2.9211))
        Line((-17.5039, -2.9211), (-17.5039, 0.0))
        RadiusArc((-17.5039, 0.0), (-18.462, 0.7137), 1.0)
    _inc_edges_sk_Sketch12_27 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch12_27 = Wire.combine(_inc_edges_sk_Sketch12_27)[0]
_wire_sk_Sketch12_27 = _wire_sk_Sketch12_27.moved(_inclined_plane_27.location)
_mkf_sk_Sketch12_27 = BRepBuilderAPI_MakeFace(_inclined_plane_27.wrapped, _wire_sk_Sketch12_27.wrapped, True)
_face_sk_Sketch12_27 = Face(_mkf_sk_Sketch12_27.Face())

# 'Sketch13': 4 segments → Line/RadiusArc profile
_inclined_plane_28 = Plane(
    origin=Vector(0.0, 0.0, 8.2),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_28) as sk_Sketch13_28:
    with BuildLine():
        Line((4.2998, 5.5539), (4.2998, -0.4461))
        Line((4.2998, -0.4461), (10.2998, -0.4461))
        Line((10.2998, -0.4461), (10.2998, 5.5539))
        Line((10.2998, 5.5539), (4.2998, 5.5539))
    _inc_edges_sk_Sketch13_28 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_28 = Wire.combine(_inc_edges_sk_Sketch13_28)[0]
_wire_sk_Sketch13_28 = _wire_sk_Sketch13_28.moved(_inclined_plane_28.location)
_mkf_sk_Sketch13_28 = BRepBuilderAPI_MakeFace(_inclined_plane_28.wrapped, _wire_sk_Sketch13_28.wrapped, True)
_face_sk_Sketch13_28 = Face(_mkf_sk_Sketch13_28.Face())

# 'Sketch13': 4 segments → Line/RadiusArc profile
_inclined_plane_29 = Plane(
    origin=Vector(0.0, 0.0, 8.2),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_29) as sk_Sketch13_29:
    with BuildLine():
        Line((-4.3002, -0.4461), (-4.3002, 5.5539))
        Line((-4.3002, 5.5539), (-10.3002, 5.5539))
        Line((-10.3002, 5.5539), (-10.3002, -0.4461))
        Line((-10.3002, -0.4461), (-4.3002, -0.4461))
    _inc_edges_sk_Sketch13_29 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_29 = Wire.combine(_inc_edges_sk_Sketch13_29)[0]
_wire_sk_Sketch13_29 = _wire_sk_Sketch13_29.moved(_inclined_plane_29.location)
_mkf_sk_Sketch13_29 = BRepBuilderAPI_MakeFace(_inclined_plane_29.wrapped, _wire_sk_Sketch13_29.wrapped, True)
_face_sk_Sketch13_29 = Face(_mkf_sk_Sketch13_29.Face())

_inclined_plane_30 = Plane(
    origin=Vector(-7.2998, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
# 'Sketch14': 7 segments → revolve profile
with BuildSketch(_inclined_plane_30) as sk_Sketch14_29:
    with BuildLine():
        Line((-11.9789, 4.5), (-10.0539, 4.5))
        RadiusArc((-10.0539, 4.5), (-9.3039, 5.25), -0.75)
        Line((-9.3039, 5.25), (-9.3039, 6.0831))
        Line((-9.3039, 6.0831), (-8.3502, 6.0831))
        Line((-8.3502, 6.0831), (-8.3502, 3.3334))
        Line((-8.3502, 3.3334), (-11.9789, 3.3334))
        Line((-11.9789, 3.3334), (-11.9789, 4.5))
    make_face()
_inclined_plane_31 = Plane(
    origin=Vector(7.3002, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
# 'Sketch15': 7 segments → revolve profile
with BuildSketch(_inclined_plane_31) as sk_Sketch15_30:
    with BuildLine():
        Line((11.9789, 4.5), (10.0539, 4.5))
        RadiusArc((10.0539, 4.5), (9.3039, 5.25), 0.75)
        Line((9.3039, 5.25), (9.3039, 6.0831))
        Line((9.3039, 6.0831), (8.3502, 6.0831))
        Line((8.3502, 6.0831), (8.3502, 3.3334))
        Line((8.3502, 3.3334), (11.9789, 3.3334))
        Line((11.9789, 3.3334), (11.9789, 4.5))
    make_face()
# 'Sketch16': 4 segments → Line/RadiusArc profile
_inclined_plane_32 = Plane(
    origin=Vector(0.0, 9.3039, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_32) as sk_Sketch16_32:
    with BuildLine():
        Line((-8.9767, -0.75), (-5.6268, -0.75))
        Line((-5.6268, -0.75), (-5.6268, -1.5008))
        Line((-5.6268, -1.5008), (-8.9767, -1.5008))
        Line((-8.9767, -1.5008), (-8.9767, -0.75))
    _inc_edges_sk_Sketch16_32 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16_32 = Wire.combine(_inc_edges_sk_Sketch16_32)[0]
_wire_sk_Sketch16_32 = _wire_sk_Sketch16_32.moved(_inclined_plane_32.location)
_mkf_sk_Sketch16_32 = BRepBuilderAPI_MakeFace(_inclined_plane_32.wrapped, _wire_sk_Sketch16_32.wrapped, True)
_face_sk_Sketch16_32 = Face(_mkf_sk_Sketch16_32.Face())

# 'Sketch16': 4 segments → Line/RadiusArc profile
_inclined_plane_33 = Plane(
    origin=Vector(0.0, 9.3039, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_33) as sk_Sketch16_33:
    with BuildLine():
        Line((5.6285, -0.75), (8.9768, -0.75))
        Line((8.9768, -0.75), (8.9768, -1.5))
        Line((8.9768, -1.5), (5.6285, -1.5))
        Line((5.6285, -1.5), (5.6285, -0.75))
    _inc_edges_sk_Sketch16_33 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16_33 = Wire.combine(_inc_edges_sk_Sketch16_33)[0]
_wire_sk_Sketch16_33 = _wire_sk_Sketch16_33.moved(_inclined_plane_33.location)
_mkf_sk_Sketch16_33 = BRepBuilderAPI_MakeFace(_inclined_plane_33.wrapped, _wire_sk_Sketch16_33.wrapped, True)
_face_sk_Sketch16_33 = Face(_mkf_sk_Sketch16_33.Face())

# 'Sketch18': 4 segments → Line/RadiusArc profile
_inclined_plane_34 = Plane(
    origin=Vector(0.0, 0.0, 11.6),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_34) as sk_Sketch18_34:
    with BuildLine():
        Line((-7.3002, -5.468), (7.2998, -5.468))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((7.2998, -5.468), (15.3217, 2.5539), -8.0219)
        RadiusArc((15.3217, 2.5539), (7.2998, 10.5757), -8.0219)
        Line((7.2998, 10.5757), (-7.3002, 10.5757))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-7.3002, 10.5757), (-15.3221, 2.5539), -8.0219)
        RadiusArc((-15.3221, 2.5539), (-7.3002, -5.468), -8.0219)
    _inc_edges_sk_Sketch18_34 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch18_34 = Wire.combine(_inc_edges_sk_Sketch18_34)[0]
_wire_sk_Sketch18_34 = _wire_sk_Sketch18_34.moved(_inclined_plane_34.location)
_mkf_sk_Sketch18_34 = BRepBuilderAPI_MakeFace(_inclined_plane_34.wrapped, _wire_sk_Sketch18_34.wrapped, True)
_face_sk_Sketch18_34 = Face(_mkf_sk_Sketch18_34.Face())

_solid_sk_Sketch18_34 = extrude(_face_sk_Sketch18_34, amount=2.4, dir=Vector(0.0, 0.0, -1.0), taper=-30.0).solid()

# 'Sketch19': 4 segments → Line/RadiusArc profile
_inclined_plane_35 = Plane(
    origin=Vector(-0.7998, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_35) as sk_Sketch19_35:
    with BuildLine():
        Line((-10.5757, 11.6), (5.468, 11.6))
        Line((5.468, 11.6), (6.8537, 9.2))
        Line((6.8537, 9.2), (-11.9614, 9.2))
        Line((-11.9614, 9.2), (-10.5757, 11.6))
    _inc_edges_sk_Sketch19_35 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch19_35 = Wire.combine(_inc_edges_sk_Sketch19_35)[0]
_wire_sk_Sketch19_35 = _wire_sk_Sketch19_35.moved(_inclined_plane_35.location)
_mkf_sk_Sketch19_35 = BRepBuilderAPI_MakeFace(_inclined_plane_35.wrapped, _wire_sk_Sketch19_35.wrapped, True)
_face_sk_Sketch19_35 = Face(_mkf_sk_Sketch19_35.Face())

# 'Sketch5': 5 segments → Line/RadiusArc profile
_inclined_plane_36 = Plane(
    origin=Vector(0.0, -8.2393, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_36) as sk_Sketch5_36:
    with BuildLine():
        Line((1.2498, -13.415), (1.2498, -12.8))
        Line((1.2498, -12.8), (-0.0002, -11.55))
        Line((-0.0002, -11.55), (-1.2502, -12.8))
        Line((-1.2502, -12.8), (-1.2502, -13.415))
        Line((-1.2502, -13.415), (1.2498, -13.415))
    _inc_edges_sk_Sketch5_36 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_36 = Wire.combine(_inc_edges_sk_Sketch5_36)[0]
_wire_sk_Sketch5_36 = _wire_sk_Sketch5_36.moved(_inclined_plane_36.location)
_mkf_sk_Sketch5_36 = BRepBuilderAPI_MakeFace(_inclined_plane_36.wrapped, _wire_sk_Sketch5_36.wrapped, True)
_face_sk_Sketch5_36 = Face(_mkf_sk_Sketch5_36.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -8.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -8.400000073 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _solid = _solid_sk_Sketch2_2
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 4.400000833 mm
    # Fusion taper angle expression: -30.00000 deg
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_3
    _solid = _solid_sk_Sketch3_3
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    # Fusion taper angle expression: 26.57 deg
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _solid = _solid_sk_Sketch4_4
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    # Fusion taper angle expression: 26.57 deg
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6_p0 --
    _face = _face_sk_Sketch7_5
    _vec = Vector(0.0, 0.0, -1.0) * 19.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 19.000000 mm
    
    # -- Extrude6_p1 --
    _face = _face_sk_Sketch7_6
    _vec = Vector(0.0, 0.0, -1.0) * 19.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 19.000000 mm
    
    # --- FEATURE: Extrude7 ---
    # -- Extrude7_p0 --
    _face = _face_sk_Sketch8_7
    _vec = Vector(-0.0, 1.0, 0.0) * -6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -6.000000 mm
    
    # -- Extrude7_p1 --
    _face = _face_sk_Sketch8_8
    _vec = Vector(-0.0, 1.0, 0.0) * -6.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -6.000000 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8_p0 --
    extrude(sk_Sketch9_9.sketch, amount=-11.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -11.000000 mm
    
    # -- Extrude8_p1 --
    extrude(sk_Sketch9_10.sketch, amount=-11.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: -11.000000 mm
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9_p0 --
    _face = _face_sk_Sketch10_11
    _vec = Vector(0.0, 0.0, -1.0) * -0.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    
    # -- Extrude9_p1 --
    _face = _face_sk_Sketch10_12
    _vec = Vector(0.0, 0.0, -1.0) * -0.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    
    # -- Extrude9_p2 --
    _face = _face_sk_Sketch10_13
    _vec = Vector(0.0, 0.0, -1.0) * -0.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    
    # -- Extrude9_p3 --
    _face = _face_sk_Sketch10_14
    _vec = Vector(0.0, 0.0, -1.0) * -0.4
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.399999619 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10_p0 --
    _face = _face_sk_Sketch11_15
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p1 --
    _face = _face_sk_Sketch11_16
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p2 --
    _face = _face_sk_Sketch11_17
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p3 --
    _face = _face_sk_Sketch11_18
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p4 --
    _face = _face_sk_Sketch11_19
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p5 --
    _face = _face_sk_Sketch11_20
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p6 --
    _face = _face_sk_Sketch11_21
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p7 --
    _face = _face_sk_Sketch11_22
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p8 --
    _face = _face_sk_Sketch11_23
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p9 --
    _face = _face_sk_Sketch11_24
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p10 --
    _face = _face_sk_Sketch11_25
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # -- Extrude10_p11 --
    _face = _face_sk_Sketch11_26
    _vec = Vector(0.0, 0.0, -1.0) * -0.2
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -0.199999809 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch12_27
    _vec = Vector(-1.0, 0.0, 0.0) * -16.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -16.000000 mm
    
    # --- FEATURE: Extrude12 ---
    # -- Extrude12_p0 --
    _face = _face_sk_Sketch13_28
    _vec = Vector(0.0, 0.0, -1.0) * -1.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -1.00000024 mm
    
    # -- Extrude12_p1 --
    _face = _face_sk_Sketch13_29
    _vec = Vector(0.0, 0.0, -1.0) * -1.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -1.00000024 mm
    
    # --- FEATURE: Revolve1 ---
    # -- Revolve1 --
    _custom_axis = Axis(
        Vector(-7.2998, 18.5039, 3.0),
        Vector(0.0, -1.0, 0.0),
    )
    revolve(sk_Sketch14_29.sketch.faces(), axis=_custom_axis, mode=Mode.SUBTRACT)
    
    # --- FEATURE: Revolve2 ---
    # -- Revolve2 --
    _custom_axis = Axis(
        Vector(7.3002, 18.5039, 3.0),
        Vector(0.0, -1.0, 0.0),
    )
    revolve(sk_Sketch15_30.sketch.faces(), axis=_custom_axis, mode=Mode.SUBTRACT)
    
    # --- FEATURE: Extrude13 ---
    # -- Extrude13_p0 --
    _face = _face_sk_Sketch16_32
    _vec = Vector(-0.0, -1.0, -0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -2.000000 mm
    
    # -- Extrude13_p1 --
    _face = _face_sk_Sketch16_33
    _vec = Vector(-0.0, -1.0, -0.0) * -2.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -2.000000 mm
    
    # --- FEATURE: Extrude14 ---
    # -- Extrude14 --
    _face = _face_sk_Sketch18_34
    _solid = _solid_sk_Sketch18_34
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 2.40000074 mm
    # Fusion taper angle expression: 30.00000 deg
    
    # --- FEATURE: Extrude15 ---
    # -- Extrude15 --
    _face = _face_sk_Sketch19_35
    _vec = Vector(-1.0, 0.0, 0.0) * -1.6
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -1.6000000387 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    _face = _face_sk_Sketch5_36
    _vec = Vector(-0.0, -1.0, -0.0) * -23.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -23.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
