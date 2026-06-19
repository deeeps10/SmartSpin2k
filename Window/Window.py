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

_inclined_plane_1 = Plane(
    origin=Vector(3.7, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
# 'Sketch4': 5 segments → revolve profile
with BuildSketch(_inclined_plane_1) as sk_Sketch4_0:
    with BuildLine():
        RadiusArc((5.7, 8.9995), (6.9, 10.8325), -2.0)
        Line((6.9, 10.8325), (3.9, 10.8326))
        Line((3.9, 10.8326), (3.9, 6.25))
        RadiusArc((3.9, 6.25), (4.8711, 8.4599), 3.0)
        RadiusArc((4.8711, 8.4599), (5.7, 8.9995), 3.0)
    make_face()
# 'Sketch5': 4 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch5_2:
    with BuildLine():
        Line((7.521, -31.6271), (-3.7, -31.6271))
        Line((-3.7, -31.6271), (-3.7, 1.7664))
        Line((-3.7, 1.7664), (7.521, 1.7664))
        Line((7.521, 1.7664), (7.521, -31.6271))
    _inc_edges_sk_Sketch5_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch5_2 = Wire.combine(_inc_edges_sk_Sketch5_2)[0]
_wire_sk_Sketch5_2 = _wire_sk_Sketch5_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch5_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch5_2.wrapped, True)
_face_sk_Sketch5_2 = Face(_mkf_sk_Sketch5_2.Face())

# 'Sketch6': 5 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(3.7, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch6_3:
    with BuildLine():
        RadiusArc((-3.9, 6.25), (-4.8711, 8.4599), -3.0)
        RadiusArc((-4.8711, 8.4599), (-5.7, 8.9995), -3.0)
        RadiusArc((-5.7, 8.9995), (-6.9, 10.8325), 2.0)
        Line((-6.9, 10.8325), (-3.9, 10.8326))
        Line((-3.9, 10.8326), (-3.9, 6.25))
    _inc_edges_sk_Sketch6_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_3 = Wire.combine(_inc_edges_sk_Sketch6_3)[0]
_wire_sk_Sketch6_3 = _wire_sk_Sketch6_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch6_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch6_3.wrapped, True)
_face_sk_Sketch6_3 = Face(_mkf_sk_Sketch6_3.Face())

# 'Sketch6': 5 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(3.7, 0.0, 0.0),
    x_dir=Vector(0.0, -1.0, 0.0),
    z_dir=Vector(-1.0, 0.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch6_4:
    with BuildLine():
        RadiusArc((-5.7, 20.5004), (-6.9, 18.6675), -2.0)
        Line((-6.9, 18.6675), (-3.9, 18.6673))
        Line((-3.9, 18.6673), (-3.9, 23.2499))
        RadiusArc((-3.9, 23.2499), (-4.8711, 21.0401), 3.0)
        RadiusArc((-4.8711, 21.0401), (-5.7, 20.5004), 3.0)
    _inc_edges_sk_Sketch6_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_4 = Wire.combine(_inc_edges_sk_Sketch6_4)[0]
_wire_sk_Sketch6_4 = _wire_sk_Sketch6_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch6_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch6_4.wrapped, True)
_face_sk_Sketch6_4 = Face(_mkf_sk_Sketch6_4.Face())

# 'Sketch12': 12 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 6.9, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch12_5:
    with BuildLine():
        Line((3.7, -10.8325), (1.2, -10.8325))
        Line((1.2, -10.8325), (1.2, -11.75))
        Line((1.2, -11.75), (3.818, -11.75))
        RadiusArc((3.818, -11.75), (5.0645, -12.1859), 2.0)
        RadiusArc((5.0645, -12.1859), (6.097, -13.907), 3.3453)
        RadiusArc((6.097, -13.907), (5.9163, -16.1304), 3.5)
        RadiusArc((5.9163, -16.1304), (4.785, -17.5007), 2.7389)
        RadiusArc((4.785, -17.5007), (3.818, -17.75), 2.0)
        Line((3.818, -17.75), (1.2, -17.75))
        Line((1.2, -17.75), (1.2, -18.6675))
        Line((1.2, -18.6675), (3.7, -18.6675))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((3.7, -18.6675), (7.6175, -14.75), -3.9175)
        RadiusArc((7.6175, -14.75), (3.7, -10.8325), -3.9175)
    _inc_edges_sk_Sketch12_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch12_5 = Wire.combine(_inc_edges_sk_Sketch12_5)[0]
_wire_sk_Sketch12_5 = _wire_sk_Sketch12_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch12_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch12_5.wrapped, True)
_face_sk_Sketch12_5 = Face(_mkf_sk_Sketch12_5.Face())

# 'Sketch10': 4 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch10_6:
    with BuildLine():
        Line((-1.1, -28.4), (-37.8, -28.4))
        Line((-37.8, -28.4), (-37.8, -1.1))
        Line((-37.8, -1.1), (-1.1, -1.1))
        Line((-1.1, -1.1), (-1.1, -28.4))
    _inc_edges_sk_Sketch10_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch10_6 = Wire.combine(_inc_edges_sk_Sketch10_6)[0]
_wire_sk_Sketch10_6 = _wire_sk_Sketch10_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch10_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch10_6.wrapped, True)
_face_sk_Sketch10_6 = Face(_mkf_sk_Sketch10_6.Face())

_solid_sk_Sketch10_6 = extrude(_face_sk_Sketch10_6, amount=-1.1, dir=Vector(-0.0, -1.0, -0.0), taper=-45.0).solid()

# 'Sketch11': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 1.1, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch11_7:
    with BuildLine():
        Line((0.0, -29.5), (-38.9, -29.5))
        Line((-38.9, -29.5), (-38.9, 0.0))
        Line((-38.9, 0.0), (-0.0, 0.0))
        Line((-0.0, 0.0), (0.0, -29.5))
    _inc_edges_sk_Sketch11_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch11_7 = Wire.combine(_inc_edges_sk_Sketch11_7)[0]
_wire_sk_Sketch11_7 = _wire_sk_Sketch11_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch11_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch11_7.wrapped, True)
_face_sk_Sketch11_7 = Face(_mkf_sk_Sketch11_7.Face())

# 'Sketch13': 4 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 2.6, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch13_8:
    with BuildLine():
        Line((37.7, -28.3), (1.2, -28.3))
        Line((1.2, -28.3), (1.2, -1.2))
        Line((1.2, -1.2), (37.7, -1.2))
        Line((37.7, -1.2), (37.7, -28.3))
    _inc_edges_sk_Sketch13_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch13_8 = Wire.combine(_inc_edges_sk_Sketch13_8)[0]
_wire_sk_Sketch13_8 = _wire_sk_Sketch13_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch13_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch13_8.wrapped, True)
_face_sk_Sketch13_8 = Face(_mkf_sk_Sketch13_8.Face())

# 'Sketch14': 8 segments → Line/RadiusArc profile
_inclined_plane_9 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch14_9:
    with BuildLine():
        Line((-3.818, -11.75), (5.4724, -11.75))
        Line((5.4724, -11.75), (5.4724, -17.75))
        Line((5.4724, -17.75), (-3.818, -17.75))
        RadiusArc((-3.818, -17.75), (-4.4783, -17.6379), 2.0)
        RadiusArc((-4.4783, -17.6379), (-5.6512, -16.6315), 2.2167)
        RadiusArc((-5.6512, -16.6315), (-6.1885, -14.4665), 3.5)
        RadiusArc((-6.1885, -14.4665), (-5.3087, -12.4167), 3.5)
        RadiusArc((-5.3087, -12.4167), (-3.818, -11.75), 2.0)
    _inc_edges_sk_Sketch14_9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch14_9 = Wire.combine(_inc_edges_sk_Sketch14_9)[0]
_wire_sk_Sketch14_9 = _wire_sk_Sketch14_9.moved(_inclined_plane_9.location)
_mkf_sk_Sketch14_9 = BRepBuilderAPI_MakeFace(_inclined_plane_9.wrapped, _wire_sk_Sketch14_9.wrapped, True)
_face_sk_Sketch14_9 = Face(_mkf_sk_Sketch14_9.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Revolve1 ---
    # -- Revolve1 --
    _custom_axis = Axis(
        Vector(3.7, 8.7521, 14.75),
        Vector(0.0, -1.0, 0.0),
    )
    revolve(sk_Sketch4_0.sketch.faces(), axis=_custom_axis, mode=Mode.ADD)
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch5_2
    _vec = Vector(-0.0, -1.0, -0.0) * -19.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -19.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    _face = _face_sk_Sketch6_3
    _vec = Vector(-1.0, 0.0, 0.0) * 2.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 2.499999627 mm
    
    # -- Extrude4_p1 --
    _face = _face_sk_Sketch6_4
    _vec = Vector(-1.0, 0.0, 0.0) * 2.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 2.499999627 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6 --
    _face = _face_sk_Sketch12_5
    _vec = Vector(-0.0, 1.0, 0.0) * -3.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -2.9999818243 mm
    
    # --- FEATURE: Extrude8 ---
    # -- Extrude8 --
    _face = _face_sk_Sketch10_6
    _solid = _solid_sk_Sketch10_6
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -1.100000069 mm
    # Fusion taper angle expression: 45.00000 deg
    
    # --- FEATURE: Extrude9 ---
    # -- Extrude9 --
    _face = _face_sk_Sketch11_7
    _vec = Vector(-0.0, -1.0, -0.0) * -1.5
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -1.499999836 mm
    
    # --- FEATURE: Extrude10 ---
    # -- Extrude10 --
    _face = _face_sk_Sketch13_8
    _vec = Vector(-0.0, 1.0, 0.0) * 1.3
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: 1.299999952 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch14_9
    _vec = Vector(-0.0, -1.0, -0.0) * -10.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -10.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)

# -- Volume Display --
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
if part.part is not None:
    _vprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.part.wrapped, _vprops)
    print(f"\n  Volume of final part: {abs(_vprops.Mass()):.2f} mm³")
