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
    origin=Vector(0.0, 0.0, 1.7017),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
# 'Sketch4': 6 segments → revolve profile
with BuildSketch(_inclined_plane_1) as sk_Sketch4_0:
    with BuildLine():
        Line((-26.7454, 0.0), (-26.7454, 30.0))
        Line((-26.7454, 30.0), (-32.4441, 30.0))
        Line((-32.4441, 30.0), (-32.4441, -2.0))
        Line((-32.4441, -2.0), (2.2521, -2.0))
        Line((2.2521, -2.0), (2.2563, 0.0))
        Line((2.2563, 0.0), (-26.7454, 0.0))
    make_face()
# 'Sketch7': circle on inclined plane
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 25.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch7_3:
    with Locations((2.2547, -1.7017)):
        Circle(radius=26.5001)

# 'Sketch8': 8 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, -2.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch8_4:
    with BuildLine():
        Line((-2.8103, -36.4078), (-3.1181, -36.9881))
        Line((-3.1181, -36.9881), (1.3494, -36.9881))
        Line((1.3494, -36.9881), (1.0646, -36.2527))
        RadiusArc((1.0646, -36.2527), (0.5644, -35.9107), 0.5)
        RadiusArc((0.5644, -35.9107), (0.1969, -35.3176), -1.5)
        RadiusArc((0.1969, -35.3176), (-1.0948, -34.9011), -1.5)
        RadiusArc((-1.0948, -34.9011), (-2.3166, -36.0266), -1.5)
        RadiusArc((-2.3166, -36.0266), (-2.8103, -36.4078), 0.5)
    _inc_edges_sk_Sketch8_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_4 = Wire.combine(_inc_edges_sk_Sketch8_4)[0]
_wire_sk_Sketch8_4 = _wire_sk_Sketch8_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch8_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch8_4.wrapped, True)
_face_sk_Sketch8_4 = Face(_mkf_sk_Sketch8_4.Face())

# 'Sketch8': 11 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, -2.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch8_5:
    with BuildLine():
        Line((-1.7317, 33.0035), (-1.4148, 33.4088))
        Line((-1.4148, 33.4088), (-5.891, 33.4088))
        Line((-5.891, 33.4088), (-5.5913, 32.8481))
        RadiusArc((-5.5913, 32.8481), (-5.0686, 32.5077), 0.5)
        RadiusArc((-5.0686, 32.5077), (-4.7012, 31.9145), -1.5)
        RadiusArc((-4.7012, 31.9145), (-3.7605, 31.4838), -1.5)
        RadiusArc((-3.7605, 31.4838), (-2.5063, 32.0025), -1.5)
        RadiusArc((-2.5063, 32.0025), (-2.1876, 32.6231), -1.5)
        Line((-2.1876, 32.6231), (-2.1421, 32.7415))
        RadiusArc((-2.1421, 32.7415), (-1.971, 32.9258), 0.5)
        RadiusArc((-1.971, 32.9258), (-1.7317, 33.0035), 0.4999)
    _inc_edges_sk_Sketch8_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_5 = Wire.combine(_inc_edges_sk_Sketch8_5)[0]
_wire_sk_Sketch8_5 = _wire_sk_Sketch8_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch8_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch8_5.wrapped, True)
_face_sk_Sketch8_5 = Face(_mkf_sk_Sketch8_5.Face())

# 'Sketch6': 10 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch6_2:
    with BuildLine():
        RadiusArc((2.7012, 24.7945), (3.2338, 24.8428), -2.4997)
        RadiusArc((3.2338, 24.8428), (4.3433, 25.3723), -2.5114)
        RadiusArc((4.3433, 25.3723), (4.8847, 26.0039), -2.4409)
        RadiusArc((4.8847, 26.0039), (5.2412, 27.1441), -2.5626)
        RadiusArc((5.2412, 27.1441), (-11.2261, 23.9746), -29.0017)
        RadiusArc((-11.2261, 23.9746), (-10.4718, 23.0482), -2.5626)
        RadiusArc((-10.4718, 23.0482), (-9.5119, 22.6022), -2.4473)
        RadiusArc((-9.5119, 22.6022), (-8.5079, 22.5828), -2.5041)
        RadiusArc((-8.5079, 22.5828), (-7.9954, 22.7357), -2.4993)
        RadiusArc((-7.9954, 22.7357), (2.7012, 24.7945), 26.5)
    _inc_edges_sk_Sketch6_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6_2 = Wire.combine(_inc_edges_sk_Sketch6_2)[0]
_wire_sk_Sketch6_2 = _wire_sk_Sketch6_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch6_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch6_2.wrapped, True)
_face_sk_Sketch6_2 = Face(_mkf_sk_Sketch6_2.Face())

# -- Isolation buffer: body_Extrude2 (kind=body) --
with BuildPart() as body_Extrude2:
    # -- Extrude2 --
    _face = _face_sk_Sketch6_2
    _vec = Vector(-0.0, 1.0, 0.0) * -30.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -30.000000 mm
    

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Revolve1 ---
    # -- Revolve1 --
    _custom_axis = Axis(
        Vector(2.2563, 15.0, 1.7017),
        Vector(0.0, 1.0, 0.0),
    )
    revolve(sk_Sketch4_0.sketch.faces(), axis=_custom_axis, mode=Mode.ADD)
    
    # --- FEATURE: Extrude2 ---
    # -- Add Extrude2 (separate body) --
    add(body_Extrude2.part)
    
    # --- FEATURE: C-Pattern1 ---
    # -- C-Pattern1 (bodies: Body17) --
    _custom_axis = Axis(
        Vector(2.2563, 15.0, 1.7017),
        Vector(0.0, 1.0, 0.0),
    )
    # Axis: _custom_axis  count=6  step=60.0deg
    # Start at 1: _pat_i=0 (0°) is already added above as the separate body.
    for _pat_i in range(1, 6):
        if body_Extrude2.part is not None: add(body_Extrude2.part.rotate(_custom_axis, _pat_i * 60.0))
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    extrude(sk_Sketch7_3.sketch, amount=5.0, taper=-26.5, mode=Mode.SUBTRACT)
    # Fusion depth expression: 5.000000 mm
    # Fusion taper angle expression: 26.500000 deg
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4_p0 --
    _face = _face_sk_Sketch8_4
    _vec = Vector(-0.0, -1.0, -0.0) * -39.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -39.000000 mm
    
    # -- Extrude4_p1 --
    _face = _face_sk_Sketch8_5
    _vec = Vector(-0.0, -1.0, -0.0) * -39.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -39.000000 mm
    

# -- Export --
# export_step(part.part, 'fusion_features.step')
# export_stl(part.part,  'fusion_features.stl')
if _has_ocp:
    _disp = part.part
    if _disp is not None:
        # Step 1: fuse Compound sub-bodies → removes internal non-manifold faces
        if isinstance(_disp, Compound):
            _subs = list(_disp.solids())
            if _subs:
                _disp = _subs[0]
                for _s in _subs[1:]:
                    _disp = fuse_solids(_disp, _s)
        # Step 2: unify co-domain surfaces — merges degenerate pole/axis faces
        # created when the revolve profile touches the rotation axis exactly
        try:
            from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
            from OCP.TopAbs import TopAbs_SOLID
            from OCP.TopoDS import TopoDS
            _uni = ShapeUpgrade_UnifySameDomain(_disp.wrapped, True, True, True)
            _uni.Build()
            _us = _uni.Shape()
            if not _us.IsNull() and _us.ShapeType() == TopAbs_SOLID:
                _disp = Solid(TopoDS.Solid_s(_us))
        except Exception:
            pass
        # Step 3: ShapeFix with small-area wire removal — purges near-zero-area
        # faces left by boolean cuts on the tiny 0.5/1.5 mm arc features
        try:
            from OCP.ShapeFix import ShapeFix_Shape
            from OCP.TopAbs import TopAbs_SOLID
            from OCP.TopoDS import TopoDS
            _sf = ShapeFix_Shape(_disp.wrapped)
            try:
                _sf.FixWireTool().SetFixSmallAreaWireMode(1)
            except Exception:
                pass
            _sf.Perform()
            _fixed = _sf.Shape()
            if not _fixed.IsNull() and _fixed.ShapeType() == TopAbs_SOLID:
                _disp = Solid(TopoDS.Solid_s(_fixed))
        except Exception:
            pass
    show(_disp)

# -- Volume Display --
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
if part.part is not None:
    _vprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(part.part.wrapped, _vprops)
    print(f"\n  Volume of final part: {abs(_vprops.Mass()):.2f} mm³")
