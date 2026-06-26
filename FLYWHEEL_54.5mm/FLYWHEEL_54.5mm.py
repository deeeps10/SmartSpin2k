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
    origin=Vector(0.0, 0.0, 34.4259),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
# 'Sketch5': 8 segments → revolve profile
# auto-repair bridge inserted: (6.7463,0.1681)->(10.238,0.1631) gap=3.492mm [LONG — verify geometry]
with BuildSketch(_inclined_plane_1) as sk_Sketch5_0:
    with BuildLine():
        Line((10.238, 0.1631), (10.238, 0.4))
        RadiusArc((10.238, 0.4), (7.25, 3.4), 2.9923)
        Line((7.25, 3.4), (7.25, 25.0))
        Line((7.25, 25.0), (6.75, 25.0))
        Line((6.75, 25.0), (6.7509, 3.4))
        Line((6.7509, 3.4), (6.7463, 0.1681))
        Line((10.238, 0.1631), (6.7463, 0.1681))
        # auto-repair bridge: gap=3.492mm [LONG — verify]
        Line((6.7463, 0.1681), (10.238, 0.1631))
    make_face()
# -- Isolation buffer: body_Revolve4 (kind=body) --
with BuildPart() as body_Revolve4:
    # -- Revolve4 --
    _custom_axis = Axis(
        Vector(6.7463, -0.4872, 34.4259),
        Vector(0.0, -1.0, 0.0),
    )
    revolve(sk_Sketch5_0.sketch.faces(), axis=_custom_axis, mode=Mode.ADD)


# -- New profile sketch for additional revolve --
with BuildSketch(_inclined_plane_1) as sk_new_revolve:
    with BuildLine():
        Polyline(
            [(0.0085, 25.0), (6.7545, 25.0), (6.7545, 0.4),
             (34.5, 0.4), (34.5, 0.0), (0.0085, 0.0)],
            close=True,
        )
    make_face()

# -- Isolation buffer: body_new_revolve (kind=body) --
with BuildPart() as body_new_revolve:
    _new_revolve_axis = Axis(
        Vector(34.5, 25.6525, 34.4259),
        Vector(0.0, -1.0, 0.0),
    )
    revolve(sk_new_revolve.sketch.faces(), axis=_new_revolve_axis, mode=Mode.ADD)


# Plane at Y=25: normal=-Y so extrude goes from Y=25 toward Y=0
# local x → world X,  local y → world Z  (y_dir = z_dir × x_dir = (0,0,1))
_profile_cut_plane = Plane(
    origin=Vector(0.0, 25.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, -1.0, 0.0),
)

# Profile 1 sketch  (world X, Z coords on Y=25 plane)
with BuildSketch(_profile_cut_plane) as sk_profile1:
    with BuildLine():
        Line((36.8241, -0.8835), (36.8241, 0.0023))
        RadiusArc((36.8241, 0.0023), (35.8541, 0.5697), 1.0)
        RadiusArc((35.8541, 0.5697), (33.1459, 0.5697), -1.5)
        RadiusArc((33.1459, 0.5697), (32.1759, 0.0023), 1.0)
        Line((32.1759, 0.0023), (32.1759, -0.8835))
        Line((32.1759, -0.8835), (36.8241, -0.8835))
    make_face()

# Profile 2 sketch  (world X, Z coords on Y=25 plane)
with BuildSketch(_profile_cut_plane) as sk_profile2:
    with BuildLine():
        Line((32.5358, 69.9772), (32.5358, 68.8679))
        RadiusArc((32.5358, 68.8679), (33.0482, 68.4945), 0.5)
        RadiusArc((33.0482, 68.4945), (35.9518, 68.4945), -1.5)
        RadiusArc((35.9518, 68.4945), (36.4642, 68.8679), 0.5)
        Line((36.4642, 68.8679), (36.4642, 69.9772))
        Line((36.4642, 69.9772), (32.5358, 69.9772))
    make_face()


# -- Build --
with BuildPart() as part:
    # --- FEATURE: Revolve4 ---
    # -- Add Revolve4 (separate body) --
    add(body_Revolve4.part)
    
    # --- FEATURE: New Revolve (fused with current bodies) ---
    if body_new_revolve.part is not None: add(body_new_revolve.part)

    # --- FEATURE: C-Pattern2 ---
    # -- C-Pattern2 (bodies: Body352) --
    _custom_axis = Axis(
        Vector(34.5, 25.6525, 34.4259),
        Vector(0.0, -1.0, 0.0),
    )
    # Axis: _custom_axis  count=70  step=5.142857deg
    for _pat_i in range(70):
        if body_Revolve4.part is not None: add(body_Revolve4.part.rotate(_custom_axis, _pat_i * 5.142857))

    # --- FEATURE: Cut Profile 1 (Y=25 → Y=0, depth=25mm) ---
    extrude(sk_profile1.sketch.faces(), amount=25, mode=Mode.SUBTRACT)

    # --- FEATURE: Cut Profile 2 (Y=25 → Y=0, depth=25mm) ---
    extrude(sk_profile2.sketch.faces(), amount=25, mode=Mode.SUBTRACT)


# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
