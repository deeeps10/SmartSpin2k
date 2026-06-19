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
    origin=Vector(0.0, 0.0, 34.4238),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
# 'Sketch2': 33 segments → revolve profile
with BuildSketch(_inclined_plane_1) as sk_Sketch2_0:
    with BuildLine():
        Line((5.3003, 14.9944), (5.3, 29.9468))
        Line((5.3, 29.9468), (5.3001, 29.9893))
        Line((5.3001, 29.9893), (5.2933, 30.0))
        Line((5.2933, 30.0), (0.0462, 30.0))
        Line((0.0462, 30.0), (0.0462, 16.5061))
        Line((0.0462, 16.5061), (0.0462, 0.0))
        Line((0.0462, 0.0), (1.918, 0.0))
        Line((1.918, 0.0), (5.1387, 0.0))
        Line((5.1387, 0.0), (5.1873, 0.0))
        Line((5.1873, 0.0), (8.1427, 0.0))
        Line((8.1427, 0.0), (10.8585, 0.0))
        Line((10.8585, 0.0), (13.3776, 0.0))
        Line((13.3776, 0.0), (13.4754, 0.0))
        Line((13.4754, 0.0), (15.8409, 0.0))
        Line((15.8409, 0.0), (18.0694, 0.0))
        Line((18.0694, 0.0), (20.1847, 0.0))
        Line((20.1847, 0.0), (20.3039, 0.0))
        Line((20.3039, 0.0), (22.3301, 0.0))
        Line((22.3301, 0.0), (24.2789, 0.0))
        Line((24.2789, 0.0), (26.1655, 0.0))
        Line((26.1655, 0.0), (28.0033, 0.0))
        Line((28.0033, 0.0), (28.1336, 0.0))
        Line((28.1336, 0.0), (29.9359, 0.0))
        Line((29.9359, 0.0), (31.712, 0.0))
        Line((31.712, 0.0), (33.9778, 0.0))
        Line((33.9778, 0.0), (34.0802, 0.0))
        Line((34.0802, 0.0), (34.1927, 0.0))
        Line((34.1927, 0.0), (34.3122, 0.0))
        Line((34.3122, 0.0), (34.4352, 0.0))
        Line((34.4352, 0.0), (34.5, 0.0))
        Line((34.5, 0.0), (34.5, 1.0))
        Line((34.5, 1.0), (19.309, 1.0))
        RadiusArc((19.309, 1.0), (5.3003, 14.9944), 14.0899)
    make_face()
# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch3_2:
    with BuildLine():
        Line((57.6507, -18.3209), (58.3682, -17.6034))
        RadiusArc((58.3682, -17.6034), (51.3236, -10.5578), -29.1999)
        Line((51.3236, -10.5578), (50.6056, -11.2758))
        RadiusArc((50.6056, -11.2758), (57.6507, -18.3209), 28.1999)
    _inc_edges_sk_Sketch3_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_2 = Wire.combine(_inc_edges_sk_Sketch3_2)[0]
_wire_sk_Sketch3_2 = _wire_sk_Sketch3_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch3_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch3_2.wrapped, True)
_face_sk_Sketch3_2 = Face(_mkf_sk_Sketch3_2.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((62.2565, -39.4075), (63.2712, -39.4075))
        RadiusArc((63.2712, -39.4075), (63.2719, -29.4443), -29.1999)
        Line((63.2719, -29.4443), (62.2565, -29.4443))
        RadiusArc((62.2565, -29.4443), (62.2565, -39.4075), 28.1999)
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch3_4:
    with BuildLine():
        Line((50.6028, -57.5748), (51.3203, -58.2924))
        RadiusArc((51.3203, -58.2924), (58.3659, -51.2477), -29.1999)
        Line((58.3659, -51.2477), (57.6479, -50.5297))
        RadiusArc((57.6479, -50.5297), (50.6028, -57.5748), 28.1999)
    _inc_edges_sk_Sketch3_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_4 = Wire.combine(_inc_edges_sk_Sketch3_4)[0]
_wire_sk_Sketch3_4 = _wire_sk_Sketch3_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch3_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch3_4.wrapped, True)
_face_sk_Sketch3_4 = Face(_mkf_sk_Sketch3_4.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch3_5:
    with BuildLine():
        Line((29.5161, -62.1806), (29.5161, -63.1953))
        RadiusArc((29.5161, -63.1953), (39.4794, -63.196), -29.1999)
        Line((39.4794, -63.196), (39.4794, -62.1806))
        RadiusArc((39.4794, -62.1806), (29.5161, -62.1806), 28.1999)
    _inc_edges_sk_Sketch3_5 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_5 = Wire.combine(_inc_edges_sk_Sketch3_5)[0]
_wire_sk_Sketch3_5 = _wire_sk_Sketch3_5.moved(_inclined_plane_5.location)
_mkf_sk_Sketch3_5 = BRepBuilderAPI_MakeFace(_inclined_plane_5.wrapped, _wire_sk_Sketch3_5.wrapped, True)
_face_sk_Sketch3_5 = Face(_mkf_sk_Sketch3_5.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch3_6:
    with BuildLine():
        Line((11.3488, -50.5269), (10.6313, -51.2444))
        RadiusArc((10.6313, -51.2444), (17.6759, -58.29), -29.1999)
        Line((17.6759, -58.29), (18.3939, -57.572))
        RadiusArc((18.3939, -57.572), (11.3488, -50.5269), 28.1999)
    _inc_edges_sk_Sketch3_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_6 = Wire.combine(_inc_edges_sk_Sketch3_6)[0]
_wire_sk_Sketch3_6 = _wire_sk_Sketch3_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch3_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch3_6.wrapped, True)
_face_sk_Sketch3_6 = Face(_mkf_sk_Sketch3_6.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch3_7:
    with BuildLine():
        Line((6.743, -29.4402), (5.7283, -29.4402))
        RadiusArc((5.7283, -29.4402), (5.7276, -39.4035), -29.1999)
        Line((5.7276, -39.4035), (6.743, -39.4035))
        RadiusArc((6.743, -39.4035), (6.743, -29.4402), 28.1999)
    _inc_edges_sk_Sketch3_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_7 = Wire.combine(_inc_edges_sk_Sketch3_7)[0]
_wire_sk_Sketch3_7 = _wire_sk_Sketch3_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch3_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch3_7.wrapped, True)
_face_sk_Sketch3_7 = Face(_mkf_sk_Sketch3_7.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_8 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_8) as sk_Sketch3_8:
    with BuildLine():
        Line((18.3968, -11.2729), (17.6793, -10.5554))
        RadiusArc((17.6793, -10.5554), (10.6337, -17.6), -29.1999)
        Line((10.6337, -17.6), (11.3517, -18.318))
        RadiusArc((11.3517, -18.318), (18.3968, -11.2729), 28.1999)
    _inc_edges_sk_Sketch3_8 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_8 = Wire.combine(_inc_edges_sk_Sketch3_8)[0]
_wire_sk_Sketch3_8 = _wire_sk_Sketch3_8.moved(_inclined_plane_8.location)
_mkf_sk_Sketch3_8 = BRepBuilderAPI_MakeFace(_inclined_plane_8.wrapped, _wire_sk_Sketch3_8.wrapped, True)
_face_sk_Sketch3_8 = Face(_mkf_sk_Sketch3_8.Face())

# 'Sketch3': 4 segments → Line/RadiusArc profile
_inclined_plane_9 = Plane(
    origin=Vector(0.0, 30.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_9) as sk_Sketch3_9:
    with BuildLine():
        Line((39.4834, -6.6672), (39.4834, -5.6524))
        RadiusArc((39.4834, -5.6524), (29.5201, -5.6517), -29.1999)
        Line((29.5201, -5.6517), (29.5201, -6.6672))
        RadiusArc((29.5201, -6.6672), (39.4834, -6.6672), 28.1999)
    _inc_edges_sk_Sketch3_9 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_9 = Wire.combine(_inc_edges_sk_Sketch3_9)[0]
_wire_sk_Sketch3_9 = _wire_sk_Sketch3_9.moved(_inclined_plane_9.location)
_mkf_sk_Sketch3_9 = BRepBuilderAPI_MakeFace(_inclined_plane_9.wrapped, _wire_sk_Sketch3_9.wrapped, True)
_face_sk_Sketch3_9 = Face(_mkf_sk_Sketch3_9.Face())

# 'Sketch5': circle on inclined plane
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 29.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch5_10:
    with Locations((34.5006, -34.4259)):
        Circle(radius=28.2006)

# ─────────────────────────────────────────────────────────────────
# Diagnostic / safety / repair utilities
# ─────────────────────────────────────────────────────────────────

def get_volume(shape):
    """Return absolute volume in mm³; 0.0 if shape is None or non-solid."""
    if shape is None:
        return 0.0
    try:
        from OCP.BRepGProp import BRepGProp
        from OCP.GProp import GProp_GProps
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape.wrapped, props)
        return abs(props.Mass())
    except Exception:
        return 0.0


def snap(label, shape, prev=None):
    """Print a volume checkpoint; return current volume."""
    vol = get_volume(shape)
    tag = "OK      " if vol > 1.0 else "ZERO-VOL"
    d   = f"  (Δ{vol - prev:+.1f})" if prev is not None else ""
    print(f"  [{tag}] {label:<46} vol={vol:.2f} mm³{d}")
    return vol


def heal_shape(shape):
    """Make a shape watertight: ShapeFix → surface sewing → solid from shell."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Shape, ShapeFix_Shell
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL
    from OCP.TopoDS import TopoDS

    sf = ShapeFix_Shape(shape.wrapped)
    sf.Perform()
    working = sf.Shape()

    sewer = BRepBuilderAPI_Sewing()
    exp = TopExp_Explorer(working, TopAbs_FACE)
    while exp.More():
        sewer.Add(exp.Current())
        exp.Next()
    sewer.Perform()
    sewn = sewer.SewedShape()

    built_solids = []
    exp_sh = TopExp_Explorer(sewn, TopAbs_SHELL)
    while exp_sh.More():
        try:
            raw_shell = TopoDS.Shell_s(exp_sh.Current())
            sfsh = ShapeFix_Shell()
            sfsh.Init(raw_shell)
            sfsh.Perform()
            mk = BRepBuilderAPI_MakeSolid()
            mk.Add(sfsh.Shell())
            if mk.IsDone():
                built_solids.append(mk.Solid())
        except Exception:
            pass
        exp_sh.Next()

    if not built_solids:
        return shape
    healed = []
    for raw in built_solids:
        try:
            sf2 = ShapeFix_Shape(raw)
            sf2.Perform()
            healed.append(sf2.Shape())
        except Exception:
            healed.append(raw)
    if len(healed) == 1:
        return Solid(healed[0])
    acc = Solid(healed[0])
    for s in healed[1:]:
        acc = fuse_solids(acc, Solid(s))
    return acc


def ensure_vol(label, shape, fallback):
    """If *shape* has no volume, try heal_shape(); on failure return *fallback*."""
    if get_volume(shape) > 1.0:
        return shape
    print(f"         !! {label}: volume=0 → attempting heal_shape()")
    healed = heal_shape(shape)
    if get_volume(healed) > 1.0:
        print(f"         !! heal succeeded")
        return healed
    print(f"         !! heal failed → rolling back to previous body")
    return fallback


def export_stl_watertight(shape, filename, linear_deflection=0.1, angular_deflection=0.5):
    """Repair shape and write a watertight binary STL."""
    import math
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    healed = heal_shape(shape)
    w = healed.wrapped
    mesh = BRepMesh_IncrementalMesh(
        w, linear_deflection, False, math.radians(angular_deflection), True)
    mesh.Perform()
    writer = StlAPI_Writer()
    writer.Write(w, filename)
    print(f"Watertight STL saved → {filename}")
    return healed


# ─────────────────────────────────────────────────────────────────
# Step-by-step build with per-step volume verification
# ─────────────────────────────────────────────────────────────────
print("\n===== STEP-BY-STEP BUILD WITH VOLUME ANALYSIS =====\n")

# ── Step 1: Revolve1 (main body of revolution) ───────────────────
_rev1_axis = Axis(Vector(34.5, -6.4501, 34.4238), Vector(0.0, -1.0, 0.0))
try:
    with BuildPart() as _bp1:
        revolve(sk_Sketch2_0.sketch.faces(), axis=_rev1_axis, mode=Mode.ADD)
    body = _bp1.part
    v = snap("Step 1  Revolve1 (main body)", body)
    body = ensure_vol("Revolve1", body, None)
except Exception as _ex:
    print(f"  [EXCEPT] Step 1  Revolve1: {_ex}")
    body = None
    v = 0.0

# ── Step 2: Extrude1 — eight boss/slot faces → NEW BODY ──────────
# These extrudes are built as a separate body (body_extrude1),
# independent of the main revolve body.
_extrude1_defs = [
    ("Extrude1_p0", _face_sk_Sketch3_2),
    ("Extrude1_p1", _face_sk_Sketch3_3),
    ("Extrude1_p2", _face_sk_Sketch3_4),
    ("Extrude1_p3", _face_sk_Sketch3_5),
    ("Extrude1_p4", _face_sk_Sketch3_6),
    ("Extrude1_p5", _face_sk_Sketch3_7),
    ("Extrude1_p6", _face_sk_Sketch3_8),
    ("Extrude1_p7", _face_sk_Sketch3_9),
]
_vec_extrude1 = Vector(0.0, 1.0, 0.0) * -30.0
body_extrude1 = None
for _name, _face in _extrude1_defs:
    try:
        _ext_solid  = Solid.extrude(_face, _vec_extrude1)
        _prev_ext   = get_volume(body_extrude1)
        body_extrude1 = _ext_solid if body_extrude1 is None else fuse_solids(body_extrude1, _ext_solid)
        snap(f"Step 2  {_name} → new body", body_extrude1, _prev_ext if body_extrude1 is not None else None)
        body_extrude1 = ensure_vol(_name, body_extrude1, body_extrude1)
    except Exception as _ex:
        print(f"  [EXCEPT] Step 2  {_name}: {_ex}")
snap("Step 2  Extrude1 body TOTAL (8 instances)", body_extrude1)

# ── Step 3: Extrude2 — tapered circle SUBTRACT ───────────────────
try:
    with BuildPart() as _bp_e2:
        extrude(sk_Sketch5_10.sketch, amount=7.0, taper=-45.0)
    _e2_tool = _bp_e2.part
    if _e2_tool is not None:
        _cand = cut_solids(body, _e2_tool)
        v     = snap("Step 3  Extrude2 SUBTRACT (tapered hole)", _cand, v)
        body  = ensure_vol("Extrude2 SUBTRACT", _cand, body)
    else:
        print("  [SKIP  ] Step 3  Extrude2 tool is None")
except Exception as _ex:
    print(f"  [EXCEPT] Step 3  Extrude2: {_ex}")

print("\n===== BUILD COMPLETE =====")
_v_body  = snap("FINAL  body (Revolve1 + Extrude2 cut)", body)
_v_ext1  = snap("FINAL  body_extrude1 (8 extrude instances)", body_extrude1)
print(f"  {'TOTAL':<10} body + body_extrude1 = {_v_body + _v_ext1:.2f} mm³")
print()

# -- Export (both bodies in one file) --
_export_bodies = [b for b in [body, body_extrude1] if b is not None and get_volume(b) > 1.0]
if _export_bodies:
    final_export = Compound(_export_bodies) if len(_export_bodies) > 1 else _export_bodies[0]
    export_step(final_export, 'fusion_features.step')
    export_stl_watertight(final_export, 'fusion_features.stl')
else:
    print("WARNING: no bodies with volume — export skipped")

if _has_ocp:
    show(body, body_extrude1)
