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

def heal_shape(shape):
    """
    Make a shape watertight using OCCT surface sewing and topology repair.
    Strategy: ShapeFix_Shape → BRepBuilderAPI_Sewing → ShapeFix_Shell → BRepBuilderAPI_MakeSolid.
    This is geometry-based (not fuzzy boolean tolerance based).
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
    from OCP.ShapeFix import ShapeFix_Shape, ShapeFix_Shell
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL
    from OCP.TopoDS import TopoDS

    # Pass 1: fix degenerate edges, bad wires, flipped face normals
    sf = ShapeFix_Shape(shape.wrapped)
    sf.Perform()
    working = sf.Shape()

    # Pass 2: sew all faces together into closed shells
    sewer = BRepBuilderAPI_Sewing()
    exp = TopExp_Explorer(working, TopAbs_FACE)
    while exp.More():
        sewer.Add(exp.Current())
        exp.Next()
    sewer.Perform()
    sewn = sewer.SewedShape()

    # Pass 3: for each closed shell, fix outward orientation → promote to solid
    built_solids = []
    exp_sh = TopExp_Explorer(sewn, TopAbs_SHELL)
    while exp_sh.More():
        try:
            raw_shell = TopoDS.Shell_s(exp_sh.Current())
            sfsh = ShapeFix_Shell()
            sfsh.Init(raw_shell)
            sfsh.Perform()           # orients all faces outward by default
            good_shell = sfsh.Shell()
            mk = BRepBuilderAPI_MakeSolid()
            mk.Add(good_shell)
            if mk.IsDone():
                built_solids.append(mk.Solid())
        except Exception:
            pass
        exp_sh.Next()

    if not built_solids:
        return shape  # could not repair — return original unchanged

    # Pass 4: final ShapeFix pass on each new solid
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

    # Fuse multiple resulting solids into one watertight body
    acc = Solid(healed[0])
    for s in healed[1:]:
        acc = fuse_solids(acc, Solid(s))
    return acc


def export_stl_watertight(shape, filename, linear_deflection=0.5, angular_deflection=0.5):
    """Sew/repair shape surfaces then write a watertight binary STL.

    linear_deflection  – chord deviation in mm (absolute). 0.5 mm is high quality
                         for parts in the 500–700 mm range.
    angular_deflection – in radians. BRepMesh_IncrementalMesh takes radians directly;
                         0.5 rad (~28°) is the standard starting value. Do NOT wrap
                         this in math.radians() — doing so converts a degree value to
                         radians, so 0.5° → 0.00873 rad, which is 57× too fine and
                         generates ~1 GB STL files for models with revolve surfaces.
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer

    healed = heal_shape(shape)
    w = healed.wrapped

    mesh = BRepMesh_IncrementalMesh(
        w,
        linear_deflection,   # chord deviation in mm (absolute)
        False,               # not relative
        angular_deflection,  # angular tolerance in radians — already in radians
        True                 # parallel
    )
    mesh.Perform()

    writer = StlAPI_Writer()
    writer.Write(w, filename)
    print(f"Watertight STL saved → {filename}")
    return healed


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

_inclined_plane_2 = Plane(
    origin=Vector(344.9999, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
# 'Sketch4': 7 segments → revolve profile
with BuildSketch(_inclined_plane_2) as sk_Sketch4_1:
    with BuildLine():
        Line((6.8361, 224.0287), (6.8361, 239.4233))
        Line((6.8361, 239.4233), (6.8361, 262.4705))
        Line((6.8361, 262.4705), (6.8361, 262.8673))
        Line((6.8361, 262.8673), (128.7799, 262.8673))
        Line((128.7799, 262.8673), (128.7799, 144.1549))
        Line((128.7799, 144.1549), (86.8361, 144.1704))
        RadiusArc((86.8361, 144.1704), (6.8361, 224.0287), 79.9505)
    make_face()
# 'Sketch8': circle on inclined plane
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch8_3:
    with Locations((190.4497, -540.5488)):
        Circle(radius=50.0006)

# 'Sketch8': circle on inclined plane
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch8_4:
    with Locations((105.6011, -261.26)):
        Circle(radius=49.9998)

# 'Sketch8': circle on inclined plane
_inclined_plane_5 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch8_5:
    with Locations((345.0002, -94.2588)):
        Circle(radius=50.0003)

# 'Sketch8': circle on inclined plane
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch8_6:
    with Locations((577.8057, -270.3349)):
        Circle(radius=49.9999)

# 'Sketch8': circle on inclined plane
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch8_7:
    with Locations((482.2887, -546.1569)):
        Circle(radius=50.0)

_inclined_plane_8 = Plane(
    origin=Vector(344.2384, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
# 'Sketch11': 6 segments → revolve profile
with BuildSketch(_inclined_plane_8) as sk_Sketch11_7:
    with BuildLine():
        Line((-43.4267, -1.8807), (-43.4267, -228.8698))
        Line((-43.4267, -228.8698), (352.8362, -228.8698))
        Line((352.8362, -228.8698), (352.8362, -4.2159))
        Line((352.8362, -4.2159), (352.8362, 52.9888))
        Line((352.8362, 52.9888), (-43.4267, 52.9888))
        Line((-43.4267, 52.9888), (-43.4267, -1.8807))
    make_face()
_inclined_plane_9 = Plane(
    origin=Vector(344.2384, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
# 'Sketch9': 34 segments → revolve profile
with BuildSketch(_inclined_plane_9) as sk_Sketch9_8:
    with BuildLine():
        Line((275.8944, 593.9944), (352.8362, 593.914))
        Line((352.8362, 593.914), (352.8362, 689.2395))
        Line((352.8362, 689.2395), (2.8361, 689.2395))
        Line((2.8361, 689.2395), (2.8361, 344.2397))
        Line((2.8361, 344.2397), (6.8361, 344.2397))
        Line((6.8361, 344.2397), (6.8361, 351.5386))
        Line((6.8361, 351.5386), (6.8361, 365.1397))
        Line((6.8361, 365.1397), (6.8361, 369.9395))
        Line((6.8361, 369.9395), (6.8361, 375.6932))
        Line((6.8361, 375.6932), (6.8361, 382.1493))
        Line((6.8361, 382.1493), (6.8361, 389.0037))
        Line((6.8361, 389.0037), (6.8361, 401.9921))
        Line((6.8361, 401.9921), (6.8361, 408.3659))
        Line((6.8361, 408.3659), (6.8361, 414.5748))
        Line((6.8361, 414.5748), (6.8361, 420.3842))
        Line((6.8361, 420.3842), (6.8361, 433.4866))
        Line((6.8361, 433.4866), (6.8361, 438.32))
        Line((6.8361, 438.32), (6.8361, 452.3988))
        Line((6.8361, 452.3988), (6.8361, 456.2916))
        Line((6.8361, 456.2916), (6.8361, 459.609))
        Line((6.8361, 459.609), (6.8361, 463.8536))
        Line((6.8361, 463.8536), (6.8361, 465.2242))
        Line((6.8361, 465.2242), (6.8361, 467.8336))
        Line((6.8361, 467.8336), (6.8361, 477.3576))
        Line((6.8361, 477.3576), (6.8361, 480.2489))
        Line((6.8361, 480.2489), (6.8361, 485.0141))
        Line((6.8361, 485.0141), (6.8361, 492.1129))
        Line((6.8361, 492.1129), (6.8361, 501.7886))
        Line((6.8361, 501.7886), (6.8361, 506.4972))
        Line((6.8361, 506.4972), (6.8361, 507.8581))
        Line((6.8361, 507.8581), (6.8361, 509.9768))
        Line((6.8361, 509.9768), (6.8361, 514.2046))
        RadiusArc((6.8361, 514.2046), (86.8361, 594.1917), 79.9942)
        Line((86.8361, 594.1917), (275.8944, 593.9944))
    make_face()
# 'Sketch12': 6 segments → Line/RadiusArc profile
_inclined_plane_10 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_10) as sk_Sketch12_10:
    with BuildLine():
        Line((368.2413, -0.0227), (384.9427, 6.5276))
        Line((384.9427, 6.5276), (309.9584, 4.6894))
        Line((309.9584, 4.6894), (321.7587, -0.0227))
        RadiusArc((321.7587, -0.0227), (331.4594, -5.6975), 10.0)
        RadiusArc((331.4594, -5.6975), (358.5406, -5.6975), -15.0)
        RadiusArc((358.5406, -5.6975), (368.2413, -0.0227), 9.9999)
    _inc_edges_sk_Sketch12_10 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch12_10 = Wire.combine(_inc_edges_sk_Sketch12_10)[0]
_wire_sk_Sketch12_10 = _wire_sk_Sketch12_10.moved(_inclined_plane_10.location)
_mkf_sk_Sketch12_10 = BRepBuilderAPI_MakeFace(_inclined_plane_10.wrapped, _wire_sk_Sketch12_10.wrapped, True)
_face_sk_Sketch12_10 = Face(_mkf_sk_Sketch12_10.Face())

# 'Sketch12': 8 segments → Line/RadiusArc profile
_inclined_plane_11 = Plane(
    origin=Vector(0.0, 352.8362, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_11) as sk_Sketch12_11:
    with BuildLine():
        RadiusArc((359.5182, -684.9446), (354.1769, -676.8505), -15.0001)
        RadiusArc((354.1769, -676.8505), (335.8231, -676.8505), -15.0005)
        RadiusArc((335.8231, -676.8505), (330.4818, -684.9446), -15.0)
        RadiusArc((330.4818, -684.9446), (325.3577, -688.6794), 5.0003)
        Line((325.3577, -688.6794), (314.268, -707.9507))
        Line((314.268, -707.9507), (378.633, -707.9507))
        Line((378.633, -707.9507), (364.6423, -688.6794))
        RadiusArc((364.6423, -688.6794), (359.5182, -684.9446), 5.0001)
    _inc_edges_sk_Sketch12_11 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch12_11 = Wire.combine(_inc_edges_sk_Sketch12_11)[0]
_wire_sk_Sketch12_11 = _wire_sk_Sketch12_11.moved(_inclined_plane_11.location)
_mkf_sk_Sketch12_11 = BRepBuilderAPI_MakeFace(_inclined_plane_11.wrapped, _wire_sk_Sketch12_11.wrapped, True)
_face_sk_Sketch12_11 = Face(_mkf_sk_Sketch12_11.Face())

# 'Sketch2': circle on inclined plane
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 6.8361, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch2:
    with Locations((344.9999, -94.2587)):
        Circle(radius=130.0003)

# -- Isolation buffer: body_Extrude2 (kind=body) --
with BuildPart() as body_Extrude2:
    # -- Extrude2 --
    extrude(sk_Sketch2.sketch, amount=80.0, taper=45.0)
    # Fusion depth expression: 79.9999952316 mm
    # Fusion taper angle expression: -45.00000 deg
    

# ─────────────────────────────────────────────────────────────────
# Diagnostic / safety utilities
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
    print(f"  [{tag}] {label:<44} vol={vol:.2f} mm³{d}")
    return vol


def ensure_vol(label, shape, fallback):
    """If *shape* has no volume: try heal_shape(); on failure return *fallback*."""
    if get_volume(shape) > 1.0:
        return shape
    print(f"         !! {label}: volume=0 → attempting heal_shape()")
    healed = heal_shape(shape)
    if get_volume(healed) > 1.0:
        print(f"         !! heal succeeded")
        return healed
    print(f"         !! heal failed → rolling back to previous body")
    return fallback


# ─────────────────────────────────────────────────────────────────
# Step-by-step build with per-step volume verification
# ─────────────────────────────────────────────────────────────────
print("\n===== STEP-BY-STEP BUILD WITH VOLUME ANALYSIS =====\n")

# ── Step 1: Extrude2 ─────────────────────────────────────────────
with BuildPart() as _bp1:
    extrude(sk_Sketch2.sketch, amount=80.0, taper=45.0)
body = _bp1.part
v = snap("Step 1  Extrude2 (tapered base circle)", body)

# ── Step 2: Revolve1 SUBTRACT ────────────────────────────────────
# Build the revolve solid standalone (ADD into empty context) then cut it.
_rev1_axis = Axis(Vector(344.9999, 86.8361, 94.2587), Vector(0.0, 1.0, 0.0))
try:
    with BuildPart() as _bp_r1:
        revolve(sk_Sketch4_1.sketch.faces(), axis=_rev1_axis, mode=Mode.ADD)
    _rev1_tool = _bp_r1.part
    if _rev1_tool is not None:
        _cand = cut_solids(body, _rev1_tool)
        v = snap("Step 2  Revolve1 SUBTRACT", _cand, v)
        body = ensure_vol("Revolve1 SUBTRACT", _cand, body)
    else:
        print("  [SKIP  ] Step 2  Revolve1 tool is None")
except Exception as _ex:
    print(f"  [EXCEPT] Step 2  Revolve1: {_ex}")

# ── Step 3: C-Pattern1 ───────────────────────────────────────────
# FIX: original code re-added the pre-revolve body at i=0 (undoing the cut).
# Correct behaviour: rotate the POST-revolve body at all 5 positions (0..4)
# and fuse them together — this replaces the single body with the 5-fold array.
_cpat_axis = Axis(Vector(340.2291, 447.3238, 342.5118), Vector(0.0, -1.0, 0.0))
_pat_src   = body
_patterned = None
for _i in range(5):
    _rot       = _pat_src.rotate(_cpat_axis, _i * 72.0)
    _patterned = _rot if _patterned is None else fuse_solids(_patterned, _rot)
body = _patterned if _patterned is not None else body
v = snap("Step 3  C-Pattern1 (5 copies @ 72° fused)", body, v)
body = ensure_vol("C-Pattern1", body, _pat_src)

# ── Step 4: Extrude5 — five cylinders ADD ───────────────────────
_cyl_defs = [
    ("Extrude5_p0", sk_Sketch8_3),
    ("Extrude5_p1", sk_Sketch8_4),
    ("Extrude5_p2", sk_Sketch8_5),
    ("Extrude5_p3", sk_Sketch8_6),
    ("Extrude5_p4", sk_Sketch8_7),
]
for _name, _sk in _cyl_defs:
    try:
        with BuildPart() as _bp_cyl:
            extrude(_sk.sketch, amount=-266.0)
        _cyl = _bp_cyl.part
        if _cyl is not None:
            _prev = get_volume(body)
            body  = fuse_solids(body, _cyl)
            v     = snap(f"Step 4  {_name} ADD", body, _prev)
            body  = ensure_vol(_name, body, body)
    except Exception as _ex:
        print(f"  [EXCEPT] Step 4  {_name}: {_ex}")

# ── Step 5: Revolve4 SUBTRACT ────────────────────────────────────
_rev4_axis = Axis(Vector(344.2384, 447.3238, 342.5118), Vector(0.0, -1.0, 0.0))
try:
    with BuildPart() as _bp_r4:
        revolve(sk_Sketch11_7.sketch.faces(), axis=_rev4_axis, mode=Mode.ADD)
    _rev4_tool = _bp_r4.part
    if _rev4_tool is not None:
        _cand = cut_solids(body, _rev4_tool)
        v = snap("Step 5  Revolve4 SUBTRACT", _cand, v)
        body = ensure_vol("Revolve4 SUBTRACT", _cand, body)
    else:
        print("  [SKIP  ] Step 5  Revolve4 tool is None")
except Exception as _ex:
    print(f"  [EXCEPT] Step 5  Revolve4: {_ex}")

# ── Step 6: Revolve2 ADD ─────────────────────────────────────────
_rev2_axis = Axis(Vector(344.2384, 19.2065, 344.2397), Vector(0.0, 1.0, 0.0))
try:
    with BuildPart() as _bp_r2:
        revolve(sk_Sketch9_8.sketch.faces(), axis=_rev2_axis, mode=Mode.ADD)
    _rev2_solid = _bp_r2.part
    if _rev2_solid is not None:
        _prev = get_volume(body)
        body  = fuse_solids(body, _rev2_solid)
        v     = snap("Step 6  Revolve2 ADD", body, _prev)
        body  = ensure_vol("Revolve2 ADD", body, body)
    else:
        print("  [SKIP  ] Step 6  Revolve2 solid is None")
except Exception as _ex:
    print(f"  [EXCEPT] Step 6  Revolve2: {_ex}")

# ── Step 7: Extrude6_p0 SUBTRACT ─────────────────────────────────
try:
    _e6p0  = Solid.extrude(_face_sk_Sketch12_10, Vector(0.0, 1.0, 0.0) * -550.0)
    _cand  = cut_solids(body, _e6p0)
    v      = snap("Step 7  Extrude6_p0 SUBTRACT", _cand, v)
    body   = ensure_vol("Extrude6_p0 SUBTRACT", _cand, body)
except Exception as _ex:
    print(f"  [EXCEPT] Step 7  Extrude6_p0: {_ex}")

# ── Step 8: Extrude6_p1 SUBTRACT ─────────────────────────────────
try:
    _e6p1  = Solid.extrude(_face_sk_Sketch12_11, Vector(0.0, 1.0, 0.0) * -550.0)
    _cand  = cut_solids(body, _e6p1)
    v      = snap("Step 8  Extrude6_p1 SUBTRACT", _cand, v)
    body   = ensure_vol("Extrude6_p1 SUBTRACT", _cand, body)
except Exception as _ex:
    print(f"  [EXCEPT] Step 8  Extrude6_p1: {_ex}")

print("\n===== BUILD COMPLETE =====")
snap("FINAL BODY", body)
print()

# -- Export --
if body is not None and get_volume(body) > 1.0:
    export_step(body, 'fusion_features.step')
    export_stl_watertight(body, 'fusion_features.stl')
else:
    print("WARNING: final body has no volume — export skipped")

if _has_ocp:
    show(body)
