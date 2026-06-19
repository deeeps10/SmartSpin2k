# Units: mm throughout.

from build123d import *
from build123d import WorkplaneList  # not in __all__, needed for hole placement
from build123d.topology import Compound
import math
import os
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

# 'Sketch1': circle on inclined plane
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 25.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with Locations((38.0, -38.0)):
        Circle(radius=38.0)

# 'Sketch2': circle on inclined plane
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 10.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with Locations((38.0, -38.0)):
        Circle(radius=35.0)

# 'Sketch3': 2 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 10.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        # Arc split: sweep=282.04deg >= 150 — emitted as two half-arcs
        RadiusArc((35.3894, -34.774), (38.0, -42.15), -4.15)
        RadiusArc((38.0, -42.15), (40.6106, -34.774), -4.15)
        Line((40.6106, -34.774), (35.3894, -34.774))
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch4': 2 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 1.0, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        # Arc split: sweep=282.04deg >= 150 — emitted as two half-arcs
        RadiusArc((-40.6106, -34.774), (-38.0, -42.15), -4.15)
        RadiusArc((-38.0, -42.15), (-35.3894, -34.774), -4.15)
        Line((-35.3894, -34.774), (-40.6106, -34.774))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

_solid_sk_Sketch4_4 = extrude(_face_sk_Sketch4_4, amount=1.0, dir=Vector(-0.0, -1.0, -0.0), taper=-45.0).solid()

# 'SketchRevolve': profile on plane x=38  →  sketch(u,v) = world(38, u, v)
# u = world-Y,  v = world-Z
_revolve_plane = Plane(
    origin=Vector(38.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),   # sketch u  →  world Y
    z_dir=Vector(1.0, 0.0, 0.0),   # plane normal  →  world X
)
with BuildSketch(_revolve_plane) as sk_revolve:
    with BuildLine():
        Line((10.0, 3.0),  (10.0, 4.5))
        Line((10.0, 4.5),  (24.5, 4.5))
        RadiusArc((24.5, 4.5), (25.0, 3.9964), -0.5104)
        Line((25.0, 3.9964), (25.0, 3.0))
        Line((25.0, 3.0),  (10.0, 3.0))   # closes along the revolve axis
    _inc_edges_revolve = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_revolve = Wire.combine(_inc_edges_revolve)[0]
_wire_revolve = _wire_revolve.moved(_revolve_plane.location)
_mkf_revolve  = BRepBuilderAPI_MakeFace(_revolve_plane.wrapped, _wire_revolve.wrapped, True)
_face_revolve = Face(_mkf_revolve.Face())

# 'SketchRevolve2': profile in plane x=38  →  local(u,v) = world(38, u, v)
# Revolve axis: (38,10,72.9678)→(38,25,72.9678)  at z=72.9678, +Y direction
_revolve2_plane = Plane(
    origin=Vector(38.0, 0.0, 0.0),
    x_dir=Vector(0.0, 1.0, 0.0),
    z_dir=Vector(1.0, 0.0, 0.0),
)
with BuildSketch(_revolve2_plane) as sk_revolve2:
    with BuildLine():
        Line((10.0, 72.9678),   (25.0, 72.9678))
        Line((25.0, 72.9678),   (25.0, 71.9715))
        RadiusArc((25.0, 71.9715), (24.5002, 71.4679), 0.5037)
        Line((24.5002, 71.4679), (10.0, 71.4679))
        Line((10.0, 71.4679),   (10.0, 72.9678))
    _inc_edges_revolve2 = list(BuildSketch._get_context().pending_edges)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_revolve2 = Wire.combine(_inc_edges_revolve2)[0]
_wire_revolve2 = _wire_revolve2.moved(_revolve2_plane.location)
_mkf_revolve2  = BRepBuilderAPI_MakeFace(_revolve2_plane.wrapped, _wire_revolve2.wrapped, True)
_face_revolve2 = Face(_mkf_revolve2.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    extrude(sk_Sketch1.sketch, amount=-25.0)
    # Fusion depth expression: -25.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    extrude(sk_Sketch2_2.sketch, amount=80.0, mode=Mode.SUBTRACT)
    # Fusion depth expression: 80.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(-0.0, 1.0, 0.0) * -54.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -54.000000 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _solid = _solid_sk_Sketch4_4
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 1.000000015 mm
    # Fusion taper angle expression: 45.0 deg
    
    # --- FEATURE: Fillet3 ---
    # -- Fillet3 --
    # Fillet radius=0.5mm  (expr: 0.5 mm)  |  1 edge(s)
    # Edge indices exported from Fusion — valid for current body state.
    # If features were added before this fillet, re-run the edge diagnostic
    # below and update the indices.
    try:
        # OCP-confirmed indices: [3]
        # OCP-confirmed indices: [3]
        fillet(part.edges()[3], radius=0.5)
    except Exception as _fe:
        print('WARNING: Fillet3 fillet failed:', _fe)
        print('  Edge vertices above — use get_edge_by_endpoints() to select manually')

    # --- FEATURE: Revolve1 ---
    # Profile on plane x=38, revolved 360° around the Y-axis at (x=38, z=3)
    # Axis: from (38, 10, 3) → (38, 25, 3)  direction = (0, 1, 0)
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
    from OCP.gp import gp_Ax1, gp_Pnt, gp_Dir
    _revolve_ax1 = gp_Ax1(gp_Pnt(38.0, 10.0, 3.0), gp_Dir(0.0, 1.0, 0.0))
    _revol_op = BRepPrimAPI_MakeRevol(_face_revolve.wrapped, _revolve_ax1, 2 * math.pi)
    _revol_op.Build()
    add(Solid(_revol_op.Shape()), mode=Mode.ADD)

    # --- FEATURE: Revolve2 ---
    # Profile on plane x=38, revolved 360° around Y-axis at (x=38, z=72.9678)
    # Axis: (38,10,72.9678) → (38,25,72.9678)  direction = (0,1,0)
    _revolve2_ax1 = gp_Ax1(gp_Pnt(38.0, 10.0, 72.9678), gp_Dir(0.0, 1.0, 0.0))
    _revol2_op = BRepPrimAPI_MakeRevol(_face_revolve2.wrapped, _revolve2_ax1, 2 * math.pi)
    _revol2_op.Build()
    add(Solid(_revol2_op.Shape()), mode=Mode.ADD)

# -- Export --
_script_dir = os.path.dirname(os.path.abspath(__file__))
export_step(part.part, os.path.join(_script_dir, 'fusion_features_b123d.step'))
export_stl(part.part,  os.path.join(_script_dir, 'fusion_features_b123d.stl'))
print(f"Exported STEP -> {os.path.join(_script_dir, 'fusion_features_b123d.step')}")
print(f"Exported STL  -> {os.path.join(_script_dir, 'fusion_features_b123d.stl')}")
if _has_ocp: show(part)

# -- Volume comparison --
_REFERENCE_STL  = r"/Users/softage/Documents/smartspin2k/untitled folder/Knob_Cup_V2.STL"
_GENERATED_STL  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fusion_features_b123d_generated.stl")

try:
    import trimesh as _tm
    export_stl(part.part, _GENERATED_STL)
    _ref_mesh = _tm.load(_REFERENCE_STL,  force='mesh')
    _gen_mesh = _tm.load(_GENERATED_STL,  force='mesh')
    _ref_vol  = abs(_ref_mesh.volume)
    _gen_vol  = abs(_gen_mesh.volume)
    _diff_pct = (_gen_vol - _ref_vol) / _ref_vol * 100.0
    print("\n========== VOLUME COMPARISON ==========")
    print(f"  Reference Volume : {_ref_vol:.3f} mm³")
    print(f"  Generated Volume : {_gen_vol:.3f} mm³")
    print(f"  Difference       : {_gen_vol - _ref_vol:.3f} mm³  ({_diff_pct:+.2f}%)")
    print("========================================\n")
except FileNotFoundError as _e:
    print(f"[Volume] File not found: {_e}")
except Exception as _e:
    print(f"[Volume] Comparison failed: {_e}")

# -- Edge coordinate diagnostic --
# Uncomment the block below to find edge coordinates for fillet/chamfer.
# Set tx, ty, tz to a point near the edge you're looking for.
# Run the script — matching edges will print their vertex coordinates.
#
# tx, ty, tz = 0, 0, 0  # <-- set target coordinates here
# for e in part.part.edges():
#     verts = e.vertices()
#     for v in verts:
#         if abs(v.X - tx) < 1 and abs(v.Y - ty) < 1 and abs(v.Z - tz) < 1:
#             print([(round(v2.X,3), round(v2.Y,3), round(v2.Z,3)) for v2 in verts])
#             break
