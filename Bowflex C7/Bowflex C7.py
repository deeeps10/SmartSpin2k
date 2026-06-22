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

# 'Sketch1': 26 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 225.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((-589.1667, 215.0), (-613.9338, 287.6767))
        # Arc split: sweep=160.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-613.9338, 287.6767), (-648.75, 293.8157), -27.5)
        RadiusArc((-648.75, 293.8157), (-660.8415, 260.5944), -27.5)
        Line((-660.8415, 260.5944), (-633.3415, 178.0944))
        Line((-633.3415, 178.0944), (-621.25, 163.6843))
        Line((-621.25, 163.6843), (-607.5, 161.2598))
        Line((-607.5, 161.2598), (-607.5, 160.0))
        Line((-607.5, 160.0), (-573.9962, 160.0))
        Line((-573.9962, 160.0), (-332.5, 160.0))
        Line((-332.5, 160.0), (-332.5, 159.1235))
        RadiusArc((-332.5, 159.1235), (-172.4804, -0.0), 159.126)
        RadiusArc((-172.4804, -0.0), (-332.5, -159.1235), 159.126)
        Line((-332.5, -159.1235), (-332.5, -160.0))
        Line((-332.5, -160.0), (-573.9962, -160.0))
        Line((-573.9962, -160.0), (-607.5, -160.0))
        Line((-607.5, -160.0), (-607.5, -161.2598))
        Line((-607.5, -161.2598), (-621.25, -163.6843))
        Line((-621.25, -163.6843), (-633.3415, -178.0944))
        Line((-633.3415, -178.0944), (-660.8415, -260.5944))
        # Arc split: sweep=160.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-660.8415, -260.5944), (-648.75, -293.8157), -27.5)
        RadiusArc((-648.75, -293.8157), (-613.9338, -287.6767), -27.5)
        Line((-613.9338, -287.6767), (-589.1667, -215.0))
        Line((-589.1667, -215.0), (-363.23, -215.0))
        Line((-363.23, -215.0), (-322.5, -215.0))
        RadiusArc((-322.5, -215.0), (-117.756, -0.0), -215.2569)
        RadiusArc((-117.756, -0.0), (-322.5, 215.0), -215.2569)
        Line((-322.5, 215.0), (-363.23, 215.0))
        Line((-363.23, 215.0), (-589.1667, 215.0))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 10 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 75.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        Line((150.0, -100.0), (150.0, -46.0))
        Line((150.0, -46.0), (-12.6667, -46.0))
        Line((-12.6667, -46.0), (-12.6667, -44.1566))
        # Arc split: sweep=207.66deg >= 150 — emitted as two half-arcs
        RadiusArc((-12.6667, -44.1566), (-69.0119, 0.0), 45.4749)
        RadiusArc((-69.0119, 0.0), (-12.6667, 44.1566), 45.4749)
        Line((-12.6667, 44.1566), (-12.6667, 46.0))
        Line((-12.6667, 46.0), (150.0, 46.0))
        Line((150.0, 46.0), (150.0, 100.0))
        Line((150.0, 100.0), (-142.3941, 100.0))
        RadiusArc((-142.3941, 100.0), (-142.3941, -100.0), 215.2569)
        Line((-142.3941, -100.0), (150.0, -100.0))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch3': 30 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 79.5962, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch3_3:
    with BuildLine():
        Line((22.7, -26.0), (22.1667, -26.0))
        Line((22.1667, -26.0), (22.1667, -25.6921))
        Line((22.1667, -25.6921), (21.9, -25.5381))
        Line((21.9, -25.5381), (22.1667, -25.0763))
        Line((22.1667, -25.0763), (22.1667, 25.0763))
        Line((22.1667, 25.0763), (21.9, 25.5381))
        Line((21.9, 25.5381), (22.1667, 25.6921))
        Line((22.1667, 25.6921), (22.1667, 26.0))
        Line((22.1667, 26.0), (22.7, 26.0))
        Line((22.7, 26.0), (66.1334, 51.0763))
        Line((66.1334, 51.0763), (66.4, 51.5381))
        Line((66.4, 51.5381), (66.6667, 51.3842))
        Line((66.6667, 51.3842), (66.9333, 51.5381))
        Line((66.9333, 51.5381), (67.2, 51.0763))
        Line((67.2, 51.0763), (110.6333, 26.0))
        Line((110.6333, 26.0), (111.1667, 26.0))
        Line((111.1667, 26.0), (111.1667, 25.6921))
        Line((111.1667, 25.6921), (111.4333, 25.5381))
        Line((111.4333, 25.5381), (111.1667, 25.0763))
        Line((111.1667, 25.0763), (111.1667, -25.0763))
        Line((111.1667, -25.0763), (111.4333, -25.5381))
        Line((111.4333, -25.5381), (111.1667, -25.6921))
        Line((111.1667, -25.6921), (111.1667, -26.0))
        Line((111.1667, -26.0), (110.6333, -26.0))
        Line((110.6333, -26.0), (67.2, -51.0763))
        Line((67.2, -51.0763), (66.9333, -51.5381))
        Line((66.9333, -51.5381), (66.6667, -51.3842))
        Line((66.6667, -51.3842), (66.4, -51.5381))
        Line((66.4, -51.5381), (66.1334, -51.0763))
        Line((66.1334, -51.0763), (22.7, -26.0))
    _inc_edges_sk_Sketch3_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch3_3 = Wire.combine(_inc_edges_sk_Sketch3_3)[0]
_wire_sk_Sketch3_3 = _wire_sk_Sketch3_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch3_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch3_3.wrapped, True)
_face_sk_Sketch3_3 = Face(_mkf_sk_Sketch3_3.Face())

# 'Sketch4': 9 segments → Line/RadiusArc profile
_inclined_plane_4 = Plane(
    origin=Vector(0.0, 79.5962, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, 1.0, 0.0),
)
with BuildSketch(_inclined_plane_4) as sk_Sketch4_4:
    with BuildLine():
        Line((57.2611, -25.8415), (42.851, -13.75))
        Line((42.851, -13.75), (39.5845, 4.7753))
        Line((39.5845, 4.7753), (48.99, 21.0662))
        Line((48.99, 21.0662), (66.6667, 27.5))
        Line((66.6667, 27.5), (84.3433, 21.0662))
        Line((84.3433, 21.0662), (93.7489, 4.7753))
        Line((93.7489, 4.7753), (90.4824, -13.75))
        Line((90.4824, -13.75), (76.0722, -25.8415))
        Line((76.0722, -25.8415), (57.2611, -25.8415))
    _inc_edges_sk_Sketch4_4 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch4_4 = Wire.combine(_inc_edges_sk_Sketch4_4)[0]
_wire_sk_Sketch4_4 = _wire_sk_Sketch4_4.moved(_inclined_plane_4.location)
_mkf_sk_Sketch4_4 = BRepBuilderAPI_MakeFace(_inclined_plane_4.wrapped, _wire_sk_Sketch4_4.wrapped, True)
_face_sk_Sketch4_4 = Face(_mkf_sk_Sketch4_4.Face())

# 'Sketch5': circle on inclined plane
_inclined_plane_5 = Plane(
    origin=Vector(0.0, -78.8461, 0.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(-0.0, -1.0, -0.0),
)
with BuildSketch(_inclined_plane_5) as sk_Sketch5_5:
    with Locations((-66.6667, 0.0)):
        Circle(radius=55.0)

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_6 = Plane(
    origin=Vector(0.0, 0.0, 150.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_6) as sk_Sketch8_6:
    with BuildLine():
        Line((598.0944, -241.7831), (604.55, -261.1499))
        Line((604.55, -261.1499), (651.9119, -233.8055))
        Line((651.9119, -233.8055), (645.4562, -214.4386))
        Line((645.4562, -214.4386), (598.0944, -241.7831))
    _inc_edges_sk_Sketch8_6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_6 = Wire.combine(_inc_edges_sk_Sketch8_6)[0]
_wire_sk_Sketch8_6 = _wire_sk_Sketch8_6.moved(_inclined_plane_6.location)
_mkf_sk_Sketch8_6 = BRepBuilderAPI_MakeFace(_inclined_plane_6.wrapped, _wire_sk_Sketch8_6.wrapped, True)
_face_sk_Sketch8_6 = Face(_mkf_sk_Sketch8_6.Face())

# 'Sketch8': 4 segments → Line/RadiusArc profile
_inclined_plane_7 = Plane(
    origin=Vector(0.0, 0.0, 150.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_7) as sk_Sketch8_7:
    with BuildLine():
        Line((604.55, 261.1499), (598.0944, 241.7831))
        Line((598.0944, 241.7831), (645.4562, 214.4386))
        Line((645.4562, 214.4386), (651.9119, 233.8055))
        Line((651.9119, 233.8055), (604.55, 261.1499))
    _inc_edges_sk_Sketch8_7 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch8_7 = Wire.combine(_inc_edges_sk_Sketch8_7)[0]
_wire_sk_Sketch8_7 = _wire_sk_Sketch8_7.moved(_inclined_plane_7.location)
_mkf_sk_Sketch8_7 = BRepBuilderAPI_MakeFace(_inclined_plane_7.wrapped, _wire_sk_Sketch8_7.wrapped, True)
_face_sk_Sketch8_7 = Face(_mkf_sk_Sketch8_7.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, 1.0) * -300.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -300.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * -150.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.ADD)
    # Fusion depth expression: -150.000000 mm
    
    # --- FEATURE: Extrude3 ---
    # -- Extrude3 --
    _face = _face_sk_Sketch3_3
    _vec = Vector(-0.0, 1.0, 0.0) * 20.4038
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 20.403838158 mm
    
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch4_4
    _vec = Vector(-0.0, 1.0, 0.0) * -200.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -200.000000 mm
    
    # --- FEATURE: Extrude5 ---
    # -- Extrude5 --
    extrude(sk_Sketch5_5.sketch, amount=21.1539, mode=Mode.SUBTRACT)
    # Fusion depth expression: 21.153850555 mm
    
    # --- FEATURE: Extrude6 ---
    # -- Extrude6_p0 --
    _face = _face_sk_Sketch8_6
    _vec = Vector(0.0, 0.0, -1.0) * 150.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 150.000000 mm
    
    # -- Extrude6_p1 --
    _face = _face_sk_Sketch8_7
    _vec = Vector(0.0, 0.0, -1.0) * 150.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 150.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
