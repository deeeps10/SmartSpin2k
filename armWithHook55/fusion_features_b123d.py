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

# 'Sketch1': 14 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, -45.0),
    x_dir=Vector(-1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, -1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((6.0, 190.034), (6.0, 189.0))
        Line((6.0, 189.0), (-90.0, 189.0))
        Line((-90.0, 189.0), (-90.0, 155.0))
        RadiusArc((-90.0, 155.0), (-60.0, 125.102), -29.8644)
        Line((-60.0, 125.102), (-60.0, -275.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-60.0, -275.0), (0.0, -335.0003), -60.0)
        RadiusArc((0.0, -335.0003), (60.0, -275.0), -60.0)
        Line((60.0, -275.0), (60.0, 125.102))
        RadiusArc((60.0, 125.102), (90.0, 155.0), -29.8188)
        Line((90.0, 155.0), (90.0, 275.0))
        Line((90.0, 275.0), (-90.0, 275.0))
        Line((-90.0, 275.0), (-90.0, 241.0))
        Line((-90.0, 241.0), (6.0, 241.0))
        Line((6.0, 241.0), (6.0, 239.966))
        # Arc split: sweep=211.56deg >= 150 — emitted as two half-arcs
        RadiusArc((6.0, 239.966), (39.0002, 215.0), 25.944)
        RadiusArc((39.0002, 215.0), (6.0, 190.034), 25.944)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# 'Sketch2': 23 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch2_2:
    with BuildLine():
        RadiusArc((22.1469, -130.774), (27.2591, -129.28), -27.8815)
        RadiusArc((27.2591, -129.28), (39.7694, -117.168), -22.3407)
        RadiusArc((39.7694, -117.168), (41.3423, -91.5556), -42.2927)
        RadiusArc((41.3423, -91.5556), (34.7803, -79.5998), -26.3026)
        RadiusArc((34.7803, -79.5998), (23.3186, -72.8062), -25.7622)
        RadiusArc((23.3186, -72.8062), (11.7836, -71.5459), -38.6916)
        RadiusArc((11.7836, -71.5459), (-5.0362, -78.0783), -25.5204)
        RadiusArc((-5.0362, -78.0783), (-13.4616, -97.6122), -26.4671)
        RadiusArc((-13.4616, -97.6122), (-8.002, -117.042), -27.7949)
        Line((-8.002, -117.042), (-35.2838, -115.395))
        Line((-35.2838, -115.395), (-35.2838, -76.5172))
        Line((-35.2838, -76.5172), (-44.6213, -76.5172))
        Line((-44.6213, -76.5172), (-44.6213, -125.403))
        Line((-44.6213, -125.403), (1.6405, -128.272))
        Line((1.6405, -128.272), (1.6405, -117.53))
        RadiusArc((1.6405, -117.53), (-4.2989, -105.006), 23.271)
        RadiusArc((-4.2989, -105.006), (-3.7228, -95.2658), 25.292)
        RadiusArc((-3.7228, -95.2658), (8.5368, -83.4848), 16.5042)
        RadiusArc((8.5368, -83.4848), (25.9851, -86.1284), 24.1844)
        RadiusArc((25.9851, -86.1284), (33.5973, -100.621), 16.8032)
        RadiusArc((33.5973, -100.621), (31.58, -111.559), 24.7844)
        RadiusArc((31.58, -111.559), (20.8653, -119.666), 16.32)
        Line((20.8653, -119.666), (22.1469, -130.774))
    _inc_edges_sk_Sketch2_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_2 = Wire.combine(_inc_edges_sk_Sketch2_2)[0]
_wire_sk_Sketch2_2 = _wire_sk_Sketch2_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch2_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch2_2.wrapped, True)
_face_sk_Sketch2_2 = Face(_mkf_sk_Sketch2_2.Face())

# 'Sketch2': 23 segments → Line/RadiusArc profile
_inclined_plane_3 = Plane(
    origin=Vector(0.0, 0.0, 40.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_3) as sk_Sketch2_3:
    with BuildLine():
        Line((22.1469, -62.7009), (20.8653, -51.5929))
        RadiusArc((20.8653, -51.5929), (31.58, -43.4859), -16.32)
        RadiusArc((31.58, -43.4859), (33.5973, -32.5479), -24.7844)
        RadiusArc((33.5973, -32.5479), (25.9851, -18.0553), -16.8032)
        RadiusArc((25.9851, -18.0553), (8.5368, -15.4117), -24.1844)
        RadiusArc((8.5368, -15.4117), (-3.7228, -27.1927), -16.5042)
        RadiusArc((-3.7228, -27.1927), (-4.2989, -36.9329), -25.292)
        RadiusArc((-4.2989, -36.9329), (1.6405, -49.4569), -23.271)
        Line((1.6405, -49.4569), (1.6405, -60.1989))
        Line((1.6405, -60.1989), (-44.6213, -57.3299))
        Line((-44.6213, -57.3299), (-44.6213, -8.4441))
        Line((-44.6213, -8.4441), (-35.2838, -8.4441))
        Line((-35.2838, -8.4441), (-35.2838, -47.3219))
        Line((-35.2838, -47.3219), (-8.002, -48.9689))
        RadiusArc((-8.002, -48.9689), (-13.4616, -29.5391), 27.7949)
        RadiusArc((-13.4616, -29.5391), (-5.0362, -10.0052), 26.4671)
        RadiusArc((-5.0362, -10.0052), (11.7836, -3.4728), 25.5204)
        RadiusArc((11.7836, -3.4728), (23.3186, -4.7331), 38.6916)
        RadiusArc((23.3186, -4.7331), (34.7803, -11.5267), 25.7622)
        RadiusArc((34.7803, -11.5267), (41.3423, -23.4825), 26.3026)
        RadiusArc((41.3423, -23.4825), (39.7694, -49.0949), 42.2927)
        RadiusArc((39.7694, -49.0949), (27.2591, -61.2069), 22.3407)
        RadiusArc((27.2591, -61.2069), (22.1469, -62.7009), 27.8815)
    _inc_edges_sk_Sketch2_3 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch2_3 = Wire.combine(_inc_edges_sk_Sketch2_3)[0]
_wire_sk_Sketch2_3 = _wire_sk_Sketch2_3.moved(_inclined_plane_3.location)
_mkf_sk_Sketch2_3 = BRepBuilderAPI_MakeFace(_inclined_plane_3.wrapped, _wire_sk_Sketch2_3.wrapped, True)
_face_sk_Sketch2_3 = Face(_mkf_sk_Sketch2_3.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, -1.0) * -90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.992: subtract bore(s) — inner loop(s) on inclined plane
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder as _MkCyl
    from OCP.gp import gp_Ax2 as _gAx2, gp_Pnt as _gPnt, gp_Dir as _gDir
    _bore_ax = _gAx2(_gPnt(0.0004, -275.0016, -45.0), _gDir(-0.0, -0.0, 1.0))
    _bore_cyl = _MkCyl(_bore_ax, 26.2514, 90.0)
    _bore_cyl.Build()
    part.part = cut_solids(part.part, Solid(_bore_cyl.Shape()))
    # Fusion depth expression: -90.000000 mm
    
    # --- FEATURE: Extrude2 ---
    # -- Extrude2_p0 --
    _face = _face_sk_Sketch2_2
    _vec = Vector(0.0, 0.0, 1.0) * 90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 90.000000 mm
    
    # -- Extrude2_p1 --
    _face = _face_sk_Sketch2_3
    _vec = Vector(0.0, 0.0, 1.0) * 90.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: 90.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
