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

# 'Sketch1': 2 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        # Arc split: sweep=302.64deg >= 150 — emitted as two half-arcs
        RadiusArc((-129.9489, -72.5941), (-141.699, -26.6301), -24.4839)
        RadiusArc((-141.699, -26.6301), (-153.4504, -72.5941), -24.4839)
        Line((-153.4504, -72.5941), (-129.9489, -72.5941))
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())
# v16.993: inner loop 0 bore face (segment-based, inclined plane)
with BuildSketch(_inclined_plane_1) as sk_Sketch1_bore0_2:
    with BuildLine():
        RadiusArc((-148.0249, -32.365), (-147.1289, -33.2498), -2.2952)
        RadiusArc((-147.1289, -33.2498), (-145.3267, -34.0107), -6.9111)
        RadiusArc((-145.3267, -34.0107), (-142.9731, -34.422), -12.5919)
        RadiusArc((-142.9731, -34.422), (-140.4245, -34.422), -15.3437)
        RadiusArc((-140.4245, -34.422), (-138.0709, -34.0107), -12.5919)
        RadiusArc((-138.0709, -34.0107), (-136.2687, -33.2498), -6.9111)
        RadiusArc((-136.2687, -33.2498), (-135.3727, -32.365), -2.279)
        Line((-135.3727, -32.365), (-126.7182, -32.365))
        # Arc split: sweep=211.41deg >= 150 — emitted as two half-arcs
        RadiusArc((-126.7182, -32.365), (-127.2913, -36.7076), -2.7477)
        RadiusArc((-127.2913, -36.7076), (-122.9557, -36.0839), -2.7477)
        Line((-122.9557, -36.0839), (-122.9702, -44.7102))
        RadiusArc((-122.9702, -44.7102), (-124.3825, -46.497), -3.0344)
        RadiusArc((-124.3825, -46.497), (-124.9802, -48.6156), -9.9078)
        RadiusArc((-124.9802, -48.6156), (-125.1366, -52.3884), -14.9057)
        RadiusArc((-125.1366, -52.3884), (-124.3825, -55.7321), -11.6271)
        RadiusArc((-124.3825, -55.7321), (-122.9488, -57.5274), -3.584)
        Line((-122.9488, -57.5274), (-122.9488, -66.2631))
        # Arc split: sweep=217.15deg >= 150 — emitted as two half-arcs
        RadiusArc((-122.9488, -66.2631), (-127.336, -65.446), -2.7481)
        RadiusArc((-127.336, -65.446), (-126.7131, -69.865), -2.7481)
        Line((-126.7131, -69.865), (-135.365, -69.865))
        RadiusArc((-135.365, -69.865), (-136.2687, -68.9707), -2.2981)
        RadiusArc((-136.2687, -68.9707), (-138.0709, -68.2098), -6.9106)
        RadiusArc((-138.0709, -68.2098), (-140.4245, -67.7977), -12.6839)
        RadiusArc((-140.4245, -67.7977), (-144.1982, -67.9551), -15.0842)
        RadiusArc((-144.1982, -67.9551), (-147.1289, -68.9707), -8.785)
        RadiusArc((-147.1289, -68.9707), (-148.0326, -69.865), -2.3144)
        Line((-148.0326, -69.865), (-156.6726, -69.865))
        # Arc split: sweep=217.59deg >= 150 — emitted as two half-arcs
        RadiusArc((-156.6726, -69.865), (-156.0596, -65.4372), -2.7488)
        RadiusArc((-156.0596, -65.4372), (-160.4488, -66.2837), -2.7488)
        Line((-160.4488, -66.2837), (-160.4488, -57.5009))
        RadiusArc((-160.4488, -57.5009), (-159.4819, -56.5442), -2.4685)
        RadiusArc((-159.4819, -56.5442), (-158.721, -54.7429), -6.9212)
        RadiusArc((-158.721, -54.7429), (-158.2559, -51.1145), -13.305)
        RadiusArc((-158.2559, -51.1145), (-158.721, -47.4871), -13.9197)
        RadiusArc((-158.721, -47.4871), (-160.4488, -44.729), -5.0049)
        Line((-160.4488, -44.729), (-160.4488, -35.9207))
        # Arc split: sweep=215.18deg >= 150 — emitted as two half-arcs
        RadiusArc((-160.4488, -35.9207), (-156.0952, -36.7723), -2.7488)
        RadiusArc((-156.0952, -36.7723), (-156.599, -32.365), -2.7488)
        Line((-156.599, -32.365), (-148.0249, -32.365))
    _inc_bore_edges_sk_Sketch1_bore0_2 = list(BuildSketch._get_context().pending_edges)
_bore_wire_sk_Sketch1_bore0_2 = Wire.combine(_inc_bore_edges_sk_Sketch1_bore0_2)[0]
_bore_wire_sk_Sketch1_bore0_2 = _bore_wire_sk_Sketch1_bore0_2.moved(_inclined_plane_1.location)
_bore_mkf_sk_Sketch1_bore0_2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _bore_wire_sk_Sketch1_bore0_2.wrapped, True)
_bore_face_sk_Sketch1_bore0_2 = Face(_bore_mkf_sk_Sketch1_bore0_2.Face())


# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, 0.0, 1.0) * -13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.993: subtract segment-based bore(s) — inclined plane
    _bore_solid_sk_Sketch1_bore0_2 = Solid.extrude(_bore_face_sk_Sketch1_bore0_2, Vector(0.0, 0.0, 1.0) * -13.0)
    part.part = cut_solids(part.part, _bore_solid_sk_Sketch1_bore0_2)
    # Fusion depth expression: -13.000000715 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
