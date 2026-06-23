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

# 'Sketch6': 164 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, -0.0, 0.0),
    x_dir=Vector(-1.0, -0.0, 0.0),
    z_dir=Vector(0.0, -1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch6:
    with BuildLine():
        # Spline from NurbsCurve3D, 30 adaptive samples
        Spline((-34.936, -2.5), (-34.4516, -2.5095), (-34.2149, -2.5635), (-33.9992, -2.666), (-33.8103, -2.8127), (-33.6433, -2.9904), (-33.3543, -3.3863), (-33.1029, -3.7986), (-32.8727, -4.2231), (-32.4395, -5.0908), (-32.0035, -5.9572), (-31.5249, -6.8), (-31.261, -7.206), (-30.9775, -7.5984), (-30.6721, -7.9743), (-30.3427, -8.3295), (-29.9876, -8.6596), (-29.6052, -8.9602), (-29.1981, -9.2287), (-28.7729, -9.4644), (-28.3366, -9.6664), (-27.8986, -9.8345), (-27.6762, -9.905), (-27.428, -9.962), (-26.8645, -10.0324), (-26.5863, -10.048), (-26.3316, -10.0525), (-25.8779, -10.0386), (-25.4524, -10.0135), (-25.0, -10.0))
        # Arc split: sweep=180.03deg >= 150 — emitted as two half-arcs
        RadiusArc((-25.0, -10.0), (-2.0, -33.0056), 23.0)
        RadiusArc((-2.0, -33.0056), (-25.0112, -56.0), 23.0)
        # Spline from NurbsCurve3D, 31 adaptive samples
        Spline((-25.0112, -56.0), (-25.466, -55.9914), (-25.897, -55.9711), (-26.3515, -55.9637), (-26.6023, -55.9725), (-26.8733, -55.9933), (-27.4254, -56.0738), (-27.6766, -56.1344), (-27.904, -56.2079), (-28.3274, -56.3874), (-28.7581, -56.6022), (-29.1778, -56.8541), (-29.5628, -57.1444), (-29.9188, -57.4674), (-30.2504, -57.8176), (-30.5586, -58.1903), (-30.844, -58.581), (-31.1075, -58.9855), (-31.3525, -59.4011), (-31.8017, -60.2563), (-32.2283, -61.1243), (-32.683, -61.9749), (-32.9365, -62.3855), (-33.2206, -62.7759), (-33.3807, -62.9565), (-33.5573, -63.1213), (-33.7566, -63.2569), (-33.979, -63.3515), (-34.2137, -63.4016), (-34.4529, -63.4136), (-34.936, -63.3867))
        Line((-34.936, -63.3867), (-184.0494, -63.3867))
        RadiusArc((-184.0494, -63.3867), (-184.8494, -64.1867), -0.8)
        Line((-184.8494, -64.1867), (-184.8494, -64.5866))
        RadiusArc((-184.8494, -64.5866), (-184.0494, -65.3867), -0.8)
        Line((-184.0494, -65.3867), (-180.1474, -65.3867))
        Line((-180.1474, -65.3867), (-179.6474, -66.0866))
        Line((-179.6474, -66.0866), (-179.0494, -66.0866))
        Line((-179.0494, -66.0866), (-179.0494, -65.3867))
        Line((-179.0494, -65.3867), (-172.1474, -65.3867))
        Line((-172.1474, -65.3867), (-171.6474, -66.0866))
        Line((-171.6474, -66.0866), (-171.0494, -66.0866))
        Line((-171.0494, -66.0866), (-171.0494, -65.3867))
        Line((-171.0494, -65.3867), (-164.1474, -65.3867))
        Line((-164.1474, -65.3867), (-163.6474, -66.0866))
        Line((-163.6474, -66.0866), (-163.0494, -66.0866))
        Line((-163.0494, -66.0866), (-163.0494, -65.3867))
        Line((-163.0494, -65.3867), (-156.1474, -65.3867))
        Line((-156.1474, -65.3867), (-155.6474, -66.0866))
        Line((-155.6474, -66.0866), (-155.0494, -66.0866))
        Line((-155.0494, -66.0866), (-155.0494, -65.3867))
        Line((-155.0494, -65.3867), (-148.1474, -65.3867))
        Line((-148.1474, -65.3867), (-147.6474, -66.0866))
        Line((-147.6474, -66.0866), (-147.0494, -66.0866))
        Line((-147.0494, -66.0866), (-147.0494, -65.3867))
        Line((-147.0494, -65.3867), (-140.1474, -65.3867))
        Line((-140.1474, -65.3867), (-139.6474, -66.0866))
        Line((-139.6474, -66.0866), (-139.0494, -66.0866))
        Line((-139.0494, -66.0866), (-139.0494, -65.3867))
        Line((-139.0494, -65.3867), (-132.1474, -65.3867))
        Line((-132.1474, -65.3867), (-131.6474, -66.0866))
        Line((-131.6474, -66.0866), (-131.0494, -66.0866))
        Line((-131.0494, -66.0866), (-131.0494, -65.3867))
        Line((-131.0494, -65.3867), (-124.1474, -65.3867))
        Line((-124.1474, -65.3867), (-123.6474, -66.0866))
        Line((-123.6474, -66.0866), (-123.0494, -66.0866))
        Line((-123.0494, -66.0866), (-123.0494, -65.3867))
        Line((-123.0494, -65.3867), (-116.1474, -65.3867))
        Line((-116.1474, -65.3867), (-115.6474, -66.0866))
        Line((-115.6474, -66.0866), (-115.0494, -66.0866))
        Line((-115.0494, -66.0866), (-115.0494, -65.3867))
        Line((-115.0494, -65.3867), (-108.1474, -65.3867))
        Line((-108.1474, -65.3867), (-107.6474, -66.0866))
        Line((-107.6474, -66.0866), (-107.0494, -66.0866))
        Line((-107.0494, -66.0866), (-107.0494, -65.3867))
        Line((-107.0494, -65.3867), (-100.1474, -65.3867))
        Line((-100.1474, -65.3867), (-99.6474, -66.0866))
        Line((-99.6474, -66.0866), (-99.0494, -66.0866))
        Line((-99.0494, -66.0866), (-99.0494, -65.3867))
        Line((-99.0494, -65.3867), (-92.1474, -65.3867))
        Line((-92.1474, -65.3867), (-91.6474, -66.0866))
        Line((-91.6474, -66.0866), (-91.0494, -66.0866))
        Line((-91.0494, -66.0866), (-91.0494, -65.3867))
        Line((-91.0494, -65.3867), (-84.1474, -65.3867))
        Line((-84.1474, -65.3867), (-83.6474, -66.0866))
        Line((-83.6474, -66.0866), (-83.0494, -66.0866))
        Line((-83.0494, -66.0866), (-83.0494, -65.3867))
        Line((-83.0494, -65.3867), (-76.1474, -65.3867))
        Line((-76.1474, -65.3867), (-75.6474, -66.0866))
        Line((-75.6474, -66.0866), (-75.0494, -66.0866))
        Line((-75.0494, -66.0866), (-75.0494, -65.3867))
        Line((-75.0494, -65.3867), (-68.1474, -65.3867))
        Line((-68.1474, -65.3867), (-67.6474, -66.0866))
        Line((-67.6474, -66.0866), (-67.0494, -66.0866))
        Line((-67.0494, -66.0866), (-67.0494, -65.3867))
        Line((-67.0494, -65.3867), (-60.1474, -65.3867))
        Line((-60.1474, -65.3867), (-59.6474, -66.0866))
        Line((-59.6474, -66.0866), (-59.0494, -66.0866))
        Line((-59.0494, -66.0866), (-59.0494, -65.3867))
        Line((-59.0494, -65.3867), (-52.1474, -65.3867))
        Line((-52.1474, -65.3867), (-51.6474, -66.0866))
        Line((-51.6474, -66.0866), (-51.0494, -66.0866))
        Line((-51.0494, -66.0866), (-51.0494, -65.3867))
        Line((-51.0494, -65.3867), (-44.1474, -65.3867))
        Line((-44.1474, -65.3867), (-43.6474, -66.0866))
        Line((-43.6474, -66.0866), (-43.0494, -66.0866))
        Line((-43.0494, -66.0866), (-43.0494, -65.3867))
        Line((-43.0494, -65.3867), (-35.0494, -65.3867))
        RadiusArc((-35.0494, -65.3867), (-33.8023, -65.3627), -4.3529)
        RadiusArc((-33.8023, -65.3627), (-31.9669, -64.3593), -3.2176)
        Line((-31.9669, -64.3593), (-28.617, -58.999))
        RadiusArc((-28.617, -58.999), (-26.9419, -58.0222), 3.27)
        Line((-26.9419, -58.0222), (-25.0, -58.0))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((-25.0, -58.0), (0.0, -33.0), -25.0)
        RadiusArc((0.0, -33.0), (-25.0, -8.0), -25.0)
        # Spline from NurbsCurve3D, 27 adaptive samples
        Spline((-25.0, -8.0), (-25.9521, -8.0529), (-26.4283, -8.0576), (-26.6709, -8.0364), (-26.9173, -7.994), (-27.1632, -7.9307), (-27.4032, -7.8482), (-27.6324, -7.7482), (-27.8485, -7.6321), (-28.2455, -7.3562), (-28.602, -7.0303), (-28.923, -6.6649), (-29.2126, -6.2709), (-29.475, -5.859), (-29.7156, -5.4361), (-30.5898, -3.699), (-31.0572, -2.844), (-31.3178, -2.4317), (-31.6038, -2.0383), (-31.921, -1.6707), (-32.2748, -1.3373), (-32.6698, -1.0509), (-33.1029, -0.8211), (-33.5564, -0.6498), (-34.0238, -0.5433), (-34.5093, -0.5072), (-35.0, -0.5))
        Line((-35.0, -0.5), (-43.0, -0.5))
        Line((-43.0, -0.5), (-43.0, -0.0))
        Line((-43.0, -0.0), (-43.598, 0.0))
        Line((-43.598, 0.0), (-44.098, -0.5))
        Line((-44.098, -0.5), (-51.0, -0.5))
        Line((-51.0, -0.5), (-51.0, -0.0))
        Line((-51.0, -0.0), (-51.598, -0.0))
        Line((-51.598, -0.0), (-52.098, -0.5))
        Line((-52.098, -0.5), (-59.0, -0.5))
        Line((-59.0, -0.5), (-59.0, -0.0))
        Line((-59.0, -0.0), (-59.598, 0.0))
        Line((-59.598, 0.0), (-60.098, -0.5))
        Line((-60.098, -0.5), (-67.0, -0.5))
        Line((-67.0, -0.5), (-67.0, -0.0))
        Line((-67.0, -0.0), (-67.598, -0.0))
        Line((-67.598, -0.0), (-68.098, -0.5))
        Line((-68.098, -0.5), (-75.0, -0.5))
        Line((-75.0, -0.5), (-75.0, -0.0))
        Line((-75.0, -0.0), (-75.598, 0.0))
        Line((-75.598, 0.0), (-76.098, -0.5))
        Line((-76.098, -0.5), (-83.0, -0.5))
        Line((-83.0, -0.5), (-83.0, -0.0))
        Line((-83.0, -0.0), (-83.598, -0.0))
        Line((-83.598, -0.0), (-84.098, -0.5))
        Line((-84.098, -0.5), (-91.0, -0.5))
        Line((-91.0, -0.5), (-91.0, -0.0))
        Line((-91.0, -0.0), (-91.598, 0.0))
        Line((-91.598, 0.0), (-92.098, -0.5))
        Line((-92.098, -0.5), (-99.0, -0.5))
        Line((-99.0, -0.5), (-99.0, -0.0))
        Line((-99.0, -0.0), (-99.598, 0.0))
        Line((-99.598, 0.0), (-100.098, -0.5))
        Line((-100.098, -0.5), (-107.0, -0.5))
        Line((-107.0, -0.5), (-107.0, -0.0))
        Line((-107.0, -0.0), (-107.598, 0.0))
        Line((-107.598, 0.0), (-108.098, -0.5))
        Line((-108.098, -0.5), (-115.0, -0.5))
        Line((-115.0, -0.5), (-115.0, -0.0))
        Line((-115.0, -0.0), (-115.598, 0.0))
        Line((-115.598, 0.0), (-116.098, -0.5))
        Line((-116.098, -0.5), (-123.0, -0.5))
        Line((-123.0, -0.5), (-123.0, -0.0))
        Line((-123.0, -0.0), (-123.598, 0.0))
        Line((-123.598, 0.0), (-124.098, -0.5))
        Line((-124.098, -0.5), (-131.0, -0.5))
        Line((-131.0, -0.5), (-131.0, -0.0))
        Line((-131.0, -0.0), (-131.598, 0.0))
        Line((-131.598, 0.0), (-132.098, -0.5))
        Line((-132.098, -0.5), (-139.0, -0.5))
        Line((-139.0, -0.5), (-139.0, -0.0))
        Line((-139.0, -0.0), (-139.598, -0.0))
        Line((-139.598, -0.0), (-140.098, -0.5))
        Line((-140.098, -0.5), (-147.0, -0.5))
        Line((-147.0, -0.5), (-147.0, -0.0))
        Line((-147.0, -0.0), (-147.598, 0.0))
        Line((-147.598, 0.0), (-148.098, -0.5))
        Line((-148.098, -0.5), (-155.0, -0.5))
        Line((-155.0, -0.5), (-155.0, -0.0))
        Line((-155.0, -0.0), (-155.598, 0.0))
        Line((-155.598, 0.0), (-156.098, -0.5))
        Line((-156.098, -0.5), (-163.0, -0.5))
        Line((-163.0, -0.5), (-163.0, -0.0))
        Line((-163.0, -0.0), (-163.598, 0.0))
        Line((-163.598, 0.0), (-164.098, -0.5))
        Line((-164.098, -0.5), (-171.0, -0.5))
        Line((-171.0, -0.5), (-171.0, -0.0))
        Line((-171.0, -0.0), (-171.598, 0.0))
        Line((-171.598, 0.0), (-172.098, -0.5))
        Line((-172.098, -0.5), (-179.0, -0.5))
        Line((-179.0, -0.5), (-179.0, -0.0))
        Line((-179.0, -0.0), (-179.598, 0.0))
        Line((-179.598, 0.0), (-180.098, -0.5))
        Line((-180.098, -0.5), (-184.0, -0.5))
        RadiusArc((-184.0, -0.5), (-185.0, -1.0), -1.0)
        Line((-185.0, -1.0), (-185.0, -1.5))
        RadiusArc((-185.0, -1.5), (-184.0, -2.5), -1.0)
        Line((-184.0, -2.5), (-34.936, -2.5))
    _inc_edges_sk_Sketch6 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch6 = Wire.combine(_inc_edges_sk_Sketch6)[0]
_wire_sk_Sketch6 = _wire_sk_Sketch6.moved(_inclined_plane_1.location)
_mkf_sk_Sketch6 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch6.wrapped, True)
_face_sk_Sketch6 = Face(_mkf_sk_Sketch6.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude4 ---
    # -- Extrude4 --
    _face = _face_sk_Sketch6
    _vec = Vector(0.0, -1.0, 0.0) * -18.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -18.000000715 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
