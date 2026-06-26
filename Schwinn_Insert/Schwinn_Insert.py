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

# 'Sketch16': 10 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch16:
    with BuildLine():
        # Spline from EllipticalArc3D, 15 adaptive samples
        Spline((-165.0787, -43.8352), (-165.2873, -44.5474), (-165.4744, -45.2655), (-165.6398, -45.9889), (-165.7833, -46.717), (-165.9048, -47.4491), (-166.0042, -48.1845), (-166.0814, -48.9225), (-166.1363, -49.6626), (-166.1689, -50.404), (-166.1792, -51.146), (-166.1671, -51.888), (-166.1327, -52.6292), (-166.076, -53.3692), (-165.997, -54.107))
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((-165.997, -54.107), (-165.9427, -54.5227), (-165.8813, -54.9374), (-165.8128, -55.351), (-165.7372, -55.7633), (-165.6545, -56.1743), (-165.5648, -56.5838), (-165.468, -56.9917))
        # Spline from EllipticalArc3D, 29 adaptive samples
        Spline((-165.468, -56.9917), (-165.2833, -57.6916), (-165.0779, -58.3857), (-164.8522, -59.0735), (-164.6063, -59.7543), (-164.3404, -60.4276), (-164.0547, -61.0927), (-163.7496, -61.7491), (-163.4252, -62.3963), (-163.0818, -63.0335), (-162.7198, -63.6604), (-162.3395, -64.2763), (-161.9411, -64.8807), (-161.5251, -65.473), (-161.0918, -66.0529), (-160.6415, -66.6197), (-160.1747, -67.173), (-159.6918, -67.7122), (-159.1932, -68.237), (-158.6794, -68.7468), (-158.1507, -69.2413), (-157.6076, -69.7199), (-157.0507, -70.1823), (-156.4803, -70.628), (-155.8971, -71.0567), (-155.3015, -71.4681), (-154.6939, -71.8616), (-154.0751, -72.2371), (-153.4454, -72.5941))
        Line((-153.4454, -72.5941), (-129.9506, -72.5941))
        # Spline from EllipticalArc3D, 29 adaptive samples
        Spline((-129.9506, -72.5941), (-129.3209, -72.2371), (-128.702, -71.8617), (-128.0944, -71.4681), (-127.4988, -71.0568), (-126.9155, -70.6281), (-126.3451, -70.1824), (-125.7882, -69.72), (-125.2451, -69.2414), (-124.7164, -68.747), (-124.2025, -68.2372), (-123.7039, -67.7124), (-123.221, -67.1732), (-122.7542, -66.6199), (-122.3039, -66.0532), (-121.8705, -65.4733), (-121.4545, -64.881), (-121.0561, -64.2766), (-120.6757, -63.6607), (-120.3136, -63.0339), (-119.9702, -62.3967), (-119.6458, -61.7496), (-119.3406, -61.0932), (-119.0549, -60.4281), (-118.7889, -59.7549), (-118.543, -59.0741), (-118.3172, -58.3863), (-118.1118, -57.6922), (-117.927, -56.9923))
        RadiusArc((-117.927, -56.9923), (-117.3155, -53.2619), -24.4101)
        # Spline from EllipticalArc3D, 15 adaptive samples
        Spline((-117.3155, -53.2619), (-117.2384, -52.0735), (-117.2195, -50.8827), (-117.2586, -49.6924), (-117.3557, -48.5055), (-117.5106, -47.3247), (-117.7229, -46.1528), (-117.9921, -44.9927), (-118.3176, -43.8472), (-118.6986, -42.7188), (-119.1341, -41.6104), (-119.6232, -40.5245), (-120.1647, -39.4638), (-120.7573, -38.4308), (-121.3995, -37.4279))
        # Spline from EllipticalArc3D, 29 adaptive samples
        Spline((-121.3995, -37.4279), (-121.8371, -36.8003), (-122.2942, -36.1866), (-122.7701, -35.5876), (-123.2646, -35.0037), (-123.7771, -34.4355), (-124.3071, -33.8837), (-124.854, -33.3487), (-125.4175, -32.8311), (-125.9968, -32.3313), (-126.5915, -31.8499), (-127.201, -31.3873), (-127.8246, -30.944), (-128.4618, -30.5204), (-129.1119, -30.117), (-129.7743, -29.7341), (-130.4483, -29.372), (-131.1334, -29.0312), (-131.8287, -28.712), (-132.5337, -28.4147), (-133.2477, -28.1396), (-133.9699, -27.887), (-134.6996, -27.657), (-135.4362, -27.45), (-136.1789, -27.2661), (-136.927, -27.1055), (-137.6798, -26.9684), (-138.4364, -26.8549), (-139.1963, -26.7651))
        # Spline from EllipticalArc3D, 15 adaptive samples
        Spline((-139.1963, -26.7651), (-140.4089, -26.6695), (-141.6249, -26.6345), (-142.841, -26.66), (-144.0544, -26.746), (-145.262, -26.8924), (-146.4608, -27.0987), (-147.6479, -27.3644), (-148.8202, -27.689), (-149.9749, -28.0715), (-151.1092, -28.511), (-152.2202, -29.0064), (-153.3051, -29.5566), (-154.3612, -30.1601), (-155.3861, -30.8154))
        # Spline from NurbsCurve3D, 20 adaptive samples
        Spline((-155.3861, -30.8154), (-155.4798, -30.8795), (-155.5902, -30.9562), (-155.8447, -31.1358), (-156.3734, -31.5211), (-157.3254, -32.273), (-158.2303, -33.0627), (-159.0793, -33.8773), (-159.8635, -34.7039), (-160.6048, -35.5645), (-161.3282, -36.4889), (-162.0221, -37.4672), (-162.6745, -38.4893), (-163.2747, -39.5445), (-163.8173, -40.6188), (-164.2992, -41.697), (-164.7172, -42.7638), (-164.8993, -43.282), (-164.9861, -43.5446), (-165.0787, -43.8352))
    _inc_edges_sk_Sketch16 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch16 = Wire.combine(_inc_edges_sk_Sketch16)[0]
_wire_sk_Sketch16 = _wire_sk_Sketch16.moved(_inclined_plane_1.location)
_mkf_sk_Sketch16 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch16.wrapped, True)
_face_sk_Sketch16 = Face(_mkf_sk_Sketch16.Face())

# 'Sketch17': 25 segments → Line/RadiusArc profile
_inclined_plane_2 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_2) as sk_Sketch17_2:
    with BuildLine():
        RadiusArc((-122.9701, -44.6466), (-124.9536, -48.6113), -5.6197)
        RadiusArc((-124.9536, -48.6113), (-124.9536, -53.613), -14.9675)
        RadiusArc((-124.9536, -53.613), (-122.9488, -57.587), -5.5608)
        Line((-122.9488, -57.587), (-122.9488, -66.2487))
        # Arc split: sweep=216.32deg >= 150 — emitted as two half-arcs
        RadiusArc((-122.9488, -66.2487), (-127.337, -65.445), -2.7543)
        RadiusArc((-127.337, -65.445), (-126.7327, -69.865), -2.7543)
        Line((-126.7327, -69.865), (-135.3058, -69.865))
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((-135.3058, -69.865), (-135.7558, -69.4193), (-136.252, -69.0256), (-136.7883, -68.6886), (-137.3582, -68.4123), (-137.955, -68.2001), (-138.5715, -68.0546), (-139.2002, -67.9774))
        RadiusArc((-139.2002, -67.9774), (-144.1975, -67.9774), -14.902)
        RadiusArc((-144.1975, -67.9774), (-148.0915, -69.865), -5.7741)
        Line((-148.0915, -69.865), (-156.6512, -69.865))
        # Arc split: sweep=208.1deg >= 150 — emitted as two half-arcs
        RadiusArc((-156.6512, -69.865), (-155.9145, -65.5838), -2.7555)
        RadiusArc((-155.9145, -65.5838), (-160.2465, -65.9085), -2.7555)
        Line((-160.2465, -65.9085), (-160.4488, -66.2838))
        Line((-160.4488, -66.2838), (-160.4488, -57.5642))
        RadiusArc((-160.4488, -57.5642), (-158.4909, -53.6177), -5.6644)
        RadiusArc((-158.4909, -53.6177), (-158.4909, -48.6162), -14.8981)
        RadiusArc((-158.4909, -48.6162), (-160.4488, -44.6703), -5.6609)
        Line((-160.4488, -44.6703), (-160.4488, -35.9361))
        # Arc split: sweep=214.39deg >= 150 — emitted as two half-arcs
        RadiusArc((-160.4488, -35.9361), (-156.0942, -36.7721), -2.7546)
        RadiusArc((-156.0942, -36.7721), (-156.5829, -32.365), -2.7546)
        Line((-156.5829, -32.365), (-148.0838, -32.365))
        RadiusArc((-148.0838, -32.365), (-144.1975, -34.2434), -5.7852)
        RadiusArc((-144.1975, -34.2434), (-139.2002, -34.2434), -14.9708)
        # Spline from EllipticalArc3D, 8 adaptive samples
        Spline((-139.2002, -34.2434), (-138.5733, -34.1662), (-137.9585, -34.0211), (-137.3632, -33.8099), (-136.7945, -33.535), (-136.2592, -33.1998), (-135.7635, -32.8082), (-135.3135, -32.365))
        Line((-135.3135, -32.365), (-126.7342, -32.365))
        # Arc split: sweep=210.52deg >= 150 — emitted as two half-arcs
        RadiusArc((-126.7342, -32.365), (-127.2915, -36.7072), -2.7543)
        RadiusArc((-127.2915, -36.7072), (-122.9557, -36.1019), -2.7543)
        Line((-122.9557, -36.1019), (-122.9701, -44.6466))
    _inc_edges_sk_Sketch17_2 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch17_2 = Wire.combine(_inc_edges_sk_Sketch17_2)[0]
_wire_sk_Sketch17_2 = _wire_sk_Sketch17_2.moved(_inclined_plane_2.location)
_mkf_sk_Sketch17_2 = BRepBuilderAPI_MakeFace(_inclined_plane_2.wrapped, _wire_sk_Sketch17_2.wrapped, True)
_face_sk_Sketch17_2 = Face(_mkf_sk_Sketch17_2.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude10 ---
    # -- Extrude10 --
    _face = _face_sk_Sketch16
    _vec = Vector(0.0, 0.0, 1.0) * -13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -13.000000715 mm
    
    # --- FEATURE: Extrude11 ---
    # -- Extrude11 --
    _face = _face_sk_Sketch17_2
    _vec = Vector(0.0, 0.0, 1.0) * -24.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid, mode=Mode.SUBTRACT)
    # Fusion depth expression: -24.000000 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
