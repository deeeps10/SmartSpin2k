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

# 'Sketch1': 219 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 0.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, -1.0, 0.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch1:
    with BuildLine():
        Line((34.4036, 63.5052), (34.936, 63.4733))
        Line((34.936, 63.4733), (215.0494, 63.4733))
        RadiusArc((215.0494, 63.4733), (215.8494, 64.2733), -0.8)
        Line((215.8494, 64.2733), (215.8494, 64.6733))
        RadiusArc((215.8494, 64.6733), (215.0494, 65.4733), -0.7999)
        Line((215.0494, 65.4733), (212.1474, 65.4733))
        Line((212.1474, 65.4733), (211.6474, 66.1733))
        Line((211.6474, 66.1733), (211.0494, 66.1733))
        Line((211.0494, 66.1733), (211.0494, 65.4733))
        Line((211.0494, 65.4733), (204.1474, 65.4733))
        Line((204.1474, 65.4733), (203.6474, 66.1733))
        Line((203.6474, 66.1733), (203.0494, 66.1733))
        Line((203.0494, 66.1733), (203.0494, 65.4733))
        Line((203.0494, 65.4733), (196.1474, 65.4733))
        Line((196.1474, 65.4733), (195.6474, 66.1733))
        Line((195.6474, 66.1733), (195.0494, 66.1733))
        Line((195.0494, 66.1733), (195.0494, 65.4733))
        Line((195.0494, 65.4733), (188.1474, 65.4733))
        Line((188.1474, 65.4733), (187.6474, 66.1733))
        Line((187.6474, 66.1733), (187.0494, 66.1733))
        Line((187.0494, 66.1733), (187.0494, 65.4733))
        Line((187.0494, 65.4733), (180.1474, 65.4733))
        Line((180.1474, 65.4733), (179.6474, 66.1733))
        Line((179.6474, 66.1733), (179.0494, 66.1733))
        Line((179.0494, 66.1733), (179.0494, 65.4733))
        Line((179.0494, 65.4733), (172.1474, 65.4733))
        Line((172.1474, 65.4733), (171.6474, 66.1733))
        Line((171.6474, 66.1733), (171.0494, 66.1733))
        Line((171.0494, 66.1733), (171.0494, 65.4733))
        Line((171.0494, 65.4733), (164.1474, 65.4733))
        Line((164.1474, 65.4733), (163.6474, 66.1733))
        Line((163.6474, 66.1733), (163.0494, 66.1733))
        Line((163.0494, 66.1733), (163.0494, 65.4733))
        Line((163.0494, 65.4733), (156.1474, 65.4733))
        Line((156.1474, 65.4733), (155.6474, 66.1733))
        Line((155.6474, 66.1733), (155.0494, 66.1733))
        Line((155.0494, 66.1733), (155.0494, 65.4733))
        Line((155.0494, 65.4733), (148.1474, 65.4733))
        Line((148.1474, 65.4733), (147.6474, 66.1733))
        Line((147.6474, 66.1733), (147.0494, 66.1733))
        Line((147.0494, 66.1733), (147.0494, 65.4733))
        Line((147.0494, 65.4733), (140.1474, 65.4733))
        Line((140.1474, 65.4733), (139.6474, 66.1733))
        Line((139.6474, 66.1733), (139.0494, 66.1733))
        Line((139.0494, 66.1733), (139.0494, 65.4733))
        Line((139.0494, 65.4733), (132.1474, 65.4733))
        Line((132.1474, 65.4733), (131.6474, 66.1733))
        Line((131.6474, 66.1733), (131.0494, 66.1733))
        Line((131.0494, 66.1733), (131.0494, 65.4733))
        Line((131.0494, 65.4733), (124.1474, 65.4733))
        Line((124.1474, 65.4733), (123.6474, 66.1733))
        Line((123.6474, 66.1733), (123.0494, 66.1733))
        Line((123.0494, 66.1733), (123.0494, 65.4733))
        Line((123.0494, 65.4733), (116.1474, 65.4733))
        Line((116.1474, 65.4733), (115.6474, 66.1733))
        Line((115.6474, 66.1733), (115.0494, 66.1733))
        Line((115.0494, 66.1733), (115.0494, 65.4733))
        Line((115.0494, 65.4733), (108.1474, 65.4733))
        Line((108.1474, 65.4733), (107.6474, 66.1733))
        Line((107.6474, 66.1733), (107.0494, 66.1733))
        Line((107.0494, 66.1733), (107.0494, 65.4733))
        Line((107.0494, 65.4733), (100.1474, 65.4733))
        Line((100.1474, 65.4733), (99.6474, 66.1733))
        Line((99.6474, 66.1733), (99.0494, 66.1733))
        Line((99.0494, 66.1733), (99.0494, 65.4733))
        Line((99.0494, 65.4733), (92.1474, 65.4733))
        Line((92.1474, 65.4733), (91.6474, 66.1733))
        Line((91.6474, 66.1733), (91.0494, 66.1733))
        Line((91.0494, 66.1733), (91.0494, 65.4733))
        Line((91.0494, 65.4733), (84.1474, 65.4733))
        Line((84.1474, 65.4733), (83.6474, 66.1733))
        Line((83.6474, 66.1733), (83.0494, 66.1733))
        Line((83.0494, 66.1733), (83.0494, 65.4733))
        Line((83.0494, 65.4733), (76.1474, 65.4733))
        Line((76.1474, 65.4733), (75.6474, 66.1733))
        Line((75.6474, 66.1733), (75.0494, 66.1733))
        Line((75.0494, 66.1733), (75.0494, 65.4733))
        Line((75.0494, 65.4733), (68.1474, 65.4733))
        Line((68.1474, 65.4733), (67.6474, 66.1733))
        Line((67.6474, 66.1733), (67.0494, 66.1733))
        Line((67.0494, 66.1733), (67.0494, 65.4733))
        Line((67.0494, 65.4733), (60.1474, 65.4733))
        Line((60.1474, 65.4733), (59.6474, 66.1733))
        Line((59.6474, 66.1733), (59.0494, 66.1733))
        Line((59.0494, 66.1733), (59.0494, 65.4733))
        Line((59.0494, 65.4733), (52.1474, 65.4733))
        Line((52.1474, 65.4733), (51.6474, 66.1733))
        Line((51.6474, 66.1733), (51.0494, 66.1733))
        Line((51.0494, 66.1733), (51.0494, 65.4733))
        Line((51.0494, 65.4733), (44.1474, 65.4733))
        Line((44.1474, 65.4733), (43.6474, 66.1733))
        Line((43.6474, 66.1733), (43.0494, 66.1733))
        Line((43.0494, 66.1733), (43.0494, 65.4733))
        Line((43.0494, 65.4733), (35.0494, 65.4733))
        Line((35.0494, 65.4733), (34.6022, 65.505))
        RadiusArc((34.6022, 65.505), (31.5627, 63.9904), -3.512)
        Line((31.5627, 63.9904), (31.0247, 63.1822))
        Line((31.0247, 63.1822), (29.442, 60.1868))
        Line((29.442, 60.1868), (28.8431, 59.3309))
        RadiusArc((28.8431, 59.3309), (26.9418, 58.1089), 3.5284)
        Line((26.9418, 58.1089), (25.0, 58.0867))
        # Arc split: sweep=180.0deg >= 150 — emitted as two half-arcs
        RadiusArc((25.0, 58.0867), (0.0, 33.0867), -25.0)
        RadiusArc((0.0, 33.0867), (25.0, 8.0866), -25.0)
        Line((25.0, 8.0866), (26.9418, 8.0644))
        RadiusArc((26.9418, 8.0644), (28.8431, 6.8424), 3.5284)
        Line((28.8431, 6.8424), (29.442, 5.9865))
        Line((29.442, 5.9865), (31.0247, 2.9911))
        Line((31.0247, 2.9911), (31.5627, 2.1829))
        RadiusArc((31.5627, 2.1829), (34.6022, 0.6683), -3.512)
        Line((34.6022, 0.6683), (35.0494, 0.7))
        Line((35.0494, 0.7), (43.0494, 0.7))
        Line((43.0494, 0.7), (43.0494, 0.0))
        Line((43.0494, 0.0), (43.6474, 0.0))
        Line((43.6474, 0.0), (44.1474, 0.7))
        Line((44.1474, 0.7), (51.0494, 0.7))
        Line((51.0494, 0.7), (51.0494, 0.0))
        Line((51.0494, 0.0), (51.6474, 0.0))
        Line((51.6474, 0.0), (52.1474, 0.7))
        Line((52.1474, 0.7), (59.0494, 0.7))
        Line((59.0494, 0.7), (59.0494, 0.0))
        Line((59.0494, 0.0), (59.6474, 0.0))
        Line((59.6474, 0.0), (60.1474, 0.7))
        Line((60.1474, 0.7), (67.0494, 0.7))
        Line((67.0494, 0.7), (67.0494, 0.0))
        Line((67.0494, 0.0), (67.6474, 0.0))
        Line((67.6474, 0.0), (68.1474, 0.7))
        Line((68.1474, 0.7), (75.0494, 0.7))
        Line((75.0494, 0.7), (75.0494, 0.0))
        Line((75.0494, 0.0), (75.6474, 0.0))
        Line((75.6474, 0.0), (76.1474, 0.7))
        Line((76.1474, 0.7), (83.0494, 0.7))
        Line((83.0494, 0.7), (83.0494, 0.0))
        Line((83.0494, 0.0), (83.6474, 0.0))
        Line((83.6474, 0.0), (84.1474, 0.7))
        Line((84.1474, 0.7), (91.0494, 0.7))
        Line((91.0494, 0.7), (91.0494, 0.0))
        Line((91.0494, 0.0), (91.6474, 0.0))
        Line((91.6474, 0.0), (92.1474, 0.7))
        Line((92.1474, 0.7), (99.0494, 0.7))
        Line((99.0494, 0.7), (99.0494, 0.0))
        Line((99.0494, 0.0), (99.6474, 0.0))
        Line((99.6474, 0.0), (100.1474, 0.7))
        Line((100.1474, 0.7), (107.0494, 0.7))
        Line((107.0494, 0.7), (107.0494, 0.0))
        Line((107.0494, 0.0), (107.6474, 0.0))
        Line((107.6474, 0.0), (108.1474, 0.7))
        Line((108.1474, 0.7), (115.0494, 0.7))
        Line((115.0494, 0.7), (115.0494, 0.0))
        Line((115.0494, 0.0), (115.6474, 0.0))
        Line((115.6474, 0.0), (116.1474, 0.7))
        Line((116.1474, 0.7), (123.0494, 0.7))
        Line((123.0494, 0.7), (123.0494, 0.0))
        Line((123.0494, 0.0), (123.6474, 0.0))
        Line((123.6474, 0.0), (124.1474, 0.7))
        Line((124.1474, 0.7), (131.0494, 0.7))
        Line((131.0494, 0.7), (131.0494, 0.0))
        Line((131.0494, 0.0), (131.6474, 0.0))
        Line((131.6474, 0.0), (132.1474, 0.7))
        Line((132.1474, 0.7), (139.0494, 0.7))
        Line((139.0494, 0.7), (139.0494, 0.0))
        Line((139.0494, 0.0), (139.6474, 0.0))
        Line((139.6474, 0.0), (140.1474, 0.7))
        Line((140.1474, 0.7), (147.0494, 0.7))
        Line((147.0494, 0.7), (147.0494, 0.0))
        Line((147.0494, 0.0), (147.6474, 0.0))
        Line((147.6474, 0.0), (148.1474, 0.7))
        Line((148.1474, 0.7), (155.0494, 0.7))
        Line((155.0494, 0.7), (155.0494, 0.0))
        Line((155.0494, 0.0), (155.6474, 0.0))
        Line((155.6474, 0.0), (156.1474, 0.7))
        Line((156.1474, 0.7), (163.0494, 0.7))
        Line((163.0494, 0.7), (163.0494, 0.0))
        Line((163.0494, 0.0), (163.6474, 0.0))
        Line((163.6474, 0.0), (164.1474, 0.7))
        Line((164.1474, 0.7), (171.0494, 0.7))
        Line((171.0494, 0.7), (171.0494, 0.0))
        Line((171.0494, 0.0), (171.6474, 0.0))
        Line((171.6474, 0.0), (172.1474, 0.7))
        Line((172.1474, 0.7), (179.0494, 0.7))
        Line((179.0494, 0.7), (179.0494, 0.0))
        Line((179.0494, 0.0), (179.6474, 0.0))
        Line((179.6474, 0.0), (180.1474, 0.7))
        Line((180.1474, 0.7), (187.0494, 0.7))
        Line((187.0494, 0.7), (187.0494, 0.0))
        Line((187.0494, 0.0), (187.6474, 0.0))
        Line((187.6474, 0.0), (188.1474, 0.7))
        Line((188.1474, 0.7), (195.0494, 0.7))
        Line((195.0494, 0.7), (195.0494, 0.0))
        Line((195.0494, 0.0), (195.6474, 0.0))
        Line((195.6474, 0.0), (196.1474, 0.7))
        Line((196.1474, 0.7), (203.0494, 0.7))
        Line((203.0494, 0.7), (203.0494, 0.0))
        Line((203.0494, 0.0), (203.6474, 0.0))
        Line((203.6474, 0.0), (204.1474, 0.7))
        Line((204.1474, 0.7), (211.0494, 0.7))
        Line((211.0494, 0.7), (211.0494, 0.0))
        Line((211.0494, 0.0), (211.6474, 0.0))
        Line((211.6474, 0.0), (212.1474, 0.7))
        Line((212.1474, 0.7), (215.0494, 0.7))
        RadiusArc((215.0494, 0.7), (215.8494, 1.5), -0.8)
        Line((215.8494, 1.5), (215.8494, 1.9))
        RadiusArc((215.8494, 1.9), (215.0494, 2.7), -0.8)
        Line((215.0494, 2.7), (34.936, 2.7))
        Line((34.936, 2.7), (34.4036, 2.6681))
        RadiusArc((34.4036, 2.6681), (33.34, 3.1738), 1.3911)
        Line((33.34, 3.1738), (32.9445, 3.69))
        Line((32.9445, 3.69), (31.8349, 5.7657))
        Line((31.8349, 5.7657), (30.9374, 7.3663))
        Line((30.9374, 7.3663), (30.2192, 8.3083))
        RadiusArc((30.2192, 8.3083), (27.1853, 10.0618), -5.4741)
        Line((27.1853, 10.0618), (25.0112, 10.0867))
        # Arc split: sweep=180.06deg >= 150 — emitted as two half-arcs
        RadiusArc((25.0112, 10.0867), (2.0, 33.0867), 23.0)
        RadiusArc((2.0, 33.0867), (25.0112, 56.0866), 23.0)
        Line((25.0112, 56.0866), (27.1853, 56.1115))
        RadiusArc((27.1853, 56.1115), (30.2192, 57.865), -5.4741)
        Line((30.2192, 57.865), (30.9374, 58.807))
        Line((30.9374, 58.807), (31.8349, 60.4076))
        Line((31.8349, 60.4076), (32.9445, 62.4833))
        Line((32.9445, 62.4833), (33.34, 62.9995))
        Line((33.34, 62.9995), (33.5668, 63.2157))
        RadiusArc((33.5668, 63.2157), (34.4036, 63.5052), 1.3767)
    _inc_edges_sk_Sketch1 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch1 = Wire.combine(_inc_edges_sk_Sketch1)[0]
_wire_sk_Sketch1 = _wire_sk_Sketch1.moved(_inclined_plane_1.location)
_mkf_sk_Sketch1 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch1.wrapped, True)
_face_sk_Sketch1 = Face(_mkf_sk_Sketch1.Face())

# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude1 ---
    # -- Extrude1 --
    _face = _face_sk_Sketch1
    _vec = Vector(0.0, -1.0, 0.0) * -18.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # Fusion depth expression: -18.000000715 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
