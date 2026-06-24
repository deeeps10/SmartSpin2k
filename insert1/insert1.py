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

# 'Sketch26': 2 segments → Line/RadiusArc profile
_inclined_plane_1 = Plane(
    origin=Vector(0.0, 0.0, 13.0),
    x_dir=Vector(1.0, 0.0, 0.0),
    z_dir=Vector(0.0, 0.0, 1.0),
)
with BuildSketch(_inclined_plane_1) as sk_Sketch26:
    with BuildLine():
        Line((-100.6649, -78.8673), (-77.1688, -78.8673))
        # Arc split: sweep=302.65deg >= 150 — emitted as two half-arcs
        RadiusArc((-77.1688, -78.8673), (-88.9169, -32.9048), -24.4826)
        RadiusArc((-88.9169, -32.9048), (-100.6649, -78.8673), -24.4826)
    _inc_edges_sk_Sketch26 = list(BuildSketch._get_context().pending_edges)
# Build inclined-plane face outside BuildSketch (bypasses BRepFill_Filling)
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
_wire_sk_Sketch26 = Wire.combine(_inc_edges_sk_Sketch26)[0]
_wire_sk_Sketch26 = _wire_sk_Sketch26.moved(_inclined_plane_1.location)
_mkf_sk_Sketch26 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _wire_sk_Sketch26.wrapped, True)
_face_sk_Sketch26 = Face(_mkf_sk_Sketch26.Face())
# v16.993: inner loop 0 bore face (segment-based, inclined plane)
with BuildSketch(_inclined_plane_1) as sk_Sketch26_bore0_2:
    with BuildLine():
        # Arc split: sweep=216.26deg >= 150 — emitted as two half-arcs
        RadiusArc((-103.0267, -75.0964), (-102.4763, -70.9147), -2.6046)
        RadiusArc((-102.4763, -70.9147), (-106.6217, -71.6929), -2.6046)
        Line((-106.6217, -71.6929), (-106.6217, -63.4261))
        RadiusArc((-106.6217, -63.4261), (-105.7994, -62.6548), -2.4627)
        RadiusArc((-105.7994, -62.6548), (-105.2885, -61.6884), -5.3181)
        RadiusArc((-105.2885, -61.6884), (-104.7932, -59.9886), -9.5459)
        RadiusArc((-104.7932, -59.9886), (-104.5916, -58.4773), -12.9963)
        RadiusArc((-104.5916, -58.4773), (-104.5614, -56.8368), -14.7157)
        RadiusArc((-104.5614, -56.8368), (-104.7084, -55.2724), -13.5163)
        RadiusArc((-104.7084, -55.2724), (-105.0114, -53.8859), -10.7158)
        RadiusArc((-105.0114, -53.8859), (-105.447, -52.7329), -7.2009)
        RadiusArc((-105.447, -52.7329), (-105.9916, -51.8691), -3.9985)
        RadiusArc((-105.9916, -51.8691), (-106.6217, -51.3501), -1.9279)
        Line((-106.6217, -51.3501), (-106.6217, -43.0628))
        # Arc split: sweep=213.87deg >= 150 — emitted as two half-arcs
        RadiusArc((-106.6217, -43.0628), (-102.5089, -43.8409), -2.6046)
        RadiusArc((-102.5089, -43.8409), (-102.9627, -39.6798), -2.6046)
        Line((-102.9627, -39.6798), (-94.9102, -39.6798))
        RadiusArc((-94.9102, -39.6798), (-94.1073, -40.4736), -2.6534)
        RadiusArc((-94.1073, -40.4736), (-93.1398, -40.9652), -5.5635)
        RadiusArc((-93.1398, -40.9652), (-91.9169, -41.3437), -8.9793)
        RadiusArc((-91.9169, -41.3437), (-90.4907, -41.5869), -12.1906)
        RadiusArc((-90.4907, -41.5869), (-88.9134, -41.6728), -14.366)
        RadiusArc((-88.9134, -41.6728), (-87.336, -41.5869), -14.1906)
        RadiusArc((-87.336, -41.5869), (-85.9098, -41.3437), -11.8752)
        RadiusArc((-85.9098, -41.3437), (-84.3335, -40.8128), -7.7898)
        RadiusArc((-84.3335, -40.8128), (-83.4626, -40.2883), -4.2373)
        RadiusArc((-83.4626, -40.2883), (-82.9166, -39.6798), -2.0856)
        Line((-82.9166, -39.6798), (-74.7886, -39.6798))
        # Arc split: sweep=210.31deg >= 150 — emitted as two half-arcs
        RadiusArc((-74.7886, -39.6798), (-75.3016, -43.7851), -2.6047)
        RadiusArc((-75.3016, -43.7851), (-71.205, -43.2069), -2.6047)
        Line((-71.205, -43.2069), (-71.205, -51.3285))
        RadiusArc((-71.205, -51.3285), (-71.8479, -51.8304), -1.8428)
        RadiusArc((-71.8479, -51.8304), (-72.2299, -52.367), -3.579)
        RadiusArc((-72.2299, -52.367), (-72.7152, -53.4318), -5.9074)
        RadiusArc((-72.7152, -53.4318), (-73.1603, -55.2469), -9.838)
        RadiusArc((-73.1603, -55.2469), (-73.311, -56.8297), -13.4887)
        RadiusArc((-73.311, -56.8297), (-73.28, -58.4909), -14.7105)
        RadiusArc((-73.28, -58.4909), (-72.9696, -60.4863), -12.6368)
        RadiusArc((-72.9696, -60.4863), (-72.5664, -61.7312), -8.6738)
        RadiusArc((-72.5664, -61.7312), (-72.0442, -62.6963), -5.1766)
        RadiusArc((-72.0442, -62.6963), (-71.205, -63.4477), -2.357)
        Line((-71.205, -63.4477), (-71.205, -71.6703))
        # Arc split: sweep=215.71deg >= 150 — emitted as two half-arcs
        RadiusArc((-71.205, -71.6703), (-75.3486, -70.9233), -2.6046)
        RadiusArc((-75.3486, -70.9233), (-74.7892, -75.0964), -2.6046)
        Line((-74.7892, -75.0964), (-82.9116, -75.0964))
        RadiusArc((-82.9116, -75.0964), (-83.4547, -74.4855), -2.0672)
        RadiusArc((-83.4547, -74.4855), (-84.3248, -73.9587), -4.2103)
        RadiusArc((-84.3248, -73.9587), (-85.4695, -73.5381), -7.4017)
        RadiusArc((-85.4695, -73.5381), (-87.3321, -73.181), -11.1604)
        RadiusArc((-87.3321, -73.181), (-89.4541, -73.1045), -14.3223)
        RadiusArc((-89.4541, -73.1045), (-90.9905, -73.246), -13.7864)
        RadiusArc((-90.9905, -73.246), (-92.3572, -73.5381), -11.1869)
        RadiusArc((-92.3572, -73.5381), (-93.5019, -73.9587), -7.7863)
        RadiusArc((-93.5019, -73.9587), (-94.372, -74.4855), -4.5222)
        RadiusArc((-94.372, -74.4855), (-94.9151, -75.0964), -2.2399)
        Line((-94.9151, -75.0964), (-103.0267, -75.0964))
    _inc_bore_edges_sk_Sketch26_bore0_2 = list(BuildSketch._get_context().pending_edges)
_bore_wire_sk_Sketch26_bore0_2 = Wire.combine(_inc_bore_edges_sk_Sketch26_bore0_2)[0]
_bore_wire_sk_Sketch26_bore0_2 = _bore_wire_sk_Sketch26_bore0_2.moved(_inclined_plane_1.location)
_bore_mkf_sk_Sketch26_bore0_2 = BRepBuilderAPI_MakeFace(_inclined_plane_1.wrapped, _bore_wire_sk_Sketch26_bore0_2.wrapped, True)
_bore_face_sk_Sketch26_bore0_2 = Face(_bore_mkf_sk_Sketch26_bore0_2.Face())


# -- Build --
with BuildPart() as part:
    # --- FEATURE: Extrude21 ---
    # -- Extrude21 --
    _face = _face_sk_Sketch26
    _vec = Vector(0.0, 0.0, 1.0) * -13.0
    _solid = Solid.extrude(_face, _vec)
    add(_solid)
    # v16.993: subtract segment-based bore(s) — inclined plane
    _bore_solid_sk_Sketch26_bore0_2 = Solid.extrude(_bore_face_sk_Sketch26_bore0_2, Vector(0.0, 0.0, 1.0) * -13.0)
    part.part = cut_solids(part.part, _bore_solid_sk_Sketch26_bore0_2)
    # Fusion depth expression: -13.000000715 mm
    

# -- Export --
export_step(part.part, 'fusion_features.step')
export_stl(part.part,  'fusion_features.stl')
if _has_ocp: show(part)
