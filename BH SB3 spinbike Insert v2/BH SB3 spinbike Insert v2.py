from build123d import *
import math

try:
    from ocp_vscode import show
    _has_ocp = True
except ImportError:
    _has_ocp = False

# Parameters
center = (-2981.1926, -568.0884, 130.0)
radius = 340.00022983  # mm
extrude_to_z = 300.0
extrude_height = extrude_to_z - center[2]  # 170.0 mm

# Build
with BuildPart() as part:
    with BuildSketch(Plane(origin=center, z_dir=(0, 0, 1))):
        Circle(radius)
    extrude(amount=extrude_height)


# ── Profile 2 ──────────────────────────────────────────────────────────────
# Points at z=130, extruded down to z=0 (amount = -130 mm)
p1 = (-2892.1825, -782.8799)
p2 = (-3098.7085, -782.8799)
arc_end   = (-2863.6768, -782.8799)
arc_ctr   = (-2981.1926, -568.1096)   # given arc centre

# Radius from centre to arc endpoints
_r = math.sqrt((p2[0] - arc_ctr[0])**2 + (p2[1] - arc_ctr[1])**2)

# Angles from centre to start / end points
_a_start = math.atan2(p2[1]    - arc_ctr[1], p2[0]    - arc_ctr[0])  # ≈ 241.3°
_a_end   = math.atan2(arc_end[1] - arc_ctr[1], arc_end[0] - arc_ctr[0])  # ≈ 298.7°

# Midpoint of the MAJOR arc (CW from 241.3° through 90° to 298.7°)
# → this arc lies on the opposite side of the chord from the minor arc (above the chord)
_a_mid = (_a_start + _a_end) / 2 - math.pi   # halfway point of the major arc
arc_mid = (arc_ctr[0] + _r * math.cos(_a_mid),
           arc_ctr[1] + _r * math.sin(_a_mid))

with BuildPart() as part2:
    with BuildSketch(Plane.XY.offset(130)):
        with BuildLine():
            Polyline([p1, p2])
            ThreePointArc(p2, arc_mid, arc_end)
            Line(arc_end, p1)
        make_face()
    extrude(amount=-130)  # from z=130 down to z=0


# ── Inner-offset cut of Profile 2 → hollows it out down to z=0 ─────────────
_offset_dist = 23.94068549  # mm, inward offset

with BuildPart() as part2_cut:
    add(part2.part)                                  # start from the solid Profile 2
    with BuildSketch(Plane.XY.offset(130)):
        with BuildLine():
            Polyline([p1, p2])
            ThreePointArc(p2, arc_mid, arc_end)
            Line(arc_end, p1)
        make_face()
        # shrink the face inward; Mode.REPLACE keeps ONLY the inner offset face
        offset(amount=-_offset_dist, kind=Kind.ARC, mode=Mode.REPLACE)
    extrude(amount=-130, mode=Mode.SUBTRACT)         # cut inner solid from z=130 → z=0


# ── Profile 3 ─ cut from z=160 up to z=300 ────────────────────────────────
# Segments: line→line→line→arc(R55)→arc(R310)→arc(R55)→line→close
_q1 = (-2981.1903, -568.0891)
_q2 = (-3192.036,  -601.7776)
_q3 = (-3232.9986, -608.3228)
_q4 = (-3291.1356, -562.134);   _r4 = -55.0003
_q5 = (-3243.5944, -403.0266);  _r5 = 310.0024
_q6 = (-3169.6485, -396.3058);  _r6 = -55.004
_q7 = (-3116.7883, -444.4893)

# Single cut instance (no subtract yet)
with BuildPart() as _p3_single:
    with BuildSketch(Plane.XY.offset(160)):
        with BuildLine():
            Polyline([_q1, _q2, _q3])
            RadiusArc(_q3, _q4, _r4)
            RadiusArc(_q4, _q5, _r5)
            RadiusArc(_q5, _q6, _r6)
            Polyline([_q6, _q7, _q1])           # _q7 then close back to _q1
        make_face()
    extrude(amount=140)                          # z=160 → z=300

# Circular pattern: fuse all 7 instances first so internal thin faces are removed,
# then subtract the unified body once for clean geometry.
_pat_axis = Axis((-2981.1903, -568.0891, 0), (0, 0, 1))

with BuildPart() as _cuts_fused:
    for i in range(7):
        add(_p3_single.part.rotate(_pat_axis, i * 360.0 / 7))

with BuildPart() as part1_cut:
    add(part.part)
    add(_cuts_fused.part, mode=Mode.SUBTRACT)

# ── Circle cut to remove thin surfaces from the pattern ─────────────────────
_circ_ctr = (-2981.1903, -568.0891)
_circ_r   = 255.00195283  # mm

with BuildPart() as part1_final:
    add(part1_cut.part)
    with BuildSketch(Plane(origin=(_circ_ctr[0], _circ_ctr[1], 160), z_dir=(0, 0, 1))):
        Circle(_circ_r)
    extrude(amount=140, mode=Mode.SUBTRACT)  # z=160 → z=300

# ── Combine all parts into one final output ──────────────────────────────────
with BuildPart() as final_output:
    add(part1_final.part)
    add(part2_cut.part)

# ── Visualise ────────────────────────────────────────────────────────────────
if _has_ocp:
    show(final_output)

# ── Export final output only ─────────────────────────────────────────────────
_out = "/Users/softage/Desktop/"
export_step(final_output.part, _out + "BH_SB3_spinbike_insert_v2.step")
export_stl( final_output.part, _out + "BH_SB3_spinbike_insert_v2.stl")