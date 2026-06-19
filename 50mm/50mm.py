
import math
from build123d import *

try:
    from ocp_vscode import show
    _has_ocp = True
except ImportError:
    _has_ocp = False

R_OUT = 34.5
R_BORE = 25.0
H = 30.0
PLATE_T = 0.4
BELL_R = 10.0
TOP_R = 9.0
TOP_FLAT_R = R_OUT - TOP_R

RIB_R = 1.0
RIB_ANGLES = [30, 90, 150, 210, 270, 330]
GROOVE_R = 1.5
GROOVE_ANGLES = [90, 270]

AXIS_X, AXIS_Z = 34.4996, 34.4217
OFFSET = Vector(0, 0.196, 0.529)

BELL_CY = PLATE_T + BELL_R
TOP_CY = H - TOP_R

prof = Curve() + [
    Line((0, 0), (R_OUT, 0)),
    Line((R_OUT, 0), (R_OUT, TOP_CY)),
    RadiusArc((R_OUT, TOP_CY), (TOP_FLAT_R, H), -TOP_R),
    Line((TOP_FLAT_R, H), (R_BORE, H)),
    Line((R_BORE, H), (R_BORE, BELL_CY)),
    RadiusArc((R_BORE, BELL_CY), (R_BORE - BELL_R, PLATE_T), BELL_R),
    Line((R_BORE - BELL_R, PLATE_T), (0, PLATE_T)),
    Line((0, PLATE_T), (0, 0)),
]
body = revolve(make_face(prof.edges()), axis=Axis.Y)

def cyl(p0, d, r, length):
    return extrude(Plane(origin=p0, z_dir=d) * Circle(r), amount=length, dir=d)


RIB_CX = R_BORE - BELL_R - RIB_R
RIB_SECTIONS = [
    (0.40, 5.950), (0.45, 5.913), (0.70, 5.845), (0.95, 5.859),
    (1.20, 5.500), (1.45, 5.290), (1.70, 4.957), (1.95, 4.783),
    (2.20, 4.416), (2.45, 4.243), (2.70, 4.044), (2.95, 3.879),
    (3.20, 3.631), (3.45, 3.460), (3.70, 3.292), (3.95, 3.117),
    (4.20, 2.941), (4.45, 2.795), (4.70, 2.638), (4.95, 2.501),
    (5.20, 2.359), (5.45, 2.233), (5.70, 2.116), (5.95, 1.996),
    (6.20, 1.882), (6.45, 1.769), (6.70, 1.685), (6.95, 1.592),
    (7.20, 1.519), (7.45, 1.437), (7.70, 1.358), (7.95, 1.295),
    (8.20, 1.245), (8.45, 1.200), (8.70, 1.153), (8.95, 1.108),
    (9.20, 1.071), (9.45, 1.053), (9.70, 1.038), (9.95, 1.021),
    (10.20, 1.007), (10.40, 1.000),
]


def rib_crest(y):
    return RIB_CX + math.sqrt(BELL_R ** 2 - (BELL_CY - y) ** 2)


rib0 = cyl((R_BORE, H, 0), (0, -1, 0), RIB_R, H - BELL_CY)
rib0 += loft([Plane(origin=(rib_crest(y) + r, y, 0), z_dir=(0, 1, 0)) *
              Circle(r) for y, r in reversed(RIB_SECTIONS)], ruled=True)
rib0 &= Pos(0, H / 2, 0) * Box(200, H, 200)
for a in RIB_ANGLES:
    body += rib0.rotate(Axis.Y, a)

grv0 = cyl((R_OUT, -1, 0), (0, 1, 0), GROOVE_R, TOP_CY + 1)
GRV_CX = TOP_FLAT_R - GROOVE_R
GRV_SECTIONS = [
    (21.00, 1.517), (21.50, 1.587), (22.00, 1.633), (22.50, 1.751),
    (22.75, 1.800), (23.00, 1.836), (23.25, 1.900), (23.50, 1.991),
    (23.75, 2.044), (24.00, 2.096), (24.25, 2.179), (24.50, 2.293),
    (24.75, 2.417), (25.00, 2.475), (25.25, 2.573), (25.50, 2.701),
    (25.75, 2.857), (26.00, 2.995), (26.25, 3.108), (26.50, 3.257),
    (26.75, 3.456), (27.00, 3.642), (27.25, 3.784), (27.50, 3.985),
    (27.75, 4.254), (28.00, 4.443), (28.25, 4.697), (28.50, 4.977),
    (28.75, 5.245), (29.00, 5.604), (29.25, 5.960), (29.50, 6.420),
    (29.80, 6.772), (30.00, 7.000), (31.50, 7.000),
]


def grv_crest(y):
    return GRV_CX + math.sqrt(TOP_R ** 2 - (y - TOP_CY) ** 2)


grv0 += loft([Plane(origin=(grv_crest(min(y, 30.0)) + r, y, 0),
                    z_dir=(0, 1, 0)) * Circle(r) for y, r in GRV_SECTIONS],
              ruled=True)
for a in GROOVE_ANGLES:
    body -= grv0.rotate(Axis.Y, a)

body = body.moved(Location((AXIS_X, 0, AXIS_Z)))
body = body.solid()

print(f"Volume: {body.volume:.2f} mm^3")
bb = body.bounding_box()
print(f"BBox:   {bb.size.X:.3f} x {bb.size.Y:.3f} x {bb.size.Z:.3f}")

import os

OUT_DIR = "/Users/softage/Downloads"
try:
    os.makedirs(OUT_DIR, exist_ok=True)
except OSError:
    OUT_DIR = os.getcwd()
    print(f"[warn] could not create the output folder, exporting to {OUT_DIR}")

stl_path = os.path.join(OUT_DIR, "Fan_Adapter_50mm_rebuilt.stl")
step_path = os.path.join(OUT_DIR, "Fan_Adapter_50mm_rebuilt.step")
export_stl(body, stl_path, tolerance=0.01, angular_tolerance=0.1)
export_step(body, step_path)
print(f"Exported:\n  {stl_path}\n  {step_path}")

if _has_ocp: show(body)
