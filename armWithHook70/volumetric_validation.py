"""
volumetric_validation.py

Place this script inside any part folder and run it:
    python volumetric_validation.py

What it does:
- Looks in its own folder for a pair: {stem}.stl and source_{stem}.stl
- Computes absolute and percentage volume difference
- Logs result to volumetric_validation.log in the same folder

Requires:
    pip install numpy-stl
"""

import logging
from datetime import datetime
from pathlib import Path

import numpy as np
from stl import mesh


LOG_FILE = "volumetric_validation.log"


# ── Logging setup ─────────────────────────────────────────────────────────────

def setup_logging(folder: Path):
    log = logging.getLogger("vol_validation")
    log.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s  %(levelname)-8s  %(message)s",
                            datefmt="%Y-%m-%d %H:%M:%S")
    fh = logging.FileHandler(folder / LOG_FILE, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(fmt)
    log.addHandler(fh)
    log.addHandler(ch)
    return log


# ── Volume calculation ────────────────────────────────────────────────────────

def signed_volume_of_triangle(v0, v1, v2):
    return np.dot(v0, np.cross(v1, v2)) / 6.0


def compute_volume(stl_path: Path) -> float:
    m = mesh.Mesh.from_file(str(stl_path))
    total = 0.0
    for triangle in m.vectors:
        total += signed_volume_of_triangle(triangle[0], triangle[1], triangle[2])
    return abs(total)


# ── Pair detection ────────────────────────────────────────────────────────────

def find_pair(folder: Path):
    """
    Find one pair of {stem}.stl and source_{stem}.stl in the folder.
    Returns (output_stl, source_stl, stem) or (None, None, None).
    """
    stl_files = [f for f in folder.iterdir() if f.suffix == ".stl"]

    # Build a map of stem → file for non-source STLs
    output_map = {f.stem: f for f in stl_files if not f.stem.startswith("source_")}

    for stem, output_file in output_map.items():
        source_file = folder / f"source_{stem}.stl"
        if source_file.exists():
            return output_file, source_file, stem

    return None, None, None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    folder = Path(__file__).parent.resolve()
    log = setup_logging(folder)

    log.info("=" * 60)
    log.info(f"volumetric_validation.py started — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log.info(f"Folder: {folder}")

    output_stl, source_stl, stem = find_pair(folder)

    if output_stl is None:
        log.error("No matching STL pair found. Expected: {stem}.stl and source_{stem}.stl in the same folder.")
        return

    log.info(f"Pair found: {output_stl.name} + {source_stl.name}")

    try:
        vol_output = compute_volume(output_stl)
        vol_source = compute_volume(source_stl)

        abs_diff = abs(vol_output - vol_source)
        pct_diff = (abs_diff / vol_source * 100) if vol_source != 0 else float("inf")

        log.info(f"Source volume : {vol_source:.2f} mm³")
        log.info(f"Output volume : {vol_output:.2f} mm³")
        log.info(f"Difference    : {abs_diff:.2f} mm³  ({pct_diff:.2f}%)")

    except Exception as e:
        log.error(f"Failed to compute volume: {e}")

    log.info("=" * 60)
    log.info(f"Log saved to: {folder / LOG_FILE}")


if __name__ == "__main__":
    main()
