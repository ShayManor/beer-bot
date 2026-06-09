#!/usr/bin/env python3
"""Turn the fused Waveshare WAVE ROVER STL into URDF-ready per-part meshes.

Downloads the official model, scales mm->m, reorients to REP-103 (x-forward,
z-up), splits chassis vs. the 4 wheels, recenters base_link at the axle plane,
extracts one canonical wheel, decimates, and writes meshes/ + config/geometry.yaml.

Run from anywhere; outputs land in the package's description/ tree.
Re-run only when the CAD changes — commit the outputs.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import urllib.request

import numpy as np
import trimesh
import yaml

CAD_URL = "https://files.waveshare.com/upload/e/ec/WAVE_ROVER_MODEL_STL.rar"
STL_NAME = "WAVE ROVER_MODEL_STL.stl"

# Official spec constants (metres).
WHEEL_RADIUS = 0.040
WHEEL_WIDTH = 0.0425
WHEEL_DIAM = 2 * WHEEL_RADIUS

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", "autonomous_rover"))
MESH_DIR = os.path.join(PKG, "description", "meshes")
CONFIG_DIR = os.path.join(PKG, "description", "config")


def fetch_stl(cache_dir):
    stl_path = os.path.join(cache_dir, STL_NAME)
    if os.path.exists(stl_path):
        return stl_path
    rar_path = os.path.join(cache_dir, "model.rar")
    print(f"downloading {CAD_URL}")
    req = urllib.request.Request(CAD_URL, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as r, open(rar_path, "wb") as f:
        f.write(r.read())
    # bsdtar (libarchive) reads rar; present on macOS/Linux dev images.
    subprocess.run(["bsdtar", "-xf", rar_path, "-C", cache_dir], check=True)
    if not os.path.exists(stl_path):
        sys.exit(f"extraction did not yield {STL_NAME!r}")
    return stl_path


def classify(components):
    """Assign each connected component to a wheel corner or the chassis.

    In the raw mesh frame the body length is +Y, width +X, height +Z. Wheels
    are the components that reach the X extremes (they bulge past the body) and
    sit at the length-ends, so they cluster at the four (sign x, sign y) corners
    low in Z. Everything else is chassis. Robust to multi-shell wheels.
    """
    all_pts = np.vstack([c.vertices for c in components])
    cx, cy, cz = all_pts.mean(axis=0)
    half_x = (all_pts[:, 0].max() - all_pts[:, 0].min()) / 2
    half_y = (all_pts[:, 1].max() - all_pts[:, 1].min()) / 2
    wheels = {("+", "+"): [], ("+", "-"): [], ("-", "+"): [], ("-", "-"): []}
    chassis = []
    for comp in components:
        ccx, ccy, ccz = comp.vertices.mean(axis=0)
        is_outer_x = abs(ccx - cx) > 0.55 * half_x
        is_end_y = abs(ccy - cy) > 0.45 * half_y
        is_low = ccz < cz
        if is_outer_x and is_end_y and is_low:
            wheels[("+" if ccx > cx else "-", "+" if ccy > cy else "-")].append(comp)
        else:
            chassis.append(comp)
    return wheels, chassis


def merged(meshes):
    return trimesh.util.concatenate(meshes)


def decimate(mesh, target_faces):
    if len(mesh.faces) <= target_faces:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(face_count=target_faces)
    except Exception as e:  # backend missing -> keep full mesh, warn
        print(f"  decimation skipped ({e}); keeping {len(mesh.faces)} faces")
        return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default=os.path.join(tempfile.gettempdir(), "waverover_cad"))
    ap.add_argument("--chassis-faces", type=int, default=8000)
    ap.add_argument("--wheel-faces", type=int, default=1500)
    args = ap.parse_args()
    os.makedirs(args.cache, exist_ok=True)
    os.makedirs(MESH_DIR, exist_ok=True)
    os.makedirs(CONFIG_DIR, exist_ok=True)

    stl = fetch_stl(args.cache)
    scene = trimesh.load(stl, force="mesh")
    scene.apply_scale(0.001)  # mm -> m
    comps = scene.split(only_watertight=False)
    print(f"loaded {len(scene.faces)} faces in {len(comps)} components")

    wheels, chassis_parts = classify(comps)
    counts = {k: len(v) for k, v in wheels.items()}
    print(f"wheel components per corner: {counts}; chassis components: {len(chassis_parts)}")
    if any(len(v) == 0 for v in wheels.values()):
        sys.exit("FAILED to find 4 wheels — inspect counts above and tune classify() thresholds")

    chassis = merged(chassis_parts)
    wheel_meshes = {k: merged(v) for k, v in wheels.items()}

    # Reorient: yaw -90 deg about Z maps mesh +Y(length)->+X(forward),
    # so the wheel axle (mesh +X) -> +Y, matching URDF axis (0 1 0).
    R = trimesh.transformations.rotation_matrix(-math.pi / 2, [0, 0, 1])
    chassis.apply_transform(R)
    for m in wheel_meshes.values():
        m.apply_transform(R)

    centers = {k: m.vertices.mean(axis=0) for k, m in wheel_meshes.items()}
    origin = np.mean(list(centers.values()), axis=0)  # centroid of 4 wheel centers
    origin[2] = float(np.mean([c[2] for c in centers.values()]))  # force axle plane to z=0
    T = trimesh.transformations.translation_matrix(-origin)
    chassis.apply_transform(T)
    for m in wheel_meshes.values():
        m.apply_transform(T)
    centers = {k: m.vertices.mean(axis=0) for k, m in wheel_meshes.items()}

    # Validate wheel geometry against the spec before trusting the split.
    for k, m in wheel_meshes.items():
        ext = m.extents  # x,y,z extents
        diam = max(ext[0], ext[2])
        if not (abs(diam - WHEEL_DIAM) < 0.012 and abs(ext[1] - WHEEL_WIDTH) < 0.015):
            sys.exit(f"wheel {k} bbox {ext} != ~cyl(d={WHEEL_DIAM}, w={WHEEL_WIDTH}); tune classify()")

    # Canonical wheel: pick +,+ wheel, recenter on its own axle.
    wheel = wheel_meshes[("+", "+")].copy()
    wheel.apply_translation(-wheel.vertices.mean(axis=0))

    chassis = decimate(chassis, args.chassis_faces)
    wheel = decimate(wheel, args.wheel_faces)
    chassis.export(os.path.join(MESH_DIR, "chassis.stl"))
    wheel.export(os.path.join(MESH_DIR, "wheel.stl"))

    # base_link frame: x forward, y left. Map corners to named wheels.
    # After -90deg yaw: mesh +Y(front) -> +X; mesh +X -> -Y (right).
    def name(sign_x, sign_y):
        front = "front" if sign_y == "+" else "rear"
        side = "right" if sign_x == "+" else "left"
        return f"{front}_{side}"

    wheel_centers = {name(sx, sy): [float(v) for v in centers[(sx, sy)]]
                     for (sx, sy) in centers}
    cext = chassis.extents
    ccenter = (chassis.bounds[0] + chassis.bounds[1]) / 2  # bbox center in base_link frame
    geometry = {
        "forward_yaw": -math.pi / 2,
        "wheel_centers": wheel_centers,
        "chassis_bbox": [float(cext[0]), float(cext[1]), float(cext[2])],
        "chassis_center": [float(ccenter[0]), float(ccenter[1]), float(ccenter[2])],
        "track_width": float(abs(wheel_centers["front_left"][1] -
                                 wheel_centers["front_right"][1])),
        "wheelbase": float(abs(wheel_centers["front_left"][0] -
                               wheel_centers["rear_left"][0])),
    }
    with open(os.path.join(CONFIG_DIR, "geometry.yaml"), "w") as f:
        yaml.safe_dump(geometry, f, sort_keys=True)
    print("wrote geometry.yaml:")
    print(yaml.safe_dump(geometry, sort_keys=True))


if __name__ == "__main__":
    main()
