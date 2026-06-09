#!/usr/bin/env python3
"""CAD + payload build-up -> inertial_params.yaml (the URDF's inertial source).

Chassis CoM/tensor come from voxelising chassis.stl (robust to non-watertight
meshes) at uniform density scaled to the bare-chassis mass; payloads are boxes
composited by parallel-axis; wheels use the analytic solid-cylinder tensor.
Deliverable-2 overwrites this file with source: identified.
"""
import os
import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", "autonomous_rover"))
CONFIG = os.path.join(PKG, "description", "config")
MESH = os.path.join(PKG, "description", "meshes")

WHEEL_RADIUS = 0.040
WHEEL_WIDTH = 0.0425
VOXEL_PITCH = 0.004  # 4 mm


def points_inertia(points, mass):
    """CoM and inertia (about CoM) of equal point masses summing to `mass`."""
    com = points.mean(axis=0)
    d = points - com
    m_i = mass / len(points)
    Ixx = (m_i * (d[:, 1] ** 2 + d[:, 2] ** 2)).sum()
    Iyy = (m_i * (d[:, 0] ** 2 + d[:, 2] ** 2)).sum()
    Izz = (m_i * (d[:, 0] ** 2 + d[:, 1] ** 2)).sum()
    Ixy = -(m_i * d[:, 0] * d[:, 1]).sum()
    Ixz = -(m_i * d[:, 0] * d[:, 2]).sum()
    Iyz = -(m_i * d[:, 1] * d[:, 2]).sum()
    I = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
    return com, I


def box_inertia(mass, lx, ly, lz):
    return mass / 12.0 * np.diag([ly * ly + lz * lz,
                                  lx * lx + lz * lz,
                                  lx * lx + ly * ly])


def parallel_axis(I_com, mass, r):
    """Shift an about-CoM tensor to a frame whose origin is offset by -r."""
    rx, ry, rz = r
    d2 = np.array([[ry * ry + rz * rz, -rx * ry, -rx * rz],
                   [-rx * ry, rx * rx + rz * rz, -ry * rz],
                   [-rx * rz, -ry * rz, rx * rx + ry * ry]])
    return I_com + mass * d2


def combine(bodies):
    """bodies: list of (mass, com(3), I_about_com(3x3)) -> combined triple."""
    M = sum(b[0] for b in bodies)
    com = np.sum([b[0] * np.asarray(b[1]) for b in bodies], axis=0) / M
    I = np.zeros((3, 3))
    for m, c, Ic in bodies:
        I += parallel_axis(Ic, m, np.asarray(c) - com)
    return M, com, I


def tensor_dict(I):
    return {"ixx": float(I[0, 0]), "ixy": float(I[0, 1]), "ixz": float(I[0, 2]),
            "iyy": float(I[1, 1]), "iyz": float(I[1, 2]), "izz": float(I[2, 2])}


def main():
    cfg = yaml.safe_load(open(os.path.join(CONFIG, "payloads.yaml")))

    chassis = trimesh.load(os.path.join(MESH, "chassis.stl"), force="mesh")
    vox = chassis.voxelized(pitch=VOXEL_PITCH).fill()
    pts = vox.points
    c_com, c_I = points_inertia(pts, cfg["chassis_bare_mass"])
    bodies = [(cfg["chassis_bare_mass"], c_com, c_I)]
    for p in cfg["payloads"]:
        I = box_inertia(p["mass"], *p["box"])
        bodies.append((p["mass"], np.asarray(p["pose"]), I))
    M, com, I = combine(bodies)

    wm = cfg["wheel_mass_each"]
    wheel_I = np.diag([
        wm / 12.0 * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2),  # Ixx
        0.5 * wm * WHEEL_RADIUS ** 2,                             # Iyy (axle)
        wm / 12.0 * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2),  # Izz
    ])

    out = {
        "meta": {"source": "cad_buildup",
                 "notes": "chassis voxel inertia + payload build-up + analytic wheels"},
        "base_link": {"mass": float(M),
                      "com": [float(v) for v in com],
                      "inertia": tensor_dict(I)},
        "wheel": {"mass": float(wm), "com": [0.0, 0.0, 0.0],
                  "inertia": tensor_dict(wheel_I)},
    }
    with open(os.path.join(CONFIG, "inertial_params.yaml"), "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"base mass {M:.3f} kg, CoM {com}, wheel {wm} kg")


if __name__ == "__main__":
    main()
