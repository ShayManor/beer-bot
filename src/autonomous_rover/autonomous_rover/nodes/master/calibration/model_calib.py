"""Metric depth-model affine calibration: fit true = a*raw + b using the floor
plane (known camera height) as geometric ground truth."""
import numpy as np

from autonomous_rover.nodes.localization.projection import backproject, valid_points
from autonomous_rover.nodes.localization.ground_plane import fit_plane


def floor_truth(points, normal, camera_height):
    """Geometric true depth for floor inlier points.

    `points` (N,3) are raw-scale camera-frame floor points; `normal` is the unit
    plane normal from RANSAC (orientation is scale-invariant). For each point the
    ray is points/z (z = points[:,2]); the true depth places the plane at
    `camera_height`: |true_z| = h / |normal . ray|. Returns (raw_z, true_z)."""
    pts = np.asarray(points, dtype=np.float64)
    z = pts[:, 2]
    rays = pts / z[:, None]                     # d_i with d_i.z == 1
    denom = rays @ np.asarray(normal, dtype=np.float64)
    true_z = np.abs(camera_height / denom)
    return z, true_z


def fit_affine(raw, true, trim_sigma=3.0):
    """Robust least-squares true = a*raw + b. Order-independent: sorts by raw for a
    median-slope seed, then two MAD-trim refit passes. Returns (a, b, residual_mean_abs)."""
    raw = np.asarray(raw, dtype=np.float64)
    true = np.asarray(true, dtype=np.float64)
    order = np.argsort(raw)
    raw, true = raw[order], true[order]
    # Median-slope seed (breakdown ~50%); symmetric halves handle odd N.
    half = len(raw) // 2
    dr = np.median(raw[-half:] - raw[:half]) if half else 0.0
    if dr > 1e-9:
        a = np.median(true[-half:] - true[:half]) / dr
        b = np.median(true - a * raw)
    else:  # degenerate spread -> ordinary least squares seed
        a, b = np.polyfit(raw, true, 1)
    keep = np.ones(len(raw), dtype=bool)
    for _ in range(2):
        resid = true - (a * raw + b)
        mad = np.median(np.abs(resid)) or 1.0
        keep = np.abs(resid) < trim_sigma * 1.4826 * mad
        if keep.sum() < 2:
            break
        a, b = np.polyfit(raw[keep], true[keep], 1)
    residual = float(np.abs(true[keep] - (a * raw[keep] + b)).mean()) if keep.any() \
        else float(np.abs(true - (a * raw + b)).mean())
    return float(a), float(b), residual
