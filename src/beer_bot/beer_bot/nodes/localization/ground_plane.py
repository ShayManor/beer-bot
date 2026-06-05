"""RANSAC dominant-plane fit and metric-scale recovery from camera height."""
from collections import namedtuple

import numpy as np

PlaneFit = namedtuple("PlaneFit", ["scale", "normal", "distance", "offset", "inliers", "residual"])


def fit_plane(points, threshold=0.02, iterations=200, min_inliers=50, seed=0):
    """RANSAC plane through (N,3) points. Returns (normal_unit, d, inlier_mask) or None.

    Plane equation: normal . x + d = 0.
    """
    points = np.asarray(points, dtype=np.float64)
    n = len(points)
    if n < 3:
        return None
    rng = np.random.default_rng(seed)
    best_inliers = None
    for _ in range(iterations):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -normal.dot(p0)
        dist = np.abs(points.dot(normal) + d)
        inliers = dist < threshold
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    if best_inliers is None or best_inliers.sum() < min_inliers:
        return None
    # Least-squares refit on the inlier set (centroid + smallest singular vector).
    pts = points[best_inliers]
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    d = -normal.dot(centroid)
    return normal, d, best_inliers


def ground_scale(points, camera_height, threshold=0.02, iterations=200, min_inliers=50, seed=0):
    """Scale factor that places the dominant plane at `camera_height` from the camera.

    Returns a PlaneFit (scale, normal, distance, inliers, residual) or None.
    """
    result = fit_plane(points, threshold, iterations, min_inliers, seed)
    if result is None:
        return None
    normal, d, inliers = result
    distance = abs(d)  # distance from camera origin to the plane (normal is unit)
    if distance < 1e-6:
        return None
    pts = np.asarray(points, dtype=np.float64)[inliers]
    residual = float(np.abs(pts.dot(normal) + d).mean())
    return PlaneFit(
        scale=camera_height / distance,
        normal=normal,
        distance=distance,
        offset=d,  # signed plane offset, kept paired with `normal`
        inliers=int(inliers.sum()),
        residual=residual,
    )
