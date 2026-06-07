"""Pinhole back-projection of a metric depth image to a camera-frame cloud."""
import numpy as np


def backproject(depth, K):
    """(depth[H,W] meters, K[3,3]) -> XYZ[H,W,3] in the camera optical frame.

    Pixels with non-finite or non-positive depth become NaN.
    """
    depth = np.asarray(depth, dtype=np.float64)
    h, w = depth.shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    xyz = np.stack([x, y, z], axis=-1)
    invalid = ~np.isfinite(z) | (z <= 0.0)
    xyz[invalid] = np.nan
    return xyz


def valid_points(xyz):
    """Flatten XYZ[H,W,3] to (N,3), keeping only finite rows."""
    pts = xyz.reshape(-1, 3)
    return pts[np.isfinite(pts).all(axis=1)]
