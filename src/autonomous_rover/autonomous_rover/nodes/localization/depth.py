"""Metric depth estimators. The real net swaps in behind DepthEstimator later."""
import numpy as np


class DepthEstimator:
    """Interface: estimate(rgb) -> float32 depth image in meters (0 == no return)."""

    def estimate(self, rgb):
        raise NotImplementedError


class StubDepthEstimator(DepthEstimator):
    """Synthetic metric depth of a flat floor at a known camera height.

    Models a camera height `camera_height` above the floor, pitched down by
    `pitch` radians. For each pixel ray the depth is the optical-axis distance to
    the floor plane; rays that point at or above the horizon get 0 (no return).
    Back-projecting the result yields a clean plane at `camera_height`, so the
    ground-plane fit recovers scale 1.0 regardless of pitch.
    """

    def __init__(self, K, camera_height, pitch=0.0):
        self.K = np.asarray(K, dtype=np.float64)
        self.h = float(camera_height)
        self.pitch = float(pitch)

    def estimate(self, rgb):
        height, width = rgb.shape[:2]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        us, vs = np.meshgrid(np.arange(width), np.arange(height))
        # Ray directions in the camera optical frame (z forward, y down).
        dx = (us - cx) / fx
        dy = (vs - cy) / fy
        dz = np.ones_like(dx)
        # Gravity-"down" unit vector in camera coords for a downward pitch.
        g = np.array([0.0, np.cos(self.pitch), np.sin(self.pitch)])
        g_dot_d = dx * g[0] + dy * g[1] + dz * g[2]
        depth = np.zeros((height, width), dtype=np.float32)
        floor = g_dot_d > 1e-6  # rays that actually reach the floor
        depth[floor] = (self.h / g_dot_d[floor]).astype(np.float32)
        return depth
