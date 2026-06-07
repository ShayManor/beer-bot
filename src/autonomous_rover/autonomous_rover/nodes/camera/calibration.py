"""Camera intrinsics: FOV-derived default, plus load/save of a calibration YAML."""
import os

import numpy as np
import yaml


def default_K(width, height, fov_deg):
    """Intrinsics from horizontal FOV. Returns (K[3,3], D[5] zeros)."""
    f = (width / 2.0) / np.tan(np.radians(fov_deg) / 2.0)
    K = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
    return K, np.zeros(5)


def load_camera_info(path, width, height, fov_deg):
    """Load K + distortion from a calibration YAML, or fall back to default_K."""
    if path and os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        K = np.array(data["camera_matrix"]["data"], dtype=float).reshape(3, 3)
        D = np.array(data["distortion_coefficients"]["data"], dtype=float)
        return K, D
    return default_K(width, height, fov_deg)


def save_calibration(path, K, D, width, height):
    """Write K + distortion in the standard ROS camera_info YAML layout."""
    K = np.asarray(K, dtype=float)
    D = np.asarray(D, dtype=float).flatten()
    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(x) for x in K.flatten()]},
        "distortion_coefficients": {"rows": 1, "cols": int(D.size), "data": [float(x) for x in D]},
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f)
