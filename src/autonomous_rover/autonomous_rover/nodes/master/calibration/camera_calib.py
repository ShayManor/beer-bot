"""Checkerboard intrinsics session. Reuses the offline calibrate_camera core."""
import numpy as np

from autonomous_rover.nodes.camera.calibrate_camera import calibrate, _board_points


def find_corners(gray, cols, rows):
    """Sub-pixel chessboard corners, or None if the board isn't found."""
    import cv2
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not found:
        return None
    return cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )


def _center(corners):
    c = corners.reshape(-1, 2).mean(axis=0)
    return [float(c[0]), float(c[1])]


class CameraCalibSession:
    """Accumulate checkerboard views, then solve for K + distortion."""

    def __init__(self, rows, cols, square, views):
        self.rows, self.cols, self.square, self.views = rows, cols, square, views
        self.objp = _board_points(rows, cols, square)
        self.obj_points, self.img_points = [], []
        self.size = None
        self.result = None

    def capture(self, gray):
        """gray = single-channel frame. Adds a view if the board is found."""
        self.size = gray.shape[::-1]
        corners = find_corners(gray, self.cols, self.rows)
        if corners is None:
            return {"found": False, "views": len(self.img_points)}
        self.obj_points.append(self.objp)
        self.img_points.append(corners)
        return {"found": True, "views": len(self.img_points), "center": _center(corners)}

    def solve(self):
        if len(self.img_points) < 3:
            raise ValueError("need >= 3 captured views")
        K, D, rms = calibrate(self.obj_points, self.img_points, self.size)
        self.result = {"K": K.tolist(), "D": np.asarray(D).flatten().tolist(),
                       "rms": float(rms), "width": self.size[0], "height": self.size[1],
                       "views": len(self.img_points)}
        return self.result
