import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from autonomous_rover.nodes.master.calibration.camera_calib import CameraCalibSession


def _synth_views(K, board, size, poses):
    img_points = []
    dist = np.zeros(5)
    for rvec, tvec in poses:
        pts, _ = cv2.projectPoints(board, np.array(rvec, float), np.array(tvec, float), K, dist)
        img_points.append(pts.astype(np.float32))
    return img_points


def test_solve_recovers_known_K():
    rows, cols, square = 6, 9, 0.025
    K = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    sess = CameraCalibSession(rows, cols, square, views=4)
    board = sess.objp
    sess.size = (640, 480)
    poses = [
        ([0.0, 0.0, 0.0], [-0.1, -0.07, 0.5]),
        ([0.3, 0.0, 0.0], [-0.1, -0.07, 0.6]),
        ([0.0, 0.3, 0.0], [-0.12, -0.07, 0.55]),
        ([-0.2, 0.2, 0.1], [-0.1, -0.06, 0.5]),
    ]
    for img in _synth_views(K, board, sess.size, poses):
        sess.obj_points.append(board)
        sess.img_points.append(img)
    res = sess.solve()
    assert abs(res["K"][0][0] - 600.0) < 5.0   # fx
    assert abs(res["K"][1][1] - 600.0) < 5.0   # fy
    assert res["rms"] < 1.0


def test_solve_requires_three_views():
    sess = CameraCalibSession(6, 9, 0.025, views=4)
    sess.size = (640, 480)
    with pytest.raises(ValueError):
        sess.solve()
