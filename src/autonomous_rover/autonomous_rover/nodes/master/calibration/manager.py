"""Ties calibration sessions to live master frames and handles Apply (write+push)."""
import os

import numpy as np

from autonomous_rover.nodes.localization.depth import (
    OnnxDepthEstimator, save_depth_affine,
)
from autonomous_rover.nodes.camera.calibration import save_calibration
from autonomous_rover.nodes.master.calibration.gitpush import commit_and_push
from autonomous_rover.nodes.master.calibration.camera_calib import CameraCalibSession
from autonomous_rover.nodes.master.calibration.model_calib import ModelCalibSession


def build_calib_estimator(cfg, K):
    """Raw (uncalibrated) depth estimator for the model flow. Patched in tests."""
    return OnnxDepthEstimator(cfg["depth_model_path"], cfg["onnx_providers"],
                              [cfg["qnn_options"] if p == "QNNExecutionProvider" else {}
                               for p in cfg["onnx_providers"]],
                              cfg["depth_input_size"])


class CalibrationManager:
    def __init__(self, get_frame, get_K, config):
        self._get_frame = get_frame
        self._get_K = get_K
        self.cfg = config
        self.camera = None
        self.model = None

    # --- helpers ----------------------------------------------------------
    def _gray(self):
        import cv2
        frame = self._get_frame()
        if frame is None:
            raise ValueError("no camera frame yet")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _params_path(self, name):
        return os.path.join(self.cfg["params_dir"], name)

    def _apply(self, paths, message):
        repo = self.cfg.get("repo_dir") or None
        if not repo:
            return {"written": paths, "pushed": False, "push_output": "no repo_dir set"}
        ok, out = commit_and_push(paths, message, cwd=repo)
        return {"written": paths, "pushed": ok, "push_output": out}

    # --- camera flow ------------------------------------------------------
    def camera_start(self, rows, cols, square, views):
        self.camera = CameraCalibSession(rows, cols, square, views)
        return {"started": True, "views": 0}

    def camera_capture(self):
        if self.camera is None:
            raise ValueError("start the camera calibration first")
        return self.camera.capture(self._gray())

    def camera_solve(self):
        if self.camera is None:
            raise ValueError("start the camera calibration first")
        return self.camera.solve()

    def camera_apply(self):
        if self.camera is None or self.camera.result is None:
            raise ValueError("solve the camera calibration first")
        r = self.camera.result
        path = self._params_path(self.cfg["camera_calib_name"])
        save_calibration(path, np.array(r["K"]), np.array(r["D"]), r["width"], r["height"])
        return self._apply([path], f"calib(camera): K/D rms={r['rms']:.3f}px")

    def camera_reset(self):
        self.camera = None
        return {"reset": True}

    def camera_undistort_jpeg(self):
        import cv2
        if self.camera is None or self.camera.result is None:
            raise ValueError("solve the camera calibration first")
        frame = self._get_frame()
        if frame is None:
            raise ValueError("no camera frame yet")
        r = self.camera.result
        K, D = np.array(r["K"]), np.array(r["D"])
        und = cv2.undistort(frame, K, D)
        ok, buf = cv2.imencode(".jpg", und)
        if not ok:
            raise ValueError("encode failed")
        return buf.tobytes()

    # --- model flow -------------------------------------------------------
    def model_start(self):
        kinfo = self._get_K()
        if kinfo is None:
            raise ValueError("no camera_info yet")
        K, _w, _h = kinfo
        est = build_calib_estimator(self.cfg, K)
        self.model = ModelCalibSession(est, K, self.cfg["camera_height"], self.cfg["ransac"])
        return {"started": True, "pairs": 0}

    def model_capture(self):
        if self.model is None:
            raise ValueError("start the model calibration first")
        return self.model.capture(self._get_frame())

    def model_solve(self):
        if self.model is None:
            raise ValueError("start the model calibration first")
        return self.model.solve()

    def model_probe(self):
        if self.model is None:
            raise ValueError("start the model calibration first")
        return {"points": self.model.probe(self._get_frame())}

    def model_apply(self):
        if self.model is None or self.model.result is None:
            raise ValueError("solve the model calibration first")
        r = self.model.result
        path = self._params_path(self.cfg["depth_affine_name"])
        save_depth_affine(path, r["a"], r["b"])
        return self._apply([path], f"calib(depth): a={r['a']:.4f} b={r['b']:.4f} res={r['residual']:.4f}")

    def model_reset(self):
        self.model = None
        return {"reset": True}

    # --- status -----------------------------------------------------------
    def status(self):
        cam = None if self.camera is None else {
            "views": len(self.camera.img_points), "target": self.camera.views,
            "result": self.camera.result}
        mod = None if self.model is None else {
            "pairs": len(self.model.raw), "result": self.model.result}
        return {"camera": cam, "model": mod}
