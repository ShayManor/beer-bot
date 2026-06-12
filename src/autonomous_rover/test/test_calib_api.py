import os
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from autonomous_rover.nodes.master.calibration.manager import CalibrationManager
from autonomous_rover.nodes.localization.depth import StubDepthEstimator


def _make_manager(tmp_path, monkeypatch, frame, K):
    # Force the model session to use the analytic stub instead of an ONNX model.
    from autonomous_rover.nodes.master.calibration import manager as M
    monkeypatch.setattr(M, "build_calib_estimator",
                        lambda cfg, K: StubDepthEstimator(K, cfg["camera_height"], 0.35))
    pushed = {}

    def fake_push(paths, msg, cwd=None):
        pushed["c"] = (paths, msg, cwd)
        return True, "ok"

    monkeypatch.setattr(M, "commit_and_push", fake_push)
    mgr = CalibrationManager(
        get_frame=lambda: frame,
        get_K=lambda: (K, frame.shape[1], frame.shape[0]),
        config=dict(camera_height=0.19, ransac=dict(threshold=0.01, iterations=200, min_inliers=50),
                    depth_model_path="x.onnx", onnx_providers=["CPUExecutionProvider"],
                    depth_input_size=4, qnn_options={},
                    params_dir=str(tmp_path), repo_dir=str(tmp_path),
                    camera_calib_name="camera_calib.yaml", depth_affine_name="depth_affine.yaml"),
    )
    return mgr, pushed


def test_model_flow_solve_and_apply_writes_and_pushes(tmp_path, monkeypatch):
    K = np.array([[300., 0., 160.], [0., 300., 120.], [0., 0., 1.]])
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    mgr, pushed = _make_manager(tmp_path, monkeypatch, frame, K)
    mgr.model_start()
    assert mgr.model_capture()["ok"] is True
    res = mgr.model_solve()
    assert abs(res["a"] - 1.0) < 0.05
    out = mgr.model_apply()
    assert out["pushed"] is True
    assert os.path.exists(os.path.join(str(tmp_path), "depth_affine.yaml"))
    assert "depth_affine.yaml" in pushed["c"][0][0]


def test_status_reports_both_flows(tmp_path, monkeypatch):
    K = np.array([[300., 0., 160.], [0., 300., 120.], [0., 0., 1.]])
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    mgr, _ = _make_manager(tmp_path, monkeypatch, frame, K)
    st = mgr.status()
    assert "camera" in st and "model" in st


def test_calib_status_endpoint(ros_ctx):
    import pytest as _pytest
    _pytest.importorskip("flask")
    from autonomous_rover.nodes.master.master_node import MasterNode
    with ros_ctx():
        node = MasterNode()
        client = node.app.test_client()
        r = client.get("/calib/status")
        assert r.status_code == 200
        body = r.get_json()
        assert "camera" in body and "model" in body
        node.destroy_node()


def test_calib_camera_capture_requires_start(ros_ctx):
    import pytest as _pytest
    _pytest.importorskip("flask")
    from autonomous_rover.nodes.master.master_node import MasterNode
    with ros_ctx():
        node = MasterNode()
        client = node.app.test_client()
        r = client.post("/calib/camera/capture")
        assert r.status_code == 400
        node.destroy_node()
