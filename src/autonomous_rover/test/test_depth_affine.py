import numpy as np
from autonomous_rover.nodes.localization import depth as D


def test_save_and_load_roundtrip(tmp_path):
    p = str(tmp_path / "depth_affine.yaml")
    D.save_depth_affine(p, 1.25, -0.04)
    scale, shift = D.load_depth_affine(p)
    assert abs(scale - 1.25) < 1e-9
    assert abs(shift + 0.04) < 1e-9


def test_load_missing_returns_identity():
    scale, shift = D.load_depth_affine("/no/such/file.yaml")
    assert (scale, shift) == (1.0, 0.0)


def test_onnx_estimator_applies_affine(monkeypatch):
    import pytest
    pytest.importorskip("cv2")
    # Stub the ORT session so no model/onnxruntime is needed.
    class FakeSess:
        def get_inputs(self):
            class I: name = "x"
            return [I()]
        def run(self, _out, _feed):
            return [np.ones((4, 4), dtype=np.float32) * 2.0]

    monkeypatch.setattr(D, "make_session", lambda *a, **k: FakeSess())
    est = D.OnnxDepthEstimator("ignored.onnx", ["CPUExecutionProvider"],
                               input_size=4, scale=1.5, shift=0.5)
    bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    out = est.estimate(bgr)
    # raw 2.0 -> 1.5*2.0 + 0.5 = 3.5 everywhere
    assert np.allclose(out, 3.5)
