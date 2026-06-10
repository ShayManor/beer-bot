import numpy as np
import pytest


def test_backproject_principal_and_offset_pixels():
    from autonomous_rover.nodes.localization.projection import backproject

    K = np.array([[100.0, 0.0, 2.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    depth = np.full((3, 5), 2.0)
    xyz = backproject(depth, K)
    # principal point (cx=2, cy=1) -> (0, 0, z)
    assert np.allclose(xyz[1, 2], [0.0, 0.0, 2.0])
    # one pixel right of cx: x = (3-2)*2/100 = 0.02
    assert np.allclose(xyz[1, 3], [0.02, 0.0, 2.0])
    # one pixel below cy: y = (2-1)*2/100 = 0.02
    assert np.allclose(xyz[2, 2], [0.0, 0.02, 2.0])


def test_backproject_invalid_depth_is_nan_and_dropped():
    from autonomous_rover.nodes.localization.projection import backproject, valid_points

    K = np.array([[100.0, 0.0, 2.0], [0.0, 100.0, 1.0], [0.0, 0.0, 1.0]])
    depth = np.full((2, 2), 1.0)
    depth[0, 0] = 0.0      # no return
    depth[0, 1] = -1.0     # invalid
    xyz = backproject(depth, K)
    assert np.isnan(xyz[0, 0]).all()
    assert np.isnan(xyz[0, 1]).all()
    pts = valid_points(xyz)
    assert pts.shape == (2, 3)  # only the two valid pixels survive


def _synth_plane(distance, n_pts=800, noise=0.0, seed=0):
    """Points on the plane y = distance (normal (0,1,0)), x/z spread out."""
    rng = np.random.default_rng(seed)
    xz = rng.uniform(-1.0, 1.0, (n_pts, 2))
    y = np.full((n_pts, 1), distance) + rng.normal(0.0, noise, (n_pts, 1))
    return np.hstack([xz[:, :1], y, xz[:, 1:]])


def test_ground_scale_recovers_known_distance_and_scale():
    from autonomous_rover.nodes.localization.ground_plane import ground_scale

    pts = _synth_plane(distance=0.40, noise=0.002)
    fit = ground_scale(pts, camera_height=0.1524)
    assert fit is not None
    assert fit.distance == pytest.approx(0.40, rel=1e-2)
    assert fit.scale == pytest.approx(0.1524 / 0.40, rel=1e-2)
    # normal is (approximately) the y axis
    assert abs(abs(fit.normal[1]) - 1.0) < 1e-2


def test_ground_scale_returns_none_on_degenerate_input():
    from autonomous_rover.nodes.localization.ground_plane import ground_scale

    assert ground_scale(np.zeros((2, 3)), camera_height=0.1524) is None


def test_ground_scale_handles_full_resolution_cloud():
    # ~150k points (a 640x480 floor) must use the economy SVD; the full SVD
    # would try to allocate an N x N matrix (~170 GiB) and crash.
    from autonomous_rover.nodes.localization.ground_plane import ground_scale

    pts = _synth_plane(distance=0.1524, n_pts=150_000, noise=0.002)
    fit = ground_scale(pts, camera_height=0.1524)
    assert fit is not None
    assert fit.distance == pytest.approx(0.1524, rel=1e-2)


def _test_K(w=160, h=120, f=120.0):
    return np.array([[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]])


def test_stub_depth_is_a_floor_at_camera_height():
    from autonomous_rover.nodes.localization.depth import StubDepthEstimator
    from autonomous_rover.nodes.localization.projection import backproject, valid_points
    from autonomous_rover.nodes.localization.ground_plane import ground_scale

    K = _test_K()
    height = 0.1524
    est = StubDepthEstimator(K, camera_height=height, pitch=0.0)
    depth = est.estimate(np.zeros((120, 160, 3), dtype=np.uint8))
    assert depth.dtype == np.float32
    assert depth.shape == (120, 160)
    assert np.isfinite(depth[depth > 0]).all()

    fit = ground_scale(valid_points(backproject(depth, K)), camera_height=height)
    assert fit is not None
    assert fit.distance == pytest.approx(height, rel=1e-2)
    assert fit.scale == pytest.approx(1.0, rel=1e-2)


def test_stub_depth_scale_is_pitch_invariant():
    from autonomous_rover.nodes.localization.depth import StubDepthEstimator
    from autonomous_rover.nodes.localization.projection import backproject, valid_points
    from autonomous_rover.nodes.localization.ground_plane import ground_scale

    K = _test_K()
    height = 0.1524
    est = StubDepthEstimator(K, camera_height=height, pitch=0.25)
    depth = est.estimate(np.zeros((120, 160, 3), dtype=np.uint8))
    fit = ground_scale(valid_points(backproject(depth, K)), camera_height=height)
    assert fit is not None
    assert fit.distance == pytest.approx(height, rel=2e-2)


def test_height_inches_zero_on_floor_positive_above():
    from autonomous_rover.nodes.localization.depth import StubDepthEstimator
    from autonomous_rover.nodes.localization.projection import backproject
    from autonomous_rover.nodes.localization.ground_plane import ground_scale
    from autonomous_rover.nodes.localization.overlay import height_inches

    K = _test_K()
    height = 0.1524
    depth = StubDepthEstimator(K, camera_height=height, pitch=0.0).estimate(
        np.zeros((120, 160, 3), dtype=np.uint8)
    )
    xyz = backproject(depth, K)
    from autonomous_rover.nodes.localization.projection import valid_points
    fit = ground_scale(valid_points(xyz), camera_height=height)

    h_floor = height_inches(xyz, fit.normal, fit.offset)
    floor = np.isfinite(h_floor)
    # Floor pixels read ~0 inches above the floor.
    assert np.nanmax(np.abs(h_floor[floor])) < 0.25
    # Shifting every point 0.1 m along the plane normal changes height by ~3.94 in
    # (sign depends on normal orientation, so compare the magnitude of the delta).
    h_shift = height_inches(xyz + fit.normal * 0.1, fit.normal, fit.offset)
    delta = np.nanmedian(h_shift[floor] - h_floor[floor])
    assert abs(delta) == pytest.approx(3.937, abs=0.2)


def test_preprocess_shape_layout_and_normalization():
    from autonomous_rover.nodes.localization.depth import preprocess

    # identity "resize" so we control the output size and skip cv2
    def fake_resize(img, size):
        w, h = size
        assert img.shape[:2] == (h, w)  # already at target here
        return img

    bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    bgr[..., 0] = 255  # B channel = 255, others 0
    x = preprocess(bgr, 4, fake_resize)

    assert x.shape == (1, 3, 4, 4)
    assert x.dtype == np.float32
    assert x.flags["C_CONTIGUOUS"]
    # BGR->RGB: R channel (index 0 after transpose) came from bgr[...,2]==0
    r = (0.0 - 0.485) / 0.229
    b = (1.0 - 0.406) / 0.225
    assert np.allclose(x[0, 0], r)   # red plane <- was 0
    assert np.allclose(x[0, 2], b)   # blue plane <- was 255


def test_postprocess_resizes_and_passthrough():
    from autonomous_rover.nodes.localization.depth import postprocess

    def fake_resize(img, size):
        w, h = size
        return np.full((h, w), float(img.flat[0]), dtype=img.dtype)

    raw = np.ones((1, 1, 8, 8), dtype=np.float32) * 2.5
    out = postprocess(raw, (3, 5), fake_resize)
    assert out.shape == (3, 5)
    assert out.dtype == np.float32
    assert np.allclose(out, 2.5)

    raw2 = np.full((3, 5), 1.0, dtype=np.float32)
    out2 = postprocess(raw2, (3, 5), fake_resize)  # already target shape -> no resize
    assert out2.shape == (3, 5)
    assert np.allclose(out2, 1.0)


def test_parse_qnn_options():
    from autonomous_rover.nodes.localization.depth import parse_qnn_options

    opts = parse_qnn_options(["backend_path=libQnnHtp.so", "htp_arch=68", "", "  "])
    assert opts == {"backend_path": "libQnnHtp.so", "htp_arch": "68"}
    assert parse_qnn_options([]) == {}
    # values may contain '=' (split on the first only)
    assert parse_qnn_options(["k=val=extra"]) == {"k": "val=extra"}


def test_make_session_missing_model_raises(tmp_path):
    from autonomous_rover.nodes.localization.depth import make_session

    with pytest.raises(FileNotFoundError):
        make_session(str(tmp_path / "nope.onnx"), ["CPUExecutionProvider"])


def test_onnx_estimator_estimate_with_fake_session(monkeypatch, tmp_path):
    cv2 = pytest.importorskip("cv2")  # estimate() needs real cv2.resize
    from autonomous_rover.nodes.localization import depth as depth_mod

    model = tmp_path / "m.onnx"
    model.write_bytes(b"stub")  # existence check only; session is faked

    class FakeSession:
        def get_inputs(self):
            class I:
                name = "input"
            return [I()]

        def run(self, _outputs, feeds):
            x = feeds["input"]
            assert x.shape == (1, 3, 8, 8)  # input_size honored
            return [np.full((1, 1, 8, 8), 3.0, dtype=np.float32)]

    monkeypatch.setattr(depth_mod, "make_session",
                        lambda *a, **k: FakeSession())

    est = depth_mod.OnnxDepthEstimator(str(model), ["CPUExecutionProvider"],
                                       input_size=8)
    bgr = np.zeros((5, 7, 3), dtype=np.uint8)
    out = est.estimate(bgr)
    assert out.shape == (5, 7)        # resized back to camera resolution
    assert out.dtype == np.float32
    assert np.allclose(out, 3.0)      # metric meters passthrough


def test_compile_qnn_builds_options_and_calls_make_session(monkeypatch, tmp_path):
    from autonomous_rover.nodes.localization import compile_qnn

    model = tmp_path / "m.onnx"
    model.write_bytes(b"stub")
    out = tmp_path / "m_ctx.onnx"

    seen = {}

    def fake_make_session(model_path, providers, provider_options=None,
                          compile_ctx=False, ctx_path=None):
        seen.update(model_path=model_path, providers=providers,
                    provider_options=provider_options,
                    compile_ctx=compile_ctx, ctx_path=ctx_path)
        return object()

    monkeypatch.setattr(compile_qnn, "make_session", fake_make_session)
    compile_qnn.main(["--model", str(model), "--out", str(out),
                      "--options", "backend_path=libQnnHtp.so", "htp_arch=68"])

    assert seen["model_path"] == str(model)
    assert seen["providers"] == ["QNNExecutionProvider", "CPUExecutionProvider"]
    assert seen["provider_options"] == [
        {"backend_path": "libQnnHtp.so", "htp_arch": "68"}, {}]
    assert seen["compile_ctx"] is True
    assert seen["ctx_path"] == str(out)
