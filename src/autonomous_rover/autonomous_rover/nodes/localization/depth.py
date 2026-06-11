"""Metric depth estimators. The real net swaps in behind DepthEstimator later."""
import os
import numpy as np


def load_depth_affine(path):
    """(scale, shift) from a depth-affine YAML, or identity (1.0, 0.0)."""
    if path and os.path.exists(path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return float(data.get("depth_scale", 1.0)), float(data.get("depth_shift", 0.0))
    return 1.0, 0.0


def save_depth_affine(path, scale, shift):
    """Write {depth_scale, depth_shift} as YAML."""
    import yaml
    with open(path, "w") as f:
        yaml.safe_dump({"depth_scale": float(scale), "depth_shift": float(shift)}, f)


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


_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocess(bgr, size, resize):
    """BGR uint8 HxWx3 -> NCHW float32 1x3x(size)x(size), RGB + ImageNet-normalized."""
    rgb = resize(bgr[..., ::-1], (size, size))  # BGR->RGB, then square resize
    x = rgb.astype(np.float32) / 255.0
    x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None], dtype=np.float32)


def postprocess(raw, out_hw, resize):
    """Model output (1x1xHxW | 1xHxW | HxW) -> float32 metric depth at out_hw (h, w)."""
    depth = np.asarray(raw, dtype=np.float32).squeeze()
    h, w = out_hw
    if depth.shape != (h, w):
        depth = resize(depth, (w, h))  # cv2 target order is (w, h)
    return depth.astype(np.float32, copy=False)


def parse_qnn_options(items):
    """["k=v", ...] -> {"k": "v"}; blanks ignored. Shared by the node and the CLI."""
    out = {}
    for item in items:
        if "=" not in item:
            continue
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def make_session(model_path, providers, provider_options=None,
                 compile_ctx=False, ctx_path=None):
    """Build an ORT InferenceSession; optionally dump a QNN context binary.

    onnxruntime is imported lazily so the stub path and CI never need it.
    """
    if not os.path.exists(model_path):
        raise FileNotFoundError(model_path)
    import onnxruntime as ort

    so = ort.SessionOptions()
    if compile_ctx:
        so.add_session_config_entry("ep.context_enable", "1")
        if ctx_path:
            so.add_session_config_entry("ep.context_file_path", ctx_path)
    providers = providers or ["CPUExecutionProvider"]
    if provider_options is None:
        provider_options = [{} for _ in range(len(providers))]
    return ort.InferenceSession(model_path, sess_options=so,
                                providers=providers,
                                provider_options=provider_options)


class OnnxDepthEstimator(DepthEstimator):
    """Depth Anything V2 Metric via ONNX Runtime. estimate(bgr) -> meters @ camera res.

    Applies the calibrated affine `scale*raw + shift` (identity by default)."""

    def __init__(self, model_path, providers=None, provider_options=None, input_size=518,
                 scale=1.0, shift=0.0):
        import cv2  # lazy: only when onnx is selected

        self._cv2 = cv2
        self._size = int(input_size)
        self._scale = float(scale)
        self._shift = float(shift)
        self._sess = make_session(model_path, providers, provider_options)
        self._in = self._sess.get_inputs()[0].name

    def estimate(self, bgr):
        cv2 = self._cv2
        pre = preprocess(bgr, self._size,
                         lambda im, s: cv2.resize(im, s, interpolation=cv2.INTER_CUBIC))
        raw = self._sess.run(None, {self._in: pre})[0]
        h, w = bgr.shape[:2]
        depth = postprocess(raw, (h, w),
                            lambda im, s: cv2.resize(im, s, interpolation=cv2.INTER_LINEAR))
        if self._scale != 1.0 or self._shift != 0.0:
            depth = (self._scale * depth + self._shift).astype(np.float32)
        return depth
