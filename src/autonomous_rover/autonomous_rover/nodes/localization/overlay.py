"""Height-above-floor visualization for on-device ruler validation."""
import numpy as np

_M_TO_IN = 39.3701


def height_inches(xyz, normal, offset):
    """Per-pixel signed height above the plane (normal . x + offset = 0), in inches.

    `normal` and `offset` must be the consistent pair from the plane fit (see
    PlaneFit.normal / PlaneFit.offset). The pair is flipped so the camera origin
    is on the positive side, giving floor == 0 and "up" positive. `xyz` is
    (H,W,3); NaN pixels stay NaN.
    """
    normal = np.asarray(normal, dtype=np.float64)
    d = float(offset)
    if d < 0.0:  # camera origin height is `d`; flip the pair so it reads positive
        normal = -normal
        d = -d
    return (xyz @ normal + d) * _M_TO_IN


def render_overlay(rgb, height_in, max_height_in=48.0):
    """Colorize height (inches) over the RGB frame with a center-crosshair readout.

    Returns a BGR uint8 image. Requires OpenCV.
    """
    import cv2

    h, w = height_in.shape
    valid = np.isfinite(height_in)
    norm = np.clip(height_in, 0.0, max_height_in) / max_height_in
    norm8 = np.where(valid, (norm * 255.0), 0).astype(np.uint8)
    color = cv2.applyColorMap(norm8, cv2.COLORMAP_JET)
    color[~valid] = 0
    out = cv2.addWeighted(rgb, 0.5, color, 0.5, 0.0)

    cx, cy = w // 2, h // 2
    cv2.drawMarker(out, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
    center = height_in[cy, cx]
    label = "n/a" if not np.isfinite(center) else f"{center:.1f} in"
    cv2.putText(out, label, (cx + 12, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 2, cv2.LINE_AA)
    return out
