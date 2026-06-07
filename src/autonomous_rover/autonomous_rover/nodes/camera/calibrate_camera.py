"""Offline chessboard camera calibration.

Run on the Rubik with a printed checkerboard. Captures views until enough corner
sets are collected, then writes K + distortion to a calibration YAML that
camera_node loads.

Usage:
  python3 -m autonomous_rover.nodes.camera.calibrate_camera \\
      --output src/autonomous_rover/autonomous_rover/params/camera_calib.yaml \\
      --rows 6 --cols 9 --square 0.025 --device 0 --views 15
"""
import argparse

import numpy as np

from autonomous_rover.nodes.camera.calibration import save_calibration


def calibrate(object_points, image_points, image_size):
    """Wrap cv2.calibrateCamera. Returns (K, D, rms)."""
    import cv2

    rms, K, D, _, _ = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    return K, D.flatten(), rms


def _board_points(rows, cols, square):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * square


def main():
    import cv2

    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--square", type=float, default=0.025)
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--views", type=int, default=15)
    args = ap.parse_args()

    objp = _board_points(args.rows, args.cols, args.square)
    obj_points, img_points = [], []
    cap = cv2.VideoCapture(args.device)
    size = None
    print(f"Show the {args.cols}x{args.rows} board; capturing {args.views} views. 'q' to abort.")
    while len(img_points) < args.views:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            obj_points.append(objp)
            img_points.append(corners)
            cv2.drawChessboardCorners(frame, (args.cols, args.rows), corners, found)
            print(f"captured {len(img_points)}/{args.views}")
        cv2.imshow("calib", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()

    if len(img_points) < 3:
        print("Not enough views; aborting.")
        return
    K, D, rms = calibrate(obj_points, img_points, size)
    save_calibration(args.output, K, D, size[0], size[1])
    print(f"Wrote {args.output} (reprojection RMS = {rms:.3f} px)")


if __name__ == "__main__":
    main()
