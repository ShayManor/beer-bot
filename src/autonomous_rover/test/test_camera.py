import numpy as np
import pytest


def test_default_K_focal_from_fov():
    from autonomous_rover.nodes.camera.calibration import default_K

    K, D = default_K(width=640, height=480, fov_deg=90.0)
    # fov 90 -> f = (w/2) / tan(45) = 320
    assert K[0, 0] == pytest.approx(320.0, rel=1e-6)
    assert K[0, 2] == pytest.approx(320.0)
    assert K[1, 2] == pytest.approx(240.0)
    assert D.shape == (5,)


def test_calibration_save_load_roundtrip(tmp_path):
    from autonomous_rover.nodes.camera.calibration import save_calibration, load_camera_info

    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.array([0.1, -0.2, 0.0, 0.0, 0.05])
    path = tmp_path / "calib.yaml"
    save_calibration(str(path), K, D, 640, 480)

    K2, D2 = load_camera_info(str(path), width=640, height=480, fov_deg=70.0)
    assert np.allclose(K, K2)
    assert np.allclose(D, D2)


def test_load_camera_info_falls_back_to_default(tmp_path):
    from autonomous_rover.nodes.camera.calibration import load_camera_info, default_K

    K, D = load_camera_info(str(tmp_path / "missing.yaml"), width=640, height=480, fov_deg=90.0)
    Kd, _ = default_K(640, 480, 90.0)
    assert np.allclose(K, Kd)


def test_camera_node_constructs_and_publishes_info(ros_ctx, spin_helper):
    rclpy = pytest.importorskip("rclpy")
    from sensor_msgs.msg import CameraInfo
    from autonomous_rover.nodes.camera.camera_node import CameraNode

    with ros_ctx({"source": "synthetic",
                  "width": 64, "height": 48,
                  "fps": 30.0}):
        node = CameraNode()
        received = []
        node.create_subscription(CameraInfo, "/camera/camera_info", received.append, 10)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        assert spin_helper(ex, lambda: len(received) > 0, timeout=3.0)
        assert received[0].width == 64 and received[0].height == 48
        node.destroy_node()


def test_camera_node_publishes_preview(ros_ctx, spin_helper):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("cv2")
    pytest.importorskip("cv_bridge")
    from sensor_msgs.msg import CompressedImage
    from autonomous_rover.nodes.camera.camera_node import CameraNode

    with ros_ctx({"source": "synthetic", "width": 64, "height": 48,
                  "fps": 30.0, "preview_every_n": 1}):
        node = CameraNode()
        received = []
        node.create_subscription(CompressedImage, "/camera/preview/compressed", received.append, 1)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        assert spin_helper(ex, lambda: len(received) > 0, timeout=3.0)
        assert received[0].format == "jpeg"
        assert len(received[0].data) > 0
        node.destroy_node()
