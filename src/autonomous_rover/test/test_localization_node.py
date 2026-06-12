import pytest


def test_localization_odom_republished_as_pose(ros_ctx, spin_helper):
    rclpy = pytest.importorskip("rclpy")
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from autonomous_rover.nodes.localization.localization_node import LocalizationNode

    with ros_ctx():
        node = LocalizationNode()
        got = []
        node.create_subscription(PoseStamped, "/pose", got.append, 10)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = 1.25
        node._on_odom(odom)

        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        assert spin_helper(ex, lambda: len(got) > 0, timeout=2.0)
        assert got[0].pose.position.x == pytest.approx(1.25)
        node.destroy_node()


def test_onnx_estimator_unset_model_path_raises_valueerror(ros_ctx):
    pytest.importorskip("rclpy")
    from autonomous_rover.nodes.localization.localization_node import LocalizationNode

    with ros_ctx({"depth_estimator": "onnx"}):
        with pytest.raises(ValueError):
            LocalizationNode()


def test_onnx_estimator_missing_model_hard_fails(ros_ctx):
    pytest.importorskip("rclpy")
    from autonomous_rover.nodes.localization.localization_node import LocalizationNode

    params = {
        "depth_estimator": "onnx",
        "depth_model_path": "/nonexistent/model.onnx",
    }
    with ros_ctx(params):
        with pytest.raises(FileNotFoundError):
            LocalizationNode()


def test_loads_depth_affine(ros_ctx, tmp_path):
    import yaml
    from autonomous_rover.nodes.localization.localization_node import LocalizationNode

    p = tmp_path / "depth_affine.yaml"
    p.write_text(yaml.safe_dump({"depth_scale": 1.7, "depth_shift": -0.2}))
    with ros_ctx({"depth_affine_file": str(p)}):
        node = LocalizationNode()
        assert abs(node.depth_scale - 1.7) < 1e-9
        assert abs(node.depth_shift + 0.2) < 1e-9
        node.destroy_node()
