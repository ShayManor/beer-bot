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
