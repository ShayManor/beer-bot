import math

import pytest


def test_mix_straight_and_turn():
    from beer_bot.nodes.e_comms.e_comms_node import ECommsNode

    # Straight ahead: both wheels equal.
    left, right = ECommsNode.mix(0.4, 0.0, lin_scale=1.0, ang_scale=0.1, max_cmd=0.5)
    assert left == pytest.approx(0.4)
    assert right == pytest.approx(0.4)

    # Spin in place: wheels opposite, symmetric.
    left, right = ECommsNode.mix(0.0, 1.0, lin_scale=1.0, ang_scale=0.1, max_cmd=0.5)
    assert left == pytest.approx(-0.1)
    assert right == pytest.approx(0.1)


def test_mix_clamps_to_max():
    from beer_bot.nodes.e_comms.e_comms_node import ECommsNode

    left, right = ECommsNode.mix(10.0, 0.0, lin_scale=1.0, ang_scale=0.1, max_cmd=0.5)
    assert left == 0.5 and right == 0.5
    left, right = ECommsNode.mix(-10.0, 0.0, lin_scale=1.0, ang_scale=0.1, max_cmd=0.5)
    assert left == -0.5 and right == -0.5


def test_mix_preserves_turn_when_saturated():
    from beer_bot.nodes.e_comms.e_comms_node import ECommsNode

    # Full forward + turn: the outer wheel would exceed max, but both scale down
    # together so a real differential remains instead of clipping flat (no turn).
    left, right = ECommsNode.mix(1.25, 1.0, lin_scale=0.4, ang_scale=0.2, max_cmd=0.49)
    assert right == pytest.approx(0.49)      # outer wheel pinned at the limit
    assert abs(right - left) > 0.1           # turn differential survives


def test_publishes_imu_in_si_units(ros_ctx, spin_helper):
    rclpy = pytest.importorskip("rclpy")
    from sensor_msgs.msg import Imu
    from beer_bot.nodes.e_comms.e_comms_node import ECommsNode, MG_TO_MS2, DPS_TO_RADS

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        node = ECommsNode()
        received = []
        sub = node.create_subscription(Imu, "imu/data", received.append, 10)
        node._publish_imu({"T": 1002, "r": 0.0, "p": 0.0, "y": 0.0,
                           "ax": 0.0, "ay": 0.0, "az": 1000.0,
                           "gx": 90.0, "gy": 0.0, "gz": 0.0,
                           "mx": 0, "my": 0, "mz": 0})
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        assert spin_helper(executor, lambda: len(received) > 0)
        msg = received[0]
        assert msg.linear_acceleration.z == pytest.approx(1000.0 * MG_TO_MS2)  # ~9.8 m/s^2
        assert msg.angular_velocity.x == pytest.approx(90.0 * DPS_TO_RADS)     # ~1.57 rad/s
        assert msg.header.frame_id == "imu_link"
        node.destroy_node()
