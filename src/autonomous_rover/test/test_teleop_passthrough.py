"""
End-to-end: frontend /teleop -> master /cmd_vel -> e_comms JSON over UART.

Runs MasterNode and ECommsNode in one ROS context, posts to the same HTTP
endpoint the browser arrow keys hit, and asserts the wheel JSON the ESP32 would
receive. A fake serial captures what e_comms writes so no hardware is needed.
"""
import json
import threading
import time

import pytest


class _FakeSerial:
    """Stand-in for pyserial: records writes, yields no feedback."""

    def __init__(self):
        self.writes = []
        self._lock = threading.Lock()

    def write(self, data):
        with self._lock:
            self.writes.append(data)
        return len(data)

    def readline(self):
        time.sleep(0.05)
        return b""

    def close(self):
        pass

    def drive_frames(self):
        frames = []
        with self._lock:
            raw = list(self.writes)
        for w in raw:
            try:
                obj = json.loads(w.decode("utf-8").strip())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(obj, dict) and obj.get("T") == 1:
                frames.append(obj)
        return frames


def _build_chain(rclpy):
    """Master + e_comms on one executor, e_comms writing to a fake serial.

    This is the whole teleop spine the browser drives: HTTP /teleop -> master
    publishes /cmd_vel -> e_comms mixes to wheels and writes the ESP32 frame.
    Returns (master, ecomms, fake, executor, client).
    """
    from autonomous_rover.nodes.master.master_node import MasterNode
    from autonomous_rover.nodes.e_comms.e_comms_node import ECommsNode

    master = MasterNode()
    ecomms = ECommsNode()
    fake = _FakeSerial()
    ecomms._ser = fake  # capture what would go to the ESP32

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(master)
    executor.add_node(ecomms)
    return master, ecomms, fake, executor, master.app.test_client()


def _expected_wheels(ecomms, v, omega):
    """The (L, R) the board should receive for a teleop (v, omega), derived
    from the node's own calibrated map so the assertion tracks config changes
    while still pinning the inversion sign (the literal -v) independently."""
    from autonomous_rover.nodes.e_comms.e_comms_node import ECommsNode

    left, right = ECommsNode.mix(
        -v, omega, ecomms.wheel_cmd_per_mps, ecomms.wheel_cmd_per_radps, ecomms.max_wheel_cmd
    )
    return (
        ECommsNode.apply_deadband(left, ecomms.min_wheel_cmd),
        ECommsNode.apply_deadband(right, ecomms.min_wheel_cmd),
    )


def _drive(client, fake, executor, spin_helper, v, omega):
    """POST a teleop command and return the wheel frame it produced."""
    before = len(fake.drive_frames())
    assert client.post("/teleop", json={"v": v, "omega": omega}).status_code == 200
    assert spin_helper(executor, lambda: len(fake.drive_frames()) > before)
    return fake.drive_frames()[-1]


def test_teleop_passes_through_to_wheel_json(ros_ctx, spin_helper):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")
    from autonomous_rover.nodes.master.master_node import MasterNode
    from autonomous_rover.nodes.e_comms.e_comms_node import ECommsNode

    # imu_rate 0 keeps the serial line quiet except for drive frames.
    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master = MasterNode()
        ecomms = ECommsNode()
        fake = _FakeSerial()
        ecomms._ser = fake  # capture what would go to the ESP32

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(master)
        executor.add_node(ecomms)

        client = master.app.test_client()

        # Arrow Up: forward. v=0.6 (kept above the deadband floor so scaling is
        # tested cleanly), omega=0 -> both wheels -0.6*wheel_cmd_per_mps
        # (board duty is inverted from linear.x; positive duty drives backward).
        assert client.post("/teleop", json={"v": 0.6, "omega": 0.0}).status_code == 200
        assert spin_helper(executor, lambda: any(
            f["L"] != 0.0 for f in fake.drive_frames()
        ))
        fwd = next(f for f in fake.drive_frames() if f["L"] != 0.0)
        expected = -0.6 * ecomms.wheel_cmd_per_mps
        assert fwd["L"] == pytest.approx(expected)
        assert fwd["R"] == pytest.approx(expected)

        # Arrow Left: turn in place. omega>0 -> left wheel slower than right.
        assert client.post("/teleop", json={"v": 0.0, "omega": 1.0}).status_code == 200
        assert spin_helper(executor, lambda: any(
            f["L"] < 0.0 < f["R"] for f in fake.drive_frames()
        ))
        turn = next(f for f in fake.drive_frames() if f["L"] < 0.0 < f["R"])
        assert turn["L"] == pytest.approx(-turn["R"])

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_reverse_inverts_sign(ros_ctx, spin_helper):
    """Arrow Down: v<0. Board duty is inverted from linear.x, so reverse drives
    both wheels to the opposite sign of forward (positive duty)."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        frame = _drive(client, fake, executor, spin_helper, v=-0.6, omega=0.0)
        expected = 0.6 * ecomms.wheel_cmd_per_mps  # positive duty == backward
        assert frame["L"] == pytest.approx(expected)
        assert frame["R"] == pytest.approx(expected)
        assert frame["L"] > 0.0 and frame["R"] > 0.0  # opposite of forward

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_turn_right(ros_ctx, spin_helper):
    """Arrow Right: omega<0 -> right wheel slower than left, mirror of left."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        frame = _drive(client, fake, executor, spin_helper, v=0.0, omega=-1.0)
        assert frame["R"] < 0.0 < frame["L"]              # mirror of turn-left
        assert frame["R"] == pytest.approx(-frame["L"])
        assert (frame["L"], frame["R"]) == pytest.approx(_expected_wheels(ecomms, 0.0, -1.0))

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_arc_keeps_both_wheels_driving(ros_ctx, spin_helper):
    """Up+Left held together (the JS 'combine to arc' path): forward with a turn
    bias -> both wheels drive the same way but at different speeds."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        # omega small enough that the inner wheel clears the deadband cleanly.
        frame = _drive(client, fake, executor, spin_helper, v=0.6, omega=0.25)
        assert frame["L"] < 0.0 and frame["R"] < 0.0      # both still going forward
        assert abs(frame["L"]) > abs(frame["R"])          # outer wheel faster -> it arcs
        assert frame["L"] != pytest.approx(frame["R"])    # a real differential, not straight
        assert (frame["L"], frame["R"]) == pytest.approx(_expected_wheels(ecomms, 0.6, 0.25))

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_stop_zeroes_wheels(ros_ctx, spin_helper):
    """Release all keys / STOP button: v=omega=0 -> an explicit zero wheel frame."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        frame = _drive(client, fake, executor, spin_helper, v=0.0, omega=0.0)
        assert frame["L"] == 0.0 and frame["R"] == 0.0

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_deadband_lifts_inner_wheel(ros_ctx, spin_helper):
    """A forward+turn where the inner wheel command would stall: the integrated
    path lifts it to the breakaway floor (min_wheel_cmd), sign preserved."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        frame = _drive(client, fake, executor, spin_helper, v=0.6, omega=1.0)
        # Raw inner wheel would be ~-0.04 (stalls); lifted to -min_wheel_cmd.
        assert frame["R"] == pytest.approx(-ecomms.min_wheel_cmd)
        assert frame["L"] < -ecomms.min_wheel_cmd          # outer wheel untouched
        assert (frame["L"], frame["R"]) == pytest.approx(_expected_wheels(ecomms, 0.6, 1.0))

        master.destroy_node()
        ecomms.destroy_node()


def test_idle_gate_blocks_teleop(ros_ctx, spin_helper):
    """idle state forces no motion: even an explicit teleop command lands as a
    stop frame at the wheels, never a drive frame. The gate lives in e_comms so
    it catches operator teleop too, not just the planner's cmd_vel."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        # Switch to idle and let the latched state reach e_comms.
        assert client.post("/state", json={"state": "idle"}).status_code == 200
        assert spin_helper(executor, lambda: ecomms._active is False)

        # A real teleop command must not move the rover.
        before = len(fake.drive_frames())
        assert client.post("/teleop", json={"v": 0.6, "omega": 0.0}).status_code == 200
        assert spin_helper(executor, lambda: len(fake.drive_frames()) > before)
        assert all(f["L"] == 0 and f["R"] == 0 for f in fake.drive_frames())

        master.destroy_node()
        ecomms.destroy_node()


def test_teleop_watchdog_stops_after_timeout(ros_ctx, spin_helper):
    """A held key that stops arriving must not strand the rover: after cmd_timeout
    with no new /cmd_vel, e_comms' watchdog emits a zero wheel frame on its own."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("flask")

    with ros_ctx({"e_comms_node.imu_rate": 0.0, "e_comms_node.cmd_timeout": 0.2}):
        master, ecomms, fake, executor, client = _build_chain(rclpy)

        moving = _drive(client, fake, executor, spin_helper, v=0.6, omega=0.0)
        assert moving["L"] != 0.0  # rover is driving

        # Stop posting; the watchdog should zero the wheels without any new command.
        assert spin_helper(
            executor,
            lambda: any(f["L"] == 0 and f["R"] == 0 for f in fake.drive_frames()),
            timeout=2.0,
        )

        master.destroy_node()
        ecomms.destroy_node()
