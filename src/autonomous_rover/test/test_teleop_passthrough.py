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
