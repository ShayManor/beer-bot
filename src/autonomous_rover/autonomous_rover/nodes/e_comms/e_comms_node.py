"""
e_comms_node — bridge between ROS2 and the WAVE ROVER ESP32 driver board.

Subscribes to a Twist (v, omega), mixes it to skid-steer left/right wheel
commands via a calibrated feedforward map, and ships JSON over UART. A reader
thread parses the board's feedback JSON and publishes the onboard IMU.

Protocol (UART @ 115200, newline-terminated JSON; ugv_base_general firmware):
  drive    {"T":1,"L":<left>,"R":<right>}      e-stop {"T":0}
  poll IMU {"T":126}  -> reply {"T":1002, r,p,y, ax,ay,az, gx,gy,gz, mx,my,mz, temp}
IMU units from firmware: r/p/y deg, accel mg, gyro dps, mag raw (0.15 uT/LSB).
"""
import json
import math
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField

try:
    import serial
except ImportError:  # allow the node/tests to import without pyserial installed
    serial = None

# Firmware command / feedback type ids.
CMD_SPEED_CTRL = 1
CMD_EMERGENCY_STOP = 0
CMD_GET_IMU_DATA = 126
FEEDBACK_IMU_DATA = 1002
FEEDBACK_BASE_INFO = 1001

# Unit conversions to ROS SI (REP-145).
MG_TO_MS2 = 9.80665e-3
DPS_TO_RADS = math.pi / 180.0
DEG_TO_RAD = math.pi / 180.0
MAG_RAW_TO_TESLA = 0.15e-6


class ECommsNode(Node):
    def __init__(self):
        super().__init__(
            "e_comms_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()

        self.serial_port = self._param("serial_port", "/dev/ttyUSB0")
        self.baud_rate = int(self._param("baud_rate", 115200))
        self.frame_id = self._param("imu_frame_id", "imu_link")
        # Skid-steer feedforward: left/right duty = v*lin +/- omega*ang, scaled
        # down together if either exceeds the clamp. Calibrate gains in phase 1.
        self.wheel_cmd_per_mps = float(self._param("wheel_cmd_per_mps", 0.4))
        self.wheel_cmd_per_radps = float(self._param("wheel_cmd_per_radps", 0.2))
        self.max_wheel_cmd = float(self._param("max_wheel_cmd", 0.49))
        # Open-loop deadband: lift any nonzero wheel command to this floor so the
        # inner wheel clears motor breakaway in a turn instead of stalling.
        self.min_wheel_cmd = float(self._param("min_wheel_cmd", 0.18))
        self.imu_rate = float(self._param("imu_rate", 50.0))
        self.cmd_timeout = float(self._param("cmd_timeout", 0.5))

        self.imu_pub = self.create_publisher(Imu, self._param("imu_topic", "imu/data"), 10)
        self.mag_pub = self.create_publisher(MagneticField, self._param("mag_topic", "imu/mag"), 10)
        self.create_subscription(Twist, self._param("cmd_vel_topic", "cmd_vel"), self._on_cmd_vel, 10)

        self._ser = None
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._last_cmd_time = 0.0
        self._open_serial()

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        if self.imu_rate > 0:
            self.create_timer(1.0 / self.imu_rate, lambda: self._send({"T": CMD_GET_IMU_DATA}))
        self.create_timer(0.1, self._watchdog)

        self.logger.info("Initialized EComms Node")

    def _param(self, name, default):
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    # -- serial -----------------------------------------------------------
    def _open_serial(self):
        if serial is None:
            self.logger.warning("pyserial not installed; running without hardware link")
            return
        try:
            self._ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.logger.info(f"Opened {self.serial_port} @ {self.baud_rate}")
        except Exception as e:
            self._ser = None
            self.logger.warning(f"Could not open {self.serial_port}: {e}; running without hardware link")

    def _send(self, payload):
        if self._ser is None:
            return
        line = (json.dumps(payload) + "\n").encode("utf-8")
        try:
            with self._write_lock:
                self._ser.write(line)
        except Exception as e:
            self.logger.error(f"Serial write failed: {e}")

    # -- drive ------------------------------------------------------------
    @staticmethod
    def mix(v, omega, lin_scale, ang_scale, max_cmd):
        """(v, omega) -> (left, right) board duty. lin_scale maps m/s, ang_scale
        maps rad/s to a per-wheel differential. If either wheel exceeds max_cmd,
        scale both down together so the turn survives instead of clipping flat."""
        left = v * lin_scale - omega * ang_scale
        right = v * lin_scale + omega * ang_scale
        peak = max(abs(left), abs(right))
        if peak > max_cmd:
            left *= max_cmd / peak
            right *= max_cmd / peak
        return left, right

    @staticmethod
    def apply_deadband(cmd, min_cmd):
        """Lift a nonzero wheel command up to min_cmd so the motor clears its
        open-loop breakaway instead of stalling; 0 stays 0, sign preserved."""
        if min_cmd > 0.0 and 0.0 < abs(cmd) < min_cmd:
            return math.copysign(min_cmd, cmd)
        return cmd

    def _on_cmd_vel(self, msg):
        self._last_cmd_time = time.monotonic()
        left, right = self.mix(
            msg.linear.x * -1, msg.angular.z, self.wheel_cmd_per_mps, self.wheel_cmd_per_radps, self.max_wheel_cmd
        )
        left = self.apply_deadband(left, self.min_wheel_cmd)
        right = self.apply_deadband(right, self.min_wheel_cmd)
        self._send({"T": CMD_SPEED_CTRL, "L": left, "R": right})

    def _watchdog(self):
        """Stop the wheels if no command has arrived within cmd_timeout."""
        if self._last_cmd_time and time.monotonic() - self._last_cmd_time > self.cmd_timeout:
            self._send({"T": CMD_SPEED_CTRL, "L": 0, "R": 0})
            self._last_cmd_time = 0.0

    # -- feedback ---------------------------------------------------------
    def _read_loop(self):
        while not self._stop.is_set():
            if self._ser is None:
                time.sleep(0.2)
                continue
            try:
                raw = self._ser.readline()
            except Exception as e:
                self.logger.error(f"Serial read failed: {e}")
                time.sleep(0.2)
                continue
            if not raw:
                continue
            try:
                data = json.loads(raw.decode("utf-8").strip())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(data, dict) and data.get("T") in (FEEDBACK_IMU_DATA, FEEDBACK_BASE_INFO):
                self._publish_imu(data)

    def _publish_imu(self, data):
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation = self._quat_from_rpy(
            data.get("r", 0.0) * DEG_TO_RAD,
            data.get("p", 0.0) * DEG_TO_RAD,
            data.get("y", 0.0) * DEG_TO_RAD,
        )
        imu.angular_velocity.x = data.get("gx", 0.0) * DPS_TO_RADS
        imu.angular_velocity.y = data.get("gy", 0.0) * DPS_TO_RADS
        imu.angular_velocity.z = data.get("gz", 0.0) * DPS_TO_RADS
        imu.linear_acceleration.x = data.get("ax", 0.0) * MG_TO_MS2
        imu.linear_acceleration.y = data.get("ay", 0.0) * MG_TO_MS2
        imu.linear_acceleration.z = data.get("az", 0.0) * MG_TO_MS2
        # Base-info frames carry orientation only; flag missing rates/accels.
        if "gx" not in data:
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        if "mx" in data:
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self.frame_id
            mag.magnetic_field.x = data["mx"] * MAG_RAW_TO_TESLA
            mag.magnetic_field.y = data["my"] * MAG_RAW_TO_TESLA
            mag.magnetic_field.z = data["mz"] * MAG_RAW_TO_TESLA
            self.mag_pub.publish(mag)

    @staticmethod
    def _quat_from_rpy(roll, pitch, yaw):
        from geometry_msgs.msg import Quaternion

        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def destroy_node(self):
        self._stop.set()
        self._send({"T": CMD_EMERGENCY_STOP})
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ECommsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
