import math
import struct
import threading
import traceback
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, CompressedImage

from flask import Flask, Response, jsonify, request

from autonomous_rover.nodes.master.web import INDEX_HTML

# PointField datatype -> struct format char.
_PF_FMT = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _read_xyz(cloud, max_points):
    """Pull (x, y, z) tuples out of a PointCloud2, capped at max_points."""
    fields = {f.name: f for f in cloud.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return [], False
    endian = ">" if cloud.is_bigendian else "<"
    fx, fy, fz = fields["x"], fields["y"], fields["z"]
    step = cloud.point_step
    data = cloud.data
    n = cloud.width * cloud.height
    pts = []
    truncated = False
    for i in range(n):
        if max_points and len(pts) >= max_points:
            truncated = True
            break
        base = i * step
        x = struct.unpack_from(endian + _PF_FMT[fx.datatype], data, base + fx.offset)[0]
        y = struct.unpack_from(endian + _PF_FMT[fy.datatype], data, base + fy.offset)[0]
        z = struct.unpack_from(endian + _PF_FMT[fz.datatype], data, base + fz.offset)[0]
        pts.append([x, y, z])
    return pts, truncated


class MasterNode(Node):
    """Operator-facing brain: a Flask API over the robot's state and telemetry.

    Publishes the desired state (idle/active) and the operator's 2D goal;
    subscribes to pose, plan, cmd_vel, and the 3D cloud map and exposes them
    over HTTP. Construction is side-effect free; main() starts the server.
    """

    def __init__(self):
        super().__init__(
            "master_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()

        self.host = str(self._param("host", "0.0.0.0"))
        self.port = int(self._param("port", 8080))
        self.goal_frame_id = str(self._param("goal_frame_id", "map"))
        self.cloud_max_points = int(self._param("cloud_max_points", 20000))
        self.teleop_linear = float(self._param("teleop_linear", 0.35))
        self.teleop_angular = float(self._param("teleop_angular", 1.0))
        log_size = int(self._param("log_buffer_size", 200))

        self._lock = threading.Lock()
        self._state = "idle"
        self._goal = None
        self._pose = None
        self._path = []
        self._cmd = {"v": None, "omega": None}
        self._cloud = None
        self._debug_image = None
        self._camera_image = None
        self._logs = deque(maxlen=log_size)

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(String, "/robot_state", latched)
        self._goal_pub = self.create_publisher(PointStamped, "/goal_point", 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(PoseStamped, "/pose", self._on_pose, 10)
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(PointCloud2, "/cloud_map", self._on_cloud, 10)
        self.create_subscription(
            CompressedImage, "/localization/debug_image/compressed", self._on_debug_image, 1
        )
        self.create_subscription(
            CompressedImage,
            self._param("camera_preview_topic", "/camera/preview/compressed"),
            self._on_camera_image,
            1,
        )

        self._publish_state()  # announce initial idle
        self.app = self._build_app()
        self._server_thread = None
        self.logger.info("Initialized Master Node")

    def _param(self, name, default):
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    # --- ROS subscriptions ------------------------------------------------
    def _on_pose(self, msg):
        p = msg.pose.position
        yaw = _yaw_from_quaternion(msg.pose.orientation)
        with self._lock:
            self._pose = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}

    def _on_path(self, msg):
        pts = [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]
        with self._lock:
            self._path = pts

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = {"v": msg.linear.x, "omega": msg.angular.z}

    def _on_cloud(self, msg):
        with self._lock:
            self._cloud = msg

    def _on_debug_image(self, msg):
        with self._lock:
            self._debug_image = bytes(msg.data)

    def _on_camera_image(self, msg):
        with self._lock:
            self._camera_image = bytes(msg.data)

    # --- commands ---------------------------------------------------------
    def _publish_state(self):
        m = String()
        m.data = self._state
        self._state_pub.publish(m)

    def _log_event(self, msg):
        stamp = self.get_clock().now().to_msg()
        t = stamp.sec + stamp.nanosec * 1e-9
        with self._lock:
            self._logs.append({"t": t, "msg": msg})
        self.logger.info(msg)

    def set_state(self, state):
        if state not in ("idle", "active"):
            raise ValueError("state must be 'idle' or 'active'")
        with self._lock:
            self._state = state
        self._publish_state()
        self._log_event(f"state -> {state}")

    def set_goal(self, x, y, frame_id=None):
        frame = frame_id or self.goal_frame_id
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self._goal_pub.publish(msg)
        goal = {"x": float(x), "y": float(y), "frame_id": frame}
        with self._lock:
            self._goal = goal
            self._state = "active"
        self._publish_state()
        self._log_event(f"goal -> ({float(x)}, {float(y)}) [{frame}], state -> active")
        return goal

    def teleop(self, v, omega):
        """Publish a manual velocity command (operator joystick / arrow keys)."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self._cmd_pub.publish(msg)

    # --- telemetry --------------------------------------------------------
    def snapshot(self):
        with self._lock:
            return {
                "state": self._state,
                "pose": self._pose,
                "heading": None if self._pose is None else self._pose["yaw"],
                "path": list(self._path),
                "speed": dict(self._cmd),
                "goal": self._goal,
            }

    def cloud_snapshot(self):
        with self._lock:
            cloud = self._cloud
        if cloud is None:
            return {"points": [], "count": 0, "truncated": False, "frame_id": None}
        pts, truncated = _read_xyz(cloud, self.cloud_max_points)
        return {
            "points": pts,
            "count": len(pts),
            "truncated": truncated,
            "frame_id": cloud.header.frame_id,
        }

    # --- HTTP -------------------------------------------------------------
    def _build_app(self):
        app = Flask("autonomous_rover_master")
        page = INDEX_HTML.replace("__TELEOP_V__", repr(self.teleop_linear)).replace(
            "__TELEOP_W__", repr(self.teleop_angular)
        )

        @app.route("/", methods=["GET"])
        def index():
            return Response(page, mimetype="text/html")

        @app.route("/health", methods=["GET"])
        def health():
            return jsonify({"ok": True})

        @app.route("/status", methods=["GET"])
        def status():
            return jsonify(self.snapshot())

        @app.route("/map", methods=["GET"])
        def get_map():
            return jsonify(self.cloud_snapshot())

        @app.route("/debug_image", methods=["GET"])
        def debug_image():
            with self._lock:
                data = self._debug_image
            if data is None:
                return jsonify({"error": "no debug image yet"}), 503
            return Response(data, mimetype="image/jpeg")

        @app.route("/camera_image", methods=["GET"])
        def camera_image():
            with self._lock:
                data = self._camera_image
            if data is None:
                return jsonify({"error": "no camera frame yet"}), 503
            return Response(data, mimetype="image/jpeg")

        @app.route("/logs", methods=["GET"])
        def get_logs():
            with self._lock:
                return jsonify({"logs": list(self._logs)})

        @app.route("/state", methods=["POST"])
        def post_state():
            body = request.get_json(silent=True) or {}
            try:
                self.set_state(body.get("state"))
            except ValueError as e:
                return jsonify({"error": str(e)}), 400
            return jsonify({"state": body.get("state")})

        @app.route("/goal", methods=["POST"])
        def post_goal():
            body = request.get_json(silent=True) or {}
            try:
                x = float(body["x"])
                y = float(body["y"])
            except (KeyError, TypeError, ValueError):
                return jsonify({"error": "body must include numeric 'x' and 'y'"}), 400
            goal = self.set_goal(x, y, body.get("frame_id"))
            return jsonify({"accepted": goal, "state": "active"})

        @app.route("/teleop", methods=["POST"])
        def post_teleop():
            body = request.get_json(silent=True) or {}
            try:
                v = float(body.get("v", 0.0))
                omega = float(body.get("omega", 0.0))
            except (TypeError, ValueError):
                return jsonify({"error": "v and omega must be numeric"}), 400
            self.teleop(v, omega)
            return jsonify({"v": v, "omega": omega})

        return app

    def start_http(self):
        if self._server_thread is not None:
            return
        self._server_thread = threading.Thread(target=self._serve, daemon=True)
        self._server_thread.start()

    def _serve(self):
        try:
            self.app.run(
                host=self.host, port=self.port, threaded=True, use_reloader=False
            )
        except Exception:
            self.logger.error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)

    node = MasterNode()
    node.start_http()

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
