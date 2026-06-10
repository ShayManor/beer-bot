import os
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import message_filters
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
from autonomous_rover.nodes.localization.depth import (
    StubDepthEstimator, OnnxDepthEstimator, parse_qnn_options,
)
from autonomous_rover.nodes.localization.projection import backproject, valid_points
from autonomous_rover.nodes.localization.ground_plane import ground_scale
from autonomous_rover.nodes.localization import overlay

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None
try:
    import cv2
except ImportError:
    cv2 = None


class LocalizationNode(Node):
    """Monocular front-end: stub metric depth, ground-plane scale, fake RGB-D for
    RTAB-Map, RTAB-Map odom -> /pose, and an optional height-overlay debug JPEG."""

    def __init__(self):
        super().__init__(
            "localization_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()

        self.camera_height = float(self._param("camera_height_m", 0.1524))
        self.pitch = float(self._param("camera_pitch_rad", 0.0))
        self.pose_frame_id = str(self._param("pose_frame_id", "map"))
        self.debug_overlay = bool(self._param("debug_overlay", True))
        self.overlay_max_in = float(self._param("overlay_max_height_in", 48.0))
        self.ransac = dict(
            threshold=float(self._param("ransac_threshold", 0.02)),
            iterations=int(self._param("ransac_iterations", 200)),
            min_inliers=int(self._param("min_inliers", 50)),
        )

        rgb_topic = str(self._param("rgb_topic", "/camera/image_raw"))
        info_topic = str(self._param("camera_info_topic", "/camera/camera_info"))
        depth_topic = str(self._param("depth_topic", "/camera/depth"))
        odom_topic = str(self._param("odom_topic", "/odom"))

        self.depth_estimator = str(self._param("depth_estimator", "stub"))
        self.model_path = self._resolve_model_path(str(self._param("depth_model_path", "")))
        self.onnx_providers = list(self._param("onnx_providers", ["CPUExecutionProvider"]))
        self.depth_input_size = int(self._param("depth_input_size", 518))
        self.qnn_options = parse_qnn_options(list(self._param("qnn_options", [])))

        self._bridge = CvBridge() if CvBridge else None
        self._estimator = self._build_estimator(None) if self.depth_estimator == "onnx" else None
        self._K = None

        self._depth_pub = self.create_publisher(Image, depth_topic, 10)
        self._pose_pub = self.create_publisher(PoseStamped, "/pose", 10)
        self._debug_pub = self.create_publisher(
            CompressedImage, "/localization/debug_image/compressed", 1
        )

        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, info_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._on_frame)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self.logger.info("Initialized Localization Node")

    def _param(self, name, default):
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    def _resolve_model_path(self, path):
        if not path or os.path.isabs(path):
            return path
        try:  # resolve against the installed package share, not the launch cwd
            share = get_package_share_directory("autonomous_rover")
            candidate = os.path.join(share, path)
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
        return path

    def _build_estimator(self, K):
        if self.depth_estimator == "stub":
            return StubDepthEstimator(K, self.camera_height, self.pitch)
        if self.depth_estimator == "onnx":
            if not self.model_path:
                raise ValueError("depth_estimator is 'onnx' but depth_model_path is not set")
            popts = [self.qnn_options if p == "QNNExecutionProvider" else {}
                     for p in self.onnx_providers]
            return OnnxDepthEstimator(self.model_path, self.onnx_providers, popts,
                                      self.depth_input_size)
        raise ValueError(f"unknown depth_estimator {self.depth_estimator!r}")

    def _on_odom(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._pose_pub.publish(ps)

    def _on_frame(self, img_msg, info_msg):
        if self._bridge is None:
            return
        K = np.array(info_msg.k, dtype=float).reshape(3, 3)
        rgb = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        if self._estimator is None or (self.depth_estimator == "stub"
                                       and not np.array_equal(K, self._K)):
            self._K = K
            self._estimator = self._build_estimator(K)

        depth = self._estimator.estimate(rgb)
        xyz = backproject(depth, K)
        fit = ground_scale(valid_points(xyz), self.camera_height, **self.ransac)
        scale = fit.scale if fit is not None else 1.0

        depth_msg = self._bridge.cv2_to_imgmsg((depth * scale).astype(np.float32), encoding="32FC1")
        depth_msg.header = img_msg.header
        self._depth_pub.publish(depth_msg)

        if self.debug_overlay and fit is not None and cv2 is not None:
            height_in = overlay.height_inches(xyz * scale, fit.normal, fit.offset * scale)
            bgr = overlay.render_overlay(rgb, height_in, self.overlay_max_in)
            ok, buf = cv2.imencode(".jpg", bgr)
            if ok:
                cmsg = CompressedImage()
                cmsg.header = img_msg.header
                cmsg.format = "jpeg"
                cmsg.data = buf.tobytes()
                self._debug_pub.publish(cmsg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
