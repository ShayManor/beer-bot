import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from autonomous_rover.nodes.camera.calibration import load_camera_info

try:
    from cv_bridge import CvBridge
except ImportError:  # allow import/construction without cv_bridge
    CvBridge = None
try:
    import cv2
except ImportError:
    cv2 = None


class CameraNode(Node):
    """Publishes RGB frames + calibrated CameraInfo from a webcam, video, or a
    synthetic generator (for dev)."""

    def __init__(self):
        super().__init__(
            "camera_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()

        self.width = int(self._param("width", 640))
        self.height = int(self._param("height", 480))
        self.fps = float(self._param("fps", 30.0))
        self.frame_id = str(self._param("frame_id", "camera_optical_frame"))
        self.source = str(self._param("source", "synthetic"))
        self.device_index = int(self._param("device_index", 0))
        self.device_path = str(self._param("device_path", ""))
        self.video_path = str(self._param("video_path", ""))
        fov_deg = float(self._param("fov_deg", 70.0))
        calib_file = str(self._param("calibration_file", ""))

        self.K, self.D = load_camera_info(calib_file, self.width, self.height, fov_deg)
        self._bridge = CvBridge() if CvBridge else None

        self._img_pub = self.create_publisher(Image, self._param("rgb_topic", "/camera/image_raw"), 10)
        self._info_pub = self.create_publisher(CameraInfo, self._param("camera_info_topic", "/camera/camera_info"), 10)

        self._cap = self._open_source()
        if self.fps > 0:
            self.create_timer(1.0 / self.fps, self._tick)
        self.logger.info("Initialized Camera Node")

    def _param(self, name, default):
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    def _open_source(self):
        if self.source in ("webcam", "video") and cv2 is not None:
            if self.source == "webcam":
                target = self.device_path or self.device_index
            else:
                target = self.video_path
            cap = cv2.VideoCapture(target)
            if self.source == "webcam":
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not cap.isOpened():
                self.logger.warning(f"Could not open source '{target}'; using synthetic frames")
                return None
            return cap
        return None

    def _read(self):
        if self._cap is not None:
            ok, frame = self._cap.read()
            return frame if ok else None
        # Synthetic moving gradient so something always publishes.
        t = self.get_clock().now().nanoseconds
        col = np.linspace(0, 255, self.width, dtype=np.uint8)
        frame = np.tile(col, (self.height, 1))
        frame = np.roll(frame, int(t // 10_000_000) % self.width, axis=1)
        return np.dstack([frame, frame, frame])

    def _make_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.k = [float(x) for x in self.K.flatten()]
        info.d = [float(x) for x in np.asarray(self.D).flatten()]
        info.distortion_model = "plumb_bob"
        return info

    def _tick(self):
        frame = self._read()
        if frame is None or self._bridge is None:
            # Still publish CameraInfo so downstream sync has intrinsics.
            self._info_pub.publish(self._make_info(self.get_clock().now().to_msg()))
            return
        stamp = self.get_clock().now().to_msg()
        img = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        self._img_pub.publish(img)
        self._info_pub.publish(self._make_info(stamp))


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
