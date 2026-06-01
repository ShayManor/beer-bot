import traceback

import rclpy
from rclpy.node import Node


class CameraNode(Node):
    def __init__(self):
        super().__init__(
            "camera_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.logger = self.get_logger()
        self.logger.info("Initialized Camera Node")


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
