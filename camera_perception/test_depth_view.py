import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthViewer(Node):

    def __init__(self):
        super().__init__('depth_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Depth Viewer started")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Normalize for visualization (important!)
        depth_normalized = cv2.normalize(
            depth_image,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        depth_normalized = depth_normalized.astype(np.uint8)

        # Optional: apply color map (much easier to see)
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1)  # VERY IMPORTANT


def main(args=None):
    rclpy.init(args=args)
    node = DepthViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()