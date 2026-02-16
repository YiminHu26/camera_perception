from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
import torch
from pathlib import Path
from .inference_node_base import *
from .utils_camera import *
from .utils_node import *

class FrameSaver(AIRNode):
    '''
    A ROS2 node that subscribes to a depth image topic, converts the depth image to a point cloud,
    transforms the point cloud to the base_link frame, and saves it as a .pt file (numpy.ndarray ).
    '''

    def __init__(self):
        super().__init__("frame_saver")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info("Waiting for one depth frame...")

    def depth_callback(self, msg):
        self.get_logger().info("Depth frame received")

        # depth_image = self.bridge.imgmsg_to_cv2(msg)
        depth_image = self.last_depth_msg

        camera = CameraInfo(
            width=self.image_width,
            height=self.image_height,
            fx=self.camera_matrix[0,0],
            fy=self.camera_matrix[1,1],
            cx=self.camera_matrix[0,2],
            cy=self.camera_matrix[1,2],
            scale=1000.0
        )

        H, W = depth_image.shape[:2]
        pcd = create_point_cloud_from_depth_image(
            depth_image, camera, organized=False
        )

        self.get_logger().info(f"Shape of pcd: {pcd.shape}")

        t = self.tf_buffer.lookup_transform(
            "base_link",
            "orbbec_femto_mega_link",
            rclpy.time.Time()
        )

        # pcd_base = transform_points(pcd, t.transform).reshape(H, W, 3)
        pcd_base = transform_points(pcd, t.transform)
        self.get_logger().info(f"Shape of pcd_base: {pcd_base.shape}")

        timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        file_path = Path(f"vmf_input_{timestamp}.pt")

        tensor = torch.from_numpy(pcd_base).float()
        torch.save(tensor, file_path)

        self.get_logger().info(f"Saved {file_path}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    pcd_frame_saver = FrameSaver()
    try:
        rclpy.spin(pcd_frame_saver)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Exiting")

    pcd_frame_saver.destroy_node()

if __name__ == "__main__":
    main()