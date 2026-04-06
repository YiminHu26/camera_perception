#!/usr/bin/env python3
import numpy as np
import torch
import open3d as o3d

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as R


def pose_to_matrix(pose_msg):
    """geometry_msgs/Pose -> 4x4 transform matrix"""
    t = np.array([
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z
    ], dtype=float)

    q = np.array([
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w
    ], dtype=float)

    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def matrix_to_pose_stamped(T, frame_id, stamp):
    """4x4 transform matrix -> PoseStamped"""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])

    q = R.from_matrix(T[:3, :3]).as_quat()  # x, y, z, w
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])

    return msg


class PlacementPoseGenerator(Node):
    def __init__(self):
        super().__init__('placement_pose_generator')

        # ---------- Parameters ----------
        self.declare_parameter('pointcloud_pt_path', '/tmp/object_cloud.pt')
        self.declare_parameter('grasp_topic', '/grasp_pose')
        self.declare_parameter('placement_topic', '/placement_pose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('placement_center_frame', 'placement_center_link')
        self.declare_parameter('broadcast_object_center_tf', True)
        self.declare_parameter('publish_in_base_frame', False)  
        # False: 发布 frame_id = placement_center_link
        # True : 发布 frame_id = base_link

        self.pointcloud_pt_path = self.get_parameter(
            'pointcloud_pt_path').get_parameter_value().string_value
        self.grasp_topic = self.get_parameter(
            'grasp_topic').get_parameter_value().string_value
        self.placement_topic = self.get_parameter(
            'placement_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.placement_center_frame = self.get_parameter(
            'placement_center_frame').get_parameter_value().string_value
        self.broadcast_object_center_tf = self.get_parameter(
            'broadcast_object_center_tf').get_parameter_value().bool_value
        self.publish_in_base_frame = self.get_parameter(
            'publish_in_base_frame').get_parameter_value().bool_value

        # ---------- ROS interfaces ----------
        self.pub = self.create_publisher(PoseStamped, self.placement_topic, 10)
        self.sub = self.create_subscription(
            PoseStamped,
            self.grasp_topic,
            self.grasp_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- Load point cloud and compute OBB ----------
        self.points_np = self.load_pt_pointcloud(self.pointcloud_pt_path)
        self.T_base_object_center, self.obb = self.compute_object_center_from_obb(self.points_np)

        self.get_logger().info(
            f'Loaded point cloud with {self.points_np.shape[0]} points')
        self.get_logger().info(
            f'object_center_link in {self.base_frame}: '
            f'[{self.T_base_object_center[0,3]:.4f}, '
            f'{self.T_base_object_center[1,3]:.4f}, '
            f'{self.T_base_object_center[2,3]:.4f}]'
        )

        # 定时广播 object_center_link tf，便于 RViz 查看
        if self.broadcast_object_center_tf:
            self.timer = self.create_timer(0.1, self.broadcast_object_center)

    def load_pt_pointcloud(self, path):
        data = torch.load(path, map_location='cpu')

        if isinstance(data, torch.Tensor):
            pts = data
        elif isinstance(data, dict):
            # 常见兼容写法
            for key in ['points', 'pointcloud', 'xyz', 'pcd']:
                if key in data:
                    pts = data[key]
                    break
            else:
                raise ValueError("`.pt` is a dict but no known point cloud key found.")
        else:
            raise ValueError("Unsupported .pt format. Expected Tensor or dict.")

        pts = pts.detach().cpu().numpy()

        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError(f'Point cloud shape must be Nx3 or Nx>=3, got {pts.shape}')

        pts = pts[:, :3]
        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]

        if len(pts) < 4:
            raise ValueError("Not enough valid points to compute a 3D bounding box.")

        return pts

    def compute_object_center_from_obb(self, points_np):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        # 经典 OBB：对大多数场景已经够用
        obb = o3d.geometry.OrientedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(points_np)
        )

        center = np.asarray(obb.center)   # in base_link
        rot = np.asarray(obb.R)           # box orientation in base_link

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = center

        return T, obb

    def broadcast_object_center(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = 'object_center_link'

        tf_msg.transform.translation.x = float(self.T_base_object_center[0, 3])
        tf_msg.transform.translation.y = float(self.T_base_object_center[1, 3])
        tf_msg.transform.translation.z = float(self.T_base_object_center[2, 3])

        q = R.from_matrix(self.T_base_object_center[:3, :3]).as_quat()
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(tf_msg)

    def grasp_callback(self, grasp_msg: PoseStamped):
        if grasp_msg.header.frame_id != self.base_frame:
            self.get_logger().error(
                f'Incoming grasp pose must be in {self.base_frame}, '
                f'but got {grasp_msg.header.frame_id}'
            )
            return

        # ^base T_grasp
        T_base_grasp = pose_to_matrix(grasp_msg.pose)

        # ^object_center T_grasp = (^base T_object_center)^-1 * ^base T_grasp
        T_object_grasp = np.linalg.inv(self.T_base_object_center) @ T_base_grasp

        # 方案1：发布“基于 placement_center_link 的位姿”
        if not self.publish_in_base_frame:
            place_msg = matrix_to_pose_stamped(
                T_object_grasp,
                self.placement_center_frame,
                self.get_clock().now().to_msg()
            )
            self.pub.publish(place_msg)
            self.get_logger().info(
                f'Published placement pose in frame {self.placement_center_frame}'
            )
            return

        # 方案2：如果你还想得到 base_link 下的绝对目标位姿，
        # 那你需要已知 ^base T_placement_center
        # 这里先假设 placement_center_link 与 base_link 重合，
        # 实际项目里请替换成 tf 查询结果
        T_base_placement_center = np.eye(4)

        # ^base T_place = ^base T_placement_center * ^placement_center T_place
        #               = ^base T_placement_center * ^object_center T_grasp
        T_base_place = T_base_placement_center @ T_object_grasp

        place_msg = matrix_to_pose_stamped(
            T_base_place,
            self.base_frame,
            self.get_clock().now().to_msg()
        )
        self.pub.publish(place_msg)
        self.get_logger().info(
            f'Published placement pose in frame {self.base_frame}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlacementPoseGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()