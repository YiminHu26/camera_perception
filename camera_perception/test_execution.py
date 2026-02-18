import threading
from .inference_node_base import *
import time
import rclpy
import cv2
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Transform
# from vmf_contact_main.train import main_module, parse_args_from_yaml
from cv_bridge import CvBridge
import os, torch
import numpy as np
import copy
from tf_transformations import quaternion_from_matrix, translation_from_matrix
from PIL import Image
from .utils_camera import *
import time

class Execution(AIRNode):

    def __init__(self):
        super().__init__(use_langsam=False)


        self.user_input_thread = threading.Thread(target=self.handle_user_input)
        self.user_input_thread.start()
        # self.agent = main_module(parse_args_from_yaml(current_file_folder + "/../vmf_contact_main/config.yaml"), learning=False)
        self.set_vel_acc(.3, .1)

    def handle_user_input(self):
        while True:
            user_input = input("Enter 's' to start next capture and 'q' to quit: ")
            if user_input == "s":
                pose_chosen = np.array([[-0.9209464,   0.14233127, 0.36276636, -0.08807984],
                                    [ 0.03427862,  0.9568921,  -0.2884139,   0.19987226],
                                    [-0.3881786,  -0.25317866, -0.88612527,  0.05802576],
                                    [ 0.          ,  0.          ,  0.          ,  1.        ]])
                quat = quaternion_from_matrix(pose_chosen)
                translation = translation_from_matrix(pose_chosen)
                print("Chosen quaternion: ", quat)

                pose_chosen = np.concatenate([translation, quat])
                print("Chosen pose: ", pose_chosen)

                if pose_chosen is not None:
                    pose = self.process_grasp(pose_chosen)
                    success = self.execute_grasp(pose)
                else:
                    print("No grasp pose detected, please try again.")
            elif user_input == "q":
                self.shutdown = True
            break

    def process_grasp(self, pose):
        pose = list_to_pose(pose)
        return pose
    
def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = Execution()
    try:
        rclpy.spin(pcd_listener)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Exiting")

    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()