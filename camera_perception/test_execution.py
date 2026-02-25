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
                pose_chosen = np.array([[ 0.8473512,   0.2894231,  -0.44523045,  0.05596709],
                                        [ 0.52747047, -0.55567735,  0.6426489,   0.02661008],
                                        [-0.06140707, -0.7793952,  -0.623516,    0.10447411],
                                        [ 0.          ,  0.          ,  0.          ,  1.        ]])

                quat = quaternion_from_matrix(pose_chosen)
                translation = translation_from_matrix(pose_chosen)
                print("Chosen quaternion: ", quat)
                print("Chosen translation: ", translation)

                pose_chosen = np.concatenate([translation, quat])
                print("Chosen pose: ", pose_chosen)

                # Chosen quaternion:  (-0.8698483506038269, -0.23478046215074366, 0.14561088760992308, 0.40870460109171164)
                # Chosen translation:  [0.05596709, 0.02661008, 0.10447411]
                # Chosen pose:  [ 0.05596709,  0.02661008,  0.10447411, -0.86984835, -0.23478046,  0.14561089,  0.4087046 ]

                # if pose_chosen is not None:
                #     pose = self.process_grasp(pose_chosen)
                #     success = self.execute_grasp(pose)
                # else:
                #     print("No grasp pose detected, please try again.")
            elif user_input == "q":
                self.shutdown = True
            break

    # def process_grasp(self, pose):
    #     pose = list_to_pose(pose)
    #     return pose
    
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