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
                # pose_chosen = np.array([[-0.92025715, -0.30032972,  0.2508563,   0.01388752],
                #                         [-0.35248688,  0.91460925, -0.19809853, -0.7115265 ],
                #                         [-0.1699406,  -0.27072513, -0.94753796,  0.00827277],
                #                         [ 0.          ,  0.          ,  0.          ,  1.        ]]) 40000 right high

                # pose_chosen = np.array([[-0.54272383, -0.6584394,   0.5214485,  -0.10903712],
                #                         [-0.66849655,  0.714493,    0.20642751, -0.06255007],
                #                         [-0.5084913,  -0.23655339, -0.8279367,   0.03678282],
                #                         [ 0.          ,  0.          ,  0.          ,  1.        ]])  # ifl demo

                # pose_chosen = np.array([[-0.5799282,   0.5954162,  -0.5560242,  -0.52425665],
                #                         [ 0.79005605,  0.24455185, -0.56214404,  0.05535678],
                #                         [-0.1987329,  -0.76529336, -0.6122347,   0.09503099],
                #                         [ 0.          ,  0.          ,  0.          ,  1.        ]]) # 40000 front high


                # pose_chosen = np.array([[ 0.33840626,  0.74378955, -0.5764185,  -0.27029335],
                #                        [ 0.226851,   -0.65897673, -0.717139,    0.25941396],
                #                        [-0.91324687,  0.11192322, -0.39173123,  0.05664881],
                #                        [ 0.          ,  0.          ,  0.          ,  1.        ]]) # 40000 front high new 3 (0.6)

                # pose_chosen = np.array([[ 0.10755713,  0.9627931,   0.24791317, -0.30129868],
                #                         [ 0.96577436, -0.04198113, -0.2559638,   0.08538453],
                #                         [-0.23603249,  0.26695895, -0.9343563,   0.10218767],
                #                         [ 0.          ,  0.          ,  0.          ,  1.        ]]) # 40000 front high new 3 (0.9)

                # pose_chosen = np.array([[ 0.87090266,  0.36365145,  0.33058473, -0.37718636],
                #                         [ 0.42729926, -0.89259994, -0.14380826, -0.10250764],
                #                         [ 0.24278383,  0.2665016,  -0.9327556,   0.02477966],
                #                         [ 0.          ,  0.          ,  0.          ,  1.        ]])   # 40000 front low new (0.8)
                
                # pose_chosen = np.array([[-0.5472112,   0.25824377, -0.7961596,  -0.29509807],
                #                          [ 0.8067911,   0.41596693, -0.41959465,  0.0487287 ],
                #                          [ 0.22281836, -0.8719413,  -0.43597057,  0.09977922],
                #                          [ 0.          ,  0.          ,  0.          ,  1.        ]]) # 40000 front high new new(0.8)

                pose_chosen = np.array([[ 0.10755713,  0.9627931,   0.24791317, -0.30129868],
                                        [ 0.96577436, -0.04198113, -0.2559638,   0.08538453],
                                        [-0.23603249,  0.26695895, -0.9343563,   0.10218767],
                                        [ 0.          ,  0.          ,  0.          ,  1.        ]]) # 240000 front high new new (0.8)



                quat = quaternion_from_matrix(pose_chosen)
                translation = translation_from_matrix(pose_chosen)
                print("Chosen quaternion: ", quat)
                print("Chosen translation: ", translation)

                pose_chosen = np.concatenate([translation, quat])
                print("Chosen pose: ", pose_chosen)

                # 40000 right high
                # Chosen quaternion:  (-0.8698483506038269, -0.23478046215074366, 0.14561088760992308, 0.40870460109171164)
                # Chosen translation:  [0.05596709, 0.02661008, 0.10447411]
                # Chosen pose:  [ 0.05596709,  0.02661008,  0.10447411, -0.86984835, -0.23478046,  0.14561089,  0.4087046 ]

                # ifl demo
                # Chosen quaternion:  (-0.37773005691187905, 0.8782302348118152, -0.008575731990090648, 0.2931861449464595)
                # Chosen translation:  [-0.10903712 -0.06255007  0.03678282]
                # Chosen pose:  [-0.10903712 -0.06255007  0.03678282 -0.37773006  0.87823023 -0.00857573  0.29318614]

                # 40000 front high
                # Chosen quaternion:  (-0.44377771212793576, -0.7804989943712283, 0.425188679628956, 0.1144432117662794)
                # Chosen translation:  [-0.52425665  0.05535678  0.09503099]
                # Chosen pose:  [-0.52425665  0.05535678  0.09503099 -0.44377771 -0.78049899  0.42518868  0.11444321]

                # Chosen quaternion:  (0.7217850235832565, 0.6679854048408792, 0.004115041864258905, 0.18112135456908393)
                # Chosen translation:  [-0.30129868  0.08538453  0.10218767]
                # Chosen pose:  [-0.30129868  0.08538453  0.10218767  0.72178502  0.6679854   0.00411504  0.18112135]

                # 40000 front high new new (0.8)
                # Chosen quaternion:  (-0.3437995095582415, -0.774459276306377, 0.41691537464684375, 0.32893205522356406)
                # Chosen translation:  [-0.29509807  0.0487287   0.09977922]
                # Chosen pose:  [-0.29509807  0.0487287   0.09977922 -0.34379951 -0.77445928  0.41691537 0.32893206]

                # 240000 front high new new (0.8)
                # Chosen quaternion:  (0.7217850235832565, 0.6679854048408792, 0.004115041864258905, 0.18112135456908393)
                # Chosen translation:  [-0.30129868  0.08538453  0.10218767]
                # Chosen pose:  [-0.30129868  0.08538453  0.10218767  0.72178502  0.6679854   0.00411504  0.18112135]



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