ros2 run arm_api2_py arm_api2_client_example 
[INFO] [1772030982.418745992] [my_node]: MyNode has been started
[INFO] [1772030982.453158688] [my_node]: [ArmApi2Client] Initializing...
[INFO] [1772030982.453885565] [my_node]: [ArmApi2Client] Waiting for action servers...
[INFO] [1772030990.472698025] [my_node]: [ArmApi2Client] Action servers are available: [False, False, False, False]
[INFO] [1772030990.474177988] [my_node]: [ArmApi2Client] Waiting for service servers...
[INFO] [1772030998.490531603] [my_node]: [ArmApi2Client] Service servers are available: [False, False, False, False, True, True]
Current pose:  geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))
Select wheter to 
                  - send [c]artesian, [j]oint_state or [p]ath goal, 
                  - [go] open gripper, [gc] close gripper,
                  - [sws] switch controller to servo, 
                  - [swj] switch controller to joint control, 
                  - [swc] switch controller to cartesian control, 
                  - [l]eft twist movement, [r]ight twist movement, 
                  - [1] base rotation ccw, [2] base rotation cw, 
                   - [s]top movement, 
                  - [pot] plan only true, [pof] plan only false, 
                  - or anything else to end:
c
[INFO] [1772031040.508076242] [my_node]: [ArmApi2Client] set_vel_acc: Waiting for service server...
[ERROR] [1772031042.513425463] [my_node]: [ArmApi2Client] set_vel_acc: service server not available
[INFO] [1772031042.514924913] [my_node]: [ArmApi2Client] list_controllers: Waiting for service server...
[INFO] [1772031042.516583986] [my_node]: [ArmApi2Client] list_controllers: request sent, waiting for response...
[INFO] [1772031042.548402823] [my_node]: [ArmApi2Client] Available controllers: 6
[INFO] [1772031042.549364844] [my_node]: [ArmApi2Client] Controller scaled_joint_trajectory_controller is already active
[INFO] [1772031042.549962563] [my_node]: [ArmApi2Client] Controller forward_position_controller is already inactive
[INFO] [1772031042.550518489] [my_node]: [ArmApi2Client] No controllers to switch
[INFO] [1772031042.554867298] [my_node]: [ArmApi2Client] change_state: Waiting for service server...
[ERROR] [1772031044.559718237] [my_node]: [ArmApi2Client] change_state: service server not available
[INFO] [1772031044.561423583] [my_node]: [ArmApi2Client] Waiting for action server...
