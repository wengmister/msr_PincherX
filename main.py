from rs_viewer import *
import threading
import time
from robot import *
from calibration_points import *
import pyrealsense2 as rs
from kabsch import *
from calibration import *



if __name__=="__main__":
    robot = Robot()
    viewer = Viewer()
    robot.robot.gripper.release()
    T_c2r = calibrate(robot, viewer)

    robot.robot.arm.go_to_home_pose()
    robot.robot.gripper.release()

    robot.robot.arm.go_to_sleep_pose()

    time.sleep(2)
    target_point = None
    while target_point == None:
        print("Acquiring target...")
        target_point = viewer.target_acquisition()
    target_formatted = np.array([target_point[0], target_point[1], target_point[2], 1])

    target_robot_frame = T_c2r @ target_formatted

    print(f"Target in robot frame: \n\
          {target_robot_frame}")


    robot.robot.arm.set_ee_pose_components(target_robot_frame[0], target_robot_frame[1], target_robot_frame[2])

    robot.robot.gripper.grasp()

    robot.shutdown()

