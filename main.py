from rs_viewer import *
import time
from robot import *
from calibration_points import *
import pyrealsense2 as rs
from kabsch import *
from calibration import *
import keyboard


def main(calibrated = False):
    robot = Robot()
    viewer = Viewer()


    if not calibrated:
        robot.robot.gripper.release()
        robot.robot.gripper.grasp()
        T_c2r = calibrate(robot, viewer)
    if calibrated:
        T_c2r = np.loadtxt('t_c2r.txt')

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


    robot.robot.arm.set_ee_pose_components(target_robot_frame[0]-0.01, target_robot_frame[1], target_robot_frame[2])

    robot.robot.gripper.grasp()

    time.sleep(2)

    robot.move_joint(0, -0.8)
    
    input("Press enter to continue")
    robot.robot.gripper.release()
    




    # def move_joint_on_m_press():
    #     print("Key 'm' pressed. Executing robot.move_joint(0, -0.5)")
    #     robot.move_joint(0, -0.5)

    # def release_gripper_on_r_press():
    #     print("Key 'r' pressed. Executing gripper release")
    #     robot.robot.gripper.release()
    
        

    # keyboard.add_hotkey('m', move_joint_on_m_press)
    # print("Press 'm' to move the joint.")
    # keyboard.wait('esc') 

    # keyboard.add_hotkey('r', release_gripper_on_r_press)
    # print("Press r to release the gripper")
    # keyboard.wait('esc') 


if __name__=="__main__":
    
    main(True)

    robot_shutdown()

    print("WE'VE DONE IT BOIS!!!!!")
