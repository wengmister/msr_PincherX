from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
# The robot object is what you use to control the robot


class Robot:
    def __init__(self):
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.robot.arm.go_to_home_pose()

    def shutdown(self):
        robot_shutdown()

    def step(self, dir, step_size):
        current_pose = self.robot.arm.get_ee_pose()
        print(current_pose)
        new_pose = current_pose
        print(f"Moving in {dir} with step {step_size}")
        match dir:
            case "x":
                new_pose[0][3] += step_size
            case "z":
                new_pose[2][3] += step_size
            case _ :
                print("unsupported movement")

        self.robot.arm.set_ee_pose_matrix(new_pose)


        

if __name__ == "__main__":

    test_robot = Robot()

    test_robot.step("x", -0.05)

    test_robot.step("z", 0.05)

    test_robot.step("x", -0.05)

    test_robot.shutdown()