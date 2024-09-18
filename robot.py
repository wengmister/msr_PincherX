from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
# The robot object is what you use to control the robot


class Robot:
    def __init__(self):
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.robot.arm.go_to_home_pose()
        self.robot.gripper.release()

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
                self.robot.arm.set_ee_pose_matrix(new_pose)
            case "y":
                self.move_joint(0, step_size)
            case "z":
                new_pose[2][3] += step_size
                self.robot.arm.set_ee_pose_matrix(new_pose)
            case _ :
                print("unsupported movement")

    def move_joint(self, j, angle):
        match j:
            case 0:
                joint = "waist"
            case 1:
                joint = "shoulder"
            case 2:
                joint = "elbow"
            case 3:
                joint = "wrist_angle"

        current_angle = self.robot.arm.get_single_joint_command(joint)
        
        new_angle = current_angle + angle

        print(f"Moving {joint} joint, current angle: {current_angle}, new angle: {new_angle}")
        self.robot.arm.set_single_joint_position(joint, new_angle)

    def gripper_close(self):
        self.robot.gripper.grasp()

    def gripper_open(self):
        self.robot.gripper.release()
                

class Gripper:
    def __init__(self):
        self.gripper
        

if __name__ == "__main__":

    test_robot = Robot()



    test_robot.move_joint(3, -0.5)

    test_robot.step("z", +0.05)

    test_robot.step("x", -0.05)

    test_robot.step("y", -0.5)

    test_robot.step("z", -0.05)

    test_robot.step("x", 0.05)

    test_robot.shutdown()