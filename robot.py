from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
import modern_robotics as mr




def htm_from_xz_only(x, z):
    """
    Produce a homogeneous transformation matrix given x and z, ignoring the y input.
    
    Parameters:
    x (float): Translation along the x-axis
    z (float): Translation along the z-axis
    
    Returns:
    numpy.ndarray: The 4x4 homogeneous transformation matrix
    """
    # Create the homogeneous transformation matrix with only translation
    T = np.eye(4)
    T[0, 3] = x  # Set translation along the x-axis
    T[2, 3] = z  # Set translation along the z-axis
    
    return T

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

    def capture(self, x, y, z):
        self.robot.arm.set_ee_pose_components(x,y,z)


    # I dont think this works         
    def robot_IK(self, desired):
        Blist = self.robot.arm.robot_des.M
        thetalist0 = np.array([0.1, 0.1, 0.1, 0.1])
        M = self.robot.arm.get_ee_pose()
        T = desired
        thetalist, success = mr.IKinBody(Blist, M, T, thetalist0, 0.01, 0.01)

        if success:
            print(f"Solution found. Moving to {thetalist}")
            return thetalist
        else:
            print(f"No solution found.")
            return self.robot.arm.get_joint_commands()



    

if __name__ == "__main__":

    test_robot = Robot()

    calibration_list = [[0.2, -0.2, 0.1]
                        [0.2 ,0.2, 0.1]
                        [-0.2, -0.2, 0.1]]

    test_robot.robot.arm.set_ee_pose_components(0.2, -0.2, 0.1)
    
    test_robot.robot.gripper.grasp()
    # test_robot.move_joint(3, -0.5)

    # test_robot.step("z", +0.05)

    # test_robot.step("x", -0.05)

    # test_robot.step("y", -0.5)

    # test_robot.step("z", -0.05)

    test_robot.shutdown()