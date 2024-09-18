from rs_viewer import *
import threading
import time
from robot import *
from calibration_points import *
import pyrealsense2 as rs
from kabsch import *


def calibrate(test_robot: Robot, test_viewer: Viewer):
    test_robot.robot.gripper.grasp()

    robot_points = generate_linspaced_coordinates(0.15, 0.05, 0.25, 2,2,2)

    camera_points = np.empty((0,3))

    for i in robot_points:
        
        test_robot.robot.arm.set_ee_pose_components(i[0], i[1], i[2])
        centroid, depth, depth_frame = test_viewer.get_centroid_and_depth()
        print(centroid, depth)
        
        # deproject 2d points to 3d space
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        point_pos = rs.rs2_deproject_pixel_to_point(intrinsics, [centroid[0], centroid[1]], depth)
        print(point_pos)

        # append
        camera_points = np.concatenate([camera_points, [point_pos]], axis=0)

    print("Calibration movement sequence finished!")
    print(f"Camera calibration points: \n\
          {camera_points}")
    print(f"Robot calibration points: \n\
          {robot_points}")

    test_robot.shutdown()
    
    camera_to_robot = kabsch_algorithm(camera_points, robot_points)
    print(f"Optimal transformation from camera to robot: \n\
          {camera_to_robot}")
    
    return camera_to_robot



if __name__=="__main__":

    test_robot = Robot()
    test_viewer = Viewer()

    calibrate(test_robot, test_viewer)

    # # test_robot.robot.gripper.release()
    # test_robot.robot.gripper.grasp()

    # robot_points = generate_linspaced_coordinates(0.15, 0.05, 0.25, 2,2,2)

    # camera_points = np.empty((0,3))

    # for i in robot_points:
        
    #     test_robot.robot.arm.set_ee_pose_components(i[0], i[1], i[2])
    #     centroid, depth, depth_frame = test_viewer.get_centroid_and_depth()
    #     print(centroid, depth)
        
    #     # deproject 2d points to 3d space
    #     intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    #     point_pos = rs.rs2_deproject_pixel_to_point(intrinsics, [centroid[0], centroid[1]], depth)
    #     print(point_pos)

    #     # append
    #     camera_points = np.concatenate([camera_points, [point_pos]], axis=0)

    # print("Calibration movement sequence finished!")
    # print(f"Camera calibration points: \n\
    #       {camera_points}")
    # print(f"Robot calibration points: \n\
    #       {robot_points}")

    # test_robot.shutdown()
    
    # camera_to_robot = kabsch_algorithm(camera_points, robot_points)
    # print(f"Optimal transformation from camera to robot: \n\
    #       {camera_to_robot}")

