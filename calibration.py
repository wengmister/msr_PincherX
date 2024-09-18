from rs_viewer import *
import threading
import time
from robot import *
from calibration_points import *
import pyrealsense2 as rs


if __name__=="__main__":

    test_robot = Robot()
    test_viewer = Viewer()


    # test_robot.robot.gripper.release()
    test_robot.robot.gripper.grasp()

    calibration_coordinates = generate_linspaced_coordinates(0.15, 0.05, 0.25, 2,2,2)

    camera_points = np.empty((0,3))

    for i in calibration_coordinates:
        
        test_robot.robot.arm.set_ee_pose_components(i[0], i[1], i[2])
        centroid, depth, depth_frame = test_viewer.get_centroid_and_depth()
        print(centroid, depth)
        
        # deproject 2d points to 3d space
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        point_pos = rs.rs2_deproject_pixel_to_point(intrinsics, [centroid[0], centroid[1]], depth)
        print(point_pos)

        # append
        camera_points = np.concatenate([camera_points, [point_pos]], axis=0)

    print(calibration_coordinates.shape)
    print(camera_points.shape)

    test_robot.shutdown()


