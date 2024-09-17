import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mask import *


class Viewer:
    def __init__(self, enable_record = False, enable_playback = False, filename = None):
        ### [1]
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print(f"Device started: {device_product_line}")

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        if enable_record:
            self.config.enable_record_to_file(filename)
        if enable_playback:
            self.config.enable_device_from_file(filename)

        # NEED TO FINISH MODIFYING CONFIG BEFORE THIS LINE
        self.profile = self.pipeline.start(self.config)
        # auto setting depth scale
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # aligns rgb to depth cam
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # set clipping distance in meters
        self.set_clipping_distance(1)

    def get_meter_distance(self, camera_unit):
        return camera_unit * self.depth_scale

    def set_clipping_distance(self, threshold):
        clipping_distance_in_meters = threshold # in meters
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale

    def get_frames(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return depth_image, color_image


    def stream(self):
        try:
            while True:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 153
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels -> expanding dimensionality to enable comparison
                bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack((bg_removed, depth_colormap))

                cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
                cv2.imshow('Align Example', images)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()

    def masked_stream(self, color_str):
        try:
            while True:
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                mask, _ = mask_hsv(color_image, color_dict_HSV[color_str][1], color_dict_HSV[color_str][0])
                print(mask.shape)


                # Remove background - Set pixels further than clipping_distance to grey
                # grey_color = 153
                # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels -> expanding dimensionality to enable comparison
                # bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                # images = np.hstack((mask, depth_colormap))

                # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
                cv2.imshow('Align Example', mask)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()



if __name__=="__main__":

    # test_viewer = Viewer(enable_playback=True, filename="test_record.bag")
    
    test_viewer = Viewer()
    test_viewer.masked_stream("purple")

    depth, color = test_viewer.get_frames()

    print(depth.shape)
    print(color)

    # color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV) #
    # plt.imshow(color)
    # plt.show()
    
    # test_viewer.stream()




## [1]"librealsense/wrappers/python/examples/align-depth2color.py at master · IntelRealSense/librealsense · GitHub", Github, Available: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py. [Accessed 17 September. 2024].
## [2]"OpenCV: Changing Colorspaces", Docs, Available: https://docs.opencv.org/4.6.0/df/d9d/tutorial_py_colorspaces.html. [Accessed 17 September. 2024].