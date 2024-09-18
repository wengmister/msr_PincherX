import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mask import *
from contour import *
from depth import *
from motion import *

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

        # enable data streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.accel)

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

                cv2.namedWindow("Align Example", cv2.WINDOW_NORMAL)
                cv2.imshow("Align Example", images)
                key = cv2.waitKey(1)
                # Press esc or "q" to close the image window
                if key & 0xFF == ord("q") or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()

    def on_trackbar(self, val):
        pass

    def create_hsv_trackbars(self, window_name):
        # Create trackbars for lower HSV values
        cv2.createTrackbar("Lower H", window_name, 0, 179, self.on_trackbar)
        cv2.createTrackbar("Lower S", window_name, 0, 255, self.on_trackbar)
        cv2.createTrackbar("Lower V", window_name, 0, 255, self.on_trackbar)

        # Create trackbars for upper HSV values
        cv2.createTrackbar("Upper H", window_name, 0, 179, self.on_trackbar)
        cv2.createTrackbar("Upper S", window_name, 0, 255, self.on_trackbar)
        cv2.createTrackbar("Upper V", window_name, 0, 255, self.on_trackbar)

    def create_threshold_trackbar(self, window_name):
        cv2.createTrackbar("Threshold", window_name, 0, 255, self.on_trackbar)

    def get_hsv_values(self, window_name):
        # Get current positions of all trackbars for lower HSV
        lower_h = cv2.getTrackbarPos("Lower H", window_name)
        lower_s = cv2.getTrackbarPos("Lower S", window_name)
        lower_v = cv2.getTrackbarPos("Lower V", window_name)

        # Get current positions of all trackbars for upper HSV
        upper_h = cv2.getTrackbarPos("Upper H", window_name)
        upper_s = cv2.getTrackbarPos("Upper S", window_name)
        upper_v = cv2.getTrackbarPos("Upper V", window_name)

        return (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v),

    def get_threshold_val(self, window_name):
        threshold = cv2.getTrackbarPos("Threshold", window_name)

        return threshold

    def masked_stream(self, preset_hsv = False):

        hsv_window_name = "hsv_window"
        threshold_window = "thresh_window"
    
        # Create a window and the trackbars for HSV control
        cv2.namedWindow(hsv_window_name)
        self.create_hsv_trackbars(hsv_window_name)

        cv2.namedWindow(threshold_window)
        self.create_threshold_trackbar(threshold_window)
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
                accel_frame = get_accel_frame(aligned_frames, 2)

                # Validate that all frames are valid
                if not aligned_depth_frame or not color_frame or not accel_frame.all():
                    continue

                # reminder to flip camera
                if accel_frame[1] > 0:
                    print(f"Camera is upside down. Flip it!!")

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # preset values
                if preset_hsv:
                    lower_hsv = (115, 90, 80)
                    upper_hsv = (135, 245, 200)
                    thresh = 120
                else:
                    lower_hsv, upper_hsv = self.get_hsv_values(hsv_window_name)
                    thresh = self.get_threshold_val(threshold_window)
                mask, _ = mask_hsv(color_image, lower_hsv, upper_hsv)
                mask = cv2.GaussianBlur(mask, (5, 5), 0)
                # convert non-3d images to 3d
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels -> expanding dimensionality to enable comparison
                mask_3d = np.dstack((mask, mask, mask))

                # Remove background
                removed_color = 0
                bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), removed_color, color_image)
                mask_bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), removed_color, mask_3d)

                # Trace contour
                contours, coms, area = ordered_contour(mask_bg_removed[:,:,0], thresh)
                
                if not area:
                    print("Too far! Else object not found")
                else:
                    distance_com = get_distance_at_point(depth_image, coms[0], coms[1])
                    print(f"The COM of the largest contour is at {coms} with area {area}. The centroid is {self.get_meter_distance(distance_com)}m away")
                    
                # Render images:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                contour_image = cv2.cvtColor(mask_bg_removed[:,:,0], cv2.COLOR_GRAY2BGR)  # Ensure it"s in BGR for color drawing


                cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)  # Draw all contours with green color
                cv2.circle(contour_image, coms, 10, (0, 0, 255), -1)
                # Display the image with cqontours
                images = np.hstack((mask_bg_removed, depth_colormap, bg_removed, contour_image))
                cv2.imshow(threshold_window, contour_image)
                # Display
                cv2.imshow(hsv_window_name, images)

                # Terminal commands
                key = cv2.waitKey(1)
                # Press esc or "q" to close the image window
                if key & 0xFF == ord("q") or key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            self.pipeline.stop()



if __name__=="__main__":

    # test_viewer = Viewer(enable_playback=True, filename="test_record.bag")
    
    test_viewer = Viewer()
    test_viewer.masked_stream(preset_hsv=True)

    # depth, color = test_viewer.get_frames()

    # print(depth.shape)
    # print(color)

    # color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV) #
    # plt.imshow(color)
    # plt.show()
    
    # test_viewer.stream()




## [1]"librealsense/wrappers/python/examples/align-depth2color.py at master · IntelRealSense/librealsense · GitHub", Github, Available: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py. [Accessed 17 September. 2024].
## [2]"OpenCV: Changing Colorspaces", Docs, Available: https://docs.opencv.org/4.6.0/df/d9d/tutorial_py_colorspaces.html. [Accessed 17 September. 2024].