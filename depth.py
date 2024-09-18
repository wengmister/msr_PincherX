import cv2
import numpy as np
import pyrealsense2 as rs


def get_distance_at_point(depth_image, x, y):

    # Ensure the coordinates are within the frame bounds
    distance = -1
    if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
        # Get the distance value from the depth image
        distance = depth_image[y, x] 
    return distance