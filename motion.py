#! /usr/lib/env python3

import pyrealsense2 as rs
import numpy as np

### [1]
def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

def get_accel_frame(frame, motion_ind):
    return accel_data(frame[motion_ind].as_motion_frame().get_motion_data())

if __name__=="__main__":
    p = initialize_camera()
    try:
        while True:
            f = p.wait_for_frames()
            accel = accel_data(f[0].as_motion_frame().get_motion_data())
            gyro = gyro_data(f[1].as_motion_frame().get_motion_data())
            print("accelerometer: ", accel)
            print("gyro: ", gyro)

    finally:
        p.stop()



### [1]"IMU data stream (Python) · Issue #3409 · IntelRealSense/librealsense · GitHub", Github, Available: https://github.com/IntelRealSense/librealsense/issues/3409. [Accessed 18 September. 2024].