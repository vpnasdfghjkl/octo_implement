import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

def camera_init() -> any:

    pipeline = rs.pipeline()
    config = rs.config()

    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 60)

    # Start streaming
    profile = pipeline.start(config)


    return pipeline

def camera_main(pipeline):

    frames = pipeline.wait_for_frames()

    color_frame = frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())



    return color_image
 
def camera_close(pipeline):
    pipeline.stop()


if __name__== "__main__":

    pipeline = camera_init()

    try:
        while True:
            # depth_image,color_image = camera_main(pipeline=pipeline)
            color_image = camera_main(pipeline=pipeline)

            cv2.imshow('RealSense', color_image)
            state_save_path='/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset1/state'
            camera_save_path = '/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset1/camera'

            if not os.path.isdir(state_save_path):
                os.makedirs(state_save_path)
                os.makedirs(camera_save_path)


            camera_path = f'{state_save_path}/../camera/{time.time()}.png ' 


            print(time.localtime())

    finally:
    # Stop streaming
        camera_close(pipeline=pipeline)

    





