import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

def camera_init() -> any:

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)
    return pipeline

#     return pipeline
# def camera_init() -> any:
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_device(connect_device[0])

def camera_main(pipeline):


    frames = pipeline.wait_for_frames()
    # 深度图
    depth_frame = frames.get_depth_frame()
    # 正常读取的视频流
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:

        print("当前没有读到任何数据，请重新检查输入！！！")


    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


    return depth_image, color_image
 
def camera_close(pipeline):
    pipeline.stop()


if __name__== "__main__":

    pipeline = camera_init()

    try:
        while True:
            depth_image,color_image = camera_main(pipeline=pipeline)

            cv2.imshow('RealSense', color_image)
            state_save_path='/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset1/state'
            camera_save_path = '/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset1/camera'

            if not os.path.isdir(state_save_path):
                os.makedirs(state_save_path)
                os.makedirs(camera_save_path)


            camera_path = f'{state_save_path}/../camera/{time.time()}.png ' 
            # cv2.imwrite(camera_path,color_image)
            # print("写入图片成功！！",camera_path)

            cv2.waitKey(25)
            print(time.localtime())

    finally:
    # Stop streaming
        camera_close(pipeline=pipeline)

    





