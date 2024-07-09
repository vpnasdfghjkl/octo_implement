import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
# import datetime
from datetime import datetime
def camera_init() -> any:

    #pipeline = rs.pipeline()
    config = rs.config()

    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

    # 获取RGB图像
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 60)

    # Start streaming
    # profile = pipeline.start(config)
    # device = profile.get_device()
    # device.hardware_reset()

    # check相机是不是进来了
    connect_device = []
    for d in rs.context().devices:
        print('Found device: ',
              d.get_info(rs.camera_info.name), ' ',
              d.get_info(rs.camera_info.serial_number))
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device.append(d.get_info(rs.camera_info.serial_number))


    if len(connect_device) < 2:
        print('Registrition needs two camera connected.But got one.')
        exit()

    # 确认相机并获取相机的内部参数
    pipeline1 = rs.pipeline()
    config.enable_device(connect_device[0])
    # pipeline_profile1 = pipeline1.start(rs_config)
    pipeline1.start(config)

    pipeline2 = rs.pipeline()
    config.enable_device(connect_device[1])
    # pipeline_profile2 = pipeline2.start(rs_config)
    pipeline2.start(config)

    return pipeline1,pipeline2

def camera_main(pipeline):

    frames = pipeline.wait_for_frames()
    
    # t_s1=frames.get_timestamp()
    # t_s2=frames2.get_timestamp()
    # dt1 = datetime.fromtimestamp(t_s1 / 1e3)
    # print("dt1",dt1)
    # dt2 = datetime.fromtimestamp(t_s2 / 1e3)

    # 深度图
    # depth_frame = frames.get_depth_frame()

    # 正常读取的视频流
    color_frame = frames.get_color_frame()

    # if not depth_frame or not color_frame:
    if not color_frame:

        print("当前没有读到任何数据，请重新检查输入！！！")
    

    # Convert images to numpy arrays
    # depth_image = np.asanyarray(depth_frame.get_data())  # 深度图像的转换
    color_image = np.asanyarray(color_frame.get_data())

    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


    # return depth_image, color_image
    return color_image
 
def camera_close(pipeline1, pipeline2):

    pipeline1.stop()
    pipeline2.stop()



if __name__== "__main__":

    pipeline1,pipeline2 = camera_init()
    try:
        
        while True:

            # depth_image1,color_image1 = camera_main(pipeline=pipeline1)
            # depth_image2,color_image2 = camera_main(pipeline=pipeline2)

            color_image1 = camera_main(pipeline=pipeline1)
            color_image2 = camera_main(pipeline=pipeline2)

            

            cv2.imshow('RealSense1', color_image1)
            cv2.imshow('RealSense2', color_image2)
            state_save_path='/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset150/state'
            camera_save_path = '/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/Dataset150/camera'

            if not os.path.isdir(state_save_path):
                os.makedirs(state_save_path)
                os.makedirs(camera_save_path)


            camera_path1 = f'{state_save_path}/../camera1/{time.time()}.png '
            camera_path2 = f'{state_save_path}/../camera2/{time.time()}.png '
            # cv2.imwrite(camera_path,color_image)
            # print("写入图片成功！！",camera_path)

            cv2.waitKey(1)
            print(time.localtime())

    finally:
    # Stop streaming
        camera_close(pipeline=pipeline1)
        camera_close(pipeline=pipeline2)

    





