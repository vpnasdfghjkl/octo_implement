#!/usr/bin/env python
"""
  机器人位姿获取
    --主要获取遥操作机械臂以及机器人本机数据集

"""

import os
import sys

import rospy
import time 
import math
import cv2

from threading import Thread

from dynamic_biped.msg import robotArmInfo
# from dynamic_biped.msg import robotArmQVVD
from dynamic_biped.msg import robotHandPosition        # 机械臂关节的角度和信息
from sensor_msgs.msg import JointState
from camera_init1 import camera_init,camera_main,camera_close

pipeline = camera_init()

class MyKuavo(object):

    """ 初始化函数 """

    def __init__(self, queue_size, datalen, file_name,robot_instance=None):

        rospy.init_node("Human_subscrib_node")

        self.latest_Q = None  # 人形机器人机械臂的位姿
        self.latest_JointPosition_QV = None  # 遥操作机械臂的位姿
        self.queue_size = queue_size

        self.count_state = 1
        self.command = 1
        self.gripper = 1

        self.datalen = datalen
        self.filteredAngle = [0] * datalen

        self.flie_state = file_name[0]
        self.flie_command = file_name[1]
        self.flie_gripper = file_name[2]


    """ ROS函数 """
    def get_RobotState_callback(self, msg):
        """机器人本体的收集函数"""

        self.state_Q = msg.q

        self.state_Q = rad_to_angle(self.state_Q)

        print(time.time(), "^"*10)

        self.count_state = self.unpickle_state(state_save_path=self.flie_state, count_state=self.count_state)


    def get_command_callback(self, msg):

        """遥操作的收集函数"""

        self.command_JointPosition = msg.position

        # print(self.command_JointPosition, end = "\n\n")

        self.command = self.unpickle_command(command_save_path=self.flie_command, command=self.command)

    def get_gripper_callback(self,msg):
        """夹爪的收集函数"""

        left_hand_position = msg.left_hand_position
        right_hand_position = msg.right_hand_position

        self.hand_position = left_hand_position + right_hand_position

        # print("夹爪的值为：", self.hand_position)

        self.gripper = self.unpickle_gripper(gripper_save_path=self.flie_gripper, count_gripper=self.gripper)

    def get_image(self):

        """捕获摄像头数据，是一个多线程程序，一直在拿取摄像头数据并且更新"""
        # depth_image, color_image = camera_main(pipeline=pipeline)
        color_image = camera_main(pipeline=pipeline)
        # cv2.imshow("color_img", color_image)
        #
        # cv2.waitKey(10)

        return color_image


    """ 功能函数 """ 
    def unpickle_state(self, count_state, state_save_path):
        """保存机器人本体信息+摄像头数据的代码"""

        if not os.path.isdir(state_save_path):
            os.makedirs(state_save_path)

        # 写入当前的位姿
        label = self.state_Q

        with open(f'{state_save_path}/{time_label}_{os.path.basename(state_save_path)}.txt',"a") as txt:

            if count_state == 1:
                txt.write("当前是state" + "\n")

            txt.write(str([time.time()]) + " " + str(label) + "\n")

        # 写入图片
        self.color_image = self.get_image()
        
        cv2.imshow("color_img", self.color_image)
        cv2.waitKey(5)

        camera_path = f'{state_save_path}/../camera/{time.time()}.png '
        cv2.imwrite(camera_path, self.color_image)

        count_state = count_state + 1
    
        return count_state

    def unpickle_command(self, command, command_save_path=None):

        if not os.path.isdir(command_save_path):

            os.makedirs(command_save_path)

        label = list(self.command_JointPosition)
      
        with open(f'{command_save_path}/{time_label}_{os.path.basename(command_save_path)}.txt',"a") as txt1:

            if command == 1:

                txt1.write("当前是command" + "\n")

            txt1.write(str([time.time()]) + " " + str(label) +"\n")
            command = command + 1
    
        return command
    
    
    def unpickle_gripper(self, count_gripper, gripper_save_path=None):

        if not os.path.isdir(gripper_save_path):

            os.makedirs(gripper_save_path)

        label = list(self.hand_position)

        with open(f'{gripper_save_path}/{time_label}_{os.path.basename(gripper_save_path)}.txt',"a") as txt2:

            if count_gripper == 1:

                txt2.write("当前是gripper" + "\n")

            txt2.write(str([time.time()]) + " " + str(label) +"\n")

            count_gripper= count_gripper + 1
    
        return count_gripper

             
    def openoffset(self):

        self._RobotState_sub = rospy.Subscriber("/robot_arm_q_v_tau", robotArmInfo, self.get_RobotState_callback)
        # self._RobotState_sub = rospy.Subscriber("/robot_arm_q_v_tau", robotArmQVVD, self.get_RobotState_callback)
        self._Robotcommand_sub = rospy.Subscriber("/kuavo_arm_traj", JointState, self.get_command_callback)
        self._gripper_sub = rospy.Subscriber("/robot_hand_position", robotHandPosition, self.get_gripper_callback)

        try:
            rospy.spin()

        except:
            camera_close(pipeline=pipeline)


def rad_to_angle(rad_list: list) -> list:
    """弧度转变为角度"""

    angle_list = [0 for _ in range(len(rad_list))]
    for i, rad in enumerate(rad_list):
        angle_list[i] = rad / math.pi * 180.0
    return angle_list


def my_mkdir(file_names: list, time_label:int):

    for flie_name in file_names:

        if not os.path.isdir(flie_name):
            os.makedirs(flie_name)

        with open(f'{flie_name}/{time_label}_{os.path.basename(flie_name)}.txt', 'w') as file:
            continue

    return file_names

# 主函数


def main(file_names: list):

    rospy.logwarn("当前正常，开始接收信息")
    Kuavo = MyKuavo(queue_size=10, datalen=14, file_name=file_names)

    Kuavo.openoffset()


if __name__ == "__main__":

    # for i in range(10):

    time_label = str(time.time())

    file_names = [f'/home/rebot801/LIuXin/ICCUB_ws/Dataset/{time_label}_Data/state',
                    f'/home/rebot801/LIuXin/ICCUB_ws/Dataset/{time_label}_Data/command',
                    f'/home/rebot801/LIuXin/ICCUB_ws/Dataset/{time_label}_Data/gripper',
                    f'/home/rebot801/LIuXin/ICCUB_ws/Dataset/{time_label}_Data/camera']

    my_mkdir(file_names=file_names, time_label = time_label)

    status = int(input("输入2开始收集数据"))

    if status == 2:

        main(file_names)

