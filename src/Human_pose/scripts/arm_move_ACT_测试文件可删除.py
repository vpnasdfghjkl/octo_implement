#!/usr/bin/env python3
"""Kuavo机器人基础手臂位置控制案例展示

这个案例演示了如何使用Kuavo机器人SDK控制机器人的手臂进行位置控制。

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 将机器人手臂控制模式设置为位置规划模式
- 发布手臂关节数据，控制手臂运动到指定位置
- 等待一段时间
- 控制手臂移动到其他位置
- 手臂归中
- 关闭手臂控制
"""

import rospy
import time
from kuavoRobotSDK import kuavo
import os
import numpy as np

from typing import List

from dynamic_biped.srv import controlEndHand, controlEndHandRequest



"""功能函数"""


def get_txt(Txt_path: str, num: int, count: int) -> list:

    array_list = []

    # 添加一个阈值，按照阈值进行文件的选择，并不是每一个都选
    if num % count == 0:

        with open(Txt_path, "r") as file:

            state_lines = file.readlines()

        state_line = state_lines[0:len(state_lines):5]

        state_line = [s.replace('[', '').replace(']', '') for s in state_line]

        for line in state_line:
            if line.strip():
                row = [float(value) for value in line.split(',')]
                array_list.append(np.array(row))

    return array_list

# def call_control_end_hand_service(left_hand_position: List[float], right_hand_position: List[float]):
#
#     """控制爪子开合的服务"""
#     hand_positions = controlEndHandRequest()
#     hand_positions.left_hand_position = left_hand_position  # 左手位置
#     hand_positions.right_hand_position = right_hand_position  # 右手位置
#
#     try:
#         rospy.wait_for_service('/control_end_hand')
#         control_end_hand = rospy.ServiceProxy('/control_end_hand', controlEndHand)
#         resp = control_end_hand(hand_positions)
#         return resp.result
#
#     except rospy.ROSException as e:
#
#         rospy.logerr("Service call failed: %s" % e)
#         return False
#

if __name__ == "__main__":

    # 初始化节点
    rospy.init_node('demo_test')

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    joint_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    src_path = "/home/rebot801/wangwei/dest"

    num = 1
    count = 50

    for file_path in os.listdir(src_path):

        data_time = os.path.basename(file_path).split('_')[0]

        txt_path = f'{src_path}/{file_path}/command/{data_time}_command.txt'

        array_list = get_txt(Txt_path=txt_path, num=num, count=count)

        if not array_list:
            num = num + 1
            continue

        else:

            for nums in array_list:

                joint_positions[7:14] = nums[0:7]

                print(joint_positions, end="\n\n")

                robot_instance.set_arm_traj_position(joint_positions)
                #
                # if nums[7] == 1:
                #
                #     time.sleep(1.5)
                #     call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 30, 80, 80, 80, 80])
                #
                # else:
                #     call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 70, 20, 20, 20, 20])

                time.sleep(0.25)

                num = num + 1
        print("^^^^^^^^^^^^^^^^^^")
print("当前程序执行结束，退出程序")
