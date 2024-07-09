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
import os
import re
from kuavoRobotSDK import kuavo
from typing import List

from dynamic_biped.srv import controlEndHand, controlEndHandRequest
import numpy as np

"""功能函数"""


def call_control_end_hand_service(left_hand_position: List[float], right_hand_position: List[float]):
    """控制爪子开合的服务"""
    hand_positions = controlEndHandRequest()
    hand_positions.left_hand_position = left_hand_position  # 左手位置
    hand_positions.right_hand_position = right_hand_position  # 右手位置

    try:
        rospy.wait_for_service('/control_end_hand')
        control_end_hand = rospy.ServiceProxy('/control_end_hand', controlEndHand)
        resp = control_end_hand(hand_positions)
        return resp.result

    except rospy.ROSException as e:

        rospy.logerr("Service call failed: %s" % e)
        return False



if __name__ == "__main__":

    # 初始化节点
    rospy.init_node('demo_test')

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 第一个测试点位，出现了机器人的手
    time.sleep(5)

    src_path = "/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/result/output"
    joint_positions = [0] * 14

    for file_name in os.listdir(src_path):
        txt_path = os.path.join(src_path, file_name)

        print(file_name)

        dell_txt = "action_list"
        new_txt = []
        array_list = []

        with open(txt_path, "r") as file:
            txt = file.readlines()

        state_line = [s.replace('[', '').replace(']', '') for s in txt]


        for line in state_line:

            if line.strip():
                row = [float(value) for value in line.split(",")]
                array_list.append(np.array(row))

        n = 1

        for nums in array_list:

            joint_positions[7:14] = nums[0:7]

            robot_instance.set_arm_traj_position(joint_positions)

            if np.around(nums[7]) == 1.0:

                time.sleep(0.3)

                call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 30, 80, 80, 80, 80])
            else:
                call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 70, 20, 20, 20, 20])


            time.sleep(0.3)


        time.sleep(2.5)
        joint_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        robot_instance.set_arm_traj_position(joint_positions)

    print("当前程序执行结束，退出程序")
