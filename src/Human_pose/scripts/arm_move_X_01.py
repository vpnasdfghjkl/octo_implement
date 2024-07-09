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

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('demo_test') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 控制进入到手臂规划模式
    # robot_instance.set_robot_arm_ctl_mode(True)
    # joint_positions = [40,0,0,0,0,0,0,0,0,0,0,0,0,40]

    joint_positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # joint_positions = [-12, 8, -26, -100, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    robot_instance.set_arm_traj_position(joint_positions)
    time.sleep(2)

    # joint_positions = [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0]

    # robot_instance.set_arm_traj_position(joint_positions)
    # time.sleep(2)
    # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    #
    # joint_positions = [0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # # joint_positions = [-12, 8, -26, -100, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #
    # robot_instance.set_arm_traj_position(joint_positions)

