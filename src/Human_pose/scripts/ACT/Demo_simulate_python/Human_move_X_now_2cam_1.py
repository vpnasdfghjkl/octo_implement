#!/usr/bin/env python3
"""Kuavo机器人基础手臂位置控制案例展示

这个案例演示了如何使用ACT进行后续机器人动作的执行

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 发布手臂关节数据，控制手臂运动到指定位置
- 关闭手臂控制
"""
import copy

import numpy as np
import rospy
import time
import math
import cv2
import os

from typing import List
from kuavoRobotSDK import kuavo
from dynamic_biped.msg import robotArmInfo
from dynamic_biped.msg import robotArmQVVD
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from sensor_msgs.msg import JointState
from Demo_log import DemoLog  # 在现有的程序中加入log

from camera_init_copy import camera_init, camera_main, camera_close

from Eval_modify_1 import main_ACT


"""先挪到初始位置"""



time.sleep(3)

pipeline1, pipeline2 = camera_init()

logger = DemoLog().log()  # 形参，定义log的函数

status = False
End_status = False  # 控制手部开的标志

image_path = f"/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/ACT/Demo_simulate_python/{time.time()}"

if not os.path.isdir(image_path):
    os.makedirs(image_path)

"""功能函数"""


class Human_ACT(object):

    def __init__(self, state: int, count: int):

        logger.info("{}".format("程序正常启动，开始人形机器人的模仿学习操作"))

        self.joint_position_right = [0] * state
        self.joint_position = [0] * count
        self.robot_instance = kuavo("3_7_kuavo")
        self.image1 = None
        self.image2 = None
        self.num = 1

    def rad_to_angle(self, rad_list: list) -> list:
        """弧度转变为角度"""

        angle_list = [0 for _ in range(len(rad_list))]

        for i, rad in enumerate(rad_list):
            angle_list[i] = rad / math.pi * 180.0

        return angle_list

    def get_RobotState_callback(self, msg):
        """获取机器人本体的相关数据信息"""

        global status

        state_Q = self.rad_to_angle(msg.q)
        # print(state_Q,"&" * 17)

        if status is True:

            if self.num == 1:

                self.image1 = camera_main(pipeline=pipeline1)

                # self.joint_position[7:14] = [1.8961101737267163, -16.171927593043982, -50.45180602562918, -46.157004427991644, -2.0398338756349528, -40.0, 7.07469416829466]
                self.joint_position[7:14] = [12.168476327643338, -11.03787950838337, -23.22371379652903, -61.53870333568624, -2.0456958017627915, -40.0, 7.2949723758556075]

            else:

                status = False

                # 从当前的视频流里边记录图片的数值
                self.image1 = copy.deepcopy(camera_main(pipeline=pipeline1))

                self.image2 = copy.deepcopy(camera_main(pipeline=pipeline2))

                # 保存图片的函数

                image_time = f"{time.time()}.png"
                image_name = os.path.join(image_path, image_time)

                cv2.imwrite(image_name, self.image1)

                # 将左胳膊的值记录下来作为运行时候的值
                self.joint_position[0:7] = copy.deepcopy(state_Q[0:7])

                # 用右胳膊做预测，所以提取的值是拿到的7到14
                self.joint_position_right[0:7] = copy.deepcopy(state_Q[7:14])

                logger.debug("机器人实际运动到达的点位为:{}".format(self.joint_position))

                # logger.info("{}".format("拿取位姿正常并且正常执行ACT的预测"))

    def pub_kuavo_arm_traj(self, joint_position: list):

        try:
            arm_traj_msg = JointState()

            joint_position = self.judgement_joint(joint_position=joint_position)

            arm_traj_msg.position = joint_position

            self.arm_traj_pub.publish(arm_traj_msg)

            return True

        except Exception as error:

            logger.error("Error:{}".format(error))

            return False

    def judgement_joint(self, joint_position: list):

        # min_lim = [-180, -200, -135, -200, -135, -60, -80, -180, -180, -180, -200, -180, -40, -40]  # 遥操作中读取
        min_lim = [-180, -5, -90, -90, -90, -90, -90, -180, -135, -90, -90, -90, -90, -90]  # 手动估计测量

        # max_lim = [30, 135, 135, 0, 135, 60, 80, 180, 180, 180, 0, 180, 40, 40]  # 遥操作中读取
        max_lim = [30, 135, 90, 0, 90, 90, 90, 30, 5, 90, 0, 90, 90, 90]  # 手动估计测量

        for i in range(len(joint_position)):

            if joint_position[i] < min_lim[i]:
                joint_position[i] = min_lim[i]

            if joint_position[i] > max_lim[i]:
                joint_position[i] = max_lim[i]

        return joint_position

    def call_control_end_hand_service(self, left_hand_position: List[float], right_hand_position: List[float]):

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

    def My_ACT(self, joint_right_position: List[float] = None, image=None):

        # 将现有结构再次分为try结构，捕获异常信息

        global status
        global End_status

        try:

            status = True

            if self.image1 is not None:

                if self.num == 1:

                    logger.info("{}".format("当前是初始化函数，只执行一次"))

                    res = self.pub_kuavo_arm_traj(self.joint_position)
                    time.sleep(0.5)
                    self.num += 1

                else:

                    camera_image1 = self.image1
                    camera_image2 = np.zeros((360, 640, 3))

                    right_list = self.joint_position_right

                    target_right_list = main_ACT(cam_followed=camera_image1, current_joints=right_list, End_status=End_status,cam_fixed=camera_image2)

                    # 14维度，将预测出来的7个维度拼到joint_position上

                    self.joint_position[7:14] = copy.deepcopy(target_right_list[0:7])

                    logger.info("模型预测得到轴的位姿和夹爪状态码分别为:{},{}".format(self.joint_position,
                                                                                      target_right_list[7]))

                    res = self.pub_kuavo_arm_traj(self.joint_position)

                    # 第一阶段
                    if target_right_list[7] < 0.4 and End_status is False:

                        self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 70, 20, 20, 20, 20])

                        # self.joint_position_right[7] = target_right_list[7]
                        self.joint_position_right[7] = 0

                    # 合爪的阶段
                    elif target_right_list[7] > 0.7 and End_status is False:

                        self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 30, 80, 80, 80, 80])

                        self.joint_position_right[7] = 1

                        if target_right_list[7] > 0.9:
                            End_status = True

                    # 开爪的阶段
                    elif target_right_list[7] < 0.9 and End_status is True:

                        self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 70, 20, 20, 20, 20])

                        self.joint_position_right[7] = 0

                        if target_right_list[7] < 0.6:
                            End_status = False

                    else:
                        self.joint_position_right[7] = target_right_list[7]

        except Exception as error:
            logger.error("Error:{}".format(error))

    def my_main(self):

        rospy.init_node('demo_test')

        self.arm_traj_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        self.arm_traj_pub_ = rospy.Subscriber("/robot_arm_q_v_tau", robotArmInfo, self.get_RobotState_callback)
        # self.arm_traj_pub_ = rospy.Subscriber("/robot_arm_q_v_tau", robotArmQVVD, self.get_RobotState_callback)

        while not rospy.is_shutdown():
            self.My_ACT()

            time.sleep(0.1)

        camera_close(pipeline1, pipeline2)

if __name__ == "__main__":
    # 初始化节点
    kuavo = Human_ACT(state=8, count=14)

    kuavo.my_main()
