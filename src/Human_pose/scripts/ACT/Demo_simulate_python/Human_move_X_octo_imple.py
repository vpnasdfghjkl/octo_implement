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

from typing import List
from kuavoRobotSDK import kuavo
from dynamic_biped.msg import robotArmInfo
from dynamic_biped.msg import robotArmQVVD
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from sensor_msgs.msg import JointState
from camera_init import camera_init, camera_main, camera_close
time.sleep(5)

from Eval_modify_1 import main_ACT

pipeline = camera_init()

status = False


"""功能函数"""

class Human_ACT(object):

    def __init__(self, state:int, count:int ):

        print("当前开始人形机器人的模仿学习操作")

        self.joint_position_right = [0] * state
        self.joint_position = [0] * count
        self.robot_instance = kuavo("3_7_kuavo")
        self.image = None
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

        if status is True:

            if self.num == 1:

                self.depth, self.image = camera_main(pipeline=pipeline)

                # 到达某个初始位置

                self.joint_position[7:14] = [1.5644865540106576, -5.433135139667643, -39.900632294200555, 0.0, -12.607800276665358, 3.0374787484520107, 4.324829964010405]

            else:

                status = False

                # 从当前的视频流里边记录图片的数值
                self.depth, self.image = copy.deepcopy(camera_main(pipeline=pipeline))

                # 将左胳膊的值记录下来作为运行时候的值
                self.joint_position[0:7] = copy.deepcopy(state_Q[0:7])

                # 用右胳膊做预测，所以提取的值是拿到的7到14
                self.joint_position_right[0:7] = copy.deepcopy(state_Q[7:14])

                print("执行一次之后阻塞住，然后去进行ACT的运动，此时joint_position,joint_position_right值是固定好的")


    def pub_kuavo_arm_traj(self, joint_position: list):

        try:
            arm_traj_msg = JointState()

            joint_position = self.judgement_joint(joint_position = joint_position)

            arm_traj_msg.position = joint_position

            # print(arm_traj_msg.position, "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")

            self.arm_traj_pub.publish(arm_traj_msg)
            return True

        except Exception as error:
            print(error)
            return False


    def judgement_joint(self, joint_position:list):

        # min_lim = [-180, -200, -135, -200, -135, -60, -80, -180, -180, -180, -200, -180, -40, -40]  # 遥操作中读取
        min_lim = [-180, -5, -90, -90, -90, -90, -90, -180, -135, -90, -90, -90, -90, -90]  # 手动估计测量

        # max_lim = [30, 135, 135, 0, 135, 60, 80, 180, 180, 180, 0, 180, 40, 40]  # 遥操作中读取
        max_lim = [30, 135, 90, 0, 90, 90, 90,  30, 5,  90, 0, 90, 90, 90]  # 手动估计测量

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

    def control_end_hand(self, hand_status):
        """控制夹爪开合的函数，0表示关，1表示开"""

        if hand_status == 0:
            left_hand_position = [0, 0, 0, 0, 0, 0]
            right_hand_position = [0, 70, 20, 20, 20, 20]

        elif hand_status == 1:
            left_hand_position = [0, 0, 0, 0, 0, 0]
            right_hand_position = [0, 30, 80, 80, 80, 80]

        else:
            print("输入错误，请重新输入")

        success = self.call_control_end_hand_service(left_hand_position, right_hand_position)

        if success:
            rospy.loginfo("Hand control service call successful.")
        else:
            rospy.loginfo("Hand control service call failed.")

    def My_ACT(self, joint_right_position: List[float] = None, image=None):

        global status
        # global num

        status = True

        if self.image is not None:

            if self.num == 1:

                print("当前是初始化函数，只执行一次")
                res = self.pub_kuavo_arm_traj(self.joint_position)
                time.sleep(0.5)
                self.num += 1

            else:

                camera_image = self.image

                right_list = self.joint_position_right

                #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", right_list)

                target_right_list = main_ACT(cam_followed=camera_image, current_joints=right_list)

                # 将本次预测出来的夹爪值赋值给下一个的夹爪值
                self.joint_position_right[7] = np.around(target_right_list[7])

                print( target_right_list[7],"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                

                # 14维度，将预测出来的7个维度拼到joint_position上
                self.joint_position[7:14] = copy.deepcopy(target_right_list[0:7])
                print(self.joint_position,"**************************************************")

                res = self.pub_kuavo_arm_traj(self.joint_position)
                # if np.around(target_right_list[7]) == 1.0 and self.num > 50:
                # if target_right_list[7] > 1:

                #     time.sleep(0.5)
                #     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 30, 80, 80, 80, 80])

                # else:
                #     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0], right_hand_position=[0, 70, 20, 20, 20, 20])

                self.num = self.num + 1


    def my_main(self):
        
        rospy.init_node('demo_test')

        self.arm_traj_pub  = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        self.arm_traj_pub_ = rospy.Subscriber("/robot_arm_q_v_tau", robotArmInfo, self.get_RobotState_callback)
        # self.arm_traj_pub_ = rospy.Subscriber("/robot_arm_q_v_tau", robotArmQVVD, self.get_RobotState_callback)


        while not rospy.is_shutdown():

            self.My_ACT()
            time.sleep(0.5)

        camera_close(pipeline=pipeline)



if __name__ == "__main__":
    # 初始化节点
    kuavo = Human_ACT(state=8, count=14)

    kuavo.my_main()


