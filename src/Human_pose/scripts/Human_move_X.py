#!/usr/bin/env python3
"""Kuavo机器人基础手臂位置控制案例展示

这个案例演示了如何使用ACT进行后续机器人动作的执行

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 发布手臂关节数据，控制手臂运动到指定位置
- 关闭手臂控制
"""
import numpy as np
import rospy
import time
import math

from typing import List
from kuavoRobotSDK import kuavo
from dynamic_biped.msg import robotArmInfo
# from dynamic_biped.msg import robotArmQVVD
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from sensor_msgs.msg import JointState
from camera_init1 import camera_init, camera_main, camera_close
from ACT.Demo_simulate_python.Eval_modify_1 import main_ACT
pipeline = camera_init()

status = False

"""功能函数"""

class Human_ACT(object):

    def __init__(self, state:int, count:int ):

        print("当前开始人形机器人的模仿学习操作")

        self.joint_position_right = [0] * state
        self.joint_position = [0] * count
        self.robot_instance = kuavo("3_7_kuavo")

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
        # print(state_Q)
        # state_Q = self.rad_to_angle(msg.Q)

        if status is True:

            # 用右胳膊做预测，所以提取的值是拿到的8到14
            self.joint_position_right[0: 7] = state_Q[7:14]

            # 从当前的视频流里边记录图片的数值
            self.depth, self.image = camera_main(pipeline=pipeline)

            # 将左胳膊的值记录下来作为运行时候的值
            self.joint_position[0:7] = state_Q[0:7]

            status = False

            # print(str(self.joint_position_right), end = "\n\n")
            print("执行一次之后阻塞住，然后去进行ACT的运动，此时joint_position,joint_position_right值是固定好的")

    def pub_kuavo_arm_traj(self, joint_position: list):

        try:
            arm_traj_msg = JointState()
            arm_traj_msg.position = joint_position
            self.arm_traj_pub.publish(arm_traj_msg)

            return True

        except Exception as error:
            print(error)
            return False

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
        status = True

        camera_image = self.image
        right_list = self.joint_position_right

        target_right_list = main_ACT(cam_followed=camera_image, current_joints=right_list)

        # 将本次预测出来的夹爪值赋值给下一个的夹爪值
        self.joint_position_right[7] = target_right_list[7]

        # 14维度，将预测出来的7个维度拼到joint_position上
        self.joint_position[7:14] = target_right_list[0:7]

        res = self.pub_kuavo_arm_traj(self.joint_position)

        # if res is True and np.around(target_right_list[8]) == 1:
        #     time.sleep(1.5)
        #     self.control_end_hand(hand_status=np.around(target_right_list[8]))
        #
        # else:
        #
        #     self.control_end_hand(hand_status=np.around(target_right_list[8]))

    def my_main(self):

        rospy.init_node('demo_test')

        self.arm_traj_pub  = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        self.arm_traj_pub_ = rospy.Subscriber("/robot_arm_q_v_tau", robotArmInfo, self.get_RobotState_callback)
        # self.arm_traj_sub = rospy.Subscriber("/robot_arm_q_v_tau", robotArmQVVD, self.get_RobotState_callback)

        num_count = 1

        while 1:

            self.My_ACT()

            if num_count == 30:
                break

            num_count = num_count + 1

        camera_close(pipeline=pipeline)



if __name__ == "__main__":
    # 初始化节点
    kuavo = Human_ACT(state=8, count=14)

    kuavo.my_main()


