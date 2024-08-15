#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Bulk Read Example      *********
#
#
# Available Dynamixel model on this example : MX or X series set to Protocol 1.0
# This example is tested with two Dynamixel MX-28, and an USB2DYNAMIXEL
# Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import rospy

import sensor_msgs.msg

class Publish():
    def __init__(self,arm_value,publisher):

        self.publish = publisher

        self.arm_value = arm_value



    def publish_arm_traj(self):
        """发布左手轨迹
        """
        # arm_min = [-180, -20, -135, -100, -135, -10, -10, -180, -180, -180, -180, -180, -5, -15]
        # arm_max = [  30, 135,  135,  100,  135,  10,  10,  180,  180,  180,  180,  180,  5,  15]

        arm_min = [-180, -20, -135, -100, -135, -10, -15, -180, -180, -180, -180, -180, -10, -15]
        arm_max = [30, 135, 135, 100, 135, 10, 15, 180, 180, 180, 180, 180, 10, 15]

        send_angle = self.arm_value
        publisher = self.publish
        

        joint_state = sensor_msgs.msg.JointState()
        positions  = [0 for _ in range(14)]

        rate = rospy.Rate(500)
        
        # 判断元素是否在范围内，并进行修正
        for i in range(len(send_angle)):
            if send_angle[i] < arm_min[i]:
                send_angle[i] = arm_min[i]
            elif send_angle[i] > arm_max[i]:
                send_angle[i] = arm_max[i]
        positions[0:14] = send_angle
        print("send_angle:",[round(x,1) for x in send_angle])
        joint_state.position = positions
        print("+" * 30,joint_state.position)
        
        publisher.publish(joint_state)
        rate.sleep()
    

if __name__ == "__main__":

    # 初始化ROS节点
    rospy.init_node('array_publisher', anonymous=True)

    # 创建一个发布者，指定发布的话题名称和消息类型

    arm_pub = rospy.Publisher('/kuavo_arm_traj', sensor_msgs.msg.JointState, queue_size=10) # kuavo_arm_traj

    rate = rospy.Rate(1)  # 1Hz

    pubb = Publish(arm_value=[-10,100,0,0,0,0,0,0,0,0,0,0,0,0],publisher=arm_pub)

    while not rospy.is_shutdown():


        pubb.publish_arm_traj()

        # 按照指定的频率休眠
        rate.sleep()
            
        



