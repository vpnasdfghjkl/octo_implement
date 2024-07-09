#!/usr/bin/env python
"""
  机器人位姿获取
    --主要获取遥操作机械臂数据集

"""


import rospy
import time    
import os
from sensor_msgs.msg import JointState


class MyKuavo(object):

    """ 初始化函数 """

    def __init__(self,queue_size,robot_instance=None):
        

        rospy.init_node("Human_subscrib_node_1")
        
        self.latest_Q = None  # 人形机器人机械臂的位姿
        self.latest_V = None  # 人形机器人机械臂的位姿
        self.latest_JointPosition_QV = None  # 遥操作机械臂的位姿
        self.queue_size = queue_size

        self.count_num = 1

        self.robot_instance = robot_instance


    """ ROS函数 """

    # command
    def get_JointPosition_callback(self, msg):
        

        self.latest_JointPosition = msg.position

        print(self.latest_JointPosition,end = "\n\n")

        self.count_num = self.unpickle(comman_save_path="/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/command", count=self.count_num)

  
    """ 功能函数 """ 

    def unpickle(self, count, comman_save_path=None, num=None):


        if not os.path.isdir(comman_save_path):

            os.makedirs(comman_save_path)


        label_list = list(self.latest_JointPosition)


        with open(f'{comman_save_path}/command.txt',"a") as txt1:

            if count == 1:

                txt1.write("当前是command" + "\n")

            txt1.write(str([time.time()]) + " " + str(label_list) +"\n")
            count = count + 1
    
        return count
        
    def openoffset(self):

        self.JointState_sub = rospy.Subscriber("/kuavo_arm_traj", JointState, self.get_JointPosition_callback)

        try:
            rospy.spin()
        except:
            pass
    


def main():

    rospy.logwarn("当前正常，开始接收信息")
    
    
    Kuavoo = MyKuavo(queue_size=10)
    Kuavoo.openoffset()



if __name__ =="__main__":

    print("按1键开始收集数据")

    if int(input()) == 1:
        

        main()


    

