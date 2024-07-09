#!/usr/bin/env python
"""
  机器人位姿获取
    --主要获取遥操作机械臂以及机器人本机数据集

"""

import math
import rospy
import time    
from dynamic_biped.msg import robotArmInfo        # 机械臂关节的角度和信息
import os
from kuavoRobotSDK import kuavo
import sys, tty, termios
import time
from pynput import keyboard

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

end_eff_state = [0,0]
end_eff_comm  = [0,0]

robot_instance = kuavo("3_7_kuavo")



class MyKuavo(object):

    """ 初始化函数 """

    def __init__(self,queue_size,datalen,robot_instance=None):

        rospy.init_node("Human_subscrib_node")

        self.latest_Q = None  # 人形机器人机械臂的位姿
        self.latest_V = None  # 人形机器人机械臂的位姿
        self.latest_JointPosition_QV = None  # 遥操作机械臂的位姿
        self.queue_size = queue_size

        self.count = 1

        self.datalen = datalen

        self.filteredAngle = [0]* datalen
        
        self.robot_instance = robot_instance


    """ ROS函数 """
    # state
    def get_RobotState_callback(self, msg):

        self.latest_Q = msg.q

        self.latest_Q = self.filter_angle(self.latest_Q)

        print(str(self.latest_Q),end = "\n\n")

        self.latest_V = msg.v
        self.count = self.unpickle(state_save_path="/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/state",count=self.count)

  
    """ 功能函数 """ 
    def on_press(self, key):

        # print("__________________",type(key.char))

        if key ==  '2':

            print("检测到了键盘输入，开始进行夹爪的规划", end="\n\n")


            try:        

                print("输入左手臂的值, 1代表关, 0代表开",end="\n\t")

                robot_l = int(input())

                # robot_l = int(self.getch())

                print("输入右手臂的值, 1代表关, 0代表开",end="\n\n")

                # robot_r = int(self.getch())
                robot_r = int(input())
                self.setEndEffector(robot_l=robot_l, robot_r=robot_r)

            # robot_l = int(sys.stdin.read(1))

            # time.sleep(2)

            # robot_l = int(input("请输入左夹爪的开合值,1代表的关,0代表开"))
            # # time.sleep(5)
            # robot_r = int(input("请输入右夹爪的开合值,1代表的关,0代表开"))
            
            except AttributeError:
                pass

    # def getch():
    #     try:
    #         tty.setraw(sys.stdin.fileno())
    #         ch = sys.stdin.read(1)
    #     finally:
    #         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    #     return ch

    

    def filter_angle(self, presentAngle):

        for i in range(self.datalen):
            #self.filteredAngle[i] = self.alpha * presentAngle[i] + (1 - self.alpha) * self.filteredAngle[i]
            self.filteredAngle[i] = math.degrees(presentAngle[i])
        # print(self.filteredAngle)
        return self.filteredAngle
    

    def unpickle(self,count,state_save_path):


        if not os.path.isdir(state_save_path):
            os.makedirs(state_save_path)

        # print(state_save_path)

        # label = self.latest_Q + end_eff_state
        label = self.latest_Q

        with open(f'{state_save_path}/state.txt',"a") as txt:

            if count == 1:
                txt.write("当前是state" + "\n")

            txt.write(str([time.time()]) + " " + str(label) + "\n")

        count = count + 1
    
        return count
        
    def openoffset(self):

        self._RobotState_sub = rospy.Subscriber("/robot_arm_q_v_tau", robotArmInfo, self.get_RobotState_callback)
        listen = keyboard.Listener(on_press=self.on_press)
        # listen.start()
        # listen.join()

        try:
            rospy.spin()
        except:

            pass
    
    def setEndEffector(self,robot_l,robot_r):

        print("+++++++++++++++++++++++++++++++",robot_l,type(robot_l))

        global end_eff_state
        global end_eff_comm

        if robot_l == 0  and robot_r == 0:
            self.robot_instance.set_robot_joller_position(0, 0)
        elif robot_l == 0 and robot_r == 1:
            self.robot_instance.set_robot_joller_position(0, 255)
        elif robot_l == 1 and robot_r == 1:
            self.robot_instance.set_robot_joller_position(255, 255)
        elif robot_l == 1 and robot_r == 0:
            self.robot_instance.set_robot_joller_position(255, 0)
        else:
            print("输入范围有误")

        end_eff_state = [robot_l, robot_r]
        end_eff_comm = [robot_l, robot_r]



def main():

   
    rospy.logwarn("当前正常，开始接收信息")
    
    Kuavoo = MyKuavo(queue_size=10, datalen=14, robot_instance=robot_instance)
    Kuavoo.openoffset()



if __name__ =="__main__":

    print("按2键开始收集数据")

    if int(input()) == 2:

        main()

    

