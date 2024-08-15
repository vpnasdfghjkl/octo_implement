#!/usr/bin/env python

import rospy
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
import time


def call_control_end_hand_service(left_hand_position, right_hand_position, is_init=False):

    if is_init:
        rospy.init_node("gripper")

    hand_positions = controlEndHandRequest()
    hand_positions.left_hand_position = left_hand_position  # 左手位置
    hand_positions.right_hand_position = right_hand_position   # 右手位置


    try:
        rospy.wait_for_service('/control_end_hand')
        control_end_hand = rospy.ServiceProxy('/control_end_hand', controlEndHand)
        resp = control_end_hand(hand_positions)
        
        return resp.result
        
    except rospy.ROSException as e:
    
        rospy.logerr("Service call failed: %s" % e)
        return False


def main(statues):

    if statues == 1:
        
        left_hand_position = [0, 70, 20, 20, 20, 20]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 2:
        left_hand_position = [0, 30, 80, 80, 80, 80]
        right_hand_position = [0,0, 0, 0, 0,0]


    elif statues == 3:

        left_hand_position = [100,100, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 4:

        left_hand_position = [100,100, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    else:
        print("当前没有这个状态，请重新输入")

    success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
    
    if success:
        rospy.loginfo("Hand control service call successful.")
    else:
        rospy.loginfo("Hand control service call failed.")
    

if __name__ == '__main__':


    while not rospy.is_shutdown():
    
        state = int(input("请输入以下数字:1,左手打开,右手打开,2,左手打开,右手关闭,3,左手关闭,右手关闭,4,左手打开，右手关闭"))

        main(statues=state)

    


    
    
   
