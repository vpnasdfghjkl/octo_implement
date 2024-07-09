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
    

# 执行动作，有四种，1：（0，0），2：（1，0），3：（1，0），4：（1，1）
def four_positions(statues):
    if statues == 1:

        left_hand_position = [0, 0, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 2:
        left_hand_position = [0, 0, 100, 100, 100, 100]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 3:

        left_hand_position = [0, 0, 0, 0, 0, 0]
        right_hand_position = [0, 0, 100, 100, 100, 100]

    elif statues == 4:

        left_hand_position = [0, 0, 100, 100, 100, 100]
        right_hand_position = [0, 0, 100, 100, 100, 100]

    else:
        print("当前没有这个状态，请重新输入")

    return left_hand_position, right_hand_position


def circle_act(start_label:int=0, end_label:int=None):

    i = start_label
    if end_label == None:
        while(1):
            state = i + 1
            if state > 4:
                state = 1
            success = call_control_end_hand_service(four_positions(statues=state)[0], four_positions(statues=state)[1], is_init=True)
            if success:
                rospy.loginfo("Hand control service call successful.")
            else:
                rospy.loginfo("Hand control service call failed.")
            time.sleep(2)
    elif type(end_label) is int:
        for _ in range(end_label):
            state = i + 1
            if state > 4:
                state = 1
            success = call_control_end_hand_service(four_positions(statues=state)[0], four_positions(statues=state)[1], is_init=True)
            if success:
                rospy.loginfo("Hand control service call successful.")
            else:
                rospy.loginfo("Hand control service call failed.")
            time.sleep(2)


def main(statues):

    if statues == 1:

        left_hand_position = [0, 0, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 2:
        left_hand_position = [100,50, 0, 0, 0, 100]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 3:

        left_hand_position = [100,100, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues == 4:

        left_hand_position = [100,100, 0, 0, 0, 0]
        right_hand_position = [0, 0, 0, 0, 0, 0]

    elif statues > 4:
        circle_act(start_label=0, end_label=int(statues))

    else:
        print("当前没有这个状态，请重新输入")

    success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
    
    if success:
        rospy.loginfo("Hand control service call successful.")
    else:
        rospy.loginfo("Hand control service call failed.")



# def main(statues):

#     if statues == 1:

#         left_hand_position = [0, 0, 0, 0, 0, 0]
#         right_hand_position = [0, 0, 0, 0, 0, 0]

#     elif statues == 2:
#         left_hand_position = [100,50, 0, 0, 0, 100]
#         right_hand_position = [0, 0, 0, 0, 0, 0]

#     elif statues == 3:

#         left_hand_position = [100,100, 0, 0, 0, 0]
#         right_hand_position = [0, 0, 0, 0, 0, 0]

#     elif statues == 4:

#         left_hand_position = [100,100, 0, 0, 0, 0]
#         right_hand_position = [0, 0, 0, 0, 0, 0]

#     else:
#         print("当前没有这个状态，请重新输入")

#     success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
    
#     if success:
#         rospy.loginfo("Hand control service call successful.")
#     else:
#         rospy.loginfo("Hand control service call failed.")
    

if __name__ == '__main__':
    
    state = int(input("请输入以下数字:1,左手打开,右手打开,2,左手关闭,右手打开,3,左手关闭,右手关闭,4,左手打开，右手关闭"))
    main(statues=state)

    


    
    
   