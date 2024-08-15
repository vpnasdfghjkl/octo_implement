import rospy
import time

from kuavoRobotSDK import kuavo

from dynamic_biped.srv import controlEndHand, controlEndHandRequest


"""先挪到初始位置"""


def go_to_start(robot_instance):

    with open("/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/traj_/traj_added2.txt", 'r') as f:
        lines = f.readlines()

        start_traj = []
        for line in lines:
            import re
            pattern = r'\[(.*?)\]'
            matchs = re.findall(pattern, line)
            array_str = matchs[1]
            line = array_str[1:-1]
            line = line.split(", ")
            start_traj.append([float(x) for x in line])

    for step in start_traj:

        robot_instance.set_arm_traj_position(step)
        time.sleep(0.015)

rospy.init_node('demo_test')

# 初始化机器人
robot_instance = kuavo("3_7_kuavo")
go_to_start(robot_instance)