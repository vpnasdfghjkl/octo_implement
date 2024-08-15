import time
from threading import Thread
import time
import socket
import json
import sys
import signal
import rospy
import time
from kuavoRobotSDK import kuavo
from dynamic_biped.srv import controlEndHand, controlEndHandRequest

def clear_socket_buffer(sock):
    """清空套接字的接收缓存"""
    try:
        sock.setblocking(0)  # 设置为非阻塞模式
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                print(f"清空缓冲区的数据: {data.decode('utf-8')}")
                if not data:
                    break
            except BlockingIOError:
                break
    except OSError as e:
        print(f"OSError: {e}")
    finally:
        sock.setblocking(1)  # 恢复为阻塞模式


def keep_receiving():
    global data_buffer
    global server_socket
    print("--------清空缓存中-------")
    clear_socket_buffer(server_socket)
    print("--------缓存清理完成-------")
    max_len = 10
    while not stop_event.set():
        data, addr = server_socket.recvfrom(648)
        json_message = data.decode('utf-8')
        data = json.loads(json_message)
        data_buffer.append(data)
        if len(data_buffer) > max_len:
            data_buffer = data_buffer[-max_len:]
        # print(data_buffer[-1])


def get_latest_data():
    rospy.init_node('demo_test') 
    robot_instance = kuavo("3_7_kuavo")
    robot_instance.set_robot_arm_ctl_mode(True)
    global data_buffer
    while True:
        data = data_buffer[-1]
        arm_pose = data["arm"]
        finger_pose = data["finger"]
        left_hand_position = finger_pose[:6]
        left_hand_position = [int(i) for i in left_hand_position]
        right_hand_position = finger_pose[6:]
        right_hand_position = [int(i) for i in right_hand_position]
        robot_instance.set_arm_traj_position(arm_pose)
        success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
        time.sleep(1/30)
        print(data)
        break

"""基于手部控制的topic"""
def call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True):

    if not is_init:
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


# data pattern  {'data': [-70, 80, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], 'arm': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}

if __name__ == '__main__':
    import threading
    global server_socket
    global data_buffer

    stop_event = threading.Event()

    fps = 15

    rospy.init_node('demo_test') 
    # rospy.init_node("gripper")
    robot_instance = kuavo("3_7_kuavo")

    socket.setdefaulttimeout(3600)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("192.168.1.232", 8088))
    data_buffer = []

    t1 = Thread(target=keep_receiving, args=())
    # t2 = Thread(target=get_latest_data, args=())
    a = input("按任意键开始进行操作")

    t1.start()
    print("数据接受开始")
    while len(data_buffer) < 10:
        continue
    print("初始数据收集完成， 开始控制")


    # robot_instance.set_robot_arm_ctl_mode(False)

    while True:

        data = data_buffer[-1]
        arm_pose = data["arm"]
        finger_pose = data["finger"]
        left_hand_position = finger_pose[:6]
        left_hand_position = [int(i) for i in left_hand_position]
        right_hand_position = finger_pose[6:]
        right_hand_position = [int(i) for i in right_hand_position]
        robot_instance.set_arm_traj_position(arm_pose)
        success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
        time.sleep(1/25)
        print(data)
        # break

    stop_event.set()
    t1.join()
    exit()




