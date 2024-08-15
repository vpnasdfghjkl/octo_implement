#!/usr/bin/env python3
"""Kuavo机器人基础手臂位置控制案例展示

这个案例演示了如何使用Kuavo机器人SDK控制机器人的手臂进行位置控制。

功能描述：
- 初始化ROS节点
- 初始化Kuavo机器人实例
- 将机器人手臂控制模式设置为位置规划模式
- 发布手臂关节数据，控制手臂运动到指定位置
- 等待一段时间
- 控制手臂移动到其他位置
- 手臂归中
- 关闭手臂控制
"""

import rospy
import time
from kuavoRobotSDK import kuavo

import socket
import pickle
import json  
import struct
import select

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


if __name__ == "__main__":

    fps = 25
    # 初始化节点
    rospy.init_node('demo_test') 

    # 初始化机器人
    robot_instance = kuavo("3_7_kuavo")

    # 控制进入到手臂规划模式
    # robot_instance.set_robot_arm_ctl_mode(True)

    socket.setdefaulttimeout(3600)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("192.168.1.232", 8088))

    a = input("按任意键开始进行操作")
    print("--------清空缓存中-------")
    clear_socket_buffer(server_socket)
    print("--------缓存清理完成-------")
    i = 0
    while True:
        data, addr = server_socket.recvfrom(1024)
        json_message = data.decode('utf-8')
        message = json.loads(json_message)
        data = message
        arm_data = data["data"]
        arm_state = data["state"]
        print(arm_data)
        robot_instance.set_arm_traj_position(arm_data)
        time.sleep(1/fps)
        # break

    #测试数据效果

    time.sleep(1/fps)

    # 关闭手臂控制
    # robot_instance.set_robot_arm_ctl_mode(False)

    # 关闭socket连接
    server_socket.close()
