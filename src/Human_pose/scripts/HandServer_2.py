import sys
import os
sys.path.append(os.getcwd())
import math
import socket
import json
import time
import threading
import torch
from angular import *
import quaternion
import numpy as np
import rospy
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from math import fabs

def build_rot(theta, rotation_axis):
    w = np.cos(theta * np.pi / 360)
    s = np.sin(theta * np.pi / 360)
    x = s * rotation_axis[0]
    y = s * rotation_axis[1]
    z = s * rotation_axis[2]

    q = quaternion.from_float_array([w, x, y, z])
    q = torch.Tensor([q.w, q.x, q.y, q.z]).float()
    rot = quaternion_to_rotation_matrix(q)

    return rot

def quaterion2degree(q):
    roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])) * 180 / math.pi
    pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1])) * 180 / math.pi
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) * 180 / math.pi
    return roll, pitch, yaw

# 定义一个函数来处理接收到的数据
# def process_hand_data(hand_data):
#     # 打印基本信息
#     print(f"DeviceID: {hand_data['DeviceID']}")
#     print(f"Frame: {hand_data['Frame']}")
#     print(f"DeviceName: {hand_data['DeviceName']}")
#     print(f"CalibrationStatus: {hand_data['CalibrationStatus']}")
#     print(f"Battery: {hand_data['Battery']}")
#
#     # 打印骨骼数据
#     print("Bones data:")
#     for bone_data in hand_data['Bones']:
#         print(bone_data)
#
# # 创建UDP套接字
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#
# # 绑定到本地地址和端口
# server_address = ('', 12345)  # 空字符串表示接受所有地址，12345是端口号
# sock.bind(server_address)
#
# print("UDP server up and listening on port:", server_address[1])
#
# while True:=
#     # 接收数据
#     data, addr = sock.recvfrom(4096)  # 缓冲区大小可根据需要调整
#     # 解码JSON格式的数据
#     hand_data = json.loads(data.decode('utf-8'))
#
#     # 处理接收到的手部数据
#     process_hand_data(hand_data)
#
#     # 打印客户端地址
#     print(f"Received data from {addr}")
#
# # 关闭套接字
# sock.close()

class HandServer_2:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 绑定到本地地址和端口
        server_address = ('192.168.1.232', 5055)  # 空字符串表示接受所有地址，12345是端口号
        self.sock.bind(server_address)
        # 打印
        print("UDP server up and listening on port:", server_address[1])
        self.save_one_data()

    def set_cloth_imu(self, l_cloth_imu, r_cloth_imu, set_init_cloth=False):
        """设置衣服imu数据"""
        self.l_cloth_imu= l_cloth_imu
        self.r_cloth_imu = r_cloth_imu
        if set_init_cloth:
            self.init_cloth_imu = [l_cloth_imu, r_cloth_imu]
    
    def get_hand_init2cloth_init(self):
        """获取手部初始相对衣服初始的旋转矩阵"""
        z_180_oris = build_rot(theta=-180, rotation_axis=[0, 0, 1])
        l_hand_init_oris = self.init_imu_data[0]
        r_hand_init_oris = self.init_imu_data[1]
        l_cloth_init_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[0]).unsqueeze(0).float())
        l_cloth_init_oris = z_180_oris.matmul(l_cloth_init_oris)
        r_cloth_init_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[1]).unsqueeze(0).float())
        r_cloth_init_oris = z_180_oris.matmul(r_cloth_init_oris)
        l_hand_init2cloth_init = l_cloth_init_oris.transpose(-2, -1).matmul(l_hand_init_oris)
        r_hand_init2cloth_init = r_cloth_init_oris.transpose(-2, -1).matmul(r_hand_init_oris)
        return [l_hand_init2cloth_init, r_hand_init2cloth_init]


    def get_relative_cloth_imu(self):
        """计算衣服imu相对初始的旋转矩阵"""
        z_180_oris = build_rot(theta=-180, rotation_axis=[0, 0, 1])
        l_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.l_cloth_imu).unsqueeze(0))
        l_cloth_oris = z_180_oris.matmul(l_cloth_oris)
        r_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.r_cloth_imu).unsqueeze(0))
        r_cloth_oris = z_180_oris.matmul(r_cloth_oris)
        l_init_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[0]).unsqueeze(0).float())
        l_init_cloth_oris = z_180_oris.matmul(l_init_cloth_oris)
        r_init_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[1]).unsqueeze(0).float())
        r_init_cloth_oris = z_180_oris.matmul(r_init_cloth_oris)
        l_relative_cloth_oris = l_init_cloth_oris.transpose(-2, -1).matmul(l_cloth_oris)
        r_relative_cloth_oris = r_init_cloth_oris.transpose(-2, -1).matmul(r_cloth_oris)
        return l_relative_cloth_oris, r_relative_cloth_oris
    
    def get_hand_oris2cloth_oris(self):
        """获取手部相对衣服的旋转"""
        l_hand_init2cloth_init, r_hand_init2cloth_init = self.get_hand_init2cloth_init()
        l_cloth_oris, r_cloth_oris = self.get_relative_cloth_imu()
        l_hand_oris = self.get_imu_data("left")
        r_hand_oris = self.get_imu_data("right")
        l_hand_oris_relative_cloth = l_hand_init2cloth_init.transpose(-2, -1).matmul(l_hand_oris)
        l_hand_oris_relative_cloth = l_cloth_oris.transpose(-2, -1).matmul(l_hand_oris_relative_cloth)
        r_hand_oris_relative_cloth = r_hand_init2cloth_init.transpose(-2, -1).matmul(r_hand_oris)
        r_hand_oris_relative_cloth = r_cloth_oris.transpose(-2, -1).matmul(r_hand_oris_relative_cloth)
        l_hand_oris_relative_cloth_euler = radian_to_degree(rotation_matrix_to_euler_angle(l_hand_oris_relative_cloth, seq="xzy"))
        r_hand_oris_relative_cloth_euler = radian_to_degree(rotation_matrix_to_euler_angle(r_hand_oris_relative_cloth, seq="xzy"))
        print(l_hand_oris_relative_cloth_euler, r_hand_oris_relative_cloth_euler)
        return l_hand_oris_relative_cloth_euler, r_hand_oris_relative_cloth_euler


    def clear_socket_buffer(self, sock):
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

    def keep_receiving(self):
        while True:
            self.save_one_data()

    def save_one_data(self, set_init=False):
        """存储手套数据"""
        save_left = False
        save_right = False
        source_data = [None, None]
        while(not save_left or not save_right):

            data, addr = self.sock.recvfrom(750)  # 缓冲区大小可根据需要调整

            # 解码JSON格式的数据
            hand_data = json.loads(data.decode('utf-8'))
            if hand_data['DeviceName'] == "UDXST1540L":
                save_left = True
                source_data[0] = hand_data
            elif hand_data['DeviceName'] == "UDXST1540R":
                save_right = True
                source_data[1] = hand_data
        self.source_data = source_data

        if set_init:
            # l_quaterion = torch.tensor(self.source_data[0][0])
            # r_quaterion = torch.tensor(self.source_data[1][0])
            # self.init_imu_data = [torch.mm(self.l_axis_oris, quaternion_to_rotation_matrix(l_quaterion)[0].float()), torch.mm(self.r_axis_oris, quaternion_to_rotation_matrix(r_quaterion)[0].float())]

            quaterion_l = self.source_data[0]["Bones"][-1]
            rotation_l = quaternion_to_rotation_matrix(torch.tensor(quaterion_l))

            quaterion_r = self.source_data[1]["Bones"][-1]
            rotation_r = quaternion_to_rotation_matrix(torch.tensor(quaterion_r))

            self.init_imu_data = [rotation_l, rotation_r]

    def get_imu_data(self, pos):
        """获取手套imu相对初始的旋转数据"""
        return_oris = True
        if pos == "left":
            if return_oris:
                quaterion_l = self.source_data[0]["Bones"][-1]
                rotation_l = quaternion_to_rotation_matrix(torch.tensor(quaterion_l))
                relative_rotation = self.init_imu_data[0].transpose(-2, -1).matmul(rotation_l)
                return relative_rotation

            quaterion_l = self.source_data[0]["Bones"][-1]
            rotation_l = quaternion_to_rotation_matrix(torch.tensor(quaterion_l))
            relative_rotation = self.init_imu_data[0].transpose(-2, -1).matmul(rotation_l)
            euler_relative = rotation_matrix_to_euler_angle(relative_rotation, "xzy")
            euler_relative = radian_to_degree(euler_relative)[0]
            return euler_relative

        elif pos == "right":
            if return_oris:
                quaterion_r = self.source_data[1]["Bones"][-1]
                rotation_r = quaternion_to_rotation_matrix(torch.tensor(quaterion_r))
                relative_rotation = self.init_imu_data[0].transpose(-2, -1).matmul(rotation_r)
                return relative_rotation

            quaterion_r = self.source_data[1]["Bones"][-1]
            rotation_r = quaternion_to_rotation_matrix(torch.tensor(quaterion_r))
            relative_rotation = self.init_imu_data[1].transpose(-2, -1).matmul(rotation_r)
            euler_relative = rotation_matrix_to_euler_angle(relative_rotation, seq="xzy")
            euler_relative = radian_to_degree(euler_relative)[0]
            return euler_relative

    def get_clamp_state(self):
        """获取握拳状态"""
        left_state = 1 if quaterion2degree(self.source_data[0]["Bones"][0])[0] < -60 and quaterion2degree(self.source_data[0]["Bones"][3])[0] < -60 and \
                          quaterion2degree(self.source_data[0]["Bones"][6])[0] < -60 and quaterion2degree(self.source_data[0]["Bones"][9])[0] < -60 else 0
        right_state = 1 if quaterion2degree(self.source_data[1]["Bones"][0])[0] < -60 and quaterion2degree(self.source_data[1]["Bones"][3])[0] < -60 and \
                          quaterion2degree(self.source_data[1]["Bones"][6])[0] < -60 and quaterion2degree(self.source_data[1]["Bones"][9])[0] < -60 else 0
        return [left_state, right_state]
    

    def get_finger_pose(self):

        """Get the finger pose of the hand"""

        res = {"left": [], "right": []}

        shi_max = [-80, -100, -63]
        zhong_max = [-80, -100, -63]
        wu_max = [-80, -100, -80]
        xiao_max = [-76, -93, -74]
        da_max_1 = [-60, -60]
        da_max_0 = [-50, -65]

        left_shi_1 = quaterion2degree(self.source_data[0]["Bones"][0])[0]
        left_shi_2 = quaterion2degree(self.source_data[0]["Bones"][1])[0]
        left_shi_3 = quaterion2degree(self.source_data[0]["Bones"][2])[0]
        left_zhong_1 = quaterion2degree(self.source_data[0]["Bones"][3])[0]
        left_zhong_2 = quaterion2degree(self.source_data[0]["Bones"][4])[0]
        left_zhong_3 = quaterion2degree(self.source_data[0]["Bones"][5])[0]
        left_wu_1 = quaterion2degree(self.source_data[0]["Bones"][6])[0]
        left_wu_2 = quaterion2degree(self.source_data[0]["Bones"][7])[0]
        left_wu_3 = quaterion2degree(self.source_data[0]["Bones"][8])[0]
        left_xiao_1 = quaterion2degree(self.source_data[0]["Bones"][9])[0]
        left_xiao_2 = quaterion2degree(self.source_data[0]["Bones"][10])[0]
        left_xiao_3 = quaterion2degree(self.source_data[0]["Bones"][11])[0]

        left_da_1 = quaterion2degree(self.source_data[0]["Bones"][12])
        left_da_2 = quaterion2degree(self.source_data[0]["Bones"][13])[1]
        left_da_3 = quaterion2degree(self.source_data[0]["Bones"][14])[1]

        left_shi_combine = (left_shi_1 + left_shi_2 + left_shi_3) / sum(shi_max)
        left_zhong_combine = (left_zhong_1 + left_zhong_2 + left_zhong_3) / sum(zhong_max)
        left_wu_combine = (left_wu_1 + left_wu_2 + left_wu_3) / sum(wu_max)
        left_xiao_combine = (left_xiao_1 + left_xiao_2 + left_xiao_3) / sum(xiao_max)
        left_da_combine_1 = (left_da_2) / da_max_1[0]

        right_shi_1 = quaterion2degree(self.source_data[1]["Bones"][0])[0]
        right_shi_2 = quaterion2degree(self.source_data[1]["Bones"][1])[0]
        right_shi_3 = quaterion2degree(self.source_data[1]["Bones"][2])[0]
        right_zhong_1 = quaterion2degree(self.source_data[1]["Bones"][3])[0]
        right_zhong_2 = quaterion2degree(self.source_data[1]["Bones"][4])[0]
        right_zhong_3 = quaterion2degree(self.source_data[1]["Bones"][5])[0]
        right_wu_1 = quaterion2degree(self.source_data[1]["Bones"][6])[0]
        right_wu_2 = quaterion2degree(self.source_data[1]["Bones"][7])[0]
        right_wu_3 = quaterion2degree(self.source_data[1]["Bones"][8])[0]
        right_xiao_1 = quaterion2degree(self.source_data[1]["Bones"][9])[0]
        right_xiao_2 = quaterion2degree(self.source_data[1]["Bones"][10])[0]
        right_xiao_3 = quaterion2degree(self.source_data[1]["Bones"][11])[0]
        right_da_1 = quaterion2degree(self.source_data[1]["Bones"][12])
        right_da_2 = quaterion2degree(self.source_data[1]["Bones"][13])[1]
        right_da_3 = quaterion2degree(self.source_data[1]["Bones"][14])[1]

        right_shi_combine = (right_shi_1 ) / shi_max[0]
        right_zhong_combine = (right_zhong_1 + right_zhong_2 + right_zhong_3) / sum(zhong_max)
        right_wu_combine = (right_wu_1 + right_wu_2 + right_wu_3) / sum(wu_max)
        right_xiao_combine = (right_xiao_1 + right_xiao_2 + right_xiao_3) / sum(xiao_max)
        right_da_combine_1 = (right_da_2) / da_max_1[0]

        res['left'] = [left_da_combine_1, left_da_combine_1, left_shi_combine, left_zhong_combine, left_wu_combine, left_xiao_combine]
        res['left'] = [math.fabs(i) * 100 for i in res['left']]
        res['right'] = [right_da_combine_1, right_da_combine_1, right_shi_combine, right_zhong_combine, right_wu_combine, right_xiao_combine]
        res['right'] = [math.fabs(i) * 100 for i in res['right']]
        # left_da_oris = quaternion_to_rotation_matrix(torch.tensor(self.source_data[0]["Bones"][12]).unsqueeze(0))
        # left_da_euler = rotation_matrix_to_euler_angle(left_da_oris, seq="xzy")
        # left_da_euler = radian_to_degree(left_da_euler)[0]  
        # print(quaterion2degree(self.source_data[0]["Bones"][12]), quaterion2degree(self.source_data[0]["Bones"][13]), quaterion2degree(self.source_data[0]["Bones"][14]), quaterion2degree(self.source_data[1]["Bones"][12]), quaterion2degree(self.source_data[1]["Bones"][13]), self.source_data[1]["Bones"][14])
        print(res)
        return res


    def get_finger_pose(self):

        """Get the finger pose of the hand"""

        res = {"left": [], "right": []}

        shi_max = [-80, -100, -63]
        zhong_max = [-80, -100, -63]
        wu_max = [-80, -100, -80]
        xiao_max = [-76, -93, -74]
        da_max_1 = [-60, -25]
        # da_max_0 = [-60]

        left_shi_1 = quaterion2degree(self.source_data[0]["Bones"][0])[0]
        left_shi_2 = quaterion2degree(self.source_data[0]["Bones"][1])[0]
        left_shi_3 = quaterion2degree(self.source_data[0]["Bones"][2])[0]
        left_zhong_1 = quaterion2degree(self.source_data[0]["Bones"][3])[0]
        left_zhong_2 = quaterion2degree(self.source_data[0]["Bones"][4])[0]
        left_zhong_3 = quaterion2degree(self.source_data[0]["Bones"][5])[0]
        left_wu_1 = quaterion2degree(self.source_data[0]["Bones"][6])[0]
        left_wu_2 = quaterion2degree(self.source_data[0]["Bones"][7])[0]
        left_wu_3 = quaterion2degree(self.source_data[0]["Bones"][8])[0]
        left_xiao_1 = quaterion2degree(self.source_data[0]["Bones"][9])[0]
        left_xiao_2 = quaterion2degree(self.source_data[0]["Bones"][10])[0]
        left_xiao_3 = quaterion2degree(self.source_data[0]["Bones"][11])[0]

        left_da_1 = quaterion2degree(self.source_data[0]["Bones"][12])[0]
        left_da_2 = quaterion2degree(self.source_data[0]["Bones"][13])[1]
        left_da_3 = quaterion2degree(self.source_data[0]["Bones"][14])[1]

        left_shi_combine = math.fabs(left_shi_1 ) / math.fabs(sum(shi_max[:1]))
        left_zhong_combine = math.fabs(left_zhong_1 + left_zhong_2 + left_zhong_3) / math.fabs(sum(zhong_max))
        left_wu_combine = math.fabs(left_wu_1 + left_wu_2 + left_wu_3) / math.fabs(sum(wu_max))
        left_xiao_combine = math.fabs(left_xiao_1 + left_xiao_2 + left_xiao_3) / math.fabs(sum(xiao_max))

        left_da_2 = min(25, fabs(left_da_2))
        left_da_combine_1 = math.fabs(left_da_3) / math.fabs(sum(da_max_1[:1]))
        left_da_combine_0 = math.fabs(left_da_2) / fabs(sum(da_max_1[1:]))

        right_shi_1 = quaterion2degree(self.source_data[1]["Bones"][0])[0]
        right_shi_2 = quaterion2degree(self.source_data[1]["Bones"][1])[0]
        right_shi_3 = quaterion2degree(self.source_data[1]["Bones"][2])[0]
        right_zhong_1 = quaterion2degree(self.source_data[1]["Bones"][3])[0]
        right_zhong_2 = quaterion2degree(self.source_data[1]["Bones"][4])[0]
        right_zhong_3 = quaterion2degree(self.source_data[1]["Bones"][5])[0]
        right_wu_1 = quaterion2degree(self.source_data[1]["Bones"][6])[0]
        right_wu_2 = quaterion2degree(self.source_data[1]["Bones"][7])[0]
        right_wu_3 = quaterion2degree(self.source_data[1]["Bones"][8])[0]
        right_xiao_1 = quaterion2degree(self.source_data[1]["Bones"][9])[0]
        right_xiao_2 = quaterion2degree(self.source_data[1]["Bones"][10])[0]
        right_xiao_3 = quaterion2degree(self.source_data[1]["Bones"][11])[0]
        right_da_1 = quaterion2degree(self.source_data[1]["Bones"][12])[0]
        right_da_2 = quaterion2degree(self.source_data[1]["Bones"][13])[1]
        right_da_3 = quaterion2degree(self.source_data[1]["Bones"][14])[1]

        right_shi_combine = math.fabs(right_shi_1) / math.fabs(sum(shi_max[:1]))
        right_zhong_combine = math.fabs(right_zhong_1 + right_zhong_2 + right_zhong_3) / math.fabs(sum(zhong_max))
        right_wu_combine = math.fabs(right_wu_1 + right_wu_2 + right_wu_3) / math.fabs(sum(wu_max))
        right_xiao_combine = math.fabs(right_xiao_1 + right_xiao_2 + right_xiao_3) / math.fabs(sum(xiao_max))
        right_da_combine_1 = math.fabs(right_da_3) / math.fabs(sum(da_max_1[:1]))

        right_da_2 = min(25, fabs(right_da_2))
        right_da_combine_1 = math.fabs(right_da_3) / sum(da_max_1[:1])
        right_da_combine_0 = right_da_2 / sum(da_max_1[1:])

        res['left'] = [left_da_combine_1, left_da_combine_0, left_shi_combine, left_zhong_combine, left_wu_combine, left_xiao_combine]
        res['left'] = [math.fabs(i) * 100 for i in res['left']]
        res['right'] = [right_da_combine_1, right_da_combine_0, right_shi_combine, right_zhong_combine, right_wu_combine, right_xiao_combine]
        res['right'] = [math.fabs(i) * 100 for i in res['right']]


        # print( quaterion2degree(self.source_data[0]["Bones"][13]))

        
        # print(res)
        # print(quaterion2degree(self.source_data[0]["Bones"][12]), )
        return res


    def print_test_data(self):
        z_180_oris = build_rot(theta=-180, rotation_axis=[0, 0, 1])
        l_init_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[0]).unsqueeze(0).float())
        l_init_cloth_oris = z_180_oris.matmul(l_init_cloth_oris)
        r_init_cloth_oris = quaternion_to_rotation_matrix(torch.tensor(self.init_cloth_imu[1]).unsqueeze(0).float())
        r_init_cloth_oris = z_180_oris.matmul(r_init_cloth_oris)
        l_init_hand_oris = quaternion_to_rotation_matrix(torch.tensor(self.source_data[0]["Bones"][-1]).unsqueeze(0).float())
        r_init_hand_oris = quaternion_to_rotation_matrix(torch.tensor(self.source_data[1]["Bones"][-1]).unsqueeze(0).float())
        l_init_cloth_euler = radian_to_degree(rotation_matrix_to_euler_angle(l_init_cloth_oris, seq="xzy"))
        r_init_cloth_euler = radian_to_degree(rotation_matrix_to_euler_angle(r_init_cloth_oris, seq="xzy"))
        l_init_hand_euler = radian_to_degree(rotation_matrix_to_euler_angle(l_init_hand_oris, seq="xzy"))
        r_init_hand_euler = radian_to_degree(rotation_matrix_to_euler_angle(r_init_hand_oris, seq="xzy"))
        hand_init2cloth_init = self.get_hand_init2cloth_init()
        hand_init2cloth_init_euler = radian_to_degree(rotation_matrix_to_euler_angle(hand_init2cloth_init[0], seq="xzy"))
        print(l_init_cloth_euler, r_init_cloth_euler, l_init_hand_euler, r_init_hand_euler)
        print(hand_init2cloth_init_euler)


"""基于手部控制的topic"""
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


def show_data(hand_server):

    while True:
        # global new_data
        # hand_server.source_data = new_data
        # print(hand_server.get_imu_data("left"))
        # print(hand_server.get_imu_data("right"))
        # print(hand_server.get_clamp_state())
        # print("")
        res = hand_server.get_finger_pose()

        left_hand_position = res["left"]
        right_hand_position = res["right"]
        print(res)

        time.sleep(1/30)


if __name__ == '__main__':

    global hand_server
    hand_server = HandServer_2()
    hand_server.clear_socket_buffer(hand_server.sock)
    t_1 = threading.Thread(target=hand_server.keep_receiving, )
    # t_2 = threading.Thread(target=show_data, args=(hand_server,))

    t_1.start()
    # time.sleep(1)
    # hand_server.save_one_data(True)
    # t_2.start()

    while not rospy.is_shutdown():

        res = hand_server.get_finger_pose()

        left_hand_position = res["left"]
        left_hand_position = [int(i) for i in left_hand_position]
        right_hand_position = res["right"]
        right_hand_position = [int(i) for i in right_hand_position]

        time.sleep(1/30)

        print(time.time(), res)

        success = call_control_end_hand_service(left_hand_position, right_hand_position, is_init=True)
        print(time.time(), success)

    
        if success:
            rospy.loginfo("Hand control service call successful.")
        else:
            rospy.loginfo("Hand control service call failed.")


