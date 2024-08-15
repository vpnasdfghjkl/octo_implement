"""
基于抓和放的逻辑判断@LiuXin
"""

# if target_right_list[7] > 0.6 and End_status_1 is True and End_status_0 is False:
#
#     if right_list[7] < 0.7:
#         End_status_1 = False
#
#         End_status_0 = True
#
#     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0],
#                                        right_hand_position=[0, 30, 80, 80, 80, 80])
#
#     self.joint_position_right[7] = target_right_list[7]
#
# elif target_right_list[7] < 0.7 and End_status_0 is True and End_status_1 is False:
#
#     if target_right_list[7] > 0.5:
#         End_status_1 = True
#         End_status_0 = False
#
#     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0],
#                                        right_hand_position=[0, 70, 20, 20, 20, 20])
#
#     self.joint_position_right[7] = target_right_list[7]


"""
基于抓和放的第二个逻辑代码 @LiTao
"""
# if target_right_list[7] < 0.5 and End_status_0 is True:
#
#     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0],
#                                        right_hand_position=[0, 70, 20, 20, 20, 20])
#
#     self.joint_position_right[7] = target_right_list[7]
#
#     # 第二个阶段
# elif target_right_list[7] >= 0.5 and End_status_0 is True and End_status_1 is False:
#
#     End_status_1 = True
#     # End_status_0 = False
#     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0],
#                                        right_hand_position=[0, 30, 80, 80, 80, 80])
#
#     self.joint_position_right[7] = target_right_list[7]
#
#     # 第三个阶段
# elif target_right_list[7] >= 0.8 and End_status_1 is True:
#     End_status_0 = False
#     self.joint_position_right[7] = target_right_list[7]
#
#     print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
#
#     # 第四个阶段
# elif target_right_list[7] < 0.8 and End_status_1 is True and End_status_0 is False:
#     End_status_1 = False
#     End_status_0 = True
#
#     self.call_control_end_hand_service(left_hand_position=[0, 0, 0, 0, 0, 0],
#                                        right_hand_position=[0, 70, 20, 20, 20, 20])
#     self.joint_position_right[7] = target_right_list[7]
#
#     # 补充逻辑
# else:
#     self.joint_position_right[7] = target_right_list[7]