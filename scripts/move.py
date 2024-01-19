#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import intera_interface

def get_joint_states():
    # 初始化ROS节点
    rospy.init_node('get_joint_states', anonymous=True)

    # 创建Sawyer机器人的接口
    limb = intera_interface.Limb('right')

    # 指定关节角度的字典
    target_joint_angles = {
        'right_j6': 1.22817285156,
        'right_j5': -1.379578125,
        'right_j4': -1.22184667969,
        'right_j3': 0.793063476562,
        'right_j2': -1.68220117188,
        'right_j1': 0.309650390625,
        'right_j0': 0.246497070313
    }

    # 检查输入的关节状态是否正确
    if len(target_joint_angles) != 7:
        print("Error: Incorrect number of joint angles provided. Expected 7.")
        return

    # 控制机械臂移动到目标关节角度
    limb.move_to_joint_positions(target_joint_angles)
    # 等待机械臂移动完成
    limb.move_to_neutral()
    # # 控制机械臂移动到指定的关节角度
    # limb.move_to_joint_positions(target_joint_angles)

    # # 指定关节角度的字典
    # target_joint_angles = {
    #     'right_j6': 1.60102734375,
    #     'right_j5': -1.37606640625,
    #     'right_j4': -1.19840039063,
    #     'right_j3': 0.380836914062,
    #     'right_j2': -1.84198730469,
    #     'right_j1': 0.30203515625,
    #     'right_j0': 0.601409179687
    # }

    # rospy.sleep(3)

    #  # 控制机械臂移动到指定的关节角度
    # limb.move_to_joint_positions(target_joint_angles)

    

if __name__ == '__main__':
    try:
        get_joint_states()
    except rospy.ROSInterruptException:
        pass

