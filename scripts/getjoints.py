#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import intera_interface

def get_joint_states():
    # 初始化ROS节点
    rospy.init_node('get_joint_states', anonymous=True)

    # 创建Sawyer机器人的接口
    limb = intera_interface.Limb('right')

    # 获取当前机械臂的关节角度
    joint_angles = limb.joint_angles()

    # 获取当前机械臂end points
    end_points = limb.endpoint_pose()

    print(end_points)

    # 输出关节角度信息
    print("Current Joint Angles:")
    for joint, angle in joint_angles.items():
        print("{}: {}".format(joint, angle))

if __name__ == '__main__':
    try:
        get_joint_states()
    except rospy.ROSInterruptException:
        pass
