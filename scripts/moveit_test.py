#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from copy import deepcopy
import numpy as np
import tf
import intera_interface
from geometry_msgs.msg import Pose, Point, Quaternion

def move_sawyer():
    # 初始化 ROS 节点
    rospy.init_node('move_sawyer', anonymous=True)

    # 初始化 MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # 创建 RobotCommander 对象，用于获取机器人信息
    robot = moveit_commander.RobotCommander()

    # 创建 PlanningSceneInterface 对象，用于接触规划场景的信息
    scene = moveit_commander.PlanningSceneInterface()

    # 创建 MoveGroupCommander 对象，用于规划和执行
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.allow_replanning(True)

    # 获取机器人当前的姿态
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current Pose: {}".format(current_pose))

    # 设置目标姿态
    target_pose = Pose()
    target_pose.position.x = current_pose.position.x + 0.1
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z
    target_pose.orientation = current_pose.orientation

    # 进行运动规划
    move_group.set_pose_target(target_pose)
    plan = move_group.plan()

    # 执行运动规划
    move_group.execute(plan, wait=True)

    # 关闭 MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_sawyer()
    except rospy.ROSInterruptException:
        pass
