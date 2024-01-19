#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(Pose)


    消息订阅方:
        订阅话题并打印接收到的消息

    实现流程:
        1.导包 
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 订阅者 对象
        4.处理订阅的消息(回调函数)
        5.设置循环调用回调函数



"""
#1.导包 
import rospy
import moveit_commander
from copy import deepcopy
import numpy as np
import tf
import baxter_interface
from geometry_msgs.msg import Pose, Point, Quaternion

def moveTime(plan, k=1.5):
    # return
    points = plan.joint_trajectory.points
    for i in range(len(points)):
        points[i].time_from_start *= k
   
def move(waypoint):
    for i in range(1,9):
        item = deepcopy(waypoint)
        (plan, fraction) = arm.compute_cartesian_path(item, 0.1, 0.0)
        if fraction == 1.0:
            moveTime(plan)
            rospy.loginfo('Waypoint planning success {}%, execute!'.format(fraction * 100))
            arm.execute(plan)
            rospy.sleep(3)
            return True
        else:
            rospy.logwarn("Waypoint planning failed {}%，ready to replanning {}/8".format(fraction * 100, i))
    rospy.logerr("Execute failed, exit...")
    exit()

# 相机坐标点转换到机器人base坐标
def transform(target_pose):
    rot_camera_to_base_matrix = tf.transformations.quaternion_matrix(rot_camera_to_base)[:3, :3]
    object_position_camera = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    object_position_base = trans_camera_to_base + np.dot(rot_camera_to_base_matrix, object_position_camera)
    return object_position_base

def doMsg(target_pose):
    global joint_angle
    waypoint = []
    object_position_base = transform(target_pose)
    nowPose = Pose()
    # 第一个点
    nowPose.position.x = object_position_base[0]
    nowPose.position.y = object_position_base[1]
    nowPose.position.z = object_position_base[2]
    print("x:", object_position_base[0], "y:", object_position_base[1], "z:", object_position_base[2])
    sub.unregister()  # 取消订阅控制信息话题
    nowPose.orientation.x = -0.028025946493451793
    nowPose.orientation.y = 0.829735028503007
    nowPose.orientation.z = -0.013602953542836449
    nowPose.orientation.w = 0.5572874379107896
    waypoint.append(deepcopy(nowPose))

    move(waypoint)
    grip.close()
    arm.set_joint_value_target(joint_angle)
    arm.go()
    rospy.loginfo("已完成")

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("grasp")
    arm = moveit_commander.move_group.MoveGroupCommander('right_arm')
    arm.allow_replanning(True)  # 允许重新规划
    arm.set_goal_position_tolerance(0.001)  # 设置xyz允许误差
    arm.set_goal_orientation_tolerance(0.001)  # 设置xyzw允许误差
    arm.set_max_acceleration_scaling_factor(0.5)  # 设置最大加速度的比例
    arm.set_max_velocity_scaling_factor(0.1)  # 设置最大速度的比例
    arm.set_goal_joint_tolerance(0.001)  # 设置关节允许误差

    # 标定结果中的平移和旋转信息
    trans_camera_to_base = np.array([1.2895931471914541, -0.2598202458600332, -0.02014832794844723]) # xyz
    rot_camera_to_base = np.array([-0.5330623681043448, -0.5262759863619231, 0.467670212517929, 0.46921495096939136]) #xyzw

    #夹爪
    grip = baxter_interface.Gripper('right') 
    grip.calibrate()

    #3.实例化 订阅者 对象
    #sub = rospy.Subscriber("chatter",Pose,doMsg,queue_size=10)
    sub = rospy.Subscriber("/detect_result_out",Pose,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rospy.spin()