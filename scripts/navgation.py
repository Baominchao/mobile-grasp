#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
from rospy import loginfo as log
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


def goal_fun(x,y,z,w):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal_pub.publish(goal)
    print(goal)

    
def callback(msg):
    if msg.data:
        goal_fun(
            -0.124807372689,
            -8.51624011993,
            0.00431156778374,
            0.999990705148
        )
        log('去往终点')


    #print(goal)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('moveit_fk_demo', anonymous=True)
    rospy.sleep(1)

    grasp_over_sub = rospy.Subscriber("/grasp_over", Bool, callback)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.sleep(7)


    # 去往抓取点
    goal_fun(
        -2.90008044243,
        -8.43901062012,
        0.0273160461498,
        0.99962684719
    )

    while not (rospy.is_shutdown()):
        pass


# header: 
#   seq: 12
#   stamp: 
#     secs: 1703675809
#     nsecs: 921167807
#   frame_id: ''
# goal_id: 
#   stamp: 
#     secs: 0
#     nsecs:         0
#   id: ''
# goal: 
#   target_pose: 
#     header: 
#       seq: 12
#       stamp: 
#         secs: 1703675809
#         nsecs: 920777291
#       frame_id: "map"
#     pose: 
#       position: 
#         x: -2.90008044243
#         y: -8.42520618439
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.0273160461498
#         w: 0.99962684719
# ---
# header: 
#   seq: 13
#   stamp: 
#     secs: 1703675854
#     nsecs: 232767867
#   frame_id: ''
# goal_id: 
#   stamp: 
#     secs: 0
#     nsecs:         0
#   id: ''
# goal: 
#   target_pose: 
#     header: 
#       seq: 13
#       stamp: 
#         secs: 1703675854
#         nsecs: 232374741
#       frame_id: "map"
#     pose: 
#       position: 
#         x: -0.527087748051
#         y: -8.43901062012
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.00705183878776
#         w: 0.999975135476
# ---
