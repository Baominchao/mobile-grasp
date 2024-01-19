#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.msg

def callback(msg):
    robotPub.publish(msg)

rospy.init_node('remap', anonymous=True)
robotSub = rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, callback) 
robotPub = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=100)

rospy.spin()

while not rospy.is_shutdown():
    pass