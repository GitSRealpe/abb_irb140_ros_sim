#!/usr/bin/env python3
import rospy
import std_msgs.msg
from irb140_commander.msg import Joints

joints_pub = rospy.Publisher("robot_commander/cmd_joints", Joints, queue_size=10)
rospy.init_node("pathSender", anonymous=True)

joints = Joints()
joints.joints.append(0)
joints.joints.append(1)
joints.joints.append(2)
joints.joints.append(3)
joints.joints[2] = 5

joints_pub.publish(joints)
