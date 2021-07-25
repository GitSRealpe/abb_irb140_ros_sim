#!/usr/bin/env python
import tf.transformations as tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from irb140_commander.msg import PoseRPY
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy

pub = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
gripper_pub = rospy.Publisher('irb_140/gripper_controller/command', JointTrajectory, queue_size=10)
rospy.init_node('poseSender', anonymous=True)
rate = rospy.Rate(1) # 10hz
rate.sleep()

grip_ctrl=JointTrajectory()
grip_points= JointTrajectoryPoint()

grip_ctrl.joint_names.append("gripper_body__left_ext")
grip_points.time_from_start = rospy.Duration(1)
h = std_msgs.msg.Header()

def gripper_action(apertura):
    h.stamp = rospy.Time.now()
    grip_ctrl.header=h
    grip_points.positions=[apertura]
    grip_ctrl.points=[grip_points]
    gripper_pub.publish(grip_ctrl)
    print("gripper comandado")
gripper_action(0.25)

punto=PoseRPY()
punto.position.x=0.6
punto.position.y=0
punto.position.z=0.6
punto.rpy.roll=0
punto.rpy.pitch=0
punto.rpy.yaw=0
print(punto)

pub.publish(punto)
print("enviado")
