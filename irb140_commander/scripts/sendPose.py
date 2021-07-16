#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from irb140_commander.msg import PoseRPY

import rospy
import copy

pub = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
rospy.init_node('poseSender', anonymous=True)
rate = rospy.Rate(1) # 10hz
rate.sleep()

punto=PoseRPY()
punto.position.x=0.6;
punto.position.y=-0.1;
punto.position.z=0.6;
punto.rpy.roll=0;
punto.rpy.pitch=0;
punto.rpy.yaw=-0.57; #radianes

pub.publish(punto)
print("enviado")
