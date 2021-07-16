#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from irb140_commander.msg import Num

import rospy
import copy

pub = rospy.Publisher('robot_commander/cmd_vel', Num, queue_size=10)
rospy.init_node('jointSender', anonymous=True)
rate = rospy.Rate(1) # 10hz
rate.sleep()

juntas=Num()
juntas.joints[0]=-1.0;
juntas.joints[2]=0.57;

pub.publish(juntas)
print("enviado")
