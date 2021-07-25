#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import std_msgs.msg
from irb140_commander.msg import PoseRPYarray
from irb140_commander.msg import PoseRPY

import rospy
import copy

pub = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
rospy.init_node('pathSender', anonymous=True)
rate = rospy.Rate(1) # 10hz
rate.sleep()

lista=PoseRPYarray()
punto=PoseRPY()
punto.position.x=0.5;punto.position.y=0.3;punto.position.z=0.5
punto.rpy.roll=0;punto.rpy.pitch=1.57;punto.rpy.yaw=0
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.4
punto.position.x=0.6
punto.rpy.pitch=0.7
lista.poses.append(copy.deepcopy(punto))
punto.position.y=-0.1
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.5
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.55
punto.rpy.pitch=0
lista.poses.append(copy.deepcopy(punto))
# print(lista)

lista=PoseRPYarray()
punto=PoseRPY()
punto.position.x=0.55;punto.position.y=0.0;punto.position.z=0.5
punto.rpy.roll=0;punto.rpy.pitch=1.57;punto.rpy.yaw=0
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.07
lista.poses.append(copy.deepcopy(punto))
punto.position.y=-0.1
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.3
lista.poses.append(copy.deepcopy(punto))
punto.position.z=0.55
punto.rpy.pitch=0
lista.poses.append(copy.deepcopy(punto))
lista.eef_step=0.01
# print(lista)

pub.publish(lista)
print("enviado")
