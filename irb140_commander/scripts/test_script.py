#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import std_msgs.msg
from irb140_commander.msg import PoseRPY


import rospy
import copy

pose_pub = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
rospy.init_node('pathSender', anonymous=True)
rate = rospy.Rate(2) # 10hz
rate.sleep()

punto=PoseRPY()

#suelta objeto

punto.position.x=0.0;punto.position.y=-0.65;punto.position.z=0.6
punto.rpy.roll=0;punto.rpy.pitch=0;punto.rpy.yaw=-1.57
pose_pub.publish(punto)
print("enviado")
msg=rospy.wait_for_message('/robot_commander/pose_done',std_msgs.msg.String)
print("trayectoria completada")

punto.position.x=0.0;punto.position.y=-0.5;punto.position.z=0.4
pose_pub.publish(punto)
print("enviado")
msg=rospy.wait_for_message('/robot_commander/pose_done',std_msgs.msg.String)
print("trayectoria completada")

punto.position.x=0.5;punto.position.y=0.1;punto.position.z=0.3
punto.rpy.pitch=1.57;punto.rpy.yaw=0
pose_pub.publish(punto)
print("enviado")
msg=rospy.wait_for_message('/robot_commander/pose_done',std_msgs.msg.String)
print("trayectoria completada")
