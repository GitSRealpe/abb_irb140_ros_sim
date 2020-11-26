#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from irb140_commander.msg import PoseRPY

import rospy

if __name__ == '__main__':

    punto=PoseRPY()
    punto.position.x=0.5
    punto.position.y=0
    punto.position.z=0.6
    punto.rpy.roll=0
    punto.rpy.pitch=0
    punto.rpy.yaw=0
    print punto

    pub = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
    rospy.init_node('poseSender', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    rate.sleep()
    pub.publish(punto)
    print "enviado"
