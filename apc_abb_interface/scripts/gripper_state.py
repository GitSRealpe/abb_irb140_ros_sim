import numpy as np
from math import pi

import rospy
import tf
from sensor_msgs.msg import JointState

rospy.init_node("traj_calc")
pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

joints = JointState()
joints.name.append("gripper_body__left_ext")

while not rospy.is_shutdown():
    joints.position = [0]
    joints.header.stamp = rospy.Time.now()
    # print(joints)
    pub.publish(joints)
    rospy.sleep(0.1)
