#!/usr/bin/env python
import tf.transformations as tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
from irb140_commander.msg import PoseRPY
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy

if __name__ == '__main__':
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
    p_o=np.array([0, 0, 0])
    p_cam=np.array([-0.1,-0.1,0.1])
    p_obj=np.array([0.5,0,0.3])

    def lookAt():
        forward=(p_o-p_cam)/np.linalg.norm(p_o-p_cam)
        print("forward: ",forward,"con norma: ",np.linalg.norm(forward))

        temp=np.array([0,0,1])
        temp=(temp)/np.linalg.norm(temp)

        right=np.cross(temp,forward)
        right=(right)/np.linalg.norm(right)
        print("right: ",right,"con norma: ",np.linalg.norm(right))

        up=np.cross(forward,right)
        up=(up)/np.linalg.norm(up)
        print("up: ",up,"con norma: ",np.linalg.norm(up))

        rot_mat=np.array([forward,right,up])
        ang_vec=-np.array([tf.euler_from_matrix(rot_mat, axes='szyx')[2],
                           tf.euler_from_matrix(rot_mat, axes='szyx')[1],
                           tf.euler_from_matrix(rot_mat, axes='szyx')[0]])
        print(ang_vec)
        pos=p_cam+p_obj

        return pos,ang_vec

    pos,ang_vec=lookAt()
    punto=PoseRPY()
    punto.position.x=pos[0]
    punto.position.y=pos[1]
    punto.position.z=pos[2]
    punto.rpy.roll=ang_vec[0]
    punto.rpy.pitch=ang_vec[1]
    punto.rpy.yaw=ang_vec[2]
    print(punto)

    pub.publish(punto)
    print("enviado")
