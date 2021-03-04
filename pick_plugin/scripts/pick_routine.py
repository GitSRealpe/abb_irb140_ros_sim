#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import std_msgs.msg
from irb140_commander.msg import PoseRPYarray
from irb140_commander.msg import PoseRPY

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospy
import copy



if __name__ == '__main__':
    path_pub = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
    gripper_pub = rospy.Publisher('irb_140/gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('pathSender', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    rate.sleep()

    lista=PoseRPYarray()
    punto=PoseRPY()
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

    # gripper_action(0.23)
    gripper_action(0.23)

    punto.position.x=0.5;punto.position.y=0.0;punto.position.z=0.5
    punto.rpy.roll=0;punto.rpy.pitch=1.57;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()

    punto.position.z=0.1
    lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()

    gripper_action(0.12)
    msg=rospy.wait_for_message('/gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto recogido")

    punto.position.z=0.3
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.0;punto.position.y=-0.7;punto.position.z=0.6
    punto.rpy.roll=0;punto.rpy.pitch=0;punto.rpy.yaw=-1.57
    lista.poses.append(copy.deepcopy(punto))
    punto.position.y=-0.78;punto.position.z=0.55
    lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()

    gripper_action(0.52)

    punto.position.x=0.0;punto.position.y=-0.65;punto.position.z=0.6
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.0;punto.position.y=-0.5;punto.position.z=0.4
    punto.rpy.pitch=1.57
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.5;punto.position.y=0.1;punto.position.z=0.3
    punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    punto.position.z=0.1
    lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()

    gripper_action(0.12)
    msg=rospy.wait_for_message('/gazebo/gripper/gripped',std_msgs.msg.String)
    print("objeto recogido")

    punto.position.z=0.3
    lista.poses.append(copy.deepcopy(punto))
    punto.position.x=0.0;punto.position.y=-0.6;punto.position.z=0.5
    punto.rpy.roll=0;punto.rpy.pitch=1.57;punto.rpy.yaw=-1.57
    lista.poses.append(copy.deepcopy(punto))
    punto.position.z=0.8
    punto.rpy.roll=0;punto.rpy.pitch=0;punto.rpy.yaw=-1.57
    # lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()

    gripper_action(0.52)

    punto.position.x=0.5;punto.position.y=0;punto.position.z=0.4
    punto.rpy.roll=0;punto.rpy.pitch=1.57;punto.rpy.yaw=0
    lista.poses.append(copy.deepcopy(punto))
    path_pub.publish(lista)
    print("enviado")
    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)
    print("trayectoria completada")
    lista.poses.clear()
