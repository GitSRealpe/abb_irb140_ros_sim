#! /usr/bin/env python
import rospy
import tf.transformations as tft
import math
import numpy as np

import std_msgs.msg
import geometry_msgs.msg

from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.transforms import *
import pose_commander
import joint_commander
from irb140_commander.msg import PoseRPY
from irb140_commander.msg import Num
from std_msgs.msg import String
#print('holaa')
#from StowingKinect3 import find_pose
print('hola')
home = [1,0,0,0,0,0]
home1 = [-1,0,0,0,0,0]
pos3 = [0.18, 0.31, 0.1, 0, 1.12, 0]
pos3a = [0.0, 0.31, 0.1, 0, 1.12, 0]
pos3b = [0.0, 0.2, 0.1, 0, 1.12, 0]
pos2 = [0.18, 0.21, -0.1, 0, 1.4, 0]
pos4 = [1.57, -0.21, -0.29, 0, 0.52, 0]

pos5 = [1.57, 0.52, -1.12, 0, 0.64, 0]
Z_OFF= 0.4
GR_OFF=0.05
pub = rospy.Publisher('robot_commander/cmd_pose', PoseRPY, queue_size=10)
pubJo = rospy.Publisher('robot_commander/cmd_pose', Num , queue_size=10)


def to_pose(x,y,z,ox,oy,oz,ow):
    punto= geometry_msgs.msg.Pose()
    punto.position.x =x
    punto.position.y =y
    punto.position.z =z
    punto.orientation.x =ox
    punto.orientation.y =oy
    punto.orientation.z =oz
    punto.orientation.w =ow
    return punto
def to_array(punto):
    punto = [punto.position.x, punto.position.y, punto.position.z, punto.orientation.x, punto.orientation.y, punto.orientation.z, punto.orientation.w ]
    return punto

def set_gripper_position(width):
    return


def to_RPY(punto1, eu):
    punto=PoseRPY()
    punto.position.x=punto1.position.x
    punto.position.y=punto1.position.y
    punto.position.z=punto1.position.z
    punto.rpy.roll=eu[0]
    punto.rpy.pitch=eu[1]
    punto.rpy.yaw=eu[2]
    return punto

def execute_grasp(cont, N, p):
    #savedepth3.find_pose()


    width = p[6*N+4]
    pred=[[0,0.62,0.9,0,0.707,0,0.707],[0,0.62,0.67,0,0.707,0,0.707], [0,0.62,0.44,0,0.707,0,0.707], [0.15-width/2,0.62,0.9,0,0.707,0,0.707],[0.15-width/2,0.62,0.67,0,0.707,0,0.707], [0.15-width/2,0.62,0.44,0,0.707,0,0.707], [-0.15+width/2,0.62,0.9,0,0.707,0,0.707],[-0.15+width/2,0.62,0.67,0,0.707,0,0.707], [-0.15+width/2,0.62,0.44,0,0.707,0,0.707]]
    pred2=[[0,0.76,0.9,0,0.707,0,0.707],[0,0.76,0.67,0,0.707,0,0.707], [0,0.76,0.44,0,0.707,0,0.707], [0.15-width/2,0.85,0.9,0,0.707,0,0.707],[0.15-width/2,0.62,0.85,0,0.707,0,0.707], [0.15-width/2,0.85,0.44,0,0.707,0,0.707], [-0.15+width/2,0.85,0.9,0,0.707,0,0.707],[-0.15+width/2,0.62,0.85,0,0.707,0,0.707], [-0.15+width/2,0.85,0.44,0,0.707,0,0.707]]

    mov = p[6*N+5]

    #punto= [p[6*N+2],-p[6*N+0],-p[6*N+1],0,0,0,1]
    punto =to_pose(p[6*N+2],-p[6*N+0],-p[6*N+1],0,0,0,1)
    #invertidos porque si
    #punto.position.x=y
    #punto.position.y=x
    #punto.position.z=-z
    #print punto
    #print punto
    #x =0.5
    #y=-0.1
    #z=0.6
    w = 1
    punto1 = geometry_msgs.msg.Pose()
    punto2 = geometry_msgs.msg.Pose()
    punto1 = convert_pose(punto,"camera_depth_frame","world")
    print('punto1', punto1)
    print(xx)
    eu =  [0, 1.5, p[6*N+3]*math.pi/180]
    q = tft.quaternion_from_euler(np.pi, 0, p[6*N+3]*math.pi/180)
    #punto2.orientation.x = q[0]
    #punto2.orientation.y = q[1]
    #punto2.orientation.z = q[2]
    #punto2.orientation.w = q[3]
    punto1.position.z = punto1.position.z +Z_OFF
    #print punto2
    print(punto1)


    #enviar arriba del objeto
    pub.publish(to_RPY(punto1,eu))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)
    #pose_commander.move_to_pose(to_array(punto1))
    #joint_commander.move_to_jointspose(pos2)

    set_gripper_position(width+GR_OFF)
    #rospy.sleep(0.5)
    punto2 = punto1
    punto2.position.z= punto2.position.z-0.15


    #enviar a la posicion del objeto
    print('punto2', to_array(punto2))
    pub.publish(to_RPY(punto2,eu))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)
    #pose_commander.move_to_pose(to_array(punto2))
    #joint_commander.move_to_jointspose(pos3)

    set_gripper_position(width)
    #rospy.sleep(0.5)


    #enviar de regreso a la pos predefinida arriba del objeto
    print('punto1a', to_array(punto1))
    pub.publish(to_RPY(punto1,eu))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)
    #pose_commander.move_to_pose(to_array(punto1))
    #joint_commander.move_to_jointspose(pos2)


    #enviar a la posicion pred al frente del estante
    punto3 =[pred[cont][0],pred[cont][1],pred[cont][2], pred[cont][3], pred[cont][4], pred[cont][5],pred[cont][6]]
    #pose_commander.move_to_pose(punto3)
    #joint_commander.move_to_jointspose(pos4)
    eu3= [0,0,1.57]
    q3 = tft.quaternion_from_euler(0, 0, 1.5)
    print('punto3: ', punto3)
    pub.publish(to_RPY(punto3,eu3))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)
    #pose_commander.move_to_pose(punto3, q3)
    #joint_commander.move_to_jointspose(pos4)


    #entrar al estante
    punto4 =[pred2[cont][0],pred2[cont][1],pred2[cont][2], pred2[cont][3], pred2[cont][4], pred2[cont][5],pred2[cont][6]]
    q4 = tft.quaternion_from_euler(0, 0, 1.5)
    print('punto4: ', punto4)
    pub.publish(to_RPY(punto4,eu3))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)
    #pose_commander.move_to_pose(punto4)
    #joint_commander.move_to_jointspose(pos5)


    #salir del estante
    #pose_commander.move_to_pose(punto3)
    #joint_commander.move_to_jointspose(pos4)
    pub.publish(to_RPY(punto3,eu3))
    msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)

    #ir a home para tomar otra foto
    joint_commander.move_to_jointspose(home)

    flag=True

    return flag

def push_object(cont, N, p):
    width = p[6*N+4]
    mov = p[6*N+5]

    punto =to_pose(p[6*N+2],-p[6*N+0],-p[6*N+1],0,0,0,1 )
    w = 1
    punto1 = geometry_msgs.msg.Pose()
    punto2 = geometry_msgs.msg.Pose()
    punto1 = convert_pose(punto,"cam","world")
    q = tft.quaternion_from_euler(np.pi, 0, p[6*N+3]*math.pi/180)
    #punto2.orientation.x = q[0]
    #punto2.orientation.y = q[1]
    #punto2.orientation.z = q[2]
    #punto2.orientation.w = q[3]
    punto1.orientation.x = 0
    punto1.orientation.y = 0
    punto1.orientation.z = 0
    punto1.orientation.w = 1
    punto1.position.z = punto1.position.z +Z_OFF
    #print punto2
    print(punto1)


    #enviar arriba del objeto
    #pose_commander.move_to_pose(to_array(punto1))
    joint_commander.move_to_jointspose(pos2)
    #rospy.sleep(0.5)
    set_gripper_position(0)
    #rospy.sleep(0.5)
    punto2 = punto1
    punto2.position.z= punto2.position.z-Z_OFF


    #enviar a la posicion del objeto
    #pose_commander.move_to_pose(to_array(punto2))
    joint_commander.move_to_jointspose(pos3)
    #rospy.sleep(0.5)
    set_gripper_position(0)
    #rospy.sleep(0.5)

    #mover el objeto a un lado de la posicion
    punto3 = punto2
    if mov == 2:
        punto3.position.y=punto3.position.y-0.1
        punto1.position.y=punto1.position.y-0.1
    elif mov ==3:
        punto3.position.x=punto3.position.x+0.1
        punto1.position.x=punto1.position.x+0.1
    elif mov ==4:
        punto3.position.x=punto3.position.x-0.1
        punto1.position.x=punto1.position.x-0.1
    elif mov ==1:
        punto3.position.y=punto3.position.y+0.1
        punto1.position.y=punto1.position.y+0.1

    #pose_commander.move_to_pose(to_array(punto3))
    joint_commander.move_to_jointspose(pos3a)

    #subir un poco el gripper
    #pose_commander.move_to_pose(to_array(punto3))
    joint_commander.move_to_jointspose(pos3b)

    #ir a home para tomar otra foto
    joint_commander.move_to_jointspose(home)
    return

if __name__ == '__main__':
    rospy.init_node('APC_stowing')

    # Home
    #pubJo.publish(home)
    #joint_commander.move_to_jointspose(home)
    cont =0
    while not rospy.is_shutdown():
        if cont < 2:
           rospy.sleep(0.5)
           #set_finger_positions([0, 0])
           #rospy.sleep(0.5)
           #pubJo.publish(home)

           #msg = rospy.wait_for_message('/robot_commander/pose_done', std_msgs.msg.String)

           #joint_commander.move_to_jointspose(home)
           raw_input('inicio.')

           flag =False
           N = 0
           while flag == False:


               msg = rospy.wait_for_message('/ggcnn/rvalues', std_msgs.msg.Float32MultiArray)
               p = list(msg.data)


               punto =to_pose(p[6*N+2],-p[6*N+0],-p[6*N+1],0,0,0,1)
               #invertidos porque si
               #punto.position.x=y
               #punto.position.y=x
               #punto.position.z=-z
               #print punto
               #print punto
               #x =0.5
               #y=-0.1
               #z=0.6
               w = 1
               punto1 = geometry_msgs.msg.Pose()
               punto2 = geometry_msgs.msg.Pose()
               punto1 = convert_pose(punto,"camera_depth_frame","world")
               print('punto1', punto1)
               print(xx)
               print(p)
               if p[6*N+5]==0.0:
                   flag = execute_grasp(cont, N, p)
                   N=N+1
               elif p[6*N+5] != 0.0:
                   push_object(cont, N, p)
           pubJo.publish(home)
           #joint_commander.move_to_jointspose(home)
           rospy.sleep(0.5)
           cont = cont+1
           print('cont: ', cont )



        raw_input('Bye')
