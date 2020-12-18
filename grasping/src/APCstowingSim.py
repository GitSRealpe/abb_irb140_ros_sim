#! /usr/bin/env python
import rospy
import tf.transformations as tft
import math
import numpy as np
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from irb140_commander.msg import PoseRPYarray
import std_msgs.msg
import geometry_msgs.msg
from irb140_commander.msg import PoseRPY
from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.transforms import *
#import pose_commander
#import joint_commander
from irb140_commander.msg import Num
from std_msgs.msg import String
import copy
home = [-1,0,0,0,0.78,0]
home1 = [-1,0,0,0,0,0]
pos3 = [0.18, 0.31, 0.1, 0, 1.12, 0]
pos3a = [0.0, 0.31, 0.1, 0, 1.12, 0]
pos3b = [0.0, 0.2, 0.1, 0, 1.12, 0]
pos2 = [0.18, 0.21, -0.1, 0, 1.4, 0]
pos4 = [1.57, -0.21, -0.29, 0, 0.52, 0]

pos5 = [1.57, 0.52, -1.12, 0, 0.64, 0]
Z_OFF= 0.35
GR_OFF=0.05
pubJo = rospy.Publisher('robot_commander/cmd_vel', Num , queue_size=10)

def to_pose(x,y,z,ox,oy,oz,ow):
    punto= geometry_msgs.msg.Pose()
    #punto.position.x =x
    #punto.position.y =y
    #punto.position.z =z
    punto.position.x =z
    punto.position.y =-x
    punto.position.z =-y

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

    width = p[6*N+4]
    pred=[[0.52,-0.346, 0.82, 0, 0, -math.pi/4],
    [0.52, -0.346, 0.59, 0, 0, -math.pi/4],
    [0.52,-0.346, 0.36,0,0,-math.pi/4],
    [0.565,-0.31, 0.82, 0,0,-math.pi/4],
    [0.565,-0.31, 0.59,0,0, -math.pi/4],
    [0.565,-0.31, 0.36,0,0,-math.pi/4],
    [0.52, -0.346, 0.82, 0, 0, -math.pi/4],
    [0.52, -0.346, 0.59, 0, 0, -math.pi/4],
    [0.52, -0.346, 0.36, 0, 0, -math.pi/4],
    [0.565,-0.31, 0.82, 0,0,-math.pi/4],
    [0.565,-0.31, 0.59,0,0, -math.pi/4],
    [0.565,-0.31, 0.36, 0, 0, -math.pi/4]]
    pred2= [[0.69,-0.516, 0.82,0,0,-math.pi/4],
    [0.69,-0.516, 0.59,0,0,-math.pi/4],
    [0.69,-0.516, 0.36,0,0,-math.pi/4],
    [0.735,-0.48, 0.82,0,0,-math.pi/4],
    [0.735,-0.48, 0.59,0,0,-math.pi/4],
    [0.735,-0.48, 0.36,0,0,-math.pi/4],
    [0.69,-0.516, 0.82,0,0,-math.pi/4],
    [0.69,-0.516, 0.59,0,0,-math.pi/4],
    [0.69,-0.516, 0.36,0,0,-math.pi/4],
    [0.735,-0.48, 0.82,0,0,-math.pi/4],
    [0.735,-0.48, 0.59,0,0,-math.pi/4],
    [0.735,-0.48, 0.36,0,0,-math.pi/4]]

    mov = p[6*N+5]


    punto =to_pose(p[6*N+0],p[6*N+1],p[6*N+2],0,0,0,1)
    punto1 = geometry_msgs.msg.Pose()
    punto2 = geometry_msgs.msg.Pose()
    punto1 = convert_pose(punto,"cam","world")
    eu =  [0, 1.5, p[6*N+3]]
    q = tft.quaternion_from_euler(0, 1.5, p[6*N+3]*math.pi/180)
    lista=PoseRPYarray()
    puntorpy=PoseRPY()
    puntorpy.position.x= punto1.position.x
    puntorpy.position.y=punto1.position.y
    puntorpy.position.z = Z_OFF
    puntorpy.rpy.roll = 0
    puntorpy.rpy.pitch = 1.57
    puntorpy.rpy.yaw = p[6*N+3]
    print(puntorpy)
    lista.poses.append(copy.deepcopy(puntorpy))
    #enviar arriba del objeto
    set_gripper_position(width+GR_OFF)
    #puntorpy.position.z= 0.06
    puntorpy.position.z= 0.15
    lista.poses.append(copy.deepcopy(puntorpy))

    set_gripper_position(width)

    #enviar de regreso a la pos predefinida arriba del objeto
    puntorpy.position.z= 0.4
    lista.poses.append(copy.deepcopy(puntorpy))

    # puntorpy.rpy.pitch=0
    # lista.poses.append(copy.deepcopy(puntorpy))

    puntorpy.position.x= pred[cont][0]
    puntorpy.position.y= pred[cont][1]
    puntorpy.position.z= pred[cont][2]
    puntorpy.rpy.roll = pred[cont][3]
    puntorpy.rpy.pitch = pred[cont][4]
    puntorpy.rpy.yaw = pred[cont][5]
    lista.poses.append(copy.deepcopy(puntorpy))



    puntorpy.position.x= pred2[cont][0]
    puntorpy.position.y= pred2[cont][1]
    puntorpy.position.z= pred2[cont][2]
    puntorpy.rpy.roll = pred2[cont][3]
    puntorpy.rpy.pitch = pred2[cont][4]
    puntorpy.rpy.yaw = pred2[cont][5]
    lista.poses.append(copy.deepcopy(puntorpy))

    puntorpy.position.x= pred[cont][0]
    puntorpy.position.y= pred[cont][1]
    puntorpy.position.z= pred[cont][2]
    puntorpy.rpy.roll = pred[cont][3]
    puntorpy.rpy.pitch = pred[cont][4]
    puntorpy.rpy.yaw = pred[cont][5]
    lista.poses.append(copy.deepcopy(puntorpy))

    #enviar a la posicion pred al frente del estante
    print lista
    pub = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
    rate = rospy.Rate(1) # 10hz
    rate.sleep()
    pub.publish(lista)

    msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)

    pubJo.publish(home)

    msg=rospy.wait_for_message('/robot_commander/joint_done',std_msgs.msg.String)

    print('finalizo mov')
    #joint_commander.move_to_jointspose(home)

    flag=True

    return flag

def push_object(cont, N, p):

    width = p[6*N+4]

    mov = p[6*N+5]


    punto =to_pose(p[6*N+0],p[6*N+1],p[6*N+2],0,0,0,1)
    punto1 = geometry_msgs.msg.Pose()
    punto2 = geometry_msgs.msg.Pose()
    punto1 = convert_pose(punto,"cam","world")
    eu =  [0, 1.5, p[6*N+3]]
    q = tft.quaternion_from_euler(0, 1.5, p[6*N+3]*math.pi/180)
    lista=PoseRPYarray()
    puntorpy=PoseRPY()
    puntorpy.position.x= punto1.position.x
    puntorpy.position.y=punto1.position.y
    puntorpy.position.z = Z_OFF
    puntorpy.rpy.roll = 0
    puntorpy.rpy.pitch = 1.57
    puntorpy.rpy.yaw = p[6*N+3]

    lista.poses.append(copy.deepcopy(puntorpy))
    #enviar arriba del objeto
    set_gripper_position(0)
    puntorpy.position.z= punto1.position.z
    lista.poses.append(copy.deepcopy(puntorpy))

    #enviar de regreso a la pos predefinida arriba del objeto
    if mov ==  3:
        puntorpy.position.x= puntorpy.position.x-0.1
    elif mov == 4:
        puntorpy.position.x= puntorpy.position.x+0.1
    elif mov == 1:
        puntorpy.position.x= puntorpy.position.y-0.1
    elif mov == 2:
        puntorpy.position.x= puntorpy.position.y+0.1


    lista.poses.append(copy.deepcopy(puntorpy))
    puntorpy.position.z= 0.35

    # puntorpy.rpy.pitch=0
    lista.poses.append(copy.deepcopy(puntorpy))

    #enviar a la posicion pred al frente del estante
    print lista
    pub = rospy.Publisher('robot_commander/cmd_path', PoseRPYarray, queue_size=10)
    rate = rospy.Rate(1) # 10hz
    rate.sleep()
    pub.publish(lista)

    #msg=rospy.wait_for_message('/robot_commander/path_done',std_msgs.msg.String)

    pubJo.publish(home)

    msg=rospy.wait_for_message('/robot_commander/joint_done',std_msgs.msg.String)

    print('finalizo mov')
    #joint_commander.move_to_jointspose(home)

    flag=True

    return flag


if __name__ == '__main__':
    rospy.init_node('APC_stowing')

    rate = rospy.Rate(1) # 10hz
    rate.sleep()

    # Home
    #joint_commander.move_to_jointspose(home)
    #pubJo.publish(home)
    cont =0
    while not rospy.is_shutdown():
        if cont < 5:

           #set_finger_positions([0, 0])
           #rospy.sleep(0.5)
           pubJo.publish(home)

           msg=rospy.wait_for_message('/robot_commander/joint_done',std_msgs.msg.String)
           # raw_input('inicio.')

           flag =False
           N = 0
           while flag == False:
               msg = rospy.wait_for_message('/ggcnn/rvalues', std_msgs.msg.Float32MultiArray)
               print('llego el mensaje del grasping')

               p = list(msg.data)
               punto =to_pose(p[6*N+0],p[6*N+1],p[6*N+2],0,0,0,1)
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
               punto1 = convert_pose(punto,"cam","world")
               print('punto1', punto1)

               print "este es el punto"
               print(p)
               if p[5]!=6:
                   if p[6*N+5]==0.0:
                       flag = execute_grasp(cont, N, p)
                       N=N+1
                   elif p[6*N+5] != 0.0:
                       push_object(cont, N, p)
               elif p[5]==6:
                   print('No hay objetos u hipotesis de grasping')

           #joint_commander.move_to_jointspose(home)
           rospy.sleep(0.5)
           cont = cont+1
           print('cont: ', cont )



        # raw_input('Bye')
