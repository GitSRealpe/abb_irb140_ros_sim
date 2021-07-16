#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def callback(data):

    plt.close('all')
    print(len(data.points))
    puntos=np.arange(0,len(data.points),1)
    joint1=[];joint2=[];joint3=[];joint4=[];joint5=[];joint6=[];
    jointv1=[];jointv2=[];jointv3=[];jointv4=[];jointv5=[];jointv6=[];
    tiempos=[];
    for i in range((len(data.points))):
        joint1.append(data.points[i].positions[0]);joint2.append(data.points[i].positions[1]);joint3.append(data.points[i].positions[2])
        joint4.append(data.points[i].positions[3]);joint5.append(data.points[i].positions[4]);joint6.append(data.points[i].positions[5])
        jointv1.append(data.points[i].velocities[0]);jointv2.append(data.points[i].velocities[1]);jointv3.append(data.points[i].velocities[2])
        jointv4.append(data.points[i].velocities[3]);jointv5.append(data.points[i].velocities[4]);jointv6.append(data.points[i].velocities[5])
        tiempos.append(round(data.points[i].time_from_start.to_sec(), 2))

    fig=plt.figure(figsize=(15,8))
    ax = fig.add_subplot(111)    # The big subplot
    fig.text(0.5, 0.04, 'x=Tiempo[s]', ha='center', va='center',bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10},fontsize=15)
    fig.text(0.06, 0.5, 'y=Angulos [rad]', ha='center', va='center', rotation='vertical',bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 10},fontsize=15)

    print(tiempos)

    def plot_joint(datosx,datosy,stilo,tag,subplot):
        plt.subplot(subplot)
        plt.plot(datosx, datosy, stilo, label=tag)  # Plot some data on the axes.
        plt.grid()
        # plt.xticks(np.arange(0,len(data.points)+1,1))
        plt.xticks(list(range(len(tiempos))),tiempos)
        plt.legend()

    plot_joint(puntos,joint1,'--ro','joint1',321)
    plot_joint(puntos,joint2,'--bo','joint2',322)
    plot_joint(puntos,joint3,'--go','joint3',323)
    plot_joint(puntos,joint4,'--ko','joint4',324)
    plot_joint(puntos,joint5,'--yo','joint5',325)
    plot_joint(puntos,joint6,'--mo','joint6',326)
    # fig.suptitle('Angulos articulares, eef_step=1cm, total puntos='+str(len(data.points)), fontsize=20)
    fig.suptitle('Angulos articulares, total puntos='+str(len(data.points)), fontsize=20)

    fig=plt.figure(figsize=(15,8))
    plot_joint(puntos,jointv1,'--ro','joint1',321)
    plot_joint(puntos,jointv2,'--bo','joint2',322)
    plot_joint(puntos,jointv3,'--go','joint3',323)
    plot_joint(puntos,jointv4,'--ko','joint4',324)
    plot_joint(puntos,jointv5,'--yo','joint5',325)
    plot_joint(puntos,jointv6,'--mo','joint6',326)
    # fig.suptitle('Velocidades articulares, eef_step=1cm, total puntos='+str(len(data.points)), fontsize=20)
    fig.suptitle('Velocidades articulares, total puntos='+str(len(data.points)), fontsize=20)
    fig.text(0.5, 0.04, 'x=Tiempo[s]', ha='center', va='center',bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 10},fontsize=15)
    fig.text(0.06, 0.5, 'y=Vel.Angular [rad/s]', ha='center', va='center', rotation='vertical',bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 10},fontsize=15)

    plt.show()

if __name__ == '__main__':

    x=np.arange(0,6,1)
    print(x)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/robot_commander/robot_traj', JointTrajectory, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
