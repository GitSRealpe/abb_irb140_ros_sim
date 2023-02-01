#! /usr/bin/env python
import rospy
import sys

import actionlib

# Brings in the messages used by the joints action, including the
# goal message and the result message.
import irb140_commander.msg
from geometry_msgs.msg import Point
from irb140_commander.msg import RPY


def pose_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (jointsAction) to the constructor.
    client = actionlib.SimpleActionClient("poserpy", irb140_commander.msg.PoseRPYAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pos = Point()
    pos.x = 0.591
    pos.y = -0.104
    # pos.z = 0.12
    pos.z = 0.3
    ori = RPY()
    ori.roll = 0
    ori.pitch = 1.57
    ori.yaw = 0
    goal = irb140_commander.msg.PoseRPYGoal(position=pos, rpy=ori)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A jointsResult


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("pose_client_py")
        result = pose_client()
        print("Result:", result.estado)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
