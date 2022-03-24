#! /usr/bin/env python
import rospy
import sys

import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import irb140_commander.msg


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient("joints", irb140_commander.msg.JointsAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = irb140_commander.msg.JointsGoal(joints=[0, 0, 0, 0, 0, 0])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("fibonacci_client_py")
        result = fibonacci_client()
        print("Result:", result.estado)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
