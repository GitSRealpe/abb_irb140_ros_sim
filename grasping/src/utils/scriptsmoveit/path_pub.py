#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('chatter', PoseArray, queue_size=10)
    rospy.init_node('path_pub', anonymous=True)
    poseArr=PoseArray()
    poseArr.header.stamp=rospy.Time.now()
    poseArr.header.frame_id="world"

    pose_dummy=Pose()
    pose_dummy.position.x=0.1
    poseArr.poses.append(pose_dummy)

    pose_dummy=Pose()
    pose_dummy.position.x=0.3
    poseArr.poses.append(pose_dummy)

    print(poseArr)
    pub.publish(poseArr)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# geometry_msgs::PoseArray  posearray;
# posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
# posearray.header.frame_id = "map" // frame id in which the array is published
# geometry_msgs::Pose p; // one pose to put in the array
# // fill p appropriately
# p.position.x = ...
# p.orientation.x = ...
# // push in array (in C++ a vector, in python a list)
# posearray.poses.push_back(p);
