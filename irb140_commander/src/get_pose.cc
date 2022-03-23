#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_msgs/String.h"
#include <sstream>
#include "iostream"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_eigen/tf2_eigen.h>
// #include <Eigen/Geometry>
// #include <irb140_commander/RPY.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pose");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate loop_rate(2);

  moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
  ros::Publisher pub = node_handle.advertise<geometry_msgs::Pose>("robot_commander/pose_actual", 1000);

  geometry_msgs::Pose pose;
  // Eigen::Quaterniond q;
  // Eigen::Vector3d ypr;
  while (ros::ok())
  {
    ROS_INFO("Pose actual del robot...");
    pose = move_group.getCurrentPose().pose;
    std::cout << pose << "\n";
    // std::cout << pose.position;
    // q.x() = pose.orientation.x;
    // q.y() = pose.orientation.y;
    // q.z() = pose.orientation.z;
    // q.w() = pose.orientation.w;
    // ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
    // std::cout << ypr << "\n";

    pub.publish(pose);
    // ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
