#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/String.h"
#include "irb140_commander/PoseRPY.h"
#include <sstream>
#include "iostream"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_pose");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("irb140_arm");

  ros::Publisher pub = node_handle.advertise<geometry_msgs::Pose>("robot_commander/pose_actual", 1000);
  ROS_INFO("Pose actual del robot...");
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
    std::cout<<pose<<"\n";
    pub.publish(pose);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
