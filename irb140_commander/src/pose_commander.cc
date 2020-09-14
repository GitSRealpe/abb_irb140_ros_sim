#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/String.h"
#include "irb140_commander/Num.h"
#include <sstream>
#include "iostream"

class MoveEnable{
  public:
    void cmdCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

  void MoveEnable::cmdCallback(const geometry_msgs::Pose::ConstPtr& msg){
    ROS_INFO("Pose recibida:");
    std::cout << msg->position <<"\n";
    std::cout << "Marco de referencia: "<<"\n";
    std::cout << "ejecutando comando..."<<"\n";

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools.publishText(text_pose, "Pose commander", rvt::WHITE, rvt::XLARGE);

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup("irb140_arm");

    ROS_INFO_NAMED("pose_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("pose_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose;
    target_pose.position=msg->position;
    target_pose.orientation=msg->orientation;
    move_group.setPoseTarget(target_pose);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pose_commander", "Visualizing plan 1 (pose goal) %s", success ? "Exito" : "FAILED");
    //actually move the real robot
    move_group.move();

  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando Pose en topico robot_commander/cmd_pose...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_pose", 1000, &MoveEnable::cmdCallback, &mv);

  ros::waitForShutdown();
  return 0;
}
