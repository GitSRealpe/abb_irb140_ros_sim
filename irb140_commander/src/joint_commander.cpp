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
    void cmdCallback(const irb140_commander::Num::ConstPtr& msg);
};

  void MoveEnable::cmdCallback(const irb140_commander::Num::ConstPtr& msg){
    ROS_INFO("Angulos articulares recibidos:");
    for (size_t i = 0; i < 6; i++) {
      std::cout << msg->joints[i] << ' ';
    }
    std::cout << "\n";
    std::cout << "ejecutando comando..."<<"\n";

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1;
    visual_tools.publishText(text_pose, "Joint commander", rvt::WHITE, rvt::XLARGE);

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup("irb140_arm");
    ROS_INFO_NAMED("joint_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("joint_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Start
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    ROS_INFO_NAMED("joint_commander: ", "Joint states: %f", joint_group_positions[0]);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = msg->joints[0];  // radians
    joint_group_positions[1] = msg->joints[1];
    joint_group_positions[2] = msg->joints[2];
    joint_group_positions[3] = msg->joints[3];
    joint_group_positions[4] = msg->joints[4];
    joint_group_positions[5] = msg->joints[5];
    move_group.setJointValueTarget(joint_group_positions);

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("joint_commander", "Visualizing plan (joint space goal) %s", success ? "OK" : "FAILED");
    //actually move the real robot
    move_group.move();

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando angulos articulares en topico robot_commander/cmd_vel...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_vel", 1000, &MoveEnable::cmdCallback, &mv);

  // // Visualization
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  // visual_tools.deleteAllMarkers();
  //
  // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // text_pose.translation().z() = 1;
  // visual_tools.publishText(text_pose, "Joint commander", rvt::WHITE, rvt::XLARGE);


  //
  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
  ros::waitForShutdown();
  return 0;
}
