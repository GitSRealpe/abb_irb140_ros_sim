#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Step through via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1;
  visual_tools.publishText(text_pose, "Joint commander", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Getting Basic Information
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("joint_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("joint_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  //ROS_INFO_NAMED("joint_commander: ", "Joint states: %f", joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("joint_commander", "Visualizing plan (joint space goal) %s", success ? "OK" : "FAILED");
  //actually move the real robot
  //move_group.move();

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Finalizado", rvt::WHITE, rvt::XLARGE);

  ros::shutdown();
  return 0;
}
