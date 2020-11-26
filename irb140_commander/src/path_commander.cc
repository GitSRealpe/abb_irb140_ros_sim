#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/String.h"
#include <sstream>
#include "iostream"

class MoveEnable{
  public:
    void cmdCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
};

  void MoveEnable::cmdCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    ROS_INFO("Path recibido");
    std::cout << msg << ' ';
    std::cout << "\n";
    std::cout << "ejecutando comando..."<<"\n";

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const robot_state::JointModelGroup* joint_model_group =
    //     move_group.getCurrentState()->getJointModelGroup("irb140_arm");
    ROS_INFO_NAMED("path_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("path_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Start
    geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.1;
    waypoints.push_back(target_pose3);  // up and left

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("path_commander", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_=trajectory;
    move_group.execute(my_plan);

  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando vector de Poses en topico robot_commander/cmd_path...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_path", 1000, &MoveEnable::cmdCallback, &mv);

  ros::waitForShutdown();
  return 0;
}
