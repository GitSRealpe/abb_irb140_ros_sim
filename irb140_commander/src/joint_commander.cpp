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

ros::Publisher pub;
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  void MoveEnable::cmdCallback(const irb140_commander::Num::ConstPtr& msg){
    ROS_INFO("Angulos articulares recibidos:");
    for (size_t i = 0; i < 6; i++) {
      std::cout << msg->joints[i] << ' ';
    }
    std::cout << "\n";
    std::cout << "ejecutando comando..."<<"\n";

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
    ROS_INFO_NAMED("/joint_commander", "Visualizing plan (joint space goal) %s", success ? "OK" : "FAILED");

    // Visualize the plan in RViz
    visual_tools_->deleteAllMarkers();
    Eigen::Isometry3d text_pose;
    text_pose.translation() = Eigen::Vector3d( 0, 0, 1 ); // translate x,y,z

    visual_tools_->publishText(text_pose, "Trayectoria Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tcp_link"), joint_model_group, rvt::LIME_GREEN);
    visual_tools_->trigger();


    //actually move the real robot
    move_group.move();

    std_msgs::String aviso;
    aviso.data = "done";
    ROS_INFO("%s", aviso.data.c_str());
    pub.publish(aviso);

  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create pose
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/moveit_visual_markers"));
  Eigen::Isometry3d pose;
  pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along Y axis by 45 degrees
  pose.translation() = Eigen::Vector3d( 0.5, 0.5, 0.5 ); // translate x,y,z

  // Publish arrow vector of pose
  ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
  visual_tools_->publishArrow(pose, rvt::RED, rvt::LARGE);
  visual_tools_->publishText(pose, "joint_commander Activo", rvt::WHITE, rvt::XLARGE);

  // Don't forget to trigger the publisher!
  visual_tools_->trigger();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando angulos articulares en topico robot_commander/cmd_vel...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_vel", 1000, &MoveEnable::cmdCallback, &mv);
  pub = node_handle.advertise<std_msgs::String>("robot_commander/joint_done", 1000);

  ros::waitForShutdown();
  return 0;
}
