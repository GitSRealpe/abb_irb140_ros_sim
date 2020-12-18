#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "geometry_msgs/PoseArray.h"
#include "irb140_commander/PoseRPYarray.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MoveEnable{
  public:
    void cmdCallback(const irb140_commander::PoseRPYarray::ConstPtr& msg);
};

ros::Publisher pub;

  void MoveEnable::cmdCallback(const irb140_commander::PoseRPYarray::ConstPtr& msg){
    ROS_INFO("Path recibido");
    std::cout << msg->poses.size() <<"\n";
    std::cout << "puntos recibidos comandando..."<<"\n";

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setMaxVelocityScalingFactor(5);

    ROS_INFO_NAMED("path_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("path_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Start
    // geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(target_pose);

    tf2::Quaternion q;
    geometry_msgs::Quaternion q_msg;

    for (int i = 0; i < msg->poses.size(); i++) {
      q.setRPY(msg->poses[i].rpy.roll,msg->poses[i].rpy.pitch,msg->poses[i].rpy.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
      q.normalize();
      q_msg = tf2::toMsg(q);

      target_pose.position=msg->poses[i].position;
      target_pose.orientation=q_msg;
      std::cout <<target_pose<<"\n";
      waypoints.push_back(target_pose);  // down

    }
    std::cout <<waypoints.size()<<"\n";
    // std::cout <<waypoints[0]<<"\n";
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("path_commander", "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    // std::cout << "Enter for continuar"<<"\n";
    // std::cin.get();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_=trajectory;
    std::cout << "Ejecutando Trayectoria"<<"\n";
    move_group.execute(my_plan);
    std::cout << "Trayectoria realizada"<<"\n";
    move_group.clearPoseTargets();

    std_msgs::String aviso;
    aviso.data = "done";
    ROS_INFO("%s", aviso.data.c_str());
    pub.publish(aviso);

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
  pub = node_handle.advertise<std_msgs::String>("robot_commander/path_done", 1000);

  ros::waitForShutdown();
  return 0;
}
