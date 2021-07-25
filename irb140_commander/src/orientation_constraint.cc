#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/String.h"
#include "irb140_commander/OrientConst.h"
#include <sstream>
#include "iostream"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MoveEnable{
  public:
    void cmdCallback(const irb140_commander::OrientConst::ConstPtr& msg);
};

ros::Publisher pub;
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  void MoveEnable::cmdCallback(const irb140_commander::OrientConst::ConstPtr& msg){
    ROS_INFO("Pose recibida:");
    std::cout << msg->pose.position <<"\n";
    std::cout << msg->pose.rpy<<"\n";
    ROS_INFO("Constraint recibida:");
    std::cout << msg->constraint<<"\n";
    std::cout << "ejecutando comando..."<<"\n";

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup("irb140_arm");

    // move_group.setMaxVelocityScalingFactor(0.1);

    ROS_INFO_NAMED("pose_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("pose_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    tf2::Quaternion q;
    geometry_msgs::Quaternion q_msg;
    q.setRPY(msg->pose.rpy.roll,msg->pose.rpy.pitch,msg->pose.rpy.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
    q_msg = tf2::toMsg(q);
    std::cout<<"cuaternio de pose \n"<<q_msg<<"\n";  // Print the quaternion components (0,0,0,1)

    tf2::Quaternion q_const;
    geometry_msgs::Quaternion q_msg_const;
    q_const.setRPY(msg->constraint.roll,msg->constraint.pitch,msg->constraint.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
    q_const.normalize();
    q_msg_const = tf2::toMsg(q_const);
    std::cout<<"cuaternio de constraint \n"<<q_msg_const<<"\n";  // Print the quaternion components (0,0,0,1)

    geometry_msgs::Pose target_pose;
    target_pose.position=msg->pose.position;
    target_pose.orientation=q_msg;
    move_group.setPoseTarget(target_pose,"");
    std::cout <<move_group.getPoseTarget("")<<"\n";

    move_group.setNumPlanningAttempts(10);

    if (msg->enable==true) {
      std::cout<<"Constraint enabled, configurando\n";  // Print the quaternion components (0,0,0,1)
      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = move_group.getEndEffectorLink().c_str();
      ocm.header.frame_id = move_group.getPlanningFrame().c_str();
      ocm.orientation = q_msg_const;
      ocm.absolute_x_axis_tolerance = 0.01;
      ocm.absolute_y_axis_tolerance = 0.01;
      ocm.absolute_z_axis_tolerance = 0.01;
      ocm.weight = 1.0;

      moveit_msgs::Constraints test_constraints;
      test_constraints.orientation_constraints.push_back(ocm);
      move_group.setPathConstraints(test_constraints);

    }else{
      std::cout<<"Constraint disabled, movimiento libre\n";  // Print the quaternion components (0,0,0,1)
      move_group.clearPathConstraints();
      // move_group.setGoalTolerance(0);
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("constraint_commander", "Visualizing plan (pose goal) %s", success ? "Exito" : "FAILED");

    // Visualize the plan in RViz
    visual_tools_->deleteAllMarkers();
    Eigen::Isometry3d text_pose;
    text_pose.translation() = Eigen::Vector3d( 0, 0, 1 ); // translate x,y,z

    visual_tools_->publishText(text_pose, "Trayectoria Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tcp_link"), joint_model_group, rvt::LIME_GREEN);
    visual_tools_->trigger();

    // std::cout << "Enter for continuar"<<"\n";
    // std::cin.get();
    //actually move the real robot
    move_group.move();
    std::cout<<"completado"<<"\n";
    move_group.clearPoseTargets();

    std_msgs::String aviso;
    aviso.data = "done";
    ROS_INFO("%s", aviso.data.c_str());
    pub.publish(aviso);

  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Oconstraint_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/moveit_visual_markers"));
  // Don't forget to trigger the publisher!
  visual_tools_->trigger();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando Constraint en topico robot_commander/cmd_Oconstraint...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_Oconstraint", 1000, &MoveEnable::cmdCallback, &mv);
  pub = node_handle.advertise<std_msgs::String>("robot_commander/Oconstraint_done", 1000);

  ros::waitForShutdown();
  return 0;
}
