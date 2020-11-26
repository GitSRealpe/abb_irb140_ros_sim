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

class MoveEnable{
  public:
    void cmdCallback(const irb140_commander::PoseRPY::ConstPtr& msg);
};

ros::Publisher pub;

  void MoveEnable::cmdCallback(const irb140_commander::PoseRPY::ConstPtr& msg){
    ROS_INFO("Pose recibida:");
    std::cout << msg->position <<"\n";
    std::cout << msg->rpy<<"\n";
    std::cout << "ejecutando comando..."<<"\n";

    moveit::planning_interface::MoveGroupInterface move_group("irb140_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup("irb140_arm");

    move_group.setMaxVelocityScalingFactor(0.1);

    ROS_INFO_NAMED("pose_commander", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("pose_commander", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    tf2::Quaternion q;
    geometry_msgs::Quaternion q_msg;
    q.setRPY(msg->rpy.roll,msg->rpy.pitch,msg->rpy.yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();
    q_msg = tf2::toMsg(q);
    std::cout<<q_msg<<"\n";  // Print the quaternion components (0,0,0,1)

    geometry_msgs::Pose target_pose;
    target_pose.position=msg->position;
    target_pose.orientation=q_msg;
    move_group.setPoseTarget(target_pose,"");
    std::cout <<move_group.getPoseTarget("")<<"\n";

    move_group.setNumPlanningAttempts(10);
    //move_group.setGoalTolerance(0.01);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pose_commander", "Visualizing plan 1 (pose goal) %s", success ? "Exito" : "FAILED");
    std::cout << "Enter for continuar"<<"\n";
    std::cin.get();
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
  ros::init(argc, argv, "pose_commander");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  static const std::string PLANNING_GROUP = "irb140_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  MoveEnable mv;
  ROS_INFO("Esperando PoseRPY en topico robot_commander/cmd_pose...");
  ros::Subscriber sub = node_handle.subscribe("robot_commander/cmd_pose", 1000, &MoveEnable::cmdCallback, &mv);
  pub = node_handle.advertise<std_msgs::String>("robot_commander/pose_done", 1000);

  ros::waitForShutdown();
  return 0;
}
