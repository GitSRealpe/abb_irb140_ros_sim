#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <irb140_commander/PoseRPYAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <sstream>
#include "iostream"

class PoseRPYAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<irb140_commander::PoseRPYAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  irb140_commander::PoseRPYFeedback feedback_;
  irb140_commander::PoseRPYResult result_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group;

public:
  PoseRPYAction(std::string name, std::string planning_group) : as_(nh_, name, false),
                                                                action_name_(name)
  {
    // register the goal and preempt callbacks
    as_.registerGoalCallback(boost::bind(&PoseRPYAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PoseRPYAction::preemptCB, this));
    as_.start();
    move_group.reset(new moveit::planning_interface::MoveGroupInterface(planning_group));
  }

  ~PoseRPYAction(void) {}

  void preemptCB()
  {
    std::cout << "cancelao \n";
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    // success = false;
  }

  void goalCB()
  {
    irb140_commander::PoseRPYGoalConstPtr goal = as_.acceptNewGoal();
    // publish info to the console for the user
    ROS_INFO("PoseRPY recibida:");
    std::cout << goal->position << "\n";
    std::cout << goal->rpy << "\n";
    std::cout << "ejecutando comando..."
              << "\n";

    move_group->setMaxVelocityScalingFactor(1);
    ROS_INFO_NAMED("pose_commander", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("pose_commander", "End effector link: %s", move_group->getEndEffectorLink().c_str());
    move_group->setNumPlanningAttempts(10);

    moveit_msgs::JointConstraint jc;
    jc.joint_name = "joint_6";
    jc.weight = 1;
    jc.tolerance_above = 1.57;
    jc.tolerance_below = 1.57;

    moveit_msgs::Constraints constraints;
    constraints.joint_constraints.push_back(jc);
    move_group->setPathConstraints(constraints);

    tf2::Quaternion q;
    q.setRPY(goal->rpy.roll, goal->rpy.pitch, goal->rpy.yaw); // Create this quaternion from roll/pitch/yaw (in radians)
    q.normalize();

    geometry_msgs::Pose target_pose;
    target_pose.position = goal->position;
    target_pose.orientation = tf2::toMsg(q);
    move_group->setPoseTarget(target_pose, "");
    std::cout << move_group->getPoseTarget("") << "\n";

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("pose_commander", "Pose plan %s", success ? "OK" : "FAILED");
    // actually move the real robot
    move_group->move();
    move_group->clearPoseTargets();

    as_.publishFeedback(feedback_);
    if (success)
    {
      result_.estado = "success";
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char **argv)
{
  std::cout << "planning_group: " << argv[1] << "\n";
  ros::init(argc, argv, "poserpy");
  PoseRPYAction poserpy("poserpy", argv[1]);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}