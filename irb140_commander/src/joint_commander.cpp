#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <irb140_commander/JointsAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// #include <sstream>
#include "iostream"

class JointsAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<irb140_commander::JointsAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  irb140_commander::JointsFeedback feedback_;
  irb140_commander::JointsResult result_;

public:
  moveit::planning_interface::MoveGroupInterfacePtr move_group;
  JointsAction(std::string name, std::string planning_group) : as_(nh_, name, false),
                                                               action_name_(name)
  {
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&JointsAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&JointsAction::preemptCB, this));
    as_.start();
    move_group.reset(new moveit::planning_interface::MoveGroupInterface(planning_group));
  }

  ~JointsAction(void) {}

  void preemptCB()
  {
    std::cout << "cancelao \n";
    ROS_INFO("%s: Preempted", action_name_.c_str());
    as_.setPreempted();
    // success = false;
  }

  void goalCB()
  {
    irb140_commander::JointsGoalConstPtr goal = as_.acceptNewGoal();
    // publish info to the console for the user
    ROS_INFO("Ejecutando comando de articulaciones");
    std::cout << "Recibido " << goal->joints.size() << " articulaciones \n";
    // start executing the action
    const robot_state::JointModelGroup *joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(move_group->getName());

    ROS_INFO_NAMED("joint_commander", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("joint_commander", "End effector link: %s", move_group->getEndEffectorLink().c_str());
    move_group->setMaxVelocityScalingFactor(0.5);
    // Start
    std::cout << "all good \n";
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    std::cout << "all good \n";
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    ROS_INFO_NAMED("joint_commander: ", "Joint states: %f", joint_group_positions[0]);

    for (size_t i = 0; i < goal->joints.size(); i++)
    {
      joint_group_positions[i] = goal->joints[i];
    }
    move_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("joint_commander", "Plan (joint space goal) %s", success ? "OK" : "FAILED");
    move_group->move();

    std::cout << "moviendose \n";

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
  ros::init(argc, argv, "joints");
  JointsAction joints("joints", argv[1]);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}