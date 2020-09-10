#include "ros/ros.h"
#include "std_msgs/String.h"
#include "irb140_commander/Num.h"
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_commander");
  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("robot_commander/cmd_vel", 1000);

  ros::Publisher chatter_pub = n.advertise<irb140_commander::Num>("robot_commander/cmd_vel", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    ROS_INFO("Robot commander node started");

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
