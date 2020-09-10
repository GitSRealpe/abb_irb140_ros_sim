cmake_minimum_required(VERSION 3.0.2)
project(irb140_commander)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Num.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES irb140_commander
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_install_python(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  scripts/pose_commander.py
  scripts/joint_commander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_executable(test_commander src/test_commander.cpp)
target_link_libraries(test_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS test_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(joint_commander src/joint_commander.cpp)
target_link_libraries(joint_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joint_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(robot_commander src/robot_commander.cpp)
target_link_libraries(robot_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS robot_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
