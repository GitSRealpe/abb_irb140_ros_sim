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
  actionlib_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  Joints.action
  PoseRPY.action
)

add_message_files(
  FILES
  Joints.msg
  PoseRPY.msg
  RPY.msg
  # PoseRPYarray.msg
  # OrientConst.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
 actionlib_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES irb140_commander
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#chmod +x mypythonscript.py para que sea ejecutable
catkin_install_python(PROGRAMS
  scripts/sendJoints.py
  scripts/sendPose.py
  scripts/sendPoseArray.py
  scripts/sendPoseArray2.py
  scripts/gripper_routine.py
  scripts/pick_routine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_executable(joint_commander src/joint_commander.cpp)
target_link_libraries(joint_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(joint_commander irb140_commander_generate_messages_cpp)
install(TARGETS joint_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pose_commander src/pose_commander.cpp)
target_link_libraries(pose_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pose_commander irb140_commander_generate_messages_cpp)
install(TARGETS pose_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(path_commander src/path_commander.cc)
# target_link_libraries(path_commander ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(path_commander irb140_commander_generate_messages_cpp)
# install(TARGETS path_commander DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(get_pose src/get_pose.cc)
target_link_libraries(get_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(get_pose irb140_commander_generate_messages_cpp)
install(TARGETS get_pose DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
