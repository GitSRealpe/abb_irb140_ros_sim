# LABSIR Bin Picking ROS Simulation packages
This a set of ROS packages for simulating the ABB IRB140 robot for a Pick and Place task, mainly for academic purposes. Aimed to the students of Robotics in the National University of Colombia.

Each package contains different functionalities, some of them require the use of the others packages, so its recommended to download or install all of them as a whole.

Please go to this repository [**Wiki**](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki) for installation and an in-depth description of the contents of this project.

**Author:** Sebastian Realpe Rua, mainly for use in the National University of Colombia robotics lab: **LABSIR**.

# Brief overview
As already stated, this project is a collection of ROS packages for the simulation of a typical robot application; [Simulation](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki/2.-Bringups-and-simulation), [Motion planning](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki/4.-Moveit-demos), [Motion request](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki/5.-Moveit-API) and some [Picking](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki/6.-Pick-and-Place-Demo) of objects. Here is a brief description of what this packages has to offer:
* [Robot description and Simulation](#robot-description-and-simulation)
* [Two-finger Gripper](#two-finger-gripper)
* [Moveit demos](#moveit-demos)
* [Joint commander](#joint-commander)
* [Pose commander](#pose-commander)
* [Path commander](#path-commander)
* [Pick and drop demo](#pick-and-drop-demo)

## Robot description and Simulation
https://user-images.githubusercontent.com/28267340/126930250-cc8e315d-f125-4196-b113-50cfdb0df5e7.mp4

## Two-finger Gripper
This is the gripper used for the project, you can find it [here](https://github.com/GitSRealpe/abb_irb140_ros_sim/tree/noetic/apc_gripper). In the video is shown the joint control of it in Gazebo using `rqt-joint-trajectory-controller`

https://user-images.githubusercontent.com/28267340/126928053-df186a64-995e-4f71-9063-aea7391f68ea.mp4

## Moveit demos
https://user-images.githubusercontent.com/28267340/126930116-884f5b8b-a0f0-4dbf-9bf8-7233949b59d8.mp4

## Robot commander
### Joint commander
https://user-images.githubusercontent.com/28267340/126929751-e79662e7-3aa4-4619-a36f-c85c7dbd8e15.mp4
### Pose commander
https://user-images.githubusercontent.com/28267340/126929735-98ffcb69-5a8e-4157-b365-a3c52ca36aa1.mp4
### Path commander
https://user-images.githubusercontent.com/28267340/126930075-794128c1-5230-4496-9548-49821fc1a29c.mp4



## Pick and drop demo
To showcase the integration of all the packages working for one objective, we have this demo that can be run with just two line commands, see more of it [here](https://github.com/GitSRealpe/abb_irb140_ros_sim/wiki/6.-Pick-and-Place-Demo).

https://user-images.githubusercontent.com/28267340/126929479-03cd2bb9-99eb-4f54-9bd6-e74f5160bb13.mp4




