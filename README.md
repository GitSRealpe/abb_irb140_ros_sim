# ABB IRB140 ROS Simulation packages
This a set of ROS packages for simulating the ABB IRB140 robot, mainly for academic porpoises. Aimed to the students of Robotics in the the Universidad Nacional de Colombia.

Each package contains different functionalities, some of them require the use of the others packages, so its recommmended to download or install all of them as a whole.

**Author:** Sebastian Realpe Rua, mainly for use in the robotics lab of Universidad Nacional de Colombia

# Installation
Run the following commands, for execution in `ros-noetic` in `Ubuntu 20.04`, assuming you already have a Catkin workspace.

```
cd ~/catkin_ws/src/
git clone -b noetic https://github.com/GitSRealpe/abb_irb140_ros_sim.git
cd ..
catkin_make
```

## Required ROS packages
The next are the required ROS packages for a proper execution of all the packages contained in this project. Just execute the command `sudo apt install <name_of_package>` and it will be automatically installed.
```
ros-noetic-desktop-full                     # basic ROS instance, needed for almost any ROS project
ros-noetic-joint-state-controller           # required for proper robot simulation
ros-noetic-joint-state-publisher-gui        # plugin for publishing joint state values
ros-noetic-controller-manager               # package that implements virtual controllers for the simulated robot
ros-noetic-joint-trajectory-controller      # controller in charge of joint trajectory manipulation
ros-noetic-rqt-joint-trajectory-controller  # rqt plugin for publishing joint trajectory messages to the simulation controllers
ros-noetic-moveit                           # package for motion planning used both in simulated and real robot
ros-noetic-moveit-visual-tools              # moveit addon used for drawing the planned trajectories in RViz

```

## Gazebo joint mimic plugin
The gripper design uses a parallel mechanism for the gripping action, so for a correct simulation in Gazebo a joint mimic plugin is needed, mostly because it enables an easy emulation of the joint mimic property native to ROS in the URDF description.

Currently this is the plugin used [roboticsgroup_upatras_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), it can be installed cloning the repo plugin in your catkin workspace with the folllowing command lines:
```
cd ~/catkin_ws/src/
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
cd ..
catkin_make
```
The plugin is already configured to the gripper description used in this project, for more indepth information you can access the [plugin repo](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins) directly.
# Bringup and simulation
There are two options or environments to simulate the robot, RViz and Gazebo
## RViz
The bringup in RViz offers a visualization of the robot description and its current state, be it just in simulation or the real robot.

Simply execute `roslaunch irb140_sim irb140_rviz.launch`, this will load the robot description in the ROS parameter server and it can be then visualized in the RViz gui, this will also execute the `joint-state-publisher-gui` so the joint angles can be set manually using sliders.
![](images/rviz.gif)
## Gazebo
Simply run `roslaunch irb140_sim irb140_gazebo.launch`


**With the help from:**

https://github.com/FreddyMartinez/abb_irb140_support as a good starting point, helped to reduce initial setup work.
