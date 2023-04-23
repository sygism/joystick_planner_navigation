joystick_planner_navigation
===========================
## Introduction
The *joystick_planner_navigation* repository was constructed as a part of the author's Bsc thesis in University of Tartu. The aim of the thesis ("A continuous teleoperating system based on shared control concept") was developing a teleoperating system based on the shared control concept. The repository contains all of the packages and files that were created while developing the system.

The system was tested on the robotplatform <a href="https://github.com/ut-ims-robotics/robotont">Robotont</a>.

The thesis can be found in UT DSpace environment [here](https://dspace.ut.ee/handle/10062/83015).

## Contents
* *joystick_planner* - ROS package containing the source code for the *joystick_planner* node.
* *joystick_planner_viz* - ROS package containing the source code for two system interface nodes.
* *nav_stack_jsp* - ROS package containing the neccessary launch files for using the *joystick_planner* node with Robotont.
* *speed_controller_jsp* - ROS package containing the source code for the *speed_controller_jsp* node.
* *testing_grounds_jsp* - ROS package containing the neccessary launch and world files for simulating the system in Gazebo.

## Videos

<a href="http://www.youtube.com/watch?feature=player_embedded&v=RIqokFGH6VY
" target="_blank"><img src="http://img.youtube.com/vi/RIqokFGH6VY/0.jpg" 
alt="alt text" width="240" height="180" border="10" /></a>

A short video demonstrating the usage of the system.

## Requirements
All the package specific dependencies and requirements are listed under the corresponding package directory.
