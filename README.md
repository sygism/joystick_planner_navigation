joystick_planner_navigation
===========================
##Introduction
The *joystick_planner_navigation* repository was constructed as a part of the author's Bsc thesis in University of Tartu. The aim of the thesis ("A continuous teleoperating system based on shared control concept") was developing a teleoperating system based on the shared control concept. The repository contains all of the packages and files that were created while developing the system.

The system was tested on the robotplatform <a href="https://github.com/ut-ims-robotics/robotont">Robotont</a>.

##Contents
* *joystick_planner* - ROS package containing the source code for the *joystick_planner* node.
* *joystick_planner_viz* - ROS package containing the source code for two system interface nodes.
* *nav_stack_jsp* - ROS package containing the neccessary launch files for using the *joystick_planner* node with Robotont.
* *speed_controller_jsp* - ROS package containing the source code for the *speed_controller_jsp* node.
* *testing_grounds_jsp* - ROS package containing the neccessary launch and world files for simulating the system in Gazebo.

##Videos

<iframe width="1280" height="720" src="https://www.youtube.com/embed/RIqokFGH6VY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

A short video demonstrating the usage of the system.

##Requirements
All the package specific dependencies and requirements are listed under the corresponding package directory.
