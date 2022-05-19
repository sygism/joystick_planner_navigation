nav_stack_jsp
=============================

## Overview
The *nav_stack_jsp* package contains the neccessary launch and configuration files for using the system with Robotont.

Refer to *link_to_thesis* for more information.

## Setup

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file):

    rosdep install nav_stack_jsp

The package requires ROS Navigation package to be built on the corresponding system.

The source code for ROS Navigation can be found <a href="https://github.com/ros-planning/navigation">here</a>.

Additional information about ROS Navigation can be found <a href="http://wiki.ros.org/navigation">here</a>.

The *joystick_planner* node requires an appropriate controller to be connected to the computer on which the system is being ran on. For example, using a Playstation
DualShock 4 controller with the Ubuntu operating system, an appropriate driver is *ds4drv*, which can be found from the apt repository:

    apt install ds4drv

For connecting the controller using Bluetooth:

    ds4drv

For connecting the controller using an USB cable:

    ds4drv --hidraw

## Usage
After successfully installing all the dependencies, for running the system on a real Robotont robot (for the best performance, the *depthimage_to_laserscan* node should be
ran on the actual robot, this launch file does not start said node automatically):

    roslaunch nav_stack_jsp nav_stack_robotont_real.launch

To run the system in a simulated environment (also launches the *depthimage_to_laserscan* node):

    roslaunch nav_stack_jsp nav_stack_robotont_sim.launch
