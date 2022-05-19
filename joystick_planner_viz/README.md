joystick_planner_viz ROS Package
=============================

## Overview
The *joystick_planner_viz* package contains two interfaces for using the teleoperating system. One of them is based on OpenCV and the other uses RViz to visualize the environment
of the robot as well as the input given by the user.

Refer to *link_to_thesis* for more information.

## Setup

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file):

    rosdep install joystick_planner_viz

The joystick_planner_viz node requires *cvui* to function properly. The appropriate header file (cvui.h) is included in the source directory.
The source code for cvui can be found <a href="https://github.com/Dovyski/cvui">here</a>.

## Usage

After successfully installing all of the dependencies, to launch the OpenCV based UI:

    roslaunch joystick_planner_viz viz.launch

To launch the RViz based UI:

    roslaunch joystick_planner_viz jsp_rviz_ui.launch
