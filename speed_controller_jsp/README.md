speed_controller_jsp ROS Package
=============================

## Overview
The *speed_controller_jsp* package acts as a medium between *move_base_flex* and the robot's hardware. The aim of this node is to scale the speed commands (*cmd_vel*) calculated
by *move_base_flex* to be in conjunction with the user's input (the vertical position of the left joystick).

Refer to *link_to_thesis* for more information.

## Parameters
* string cmd_vel_sub_topic (default: /move_base_flex/cmd_vel) - set the topic from which the move_base velocity commands should be read from.
* string cmd_vel_pub_topic (default: /cmd_vel) - set the topic to which the scaled velocity commands will be published to.
*	string speed_modifier_sub_topic (default: /joystick_planner/speed_modifier) - set the topic from which the vertical position of the operator's gamepad should be read from.
*	double max_vel_x (default: 0.5) - set the maximum linear velocity for the robot.
*	double min_vel_x (default: 0.1) - set the minimum linear velocity for the robot.

## Setup

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file):

    rosdep install speed_controller_jsp
