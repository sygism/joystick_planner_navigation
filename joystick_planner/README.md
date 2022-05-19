joystick_planner ROS Package
=============================

## Overview
The *joystick_planner* package acts as a global planner for ROS Navigation. The planner reads the input from a joystick based on which a global path is generated.

Refer to *link_to_thesis* for more information.

## Parameters
| Parameter name | Data type | Default value | Description |
| -------------- | --------- | ------------- | ----------- |
| scale_linear  | double    | 1.0           | Set the value by which the left stick input will be multiplied by (scaled) |
| scale_angular  | double    | 1.0           | Set the value by which the right stick input will be multiplied by (scaled) |
| odom_node | string | odom | Set the topic to which the robot's odometry is published to |
| controller_type | string | DS4 | Set the controller mapping |
| robot_base_frame | string | base_link | Set the definition of the robot's base frame |
| local_planner | string | teb_local_planner/TebLocalPlannerROS | Set the name of the local planner used in the system |
| path_exe_topic | string | move_base_flex/exe_path | Set the topic to which the generated Actions should be published to |
| input_status_topic | string | joystick_planner/input_status | Set the topic to which the planner status should be published to |
| speed_modifier_topic | string | joystick_planner/speed_modifier | Set the topic to which the left joystick y-position should be published to |

## Setup

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file):

    rosdep install joystick_planner
