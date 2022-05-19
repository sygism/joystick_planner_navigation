testing_grounds_jsp ROS Package
=============================

## Introduction
The *testing_grounds_jsp* package is a simulation environment for testing the *joystick_planner* navigation package. It features different gazebo worlds and a model of Robotont with a mounted Intel® RealSense™ depth camera D435.

## Dependencies
* <a href="http://wiki.ros.org/gazebo_ros_pkgs">*gazebo_ros_pkgs*</a>
* <a href="http://wiki.ros.org/gazebo_ros_control">*gazebo_ros_control*</a>
* <a href="http://wiki.ros.org/cmake_modules">*cmake_modules*</a>
* <a href="http://wiki.ros.org/roscpp">*roscpp*</a>
* <a href="https://github.com/IntelRealSense/realsense-ros">*realsense-ros*</a>
* <a href="https://github.com/pal-robotics/realsense_gazebo_plugin">*realsense_gazebo_plugin*</a>
* <a href="https://github.com/robotont/robotont_description">*robotont_description*</a>

The Robotont URDF models and spawn files included in the package are based on the ones found in the <a href="https://github.com/robotont/robotont_nuc_description">Robotont GitHub</a>.

## Setup

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file):

    rosdep install testing_grounds_jsp

## Usage

To spawn Robotont with a depth camera in a custom world

    roslaunch testing_grounds_jsp test_scenario_1.launch world:=*path_to_world*
