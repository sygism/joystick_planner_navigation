#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import mbf_msgs.msg as mbf_msgs
import actionlib
import time

robot_pose_x = 0
robot_pose_y = 0
path_client = None
prev_time = 0


def on_receive_joystick_inp(msg):
    global js_x, js_y, prev_time, path_client

    js_x = msg.linear.x
    js_y = msg.linear.y
    
    if (js_x != 0 or js_y != 0) and time.time() > prev_time + 5:
            tot_time = np.linspace(0, 5, 100)
            glob_x = robot_pose_x + (2 * -js_x) * tot_time
            glob_y = robot_pose_y + (2 * js_y) * tot_time

            path = Path()
            path.header.frame_id = "base_link"
            path.header.stamp = rospy.Time.now()

            for i in range(100):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "odom"
                pose.pose.position.x = glob_x[i]
                pose.pose.position.y = glob_y[i]
                pose.pose.position.z = 0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            
            path_client.send_goal(mbf_msgs.ExePathGoal(path=path))
            rospy.loginfo("## Set new path ##")
            prev_time = time.time()


def update_robot_position(data):
    global robot_pose_y, robot_pose_x

    robot_pose_x = data.pose.pose.position.x
    robot_pose_y = data.pose.pose.position.y


def start_planner():
    global path_client
    
    rospy.Subscriber("/controller_inp", Twist, on_receive_joystick_inp)
    rospy.Subscriber("/odometry/filtered", Odometry, update_robot_position)
    path_client = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
    rospy.spin() # start looping


if __name__ == "__main__":
    try:
        rospy.init_node('joystick_input_sub_py')
        start_planner()
    except rospy.ROSInterruptException:
        rospy.loginfo("Killed process 'joystick_input_sub_py")



