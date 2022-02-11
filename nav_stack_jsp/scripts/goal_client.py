#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def user_goal_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    user_goal = goal_listener()  # block until goal entered from bash
    goal.target_pose.pose.position.x = float(user_goal[0])
    goal.target_pose.pose.position.y = float(user_goal[1])
    goal.target_pose.pose.orientation.w = 1.0;
    
    rospy.loginfo("Sending goal: " + str(user_goal[0]) + ", " + str(user_goal[1]))

    # send goal to action server
    client.send_goal(goal)

    # wait for server confirmation (blocks)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server unreachable!")
        rospy.signal_shutdown("Action server unreachable!")
    else:
        return client.get_result()


def goal_listener():
    print("Enter navigation goal as: 'x,y' (no space between values, separated by a coma)\n")
    while True:
        goal_input = input()
        goal_input = goal_input.split(",")

        try:
            x = int(goal_input[0])
            y = int(goal_input[1])
            return x, y
        except ValueError:
            print("Enter the values in the correct form!\n")
            continue


if __name__ == "__main__":
    try:
        rospy.init_node('user_goal_client_py')
        result = user_goal_client()
        if result:
            rospy.loginfo("Successfully moved to user goal.\n")
    except rospy.ROSInterruptException:
        rospy.loginfo("Killed process 'user_goal_client_py")





