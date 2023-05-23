#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(robot_name, goal_pose):
    # Create a SimpleActionClient for the move_base action server
    action_client = actionlib.SimpleActionClient(robot_name + "/move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    action_client.wait_for_server()

    # Create a goal object
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = goal_pose

    # Send the goal to the action server
    rospy.loginfo("Sending goal to " + robot_name + "...")
    action_client.send_goal(goal)

    # Wait for the robot to reach the goal
    action_client.wait_for_result()
    rospy.loginfo("Goal reached by " + robot_name)

if __name__ == '__main__':
    rospy.init_node('multi_rosbot_move')

    # Specify the goal pose for each robot (modify as needed)
    robot1_goal_pose = ...  # Specify the goal pose for robot 1
    robot2_goal_pose = ...  # Specify the goal pose for robot 2

    # Move robot 1 to the goal
    move_to_goal("robot1", robot1_goal_pose)

    # Move robot 2 to the goal
    move_to_goal("robot2", robot2_goal_pose)
