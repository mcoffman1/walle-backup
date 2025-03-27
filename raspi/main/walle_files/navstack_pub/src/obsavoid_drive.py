#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node('simple_navigation_goals')

    # Create an action client for the move_base action
    client = actionlib.SimpleActionClient('/move_base_simple/goal', MoveBaseAction)
    
    # Wait for the action server to come up
    rospy.loginfo("Waiting for the move_base action server to come up")
    client.wait_for_server()

    goal = MoveBaseGoal()

    # Set the goal target pose
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Hooray, the base moved 1 meter forward")
    else:
        rospy.loginfo("The base failed to move forward 1 meter for some reason")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

