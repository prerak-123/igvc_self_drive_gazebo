#! /usr/bin/env python
import rospy
import sys
from __future__ import print_function
import actionlib
import intersection.msg #Depends on package name

def intersection_client(goal):
	client = actionlib.SimpleActionClient('intersection_turning', intersection.msg.IntersectionAction)
	client.wait_for_server()
	goal = intersection.msg.IntersectionGoal(goal = goal)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('Intersection_client')
        goal = int(input("ENTER THE DIRECTION YOU WANT TO TURN IN: "))
        result = intersection_client(goal)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
