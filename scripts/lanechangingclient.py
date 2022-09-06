import rospy
import sys

import actionlib

import lane_changing.msg

def lanechanging_client():
    client = actionlib.SimpleActionClient('lanechanging', lane_changing.msg.lanechangingAction)
    client.wait_for_server()

    #give goal
    goal = lane_changing.msg.lanechangingGoal(lanechangeBeforeDistance = 10)

    client.send_goal(goal)
    client.wait_for_result
    rospy.loginfo("stop_line_action_client is running")
    n = client.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('lanechanging')
        result = lanechanging_client
        print("result:", result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file = sys.stderr)