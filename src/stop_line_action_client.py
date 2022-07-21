#! /usr/bin/env python

from __future__ import print_function
import rospy

import actionlib

import igvc_action_server.msg
# import architecture.msg

def stop_line_client():
    
    client = actionlib.SimpleActionClient('stop_action',igvc_action_server.msg.stopBehaviourAction)
    
    client.wait_for_server()
    
    goal = igvc_action_server.msg.stopBehaviourGoal(stopBeforeDistance=1)
    # feedback = igvc_action_server.msg.stopBehaviourAction(flo)
    
    client.send_goal(goal)
    
    client.wait_for_result()
    rospy.loginfo("stop_line_action_client is running")
    rospy.spin()
    print(client.get_result())
    return client.get_result()
    # return client.get_result().success
    

if __name__ == '__main__':
    try:
        rospy.init_node('stop_line_action_client')
        result = stop_line_client()
        print("result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
