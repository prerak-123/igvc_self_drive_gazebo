#!/usr/bin/env python
from __future__ import print_function
from pdb import line_prefix
from sre_constants import SUCCESS
import rospy
from geometry_msgs.msg import Twist
from obst_detect.msg import obstacle_detection_msg
from obst_distance.msg import obstacle_distance_msg
import stop_line_action_client
import lanechangingclient
import intersection_client
import igvc_action_server.msg
import yaml
#import yaml
from yaml.loader import SafeLoader

with open('task_sequence.yaml') as f:
    data = yaml.load(f, Loader=SafeLoader)
    print(data)

# condiition check function

task_dictionary = { "lane_following": lane_following,
                    "stop": stop_execution, 
                    "lane_changing": lane_changing
                    "intersection": intersection_execution
                    # TO BE COMPLETED (put all the action server functions here)
                    }

client_dictionary = { "stopline": stop_line_action_client
                      "lanechange": lanechangingclient
                      "intersection": intersection_client   
                     }

def master():
    def __init__(self, task_number_input):
        self.tasks_list = readYAML(task_number_input) # implement this function afterwards

        self.ongoing_task = None
        self.next_task = None
        self.obstacle_detection = obstacle_detection_msg()
        self.obstacle_distance = obstacle_distance_msg()

        obstacle_detection_subscriber = rospy.Subsriber("/CV/obstacle_detection", obstacle_detection_msg, obstacle_detection_CB)
        obstacle_distance_subscriber = rospy.Subsriber("/CV/obstacle_distance", obstacle_distance_msg, obstacle_distance_CB)


    def obstacle_distance_CB(self, msg):
        self.obstacle_distance = msg

    def obstacle_detection_CB(self, msg):
        self.obstacle_detection = msg

    def lane_following(self):
        pass
    
    def stop_execution(self, obstacle):   ############### not sure
        if obstacle_detection_msg.barrel == 1 or obstacle_detection_msg.stop_sign == 1 or obstacle_detection_msg.mannequin == 1:
            #### somehow make sure that the object(s) detected is same as that in task sequence
                if (obstacle_distance_msg.barrel != -1 and obstacle_distance_msg.barrel < threshold) or (obstacle_distance_msg.stopline != -1 and obstacle_distance_msg.stop_line < threshold) or (obstacle_distance_msg.mannequin != -1 and obstacle_distance_msg.mannequin < threshold):
                    return 1
        else:
            return 0

    def lane_changing(self):
        if obstacle_detection_msg.barrel == 1 or obstacle_detection_msg.mannequin == 1:
            if (obstacle_distance_msg.barrel != -1 and obstacle_distance_msg.barrel < threshold) or (obstacle_distance_msg.mannequin != -1 and obstacle_distance_msg.mannequin < threshold):
                return 1
        else:
            return 0

    def intersection_execution(self, turn_direction):
        if obstacle_detection_msg.intersection == 1:
            #### see if intersection is part of task sequence
                #### one way sign??? not sure
                    return 1
        else:
            return 0

    def run(self):

        while ((self.next_task)!="end"):

            self.ongoing_task = self.task_list.pop(0)
            self.next_task = self.task_list[1]

            while(!task_dictionary[self.next_task]): ## tis line checks whether the conditions for the next task have been satisfied or not, basically till all the conditions are not satisfied the current task keeps on being executed
                if task_dictionary[self.ongoing_task]:
                    ## here call the action server of the ongoing task, how?????????????
                else:
                    print("failsafe mode should be flagged")
                



            # check the condition for next task
                # go for the next iteration
            # else
                # rerun this iteration







