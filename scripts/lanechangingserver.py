#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import String, Float64
import lane_changing.msg #is there a custom messge type for this?? change cmakelists for this

 #may have to integrate using distance from stop line action server
#make subscriber take distance from stop line distance node
class lanechanging(object):
    #cvObstacleDistance = 10.0
    feedback = lane_changing.msg.lanechangingFeedback()
    result = lane_changing.msg.lanechangingResult()

    def __init__(self, lanechanging):
        self.cvObstacleDistance = 10.0
        self.steering_angle = None
        self.action_name = lanechanging
        self.action_server = actionlib.SimpleActionServer(self.action_name, lane_changing.msg.lanechangingAction, 
                                                          execute_cb = self.server_callback, auto_start = False)
        self.cv_obstacle_distance = rospy.Subscriber("dm/distance", Float64, self.obstacle_distance) #not exactly sure if keeping a subscriber like this will work for a server. need verification
        self.action_server.start()

    def server_callback(self, goal):
        lane_pub = rospy.Publisher('which_lane_offset', String, queue_size=10) #may have to change topic names
        rospy.init_node('Lane_change_server', anonymous=True)
        mid_lane_pub = rospy.Publisher('mid_lane_status', String, queue_size=10)

        #rospy.init_node('mid_lane_status', anonymous=True) #why initialise a node twice

        r = rospy.Rate(1)
        success = True 

        ## can inititalize here
        # lane_algo = lane_det2.opencv_algo()
        '''the if self.action_server.is_preempt_requested() seems rather confusing to me- need help to understand'''
        while self.cvObstacleDistance > goal.lanechangeBeforeDistance:
            if self.action_server.is_preempt_requested():
                rospy.loginfo("%s Preempted" % self.action_name)
                self.action_server.set_preempted()
                success = False
                break
                #current lane to be given as feedback - ask CV
                #
            else:
                lane_pub.publish("left")                                   ## turn off middle lane by making some 255 to 0 in run_algo of lane_det2
                mid_lane_pub.publish("off")                                ## make offset by the left most lane and change sign of offset

                self.feedback.dist_to_obstacle = self.cvObstacleDistance
                ## somehow give steering angle & current lane????????????
                self.feedback.steering_angle = rospy.Subscriber("steering_cmd", Float64, self.steering_callback) #make steering_callback function
                      ######################what is the type of the message and the callback function?
        
            ## put rospy.loginfo if you want
            #define steering callback function
            #define self.obstacle_distance distance
            self.action_server.publish_feedback(self.feedback)

            r.sleep()

        if success:
            ## put result here
            self.result.current_lane = self.feedback.current_lane
            self.feedback.dist_to_obstacle = self.feedback.dist_to_obstacle
            ## put rospy.loginfo here
            self.action_server.set_succeeded(self.result)    ## result variable

        def obstacle_distance(self, msg):
            self.cvObstacleDistance = msg

        def steering_callback(self, msg):
            self.steering_angle = msg

if __name__ == "__main__":
    rospy.init.node("lanechanging")
    server = lanechanging(rospy.get_name())
    rospy.spin()



