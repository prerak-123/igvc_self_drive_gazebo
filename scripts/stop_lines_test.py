#!/usr/bin/env python

import rospy
from math import sqrt
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

class Distance:
    def __init__(self, obstacle_x, obstacle_y):
        
        self.pub = rospy.Publisher('dm/distance', Float32, queue_size=30)
        self.car_x = None
        self.car_y = None
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y
        
        self.rate = rospy.Rate(1000)
        
        rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)
    
    def callback(self, data):
        #print(data.pose[data.name.index('vehicle')].position.x)
        self.car_x = data.pose[data.name.index('vehicle')].position.x
        self.car_y = data.pose[data.name.index('vehicle')].position.y
        self.distance()
        self.rate.sleep()
    
    def distance(self):
        dist = sqrt((self.obstacle_x - self.car_x) ** 2 + (self.obstacle_y - self.car_y) ** 2)
        self.pub.publish(dist)
        print(dist, "Successfully Published")

if __name__ == "__main__":
    try :
        rospy.init_node("distance_publish")
        obs_x = float(input("Enter X coordinates of Obstacle "))
        obs_y = float(input("Enter Y coordinates of Obstacle "))
        dist = Distance(obs_x, obs_y)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")