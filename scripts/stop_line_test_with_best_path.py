#!/usr/bin/env python3

import rospy
from math import sqrt
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Distance:
    def __init__(self, obstacle_x, obstacle_y):
        
        self.dist_pub = rospy.Publisher('dm/distance', Float32, queue_size=30)
        self.path_pub = rospy.Publisher('/best_path', Path, queue_size = 10) #look into the queue size 
        self.car_x = None
        self.car_y = None
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y
        self.car_dist = None
        self.EPSILON = 0.01 #Tunable variable
        
        rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)
    
    def callback(self, data):
        #print(data.pose[data.name.index('vehicle')].position.x)
        self.car_x = data.pose[data.name.index('vehicle')].position.x
        self.car_y = data.pose[data.name.index('vehicle')].position.y
        self.distance()
        self.path()
    
    def distance(self):
        dist = sqrt((self.obstacle_x - self.car_x) ** 2 + (self.obstacle_y - self.car_y) ** 2)
        if self.car_dist == None:
            self.car_dist = dist
            self.dist_pub.publish(dist)
            print(dist, "Successfully Published")
            
        elif abs(self.car_dist - dist) > self.EPSILON:
            self.car_dist = dist
            self.pub.publish(dist)
            print(dist, "Successfully Published")

    def path_planner(self):
        vec_x = (self.obstacle_x - self.car_x)/10
        vec_y = (self.obstacle_y - self.car_y)/10
        path_list = [[None,None],[None,None],[None,None],[None,None],[None,None],[None],[None,None],[None,None],[None,None],[None,None]]
        for i in range(10):
            x_pub = self.car_x + vec_x*i
            y_pub = self.car_y + vec_y*i
            path_list[i][0] = x_pub
            path_list[i][1] = y_pub
        return(path_list)

    def path(self):
        msg = Path()
        msg.header.frame_id = "/map"
        msg.header.stamp = rospy.Time.now()
        path_list = self.path_planner()
        for i in path_list :
            pose = PoseStamped()
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            msg.poses.append(pose)
        self.path_pub.publish(msg)

if __name__ == "__main__":
    try :
        rospy.init_node("distance_publish")
        obs_x = float(input("Enter X coordinates of Obstacle "))
        obs_y = float(input("Enter Y coordinates of Obstacle "))
        dist = Distance(obs_x, obs_y)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
