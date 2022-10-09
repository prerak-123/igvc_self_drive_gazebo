#!/usr/bin/env python3
from __future__ import print_function

import numpy as np
import cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import AgglomerativeClustering
from igvc_self_drive_gazebo.msg import CV

algo = AgglomerativeClustering(3)

class DMNode():
    def __init__(self):
        rospy.Subscriber('/cv/output', CV, self.callback)
        #rospy.Subscriber('/cv/custom_msg', CV, self.callback)
        occgrid_topic = "/cv/laneoccgrid"
        local_goal_topic = "/move_base_simple/goal_cv"
        
        self.image = None

        self.local_goal = PoseStamped()
        self.local_goal.header.frame_id = "occgrid"

        self.Final_Grid = OccupancyGrid()
        self.Final_Grid.header.frame_id = "occgrid";
        self.Final_Grid.info.resolution = 0.01; #1cm per pixel
        self.Final_Grid.info.width = 1280;
        self.Final_Grid.info.height = 720;

        self.Final_Grid.info.origin.position.x = 0;
        self.Final_Grid.info.origin.position.y = 0;
        self.Final_Grid.info.origin.position.z = 0;

        self.Final_Grid.info.origin.orientation.x = 0;
        self.Final_Grid.info.origin.orientation.y = 0;
        self.Final_Grid.info.origin.orientation.z = -0.707;
        self.Final_Grid.info.origin.orientation.w = 0.707;

        self.occgrid_publisher = rospy.Publisher(occgrid_topic, OccupancyGrid, queue_size = 1)
        self.local_goal_publisher = rospy.Publisher(local_goal_topic, PoseStamped , queue_size =1)
        self.rate = rospy.Rate(10)
        
        self.warped = None
        self.lane_occ_grid_img = None
        
        self.x0 = None
        self.y0 = None
        
        self.x1 = None
        self.y1 = None
        
        self.x2 = None
        self.y2 = None

    def callback(self, data):
        self.warped = np.array(data.warped, dtype="uint8")
        self.warped = np.reshape(self.warped, tuple(data.warped_shape))
        
        self.lane_occ_grid_img = np.array(data.occGrid, dtype="uint8")
        self.lane_occ_grid_img = np.reshape(self.lane_occ_grid_img, tuple(data.occGrid_shape))
        
        self.x0 = np.array(data.x0, dtype='uint32')
        self.x1 = np.array(data.x1, dtype='uint32')
        self.x2 = np.array(data.x2, dtype='uint32')
        self.y0 = np.array(data.y0, dtype='uint32')
        self.y1 = np.array(data.y1, dtype='uint32')
        self.y2 = np.array(data.y2, dtype='uint32')
        
        # for i in range(len(self.x1)):
        #     print(self.lane_occ_grid_img[self.x1[i], self.y1[i]])
        
        self.run_algo()
    
    def markLocalGoal(self, x_array,y_array, warped):
        coefficients = np.polyfit(y_array, x_array, 7)
        # print(coefficients)

        poly = np.poly1d(coefficients)
        new_y = np.linspace(y_array[0], y_array[-1])
        new_x = poly(new_y)

        # fig, ax = plt.subplots()
        # ax.imshow(warped)

        # plt.savefig("line.jpg")

        dist = -150 #-130   # tunable parameter!  
        poly_derivative = np.polyder(poly)
        new_y_offset = new_y - ( dist * np.sin(np.arctan(poly_derivative(new_y))) )
        new_x_offset = new_x + ( dist * np.cos(np.arctan(poly_derivative(new_y))) )


        # ax.plot(new_x_offset, new_y_offset, '--', linewidth=1, color='green')
        # plt.savefig("final.jpg")

        localgoal_ind = np.argmin(np.absolute(new_y_offset))
        
        # print("new_x_offset", new_x_offset)
        # print("new_y_offset", new_y_offset)
        # print(int(new_y_offset[localgoal_ind]),int(new_x_offset[localgoal_ind]))
        warped = warped*200
        warped = cv2.cvtColor(warped, cv2.COLOR_GRAY2RGB)
        #print(int(new_x_offset[localgoal_ind]), int(new_y_offset[localgoal_ind]))
        image21 = cv2.circle(warped,(int(new_x_offset[localgoal_ind]),int(new_y_offset[localgoal_ind])),1,(0,255,255),15)
        
        for i in range(len(self.x0)):
            image21 = cv2.circle(image21,(self.x0[i], self.y0[i]),1,(0,0,255),5)
            image21 = cv2.circle(image21,(self.x1[i], self.y1[i]),1,(0,255,0),5)
            image21 = cv2.circle(image21,(self.x2[i], self.y2[i]),1,(255,0,0),5)
            
        cv2.imshow("TEJU2",image21)

        return int(new_x_offset[localgoal_ind]), int(new_y_offset[localgoal_ind])

    def publishLocalGoal(self, x , y):
        self.local_goal.header.stamp = rospy.Time.now()
        self.local_goal.pose.position.x = (720 - y) * self.Final_Grid.info.resolution #- x * self.Final_Grid.info.resolution
        self.local_goal.pose.position.y = - x * self.Final_Grid.info.resolution #(720 - y) * self.Final_Grid.info.resolution
        self.local_goal.pose.position.z = 0
        self.local_goal.pose.orientation.x = 0
        self.local_goal.pose.orientation.y = 0
        self.local_goal.pose.orientation.z = 0
        self.local_goal.pose.orientation.w = 1

        self.local_goal_publisher.publish(self.local_goal)
        
    def run_algo(self):
        localgoal_x, localgoal_y = self.markLocalGoal(self.x1,self.y1,self.warped)

        # publish local goal
        self.publishLocalGoal(localgoal_x, localgoal_y)

        cv2.waitKey(1)
        self.rate.sleep()
        pass
        
if __name__ == '__main__':
    print("starting the node")
    try:
        print("initialinzing node")
        rospy.init_node("DMNode")
        print("creating obejct")
        obj = DMNode()
        print("entering while loop")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
