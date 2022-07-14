#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import AgglomerativeClustering
from igvc_self_drive_gazebo.msg import CV

algo = AgglomerativeClustering(3)

class opencv_algo():
    def __init__(self):
        center_CamImg_topic = "/camera_front/image_raw"
        occgrid_topic = "/cv/laneoccgrid"
        local_goal_topic = "/move_base_simple/goal_cv"

        rospy.Subscriber(center_CamImg_topic, Image, self.cam_Callback)

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

    def cam_Callback(self, img_msg):
        bridge = CvBridge()
        try:
            # print("running cv bridge")
            img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.image = img
            self.run_algo()
        except CvBridgeError as e:
            print(e)
            return
        # print("shape of the img inside callback", self.image.shape)
        # cv2.imwrite("cam_img.png", img)
        # cv2.imshow('image',self.image)

    def do_ipm(self, image1):
        (h, w) = (image1.shape[0], image1.shape[1]) 
        source = np.float32([[w // 2 -150 , h * .575], [w // 2 + 150, h * .575], [-200, h], [w+200 , h]])
        destination = np.float32([[0, 0], [w , 0], [0, h], [w, h]])
        M = cv2.getPerspectiveTransform(source, destination)
        invM = cv2.getPerspectiveTransform(destination,source)
        warped = cv2.warpPerspective(image1, M, (image1.shape[1], image1.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)

        return warped

    def get_occ_grid_image(self,image):
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, frame1 = cv2.threshold(img, 180, 256, cv2.THRESH_BINARY)
        # cv2_imshow(frame1)
        kernel = np.ones((4,4), np.uint8)
        frame1 = cv2.erode(frame1, kernel, iterations=3)
        # cv2_imshow(frame1)
        frame1 = cv2.dilate(frame1, kernel, iterations=2)

        return frame1

    def getlane_pixels(self,image):
        frame1 = cv2.ximgproc.thinning(image, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        wheres = np.where(frame1 > 150)
        # print(wheres[0].shape)
        clustered = algo.fit_predict(wheres[1].reshape([-1, 1]))

        mark = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

        x0 =  wheres[1][clustered == 0]
        y0 = wheres[0][clustered == 0]
        # print(mark[x0,y0])
        mark[y0,x0,:] = [255,0,0] # blue the leftmost
        # cv2.imshow("TEJU2a",mark)

        x1 =  wheres[1][clustered == 1]
        y1 = wheres[0][clustered == 1]

        mark[y1,x1] = [0,255,0] # the green- rightmost
        # cv2.imshow("TEJU2b",mark)

        x2 =  wheres[1][clustered == 2]
        y2 = wheres[0][clustered == 2]

        mark[y2,x2] = [0,0,255] # the red - middle marking
        cv2.imshow("TEJU2c",mark)

        return x0,y0,x1,y1,x2,y2

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

        image21 = cv2.circle(warped,(int(new_x_offset[localgoal_ind]),int(new_y_offset[localgoal_ind])),1,(0,255,255),15)
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

    def publishLaneOccGrid(self, image):
        self.Final_Grid.info.map_load_time = rospy.Time.now()

        occ_grid_img = cv2.resize(image, (1280,720))

        # cv2.imshow("TEJU",occ_grid_img)

        occ_grid_img = cv2.flip(occ_grid_img, 0)
        occ_grid_img = (occ_grid_img//2) #- 127
        # print("shape of occ grid$$$$$$$$", occ_grid_img.shape)
        occ_grid_img = np.array(occ_grid_img, dtype= "int8")
        temp = list(occ_grid_img.flatten())
        # temp = list(reversed(temp))

        for i in range(len(temp)):
            temp[i] = int(temp[i])

        self.Final_Grid.data = temp

        self.occgrid_publisher.publish(self.Final_Grid)
        # print("occ grid is published!!!")


    def run_algo(self):
        # cv2.imshow("TEJU", self.image)
        # print("@@@@@@@@@@@@@@@@",self.image.shape)
        warped = self.do_ipm(self.image)
        # cv2.imshow("TEJU1", warped)
        lane_occ_grid_img = self.get_occ_grid_image(warped)

        # publish lane occ grid
        self.publishLaneOccGrid(lane_occ_grid_img)

        x0,y0,x1,y1,x2,y2 = self.getlane_pixels(lane_occ_grid_img)
        print(x1)
        localgoal_x, localgoal_y = self.markLocalGoal(x1,y1,warped)

        # publish local goal
        self.publishLocalGoal(localgoal_x, localgoal_y)

        cv2.waitKey(1)
        self.rate.sleep()


if __name__ == '__main__':
    print("starting the node")
    try:
        print("initialinzing node")
        rospy.init_node("laneDetection_and_laneOccGrid_pubNode")
        print("creating obejct")
        obj = opencv_algo()
        print("entering while loop")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
