#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
from tkinter import W

import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import DBSCAN
from itertools import groupby
import numpy.polynomial.polynomial as poly
from igvc_self_drive_gazebo.msg import Task  # DM subsystem should change accordingly
from igvc_self_drive_gazebo.msg import CV  # DM subsystem should change accordingly

algo = AgglomerativeClustering(n_clusters=3, linkage='single')
import warnings

warnings.filterwarnings("ignore")
DEBUG = True



# ------------------------------------------------------------------------------------------------------------
class Stitcher:

    def __init__(self):
        self.homographies = [
            np.array([[-7.48133125e+00, 1.30233504e+00, 3.52516081e+03],
                      [-2.03382402e+00, -4.68397043e+00, 2.06584398e+03],
                      [-5.63899314e-03, 1.26338052e-04, 1.00000000e+00]]),
            np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            np.array([[3.37596043e-02, 8.19797200e-02, 1.21852515e+02],
                      [-1.25724107e-01, 3.33133522e-01, 2.44742035e+01],
                      [-3.52480814e-04, 3.07103124e-05, 3.91960568e-01]])
        ]
        # self.masks = m

    def stitch(self, left_image, front_image, right_image, h, w):
        images = [left_image, front_image, right_image]

        # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        # panorama_size = (4096, 4096)
        panorama_size = (w, h)
        blank = np.zeros((h, w, 3), np.uint8)
        translation = np.array(
            [[1, 0, (panorama_size[0] - left_image.shape[1]) / 2],
             [0, 1, (panorama_size[1] - left_image.shape[0]) / 2], [0, 0, 1]])
        # stitched_images = []
        for i, image in enumerate(images):
            temp = cv2.warpPerspective(
                image, np.matmul(translation, self.homographies[i]),
                panorama_size)
            blank = np.maximum(blank, temp)
            # stitched_images.append(temp)
            # cv2.imwrite(os.path.join(self.stitched_path,str(i+1)+".jpg"), temp)
        cv2.imshow("Blank", blank)
        # cropped = blank[1873:2794,:]
        # base = 1
        # balanced_images = self.exposureBalance(images)
        # blend = blender()
        # for i, image in enumerate(balanced_images):
        #     if (i != base):
        #         out1 = image
        #         out3 = masks[i]
        #         lpb = self.Laplacian_blending(out1,lpb,out3,5)
        # cv2.imshow(lpb)
        return blank

    def exposureBalance(self, images):
        balanced = []
        for i, img in enumerate(images):
            ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
            channels = cv2.split(ycrcb)
            cv2.equalizeHist(channels[0], channels[0])
            cv2.merge(channels, ycrcb)
            cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR, img)
            balanced += [img]
        return balanced

    def Laplacian_blending(self, img1, img2, mask, levels=4):
        G1 = img1.copy()
        G2 = img2.copy()
        GM = mask.copy()
        gp1 = [G1]
        gp2 = [G2]
        gpM = [GM]

        for i in range(levels - 1):
            G1 = cv2.pyrDown(G1)
            G2 = cv2.pyrDown(G2)
            GM = cv2.pyrDown(GM)
            gp1.append(np.float32(G1))
            gp2.append(np.float32(G2))
            gpM.append(np.float32(GM))

        lp1 = [gp1[levels - 1]]
        lp2 = [gp2[levels - 1]]
        gpMr = [gpM[levels - 1]]

        for i in range(levels - 1, 0, -1):
            L1 = np.subtract(gp1[i - 1], cv2.pyrUp(gp1[i]))
            L2 = np.subtract(gp2[i - 1], cv2.pyrUp(gp2[i]))
            lp1.append(L1)
            lp2.append(L2)
            gpMr.append(gpM[i - 1])

        LS = []
        for i, (l1, l2, gm) in enumerate(zip(lp1, lp2, gpMr)):
            ls = l1 * (gm) + l2 * (1 - gm)
            LS.append(ls)

        ls_ = LS[0]
        for i in range(1, levels):
            ls_ = cv2.pyrUp(ls_)
            ls_ = cv2.add(ls_, LS[i])
        return ls_


# -------------------------------------------------------------------------------------------------------------------------------


class opencv_algo:

    def __init__(self):
        center_CamImg_topic = '/camera_front/image_raw'
        left_CamImg_topic = '/camera_left/image_raw'
        right_CamImg_topic = '/camera_right/image_raw'
        occgrid_topic = '/cv/laneoccgrid'
        task_topic = '/dm/task'  # DM needs to publish this topic continuously
        cv_output_topic = '/cv/output'
        #self.pub = rospy.Publisher('cv/custom_msg', CV, queue_size=10)

        # to sync the input from all 3 cameras
        front_image_sub = message_filters.Subscriber(center_CamImg_topic,
                                                     Image)
        left_image_sub = message_filters.Subscriber(left_CamImg_topic, Image)
        right_image_sub = message_filters.Subscriber(right_CamImg_topic, Image)
        ts = message_filters.TimeSynchronizer(
            [front_image_sub, left_image_sub, right_image_sub], 10)
        ts.registerCallback(self.cam_Callback)

        task_topic_sub = rospy.Subscriber(task_topic, Task, self.task_callback)

        self.stitcher = Stitcher()
        self.stitching = False

        self.front_image = None
        self.left_image = None
        self.right_image = None

        self.task_list = {'Left': 0, 'Straight': 1, 'Right': 2} #Remember to update
        self.task = 0 # Set this to None when task topic has been set up

        self.curved_lane_threshold = 0.3
        self.binary_threshhold = 100

        self.Cam_Img_width = 1280
        self.Cam_Img_height = 720

        self.Occupancy_Grid = OccupancyGrid()
        self.Occupancy_Grid.header.frame_id = 'occgrid'
        self.Occupancy_Grid.info.resolution = 0.01  # 1cm per pixel
        self.Occupancy_Grid.info.width = 1280
        self.Occupancy_Grid.info.height = 720

        self.Occupancy_Grid.info.origin.position.x = 0
        self.Occupancy_Grid.info.origin.position.y = 0
        self.Occupancy_Grid.info.origin.position.z = 0

        self.Occupancy_Grid.info.origin.orientation.x = 0
        self.Occupancy_Grid.info.origin.orientation.y = 0
        self.Occupancy_Grid.info.origin.orientation.z = -0.707
        self.Occupancy_Grid.info.origin.orientation.w = 0.707

        self.OccupancyGrid_publisher = rospy.Publisher(occgrid_topic,
                                                       OccupancyGrid,
                                                       queue_size=1)
        self.Output_publisher = rospy.Publisher(cv_output_topic,
                                                CV,
                                                queue_size=1)
        self.rate = rospy.Rate(10)

    def task_callback(self, task):
        self.task = self.task_list[task.name]

    def cam_Callback(self, front_img_msg, left_img_msg, right_img_msg):
        bridge = CvBridge()
        try:
            front_img = bridge.imgmsg_to_cv2(front_img_msg, 'bgr8')
            left_img = bridge.imgmsg_to_cv2(left_img_msg, 'bgr8')
            right_img = bridge.imgmsg_to_cv2(right_img_msg, 'bgr8')
            self.front_image = front_img
            self.left_image = left_img
            self.right_image = right_img
            self.run_algo()
        except CvBridgeError as e:
            print(e)
            return

    def do_ipm(self, image1, **kwargs):
        (h, w) = (image1.shape[0], image1.shape[1])
        h1, w1 = kwargs.get("grid_height",
                            self.Occupancy_Grid.info.height), kwargs.get(
                                "grid_width", self.Occupancy_Grid.info.width)
        h2, w2 = self.Cam_Img_height, self.Cam_Img_width
        source = np.float32([[w // 2 - 150, h // 2 + 720 * .075],
                             [w // 2 + 150, h // 2 + 720 * .075],
                             [w // 2 - 1280 // 2 - 200, h // 2 + 720 // 2],
                             [w // 2 + 1280 // 2 + 200, h // 2 + 720 // 2]])
        destination = np.float32([[w1 // 2 - w2 // 2, h1 // 2 - h2 // 2],
                                  [w1 // 2 + w2 // 2, h1 // 2 - h2 // 2],
                                  [w1 // 2 - w2 // 2, h1 // 2 + h2 // 2],
                                  [w1 // 2 + w2 // 2, h1 // 2 + h2 // 2]])
        M = cv2.getPerspectiveTransform(source, destination)
        invM = cv2.getPerspectiveTransform(destination, source)
        warped = cv2.warpPerspective(image1,
                                     M, (w1, h1),
                                     flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=0)

        return warped

    def f(self, x, poly):
        ans = 0. 
        deg = len(poly)-1
        for i in range(len(poly)):
            ans += poly[deg-i]*(x**(deg-i))
        return ans


    def intersection_handling(self, image):
        print('Handling Intersection')

        EPS = 8  # hyperparameter used in Intersection Handling

        image = cv2.resize(image, (1280, 720), interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        warped = self.do_ipm(image)
        ret, image = cv2.threshold(warped, 230, 256, cv2.THRESH_BINARY)
        (h, w) = (image.shape[0], image.shape[1])
        new_image = np.zeros(image.shape)

        for i in range(h - 1, -1, -1):
            for j in range(w):
                if self.task == 2:
                    j = w - 1 - i
                if image[i][j] > 0:
                    new_image[i][j] = 1
                    break

        wheres = np.where(new_image > 0)
        x = np.array([np.array(wheres[0]), np.array(wheres[1])])
        x = x.transpose()

        dbscan = DBSCAN(eps=EPS, min_samples=1).fit(x)
        labels = dbscan.labels_

        cluster_len = np.zeros((max(labels) + 1, 1), dtype='int64')
        for i in range(len(wheres[0])):
            cluster_len[labels[i]] += 1
        label = np.argmax(cluster_len)

        final_image = np.zeros(image.shape)
        for i in range(x.shape[0]):
            if labels[i] == label:
                final_image[x[i][0]][x[i][1]] = 1

        x0 = []
        x1 = []
        x2 = []
        y0 = []
        y1 = []
        y2 = []

        if self.task == 0:
            y0 = x[:, 0]
            x0 = x[:, 1]
        elif self.task == 2:
            y2 = x[:, 0]
            x2 = x[:, 1]

        poly = np.polyfit(y0, x0, 4)
        poly_der = np.polyder(poly)

        # slopes = np.gradient(y0, x0)
        # where = np.isnan(slopes)
        # slopes[where] = 1000
        # s0 = slopes[0]
        # print(slopes)
        # for s in slopes:
        #     if abs(s - s0) > 0.2:
        #         m = -1 / s
        #         break

        # m = 0  # Safe Option
        # R = 1 / math.sqrt(1 + m * m)
        # if self.task == 0:
        #     D = 300 * R
        # else:
        #     D = -300 * R

        for i in range(len(x0)):
            # if abs(slopes[i]) > 0:
            #     m = -1 / slopes[i]
            # else :
            #     m = 1000
            m = self.f(y0[i], poly_der)

            R = 1 / math.sqrt(1 + m * m)
            if self.task == 0:
                D = 120 * R
            else:
                D = -120 * R

            print(math.atan(m)*(180/math.pi))
            x1.append(int(x0[i] + D))
            x2.append(int(x0[i] + 2 * D))
            y1.append(int(y0[i] + m * D))
            y2.append(int(y0[i] + 2 * m * D))
            if x1[-1] >= 0 and x1[-1] < w and y1[-1] >= 0 and y1[-1] < h:
                final_image[y1[-1]][x1[-1]] = 1
            if x2[-1] >= 0 and x2[-1] < w and y2[-1] >= 0 and y2[-1] < h:
                final_image[y2[-1]][x2[-1]] = 1

        if self.task == 2:
            x0, x2 = x2, x0
            y0, y2 = y2, y0

        return (x0, y0, x1, y1, x2, y2), final_image

    def straight_lane_detection(self, image):
        print('Handling Straight Lane')

        EPS = 8  # hyperparameter used in Straight Lane Handling

        image = cv2.resize(image, (1280, 720), interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        warped = self.do_ipm(image)
        ret, image = cv2.threshold(warped, 230, 256, cv2.THRESH_BINARY)
        (h, w) = (image.shape[0], image.shape[1])
        new_image = np.zeros(image.shape)

        for i in range(h - 1, -1, -1):
            for j in range(w):
                if image[i][j] > 0:
                    new_image[i][j] = 1
                    break

        wheres = np.where(new_image > 0)
        x = np.array([np.array(wheres[0]), np.array(wheres[1])])
        x = x.transpose()

        dbscan = DBSCAN(eps=EPS, min_samples=1).fit(x)
        labels = dbscan.labels_

        cluster_len = np.zeros((max(labels) + 1, 1), dtype='int64')
        for i in range(len(wheres[0])):
            cluster_len[labels[i]] += 1
        label = np.argmax(cluster_len)

        final_image = np.zeros(image.shape)
        for i in range(x.shape[0]):
            if labels[i] == label:
                final_image[x[i][0]][x[i][1]] = 1

        x0 = x[:, 1]
        x1 = []
        x2 = []
        y0 = x[:, 0]
        y1 = []
        y2 = []

        # get m
        m = 0
        slopes = np.gradient(y0, x0)
        mean_slope = np.nanmean(slopes)
        m = -1 / mean_slope
        D = 300
        R = 1 / math.sqrt(1 + m * m)
        for i in range(len(x0)):
            x1.append(x0[i] + D * R)
            x2.append(x0[i] + 2 * D * R)
            y1.append(y0[i] + D * m * R)
            y2.append(y0[i] + 2 * D * m * R)
            if int(x0[i] + D * R) >= 0 and int(x0[i] + D * R) < w and int(
                    y0[i] + D * m * R) >= 0 and int(y0[i] + D * m * R) < h:
                final_image[int(y0[i] + D * m * R)][int(x0[i] + D * R)] = 1
            if int(x0[i] +
                   2 * D * R) >= 0 and int(x0[i] + 2 * D * R) < w and int(
                       y0[i] + 2 * D * m * R) >= 0 and int(y0[i] +
                                                           2 * D * m * R) < h:
                final_image[int(y0[i] + 2 * D * m * R)][int(x0[i] +
                                                            2 * D * R)] = 1

        return (x0, y0, x1, y1, x2, y2), final_image

    def current_lane(self, x0, y0, x2, y2, w):
        cx = w / 2
        cy = 0
        idx = np.random.randint(len(y0))

        llv = x0[idx]
        rlv = x2[idx]
        right_dist = abs(rlv - cx)
        left_dist = abs(llv - cx)

        if right_dist < left_dist:
            print('Lane : ', 2)
            return 2
        else:
            print('Lane : ', 1)
            return 1

    # Will be modelled later
    # def stop_line_mask_print(self,image):
    #     #Prints stopline end diagonal co-ordinates & returns lane image with stoplines masked
    #     stopline_masked = stop_lines.lane_without_stoplines(image)

    #     return stopline_masked

    def publish_Occupancy_Grid(self, occ_grid_list):
        self.Occupancy_Grid.info.map_load_time = rospy.Time.now()
        self.Occupancy_Grid.data = np.array(
            [int(value*127) for value in occ_grid_list])
        self.OccupancyGrid_publisher.publish(self.Occupancy_Grid)

    def publish_CV_Outputs(self, occ_grid, binary_warped, x0, y0, x1, y1, x2,
                           y2):
        occ_grid = cv2.resize(occ_grid, (1280, 720))
        occ_grid = cv2.flip(occ_grid, 0)
        kernel = np.ones((5, 5), np.uint8)
        occ_grid = cv2.dilate(occ_grid, kernel, iterations=3)

        occ_grid = np.array(occ_grid, dtype='int8')
        occ_grid_dim = np.shape(occ_grid)
        occ_grid_dim = list(occ_grid_dim)
        occ_grid_list = list(occ_grid.flatten())
        occGrid = np.array([126 * int(value) for value in occ_grid_list])
        warped_list_dim = np.shape(binary_warped)
        warped_list_dim = list(warped_list_dim)
        warped_list = list(binary_warped.flatten())
        warped = np.array([int(value) for value in warped_list])


        x0 = list(map(int,x0))
        x1 = list(map(int,x1))
        x2 = list(map(int,x2))
        y0 = list(map(int,y0))
        y1 = list(map(int,y1))
        y2 = list(map(int,y2))

        output = CV()
        output.x0 = list(x0)
        output.x1 = list(x1)
        output.x2 = list(x2)
        output.y0 = list(y0)
        output.y1 = list(y1)
        output.y2 = list(y2)

        output.warped = list(warped)
        output.warped_shape = warped_list_dim

        output.occGrid = list(occGrid)
        output.occGrid_shape = occ_grid_dim

        self.publish_Occupancy_Grid(occ_grid_list)
        self.Output_publisher.publish(output)

    def run_algo(self):

        if self.stitching:
            image = self.stitcher.stitch(self.left_image,
                                         self.front_image,
                                         self.right_image,
                                         h=1000,
                                         w=2000)
        elif self.task == 0:
            image = self.left_image
        elif self.task == 2:
            image = self.right_image
        else:
            image = self.front_image

        if self.task == 0 or self.task == 2:
            (x0, y0, x1, y1, x2,
             y2), binary_warped = self.intersection_handling(image)
        elif self.task == 1:
            (x0, y0, x1, y1, x2,
             y2), binary_warped = self.straight_lane_detection(image)

        # x is rightwards from top left origin, y is downwards from top left origin
        if DEBUG:
            cv2.imshow('warped', binary_warped)

        ## When on straight lane predict the current lane
        if self.task == 1:
            current_lane = self.current_lane(x0, y0, x2, y2,
                                             binary_warped.shape[1])

        ## Get Lane Occupancy grid from birds eye view
        lane_occ_grid_img = binary_warped
        #print(x0,x1)
        ## Publish Lane Occupancy grid
        self.publish_CV_Outputs(lane_occ_grid_img, binary_warped, x0, y0, x1,
                                y1, x2, y2)

        cv2.waitKey(1)
        self.rate.sleep()


if __name__ == '__main__':
    print('[INFO] Starting the Node')
    try:
        print('[INFO] Initializing Node')
        rospy.init_node('CV Lane Detection Node')
        print('[INFO] Creating Object')
        obj = opencv_algo()
        print("[INFO] Using Camera Feed")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[INFO] Node Terminated.')
