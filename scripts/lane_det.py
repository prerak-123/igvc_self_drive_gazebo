#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

class opencv_algo():
    def __init__(self):
        center_CamImg_topicname = "/camera_front/image_raw"
        occgrid_topicname = "/cv/laneoccgrid"

        rospy.Subscriber(center_CamImg_topicname, Image, self.cam_Callback)

        self.image = None

        self.occgrid_publisher = rospy.Publisher("occgrid_topicname", OccupancyGrid, queue_size = 1)
        self.rate = rospy.Rate(10)

    def cam_Callback(self, img_msg):
        bridge = CvBridge()
        try:
            print("running cv bridge")
            img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.image = img
            self.run_algo()
        except CvBridgeError as e:
            print(e)
            return
        print("shape of the img inside callback", self.image.shape)
        # cv2.imwrite("cam_img.png", img)
        # cv2.imshow('image',self.image)

    @staticmethod
    def make_coordinates(image, line_parameters):
        try:
            slope, intercept = line_parameters
        except TypeError:
            slope, intercept = 0, 0
        y1 = image.shape[0]
        y2 = int(y1 * (3 / 5))
        if slope != 0:
            x1 = int((y1 - intercept) / slope)
            x2 = int((y2 - intercept) / slope)
        else:
            x1 = 0
            x2 = 0
        return np.array([x1, y1, x2, y2])

    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, y1), (x2, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = self.make_coordinates(image, left_fit_average)
        right_line = self.make_coordinates(image, right_fit_average)
        return np.array([left_line, right_line])

    @staticmethod
    def canny(image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    @staticmethod
    def display_lines(image, lines):
        line_image = np.zeros_like(image)
        if lines is not None:
            count = 0
            c_values = []
            m_values = []
            x1_values = []
            y1_values = []
            x2_values = []
            y2_values = []
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                m = (y2 - y1) / (x2 - x1)
                c = y1 - (m * x1)
                # if m < 0.01 and m > -0.01:
                if m < 0.01 and m > -0.01:
                    x1_values.append(x1)
                    y1_values.append(y1)
                    x2_values.append(x2)
                    y2_values.append(y2)
                    m_values.append(m)
                    c_values.append(c)
                    count = count + 1

            c_values.sort()
            ic = 0
            x = 1
            for i in range(count):
                if i == 0:
                    cv2.putText(line_image, "Stop Line", (x1_values[i] - 10, y1_values[i] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
                    # cv2.line(line_image, (x1_values[i] - 10, y1_values[i]), (x2_values[i] + 10, y2_values[i]), (0, 255, 0),10)
                    cv2.rectangle(line_image, (x1_values[i] - 10, y1_values[i] - 15),
                                  (x2_values[i] + 10, y2_values[i] + 10), (0, 0, 255), 2)
                    print("Stop Line " + str(i + 1) + " : " + "y = " + str(m_values[i]) + "x + " + str(c_values[i]))
                    return line_image, [x1_values[i] - 10, y1_values[i] - 15, x2_values[i] + 10, y2_values[i] + 10]
        return line_image

    @staticmethod
    def region_of_interest(image):
        imshape = image.shape
        lower_left = [imshape[1] / 4, imshape[0]]
        lower_right = [imshape[1] - imshape[1] / 4, imshape[0]]
        top_left = [imshape[1] / 2 - imshape[1] / 10, imshape[0] / 2 + imshape[0] / 10]
        top_right = [imshape[1] / 2 + imshape[1] / 10, imshape[0] / 2 + imshape[0] / 10]
        vertices1 = [np.array([lower_left, top_left, top_right, lower_right], dtype=np.int32)]
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, vertices1, 255)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    @staticmethod
    def find_lane_pixels(image):
        histogram = np.sum(image[image.shape[0] // 2:, :], axis=0)
        out_img = np.dstack((image, image, image)) * 255
        midpoint = np.int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        nwindows = 9
        margin = 100
        minpix = 50
        window_height = np.int(image.shape[0] // nwindows)
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = image.shape[0] - (window + 1) * window_height
            win_y_high = image.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 4)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 4)

            # Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        return leftx, lefty, rightx, righty, out_img

    # Fit a poly to perform a directed search in well known areas
    @staticmethod
    def fit_poly(img_shape, leftx, lefty, rightx, righty):
        # Fit a second order polynomial to each with np.polyfit()
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        # Generate x and y values for plotting
        ploty = np.linspace(0, img_shape[0] - 1, img_shape[0])
        # Calc both polynomials using ploty, left_fit and right_fit
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        return left_fitx, right_fitx, ploty

    def search_around_poly(self, image):
        margin = 50

        # Grab activated pixels
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(image)
        if (len(leftx) == 0) or (len(rightx) == 0) or (len(righty) == 0) or (len(lefty) == 0):
            out_img = np.dstack((image, image, image)) * 255
            left_curverad = 0
            right_curverad = 0
        else:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            # Set the area of search based on activated x-values ###
            # within the +/- margin of our polynomial function ###
            left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy +
                                           left_fit[2] - margin)) & (nonzerox < (left_fit[0] * (nonzeroy ** 2) +
                                                                                 left_fit[1] * nonzeroy + left_fit[
                                                                                     2] + margin)))
            right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy +
                                            right_fit[2] - margin)) & (nonzerox < (right_fit[0] * (nonzeroy ** 2) +
                                                                                   right_fit[1] * nonzeroy + right_fit[
                                                                                       2] + margin)))

            # Again, extract left and right line pixel positions
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

            # Fit new polynomials
            left_fitx, right_fitx, ploty = self.fit_poly(image.shape, leftx, lefty, rightx, righty)

            ym_per_pix = 30 / 720  # meters per pixel in y dimension
            xm_per_pix = 3.7 / 650  # meters per pixel in x dimension

            # Calculate the curvature
            left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
            right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)
            y_eval = np.max(ploty)
            left_curverad = ((1 + (
                    2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
                2 * left_fit_cr[0])
            right_curverad = ((1 + (
                    2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
                2 * right_fit_cr[0])

            ## Visualization ##
            # Create an image to draw on and an image to show the selection window
            out_img = np.dstack((image, image, image)) * 255
            window_img = np.zeros_like(out_img)
            # Color in left and right line pixels
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

            # Generate and draw a poly to illustrate the lane area
            left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            points = np.hstack((left, right))
            out_img = cv2.fillPoly(out_img, np.int_(points), (0, 200, 255))

        return out_img, left_curverad, right_curverad

    def run_algo(self):
        print("shape of the img for self image RUN", self.image.shape)
        image = self.image
        print("shape of the img fro image RUN", image.shape)
        # image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)
        low_grey = np.array([245, 245, 245])
        high_grey = np.array([255, 255, 255])
        mask = cv2.inRange(image, low_grey, high_grey)
        c_image = cv2.bitwise_and(image, image, mask=mask)
        # cv2_imshow(c_image)
        lane_image = np.copy(c_image)
        canny_image = self.canny(lane_image)
        cropped_image = self.region_of_interest(canny_image)
        # cv2_imshow(cropped_image)
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=100, maxLineGap=30)
        # averaged_lines = self.average_slope_intercept(lane_image, lines)
        try:
            line_image, res_list = self.display_lines(lane_image, lines)
            print(res_list)
        except:
            pass

        image1 = self.image
        try:
            image1[res_list[1]:res_list[3], res_list[0]:res_list[2]] = 0
        except:
            pass
        # !BEWARE following parametes not configured for IPM, just an roi selector, dont use these for sedrica in
        # general, above ones can be used
        (h, w) = (image1.shape[0], image1.shape[1])  # 0.625
        # source = np.float32([[w // 2 -30 , h * .525], [w // 2 + 30, h * .525], [-100, h], [w + 100, h]])
        source = np.float32([[w // 2 - 150, h * .575], [w // 2 + 150, h * .575], [-200, h], [w + 200, h]])
        # destination = np.float32([[100, 0], [w - 100, 0], [100, h], [w - 100, h]])
        destination = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
        M = cv2.getPerspectiveTransform(source, destination)
        # M = cv2.getPerspectiveTransform(s, t)
        # invM = cv2.getPerspectiveTransform(t,s)
        invM = cv2.getPerspectiveTransform(destination, source)
        warped = cv2.warpPerspective(image1, M, (image1.shape[1], image1.shape[0]), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        img = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        ret, frame1 = cv2.threshold(img, 180, 256, cv2.THRESH_BINARY)
        kernel = np.ones((4, 4), np.uint8)
        frame1 = cv2.erode(frame1, kernel, iterations=1)
        frame1 = cv2.dilate(frame1, kernel, iterations=2)
        output = frame1
        resized_output = cv2.resize(output, (512,512))
        resized_warped = cv2.resize(warped, (512,512))

        # cv2.imshow("TEJU",resized_output)
        # cv2.imshow("TEJU1",resized_warped)

        cv2.waitKey(1)
        # occ grid seedha publish krskte yaha se
        Final_Grid = OccupancyGrid()
        Final_Grid.info.map_load_time = rospy.Time.now();
        Final_Grid.header.frame_id = "camera_front";
        Final_Grid.info.resolution = 0.05; #5cm per pixel
        Final_Grid.info.width = 400;
        Final_Grid.info.height = 600;

        Final_Grid.info.origin.position.x = 0;
        Final_Grid.info.origin.position.y = 0;
        Final_Grid.info.origin.position.z = 0;

        Final_Grid.info.origin.orientation.x = 0;
        Final_Grid.info.origin.orientation.y = 0;
        Final_Grid.info.origin.orientation.z = 0;
        Final_Grid.info.origin.orientation.w = 1;

        occ_grid_img = cv2.resize(output, (400,600))
        cv2.imshow("TEJU",occ_grid_img)
        occ_grid_img = cv2.flip(occ_grid_img, 0)
        occ_grid_img = (occ_grid_img//2) #- 127
        print("shape of occ grid$$$$$$$$", occ_grid_img.shape)
        occ_grid_img = np.array(occ_grid_img, dtype= "int8")
        temp = list(occ_grid_img.flatten())
        # temp = list(reversed(temp))

        for i in range(len(temp)):
            temp[i] = int(temp[i])

        Final_Grid.data = temp

        # final_grid = occ_grid_img

        # for i in range(final_grid.shape[0]):
        #     for j in range(final_grid.shape[1]):



        self.occgrid_publisher.publish(Final_Grid)
        print("occ grid is published!!!")
        self.rate.sleep()



        # return output


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
