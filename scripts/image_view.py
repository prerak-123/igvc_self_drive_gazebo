#!/usr/bin/env python

from __future__ import print_function

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 
import rospy
from sensor_msgs.msg import Image


class alphasense_view():
  def __init__(self):

    cam_img_topic = '/camera_front/image_raw'

    rospy.Subscriber(cam_img_topic, Image, self.cam_Callback)

    self.cv_image_0 = None
    self.cv_image_1 = None
    self.cv_image_2 = None
    self.cv_image_3 = None
    self.cv_image_4 = None
    self.cv_image_0_br = None
    self.cv_image_0_ar = None


  def cam_Callback(self, img_msg):
    # rospy.loginfo("START of PROCESS")
    bridge = CvBridge()
    try:
      cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
      print(e)
      return
   
    cv2.imwrite('cam_img.png',cv_img)

if __name__ == '__main__':
  try:
    rospy.init_node('image_view_node')
    alpha = alphasense_view()    
    rospy.spin()



  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")


