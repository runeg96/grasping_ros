#! /usr/bin/env python
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import numpy as np

rospy.init_node("cv_img")

cvbridge = CvBridge()

here = os.path.dirname(os.path.abspath(__file__))
index = 0
view_counter = 0
view = ["side","head","top"]

while not rospy.is_shutdown():
    if view_counter == 3:
        view_counter = 0
        index += 1
    #wait for human input before taking new image
    raw_input("press enter to continue")
    
    #wait for depth image
    msg = rospy.wait_for_message("/ptu_camera/camera/aligned_depth_to_color/image_raw",Image,rospy.Duration(1))
    img = cvbridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")

    #save image in dataset folder
    depth_name = os.path.join(here, "../dataset/{}_{}_depth.png".format(index, view[view_counter]))
    depth_name_tiff = os.path.join(here, "../dataset/{}_{}_depth.tiff".format(index, view[view_counter]))

    cv.imwrite(depth_name, img)
    cv.imwrite(depth_name_tiff, img.astype(np.float32)/1000)

    #wait for color image
    msg_color = rospy.wait_for_message("/ptu_camera/camera/color/image_raw", Image, rospy.Duration(1))
    img_color = cvbridge.imgmsg_to_cv2(msg_color, desired_encoding="passthrough")
    
    #convert stupid cv image to RGB
    img_color = cv.cvtColor(img_color, cv.COLOR_BGR2RGB)

    #Save color image in dataset folder
    img_name = os.path.join(here, "../dataset/{}_{}_color.png".format(index, view[view_counter]))
    cv.imwrite(img_name, img_color)

    #Count of images
    
    view_counter += 1


