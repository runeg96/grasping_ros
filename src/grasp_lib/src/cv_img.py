#! /usr/bin/env python
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("cv_img")

cvbridge = CvBridge()

msg = rospy.wait_for_message("/ptu_camera/camera/aligned_depth_to_color/image_raw",Image,rospy.Duration(1))
img = cvbridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
print(img.dtype)
cv.imwrite("Depth.png", img)

msg_color = rospy.wait_for_message("/ptu_camera/camera/color/image_raw", Image, rospy.Duration(1))
img_color = cvbridge.imgmsg_to_cv2(msg_color, desired_encoding="passthrough")
print(img_color.dtype)
cv.imwrite("Color.png", img_color)

msg_depth = rospy.wait_for_message("/ptu_camera/camera/depth/image_rect_raw", Image, rospy.Duration(1))
img_depth = cvbridge.imgmsg_to_cv2(msg_color, desired_encoding="passthrough")
print(img_depth.dtype)
cv.imwrite("Depth2.png", img_depth)

