#! /usr/bin/env python

import rospy
import math
import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from tf.transformations import euler_from_quaternion

from ggcnn.msg import Grasp


def draw_grasp(grasp, camModel, debug=False):
    Points = list()

    # Project point to image place (grasp center)
    grasp_center = PinholeCameraModel.project3dToPixel(camModel, [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    Points.append(grasp_center)

    # Get yaw from quaternion
    angle = euler_from_quaternion([grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w])
    yaw = angle[2]

    # Calculate end points of gripper fingers (based on width and angle)
    Points.append((grasp_center[0]-math.cos(yaw)*new_grasp.width, grasp_center[1]-math.sin(yaw)*new_grasp.width))
    Points.append((grasp_center[0]+math.cos(yaw)*new_grasp.width, grasp_center[1]+math.sin(yaw)*new_grasp.width))

    # Draw points
    for point in Points:
        point = (int(x) for x in point)
        cv.circle(img, tuple(point), 5, (0, 0, 255), -1)


    im_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

    img_msg = bridge.cv2_to_imgmsg(im_rgb, encoding="passthrough")
    image_pub.publish(img_msg)

    # Show images in opencv window for debug
    if debug:
        cv.imshow("grasp", im_rgb)
        cv.waitKey(10000)
        cv.destroyAllWindows()



if __name__ == '__main__':
    
    rospy.init_node('visualize_grasp')
    
    image_pub = rospy.Publisher(rospy.get_param('~output/image_points'), Image, queue_size=1)

    bridge = CvBridge()

    # Load camera info
    ci = rospy.wait_for_message(rospy.get_param('~camera/info_topic'), CameraInfo, timeout=None)
    cam = PinholeCameraModel()
    cam.fromCameraInfo(ci)
    # Load camera image
    img_msg = rospy.wait_for_message(rospy.get_param('~camera/color_topic'), Image, timeout=None)
    img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

    # rospy.Subscriber(rospy.get_param('~input/grasp_topic'), Grasp, grasp_callback)

    # Test grasp
    new_grasp = Grasp()

    new_grasp.pose.position.x = 0.0
    new_grasp.pose.position.y = 0.0
    new_grasp.pose.position.z = -0.06

    new_grasp.pose.orientation.x = 0.98
    new_grasp.pose.orientation.y = 0.18
    new_grasp.pose.orientation.z = 0
    new_grasp.pose.orientation.w = 0

    new_grasp.width = 100.0 # 0.26
    new_grasp.quality = 0.99


    draw_grasp(new_grasp, cam, debug=True)

 
