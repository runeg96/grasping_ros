#! /usr/bin/env python

import rospy
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import tf.msg

from ggcnn.msg import Grasp

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point
from tf.transformations import euler_from_quaternion

from grasp_utils.utils import width_m_to_pixel, camera_to_pixel


img = np.zeros((480,640))
depth = np.zeros((480,640))

def draw_grasp(grasp, image, depth, camInfo):
    Points = list()

    # Project point to image plane (grasp center)
    grasp_center = camera_to_pixel(camInfo, [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    Points.append(grasp_center)

    # Index depth and convert m to pixel (width)
    depth_m = depth[int(grasp_center[0])][int(grasp_center[1])]
    width_img = width_m_to_pixel(grasp.width, depth_m, camInfo) / 2

    # Get yaw from quaternion
    angle = euler_from_quaternion([grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w])
    yaw = angle[2]

    # Calculate end points of gripper fingers (based on width and angle)
    Points.append((grasp_center[0]-math.cos(yaw)*width_img, grasp_center[1]-math.sin(yaw)*width_img))
    Points.append((grasp_center[0]+math.cos(yaw)*width_img, grasp_center[1]+math.sin(yaw)*width_img))

    # Draw points
    for point in Points:
        point = (int(x) for x in point)
        cv.circle(image, tuple(point), 5, (0, 0, 255), -1)

    im_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    img_msg = bridge.cv2_to_imgmsg(im_rgb, encoding="passthrough")
    image_pub.publish(img_msg)



def create_grasp_markers(new_grasp, type="gripper"):

    width_m = new_grasp.width
    depth = 0.06

    grasp_marker = Marker()

    grasp_marker.header.frame_id = 'grasp'
    # grasp_marker.header.stamp = rospy.Time.now()
    grasp_marker.lifetime = rospy.Duration(0)
    grasp_marker.type = grasp_marker.ARROW
    grasp_marker.action = grasp_marker.ADD
    grasp_marker.ns = 'grasp_markers'

    grasp_marker.scale.x, grasp_marker.scale.y, grasp_marker.scale.z = 0.02, 0.03, 0.05
    grasp_marker.color.a = 1.0
    grasp_marker.pose.orientation.w = 1.0

    if type == "arrow":
        grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (1.0, 0.0, 0.0)

        grasp_marker.id = 1
        start = Point(0.10+depth, 0.0, 0.0)
        end = Point(depth, 0.0, 0.0)
        grasp_marker.points.append(start)
        grasp_marker.points.append(end)

        marker_pub.publish(grasp_marker)

        grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 0.0, 1.0)

        grasp_marker.id = 2
        start = Point(0.0, 0.1+width_m/2, 0.0)
        end = Point(depth, width_m/2, 0.0)
        grasp_marker.points[0] = start
        grasp_marker.points[1] = end

        marker_pub.publish(grasp_marker)

        grasp_marker.id = 3
        start = Point(0.0, -0.1-width_m/2, 0.0)
        end = Point(depth, -width_m/2, 0.0)
        grasp_marker.points[0] = start
        grasp_marker.points[1] = end

        marker_pub.publish(grasp_marker)

    elif type == "gripper":
        grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 1.0, 0.0)
        grasp_marker.type = grasp_marker.LINE_LIST
        grasp_marker.id = 4
        grasp_marker.points.append(Point(0.10+depth, 0.0, 0.0))
        grasp_marker.points.append(Point(depth, 0.0, 0.0))
        grasp_marker.points.append(Point(depth, width_m/2, 0.0))
        grasp_marker.points.append(Point(depth, -width_m/2, 0.0))
        grasp_marker.points.append(Point(depth, width_m/2, 0.0))
        grasp_marker.points.append(Point(0.0, width_m/2, 0.0))
        grasp_marker.points.append(Point(depth, -width_m/2, 0.0))
        grasp_marker.points.append(Point(0.0, -width_m/2, 0.0))

        marker_pub.publish(grasp_marker)


def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def depth_callback(msg):
    global depth
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def grasp_callback(msg):
    global img, depth, ci

    # Setting up grasping frame
    t = TransformStamped()
    t.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'grasp'
    t.transform.translation = msg.pose.position
    t.transform.rotation = msg.pose.orientation

    tfm = tf.msg.tfMessage([t])
    tf_pub.publish(tfm)

    create_grasp_markers(msg)

    if rospy.get_param('~options/draw_image'):
        draw_grasp(msg, img, depth, ci)



if __name__ == '__main__':

    rospy.init_node('visualize_grasp')

    image_pub = rospy.Publisher(rospy.get_param('~output/image_points'), Image, queue_size=1)
    marker_pub = rospy.Publisher(rospy.get_param('~output/marker_topic'), Marker, queue_size=10)
    tf_pub = rospy.Publisher('/tf', tf.msg.tfMessage, queue_size=1)

    # if draw_image = True
    if rospy.get_param('~options/draw_image'):
        bridge = CvBridge()

        # Load camera info
        ci = rospy.wait_for_message(rospy.get_param('~camera/info_topic'), CameraInfo, timeout=None)

        rospy.Subscriber(rospy.get_param('~~camera/color_topic'), Image, image_callback)
        rospy.Subscriber(rospy.get_param('~~camera/depth_topic'), Image, depth_callback)

    rospy.Subscriber(rospy.get_param('~input/grasp_topic'), Grasp, grasp_callback)

    rospy.spin()
