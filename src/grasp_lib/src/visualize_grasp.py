#! /usr/bin/env python

import rospy
import math
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import tf.msg

# from ggcnn.msg import Grasp
from grasp_lib.msg import Grasp

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from grasp_utils.utils import width_m_to_pixel, width_pixel_to_m, camera_to_pixel, pixel_to_camera


def draw_grasp(grasp, image, camInfo):
    Points = list()

    # Project point to image plane (grasp center)
    grasp_center = (grasp.pose2D.x, grasp.pose2D.y)
    Points.append(grasp_center)

    width_img = grasp.width_pixel
    yaw = grasp.pose

    # Calculate end points of gripper fingers (based on width and angle)
    Points.append((grasp_center[0]-math.cos(yaw)*width_img, grasp_center[1]-math.sin(yaw)*width_img))
    Points.append((grasp_center[0]+math.cos(yaw)*width_img, grasp_center[1]+math.sin(yaw)*width_img))

    # Calculate rectangle bounds
    x = (0.4 * width_img * math.cos(yaw-math.pi/2)*width_img) / width_img
    y = (0.4 * width_img * math.sin(yaw-math.pi/2)*width_img) / width_img
    Points.append((Points[1][0]+x, Points[1][1]+y))
    Points.append((Points[1][0]-x, Points[1][1]-y))
    Points.append((Points[2][0]-x, Points[2][1]-y))
    Points.append((Points[2][0]+x, Points[2][1]+y))

    # Draw points
    for point in Points[0:3]:
        point = (int(x) for x in point)

        point = list(point)
        #Make sure points stay within image
        point[0] = point[0] if point[0] < camInfo.width else camInfo.width -1
        point[1] = point[1] if point[1] < camInfo.height else camInfo.height -1
        point[0] = point[0] if point[0] > 0 else 1
        point[1] = point[1] if point[1] > 0 else 1

        cv.circle(image, tuple(point), 11, (0, 0, 255), -1)

    # Draw rectangle
    cv.drawContours(image, [np.array(Points[3:]).astype(int)], 0, (255,0,0), 2)

    im_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    img_msg = bridge.cv2_to_imgmsg(im_rgb, encoding="passthrough")
    image_pub.publish(img_msg)



def create_grasp_markers(new_grasp, type="gripper"):

    width_m = new_grasp.width_meter
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
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

def fill_grasp(grasp):
    global ci, depth
    
    # Construct 2D pose from 3D
    grasp_center = camera_to_pixel(ci, [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    grasp.pose2D.x = grasp_center[0]
    grasp.pose2D.y = grasp_center[1]

    angle = euler_from_quaternion([grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w])
    grasp.pose2D.theta = angle[2]


    # Construct 3D pose from 2D
    depth_m = depth[int(grasp.pose2D.x)][int(grasp.pose2D.x)] / 1000

    grasp_point = pixel_to_camera(ci,(grasp.pose2D.x, grasp.pose2D.x), depth_m)
    grasp.pose.x = grasp_point[0]
    grasp.pose.y = grasp_point[1]
    grasp.pose.z = grasp_point[2]

    rot = Rotation.from_euler('xyz', [0, 0, grasp.pose2D.theta])
    q = rot.as_quat()
    grasp.pose.orientation.x = q[0]
    grasp.pose.orientation.y = q[1]
    grasp.pose.orientation.z = q[2]
    grasp.pose.orientation.w = q[3]


    # Width conversions
    grasp.width_pixel = width_m_to_pixel(grasp.width_m, depth_m, ci) / 2

    grasp.width_meter = width_pixel_to_m(grasp.width_pixel, depth_m, ci)

    return grasp
    


def grasp_callback(msg):
    global img, depth, ci

    msg = fill_grasp(msg)

    # Setting up grasping frame
    t = TransformStamped()
    t.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'grasp'
    t.transform.translation = msg.pose.position

    q = msg.pose.orientation
    q_rot = quaternion_from_euler(math.pi/2, math.pi/2, 0)
    q_new = quaternion_multiply(q_rot, [q.x, q.y, q.z, q.w])

    t.transform.rotation.x = q_new[0]
    t.transform.rotation.y = q_new[1]
    t.transform.rotation.z = q_new[2]
    t.transform.rotation.w = q_new[3]

    tfm = tf.msg.tfMessage([t])
    tf_pub.publish(tfm)

    create_grasp_markers(msg)

    if rospy.get_param('~options/draw_image') and img is not None:
        draw_grasp(msg, img, ci)



if __name__ == '__main__':

    rospy.init_node('visualize_grasp')

    image_pub = rospy.Publisher(rospy.get_param('~output/image_points'), Image, queue_size=1)
    marker_pub = rospy.Publisher(rospy.get_param('~output/marker_topic'), Marker, queue_size=10)
    tf_pub = rospy.Publisher('/tf', tf.msg.tfMessage, queue_size=1)

    # Load camera info
    ci = rospy.wait_for_message(rospy.get_param('~camera/info_topic'), CameraInfo, timeout=None)

    img, depth = None, None

    # if draw_image = True
    if rospy.get_param('~options/draw_image'):
        bridge = CvBridge()

        rospy.Subscriber(rospy.get_param('~~camera/color_topic'), Image, image_callback)
        rospy.Subscriber(rospy.get_param('~~camera/depth_topic'), Image, depth_callback)

    rospy.Subscriber(rospy.get_param('~input/grasp_topic'), Grasp, grasp_callback)

    rospy.spin()
