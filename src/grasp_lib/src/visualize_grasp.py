#! /usr/bin/env python

import rospy
import math
import cv2 as cv
from cv_bridge import CvBridge
import tf.msg

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point
from image_geometry import PinholeCameraModel
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

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


def old_create_marker(grasp):
    ''' Old function for making grasp'''
    grasp_marker = Marker()
    grasp_marker.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    grasp_marker.type = grasp_marker.ARROW
    grasp_marker.action = grasp_marker.ADD
    
    grasp_marker.scale.x, grasp_marker.scale.y, grasp_marker.scale.z = 0.3, 0.05, 0.05
    grasp_marker.color.a = 1.0
    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 0.0, 1.0)
      
    grasp_marker.ns = 'grasp'
    grasp_marker.id = 1

    grasp_marker.pose = grasp.pose

    grasp_marker.lifetime = rospy.Duration(0)
    grasp_marker.header.stamp = rospy.get_rostime()

    q_orig = [grasp_marker.pose.orientation.x, grasp_marker.pose.orientation.y, grasp_marker.pose.orientation.z, grasp_marker.pose.orientation.w]

    marker_pub.publish(grasp_marker)


    q_rot_pos = quaternion_from_euler(0, 0, math.pi/2)
    q_rot_neg = quaternion_from_euler(0, 0, -math.pi/2)

    q_new = quaternion_multiply(q_rot_pos, q_orig)
    grasp_marker.pose.orientation.x, grasp_marker.pose.orientation.y, grasp_marker.pose.orientation.z, grasp_marker.pose.orientation.w = q_new
    grasp_marker.id = 2
    grasp_marker.pose.position.x -= 1.0
    marker_pub.publish(grasp_marker)

    q_new = quaternion_multiply(q_rot_neg, q_orig)
    grasp_marker.pose.orientation.x, grasp_marker.pose.orientation.y, grasp_marker.pose.orientation.z, grasp_marker.pose.orientation.w = q_new
    grasp_marker.id = 3
    marker_pub.publish(grasp_marker)

def create_marker():

    grasp_marker = Marker()
    grasp_marker.header.frame_id = 'grasp'
    grasp_marker.type = grasp_marker.ARROW
    grasp_marker.action = grasp_marker.ADD
    
    grasp_marker.scale.x, grasp_marker.scale.y, grasp_marker.scale.z = 0.02, 0.03, 0.05
    grasp_marker.color.a = 1.0
    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (1.0, 0.0, 0.0)
      
    grasp_marker.ns = 'grasp'
    grasp_marker.id = 1

    start = Point()
    end = Point()
    start.x, start.y, start.z = 0.12, 0.0, 0.0
    end.x, end.y, end.z = 0.02, 0.0, 0.0
    grasp_marker.points.append(start)
    grasp_marker.points.append(end)
    # grasp_marker.pose.position.x, grasp_marker.pose.position.y, grasp_marker.pose.position.z = 0.0, 0.0, 0.0
    grasp_marker.pose.orientation.x, grasp_marker.pose.orientation.y, grasp_marker.pose.orientation.z, grasp_marker.pose.orientation.w = 0.0, 0.0, 0.0, 1.0 

    grasp_marker.lifetime = rospy.Duration(0)
    grasp_marker.header.stamp = rospy.get_rostime()
    marker_pub.publish(grasp_marker)


    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 0.0, 1.0)
    start.x, start.y, start.z = 0.0, 0.126, 0.0
    end.x, end.y, end.z = 0.0, 0.026, 0.0    # TODO implement gripper width here
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end
    grasp_marker.id = 2
    marker_pub.publish(grasp_marker)

    start.x, start.y, start.z = 0.0, -0.126, 0.0
    end.x, end.y, end.z = 0.0, -0.026, 0.0   # TODO implement gripper width here
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end
    grasp_marker.id = 3
    marker_pub.publish(grasp_marker)
    
    


if __name__ == '__main__':
    
    rospy.init_node('visualize_grasp')
    
    image_pub = rospy.Publisher(rospy.get_param('~output/image_points'), Image, queue_size=1)
    marker_pub = rospy.Publisher(rospy.get_param('~output/marker_topic'), Marker, queue_size=10)
    tf_pub = rospy.Publisher('/tf', tf.msg.tfMessage, queue_size=1)

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

    # Setting up grasping frame
    t = TransformStamped()
    t.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'grasp'
    t.transform.translation.x = new_grasp.pose.position.x
    t.transform.translation.y = new_grasp.pose.position.y
    t.transform.translation.z = new_grasp.pose.position.z

    t.transform.rotation.x = new_grasp.pose.orientation.x
    t.transform.rotation.y = new_grasp.pose.orientation.y
    t.transform.rotation.z = new_grasp.pose.orientation.z
    t.transform.rotation.w = new_grasp.pose.orientation.w

    tfm = tf.msg.tfMessage([t])
    tf_pub.publish(tfm)

    draw_grasp(new_grasp, cam, debug=True)
    # old_create_marker(new_grasp)
    create_marker()

    rospy.spin()
 
