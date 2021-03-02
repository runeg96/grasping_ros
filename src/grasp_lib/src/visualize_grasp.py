#! /usr/bin/env python

import rospy
import math
import cv2 as cv
from cv_bridge import CvBridge
import tf.msg

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point
from image_geometry import PinholeCameraModel
from tf.transformations import euler_from_quaternion

from ggcnn.msg import Grasp


def draw_grasp(grasp, camModel, debug=False):
    Points = list()
    
    width_img = grasp.width * 600 / 2 # TODO fix pixel width
    
    # width_m = width_img / 300.0 * 2.0 * depth_crop * np.tan(self.cam_fov * self.img_crop_size/depth.shape[0] / 2.0 / 180.0 * np.pi)


    # Project point to image place (grasp center)
    grasp_center = PinholeCameraModel.project3dToPixel(camModel, [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    Points.append(grasp_center)

    # Get yaw from quaternion
    angle = euler_from_quaternion([grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w])
    yaw = angle[2]

    # Calculate end points of gripper fingers (based on width and angle)
    Points.append((grasp_center[0]-math.cos(yaw)*width_img, grasp_center[1]-math.sin(yaw)*width_img))
    Points.append((grasp_center[0]+math.cos(yaw)*width_img, grasp_center[1]+math.sin(yaw)*width_img))

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


def create_grasp_markers(new_grasp):
    width_m = new_grasp.width

    grasp_marker = Marker()
    
    grasp_marker.header.frame_id = 'grasp'
    grasp_marker.lifetime = rospy.Duration(0)
    grasp_marker.type = grasp_marker.ARROW
    grasp_marker.action = grasp_marker.ADD
    grasp_marker.ns = 'grasp_markers'
    
    grasp_marker.scale.x, grasp_marker.scale.y, grasp_marker.scale.z = 0.02, 0.03, 0.05
    grasp_marker.color.a = 1.0
    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (1.0, 0.0, 0.0)
    grasp_marker.pose.orientation.w = 1.0 
    
    grasp_marker.id = 1
    start = Point(0.12, 0.0, 0.0)
    end = Point(0.02, 0.0, 0.0)
    grasp_marker.points.append(start)
    grasp_marker.points.append(end)

    marker_pub.publish(grasp_marker)


    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 0.0, 1.0)

    grasp_marker.id = 2
    start = Point(0.0, 0.1+width_m/2, 0.0)
    end = Point(0.02, width_m/2, 0.0) 
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end

    marker_pub.publish(grasp_marker)

    grasp_marker.id = 3
    start = Point(0.0, -0.1-width_m/2, 0.0)
    end = Point(0.02, -width_m/2, 0.0) 
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end

    marker_pub.publish(grasp_marker)


def grasp_callback(msg):

    # Setting up grasping frame
    t = TransformStamped()
    t.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'grasp'
    t.transform.translation = msg.pose.position
    t.transform.rotation = msg.pose.orientation

    tfm = tf.msg.tfMessage([t])
    tf_pub.publish(tfm)

    draw_grasp(msg, cam, debug=False)
    create_grasp_markers(msg)



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

    rospy.Subscriber(rospy.get_param('~input/grasp_topic'), Grasp, grasp_callback)

    rospy.spin()
 
