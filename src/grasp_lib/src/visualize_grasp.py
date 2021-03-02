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
    grasp_marker.pose.orientation.w = 1.0 

    grasp_marker.lifetime = rospy.Duration(0)
    grasp_marker.header.stamp = rospy.Time.now()
    marker_pub.publish(grasp_marker)


    grasp_marker.color.r, grasp_marker.color.g, grasp_marker.color.b = (0.0, 0.0, 1.0)
    start.x, start.y, start.z = 0.0, 0.1+width_m/2, 0.0
    end.x, end.y, end.z = 0.0, width_m/2, 0.0   
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end
    grasp_marker.id = 2
    marker_pub.publish(grasp_marker)

    start.x, start.y, start.z = 0.0, -0.1-width_m/2, 0.0
    end.x, end.y, end.z = 0.0, -width_m/2, 0.0   
    grasp_marker.points[0] = start
    grasp_marker.points[1] = end
    grasp_marker.id = 3
    marker_pub.publish(grasp_marker)
    
def grasp_callback(msg):

    # Setting up grasping frame
    t = TransformStamped()
    t.header.frame_id = rospy.get_param('~grasp/grasp_frame')
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = 'grasp'
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

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

    # Test grasp
    # new_grasp = Grasp()

    # new_grasp.pose.position.x = 0.0
    # new_grasp.pose.position.y = 0.0
    # new_grasp.pose.position.z = -0.06

    # new_grasp.pose.orientation.x = 0.98
    # new_grasp.pose.orientation.y = 0.18
    # new_grasp.pose.orientation.z = 0
    # new_grasp.pose.orientation.w = 0

    # new_grasp.width = 0.26
    # new_grasp.quality = 0.99
   


    # draw_grasp(new_grasp, cam, debug=True)
    # create_grasp_markers(new_grasp)

    rospy.spin()
 
