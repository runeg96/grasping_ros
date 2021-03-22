#! /usr/bin/env python


import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from grasp_lib.msg import Grasp

def gg_callback(msg):
    '''
    ggcnn: msg.data = [x, y, z, ang, width, depth_center]
    '''
    new_grasp = Grasp()
    new_grasp.name = "ggcnn"
    new_grasp.pose2D.x = msg.data[7]
    new_grasp.pose2D.y = msg.data[6]
    new_grasp.pose2D.theta = msg.data[3]
    new_grasp.width_meter = msg.data[4]/1000

    new_grasp.quality = msg.data[8]
    grasp_pub.publish(new_grasp)

def vgn_callback(msg):

    transform = tf_buffer.lookup_transform("ptu_camera_color_optical_frame",
                                       "task", #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

    pose_stamp = PoseStamped()
    pose_stamp.header.frame_id = "task"
    pose_stamp.pose = msg.pose

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamp, transform)

    msg.pose = pose_transformed.pose

    grasp_pub.publish(msg)


rospy.init_node("grasptype_parser_node")

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

grasp_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasp_topic"), Grasp, queue_size=1)

rospy.Subscriber('ggcnn/out/command', Float32MultiArray, gg_callback)
rospy.Subscriber("vgn/output", Grasp, vgn_callback)
rospy.spin()
