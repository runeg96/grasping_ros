#! /usr/bin/env python


import rospy

from tf.transformations import quaternion_from_euler

from std_msgs.msg import Float32MultiArray
from grasp_lib.msg import Grasp

def gg_callback(msg):
    '''
    ggcnn: msg.data = [x, y, z, ang, width, depth_center]
    '''

    new_grasp = Grasp()

    new_grasp.pose.position.x = msg.data[0]
    new_grasp.pose.position.y = msg.data[1]
    new_grasp.pose.position.z = msg.data[2]


    q = quaternion_from_euler(0.0, 0.0, msg.data[3])
    new_grasp.pose.orientation.x = q[0]
    new_grasp.pose.orientation.y = q[1]
    new_grasp.pose.orientation.z = q[2]
    new_grasp.pose.orientation.w = q[3]


    new_grasp.width_meters = msg.data[4]/1000

    new_grasp.quality = -1.0 # Unknown
    print(new_grasp)
    grasp_pub.publish(new_grasp)



rospy.init_node("grasptype_parser_node")

grasp_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasp_topic"), Grasp, queue_size=1)

rospy.Subscriber('ggcnn/out/command', Float32MultiArray, gg_callback)
rospy.spin()
