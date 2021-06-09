#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2


def CB(msg):
    msg.header.stamp = rospy.Time.now()
    point_pub.publish(msg)


rospy.init_node("pointcloud_sub")
point_sub = rospy.Subscriber('/ptu_camera/camera/depth/color/points', PointCloud2, CB)
point_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=1)
rospy.spin()
