#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def broadcast_static(target_frame, source_frame):
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = target_frame
    msg.child_frame_id = source_frame
    msg.transform.translation.x = -0.15
    msg.transform.translation.y = 0.0
    msg.transform.translation.z = 1.0

    msg.transform.rotation.x = 0
    msg.transform.rotation.y = 0
    msg.transform.rotation.z = 0
    msg.transform.rotation.w = 1
    
    static_broadcaster.sendTransform(msg)


rospy.init_node('task_broadcaster_node')

static_broadcaster = tf2_ros.StaticTransformBroadcaster()

broadcast_static('world', 'task')

rospy.spin()

