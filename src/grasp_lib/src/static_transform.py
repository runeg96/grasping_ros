#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "ptu_camera_depth_optical_frame"
        static_transformStamped.child_frame_id = "task"

        static_transformStamped.transform.translation.x = 0.166
        static_transformStamped.transform.translation.y = 0.101
        static_transformStamped.transform.translation.z = 0.515

        # quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
        static_transformStamped.transform.rotation.x = -0.679
        static_transformStamped.transform.rotation.y = 0.726
        static_transformStamped.transform.rotation.z = -0.074
        static_transformStamped.transform.rotation.w = -0.081

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
