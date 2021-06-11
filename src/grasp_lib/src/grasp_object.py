#! /usr/bin/env python
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from robot_moveit_commander import robot_class as robot
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf
from math import pi



rospy.init_node('dummy_move')

listener = tf.TransformListener()
# Reset robot to an initial state (home and ready, gripper oppened)
robot_client = robot.Robot()

group = robot_client.right_arm

robot_client.rs_gripper.activateGripper()


while not rospy.is_shutdown():
    robot_client.rs_gripper.genCommand("open")
    robot_client.rs_gripper.genCommand("pinch")

    robot_client.setNamedTarget("ready", "right", move=True)
    current_pose = group.get_current_pose().pose

    try:
        listener.waitForTransform('world','grasp_ggcnn0',rospy.Time(0),rospy.Duration(5))
        (trans, rot) = listener.lookupTransform('world', 'grasp_ggcnn0', rospy.Time(0))
        print("trans: ",trans," rot: ",rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    robot_client.rs_gripper.genCommand("open")

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]+0.2

    angle = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
    angle = quaternion_from_euler(-pi/2, pi/2, angle[2]+pi/2)

    pose.orientation.x = angle[0]
    pose.orientation.y = angle[1]
    pose.orientation.z = angle[2]
    pose.orientation.w = angle[3]

    robot_client.rs_gripper.genCommand("open")

    waypoints = list()
    waypoints.append(pose)
    robot_client.linear_move("right", waypoints)

    # robot_client.setTarget(pose,"right")
    robot_client.rs_gripper.genCommand("open")

    waypoints = list()
    pose.position.z = trans[2] - 0.02
    waypoints.append(pose)
    robot_client.linear_move("right",waypoints)

    robot_client.rs_gripper.genCommand("close")
    robot_client.rs_gripper.genCommand("close")
    robot_client.rs_gripper.genCommand("close")
    rospy.sleep(2)

    waypoints = list()
    pose.position.z = trans[2] + 0.3
    waypoints.append(pose)
    robot_client.linear_move("right", waypoints)

    waypoints = list()
    pose.position.x = 0.5
    pose.position.y = 0.1
    pose.position.z = 1.3
    waypoints.append(pose)
    robot_client.linear_move("right", waypoints)

    robot_client.rs_gripper.genCommand("open")


# -0.69699
# 0.01604
# 0.71689
# 0.0033459
