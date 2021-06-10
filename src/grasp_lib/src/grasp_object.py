#! /usr/bin/env python
from robot_moveit_commander import robot_class as robot
import rospy
from geometry_msgs.msg import PoseStamped, Pose


rospy.init_node('dummy_move')

# Reset robot to an initial state (home and ready, gripper oppened)
robot_client = robot.Robot()

group = robot_client.right_arm
current_pose = group.get_current_pose().pose

robot_client.rs_gripper.activateGripper()
print(current_pose)

robot_client.rs_gripper.genCommand("close")

robot_client.setNamedTarget("home", "right", move=True)
