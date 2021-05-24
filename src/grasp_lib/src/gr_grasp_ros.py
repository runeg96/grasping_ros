#! /usr/bin/env python3
""" ROS implementation of gr_grasp
"""

import os
import sys
import rospy

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../gr_grasp'))

import argparse
import logging


import numpy as np
import torch.utils.data

from grasp_utils.utils import pixel_to_camera, width_pixel_to_m
from grasp_lib.msg import Grasp, Grasps

from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation

from hardware.device import get_device
from inference.post_process import post_process_output
from utils.dataset_processing.grasp import detect_grasps
from utils.data.camera_data import CameraData

import cv2


logging.basicConfig(level=logging.INFO)


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate network')
    parser.add_argument('--network', type=str, default='../resources/epoch_17_iou_0.96',
                        help='Path to saved network to evaluate')
    parser.add_argument('--use-depth', type=int, default=1,
                        help='Use Depth image for evaluation (1/0)')
    parser.add_argument('--use-rgb', type=int, default=0,
                        help='Use RGB image for evaluation (1/0)')
    parser.add_argument('--n-grasps', type=int, default=1,
                        help='Number of grasps to consider per image')
    parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,
                        help='Force code to run in CPU mode')
    args, unknown = parser.parse_known_args()
    # args = parser.parse_args()
    return args


def parse_grasp_to_rviz(grasp_center, width_pixel, quality, angle):
        vis_grasp = Grasp()
        vis_grasp.name = "gr"
        vis_grasp.pose2D.x = grasp_center[1] + 208
        vis_grasp.pose2D.y = grasp_center[0] + 128
        vis_grasp.pose2D.theta = - angle
        vis_grasp.width_pixel = width_pixel
        vis_grasp.quality = quality
        grasp_pub.publish(vis_grasp)


def parse_grasps_to_rviz(grasps):
    temp_grasp = Grasp()
    vis_grasps = Grasps()

    temp_grasp.name = "gr"

    for grasp in grasps:
        if grasp.quality > 0.3:
            temp_grasp.pose2D.x = grasp.center[1] + 208
            temp_grasp.pose2D.y = grasp.center[0] + 128
            temp_grasp.pose2D.theta = -grasp.angle

            temp_grasp.quality = grasp.quality
            temp_grasp.width_pixel = grasp.width

            vis_grasps.grasps.append(temp_grasp)

    grasps_pub.publish(vis_grasps)

def numpy_to_imgmsg(image, stamp=None):

    rosimage = Image()
    rosimage.height = image.shape[0]
    rosimage.width = image.shape[1]
    if image.dtype == np.uint8:
        rosimage.encoding = '8UC3'
        rosimage.data = image.ravel().tolist()
        rosimage.step = len(rosimage.data) // rosimage.height
    else:
        rosimage.encoding = '32FC1'
        rosimage.data = np.array(image.flat, dtype=np.float32).tostring()
        rosimage.step = len(rosimage.data) // rosimage.height
    if stamp is not None:
        rosimage.header.stamp = stamp
    return rosimage


if __name__ == '__main__':
    args = parse_args()

    rospy.init_node("gr_grasp_ros")
    grasp_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasp_topic"), Grasp, queue_size=1)
    grasps_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasps_topic"), Grasps, queue_size=1)
    grasp_img_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
    cam_data = CameraData(include_depth=args.use_depth, include_rgb=args.use_rgb)
    # Load Network
    logging.info('Loading model...')
    net = torch.load(args.network)
    logging.info('Done')

    # Get the compute device
    device = get_device(args.force_cpu)

    while not rospy.is_shutdown():

        color_msg = rospy.wait_for_message('ptu_camera/camera/color/image_raw', Image, timeout=rospy.Duration(1))
        rgb_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1)

        depth_msg = rospy.wait_for_message('ptu_camera/camera/aligned_depth_to_color/image_raw', Image, timeout=rospy.Duration(1))
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width,-1)
        depth_image = depth_image.astype(np.float32)/1000

        x, depth_img, rgb_img = cam_data.get_data(rgb=None, depth=depth_image)

        with torch.no_grad():
            xc = x.to(device)
            pred = net.predict(xc)

            q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
            q_img = np.clip(q_img, 0.0, 1.0-1e-3)
            grasps = detect_grasps(q_img, ang_img, width_img,args.n_grasps)

            grasp_img = cv2.applyColorMap((q_img*255).astype(np.uint8), cv2.COLORMAP_JET)

            grasp_img = numpy_to_imgmsg(grasp_img)
            grasp_img.header = depth_msg.header
            grasp_img_pub.publish(grasp_img)
            # print(grasps)
            try:
                parse_grasp_to_rviz(grasps[0].center,grasps[0].width, grasps[0].quality, grasps[0].angle)
                parse_grasps_to_rviz(grasps)
            except Exception as e:
                print(e)
