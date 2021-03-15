#! /usr/bin/env python3

import os
import sys
import rospy

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../gr_grasp'))

import argparse
import logging

import numpy as np
import torch.utils.data

from grasp_utils.utils import pixel_to_camera
from ggcnn.msg import Grasp

from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation

from hardware.device import get_device
from inference.post_process import post_process_output
from utils.dataset_processing.grasp import detect_grasps

logging.basicConfig(level=logging.INFO)


def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate network')
    parser.add_argument('--network', type=str, default='../resources/epoch_17_iou_0.96',
                        help='Path to saved network to evaluate')
    parser.add_argument('--use-depth', type=int, default=1,
                        help='Use Depth image for evaluation (1/0)')
    parser.add_argument('--use-rgb', type=int, default=1,
                        help='Use RGB image for evaluation (1/0)')
    parser.add_argument('--n-grasps', type=int, default=1,
                        help='Number of grasps to consider per image')
    parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,
                        help='Force code to run in CPU mode')
    args, unknown = parser.parse_known_args()
    # args = parser.parse_args()
    return args

def numpy_to_torch(s):
    if len(s.shape) == 2:
        return torch.from_numpy(np.expand_dims(s, 0).astype(np.float32))
    else:
        return torch.from_numpy(s.astype(np.float32))

def parse_grasp_to_rviz(grasp_point, width, quality, angle):
        vis_grasp = Grasp()
        vis_grasp.pose.position.x = grasp_point[0]
        vis_grasp.pose.position.y = grasp_point[1]
        vis_grasp.pose.position.z = grasp_point[2]

        # Create a rotation object from Euler angles
        rot = Rotation.from_euler('xyz', [0, 0, angle])

        # Convert to quaternions
        q = rot.as_quat()

        vis_grasp.pose.orientation.x = q[0]
        vis_grasp.pose.orientation.y = q[1]
        vis_grasp.pose.orientation.z = q[2]
        vis_grasp.pose.orientation.w = q[3]
        vis_grasp.quality = quality
        vis_grasp.width = width
        grasp_pub.publish(vis_grasp)


def normalise(img):
    """
    Normalise the image by converting to float [0,1] and zero-centering
    """
    img = img.astype(np.float32) / 255.0
    img -= img.mean()
    return img


if __name__ == '__main__':
    args = parse_args()

    rospy.init_node("gr_grasp_ros")

    grasp_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasp_topic"), Grasp, queue_size=1)
    img_pub = rospy.Publisher("gr_grasp/image", Image, queue_size=1)

    cam_info = rospy.wait_for_message('ptu_camera/camera/color/camera_info', CameraInfo, timeout=rospy.Duration(1))
    
    # Load Network
    logging.info('Loading model...')
    net = torch.load(args.network)
    logging.info('Done')

    # Get the compute device
    device = get_device(args.force_cpu)

    while True:

        color_msg = rospy.wait_for_message('ptu_camera/camera/color/image_raw', Image, timeout=rospy.Duration(1))
        rgb_image = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1)
        rgb_img = normalise(rgb_image)

        depth_msg = rospy.wait_for_message('ptu_camera/camera/aligned_depth_to_color/image_raw', Image, timeout=rospy.Duration(1))
        depth_image = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width, -1)
        depth_img = normalise(depth_image)

        x =  numpy_to_torch(np.concatenate((np.expand_dims(depth_img.transpose(2,0,1), 0), np.expand_dims(rgb_img.transpose(2,0,1), 0)), 1))

        with torch.no_grad():
            xc = x.to(device)
            pred = net.predict(xc)

            q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
            grasps = detect_grasps(q_img, ang_img, width_img)
            depth = depth_image[grasps[0].center[0]][grasps[0].center[1]]
            grasp_point = pixel_to_camera(cam_info,(grasps[0].center[0],grasps[0].center[1]),depth/1000)

            parse_grasp_to_rviz(grasp_point, grasps[0].width, grasps[0].quality, grasps[0].angle)
