#! /usr/bin/env python3

import os
import sys
import rospy

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../gr_grasp'))

import argparse
import logging

import matplotlib.pyplot as plt
import numpy as np
import torch.utils.data

from sensor_msgs.msg import Image

# from hardware.camera import RealSenseCamera
from hardware.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.visualisation.plot import save_results, plot_results
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
    parser.add_argument('--n-grasps', type=int, default=10,
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
    # # Connect to Camera
    # logging.info('Connecting to camera...')
    # cam = RealSenseCamera(device_id=830112070066)
    # cam.connect()
    # cam_data = CameraData(include_depth=args.use_depth, include_rgb=args.use_rgb)

    # Load Network
    logging.info('Loading model...')
    net = torch.load(args.network)
    logging.info('Done')

    # Get the compute device
    device = get_device(args.force_cpu)

    try:
        fig = plt.figure(figsize=(10, 10))
        while True:
            # image_bundle = cam.get_image_bundle()
            # rgb = image_bundle['rgb']
            # depth = image_bundle['aligned_depth']
            # x, depth_img, rgb_img = cam_data.get_data(rgb=rgb, depth=depth)

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
                print(grasps[0].center[0],grasps[0].center[1])
                
                plot_results(fig=fig,
                             rgb_img=rgb_image,
                             depth_img=depth_image,
                             grasp_q_img=q_img,
                             grasp_angle_img=ang_img,
                             no_grasps=args.n_grasps,
                             grasp_width_img=width_img)
    finally:
        save_results(
            rgb_img=rgb_image,
            depth_img=depth_image,
            grasp_q_img=q_img,
            grasp_angle_img=ang_img,
            no_grasps=args.n_grasps,
            grasp_width_img=width_img
        )
