#! /usr/bin/env python3
""" ROS implementation of graspnet_baseline demo
    Author: Jan Kjaer Joorgensen and Rune
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse

import rospy

from PIL import Image

import torch
from graspnetAPI import GraspGroup
from ggcnn.msg import Grasp

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import CameraInfo as CamInfo
from sensor_msgs.msg import Image as Img

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../graspnet_baseline/models'))
sys.path.append(os.path.join(ROOT_DIR, '../../graspnet_baseline/dataset'))
sys.path.append(os.path.join(ROOT_DIR, '../../graspnet_baseline/utils'))

from graspnet import GraspNet, pred_decode
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--mask_path', required=True, help='Path to binary mask')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs, unknown = parser.parse_known_args()

def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data():
    # load data
    color_msg = rospy.wait_for_message('ptu_camera/camera/color/image_raw', Img, timeout=rospy.Duration(1))
    color_img = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(color_msg.height, color_msg.width, -1)

    depth_msg = rospy.wait_for_message('ptu_camera/camera/aligned_depth_to_color/image_raw', Img, timeout=rospy.Duration(1))
    depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)

    color = np.array((color_img), dtype=np.float32) / 255.0


    workspace_mask = np.array(Image.open(cfgs.mask_path), dtype=bool)

    cam_info = rospy.wait_for_message('ptu_camera/camera/color/camera_info', CamInfo, timeout=rospy.Duration(1))

    # generate cloud
    camera = CameraInfo(cam_info.width, cam_info.height, cam_info.K[0], cam_info.K[4], cam_info.K[2], cam_info.K[5], 1000)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:50]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])

def demo():
    global net
    end_points, cloud = get_and_process_data()
    gg = get_grasps(net, end_points)
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    gg.sort_by_score()
    print(gg[0])
    grasp_paser(gg[0])
    # vis_grasps(gg, cloud)

def grasp_paser(gg):
    vis_grasp = Grasp()
    vis_grasp.pose.position.x = gg.translation[0]
    vis_grasp.pose.position.y = gg.translation[1]
    vis_grasp.pose.position.z = gg.translation[2]

    r = R.from_matrix(gg.rotation_matrix)
    q = r.as_quat()

    vis_grasp.pose.orientation.x = q[0]
    vis_grasp.pose.orientation.y = q[1]
    vis_grasp.pose.orientation.z = q[2]
    vis_grasp.pose.orientation.w = q[3]
    vis_grasp.quality = gg.score
    vis_grasp.width = gg.width
    
    grasp_pub.publish(vis_grasp)

if __name__=='__main__':
    rospy.init_node('graspnet_baseline_ros')
    grasp_pub = rospy.Publisher(rospy.get_param("visualize_grasp/input/grasp_topic"), Grasp, queue_size=1)
    net = get_net()
    while not rospy.is_shutdown():
        demo()
