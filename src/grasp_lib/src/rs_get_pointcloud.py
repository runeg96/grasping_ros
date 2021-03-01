#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from pypcd import pypcd

from gpd_ros.msg import CloudSamples, CloudSources
from sensor_msgs.msg import Image

from vision_utils.img import image_to_numpy
from vision_utils.logger import get_logger

logger = get_logger()


def get_mask_index(mask):
    """Get pixel indices of binary image (segmask).

    Keyword arguments:
    mask   --  np.array Binary image (segmask)
    """
    mask = np.swapaxes(mask, 0, 1)
    idx = list(zip(*np.where(mask > 0)))
    return idx


def segment_cloud(cloud, pixels):
    """Segment a pointcloud, using a list of pixel values.

    Keyword arguments:
    cloud   --  PointCloud2 raw cloud
    pixels  --  np.array() list of pixel pairs for segmentation
    """
    # Read 3D values for list of pixels, and unpack generator
    points3d = pc2.read_points(cloud, skip_nans=True, uvs=pixels)
    X = np.array(list(points3d))

    new_cloud = pypcd.make_xyz_rgb_point_cloud(X.astype(np.float32))
    msg = new_cloud.to_msg()
    msg.header.frame_id = 'base_camera_color_optical_frame'
    msg.header.stamp = rospy.Time.now()

    return msg


def pc2_to_cloudsamples(pc2_cloud, sample = (0, 0, 0)):
    # Setup CloudSources type
    source = CloudSources()
    source.cloud = data
    source.view_points = [Point(0, 0, 0)]

    # Setup CloudSamples type
    out = CloudSamples()
    out.cloud_sources = source
    out.samples = [Point(sample[0],sample[1],sample[2])]
    return out


def pixel_to_3d(cloud, center, size):
    """Get a 3D point from a pair of pixel values. Returns average after search.

    Keyword arguments:
    cloud   --  PointCloud2 raw cloud
    center  --  tuple (x, y) pixel value or center of search
    size    --  int search area radius
    """
    lst = []

    for i in range(center[0]-size,center[0]+size):
        for j in range(center[1]-size, center[1]+size):
            try:
                point3d = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=[(i,j)])
                lst.append(list(point3d))
                print(list(point3d))
            except Exception as e:
                print(e)

    return (np.mean([i[0] for i in lst[0]]), np.mean([i[1] for i in lst[0]]), np.mean([i[2] for i in lst[0]]))

pub = rospy.Publisher('/cloud_stitched', PointCloud2, queue_size=1)

def got_mask_CB(msg):
    segmask = image_to_numpy(msg, channels=1)[:,:,0]

    logger.debug("Got segmask, waiting for pointcloud ...")
    data = rospy.wait_for_message('/base_camera/camera/depth_registered/points', PointCloud2)

    # TODO - implement with vision
    # segmask = cv2.imread("../../object_detection/['contours', 'yolov3']_mask4.png")[:,:,0]

    seg_list = get_mask_index(segmask)

    seg_cloud = segment_cloud(data, seg_list)

    logger.info("Publishing segmented point cloud")
    pub.publish(seg_cloud)


rospy.init_node('RS_get_pointcloud')

vision = rospy.Subscriber('/vision_object_masks', Image, got_mask_CB)

rospy.spin()
