#! /usr/bin/env python3
import rospy

import numpy as np

import cv2
from skimage.draw import circle
from skimage.feature import peak_local_max

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from ggcnn_torch import predict
from sklearn.cluster import DBSCAN


rospy.init_node('ggcnn_detection')

# Output publishers.
grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
width_pub = rospy.Publisher('ggcnn/img/width', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)

# Initialise some globals.
prev_mp = np.array([150, 150])

# Get the camera parameters
print("Waiting for camera info")
camera_info_msg = rospy.wait_for_message('/ptu_camera/camera/aligned_depth_to_color/camera_info', CameraInfo)
print("Found camera info")
K = camera_info_msg.K
fx = K[0]
cx = K[2]
fy = K[4]
cy = K[5]


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


def depth_callback(depth_msg):

    global prev_mp
    global fx, cx, fy, cy

    depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width)
    depth = depth.astype(np.float32)/1000
    # depth = depth/1000
    # Crop a square out of the middle of the depth and resize it to 300*300
    crop_size = 300
    crop_offset = 40
    out_size = 300

    points_out, ang_out, width_out, depth_crop = predict(depth, crop_size=crop_size, out_size=out_size, crop_y_offset=crop_offset, filters=(2.0, 2.0, 2.0))

    # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
    depth_center = depth_crop[100:141, 130:171].flatten()
    depth_center.sort()
    depth_center = depth_center[:10].mean() * 1000.0

    # Calculate the best pose from the camera intrinsics.
    maxes = None
    ALWAYS_MAX = True  # Use ALWAYS_MAX = True for the open-loop solution.

    if ALWAYS_MAX:
        # Track the global max.
        max_pixels = np.argwhere(points_out == np.amax(points_out))
        clustering = DBSCAN(eps=5, min_samples=1).fit(max_pixels)
        max_pixel = np.mean(max_pixels[clustering.labels_ == 0],axis=0).astype(np.int64)

        score = points_out[max_pixel[0], max_pixel[1]]
        prev_mp = max_pixel.astype(np.int)
    else:
        # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
        maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
        if maxes.shape[0] == 0:
            rospy.logerr('No Local Maxes')
            return
        max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

        # Keep a global copy for next iteration.
        prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int)

    ang = ang_out[max_pixel[0], max_pixel[1]]
    width = width_out[max_pixel[0], max_pixel[1]]

    # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.

    max_pixel = ((np.array(max_pixel) / 300.0 * 300.0) + np.array([(480 - crop_size)//2 - crop_offset, (640 - crop_size) // 2]))
    max_pixel = np.round(max_pixel).astype(np.int)

    point_depth = depth[max_pixel[0], max_pixel[1]]

    # Compute the actual position.
    x = (max_pixel[1] - cx)/fx * point_depth
    y = (max_pixel[0] - cy)/fy * point_depth
    z = point_depth

    if np.isnan(z):
        return

    # Draw grasp markers on the points_out and publish it. (for visualisation)
    grasp_img = cv2.applyColorMap((points_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
    ang_out = ang_out.astype(np.float32) + abs(ang_out.min())
    ang_out = ang_out/ang_out.max()

    ang_img = cv2.applyColorMap((ang_out*-255).astype(np.uint8), cv2.COLORMAP_JET)
    width_out = width_out.astype(np.float32) + abs(width_out.min())
    width_out = width_out/width_out.max()
    # width_out -= width_out.mean()
    width_img = cv2.applyColorMap((width_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
    grasp_img_plain = grasp_img.copy()

    if score > 0.2:
        rr, cc = circle(prev_mp[0], prev_mp[1], 5)
        grasp_img[rr, cc, 0] = 0
        grasp_img[rr, cc, 1] = 255
        grasp_img[rr, cc, 2] = 0

    # Publish the output images (not used for control, only visualisation)
    grasp_img = numpy_to_imgmsg(grasp_img)
    grasp_img.header = depth_msg.header
    grasp_pub.publish(grasp_img)

    grasp_img_plain = numpy_to_imgmsg(grasp_img_plain)
    grasp_img_plain.header = depth_msg.header
    grasp_plain_pub.publish(grasp_img_plain)

    depth_pub.publish(numpy_to_imgmsg(depth_crop))
    ang_img = numpy_to_imgmsg(ang_img)
    ang_img.header = depth_msg.header
    ang_pub.publish(ang_img)

    width_img = numpy_to_imgmsg(width_img)
    width_img.header = depth_msg.header
    width_pub.publish(width_img)

    # Output the best grasp pose relative to camera.
    cmd_msg = Float32MultiArray()
    cmd_msg.data = [x, y, z, ang, width, depth_center, max_pixel[0], max_pixel[1], score]
    cmd_pub.publish(cmd_msg)


depth_sub = rospy.Subscriber('/ptu_camera/camera/aligned_depth_to_color/image_raw', Image, depth_callback, queue_size=1)
print("GGCNN IS RUNNING")

while not rospy.is_shutdown():
    rospy.spin()
