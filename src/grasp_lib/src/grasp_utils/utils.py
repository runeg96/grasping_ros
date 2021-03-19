import numpy as np
import math
import pyrealsense2 as rs

FOCAL_D345 = 1.93 #mm


def width_pixel_to_m(width_pixel, depth_m, cameraInfo):
    """Convert pixel measure to meters

    Keyword arguments:
    width_pixel -- measurement in pixels
    depth_m     -- float depth in meters
    cameraInfo  -- sensor_msgs/CameraInfo
    """

    fovx = 2 * math.atan(cameraInfo.width / (2 * cameraInfo.K[0]))
    return (2 * depth_m * math.tan(fovx/2)) / cameraInfo.width * width_pixel

def width_m_to_pixel(width_m, depth_m, cameraInfo):
    """Convert meter measure to pixels

    Keyword arguments:
    width_m     -- float measurement in meters
    depth_m     -- float depth in meters
    cameraInfo  -- sensor_msgs/CameraInfo
    """

    fovx = 2 * math.atan(cameraInfo.width / (2 * cameraInfo.K[0]))
    return (width_m * cameraInfo.width) / (2 * depth_m * math.tan(fovx/2)) 

def old_width_pixel_to_m(width_pixel, center, angle, cameraInfo):
    p1 = [center[0]-math.cos(angle)*width_pixel, center[1]-math.sin(angle)*width_pixel]
    p2 = [center[0]+math.cos(angle)*width_pixel, center[1]+math.sin(angle)*width_pixel]

    P1 = pixel_to_camera(cameraInfo, p1)
    P2 = pixel_to_camera(cameraInfo, p2)

    return math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2 + (P2[2] - P1[2])**2)

def cam_to_world(cam_point, world_to_cam):
    """Convert from camera_frame to world_frame

    Keyword arguments:
    cam_pose   -- PoseStamped from camera view
    cam_frame  -- The frame id of the camera
    """

    obj_vector = np.concatenate((cam_point, np.ones(1))).reshape((4, 1))
    world_point = np.dot(world_to_cam, obj_vector)

    world_point = [p[0] for p in world_point]
    return world_point[0:3]

def pixel_to_camera(cameraInfo, pixel, depth):
    """Deproject 2D pixel to 3D point

    Keyword arguments:
    cameraInfo  -- sensor_msgs/CameraInfo
    pixel       -- int tuple pixel to be deprojected
    depth       -- float depth value at pixel
    """
    _intrinsics = rs.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    
    _intrinsics.model  = rs.distortion.none
    # _intrinsics.coeffs = [i for i in cameraInfo.D]
    x,y = pixel
    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x,y], depth)

    return result

def camera_to_pixel(cameraInfo, point):
    """Project 3D point to 2D pixel

    Keyword arguments:
    cameraInfo  -- sensor_msgs/CameraInfo
    point       -- float array of 3D point
    """
    _intrinsics = rs.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    
    _intrinsics.model  = rs.distortion.none
    # _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = rs.rs2_project_point_to_pixel(_intrinsics, point)

    return result
