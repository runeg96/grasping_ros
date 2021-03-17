import numpy as np
import math
import pyrealsense2 as rs

FOCAL_D345 = 1.93 #mm


def width_pixel_to_m(width_pixel, depth_m, cam_info):
    fovx = 2 *math.atan(cam_info.width / 2 * cam_info.K[0])
    return width_pixel / cam_info.width * 2 * depth_m * math.tan(fovx/2)


def new_width_pixel_to_m(width_pixel, center, angle, cam_info):
    p1 = [center[0]-math.cos(angle)*width_pixel, center[1]-math.sin(angle)*width_pixel]
    p2 = [center[0]+math.cos(angle)*width_pixel, center[1]+math.sin(angle)*width_pixel]

    P1 = pixel_to_camera(cam_info, p1)
    P2 = pixel_to_camera(cam_info, p2)

    return math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2 + (P2[2] - P1[2])**2)


def width_m_to_pixel(width_m, depth_m):
    return (width_m * (FOCAL_D345/1000)) / depth_m 

# def width_pixel_to_m(width_pixel, depth_m):
#     return (width_pixel * depth_m) / (FOCAL_D345/1000)
    

def cam_to_world(cam_point, world_to_cam):
    """Convert from camera_frame to world_frame

    Keyword arguments:
    cam_pose   -- PoseStamped from camera view
    cam_frame  -- The frame id of the camera
    """
    # cam_point = np.array([cam_pose[0], cam_pose[1], cam_pose[2]])

    obj_vector = np.concatenate((cam_point, np.ones(1))).reshape((4, 1))
    world_point = np.dot(world_to_cam, obj_vector)

    world_point = [p[0] for p in world_point]
    return world_point[0:3]

def pixel_to_camera(cameraInfo, pixel, depth):
    _intrinsics = rs.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    x,y = pixel
    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x,y], depth)
    # return rs.rs2_deproject_pixel_to_point(depth_intrin, pixel, 1.0)
    # x = result[0]
    # y = result[1]
    # depth = result[2]
    # z_squared = depth**2 - x**2 - y**2
    # z = np.sqrt(z_squared)
    # result[2] = z

    return result

def camera_to_pixel(cameraInfo, point):
    _intrinsics = rs.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = rs.rs2_project_point_to_pixel(_intrinsics, point)

    return result
