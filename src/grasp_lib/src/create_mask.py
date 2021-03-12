import numpy as np
import cv2 as cv
from PIL import Image

img = np.ones((480,640))
cv.imwrite("workspace_mask_640.png",img)
# workspace_mask = np.array(Image.open("/home/slave/Documents/workspaces/handover_ws/workspace_mask_1280.png"),dtype=bool)
# print(workspace_mask)
#
# workspace_mask = np.array(Image.open("/home/slave/Documents/workspaces/handover_ws/src/lh7-handover/grasping_ros/src/graspnet_baseline/doc/example_data/workspace_mask.png"))
# print(workspace_mask)
