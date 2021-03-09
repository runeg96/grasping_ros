import cv2 as cv
import numpy as np
from PIL import Image


workspace_mask = np.array(Image.open("/home/slave/Documents/workspaces/handover_ws/src/lh7-handover/grasping_ros/src/graspnet_baseline/doc/example_data/workspace_mask.png"))
print(workspace_mask)
