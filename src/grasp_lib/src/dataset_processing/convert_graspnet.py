import os
import glob
import sys
import numpy as np

file_path = "/home/slave/Documents/Datasets/Graspnet"

test = False
if test:
    graspf = glob.glob(os.path.join(file_path, 'scene_01*', 'realsense/rect', '*.npy'))
else:
    graspf = glob.glob(os.path.join(file_path, 'scene_00*', 'realsense/rect', '*.npy'))

for file in graspf:
    #Load data
    print("Processing: ", file)
    f = np.load(file)
    
    # alocate room for data
    array = np.zeros((len(f),5))
    for idx, l in enumerate(f):
        x, y, ox,oy ,h ,_ ,_ = l

        # Calculate angle and width
        angle = np.arctan2(oy-y, ox-x)
        w = np.hypot(ox - x, oy - y)

        # Insert data in temp array
        array[idx] = [x,y,angle,w,h]
    output_line = file[:-4] + '_simple' + file[-4:]
    np.save(output_line,array)
        #crop out BoundingBoxes outside of image
