import os
import glob
import sys
import numpy as np
from tqdm import tqdm

file_path = "/home/slave/Documents/Datasets/Graspnet"
TOTAL_SCENE_NUM = 190
camera = "realsense"
fric_coef_thresh = 0.4
split = "train"

sceneIds = []
if split == 'all':
    sceneIds = list(range(TOTAL_SCENE_NUM))
elif split == 'train':
    sceneIds = list(range(100))
elif split == 'test':
    sceneIds = list(range(100, 190))
elif split == 'test_seen':
    sceneIds = list(range(100, 130))
elif split == 'test_similar':
    sceneIds = list(range(130, 160))
elif split == 'test_novel':
    sceneIds = list(range(160, 190))

rectLabelPath = []
for i in tqdm(sceneIds, desc='Loading data path...'):
    for img_num in range(256):
        rectLabelPath.append(os.path.join(file_path, 'scenes', 'scene_'+str(i).zfill(4), camera, 'rect', str(img_num).zfill(4)+'.npy'))

for idx, file in enumerate(tqdm(rectLabelPath, desc='Converting data')):
    #Load data
    f = np.load(file)
    mask = f[:,5] >= (1.1 - fric_coef_thresh)
    f = f[mask]


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
