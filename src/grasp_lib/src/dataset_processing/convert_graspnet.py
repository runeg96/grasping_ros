import os
import glob
import sys
import numpy as np
from tqdm import tqdm

file_path = "/raid/Graspnet/Graspnet"
TOTAL_SCENE_NUM = 190
camera = "realsense"
fric_coef_thresh = 0.4
# split = "all"
splits = ["train", "test", "test_seen", "test_similar", "test_novel"]

sceneIds = []

for split in tqdm(splits):
    # select what data to be converted
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

    # Find grasp file paths
    for i in tqdm(sceneIds, desc='Loading data path split: {}'.format(split)):
        for img_num in range(256):
            rectLabelPath.append(os.path.join(file_path,'scene_'+str(i).zfill(4), camera, 'rect', str(img_num).zfill(4)+'.npy'))

    mean_array = np.empty(len(rectLabelPath))
    # Go though all files in scenes
    for index, file in enumerate(tqdm(rectLabelPath, desc='Converting data split: {}'.format(split))):
        #Load data
        f = np.load(file)

        #discard all grasps with a high friction value(Grasp force closure)
        mask = f[:,5] >= (1.1 - fric_coef_thresh)
        f = f[mask]

        # alocate room for data
        array = np.zeros((len(f),5))

        # Go though all lines in file and convert to jacquard format
        for idx, l in enumerate(f):
            x, y, ox,oy ,h ,_ ,_ = l

            # Calculate angle and width
            angle = -np.arctan2(oy-y, ox-x)
            w = np.hypot(ox - x, oy - y) * 2

            # Insert data in temp array
            array[idx] = [x,y,angle,w,h]

        # Calculate mean of BoundingBoxes
        min_x = max(0, np.min(array[:,0]))
        max_x = min(1280, np.max(array[:,0]))
        mean_x = np.mean([min_x,max_x])
        mean_array[index] = mean_x

        #Save grasp data for each file
        output_line = file[:-4] + '_fric02' + file[-4:]
        np.save(output_line,array)

    #Save means for all files in root dir
    np.save(file_path + "/"+ split + "_mean02.npy", mean_array)
