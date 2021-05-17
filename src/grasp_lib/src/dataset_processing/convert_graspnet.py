import os
import glob
import sys
import numpy as np
from tqdm import tqdm

file_path = "/raid/Graspnet/Graspnet"
TOTAL_SCENE_NUM = 190
camera = "realsense"
fric_coef_thresh = 0.4
fric = str(fric_coef_thresh).replace('.','')
# split = "all"
splits = ["train", "test", "test_seen", "test_similar", "test_novel"]

sceneIds = []

def cornell_format(center, angle, width=60, height=30):
    """
    Convert to GraspRectangle
    :return: GraspRectangle representation of grasp.
    """
    xo = np.cos(angle)
    yo = np.sin(angle)

    y1 = center[0] + width / 2 * yo
    x1 = center[1] - width / 2 * xo
    y2 = center[0] - width / 2 * yo
    x2 = center[1] + width / 2 * xo

    return np.array(
        [
         [y1 - height/2 * xo, x1 - height/2 * yo],
         [y2 - height/2 * xo, x2 - height/2 * yo],
         [y2 + height/2 * xo, x2 + height/2 * yo],
         [y1 + height/2 * xo, x1 + height/2 * yo],
         ]
    ).astype(np.float)


for count, split in enumerate(splits):
    print("Dataset: ", count,"/",len(splits))
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

    mean_array = np.zeros(len(rectLabelPath))
    # Go though all files in scenes
    for index, file in enumerate(tqdm(rectLabelPath, desc='Converting data split: {}'.format(split))):

        array = []

        #Load data
        f = np.load(file)

        #discard all grasps with a high friction value(Grasp force closure)
        mask = f[:,5] >= (1.1 - fric_coef_thresh)
        f = f[mask]

        mean_array[index] = np.mean(f[:,0])

        for l in f:
            x, y, ox,oy ,h ,_ ,_ = l
            if mean_array[index] - 360 < x < mean_array[index] + 360 and 0 < y < 720:

                angle = -np.arctan2(oy-y, ox-x)
                w = np.hypot(ox - x, oy - y) * 2

                # Insert data in temp array
                array.append(cornell_format(np.array([y,x]),angle,w,h))

        array = np.array(array)


        #Save grasp data for each file
        output_line = file[:-4] + '_fric' + fric + file[-4:]
        np.save(output_line,array)

    #Save means for all files in root dir
    np.save(file_path + "/"+ split + "_mean" + fric + ".npy", mean_array)
