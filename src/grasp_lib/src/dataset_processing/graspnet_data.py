import os
import glob
import sys
import numpy as np
import time
from tqdm import tqdm


ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../../../ggcnn'))

from utils.data.grasp_data import GraspDatasetBase
from dataset_processing import grasp, image

class GraspnetDataset(GraspDatasetBase):
    """
    Dataset wrapper for the Graspnet 1-billion dataset.
    """
    def __init__(self, file_path, start=0.0, end=1.0, ds_rotate=0, **kwargs):
        """
        :param file_path: Graspnet Dataset directory.
        :param start: If splitting the dataset, start at this fraction [0,1]
        :param end: If splitting the dataset, finish at this fraction
        :param ds_rotate: If splitting the dataset, rotate the list of items by this fraction first
        :param kwargs: kwargs for GraspDatasetBase
        """
        super(GraspnetDataset, self).__init__(**kwargs)

        TOTAL_SCENE_NUM = 190
        camera = "realsense"
        split = "train"
        sceneIds = []
        fric = "02"
        self.mean_file = ""

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



        graspf = []
        for i in tqdm(sceneIds, desc='Loading data path...'):
            for img_num in range(256):
                graspf.append(os.path.join(file_path,'scene_'+str(i).zfill(4), camera, 'rect', str(img_num).zfill(4)+'_fric' + fric +'.npy'))

        graspf.sort()
        l = len(graspf)

        if l == 0:
            raise FileNotFoundError('No dataset files found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('rect', 'depth') for f in graspf]
        depthf = [f.replace('_fric' + fric + '.npy', '.png') for f in depthf]

        rgbf = [f.replace('depth', 'rgb') for f in depthf]

        self.mean_file = file_path + "/" + split + "_mean" + fric + ".npy"
        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

    def _get_crop_attrs(self, idx):
        f = np.load(self.mean_file)
        center_x = int(f[idx])
        top = 0
        left = max(0, min(center_x - 720 // 2, 1280 - 720))
        return center_x, left, top

    def get_gtbb(self, idx, rot=0, zoom=1.0):
        center, left, top = self._get_crop_attrs(idx)
        gtbbs = grasp.GraspRectangles.load_from_graspnet_file(self.grasp_files[idx], scale = self.output_size / 720, mean=center)
        gtbbs.offset((-top, int(-left*0.416666667)))
        gtbbs.zoom(zoom, (self.output_size//2, self.output_size//2))
        return gtbbs

    def get_depth(self, idx, rot=0, zoom=1.0):
        depth_img = image.DepthImage.from_png(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.crop((top, left), (min(720, top + 720), min(1280, left + 720)))
        depth_img.inpaint()
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = image.Image.from_file(self.rgb_files[idx])
        print(self.rgb_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        rgb_img.crop((top, left), (min(720, top + 720), min(1280, left + 720)))
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
        return rgb_img.img
