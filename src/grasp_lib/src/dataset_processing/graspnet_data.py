import os
import glob
import sys
import numpy as np


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
        test = False
        if test:
            graspf = glob.glob(os.path.join(file_path, 'scene_01*', 'realsense/rect', '*.npy'))
        else:
            graspf = glob.glob(os.path.join(file_path, 'scene_00*', 'realsense/rect', '*.npy'))

        graspf.sort()
        l = len(graspf)

        if l == 0:
            raise FileNotFoundError('No dataset files found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('rect', 'depth') for f in graspf]
        depthf = [f.replace('.npy', '.png') for f in depthf]

        rgbf = [f.replace('depth', 'rgb') for f in depthf]

        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

    def _get_crop_attrs(self, idx):
        left = 280
        top = 0
        center = (640,360)
        return center, left, top

    def get_gtbb(self, idx, rot=0, zoom=1.0):
        gtbbs = grasp.GraspRectangles.load_from_graspnet_file(self.grasp_files[idx], scale = self.output_size / 720)
        center, left, top = self._get_crop_attrs(idx)
        gtbbs.offset((-top//2, -120))
        gtbbs.zoom(zoom, (self.output_size//2, self.output_size//2))
        return gtbbs

    def get_depth(self, idx, rot=0, zoom=1.0):
        depth_img = image.DepthImage.from_png(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.crop((top, left), (720,1000))
        depth_img.inpaint()
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = image.Image.from_file(self.rgb_files[idx])
        print(self.rgb_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        rgb_img.crop((top, left), (720,1000))
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
        return rgb_img.img
