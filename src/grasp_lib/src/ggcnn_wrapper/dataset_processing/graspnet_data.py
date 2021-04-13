import os
import glob
import sys

import numpy as np
import math

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../../../ggcnn'))

from .grasp_data import GraspDatasetBase
from utils.dataset_processing import grasp, image


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

        graspf = glob.glob(os.path.join(file_path, '*', 'realsense', '0*.txt'))
        graspf.sort()
        l = len(graspf)

        if l == 0:
            raise FileNotFoundError('No dataset files found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('realsense', 'realsense/depth') for f in graspf]
        depthf = [f.replace('.txt', '.png') for f in depthf]

        rgbf = [f.replace('depth', 'rgb') for f in depthf]

        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

    def load_from_graspnet_file(self, fname, scale=1.0):
        grs = []
        f = np.load(fname)
        for l in f:
            x = l[0]
            y = l[1]
            ox = l[2]
            oy = l[3]

            angle = math.atan2(oy-y, ox-x)

            w = math.hypot(ox - x, oy - y)
            h = l[4]

            grs.append(Grasp(np.array([x, y]), angle, w, h).as_gr)
            # cx, cy, ox, oy, h, q, oid 
            # grs = cls(grs)
        grs.scale(scale)
        return grs  

    def get_gtbb(self, idx, rot=0, zoom=1.0):
        gtbbs = grasp.GraspRectangles.load_from_jacquard_file(self.grasp_files[idx], scale=self.output_size / 1024.0)
        c = self.output_size//2
        gtbbs.rotate(rot, (c, c))
        gtbbs.zoom(zoom, (c, c))
        return gtbbs

    def get_depth(self, idx, rot=0, zoom=1.0):
        depth_img = image.DepthImage.from_tiff(self.depth_files[idx])
        depth_img.rotate(rot)
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = image.Image.from_file(self.rgb_files[idx])
        rgb_img.rotate(rot)
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
        return rgb_img.img
