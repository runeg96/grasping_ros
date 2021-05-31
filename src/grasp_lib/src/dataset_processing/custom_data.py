import os
import glob
import sys

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, '../../../../ggcnn'))

from utils.data.grasp_data import GraspDatasetBase
from dataset_processing import grasp, image


class CustomDataset(GraspDatasetBase):
    """
    Dataset wrapper for the Cornell dataset.
    """
    def __init__(self, file_path, start=0.0, end=1.0, ds_rotate=0, **kwargs):
        """
        :param file_path: Cornell Dataset directory.
        :param start: If splitting the dataset, start at this fraction [0,1]
        :param end: If splitting the dataset, finish at this fraction
        :param ds_rotate: If splitting the dataset, rotate the list of items by this fraction first
        :param kwargs: kwargs for GraspDatasetBase
        """
        super(CustomDataset, self).__init__(**kwargs)

        split = "head"
        if split == 'all':
            graspf = self.grasp_files = glob.glob(os.path.join(file_path, '*_annotations.txt'))
        else:
            graspf = self.grasp_files = glob.glob(os.path.join(file_path, '*_{}_annotations.txt'.format(split)))
            print("Using {} dataset".format(split))

        self.length = len(graspf)
        graspf.sort()
        l = len(graspf)
        if l == 0:
            raise FileNotFoundError('No dataset files found. Check path: {}'.format(file_path))

        if ds_rotate:
            graspf = graspf[int(l*ds_rotate):] + graspf[:int(l*ds_rotate)]

        depthf = [f.replace('_annotations.txt', '_depth.tiff') for f in graspf]
        depthf = [f.replace('color', 'depth') for f in depthf]
        rgbf = [f.replace('_annotations.txt', '_color.png') for f in graspf]

        self.grasp_files = graspf[int(l*start):int(l*end)]
        self.depth_files = depthf[int(l*start):int(l*end)]
        self.rgb_files = rgbf[int(l*start):int(l*end)]

    def _get_crop_attrs(self, idx):
        gtbbs = grasp.GraspRectangles.load_from_cornell_file(self.grasp_files[idx], scale=1.0)
        center = gtbbs.center
        left = max(0, min(center[1] - 480 // 2, 640 - 480))
        top = max(0, min(center[0] - 480 // 2, 640 - 480))
        return center, left, top

    def get_gtbb(self, idx, rot=0, zoom=1.0):
        gtbbs = grasp.GraspRectangles.load_from_cornell_file(self.grasp_files[idx], scale = self.output_size / 480)
        center, left, top = self._get_crop_attrs(idx)
        center = (center[0] * (self.output_size / 480), center[1] * (self.output_size / 480))
        gtbbs.rotate(rot, center)
        gtbbs.offset((-top*(self.output_size / 480), -left*(self.output_size / 480)))
        gtbbs.zoom(zoom, (self.output_size//2, self.output_size//2))
        return gtbbs

    def get_depth(self, idx, rot=0, zoom=1.0):
        depth_img = image.DepthImage.from_tiff(self.depth_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        depth_img.rotate(rot, center)
        # depth_img.crop((top, left), (min(480, top + self.output_size), min(640, left + self.output_size)))
        depth_img.crop((top, left), (min(480, top + 480), min(640, left + 480)))
        depth_img.normalise()
        depth_img.zoom(zoom)
        depth_img.resize((self.output_size, self.output_size))
        return depth_img.img

    def get_rgb(self, idx, rot=0, zoom=1.0, normalise=True):
        rgb_img = image.Image.from_file(self.rgb_files[idx])
        center, left, top = self._get_crop_attrs(idx)
        rgb_img.rotate(rot, center)
        rgb_img.crop((top, left), (min(480, top + 480), min(640, left + 480)))
        rgb_img.zoom(zoom)
        rgb_img.resize((self.output_size, self.output_size))
        if normalise:
            rgb_img.normalise()
            rgb_img.img = rgb_img.img.transpose((2, 0, 1))
        return rgb_img.img
