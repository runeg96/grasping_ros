import cv2 as cv

import argparse
import glob
import os

import numpy as np

parser = argparse.ArgumentParser(description='Generate depth images(PNG) from Cornell PCD files.')
parser.add_argument(
    'path', type=str, help='Path to Cornell Grasping Dataset')
args = parser.parse_args()

pcds = glob.glob(os.path.join(args.path, '*', 'pcd*[0-9]d.tiff'))
pcds.sort()

for pcd in pcds:
    print(pcd)
    imfile = cv.imread(pcd,-1)
    of_name = pcd.replace('d.tiff', '.png')
    cv.imwrite(of_name, imfile)
    # imsave(of_name, di.img.astype(np.float32))
