import os
import glob
from tqdm import tqdm
counter = 0
for i in tqdm(range(100,190)):
    graspf = glob.glob(os.path.join("/home/slave/Documents/Datasets/Graspnet",'scene_{}'.format(str(i).zfill(4)),'realsense', '0*.npy'))
    # print(graspf)
    try:
        os.mkdir("/home/slave/Documents/Datasets/Graspnet/scene_{}/realsense/rect".format(str(i).zfill(4)))
    except OSError:
        pass
        # print(OSError)
    depthf = [f.replace('realsense', 'realsense/rect') for f in graspf]
    for file, file_to in zip(graspf,depthf):
        # print("From: ",file,"To: ",file_to)
        os.rename(file,file_to)
    # print(depthf)
