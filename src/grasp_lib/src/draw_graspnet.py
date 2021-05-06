from graspnetAPI import GraspNet
import open3d as o3d
import cv2
import numpy as np

####################################################################
graspnet_root = '/home/slave/Documents/Datasets/Graspnet' # ROOT PATH FOR GRASPNET
####################################################################

sceneId = 99
annId = 50

# initialize a GraspNet instance
g = GraspNet(graspnet_root, camera='realsense', split='train')

fric = np.linspace(0.0,1.0,11)
print(fric)
for i in fric:
    # load rectangle grasps of scene 1 with annotation id = 3, camera = realsense and fric_coef_thresh = 0.2
    rect_grasp = g.loadGrasp(sceneId = sceneId, annId = annId, format = 'rect', camera = 'realsense', fric_coef_thresh = round(i,1))
    print('rectangle grasp:\n{}'.format(rect_grasp))

    # visualize the rectanglegrasps using opencv
    bgr = g.loadBGR(sceneId = sceneId, annId = annId, camera = 'realsense')
    img = rect_grasp.to_opencv_image(bgr, numGrasp = 0)

    while(1):
        cv2.imshow('rectangle grasps fric: {:.1f}'.format(i), img)
        k = cv2.waitKey(33)
        if k==27:    # Esc key to stop
            cv2.destroyAllWindows()
            break
        elif k==-1:  # normally -1 returned,so don't print it
            continue
        else:
            print(k) # else print its value
