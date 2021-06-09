
import numpy as np

views = ["side", "head", "top"]
for view in views:
    for i in range(50):
        with open('/home/slave/Documents/workspaces/handover_ws/src/lh7-handover/grasping_ros/src/grasp_lib/dataset/{}_{}_color.txt'.format(i, view), 'a+') as file:
            file.seek(0)
            f = open("/home/slave/Documents/workspaces/handover_ws/src/lh7-handover/grasping_ros/src/grasp_lib/dataset/{}_{}_annotations.txt".format(i, view), "a")
            
            for line in file:
                y = line.split()
                arr = np.array(y)
                arr = arr.reshape(-1, 2)

                for x in arr:
                    f.write(str(x[0]) + " " + str(x[1]) + "\n") 

            f.close()
