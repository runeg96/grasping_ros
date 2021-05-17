
import numpy as np


for i in range(26):
    with open('rect_labels/color_{}.txt'.format(i), 'a+') as file:
        file.seek(0)
        f = open("rect_labels/color_{}_newer_annotations.txt".format(i), "a")
        
        for line in file:
            y = line.split()
            arr = np.array(y)
            arr = arr.reshape(-1, 2)
            # print(arr)
            for x in arr:
                f.write(str(x[0]) + " " + str(x[1]) + "\n") 

        f.close()