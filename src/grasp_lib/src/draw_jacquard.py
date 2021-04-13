import numpy as np
import cv2 as cv


def convert(grasp):

    center = (grasp[0], grasp[1])
    angle = grasp[2]/180.0*np.pi
    print(angle)
    length = grasp[3]
    width = grasp[4]

    xo = np.cos(angle)
    yo = np.sin(angle)

    y1 = center[0] + length / 2 * yo
    x1 = center[1] - length / 2 * xo
    y2 = center[0] - length / 2 * yo
    x2 = center[1] + length / 2 * xo

    return np.array(
        [
        [y1 - width/2 * xo, x1 - width/2 * yo],
        [y2 - width/2 * xo, x2 - width/2 * yo],
        [y2 + width/2 * xo, x2 + width/2 * yo],
        [y1 + width/2 * xo, x1 + width/2 * yo],
        ]
    )

path = '/home/jan/Downloads/Jacquard_Samples/Samples/61921ef74746301497380cf93ab53c21/0_61921ef74746301497380cf93ab53c21'

image = cv.imread(path + '_RGB.png')

with open(path + '_grasps.txt', 'a+') as file:
    file.seek(0)
    
    for line in file:
        y = line.split(';')
        y[-1] = y[-1].rstrip('\r\n')
        color = list(np.random.choice(range(256), size=3))
        cv.drawContours(image, [np.array(convert([float(i) for i in y])).astype(int)], 0, color, 1)

cv.imwrite('output.png', image)