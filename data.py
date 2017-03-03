
import os
import numpy as np
from matplotlib import image as mpimg

def chessboardFrames():
    
    # read the chessboard calibration images into a list
    chessboardFrames = []
    file_dir = './udacity/CarND-Advanced-Lane-Lines-master/camera_cal'
    files = os.listdir(file_dir)
    for file in files:
        if file.endswith('.jpg'):
            chessboardFrames.append(mpimg.imread(file_dir + '/' + file))
            
    return chessboardFrames

def chessboardGridSize():
    return (9,6)

def birdsEyeSourcePoints():
    return np.float32([[202,719], [578,460], [703,460], [1110,719]])

def birdsEyeDestinationPoints():
    return np.float32([[302,719], [302,0], [1010,0], [1010,719]])