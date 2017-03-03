
import cv2
from camera import Camera
from matplotlib import image as mpimg
from road import *
from data import *

from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from moviepy.editor import VideoFileClip

# 1) create the camera object and calibrate it
print("Calibrating camera...")
camera = Camera()
camera.calibrateDistortion(chessboardFrames(), chessboardGridSize())
camera.calibrateBirdsEye(birdsEyeSourcePoints(), birdsEyeDestinationPoints())

# 2) create the road and have it process our images
print("Building road object...")
road = Road(camera)

# 3) Process the video
output = './udacity/CarND-Advanced-Lane-Lines-master/project_video_out.mp4'
clip1 = VideoFileClip("./udacity/CarND-Advanced-Lane-Lines-master/project_video.mp4")
out_clip = clip1.fl_image(road.processFrame)
%time out_clip.write_videofile(output, audio=False)