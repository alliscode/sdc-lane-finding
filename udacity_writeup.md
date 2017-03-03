**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/chessboard_corrected.png "Undistorted"
[image2]: ./examples/corrected.png "Road Transformed"
[image3]: ./examples/thresholded.png "Binary Example"
[image4]: ./examples/warp_lines.png "Warp Example"
[image5]: ./examples/curve_fit.png "Fit Visual"
[image6]: ./examples/output.png "Output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!
###Camera Calibration

####1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for computing the matrix transform and distortion coefficients is located in camera.py in the `calibrateDistortion()` function (line 18). The code that manages the chessboard images used in the calibration is located in data.py in the `chessboardFrames()` and `chessboardGridSize()` functions (lines 6 and 18 respectively).

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `frame_obj_points` is just a replicated array of coordinates, and `object_points` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `image_points` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection. After running all of the provided chessboard images through this process, I verify that a minimum number of images were successfully detected before moving on.

I then used the output `object_points` and `image_points` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![Undistorted chessboard image.][image1]

###Pipeline (single images)
The code for my image processing pipeline can be found in road.py in the `processFrame()` function (line 208). After playing around with several different pipelines and thresholding technique, I found that warping images to a bird's-eye view before the thresholding step gave very good results and I decided to keep going with technique. 

####1. Provide an example of a distortion-corrected image.
To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![An undistorted video frame.][image2]

####2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (see code in road.py, `threshold()` function, starting at line 255). For color thresholding, I found that a combination of grayscale and saturation channel produced excellent results (lines 266-267). For gradient thresholding, I found that a second derivative of a gaussian function worked very well to extract the lane lines and ignore the other noise in the image (lines 270-280). Once the gradient was calculated on the selected color channels, I used a percentile based threshold to remove all but the top few percent of pixel intensities (line 278-283). Here's an example of my output for this step.

![Binary image after thresholding.][image3]

####3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is in camera.py in the `calibrateBirdsEye()`, `toBirdsEye()`, and `fromBirdsEye()` functions (lines 71, 98, 113 respectively). The `calibrateBirdsEye()` function takes as inputs source (`src_points`) and destination (`dest_points`) points and uses them to calculate a perspective transform matrix. The src and dest points were manually retrieved by studying images from straight sections of roadway and then hard coded in the `birdsEyeSourcePoints()` and `birdsEyeDestinationPoints()` function in data.py. These functions look as follows:

```
def birdsEyeSourcePoints():
    return np.float32([[202,719], [578,460], [703,460], [1110,719]])

def birdsEyeDestinationPoints():
    return np.float32([[302,719], [302,0], [1010,0], [1010,719]])

```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 202,719     | 302,719       | 
| 578,460     | 302,0     |
| 703,460     | 1010,0     |
| 1110,719     | 1010,719        |

The `toBirdsEye()` function is used to transform an input image using the matrix calculated in the calibration step. I verified that my perspective transform was working as expected by drawing the `src_points` and `dest_points` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![Warped image][image4]

####4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The code to identify lane-line pixels is in the `Road.findLaneBases()` and `LaneLine.fromBasePoint()` funtions in road.py (lines 287 and 103 respectively). The first step is to pass the binary image to `Road.findLaneBases()` which uses a histogram of the bottom half of the image to detect the starting positions of the lane lines. This section of code also goes into more complex logic that attempts to use information from previously found lane lines in order to save processing time and ensure that the correct locations are chosen.

The next step is to construct new LaneLine objects from the starting points by calling `LaneLine.fromBasePoint()`. This function starts at the base point of the lane line and moves it's way up, searching for valid pixels in a narrow search box around the previously found locations. This code contains logic that expands the width of the search box everytime a valid point is not found in a row which give the algorithm enough robustness to locate curved lane lines even when large sections of lane line are missing.

![Lane lines found on binary image.][image5]

####5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The radius of curvature as well as the offset of the vehicle is calculated in the `LaneLine.curvature()` function of road.py (line 87). The curvature for each lane line is calculated by feeding the coefficients of the least squares fit into the equation for the curvature. The curvature of the two lane lanes are then averaged to create a single estimate of the curvature of the lane.

####6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The code to draw the lane back onto the corrected image is located in the `Road.__drawLane()` function in road.py (line 408). This function colors the detected lane section green and then warps the image back to the original perspective. Text describing the current lane curvature and vehicle offset are also added to the image at this point. Here is an example of my result on a test image:

![Sample output of procsessing pipeline.][image6]

---

###Pipeline (video)

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](https://www.youtube.com/watch?v=aYrzBsuoNE8&feature=youtu.be)

---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

I experimented a lot with getting the binary image to be as clean as possible and in this process I found that performing the binary thresholding after the birds-eye warp provided very nice results. This is the method that I stuck with to the end and I think that it did work quite well for the project video. Running the pipeline in this order allowed me to do pretty aggressive thresholding on the gradient in the x direction. However I'm not sure that this is the best strategy when there is a need to process roads with much sharper turns. On such roads, the lane lines could easily end up transitioning to gradients that point further along the y direction that the x direction and I'm afraid that the current version of the pipeline could fail. 
One idea that I have to improve on this is to perform dynamic gradient filtering and thresholding depending on characteristics of the image. For example, the bottom half of a frame could be processed similar to the current strategy in order to find the base point of the lane lines but then processing further up the image would vary based on the direction that the lane lines appear to be heading.

Another downside of the current implementation is the fact that  the pipeline is very, running at about 2-3 frames per second. This would certainly not work for a real driving system that needs the video frames processed in real time. I think that moving to c++ is a necessary first step in improving the speed of the pipeline.

### References:
[Camera calibration with OpenCV](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html)

[Real time lane detection](http://www.vision.caltech.edu/malaa/publications/aly08realtime.pdf)

