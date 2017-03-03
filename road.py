
import cv2
import numpy as np
from scipy import ndimage
from collections import deque
from copy import copy, deepcopy
from geometry import *
from camera import *
import heapq
from heapq import heappush, heappop
import math

class LaneLine:
    
    def __init__(self):
        """A class to represent a lane line.
        
        """
        
        self.points = []
        self.search_rects = []
        self.fit_coefficients = None
        self.fit_coefficients_world = None
        self.baseCoordinate = None
        self.hasVerifiedBaseCoordinate = False
        self.__filter_rate = 0.2
        
    def __copy__(self):
        new = type(self)(self.origin, self.size)
        new.points = self.points
        new.search_rects = self.search_rects
        new.fit_coefficients = self.fit_ceofficients
    
    def __deepcopy__(self, memo):
        new = type(self)(self.origin, self.size)
        new.points = deepcopy(self.points)
        new.search_rects = deepcopy(self.search_rects)
        new.fit_coefficients = deepcopy(self.fit_ceofficients)
        
    @property
    def points_x(self):
        return [p.x for p in self.points]
    
    @property
    def points_y(self):
        return [p.y for p in self.points]
        
    def addPoint(self, point : Point):
        """Add a point to the lane line.
        
        Args:
            point: The point to add to the lane line.
        
        """
        
        self.points.append(point)
        
    def addSearcRect(self, rect : Rect):
        """Adds a search rect to the lane line.
        
        Args:
            rect: The rect to add to the lane line.
            
        """
        self.search_rects.append(rect)
        
    def fit(self, previous, x_scale, y_scale):
        """Fits the points in the line with a 2nd degree polynomial.
        
        Args: 
            previous_fit: The previous fit of the lane line. If provided, the current fit will be
                          be used to update the previous fit as in a low pass filter. 
        
        """
        
        # get fits in image space and world space
        self.fit_coefficients = np.polyfit(self.points_y, self.points_x, 2)
        self.fit_coefficients_world = np.polyfit(np.array(self.points_y)/y_scale, np.array(self.points_x)/x_scale, 2)
        
        # if a previous frame is provided, use it to low pass filter this one to help smooth out fluctuations
        if previous is not None:
            self.fit_coefficients = self.__filter_rate*self.fit_coefficients + (1-self.__filter_rate)*previous.fit_coefficients
            self.fit_coefficients_world = self.__filter_rate*self.fit_coefficients_world + (1-self.__filter_rate)*previous.fit_coefficients_world
        
        return self.fit_coefficients
    
    def curvature(self, y):
        """Calculates the radius of curvature of the lane at a given point.
        
        Params:
            y: The y point  at which to evaluate the radius of the lane.
            
        Returns:
            The curvature of the lane at y. 
        """
        if self.fit_coefficients_world is None:
            raise ValueError('The lane must be fit before calculating the curvature.')
        
        A, B = self.fit_coefficients_world[0], self.fit_coefficients_world[1]
        return ((1 + (2*A*y + B)**2)**1.5) / (2*A)
        
    @classmethod
    def fromBasePoint(cls, image, x_pos):
        """Creates a new LaneLine by starting at the bottom of the image and following the
        lane up. The imput image must be a thresholded binary image that highlights the lanes.
        
        Args:
            image: A thresholded binary image.
            x_pos: The starting point at the bottom of the image.
        
        """
        
        # setup
        box_height = 50
        image_height = image.shape[0]
        initial_box_width = 100
        box_fanout = 30
        avg_thresh = 0.01
        num_rows = math.ceil(image_height/box_height)
        lane = LaneLine()

        # define the original search rect
        origin = Point(int(max(x_pos-initial_box_width/2, 0)), int(image_height-1 - box_height))
        box_size = Size(int(initial_box_width), int(box_height))
        rect = Rect(origin, box_size)

        # search each row
        for row in range(num_rows-1):

            # restate the rect coordinates in image space and slice the image
            top, bottom = rect.origin.y, rect.origin.y + rect.size.height
            left, right = rect.origin.x, rect.origin.x + rect.size.width
            section = image[top:bottom, left:right]

            # test if the current slice contains enough active pixels 
            if np.mean(section) > avg_thresh:

                # use the center of mass (centroid) as location of the point within the line
                cm = np.int32(ndimage.measurements.center_of_mass(section)[::-1])
                found_point = rect.origin + cm
                lane.addPoint(found_point)

                # since we found a valid point, we reset the box size and move up a row
                new_origin = Point(int(found_point.x - initial_box_width/2), max(rect.origin.y-box_height, 0))
                new_size = Size(initial_box_width + 2*box_fanout, box_height)
                
                # update the base coordinate 
                if row == 0: 
                    lane.baseCoordinate = found_point.x
                    lane.hasVerifiedBaseCoordinate = True

            else:
                # update the base coordinate
                if row == 0:
                    lane.baseCoordinate = x_pos
                    lane.hasVerifiedBaseCoordinate = False
                    lane.addPoint(Point(x_pos, image_height-1))

                # since we did not find a valid point, we expand the search rect and move up a row.
                # This expansion of the search grid with is required to handle curving lane lines
                new_origin = Point(max(rect.origin.x - box_fanout, 0), max(rect.origin.y-box_height, 0))
                new_size = Size(rect.size.width + 2*box_fanout, box_height)

            # record the rect and adjust the for the next row
            lane.addSearcRect(rect)
            rect = Rect(new_origin, new_size)

        return lane
    
    # Private helper methods
    
    @staticmethod
    def normal8Bit(image):
        retval = image - np.min(image)
        return np.uint8(255 * retval/np.max(retval))

    
class Road:
    """A class to represent a road with 2 lane lines.
    """
    
    def __init__(self, camera : Camera, x_scale=191, y_scale=25.7, lane_width=3.7):
        """
        
        Args:
            camera: The camera used to correct image distortion and warp them to a birds eye view.
            x_scale: The scale of image coordinates to world coordinates. Measured in meters/pixel.
        """
        self.camera = camera
        self.l_history = deque()
        self.r_history = deque()
        self.currentLaneWidth = None
        self.currentLaneOffset = None
        self.currentLaneCurvature = None
        self.currvatureDirection = None
        self.__history_size = 5
        self.__x_scale = x_scale
        self.__y_scale = y_scale
        self.__target_lane_width = lane_width
        
        # used for debug and reporting
        self.base_map = None
        
    def clearHistory(self):
        self.l_history = deque()
        self.r_history = deque()
    
    def processFrame(self, image):
        """A method to process an image with the entire pipeline. This is the workhorse of the
        road class.
        
        Args:
            image: The image to be processed.
            
        Returns:
            The original image with a green lane superimposed on top.
        """
        
        # run the image through the processing pipeline
        corrected = self.camera.undistortImage(image)
        birdsEye = self.camera.toBirdsEye(corrected)
        thresholded = self.threshold(birdsEye)
        l_base, r_base = self.findLaneBases(thresholded)
        l_lane = LaneLine.fromBasePoint(thresholded, l_base)
        r_lane = LaneLine.fromBasePoint(thresholded, r_base)
        
        # fit the lane lines
        l_previous = self.l_history[-1] if len(self.l_history) > 0 else None
        r_previous = self.r_history[-1] if len(self.r_history) > 0 else None
        l_lane.fit(l_previous, self.__x_scale, self.__y_scale)
        r_lane.fit(r_previous, self.__x_scale, self.__y_scale)
        
        # save some stats about the detected lines
        self.currentLaneWidth = (r_lane.baseCoordinate - l_lane.baseCoordinate)
        self.currentLaneOffset = (r_lane.baseCoordinate - image.shape[1]/2) - self.currentLaneWidth/2
        self.currentLaneWidth /= self.__x_scale
        self.currentLaneOffset /= self.__x_scale
                
        # save the current curvature
        l_curvature = l_lane.curvature(image.shape[0])
        r_curvature = r_lane.curvature(image.shape[0])
        self.currentLaneCurvature = np.mean([l_curvature, r_curvature])
        self.currvatureDirection = 'left' if self.currentLaneCurvature < 0 else 'right'
        
        # save the history
        self.l_history.append(l_lane)
        if len(self.l_history) > self.__history_size : self.l_history.popleft()
            
        self.r_history.append(r_lane)
        if len(self.r_history) > self.__history_size : self.r_history.popleft()
        
        # return the original image with the lane superimposed
        return self.__drawLane(corrected, l_lane, r_lane)
    
    def threshold(self, image):
        """Thresholds the input image into a binary image that highlights the lane lines.
        
        Args:
            image: The image to be processed.
            
        Returns:
            The thresholded binary image.
        """
        
        # get the gray scale and saturation channel from the imput image
        g_channel = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        s_channel = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)[:,:,2]

        # run both chanels through a second derivative gaussian filter
        grad_s = LaneLine.normal8Bit(ndimage.gaussian_filter(s_channel.astype(np.float32), sigma=(35,10), order=(0,2)))
        grad_g= LaneLine.normal8Bit(ndimage.gaussian_filter(g_channel.astype(np.float32), sigma=(35,10), order=(0,2)))

        # invert the gradients to make them more presentable (white lane lines on black background)
        grad_s  = np.max(grad_s) - grad_s
        grad_g  = np.max(grad_g) - grad_g

        # calculate a threshold based on the percentile of the filtered image
        thresh_s = np.percentile(grad_s, 97.5)
        thresh_g = np.percentile(grad_g, 98.6)

        # binarize the image with the thresholds calculated above
        binary_out = np.zeros_like(s_channel)
        binary_out[(grad_s > thresh_s) | (grad_g > thresh_g)] = 1
        
        return binary_out
    
    def findLaneBases(self, image):
        """Looks for the most likely position of the lane line at the bottom of the input image.
        
        Args:
            image: A thresholded binary image that highlights the lane lines.
            
        Returns:
            The left and right lane coordinates.
        """
        
        # leave out the edges to avoid noise
        l_buffer = 100
        r_buffer = 100
        
        if len(self.l_history) > 0 and len(self.r_history) > 0:
            
            # if we have history and the both lanes had verified base coodinates on the previous frame
            # then it's safe to reuse those coordinates. 
            if self.l_history[-1].hasVerifiedBaseCoordinate and self.r_history[-1].hasVerifiedBaseCoordinate:
                return (self.l_history[-1].baseCoordinate , self.r_history[-1].baseCoordinate)
            
            # if just one of the lanes from the previous frame was verified, then we use it as a reference 
            # to search for the other. The thinking that the distance between lanes should not change to 
            # dramatically from frame to frame.
            
            if self.l_history[-1].hasVerifiedBaseCoordinate:
                l_base_coord = self.l_history[-1].baseCoordinate 
                return (l_base_coord, self.__findRightLaneBase(image, l_base_coord, l_buffer))

            if self.r_history[-1].hasVerifiedBaseCoordinate:
                r_base_coord = self.r_history[-1].baseCoordinate 
                return (self.__findLeftLaneBase(image, r_base_coord, r_buffer), r_base_coord)
            
        # if we made this far then we don't have any usable historical lane posistion. So we need to 
        # find both lanes at once...
        
        # calculate a histogram of the bottom half of the image along the vertical axis. The peeks 
        # will indicate the most likely location of the lanes (but not always, see below)
        histogram = np.sum(image[-int(image.shape[0]/2):,:], axis=0)
        self.base_map = histogram
        
        # split the image in two halves horizontally and look for the histogram peeks on each side
        # if the input image is thresholded well enough then this will produce a good measure of the
        # position of our lane lines at the bottom of the image
        center_point = int(histogram.shape[0]/2)
        
        # define the range of acceptable lane separation
        min_distance = self.__target_lane_width * self.__x_scale * 0.85
        max_distance = self.__target_lane_width * self.__x_scale * 1.1
        
        # get the peek from each half
        left_max = l_buffer + np.argmax(histogram[l_buffer:center_point])
        right_max = np.argmax(histogram[center_point:-r_buffer]) + center_point
        lane_distance = right_max - left_max
        
        # check if the histogram maximums from each side fall into the acceptable lane separation zone.
        if lane_distance > min_distance and lane_distance < max_distance:
            return left_max, right_max
                
        # grab the top k maximums from each side and look for a pair with good separation
        top_k = 25
        l_max_ind = np.argpartition(histogram[l_buffer:center_point], -top_k)[-top_k:]
        r_max_ind = np.argpartition(histogram[center_point:-r_buffer], -top_k)[-top_k:]

        # look for a pair that fit between the min and max distance of a lane line
        heap = []
        for xl in l_max_ind:
            for xr in r_max_ind:
                distance = xr + center_point - xl
                if distance > min_distance and distance < max_distance:
                    error = np.absolute(self.__target_lane_width * self.__x_scale - distance)
                    heappush(heap, (error, xl+l_buffer, xr+center_point))
        if len(heap) > 0:
            return heap[0][1], heap[0][2]
        
        return (left_max, right_max)
    
    def __findLeftLaneBase(self, image, right_position, buffer):
        top_k = 25
        center_point = int(image.shape[1]/2)
        histogram = np.sum(image[-int(image.shape[0]/2):,buffer:center_point], axis=0)
        l_max_ind = np.argpartition(histogram, -top_k)[-top_k:]
        
        min_distance = self.currentLaneWidth * self.__x_scale * 0.85
        max_distance = self.currentLaneWidth * self.__x_scale * 1.1
        
        # look for a pair that fit between the min and max distance of a lane line
        heap = []
        for xl in l_max_ind:
            distance = right_position - xl
            if distance > min_distance and distance < max_distance:
                error = np.absolute(self.currentLaneWidth * self.__x_scale - distance)
                heappush(heap, (error, xl))
                
        if len(heap) == 0: 
            return right_position - self.currentLaneWidth * self.__x_scale
        
        return (heap[0][1] + buffer)
        
    def __findRightLaneBase(self, image, left_position, buffer):
        top_k = 25
        center_point = int(image.shape[1]/2)
        histogram = np.sum(image[-int(image.shape[0]/2):,center_point:-buffer], axis=0)
        r_max_ind = np.argpartition(histogram, -top_k)[-top_k:]
        
        min_distance = self.currentLaneWidth * self.__x_scale * 0.85
        max_distance = self.currentLaneWidth * self.__x_scale * 1.1
        
        # look for a pair that fit between the min and max distance of a lane line
        heap = []
        for xr in r_max_ind:
            distance = center_point - left_position + xr
            if distance > min_distance and distance < max_distance:
                error = np.absolute(self.currentLaneWidth * self.__x_scale - distance)
                heappush(heap, (error, xr))
                
        if len(heap) == 0: 
            return left_position + self.currentLaneWidth * self.__x_scale
        
        return (heap[0][1] + center_point)
    
    def __drawLane(self, image, left, right):
        
        y = np.arange(0, image.shape[0], 1)
        x_left = np.polyval(left.fit_coefficients, y)
        x_right = np.polyval(right.fit_coefficients, y)

        lane_fill = np.zeros(shape=image.shape[0:2], dtype=np.uint8)
        lane_fill = np.dstack((lane_fill, lane_fill, lane_fill))

        left_points = np.array([np.transpose(np.vstack((x_left, y)))])
        right_points = np.array([np.flipud(np.transpose(np.vstack((x_right, y))))])
        all_points = np.hstack((left_points, right_points))

        cv2.fillPoly(lane_fill, np.int_(all_points), (0, 255, 0))
        unwarped = self.camera.fromBirdsEye(lane_fill)
        result = cv2.addWeighted(image, 1, unwarped, 0.3, 0)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        offset_direction = 'left' if self.currentLaneOffset > 0 else 'right'
        currvature_text = 'Curvature: {0:0.3f} km ({1})'.format(np.absolute(self.currentLaneCurvature)/1000, self.currvatureDirection)
        offset_text = 'Offset: {0:0.3f} m ({1} of center)'.format(np.absolute(self.currentLaneOffset), offset_direction)
        cv2.putText(result,currvature_text,(10,50), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(result,offset_text,(10,90), font, 1,(255,255,255),2,cv2.LINE_AA)
        
        return result