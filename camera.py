
import cv2
import numpy as np
from geometry import *

class Camera():
    
    def __init__(self):
        """A class that represents a camera. This object can be used to perform operations on images
        associated with a given camera.
        
        """
        self.m_distort = None
        self.distortion_coeffs = None
        self.birds_eye_t = None
        self.birds_eye_t_inv = None
    
    def calibrateDistortion(self, chessboard_images, grid_size, min_examples=10):
        """Calibrates the camera instance using images of a chessboard with a known grid size.
        
        Note: I only expect 15 - 20 images to be provided so the array based approach is workable.
        However, it may be better to provide an implementation that excepts a generator for the
        images.
        
        Args:
            chessboard_images: A numpy array of chessboard images.
            grid_size: The grid size of the chessboard images : (x, y)
            min_examples: The minimum number of successfully recognized chessboard images. If this
                          minimum is not reached a ValueError is raised.
        
        """
        
        # create an array to represent the object point of each frame. This array represents the
        # actual locations of the corners in each frame however the z coordinate is fixed at zero
        frame_obj_points = np.zeros((grid_size[0]*grid_size[1], 3), np.float32)
        frame_obj_points[:,:2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1,2)
        
        # holders for the object and image points in each frame as well as images superimposed
        # with chessboard corners
        object_points = []
        image_points = []
        drawn_images = []
        
        for frame in chessboard_images:
            
            # convert to gray and find the chessboard corners
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, grid_size, None)
            
            # cv2 may fail to find the chessboard corners in the image that we provided and will set ret
            # to False. In this case we just move on to the next frame.
            if ret:
                
                # save the obj and img points as well as drawn frame
                object_points.append(frame_obj_points)
                image_points.append(corners)
                drawn_images.append(cv2.drawChessboardCorners(frame, grid_size, corners, ret))
            
        # make sure we found the corners in enough frames
        if len(drawn_images) < min_examples:
            raise ValueError('Found corners in {0} frames, {0} required.'.format(len(drawn_images), min_examples))
        
        # now we use the obj and img points to create the camera matrix
        ret, self.m_distort, self.distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, frame.shape[0:2] ,None, None)
            
        if not ret:
            raise ValueError('Failed to find camera matrix from provided images.')
            
        return drawn_images
    
    def calibrateBirdsEye(self, src_points, dest_points):
        """Calibrates the camera for a birds-eye transformation.
        
        Args:
            src_points: 4 points from the source image.
            dest_points: 4 points that represent the transformed src_points.
        
        """
        
        self.birds_eye_t = cv2.getPerspectiveTransform(src_points, dest_points)
        self.birds_eye_t_inv = cv2.getPerspectiveTransform(dest_points, src_points)
    
    def undistortImage(self, image):
        """Undistorts the imput image using the matrix and distortion coefficients found during the
        calibration. The camera must be calibrated before calling this.
        
        Args:
            image: The image to transform.
            
        """
        
        # make sure the calibration step has been performed
        if self.m_distort is None or self.distortion_coeffs is None:
            raise ValueError('The camera must be successfully calibrated before it can correct images')
            
        return cv2.undistort(image, self.m_distort, self.distortion_coeffs, None, None)
    
    def toBirdsEye(self, image):
        """Transform an imput image to a birds-eye view using the transformation found in the calibrateBirdsEye
        step. The camera must be calibrated before calling this.
        
        Args:
            image: The image to transform.
            
        """
        
        # make sure the calibration step has been performed
        if self.birds_eye_t is None:
            raise ValueError('The camera must be calibrated for a birds eye view before it can transform images')
            
        return self.__warp__(image, self.birds_eye_t)
    
    def fromBirdsEye(self, image):
        """Transform an imput image from a birds-eye view using the inverse of the transformation found in the 
        calibrateBirdsEye step. The camera must be calibrated before calling this.
        
        Args:
            image: The image to transform.
            
        """
        
        # make sure the calibration step has been performed
        if self.birds_eye_t is None:
            raise ValueError('The camera must be calibrated for a birds eye view before it can transform images')
            
        return self.__warp__(image, self.birds_eye_t_inv)
    
    def __warp__(self, image, M):
        # TODO: better way to reverse image shape?
        return cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))
        