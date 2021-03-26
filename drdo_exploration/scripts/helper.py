#!/usr/bin/env python

# task: help the pointcloud exploration
from __future__ import print_function
from __future__ import division

import cv2
import numpy as np
import random
import scipy.ndimage
from numpy.lib.stride_tricks import as_strided

import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import ros_numpy
import tf
from cv_bridge import CvBridge, CvBridgeError

from drdo_exploration.msg import direction



class Helper:
  

  def defineParameters(self):
    ## 1/n decay
    # decay_sequence = np.ones(KERNEL_SIZE//2, dtype=float)/(1+np.arange(KERNEL_SIZE//2))

    ## BELL CURVE decay
    '''
    e^-{(x)^2/DECAY_RATE}
    '''
    # decay_sequence = np.ones(KERNEL_SIZE//2, dtype=float)*np.exp(1)

    # decay_power = np.arange(KERNEL_SIZE//2)
    # decay_power = -1.*np.power(decay_power,2)/DECAY_RATE
    # decay_sequence = np.power(decay_sequence, decay_power)


    ## BUTTERWORTH decay
    '''
    1-1/(1+(d/x)^2n)
    '''

    KERNEL_SIZE = 180
    DECAY_RATE = 5
    DECAY_CUTOFF = 100
    decay_sequence = 1.0+np.arange(KERNEL_SIZE//2)
    decay_sequence = DECAY_CUTOFF/decay_sequence
    decay_sequence = np.power(decay_sequence, 2*DECAY_RATE)
    decay_sequence = 1/(1+decay_sequence)
    decay_sequence = (1 - decay_sequence)
    

    self.kernel_right = np.concatenate((np.zeros(KERNEL_SIZE//2),
                      decay_sequence))
    self.kernel_left = self.kernel_right[::-1]


    KERNEL_SIZE = 180
    DECAY_RATE = 5
    DECAY_CUTOFF = 100
    decay_sequence = 1.0+np.arange(KERNEL_SIZE//2)
    decay_sequence = DECAY_CUTOFF/decay_sequence
    decay_sequence = np.power(decay_sequence, 2*DECAY_RATE)
    decay_sequence = 1/(1+decay_sequence)
    decay_sequence = 1 - decay_sequence

    self.kernel_bottom = np.concatenate((np.zeros(KERNEL_SIZE//2),
                      decay_sequence))
    self.kernel_top = self.kernel_bottom[::-1]

    self.sky_ground_mask = None

    self.POINTCLOUD_CUTOFF = 10

    # Penalization tunables
    self.K_vertical = 0.5
    self.K_horizontal = 0.5

    # Penalty references
    self.Z_REF = 2.5
    self.TARGET_DIST = 0.4 # 0-1, representing depth

    # Penalty factors
    self.K_HORZ_MOVE =  0
    self.K_VERT_MOVE =  1e-1
    self.K_ALT = 1e-1
    self.K_DIST = 5e-1

    # Danger distance threshold
    self.DANGER_DISTANCE = 1 # In metres
    self.THRESHOLD_FRACTION = 0.9 # Fraction

    self.DILATION_KERNEL = (50,150)

    # self.PROXIMITY_THRESH = 3.


  def filterSkyGround(self, cleaned_cv_img):
    ## Filtering sky and ground ==> dont_see_mask -----------------------------------------
    
    height, width = [480, 640]
    '''
    I have assumed that the origin is at the top left corner.
    '''
    FOCAL_LENGTH = 554.25 # From camera_info
    LOWER_LIMIT = 0.5
    UPPER_LIMIT= 4.5
    IMAGE_PLANE_DISTANCE = self.POINTCLOUD_CUTOFF
    '''
    Half height is the original height in meters when the distance is 10m.
    '''
    HALF_PIXELS = height/2
    # HALF_HEIGHT = (HALF_PIXELS/FOCAL_LENGTH)*IMAGE_PLANE_DISTANCE 
    

    self.sky_ground_mask = np.ones(cleaned_cv_img.shape, dtype=bool)

    '''
    1. For upper limit, the range is 0 to (image_H_PIXELS - (half_pixels+  rest pixels))
    This rest_pixels is calculated usng the given equation
    2. For lower limit, the range is half_pixels+remaining to image_H_PIXELS.
    The remaining is calculated using the given equation.
    '''
    sky_limit = int((HALF_PIXELS-(UPPER_LIMIT-self.curr_position[2])*FOCAL_LENGTH/IMAGE_PLANE_DISTANCE))
    ground_limit = int(HALF_PIXELS+((self.curr_position[2]-LOWER_LIMIT)*FOCAL_LENGTH/IMAGE_PLANE_DISTANCE))
    if sky_limit>=0 and sky_limit<height:
      self.sky_ground_mask[:sky_limit,:] = 0
    if ground_limit>=0 and ground_limit<height:
      self.sky_ground_mask[ground_limit:,:] = 0

    # temp_cv_img = cleaned_cv_img.copy()
    cleaned_cv_img = np.multiply(cleaned_cv_img,self.sky_ground_mask)

    # cv2.imshow("After sky ground filter image", cleaned_cv_img.astype(float))
    # cv2.waitKey(3)

    return cleaned_cv_img


  def pixel_to_dirn(self, h, w):
    height, width = [480, 640]
    target_px = np.array([h-height//2, w-width//2])
    
    FOCAL_LENGTH = 554.25 # From camera_info
    IMAGE_PLANE_DISTANCE = self.POINTCLOUD_CUTOFF
    xp = (IMAGE_PLANE_DISTANCE/FOCAL_LENGTH)*target_px[1]
    yp = (IMAGE_PLANE_DISTANCE/FOCAL_LENGTH)*target_px[0]
    zp = IMAGE_PLANE_DISTANCE

    # print(xp, yp, zp)

    ps = PointStamped()
    ps.header.frame_id = "depth_cam_link"
    ps.header.stamp = rospy.Time(0)
    ps.point.x = zp
    ps.point.y = -xp
    ps.point.z = -yp
    # mat = self.listener.transformPoint("/map", ps)
    # return mat
    return ps

  def detectDanger(self, penalized_cv_img):
    danger_flag = 0
    # danger_left, danger_right = 0, 0
    # threshold_img_left, threshold_img_right = np.ones()
    thresholded_img = np.multiply((penalized_cv_img < 1.*self.DANGER_DISTANCE/self.POINTCLOUD_CUTOFF), self.sky_ground_mask)

    if np.sum(thresholded_img) > self.THRESHOLD_FRACTION * np.sum(self.sky_ground_mask):
      danger_flag = 1
      # print("DANGERRRRRRR")
    return danger_flag

  def findTarget(self, penalized_cv_img, cleaned_cv_img):
    '''
    Find (u,v) pixel coordinates that's the
    best candidate for target
    '''
    height, width = penalized_cv_img.shape
    max_intensity = np.max(penalized_cv_img)
    candidates = penalized_cv_img == max_intensity
    candidates = candidates.astype(float)

    nonzero_candidates = candidates.nonzero()
    y_values = nonzero_candidates[0]
    x_values = nonzero_candidates[1]
    length = len(x_values)

    # Finding median by x pixel position
    idx = np.argpartition(x_values, len(x_values) // 2)[len(x_values) // 2]

    # idx = random.randint(0, len(nonzero_candidates[0])-1)
    target = np.array([nonzero_candidates[0][idx],
               nonzero_candidates[1][idx]])

    # print("Target Depth: ", self.POINTCLOUD_CUTOFF*cleaned_cv_img[target[0],
            # target[1]])
    
    
    return target, 0

  
  def calculatePenalty(self, cleaned_cv_img):
  
    # Penalty for distance
    # penalized_cv_img = penalizeObstacleProximity(cleaned_cv_img) # Using edge-extension visor
    dilated_img = self.penalizeObstacleProximity(cleaned_cv_img) # Using grayscale dilation
    
    
    # thresh_dilation = self.dilateImage(1.*(dilated_img < 
    #     self.PROXIMITY_THRESH/self.POINTCLOUD_CUTOFF))
    # thresh dilation gives points less than 3m away


    cv2.imshow("Image after dilation penalty", dilated_img)
    cv2.waitKey(1)

    # Penalty for moving away from center
    vert_pen = self.vertical_veering_penalty()
    horz_pen = self.horizontal_veering_penalty()

    # Penalty for being off midlevel in world height
    z_pen = self.world_z_penalty()

    # Penalty for deviation from self.TARGET_DIST intensity
    dist_pen = self.distance_penalty(dilated_img)

    # Apply all
    penalized_cv_img = (dilated_img 
                        - self.K_VERT_MOVE * vert_pen 
                        - self.K_HORZ_MOVE * horz_pen 
                        - self.K_ALT * z_pen
                        - self.K_DIST * dist_pen)
    return penalized_cv_img
  

  def distance_penalty(self, dilated_img):
    #---------------------------------------------------------#
    ## Penalize distance from vertical centerline
    cv2.imshow("Depth Deviation Penalty", (1 - np.abs(dilated_img - self.TARGET_DIST)/self.TARGET_DIST).astype(float))
    return np.abs(dilated_img - self.TARGET_DIST)/self.TARGET_DIST

  
  def vertical_veering_penalty(self):
    #---------------------------------------------------------#
    ## Penalize distance from vertical centerline

    y_dist_penalty = np.arange(480) - 479/2.
    y_dist_penalty = np.abs(y_dist_penalty)
    y_dist_penalty = np.matlib.repmat(y_dist_penalty,640,1).T

    y_dist_penalty = y_dist_penalty/(np.max(y_dist_penalty))
    # cv2.imshow("Vertical Veering Penalty", y_dist_penalty.astype(float))
    return y_dist_penalty
  

  def horizontal_veering_penalty(self):
    #---------------------------------------------------------#
    ## Penalize distance from horizontal centerline

    x_dist_penalty = np.arange(640) - 639/2.
    x_dist_penalty = np.abs(x_dist_penalty)
    x_dist_penalty = np.matlib.repmat(x_dist_penalty, 480, 1)

    x_dist_penalty = x_dist_penalty/(np.max(x_dist_penalty))
    # cv2.imshow("Horizontal Veering Penalty", x_dist_penalty.astype(float))
    return x_dist_penalty

  
  def world_z_penalty(self):
  #---------------------------------------------------------#
  ## Penalize deviation of z-coordinate from self.Z_REF    

    err = (self.curr_position[2]-self.Z_REF)/self.Z_REF
    z_penalty = np.arange(480)*np.abs(err)/480
    if err>0:
      z_penalty = z_penalty[::-1]

    z_penalty = np.matlib.repmat(z_penalty,640,1).T

    # cv2.imshow("Global Altitude Deviation Penalty", z_penalty.astype(float))
    return z_penalty
  
  
  def penalizeObstacleProximity(self, cleaned_cv_img):
    penalized_cv_img = cleaned_cv_img.copy()
    
    #---------------------------------------------------------#
    '''
    Calculate horizontal differences only finding increasing brightnesses
    ----------
    Increasing brightness => Darker(closer) to brighter(farther)
    So danger obstacle is on the left of the edge line
    '''
    right_vertical_edge = cleaned_cv_img[:,1:] - cleaned_cv_img[:,0:-1]
    right_vertical_mask = (right_vertical_edge > 0.1).astype(float)
    # This matrix is basically blips at the pixels of right_vertical_edge
    
    
    right_vertical_penalty = self.K_vertical*scipy.ndimage.convolve1d(right_vertical_mask,
        weights= self.kernel_right, mode='constant', cval=0, axis=1)

    # cv2.imshow("Vertical right edge Penalty", right_vertical_penalty.astype(float))


    '''
    Calculate horizontal differences only finding decreasing brightnesses
    ----------
    Decreasing brightness => Brighter(farther) to darker(closer)
    So danger obstacle is on the right of the edge line
    '''
    left_vertical_edge = cleaned_cv_img[:,0:-1] - cleaned_cv_img[:,1:]
    left_vertical_mask = (left_vertical_edge > 0.1).astype(float)
    # This matrix is basically blips at the pixels of left_vertical_edge

    left_vertical_penalty = self.K_vertical*scipy.ndimage.convolve1d(left_vertical_mask,
        weights= self.kernel_left, mode='constant', cval=0, axis=1)

    # cv2.imshow("Vertical left edge Penalty", left_vertical_penalty.astype(float))
     
    '''
    Calculate vertical differences only finding decreasing brightnesses
    ----------
    Decreasing brightness => Brighter(farther) to darker(closer)
    So danger obstacle is on the bottom of the edge line
    '''
    bottom_horizontal_edge = cleaned_cv_img[0:-1,:] - cleaned_cv_img[1:,:]
    bottom_horizontal_mask = (bottom_horizontal_edge > 0.1).astype(float)
    # This matrix is basically blips at the pixels of bottom_horizontal_edge

    bottom_horizontal_penalty = self.K_horizontal*scipy.ndimage.convolve1d(bottom_horizontal_mask,
        weights= self.kernel_bottom, mode='constant', cval=0, axis=0)

    # cv2.imshow("Horizontal bottom edge Penalty", bottom_horizontal_penalty.astype(float))

    '''
    Calculate vertical differences only finding increasing brightnesses
    ----------
    Increasing brightness => Darker(closer) to brighter(farther)
    So danger obstacle is on the top of the edge line
    '''
    top_horizontal_edge = cleaned_cv_img[1:,:] - cleaned_cv_img[0:-1,:]
    top_horizontal_mask = (top_horizontal_edge > 0.1).astype(float)
    # This matrix is basically blips at the pixels of top_horizontal_edge

    top_horizontal_penalty = self.K_horizontal*scipy.ndimage.convolve1d(top_horizontal_mask,
        weights= self.kernel_top, mode='constant', cval=0, axis=0)

    # cv2.imshow("Horizontal top edge Penalty", top_horizontal_penalty.astype(float))


    penalized_cv_img[:,0:-1] = penalized_cv_img[:,0:-1] - right_vertical_penalty
    penalized_cv_img[:,1:] = penalized_cv_img[:,1:] - left_vertical_penalty
    penalized_cv_img[:,0] = np.zeros(480)
    penalized_cv_img[:,-1] = np.zeros(480)

    # penalized_cv_img[0:-1,:] = penalized_cv_img[0:-1,:] - bottom_horizontal_penalty
    # penalized_cv_img[1:,:] = penalized_cv_img[1:,:] - top_horizontal_penalty
    # penalized_cv_img[0,:] = np.zeros(640)
    # penalized_cv_img[-1,:] = np.zeros(640)
    

    penalized_cv_img.clip(min=0)


    return penalized_cv_img
  

  def dilateImage(self, cleaned_cv_img):
    img = scipy.ndimage.grey_dilation((1.-cleaned_cv_img), size=self.DILATION_KERNEL, mode='constant', cval=0.0)

    return (1.-img)
