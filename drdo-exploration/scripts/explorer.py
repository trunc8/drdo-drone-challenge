#!/usr/bin/env python

# task: pointcloud exploration

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import ros_numpy
import tf
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import scipy.ndimage

class Exploration:
  def __init__(self):

    self.curr_position = None
    self.curr_orientation = None
    self.init_pose = None

    pc2_topic = '/depth_camera/depth/points'
    pose_topic = '/mavros/global_position/local'
    pc2_img_topic = '/depth_camera/depth/image_raw'
    rospy.Subscriber(pc2_img_topic, Image, self.pc2ImageCallback)
    rospy.Subscriber(pc2_topic, PointCloud2, self.pc2Callback)
    rospy.Subscriber(pose_topic, Odometry, self.positionCallback)


  def positionCallback(self, local_pose_msg):
    self.curr_position = [local_pose_msg.pose.pose.position.x,
                          local_pose_msg.pose.pose.position.y,
                          local_pose_msg.pose.pose.position.z]
    quaternion = [local_pose_msg.pose.pose.orientation.x,
                   local_pose_msg.pose.pose.orientation.y,
                   local_pose_msg.pose.pose.orientation.z,
                   local_pose_msg.pose.pose.orientation.w]

    self.curr_orientation = tf.transformations.euler_from_quaternion(quaternion)
    # print(self.curr_position, self.curr_orientation)


  def pc2Callback(self, pc2_msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
    mask = (xyz_array[:,2] < 4.5) & (xyz_array[:,2] > 0.5)
    xyz_array = xyz_array[mask]
    dist = np.linalg.norm(xyz_array, axis=1)
    # print(max(dist))
    # print("Received")
    # print(len(xyz_array))


  def pc2ImageCallback(self, pc2_img_msg):
    bridge = CvBridge()
    pc2_img_msg.encoding = "32FC1"
    try:
      cv_img = bridge.imgmsg_to_cv2(pc2_img_msg, pc2_img_msg.encoding)
    except CvBridgeError as e:
      print(e)
      return
    
    cv_image_array = np.array(cv_img, dtype = np.dtype('f8'))
    cv_image_norm = cv_image_array/10
    bloated_cv_img = self.bloatImage(cv_image_norm)
    bloated_cv_img[np.isnan(bloated_cv_img)] = 1
    
    cv2.imshow("Bloated img", bloated_cv_img)
    cv2.waitKey(3)


  def bloatImage(self, cv_image_norm):
    bloated_cv_img = cv_image_norm.copy()
    # cv2.imshow("Depth raw image", cv_image_norm)
    # cv2.waitKey(3)
    
    safety_threshold = 15 # in pixels (needs tuning)
    
    '''
    Calculate horizontal differences only finding decreasing brightnesses
    ----------
    Decreasing brightness => Brighter(farther) to darker(closer)
    So danger obstacle is on the right of the edge line
    '''
    left_vertical_edge = cv_image_norm[:,0:-1] - cv_image_norm[:,1:]
    left_vertical_edge = left_vertical_edge.clip(min=0)
    left_vertical_mask = left_vertical_edge > 0.1
    kernel = np.concatenate((np.ones(safety_threshold//2),
                            np.zeros(safety_threshold//2)))
    left_vertical_mask = scipy.ndimage.convolve1d(left_vertical_mask, weights=kernel, axis=1)
    left_vertical_mask = left_vertical_mask > 0.1

    bloated_cv_img[:,1:][left_vertical_mask] = 0

    # print(np.min(left_vertical_edge[i]))
    # print(np.max(left_vertical_edge[i]))
    # print(left_vertical_edge[i].shape)
    # print(left_vertical_mask.shape)

    # cv2.imshow("Left Vertical Edge", left_vertical_edge)
    # cv2.waitKey(3)


    '''
    Calculate horizontal differences only finding increasing brightnesses
    ----------
    Increasing brightness => Darker(closer) to brighter(farther)
    So danger obstacle is on the left of the edge line
    '''
    right_vertical_edge = cv_image_norm[:,1:] - cv_image_norm[:,0:-1]
    right_vertical_edge = right_vertical_edge.clip(min=0)
    right_vertical_mask = right_vertical_edge > 0.1
    kernel = np.concatenate((np.zeros(safety_threshold//2),
                            np.ones(safety_threshold//2)))
    right_vertical_mask = scipy.ndimage.convolve1d(right_vertical_mask, weights=kernel, axis=1)
    right_vertical_mask = right_vertical_mask > 0.1

    bloated_cv_img[:,0:-1][right_vertical_mask] = 0

    # print(np.min(right_vertical_edge[i]))
    # print(np.max(right_vertical_edge[i]))
    # print(right_vertical_edge[i].shape)
    
    # cv2.imshow("Right Vertical Edge", right_vertical_edge)
    # cv2.waitKey(3)

    return bloated_cv_img
    

if __name__ == '__main__':
  try:
    rospy.init_node('pc2xyz_node')
    exploration = Exploration()    
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")
