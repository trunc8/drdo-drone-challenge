#!/usr/bin/env python

# task: pointcloud exploration
from __future__ import print_function
from __future__ import division

import cv2
import numpy as np
import random
import scipy.ndimage


import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
import ros_numpy
import tf
from cv_bridge import CvBridge, CvBridgeError

from drdo_exploration.msg import direction

from helper import Helper


class Exploration(Helper):
  def __init__(self):

    self.curr_position = np.zeros(3)
    self.curr_orientation = np.zeros(3)
    self.init_pose = None
    self.pc2_arr = None
    self.listener = tf.TransformListener()

    pose_topic = '/mavros/global_position/local'
    pc2_img_topic = '/depth_camera/depth/image_raw'
    rospy.Subscriber(pc2_img_topic, Image, self.pc2ImageCallback)
    rospy.Subscriber(pose_topic, Odometry, self.positionCallback)
    dirn_topic = '/target_vector'
    self.pub = rospy.Publisher(dirn_topic, direction, queue_size=10)

    KERNEL_SIZE = 640
    decay_sequence = np.ones(KERNEL_SIZE//2, dtype=float)/(1+np.arange(KERNEL_SIZE//2))
    self.kernel_right = np.concatenate((np.zeros(KERNEL_SIZE//2),
                                        decay_sequence))
    self.kernel_left = self.kernel_right[::-1]


  def positionCallback(self, local_pose_msg):
    self.curr_position = [local_pose_msg.pose.pose.position.x,
                          local_pose_msg.pose.pose.position.y,
                          local_pose_msg.pose.pose.position.z]
    quaternion = [local_pose_msg.pose.pose.orientation.x,
                   local_pose_msg.pose.pose.orientation.y,
                   local_pose_msg.pose.pose.orientation.z,
                   local_pose_msg.pose.pose.orientation.w]

    self.curr_orientation = tf.transformations.euler_from_quaternion(quaternion)


  def pc2ImageCallback(self, pc2_img_msg):
    bridge = CvBridge()
    pc2_img_msg.encoding = "32FC1"
    try:
      cv_img = bridge.imgmsg_to_cv2(pc2_img_msg, pc2_img_msg.encoding)
    except CvBridgeError as e:
      print(e)
      return
    
    cv_image_array = np.array(cv_img, dtype = np.dtype('f8'))
    POINTCLOUD_CUTOFF = 10
    cv_image_norm = cv_image_array/POINTCLOUD_CUTOFF

    

    cleaned_cv_img = cv_image_norm.copy()
    cleaned_cv_img[np.isnan(cleaned_cv_img)] = 1.0

    
    cleaned_cv_img = self.filterSkyGround(cleaned_cv_img)


    penalized_cv_img = self.penalizeObstacleProximity(cleaned_cv_img)

    #image_operation to apply colllision avoidance with drone

    collision_cv_img = self.collision_avoidance(cleaned_cv_img)
    


    target = self.findTarget(penalized_cv_img)

    #cv2.circle(penalized_cv_img, (target[1],target[0]), 20, 0, -1)
    #cv2.imshow("Penalized image", penalized_cv_img)
    cv2.waitKey(3)

    ps = self.pixel_to_dirn(target[0],target[1])
    dirn = np.array([ps.point.x, ps.point.y, ps.point.z])
    dirn = 1.*dirn/np.linalg.norm(dirn)

    dirn_msg = direction()
    dirn_msg.vec_x = dirn[0]
    dirn_msg.vec_y = dirn[1]
    dirn_msg.vec_z = dirn[2]
    
    #print("%.2f %.2f %.2f"%(dirn[0], -dirn[1], -dirn[2]))
    
    self.pub.publish(dirn_msg)







 

if __name__ == '__main__':
  try:
    rospy.init_node('pc2xyz_node')
    exploration = Exploration()    
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")
