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
    i = 400
    print(np.min(cv_image_array[i]))
    print(np.max(cv_image_array[i]))
    print(cv_image_array[i].shape)

    cv_image_norm = cv_image_array/10
    cv2.imshow("Depth raw image", cv_image_norm)
    cv2.waitKey(3)
    # cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    # # Resize to the desired size
    # cv_image_resized = cv2.resize(cv_image_norm, (pc2_img_msg.width, pc2_img_msg.height), interpolation = cv2.INTER_CUBIC)
    # depthimg = np.array(cv_image_resized)


    
    # print(cv_image_norm)



if __name__ == '__main__':
  try:
    rospy.init_node('pc2xyz_node')
    exploration = Exploration()    
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("node terminated.")
