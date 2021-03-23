#!/usr/bin/env python
import rospy
from pylab import *
import numpy as np
import time
from matplotlib import pyplot as plt
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, Point, TransformStamped
import tf, geometry_msgs, tf2_ros
from tf import TransformBroadcaster
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import roslib
from gazebo_msgs.msg import ModelStates

#PLEASE NOTEE THAT THIS SCRPT WILL RUN ONLY AFTER THE IMU IS INITIALISED PROPERLY. "FCU: EKF2 IMU1 is using GPS" SHOULD BE PRINTED ON THE CONSOLE FIRST. 
#OTHERWISE IT WILL THROW SOME ERROR

def handle_pose(msg):
    st = tf2_ros.StaticTransformBroadcaster()
    br = tf.TransformBroadcaster()

    tf2Stamp = TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = "base_link"
    tf2Stamp.child_frame_id = "depth_cam_link"
    tf2Stamp.transform.translation.x = 0.1
    tf2Stamp.transform.translation.y = 0.0
    tf2Stamp.transform.translation.z = 0.0

    quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

    tf2Stamp.transform.rotation.x = quat[0]
    tf2Stamp.transform.rotation.y = quat[1]
    tf2Stamp.transform.rotation.z = quat[2]
    tf2Stamp.transform.rotation.w = quat[3]

    st.sendTransform(tf2Stamp)

    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "/map")

def callback(value):
	global pc_arr
	pc_arr = ros_numpy.numpify(value)

def pixel_to_depth(h,w,arr):										#h,w are image coordinates
	xp,yp,zp = arr['x'][h][w],arr['y'][h][w],arr['z'][h][w]
	ps = PointStamped()
	ps.header.frame_id = "depth_cam_link"
	ps.header.stamp = rospy.Time(0)
	ps.point.x = zp
	ps.point.y = -xp
	ps.point.z = -yp
	mat = listener.transformPoint("/map", ps)
	return mat

def cam_frame(data):
	global pc_arr
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	# cv2.imshow('camera_feed',img)
	# cv2.waitKey(30)
	cloud_arr = pc_arr
	mat1 = pixel_to_depth(320,240,cloud_arr)					
	# print(mat1.point.x, mat1.point.y, mat1.point.z)
	# print("--------------------------------------------------------------")

if __name__ == '__main__':
	pc_arr = None
	rospy.init_node('world_coordinate', anonymous=True)
	print('Node_initialised')
	listener = tf.TransformListener()
	time.sleep(5)
	rospy.Subscriber("/depth_camera/rgb/image_raw", Image, cam_frame)
	rospy.Subscriber("/depth_camera/depth/points", PointCloud2, callback)
	rospy.Subscriber('/mavros/local_position/pose',
					PoseStamped,
					handle_pose)
	print('Broadcasting...')
	rospy.spin()