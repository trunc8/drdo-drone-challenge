#!/usr/bin/env python
import rospy
import numpy as np
from drone_path_planner.msg import teleopData
from helper import Helper
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import time
from cv_bridge import CvBridge, CvBridgeError
import tf
from nav_msgs.msg import Odometry
import cv2





arr = []
length = []
check2 = []
final = []


class Surveil(Helper):
	def __init__(self):
		self.curr_position = np.zeros(3)
		self.curr_orientation = np.zeros(3)
		self.init_pose = None
		self.pc2_arr = None
		self.listener = tf.TransformListener()
		rospy.Subscriber('/mavros/global_position/local', Odometry, self.positionCallback)
			# dirn_topic = '/target_vector'
			# self.pub = rospy.Publisher(dirn_topic, direction, queue_size=10)
		rospy.Subscriber('/depth_camera/depth/image_raw', Image, self.ImageCallback)
		rospy.Subscriber("/safesearch/surveil",Bool, self.start_surveil_callback) 

		self.drone_move_pub = rospy.Publisher('/safesearch/teleop',teleopData,queue_size = 1)  
		self.safesearch_complete_pub = rospy.Publisher('/safesearch/complete',Bool,queue_size=1)   
		self.surveil_flag = 0
		self.indicator =  0
		self.defineParameters()
		self.rate = rospy.Rate(10)
		# self.new_waypoint_found = bool()
		# self.target = None
		# self.intensity_at_target = None
		# self.target_array=np.zeros(18)
		# self.target_intensity_array = np.zeros(18)
		# #self.target_xyz_array = np.zeros(18)#shape 18 values
		# self.THRESHOLD_INTENSITY = None #tunable parameter

		def positionCallback(self, local_pose_msg):
			self.curr_position = [local_pose_msg.pose.pose.position.x,
						  local_pose_msg.pose.pose.position.y,
						  local_pose_msg.pose.pose.position.z]
			quaternion = [local_pose_msg.pose.pose.orientation.x,
				   local_pose_msg.pose.pose.orientation.y,
				   local_pose_msg.pose.pose.orientation.z,
				   local_pose_msg.pose.pose.orientation.w]

			self.curr_orientation = tf.transformations.euler_from_quaternion(quaternion)    

	def findLIS(A, n):
		check = [] 
		hash = dict() 
	  
		LIS_size, LIS_index = 1, 0
	  
		hash[A[0]] = 1
		for i in range(1, n): 
			if A[i] - 1 not in hash: 
				hash[A[i] - 1] = 0
	  
			hash[A[i]] = hash[A[i] - 1] + 1
			if LIS_size < hash[A[i]]: 
				LIS_size = hash[A[i]] 
				LIS_index = A[i] 
		  
		length.append(LIS_size)
	  
		start = LIS_index - LIS_size + 1
		while start <= LIS_index: 
			check.append(start)
			start += 1

		return check
	

	def ImageCallback(self, img_msg):
		bridge = CvBridge()
		img_msg.encoding = "32FC1"
		try:
		  cv_img = bridge.imgmsg_to_cv2(img_msg, img_msg.encoding)
		except CvBridgeError as e:
		  print(e)
		  return

		cv_image_array = np.array(cv_img, dtype = np.dtype('f8'))
		cv_image_norm = cv_image_array/self.POINTCLOUD_CUTOFF
		cleaned_cv_img = cv_image_norm.copy()
		cleaned_cv_img[np.isnan(cleaned_cv_img)] = 1.0
		cleaned_cv_img = self.filterSkyGround(cleaned_cv_img)
		penalized_cv_img = self.penalizeObstacleProximity(cleaned_cv_img)
		collision_cv_img = self.collision_avoidance(cleaned_cv_img)
		self.target = self.findTarget(penalized_cv_img)
		print("target pixel" , self.target)
		self.intensity_at_target = penalized_cv_img[self.target]
		print("intensity_at_target pixel",self.intensity_at_target)
		dest_cv_img = cv2.circle(penalized_cv_img, (self.target[1],self.target[0]), 20, 0, -1)
		dest_cv_img = cv2.circle(penalized_cv_img, (self.target[1],self.target[0]), 10, 1, -1)
		cv2.imshow("destination img", dest_cv_img)
		cv2.waitKey(1)
		return self.target 

	def find_good_waypoint(self):
		pass

	def emergency(self):
		pass

	def go_to_height(self, h):
		opt_height_command = teleopData()
		opt_height_command.decision = 4
		opt_height_command.delta = h
		self.drone_move_pub.publish(opt_height_command)
		time.sleep(6)
		print("height reached" , h)

	def scan_using_yaw(self, initial_angle , direction):
		yaw_command = teleopData()
		yaw_command.decision = 5
		yaw_command.delta = initial_angle
		self.drone_move_pub.publish(yaw_command)
		time.sleep(7)
		print("reache init deg",initial_angle)
		for i in range(19):
			yaw_command.delta = 10 * direction
			self.drone_move_pub.publish(yaw_command)
			time.sleep(3)
			print("yaw",(initial_angle+ (i*10)))
			self.target_array.append(self.target)
			print()
			self.target_intensity_array.append(self.intensity_at_target)
		print("yaw completed")	
			

	def start_surveil_callback(self,msg):
		self.surveil_flag = msg.data

		if (self.surveil_flag == 1 and self.indicator == 0):
			self.indicator = 1
			self.go_to_height(2.5)
			self.scan_using_yaw(-90 , 1)
		  #if not(self.find_good_waypoint()):
			self.go_to_height(4)
			self.scan_using_yaw(0 , -1)
			#if not(self.find_good_waypoint()):
			self.go_to_height(1)
			self.scan_using_yaw(0 , 1)
			#	if not(self.find_good_waypoint()):
					#self.emergency()  #to have rtl like function

	

			


if __name__ == '__main__':
  rospy.init_node('surveil_node')
  rospy.loginfo("surveil_node created")
  try:
	surveil_node_obj = Surveil()

	rospy.spin()

   
  except rospy.ROSInterruptException:
	pass
						
