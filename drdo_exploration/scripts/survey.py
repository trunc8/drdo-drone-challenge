#!/usr/bin/env python
import rospy
import numpy as np
from drdo_exploration.msg import teleopData
from helper import Helper
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import time
from cv_bridge import CvBridge, CvBridgeError
import tf
from nav_msgs.msg import Odometry
import cv2





arr = []
length = []
check2 = []
final = []

def findLIS(A, n):
		global final, length, check2, arr
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

class Survey(Helper):
	def __init__(self):
		global final, length, check2, arr
		self.curr_position = np.zeros(3)
		self.curr_orientation = np.zeros(3)
		self.init_pose = None
		self.pc2_arr = None
		self.listener = tf.TransformListener()
			 
		rospy.Subscriber('/mavros/global_position/local', Odometry, self.positionCallback,queue_size=1)

		rospy.Subscriber('/depth_camera/depth/image_raw', Image, self.ImageCallback, queue_size=1)
		rospy.Subscriber("/safesearch/start", Int16, self.start_survey_callback,queue_size=1) 

		self.drone_move_pub = rospy.Publisher('/safesearch/teleop',teleopData,queue_size = 1)  
		self.safesearch_complete_pub = rospy.Publisher('/safesearch/complete',Int16 ,queue_size=1)

		self.survey_flag = 0
		self.indicator =  0
		self.safesearch_complete_flag = Int16()
		self.defineParameters()

		self.rate = rospy.Rate(10)

		self.CONE = 120.0 #130
		self.NO_OF_POINTS_TO_CHECK = 11  #int((self.CONE/self.STEP_SIZE)) + 1
		self.STEP_SIZE = self.CONE / float(self.NO_OF_POINTS_TO_CHECK - 1) 

		self.new_waypoint_found = bool()
		self.target = None
		self.intensity_at_target = None
		self.target_array = [None]* self.NO_OF_POINTS_TO_CHECK
		self.target_intensity_array = [None]*self.NO_OF_POINTS_TO_CHECK
		#self.target_xyz_array = np.zeros(18)#shape 18 values
		self.THRESHOLD_INTENSITY = 0.25 * np.ones(len(self.target_intensity_array))#tunable parameter
		


		self.best_intensity_index = None
		self.best_yaw_angle = None
		self.direction = None

	def positionCallback(self, local_pose_msg):
		global final, length, check2, arr
		self.curr_position = [local_pose_msg.pose.pose.position.x,
							local_pose_msg.pose.pose.position.y,
							local_pose_msg.pose.pose.position.z]
		quaternion = [local_pose_msg.pose.pose.orientation.x,
					 local_pose_msg.pose.pose.orientation.y,
					 local_pose_msg.pose.pose.orientation.z,
					 local_pose_msg.pose.pose.orientation.w]

		self.curr_orientation = tf.transformations.euler_from_quaternion(quaternion)    

	


	def ImageCallback(self, img_msg):
		global final, length, check2, arr
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
		penalized_cv_img = self.calculatePenalty(cleaned_cv_img)
		# collision_cv_img = self.collision_avoidance(cleaned_cv_img)
		self.target, _ = self.findTarget(penalized_cv_img, cleaned_cv_img)
		#print("target pixel" , self.target)
		self.intensity_at_target = penalized_cv_img[self.target[0],self.target[1]]
		#print("intensity_at_target pixel",self.intensity_at_target)
		# dest_cv_img = cv2.circle(penalized_cv_img, (self.target[1],self.target[0]), 20, 0, -1)
		# dest_cv_img = cv2.circle(penalized_cv_img, (self.target[1],self.target[0]), 10, 1, -1)
		# cv2.imshow("destination img", dest_cv_img)

		# cv2.waitKey(1)
		return self.target 

	def find_good_waypoint(self):
		global final, length, check2, arr
		final = []
		good_intensities = np.array(self.target_intensity_array >= self.THRESHOLD_INTENSITY)
		idx_good_intensities = np.array(np.where(good_intensities == 1))
		print("idx_good_intensities1",idx_good_intensities)
		print("len if idx_good_int",len(idx_good_intensities[0]))


		if len(idx_good_intensities[0])>0:

			print("idx_good_intensities2",idx_good_intensities)
			idx_good_intensities = idx_good_intensities[0]
			n = len(idx_good_intensities)

			print("idx_good_intensities3",idx_good_intensities)
			arr = findLIS(idx_good_intensities, n)
			check2.append(arr)
			for i in range(1,n-1):
				ele = np.where(arr[0])
				idx_good_intensities = np.delete(idx_good_intensities,ele)
				check2.append(findLIS(idx_good_intensities,n-i))
			for i in range(len(check2)-1):
				if len(check2[i]) == max(length):
						if check2[i+1] != check2[i]:
							final.append(check2[i])
			if len(final) == 0:
				final.append(idx_good_intensities[0])
			print(final)
			final = np.array(final).flatten()
			try:
				if len(final) % 2 == 1:
					self.best_intensity_index = np.median(final)
				else:
					self.best_intensity_index = final[int(len(final)/2)]	
				self.best_yaw_angle = -1 * (self.direction) * ((self.NO_OF_POINTS_TO_CHECK-1) - self.best_intensity_index) * self.STEP_SIZE
				print("try block successful")
			except:
				print("Oh no no no")
				pass

			print("past try-except")
			return 1
		
		else:   
			return 0



	def emergency(self):
		global final, length, check2, arr
		print("NO WAYPOINT FOUND !")
		pass

	def go_to_height(self, h):
		global final, length, check2, arr
		opt_height_command = teleopData()
		opt_height_command.decision = 4
		opt_height_command.delta = h
		self.drone_move_pub.publish(opt_height_command)
		time.sleep(4)
		print("height reached" , h)

	def scan_using_yaw(self, initial_angle , direction):
		global final, length, check2, arr
		self.direction = direction
		yaw_command = teleopData()
		yaw_command.decision = 5
		yaw_command.delta = initial_angle
		self.drone_move_pub.publish(yaw_command)
		time.sleep(4)
		print("reache init deg",initial_angle)
		for i in range(self.NO_OF_POINTS_TO_CHECK):
			yaw_command.delta = self.STEP_SIZE * direction
			self.drone_move_pub.publish(yaw_command)
			time.sleep(2)
			print("yaw",(initial_angle+ ((i)*self.STEP_SIZE)))
			self.target_array[i] = self.target.copy()
			print("target_array", self.target_array)
			self.target_intensity_array[i] = self.intensity_at_target
			print("target_intensity_array", self.target_intensity_array)  
		

	def start_survey_callback(self,msg):
		global final, length, check2, arr
		self.survey_flag = msg.data

		if (self.survey_flag == 1 and self.indicator == 0):
			self.indicator = 1
			self.go_to_height(2.5)
			self.scan_using_yaw(-1*(self.CONE/2.0) , 1)
			if not(self.find_good_waypoint()):
				self.go_to_height(4)
				self.scan_using_yaw(0 , -1)
				if not(self.find_good_waypoint()):
					self.go_to_height(1)
					self.scan_using_yaw(0 , 1)
			# if not(self.find_good_waypoint()):

					#self.emergency()  #to have rtl like function
			else:
				print("FOUND NICE WAYPOINT")
				final_yaw_command = teleopData()
				final_yaw_command.decision = 5
				final_yaw_command.delta = self.best_yaw_angle
				self.drone_move_pub.publish(final_yaw_command)
				time.sleep(4)
				self.indicator = 0
				self.safesearch_complete_flag.data = 1
				self.safesearch_complete_pub.publish(self.safesearch_complete_flag)

	

			


if __name__ == '__main__':
	rospy.init_node('surveil_node')
	rospy.loginfo("surveil_node created")
	try:
		surveil_node_obj = Survey()

		rospy.spin()
	 
	except rospy.ROSInterruptException:
		pass
						
