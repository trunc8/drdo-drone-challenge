#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt
from math import sqrt
from drone_path_planner.msg import teleopData
from mavros_msgs.srv import SetMode
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

x=None
y=None
z=None
p=None


def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')    

    try:
        isModeChanged_guided= False 
        rate2=rospy.Rate(1)
        while not isModeChanged_guided :            
            flightModeService = rospy.ServiceProxy('/mavros/set_mode',SetMode)
            isModeChanged_guided = flightModeService(custom_mode='LAND') #return true or false
            rospy.loginfo("Flight landing succesfull!!")
            rate2.sleep()
    except rospy.ServiceException:
        print "service set_mode call failed. LAND Mode could not be set. Check that GPS is enabled"



def callback_opencv(data):
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")

	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
	arucoParams = cv2.aruco.DetectorParameters_create()
		 
	img = cv2.medianBlur(img,3)
	(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)

	cX = 0
	cY = 0

	cv2.circle(img,(img.shape[1]//2,img.shape[0]//2),4,(255,0,0),-1)

	if ids is None: ##If no marker is detected
		print("FINDING MARKERS!!!")
		flag_stop = Int16()
		flag_stop.data = 0
		pub_stop_explore.publish(flag_stop)
		cv2.imshow("frame",img)
		cv2.waitKey(3)
	else:
		a = np.where(ids==0)  #tuple containing index of the aruco with id zero.
		if a[0].size==0:
			print("2")
			flag_stop = Int16()
			flag_stop.data = 0
			pub_stop_explore.publish(flag_stop)
			cv2.imshow("frame",img)
			cv2.waitKey(3)
		else:
			print("2")
			flag_stop = Int16()
			flag_stop.data = 1
			pub_stop_explore.publish(flag_stop)
			cv2.imshow("frame",img)
			cv2.waitKey(3)
			
			print("Found markers. Moving towards it")
			corners = corners[a[0][0]]
			(topLeft, topRight, bottomRight, bottomLeft) = corners[0]
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)

			cv2.circle(img, (cX, cY), 4, (0,255,0), -1)
			# cv2.circle(img,(img.shape[1]//2,img.shape[0]//2),4,(255,0,0),-1)

			cv2.imshow("frame",img)
			cv2.waitKey(3)

			edge_distance = sqrt( (topLeft[0]-topRight[0])**2 + (topLeft[0]-topRight[0])**2 )//2
			distance = 	sqrt( (cX-img.shape[1]//2)**2 + (cY-img.shape[0]//2)**2 )

			limit= 5

			print(distance)

			if(distance>100 and distance<300):
				
				Delta = 0.1

				pose_msg = PoseStamped()
				pose_msg.pose.position.x = x + (cX-(img.shape[1]//2))*Delta   
				pose_msg.pose.position.y= y - (cY-(img.shape[0]//2))*Delta  
				pose_msg.pose.position.z = z
				pose_msg.pose.orientation = p

				pub_set_point_local.publish(pose_msg)

			elif(distance>edge_distance and distance<100):
				
				Delta = 0.01

				pose_msg = PoseStamped()
				pose_msg.pose.position.x = x + (cX-(img.shape[1]//2))*Delta   
				pose_msg.pose.position.y= y - (cY-(img.shape[0]//2))*Delta  
				pose_msg.pose.position.z = z
				pose_msg.pose.orientation = p

				pub_set_point_local.publish(pose_msg)

			else:
				print("Landing")
				setStabilizeMode() 
		
		
def gps_data_callback(msg):
	global x,y,z,p
	''' 
	msgp is of PoseStamped msg type that we need to publish for going to the goal_point.
	the use of roll, pitch, and yaw needs to be looked into.
	''' 
	x = msg.pose.pose.position.x 
	y = msg.pose.pose.position.y 
	z = msg.pose.pose.position.z
	p = msg.pose.pose.orientation
	

	


if __name__ == '__main__':

	 rospy.init_node('listener', anonymous=True)

	 pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
	 pub_stop_explore = rospy.Publisher("/stop_exploring_flag", Int16,queue_size=10)

	 sub_gps=rospy.Subscriber("/mavros/global_position/local",Odometry, gps_data_callback)
	 rospy.Subscriber("/camera/color/image_raw/", Image, callback_opencv)
	 
	 rospy.spin()