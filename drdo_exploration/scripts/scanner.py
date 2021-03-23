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
from drdo_exploration.msg import aruco_detect

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

	aruco = aruco_detect()
	aruco.cX = 0.0
	aruco.cY = 0.0
	aruco.distance = 0.0
	aruco.edge_distance=0.0


	if ids is None: ##If no marker is detected
		print("FINDING MARKERS!!!")
		# cv2.imshow("frame",img)
		# cv2.waitKey(3)
		aruco.flag = 0
	else:
		a = np.where(ids==0)  #tuple containing index of the aruco with id zero.
		if a[0].size==0:
			# cv2.imshow("frame",img)
			# cv2.waitKey(3)
			aruco.flag = 0
		else:
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

<<<<<<< HEAD
			cv2.imshow("frame",img)
			cv2.waitKey(3)
			print ("Cx is ", cX)
			print ("CY is ", cY)
			print ("Image_shape_0",img.shape[0])
			print ("Image_shape_0",img.shape[1])
			edge_distance = sqrt( (topLeft[0]-topRight[0])**2 + (topLeft[0]-topRight[0])**2 )//2
=======
			# cv2.imshow("frame",img)
			# cv2.waitKey(3)

			#edge_distance = sqrt( (topLeft[0]-topRight[0])**2 + (topLeft[0]-topRight[0])**2 )//2
>>>>>>> aff46c525661423cd9965fa7492a567a80bc5ca0
			distance = 	sqrt( (cX-img.shape[1]//2)**2 + (cY-img.shape[0]//2)**2 )
			print(distance)
			aruco.flag = 1
			aruco.cY = cX-img.shape[1]//2
			aruco.cX = -cY+img.shape[0]//2
			aruco.distance = distance
			aruco.edge_distance=edge_distance

	pub_aruco_detect.publish(aruco)



if __name__ == '__main__':

	 rospy.init_node('aruco_detector', anonymous=True)
	 pub_aruco_detect = rospy.Publisher("/aruco_detect", aruco_detect,queue_size=10)
	 rospy.Subscriber("/camera/color/image_raw/", Image, callback_opencv)
	 
	 rospy.spin()