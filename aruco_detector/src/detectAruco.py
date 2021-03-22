#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
from drone_path_planner.msg import teleopData
from mavros_msgs.srv import SetMode

#Camera matrix if needed
# cameraMat = np.zeros((3,3))
# cameraMat[0][0] = 474.250810
# cameraMat[0][2] = 403.777430
# cameraMat[1][1] = 474.152947
# cameraMat[1][2] = 399.072316
# cameraMat[2][2] = 1.0

#Distortion matrix
# dist_mat = np.array([-0.000568,-0.000983,-0.000168,0.001396,0.000000])

first1 = True
first2 = True
counter1 = 1

def loadImages(data):

     #initializing some varibles

     imgMat = np.zeros((len(data.data)))
     index = 0
     global first2
     global first1
     global counter1
     counter1 += 1 #To count how many number of times this function is called

     ## Converting uchar image matrix to int using ord function
     for i in data.data:
          imgMat[index] = ord(i)
          index += 1

     
     ##Actual Aruco detection part:     
     imgMat = imgMat.reshape((data.height,data.width,3))
     arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
     arucoParams = cv2.aruco.DetectorParameters_create()
     imgMat = imgMat.astype(np.uint8)
     imgMat = cv2.medianBlur(imgMat,3)
     (corners, ids, rejected) = cv2.aruco.detectMarkers(imgMat, arucoDict,parameters=arucoParams)

     if ids is None: ##If no marker is detected
          print("FINDING MARKERS:")
     else:     
          msg = teleopData()
          if first2: #Run this if statement only once; once a marker is detected, drop to a height of 3
               msg.decision = 4
               msg.delta = 3
               pub.publish(msg)
               first2 = False


          markerIndex = None     #Stores index of marker 0 in the array of identified markers
          print(ids)      #Print all the identified ids

          ##Search for id 0
          for i in range(len(ids)):
               if ids[i][0] == 0:
                    markerIndex = i
               
          if markerIndex is not None: #Only if id 0 is found

               #Calculate the center of the aruco marker
               center = (corners[markerIndex][0][0] + corners[markerIndex][0][1] + corners[markerIndex][0][2] + corners[markerIndex][0][3])/4.0
          
               #Get the scaled distance to the center of the aruco marker
               lengthOfEdgeInPixel = math.sqrt((corners[markerIndex][0][0][0] - corners[markerIndex][0][1][0])**2 + (corners[markerIndex][0][0][1] - corners[markerIndex][0][1][1])**2)
               scaleFactor = 0.13/lengthOfEdgeInPixel
               scaleFactor2 = 0.1778/lengthOfEdgeInPixel
               translationVectorInPixels = [(center[0] - 320),(center[1] - 240)] #Translation in terms of pixels
               translationVectorDrone = [-scaleFactor*translationVectorInPixels[1],scaleFactor*translationVectorInPixels[0]] # Actual translation distance

               ##Optimize distance in X
               if((abs(translationVectorInPixels[0]) > 0.1/scaleFactor2) and counter1%3 == 0):
                    msg.decision = 2
                    msg.delta = translationVectorDrone[1]
                    print(translationVectorDrone[1])
                    pub.publish(msg)
               
               ##Optimize distance in y
               elif((abs(translationVectorInPixels[1]) > 0.1/scaleFactor2) and counter1%3 == 0):
                    msg.decision = 1
                    msg.delta = translationVectorDrone[0]
                    print(translationVectorDrone[0])
                    pub.publish(msg)
               
               ##Once both x and y get optimized, decrease the altitude
               elif((abs(translationVectorInPixels[0]) < 0.1/scaleFactor2) and (abs(translationVectorInPixels[1]) < 0.1/scaleFactor2) and first1):
                    # msg.decision = 4
                    # msg.delta = 1.5
                    # pub.publish(msg)
                    first1 = False
                    setStabilizeMode()

               #Display what the camera sees
     imgMat = imgMat.astype(np.float)
     imgMat = np.divide(imgMat,255.0)
     cv2.imshow("Label",imgMat)
     cv2.waitKey(3)


#Function to call Land service
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


if __name__ == '__main__':

     rospy.init_node('listener', anonymous=True)
     pub = rospy.Publisher("/drone/teleop",teleopData)
     rospy.Subscriber("/camera/color/image_raw/", Image, loadImages)
     rospy.spin()
