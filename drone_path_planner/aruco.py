#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from PIL import Image as C
import matplotlib.pyplot as plt
import math
from drone_path_planner.msg import teleopData


cameraMat = np.zeros((3,3))
cameraMat[0][0] = 474.250810
cameraMat[0][2] = 403.777430
cameraMat[1][1] = 474.152947
cameraMat[1][2] = 399.072316
cameraMat[2][2] = 1.0

dist_mat = np.array([-0.000568,-0.000983,-0.000168,0.001396,0.000000])

toPublish = teleopData()

first1 = True
first2 = True

counter1 = 1

def loadImages(data):
     imgMat = np.zeros((len(data.data)))
     global counter1
     index = 0
     counter1 += 1
     for i in data.data:
          imgMat[index] = ord(i)
          index += 1
     
     imgMat = imgMat.reshape((data.height,data.width,3))
     arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
     arucoParams = cv2.aruco.DetectorParameters_create()
     imgMat = imgMat.astype(np.uint8)
     imgMat = cv2.medianBlur(imgMat,3)
     (corners, ids, rejected) = cv2.aruco.detectMarkers(imgMat, arucoDict,parameters=arucoParams)

     if ids is None:
          print("LP")
     else:
          for id in ids:
               print(id)

               center = [0,0]
               center[0] = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4.0
               center[1] = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4.0
               lengthOfEdgeInPixel = math.sqrt((corners[0][0][0][0] - corners[0][0][1][0])*2 + (corners[0][0][0][1] - corners[0][0][1][1])*2)

               scaleFactor = 0.1778/lengthOfEdgeInPixel

               translationVectorInPixels = [(center[0] - 320),(center[1] - 240)]
               translationVectorDrone = [-scaleFactor*translationVectorInPixels[1],scaleFactor*translationVectorInPixels[0]]
               msg = teleopData()
               global first1
               global first2
               if((abs(translationVectorInPixels[0]) > 0.1/scaleFactor) and counter1%10 == 0):

                    msg.decision = 2
                    msg.delta = translationVectorDrone[1]
                    print(translationVectorDrone[1])
                    pub.publish(msg)
                    first1 = False
               elif((abs(translationVectorInPixels[1]) > 0.1/scaleFactor) and counter1%10 == 0):
                    msg.decision = 1
                    msg.delta = translationVectorDrone[0]
                    print(translationVectorDrone[0])
                    pub.publish(msg)
                    first2 = False

               print("PIXELS", translationVectorInPixels)
               imgMat = imgMat.astype(np.float)
               imgMat = np.divide(imgMat,255.0)
               cv2.imshow("Label",imgMat)
               cv2.waitKey(3)


if __name__ == '_main_':

     rospy.init_node('listener', anonymous=True)

     pub = rospy.Publisher("/drone/teleop",teleopData)

     rospy.Subscriber("/camera/color/image_raw/", Image, loadImages)

     rospy.spin()