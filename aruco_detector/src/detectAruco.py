#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from PIL import Image as C
import matplotlib.pyplot as plt


cameraMat = np.zeros((3,3))
cameraMat[0][0] = 474.250810
cameraMat[0][2] = 403.777430
cameraMat[1][1] = 474.152947
cameraMat[1][2] = 399.072316
cameraMat[2][2] = 1.0

dist_mat = np.array([-0.000568,-0.000983,-0.000168,0.001396,0.000000])

def loadImages(data):
     imgMat = np.zeros((len(data.data)))
     index = 0
     for i in data.data:
          imgMat[index] = ord(i)
          index += 1
     
     imgMat = imgMat.reshape((data.height,data.width,3))
     arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
     arucoParams = cv2.aruco.DetectorParameters_create()
     imgMat = imgMat.astype(np.uint8)
     imgMat = cv2.medianBlur(imgMat,3)
     (corners, ids, rejected) = cv2.aruco.detectMarkers(imgMat, arucoDict,parameters=arucoParams)
     vecs = cv2.aruco.estimatePoseSingleMarkers(corners,0.1778,cameraMat,dist_mat)
     if ids is None:
          print("LP")
     else:
          print(ids)
          print("R VEC ",vecs[0])
          print("T VEC ",vecs[1])

          R_ct = np.matrix(cv2.Rodrigues(vecs[0])[0])
          R_tc = R_ct.T
          pos_camera = -R_ct*np.matrix(vecs[1]).T

          print(pos_camera)
     
          print(corners)
          imgMat = imgMat.astype(np.float)
          imgMat = np.divide(imgMat,255.0)
          cv2.imshow("Label",imgMat)
          cv2.waitKey(3)

def listner():

     rospy.init_node('listener', anonymous=True)

     rospy.Subscriber("/camera/color/image_raw/", Image, loadImages)

     rospy.spin()

if __name__ == '__main__':
     listner()
