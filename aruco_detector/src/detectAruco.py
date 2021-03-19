#!/usr/bin/env python
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <opencv2/aruco.hpp>

# using namespace std;

# void loadImages(sensor_msgs::ImageConstPtr &data){

# // vector<unsigned char> tmp = data.data;
# // int h = data.height;
# // int w = data.width;

# // //cv::Mat img3 = cv::Mat(h,w,tmp.data()).clone();
# // cv::Mat img2(h,w,tmp.data());

# cout<<"HELLO"<<endl;

# }

# int main(int argc, char **argv){

#      ros::init(argc,argv,"GetImages");

#      ros::NodeHandle n;

#      ros::Subscriber getImages = n.subscribe("/camera/color/image_raw/", 1000, loadImages);

#      ros::spin();

#      return 0;
# }

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
     # cv2.aruco.drawAxis(tx,cameraMat,dist_mat,vecs[1],vecs[0],0.1778)
     # cv2.imshow("some",tx)
     print("T VEC ",vecs[0])
     print("R VEC ",vecs[1])
     
     print(corners)
     if ids is None:
          print("LP")
     else:
          print(ids)
     #imgMat = np.divide(imgMat,255.0)
     # k = 0
     # for g in imgMat:
     #      imgMat[k] = imgMat[k]/255.0
     #      k += 1
     imgMat = imgMat.astype(np.float)
     imgMat = np.divide(imgMat,255.0)
     plt.imshow(imgMat)
     plt.show()
def listner():

     rospy.init_node('listener', anonymous=True)

     rospy.Subscriber("/camera/color/image_raw/", Image, loadImages)

     rospy.spin()

if __name__ == '__main__':
     listner()