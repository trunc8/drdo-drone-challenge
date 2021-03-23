#!/usr/bin/env python
# from __future__ import print_function

import rospy

import math
##from hector_uav_msgs.msg import PoseActionGoal

from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import *
from drdo_exploration.msg import direction #Here direction is the message containing target co-ordinates.
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped

import numpy as np

#global variable
latitude =0.0
longitude=0.0

takeoff = False


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e
        
def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e


def setTakeoffMode():
    global takeoff
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        response = takeoffService(altitude = 2.5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        # print("entering")
        takeoff = response.success
        if takeoff:
        	rospy.signal_shutdown("Hehe")
        # print("Takeoff:", takeoff)
        # takeoff=1
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e
    
    

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude



if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    try:   
        
        while not takeoff:
            setGuidedMode()
            setArm()
            setTakeoffMode()
            rospy.sleep(1)
        print("SUCCESS!!")
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass
