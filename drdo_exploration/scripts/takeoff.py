#!/usr/bin/env python
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

takeoff =0


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
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        print("entering")
        takeoff=1
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e
    
    

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

class moveCopter:
    def __init__(self):
        
        '''
        All values are initialized to zero.
        pub_set_point_local publishes the goal_point co-ordinates.
        sub_gps and sub_targ_vector subscribes to the global co-ordinates of the drone and the local co-ordinates of the goal_point.
        '''
        self.x_pose=0.0
        self.y_pose=0.0
        self.z_pose=0.0
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.targ_x=0.0
        self.targ_y=0.0
        self.targ_z=0.0
        self.rel_yaw = 0.0

        #rospy.init_node('navigator_node')
        self.pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
        self.sub_gps=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
        self.sub_targ_vector=rospy.Subscriber("/target_vector",direction, self.targ_vector_callback)
        self.msgp=PoseStamped()
        self.rate=rospy.Rate(1)

        self.e_prev = 0.0
        self.t_prev = 0.0
        self.I = 0.0
      


    def gps_data_callback(self,msg):
        ''' 
        msgp is of PoseStamped msg type that we need to publish for going to the goal_point.
        the use of roll, pitch, and yaw needs to be looked into.
        ''' 
        self.x_pose = msg.pose.pose.position.x   
        self.y_pose = msg.pose.pose.position.y   
        self.z_pose = msg.pose.pose.position.z
        rot_q =msg.pose.pose.orientation
        self.msgp.pose.orientation=msg.pose.pose.orientation
        (self.roll ,self.pitch ,self.yaw)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])



        #print("gps_data_callback: %.2fm %.2fm %.2f deg"%(self.x_pose, self.y_pose, self.yaw*180/3.14))
        

    def targ_vector_callback(self,msg):
        '''
        The local co-ordinates of the targets are read into the variables.
        As soon as we get that, we call the move_to_target function also which commands drone to go to that point
        '''
        self.targ_x=msg.vec_x
        self.targ_y=msg.vec_y
        self.targ_z=msg.vec_z
        self.rel_yaw = math.atan2(self.targ_y,self.targ_x)
        self.move_to_target()
        # self.rate.sleep()


    def move_to_target(self): 
        '''
        Here we find the final global co-ordinates by adding gps pose and message we figured out. 
        '''
        # print(self.msgp)
        q = quaternion_from_euler(0, 0, self.yawPID())
        # print(q)
        delta = 0.1
        delta_x = self.targ_x*np.cos(self.yaw)-self.targ_y*np.sin(self.yaw)
        delta_y = self.targ_x*np.sin(self.yaw)+self.targ_y*np.cos(self.yaw)
        delta_x = delta_x*delta
        delta_y = delta_y*delta
        self.msgp.pose.position.z=self.z_pose + self.targ_z*delta
        self.msgp.pose.position.x=self.x_pose + delta_x
        self.msgp.pose.position.y=self.y_pose + delta_y
        
        self.msgp.pose.orientation.x = q[0]
        self.msgp.pose.orientation.y = q[1]
        self.msgp.pose.orientation.z = q[2]
        self.msgp.pose.orientation.w = q[3]
        self.pub_set_point_local.publish(self.msgp)
        # print("Target pose")
        # print(self.msgp.pose.position.x, self.msgp.pose.position.y, self.msgp.pose.position.z)

        
    def yawPID(self):
    
      Kp = 0.8
      Kd = 0.1  
      Ki = 0

      e = self.rel_yaw
      #print(e*180/3.14)
      dt = 1

      P = Kp*e
      I = self.I + Ki*e*(dt)
      D = Kd*(e - self.e_prev)/(dt)

      Yaw = self.yaw + P + I + D

      self.e_prev = e
      self.I = I
      return Yaw
        
        
        
    

if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # setGuidedMode()
    # setArm()
    # setTakeoffMode()
    #setLandMode()
    # spin() simply keeps python from exiting until this node is stopped
    try:   
        if takeoff==0:
            setGuidedMode()
            setArm()
            setTakeoffMode()
            #setLandMode()
        else:
            print("not entering")
            moveCopter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    #listener()
    #myLoop()
    #rospy.spin()