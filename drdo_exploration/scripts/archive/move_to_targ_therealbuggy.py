#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped 
from math import atan2, cos, sin
from nav_msgs.msg import *
from drdo_exploration.msg import direction #Here direction is the message containing target co-ordinates.
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


from mavros_msgs.msg import PositionTarget

from std_msgs.msg import Int16
from drdo_exploration.msg import aruco_detect

import numpy as np


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
    self.flag=0

    rospy.init_node('navigator_node')
    self.sub_gps=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback,queue_size=1)
    self.sub_aruco_detect = rospy.Subscriber("/aruco_detect", aruco_detect, self.aruco_detect_callback,queue_size=1)
    self.sub_targ_vector=rospy.Subscriber("/target_vector",direction, self.targ_vector_callback,queue_size=1)
    self.pub_set_point_local = rospy.Publisher('/mavros/setpoint_point/local', PoseStamped,queue_size=1)
    self.pub_set_point_raw = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=1)
    self.msgp_pose=PoseStamped()
    self.msgp_raw=PositionTarget()
    self.rate=rospy.Rate(4)

    self.e_prev = 0.0
    self.t_prev = 0.0
    self.I = 0.0
    self.edge_distance=0.0


    #### TUNABLES ######
    self.DELTA = 0.1 # m
    self.Kp = 0.1
    self.Kd = 0
    self.Ki = 0
    self.ERROR_THRESHOLD_FOR_INTEGRATOR = 0.2
    self.WINDUP_THRESHOLD = 0.2


  def gps_data_callback(self,msg):
    self.x_pose = msg.pose.pose.position.x   
    self.y_pose = msg.pose.pose.position.y   
    self.z_pose = msg.pose.pose.position.z
    rot_q =msg.pose.pose.orientation
    self.msgp_pose.pose.orientation=msg.pose.pose.orientation
    (self.roll ,self.pitch ,self.yaw)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])
            

  def targ_vector_callback(self,msg):
    self.targ_x=msg.vec_x
    self.targ_y=msg.vec_y
    self.targ_z=msg.vec_z
    self.rel_yaw = math.atan2(self.targ_y,self.targ_x)

  def aruco_detect_callback(self,msg):
    '''
    The local co-ordinates of the targets are read into the variables.
    As soon as we get that, we call the move_to_target function also which commands drone to go to that point
    '''
    self.flag=msg.flag
    self.cX=msg.cX
    self.cY=msg.cY
    self.distance=msg.distance
    self.edge_distance=msg.edge_distance

  def navigate(self):
    # this is the main funtion which will run in loop for all operation
    while not rospy.is_shutdown():
      if (self.flag):     
        Delta = self.distance/3000      

        self.msgp_pose.pose.position.x = self.x_pose + (self.cX)*Delta*cos(self.yaw)+(self.cY)*Delta*sin(self.yaw)
        self.msgp_pose.pose.position.y = self.y_pose - (self.cY)*Delta*cos(self.yaw)+(self.cX)*Delta*sin(self.yaw)  
        self.msgp_pose.pose.position.z = self.z_pose
        self.msgp_pose.pose.orientation = self.msgp_pose.pose.orientation

        self.pub_set_point_local.publish(self.msgp_pose)
        print("loop 1")
        if (self.distance<self.edge_distance):
          self.setLandMode()
      else:
        print("loop 2")
        self.goStraight()
      self.rate.sleep()

  def goStraight(self):
    self.msgp_raw.header.stamp = rospy.Time.now()
    self.msgp_raw.header.frame_id = ""
    self.msgp_raw.coordinate_frame = 8
    self.msgp_raw.type_mask = 448

    # here yaw should be the angle b//w current heading and targ vector
    # delta_x = self.targ_x*np.cos(self.rel_yaw)-self.targ_y*np.sin(self.rel_yaw) 
    # delta_y = self.targ_x*np.sin(self.rel_yaw)+self.targ_y*np.cos(self.rel_yaw)  
    # delta_x = delta_x
    # delta_y = delta_y
    
    self.msgp_raw.position.x = self.targ_x* self.DELTA
    self.msgp_raw.position.y = -self.targ_y* self.DELTA
    self.msgp_raw.position.z = self.targ_z* self.DELTA
    self.msgp_raw.velocity.x = 0
    self.msgp_raw.velocity.y = 0
    self.msgp_raw.velocity.z = 0
    self.msgp_raw.acceleration_or_force.x = 0
    self.msgp_raw.acceleration_or_force.y = 0
    self.msgp_raw.acceleration_or_force.z = 0
    # input_yaw = float(input("Enter yaw"))
    # raw_msg.yaw = input_yaw
    self.msgp_raw.yaw = -self.rel_yaw
    # self.msgp_raw.yaw = self.rel_yaw
    # raw_msg.yaw = min(LOWER_CLAMP, max(raw_msg.yaw, UPPER_CLAMP))
    self.msgp_raw.yaw_rate = 0.0
    #print (raw_msg)      
    self.pub_set_point_raw.publish(self.msgp_raw)
    # self.targ_x =0
    # self.targ_y =0
    # self.targ_z =0
    # self.rel_yaw=0
    # control frequncy of publishing aruco for controlling speed of bot


  def yawPID(self):
  
    e = self.rel_yaw
    #print(e*180/3.14)
    dt = 1

    P = self.Kp*e
    if abs(e) < self.ERROR_THRESHOLD_FOR_INTEGRATOR and abs(self.I) < self.WINDUP_THRESHOLD:
      self.I = self.I + self.Ki*e*(dt)
    D = self.Kd*(e - self.e_prev)/(dt)

    Yaw = P + self.I + D

    self.e_prev = e
    return Yaw

  def setLandMode(self):
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
      print ("service set_mode call failed. LAND Mode could not be set. Check that GPS is enabled")

      

if __name__ == '__main__':
  try:   
    moveCopter_obj=moveCopter()
    moveCopter_obj.navigate()
    rospy.spin()

  except rospy.ROSInterruptException:
    pass