#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import *
from drone_exploration.msg import direction #Here direction is the message containing target co-ordinates.
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
#from geometry_msgs import PoseStamped


class moveCopter:
    def __init__(self):
        self.rate=rospy.Rate(1)
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
        self.pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.sub_gps=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
        self.sub_targ_vector=rospy.Subscriber("/target_vector",direction, targ_vector_callback )
        self.msgp=PoseStamped()

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
        (self.roll ,self.pitch ,self.theta)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])

    def targ_vector_callback(self,msg):
        '''
        The local co-ordinates of the targets are read into the variables.
        As soon as we get that, we call the move_to_target function also which commands drone to go to that point
        '''
        self.targ_x=msg.vec_x
        self.targ_y=msg.vec_y
        self.targ_z=msg.vec_z
        move_to_target()

    def move_to_target(self): 
        '''
        Here we find the final global co-ordinates by adding gps pose and message we figured out. 
        '''
        self.msgp.pose.position.z=self.z_pose + self.targ_z*2
        self.msgp.pose.position.x=self.x_pose+  self.targ_x*2
        self.msgp.pose.position.y=self.y_pose+  self.targ_y*2
        self.pub_set_point_local.publish(self.msgp)
        

if __name__ == '__main__':
  try:   
    moveCopter()

  except rospy.ROSInterruptException:
    pass
        

    

