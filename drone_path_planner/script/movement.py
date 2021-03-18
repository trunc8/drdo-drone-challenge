#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep

v=2       #reference linear velocity
w=1.25    #reference angular velocity
x_pose=0.0f;
y_pose=0.0f;
z_pose=0.0f;
roll=0.0f;
pitch=0.0f
yaw=0.0f;

def move_forward(delta):
    

def rotate(w):
    pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    rospy.init_node('drone_move',anonymous=True)
    vel=Twist()
    vel.linear.x=vel.linear.y=vel.linear.z=0
    vel.angular.x=vel.angular.y=0
    vel.angular.z=w
    
    t0=rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        pub.publish(vel)
        if rospy.Time.now().to_sec()>1+t0:
            break 

    vel.angular.z=0
    pub.publish(vel)

def move():
    pub_set_point_local=rospy.Publisher('/mavros/setpoint_raw/local',Twist,queue_size=10)
    rospy.Subscriber("/mavros/global_position/local",nav_msgs/Odometry,gps_data_callback)
    rospy.init_node('drone_move',anonymous=True)
    vel=Twist()
    vel.linear.x=vel.linear.y=vel.linear.z=0
    vel.angular.x=vel.angular.y=vel.angular.z=0
    pub.publish(vel)
    sleep(1)
    #movement algo comes here

if __name__=="__main__":
   `try:
       move()
    except rospy.ROSInterruptException:
        pass