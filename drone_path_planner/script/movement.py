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
#!/usr/bin/env python


import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2

x = 0.0
y = 0.0
z = 0.0
theta = 0.0

def newOdom (msg) :
    global x
    global y
    global z
    global theta
    
    x = msg.pose.position.x   ##pose.position.x
    y = msg.pose.position.y    ##pose.position.y
    z = msg.pose.position.z
    rot_q =msg.pose.orientation
    (roll ,pitch ,theta)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])

rospy.init_node("speed_controller")
sub = rospy.Subscriber("/ground_truth_to_tf/pose",PoseStamped ,newOdom)
pub = rospy.Publisher("/cmd_vel/",Twist,queue_size=1)   
  
speed =Twist()
r= rospy.Rate(1000) 
goal = Point()
goal.x =9
goal.y =2
goal.z =7

while not rospy.is_shutdown():

    inc_x=goal.x - x
    inc_y=goal.y - y
    inc_z=goal.z - z
    print(x,y,z)
    print(goal.x,goal.y,goal.z)
    print(inc_x,inc_y,inc_z)
    angle_to_goal = atan2(inc_y,inc_x)
    if inc_z > 0.1 :

        speed.linear.z = 0.5
    else :
 
       
        if abs(angle_to_goal - theta )>0.1 :
            
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            print(speed.linear.z)

        else :
            
            speed.linear.x = 0.5
            speed.angular.z = 0.0
            print(speed.linear.z)

    pub.publish(speed)       
    r.sleep()      
