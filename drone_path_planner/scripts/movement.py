#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import Odometry
from drone_path_planner.msg import teleopData
#from geometry_msgs import PoseStamped
class navigation:
    def __init__(self):
        self.rate=rospy.Rate(1)
        self.x_pose=0.0
        self.y_pose=0.0
        self.z_pose=0.0
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.delta=0.0
        self.decision=0
        self.pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.sub1=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
        self.subl2=rospy.Subscriber("/drone/teleop",teleopData,self.decision_calback)
        self.msgp=PoseStamped()
    def decision_calback(self,msg):
        self.decision=msg.decision
        self.delta=msg.delta
    def gps_data_callback(self,msg):
        self.x_pose = msg.pose.pose.position.x   
        self.y_pose = msg.pose.pose.position.y   
        self.z_pose = msg.pose.pose.position.z
        print(self.z_pose)
        rot_q =msg.pose.pose.orientation
        self.msgp.pose.orientation=msg.pose.pose.orientation
        (self.roll ,self.pitch ,self.theta)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])

    def move_forward(self):
        self.msgp.pose.position.z=self.z_pose
        self.msgp.pose.position.x=self.x_pose+self.delta*cos(self.yaw)
        self.msgp.pose.position.y=self.y_pose+self.delta*sin(self.yaw)
    def move_up(self):
        self.msgp.pose.position.z=self.z_pose+self.delta
        self.msgp.pose.position.x=self.x_pose
        self.msgp.pose.position.y=self.y_pose
    def move_right(self):
        self.msgp.pose.position.z=self.z_pose
        self.msgp.pose.position.x=self.x_pose+self.delta*sin(self.yaw)
        self.msgp.pose.position.y=self.y_pose-self.delta*cos(self.yaw)       
    def nav(self):
        while not rospy.is_shutdown():
            if (self.decision==1):
                self.move_forward()
                self.pub_set_point_local.publish(self.msgp)
                self.decision=0
            elif (self.decision==2):
                self.move_right()
                self.pub_set_point_local.publish(self.msgp)
                self.decision=0
            elif (self.decision==3):
                self.move_up()
                self.pub_set_point_local.publish(self.msgp)
                self.decision=0
            self.rate.sleep()

        

if __name__ == '__main__':
  rospy.init_node('navigator_node')
  rospy.loginfo("navigator_node created")
  try:
    navigation_obj = navigation()    
 
    ## Wait until node has loaded completely
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to accept the commands for teleop...")
    navigation_obj.nav()

  except rospy.ROSInterruptException:
    pass
