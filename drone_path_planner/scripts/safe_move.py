#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point,Twist
from geometry_msgs.msg import PoseStamped
from math import atan2, cos, sin
from nav_msgs.msg import *
from drone_path_planner.msg import teleopData
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from std_msgs.msg import Int16
#from geometry_msgs import PoseStamped
class navigation:
    def __init__(self):
        self.rate=rospy.Rate(1)
        self.x_pose=0.0
        self.y_pose=0.0
        self.z_pose=0.0
        self.x=0.0
        self.y=0.0
        self.z=0.0        
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.delta=0.0
        self.decision=0
        self.indicator=0.0
        self.safesearch_flag=0.0
        self.pub_set_point_local=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.sub1=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
        self.sub_safesaerch_start=rospy.Subscriber("/safesearch/start",Int16,self.safesearch_start_callback)
        self.subl2=rospy.Subscriber("/safesearch/teleop",teleopData,self.decision_calback)
        self.msgp=PoseStamped()


    def safesearch_start_callback(self,msg):
        self.safesearch_flag=msg.data
        if(self.safesearch_flag==1  and self.indicator==0):
            self.x_pose=self.x
            self.y_pose=self.y
            self.z_pose=self.z
            self.indicator=1
           # print(self.indicator)
            print("pose reset")

        elif (self.safesearch_flag==0 and self.indicator==1):
            self.indicator=0


    def decision_calback(self,msg):
        self.decision=msg.decision
        self.delta=msg.delta

    def gps_data_callback(self,msg):
        self.x = msg.pose.pose.position.x   
        self.y = msg.pose.pose.position.y   
        self.z_pose = msg.pose.pose.position.z
        rot_q =msg.pose.pose.orientation
        self.msgp.pose.orientation=msg.pose.pose.orientation
        (self.roll ,self.pitch ,self.yaw)=euler_from_quaternion([rot_q.x ,rot_q.y,rot_q.z ,rot_q.w])

    def move_yaw(self) :
        self.msgp.pose.position.z=self.z_pose
        self.msgp.pose.position.x=self.x_pose
        self.msgp.pose.position.y=self.y_pose
        self.yaw=self.yaw+self.delta*(3.14/180)
        q=quaternion_from_euler(self.roll ,self.pitch ,self.yaw)
        self.msgp.pose.orientation.x = q[0]
        self.msgp.pose.orientation.y = q[1]
        self.msgp.pose.orientation.z = q[2]
        self.msgp.pose.orientation.w = q[3]
    def set_z(self):
        self.msgp.pose.position.x=self.x_pose
        self.msgp.pose.position.y=self.y_pose
        self.msgp.pose.position.z=self.delta       
    def nav(self):
        while not rospy.is_shutdown():
            if (self.decision==4):
                self.set_z()
                self.pub_set_point_local.publish(self.msgp)
                self.decision=0
            elif (self.decision==5) :
                self.move_yaw()
                self.pub_set_point_local.publish(self.msgp)
                self.decision=0
            self.rate.sleep()
        

if __name__ == '__main__':
  rospy.init_node('survey_navigator')
  rospy.loginfo("surveil_node created")
  try:
    navigation_obj = navigation()  
    #setStabilizeMode()
    # setArm()
    # setTakeoffMode()
    rate4=rospy.Rate(1000)
    rate4.sleep()
    # rospy.loginfo("Flight take off done successful and is ready for accepting furthur commands")
    ## Wait until node has loaded completely
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to accept the commands for teleop...")
    navigation_obj.nav()

  except rospy.ROSInterruptException:
    pass
