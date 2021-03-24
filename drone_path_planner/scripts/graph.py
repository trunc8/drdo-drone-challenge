#!/usr/bin/env python
import rospy
##from hector_uav_msgs.msg import PoseActionGoal
##from geometry_msgs import PoseStamped
from time import sleep
from geometry_msgs.msg import PoseStamped, Pose
from math import atan2, cos, sin
from nav_msgs.msg import * 
from std_msgs.msg import Int16
#from geometry_msgs import PoseStamped
# class map_graph_nodes:
# 	def __init__(self,parent,msg):
# 		self.position=msg.pose.position
# 		self.oreintation=msg.pose.oreintation
class safe_searcher_history:
	def __init__(self):
		self.sub1=rospy.Subscriber("/mavros/global_position/local",Odometry, self.gps_data_callback)
		self.gps_pose = Pose()   # change to pose
		self.node_list=[self.gps_pose]   # I need to figure out how to put current pose here while initialization
		self.index=0
		self.number_of_node_max=100
		# self.sub2=rospy.Subscriber("/back_track/flag",Int16, self.back_track_callback)
		self.sub_danger_flag=rospy.Subscriber("/safesearch/start",Int16,self.danger_flag_callback)
		# self.pub_data=rospy.Publisher('/back_track/data',Odometry,queue_size=10)
		self.publish
		self.counter=0

	def gps_data_callback(self,msg):
        self.gps_pose.position=msg.pose.pose.position
        self.gps_pose.oreintation=msg.pose.pose.oreintation
        self.counter=self.counter+1
        if(counter==4):
        	self.add_node(self.gps_pose)
        	counter=0
    def back_track_callback(self,msg):
    	if (msg.data==1):
    		self.pub_data.publish(self.node_list[index])
    		self.node_list.pop(self.index)
    		self.index=self.index-1
	def dist(self,msg):
		return sqrt((self.node_list[self.index].pose.position.x-msg.pose.position.x)**2+(self.node_list[self.index].pose.position.y-msg.pose.position.y)**2+(self.node_list[self.index].pose.position.z-msg.pose.position.z)**2)
	def add_node(self,msg):
		if (dist(msg)>0.4):
			if (number_of_node_max==index):
				node_list.pop(0)
				self.index-=1
			node_list.append(msg)
			self.index+=1

if __name__ == '__main__':
  rospy.init_node('safety_searcher')
  rospy.loginfo("safety_searcher nnode initialized")
  try:
  	back_track_obj=safe_searcher_history()
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to listen for safesearch/start")
    back_track_obj.back_track_flag()    

  except rospy.ROSInterruptException:
    pass	



