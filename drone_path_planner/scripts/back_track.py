#!/usr/bin/env python
import time
from geometry_msgs.msg import Pose
class back_track :
	def __init__ (self):
		self.pub_back_track_flag=rospy.Publisher("/back_track/flag",Int16,queue_size=10)
		self.flag_msg=Int16()
		# self.sub_back_track_data=rospy.Subscriber("/back_track/data",Odometry, self.back_track_data_callback)
		# add publisher to go to pose function
		self.sub_danger_flag=rospy.Subscriber("/safesearch/start",Int16,self.danger_flag_callback)
		self.danger_flag=0
		self.rate=rospy.Rate(1)
		self.pose_data=Pose()
		self.indicator=0
	def danger_flag_callback(self,msg):
		self.danger_flag=msg.data
		if(danger_flag==1  and elf.indicator==0):
			self.pub_back_track_flag.publish(msg) # may be replace this with backtrach flag only after that history will take care
			self.indicator==1
		elif (danger_flag==0 and self.indicator==1):
			self.indicator=0
	# def back_track_data_callback(self,msg):
	# 	self.pose_data=msg
	# def start_backtarck(self):
	# 	while not rospy.is_shutdown():
	# 		if(self.danger_flag==1):
	# 			indicator=1
	# 			self.flag_msg.data=1
	# 			self.pub_back_track_flag.publish(self.flag_msg)
	# 			time.sleep(0.1) # to recieve data from the history node
	# 			# write go to pose wala function which waits until it reaches the pose
	# 			time.sleep(0.1) # waiting to recieve a new danger_flag after having moved back
	# 		if(self.indicator==1 && self.danger_flag==0):
	# 			#safe serach to start now
	# 			self.indicator=0
	# 		self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('safety_searcher')
  rospy.loginfo("safety_searcher nnode initialized")
  try:
  	back_track_obj=back_track()
    while (rospy.get_time()==0):
      pass
    rospy.loginfo("Beginning to listen for safesearch/start")
    back_track_obj.back_track_flag()    

  except rospy.ROSInterruptException:
    pass







	