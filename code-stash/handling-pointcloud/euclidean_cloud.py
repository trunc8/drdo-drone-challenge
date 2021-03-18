#!/usr/bin/env python
# PointCloud2 euclidean distance color cube
import rospy
import struct
import colorsys
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class Euclidean_Point_Cloud:
	def __init__(self):
		self.num_pts = 8
		self.odom = None
		self.loop_rate = rospy.Rate(1)
		self.pub = rospy.Publisher('euclidean_point_cloud2', PointCloud2, queue_size=10)
		rospy.Subscriber('odom', Odometry, self.odomCallback)

	def odomCallback(self, msg):
		self.odom = msg
			
	def start(self):
		rospy.loginfo("Start point")
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
				  PointField('y', 4, PointField.FLOAT32, 1),
				  PointField('z', 8, PointField.FLOAT32, 1),
				  PointField('rgb', 16, PointField.UINT32, 1),
				  # PointField('rgba', 12, PointField.UINT32, 1),
				  ]
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "odom"


		while not rospy.is_shutdown():
			rospy.loginfo("Running")
			if self.odom is not None:
				x_bot = self.odom.pose.pose.position.x
				y_bot = self.odom.pose.pose.position.y
				z_bot = self.odom.pose.pose.position.z
			else:
				continue

			points = []
			for i in range(self.num_pts):
				for j in range(self.num_pts):
					for k in range(self.num_pts):
						x = float(i) / self.num_pts
						y = float(j) / self.num_pts
						z = float(k) / self.num_pts
						pt = [x, y, z, 0]


						pt1 = np.array([x_bot, y_bot, z_bot])
						pt2 = np.array([x, y, z])
						pt3 = np.array([self.num_pts,self.num_pts,self.num_pts])
						pt4 = np.array([0,0,0])
						max_dist = np.linalg.norm(pt1-pt3)
						min_dist = np.linalg.norm(pt1-pt4)
						dist = np.linalg.norm(pt1-pt2)

						h = 2*dist/(max_dist-min_dist)
						s = 1
						v = 1
						# r = int(x * 255.0)
						# g = int(y * 255.0)
						# b = int(z * 255.0)
						r,g,b = colorsys.hsv_to_rgb(h,s,v)
						r = int(r*255)
						g = int(g*255)
						b = int(b*255)
						a = 255
						print r, g, b, a
						rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
						print hex(rgb)
						pt[3] = rgb
						points.append(pt)

			pc2 = point_cloud2.create_cloud(header, fields, points)
			pc2.header.stamp = rospy.Time.now()
			self.pub.publish(pc2)
			self.loop_rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('create_euclidean_cloud', anonymous=True)
		eu_obj = Euclidean_Point_Cloud()
		eu_obj.start()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated.")