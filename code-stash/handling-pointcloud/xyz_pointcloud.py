#!/usr/bin/env python

# task: print xyz from sensor_msgs::PointCloud2 message from /camera/depth/points

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy

def pc2Callback(pc2_msg):
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
    print("Received")
    print(xyz_array)

if __name__ == '__main__':
    try:
        rospy.init_node('pc2xyz_node')
        pc2_topic = '/camera/depth/points'
        pc2_subscriber = rospy.Subscriber(pc2_topic, PointCloud2, pc2Callback)
        rospy.spin()
    except rospy.ROSInterrupyException:
        rospy.loginfo("node terminated.")
