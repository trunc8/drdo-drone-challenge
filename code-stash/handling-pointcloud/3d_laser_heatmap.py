#!/usr/bin/env python

# task: store the exposure levels for each voxel of the
# world. Here exposure is derived from pointcloud2 data
# associated with the Kinect sensor on the waffle tb3
# approach: store heatmap as 3d numpy array, each element
# is a voxel. First subscribe to pointcloud data, convert
# to x,y,z coordinates in rgb_optical_frame, using tf
# further convert to x,y,z coordinates in world frame
# (odom).
# Locally x,y,z will need to be converted to d,theta,phi
# since the formula for exposure is in spherical coordinates.
# This computation is done for each point in the pointcloud
# and the obtained exposure intensity is added to the 
# 3d numpy array(heatmap)

from geometry_msgs.msg import PointStamped, PoseStamped
from math import floor, ceil
import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import tf2_geometry_msgs
import tf2_ros
import tf2_sensor_msgs

class Heatmap3D:
    def __init__(self):
        self.Lx = 5
        self.Ly = 5
        self.Lz = 2
        self.r = 10 # Resolution that 1 metre is divided into
        self.world = np.zeros([self.Lx*self.r, self.Ly*self.r, self.Lz*self.r])
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = None

    def updateHeatmap(self, pc2_msg):
        target_frame = 'odom'
        source_frame = 'camera_rgb_optical_frame'
        while (self.transform is None):
            try:
                self.transform = self.tf_buffer.lookup_transform(target_frame,
                    source_frame,
                    rospy.Time(0), # get the tf at first available time
                    rospy.Duration(1.0) # wait for 1 second
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
                    rospy.loginfo("Exception occurred")
                    continue
        rospy.loginfo("Transform starts")
        tf_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(pc2_msg,
            self.transform)

        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
        tf_xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tf_cloud)
        # ps = PoseStamped()
        # ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = 'camera_rgb_optical_frame'
        # if(len(xyz_array)==len(tf_xyz_array)):
            # print("they are equal!")
        rospy.loginfo("Updating world")
        for (pt,tf_pt) in zip(xyz_array, tf_xyz_array):
            tf_x = int(tf_pt[0])
            tf_y = int(tf_pt[1])
            tf_z = int(tf_pt[2])
            increment = 0.001
            self.world[tf_x*self.r, tf_y*self.r, tf_z*self.r] += increment

            # ps.pose.position.x = pc2_point[0]
            # ps.pose.position.y = pc2_point[1]
            # ps.pose.position.z = pc2_point[2]
            # ps.pose.orientation.x = 0
            # ps.pose.orientation.y = 0
            # ps.pose.orientation.z = 0
            # ps.pose.orientation.w = 1
            # target_ps = tf2_geometry_msgs.do_transform_pose(ps, self.transform)
            # target_pt = target_ps.pose.position
            # if (target_pt.x>=0 and target_pt.x<=self.Lx and
                # target_pt.y>=0 and target_pt.y<=self.Ly and
                # target_pt.z>=0 and target_pt.z<=self.Lz):
                # # increment is obtained from function of UV exposure
                # increment = 0.001 # dummy value
                # self.world[0, 0, 0] += increment

            # target_pt = self.tf_buffer.transform(pt, target_frame)

        rospy.loginfo("Iteration ended successfully!")
        self.transform = None


if __name__ == '__main__':
    try:
        rospy.init_node('heatmap_node', anonymous=False)
        hm3d = Heatmap3D()
        pc2_topic = '/camera/depth/points'
        pc2_subscriber = rospy.Subscriber(pc2_topic, PointCloud2,
            hm3d.updateHeatmap)
        rospy.spin()
    except ROSInterruptException:
        rospy.loginfo("Node terminated.")

