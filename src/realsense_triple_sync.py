#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import message_filters
import math as m

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration, get_point_cloud_from_topic


def cameras_callback(point_cloud1_msg, point_cloud2_msg, point_cloud3_msg):
    timestamp_1 = point_cloud1_msg.header.stamp
    timestamp_2 = point_cloud2_msg.header.stamp
    timestamp_3 = point_cloud3_msg.header.stamp
    print("PC_1: ", timestamp_1, " PC_2: ", timestamp_2, " PC_3: ", timestamp_3)

    print("PC_1 - PC_2: ", timestamp_1 - timestamp_2)

    print("PC_1 - PC_3: ", timestamp_1 - timestamp_3)

# Node init 
rospy.init_node('realsense_server', anonymous = True)

# Subscriber definition
point_cloud_1 = message_filters.Subscriber("PointCloud_M", PointCloud2)
point_cloud_2 = message_filters.Subscriber("PointCloud_R", PointCloud2)
point_cloud_3 = message_filters.Subscriber("PointCloud_L", PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([point_cloud_1, point_cloud_2, point_cloud_3], 3, 0.00000005, allow_headerless=True)
ts.registerCallback(cameras_callback)

# Publisher definition
#pub_odom = rospy.Publisher('odom_t265_sync', Odometry, queue_size=20)
#pub_pointcloud = rospy.Publisher("point_cloud2_sync", PointCloud2, queue_size=2)
rate = rospy.Rate(30) # 30hz

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()

#print("Start node")
rospy.loginfo("Realsense server is run")


while not rospy.is_shutdown():
    
    # Get data from cameras
    b=1
    rate.sleep()

