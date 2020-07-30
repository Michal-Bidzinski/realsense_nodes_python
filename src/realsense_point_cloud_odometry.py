#!/usr/bin/python3.6

import pyrealsense2 as rs
import rospy
import math as m
import struct
import time
import cv2
import numpy as np
import open3d as o3d

# for trajectory 
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2


# D435
pipeline_point_cloud = rs.pipeline()
config_point_cloud = rs.config()
config_point_cloud.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_point_cloud.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# T265
pipeline_trajectory = rs.pipeline()
config_trajectory = rs.config()
config_trajectory.enable_stream(rs.stream.pose)

# Start streaming
pipeline_point_cloud.start(config_point_cloud)
pipeline_trajectory.start(config_trajectory)

# Start streaming with requested config
config_point_cloud.enable_record_to_file('test1.bag')
config_trajectory.enable_record_to_file('test2.bag')

# Align to depth 
align_to = rs.stream.color
align = rs.align(align_to)

# Node init and publisher definition
rospy.init_node('mapping', anonymous = True)
pub_path = rospy.Publisher("/my_path", Path, queue_size = 100)
pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
pub_image = rospy.Publisher("image_reaslsense", Image, queue_size=2)
rate = rospy.Rate(30) # 30hz

# init trajectory variables
my_path = Path()
my_path.header.frame_id = 'map'

# init pointclaud variables
# Get stream profile and camera intrinsics
profile = pipeline_point_cloud.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
colorizer = rs.colorizer()
old_points = np.array([[0, 0, 0]]) 

# get camera data
# depth
profile = pipeline_point_cloud.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# color
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()


print("Start node")


while not rospy.is_shutdown():
    
    # Get data from cameras
    frames = pipeline_point_cloud.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    trajectory = pipeline_trajectory.wait_for_frames()
    pose = trajectory.get_pose_frame()

    # create path, get position and orientation
    my_path, position, orientation, odom = get_path_position_orientation(pose, my_path)

    #publish path
    pub_path.publish(my_path)

    # create point_cloud
    verts, _, color_image = get_point_cloud(depth_frame, color_frame, pc, decimate, colorizer)
    points = transform_point_cloud(verts, position, orientation)
    pc2, old_points = create_PointCloud2(points, old_points, color_image)
    pub_pointcloud.publish(pc2)

    rate.sleep()

# Stop streaming
pipeline_point_cloud.stop()
pipeline_trajectory.stop()

