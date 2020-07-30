#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import math as m
import struct
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Header


# D435
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Start streaming with requested config
config.enable_record_to_file('test1.bag')

# Node init and publisher definition
rospy.init_node('rgb_image', anonymous = True)
pub_color = rospy.Publisher("realsense_rgb", Image, queue_size=2)
pub_camera_info = rospy.Publisher("camera_info", CameraInfo, queue_size=2)
rate = rospy.Rate(30) # 30hz

# get color camera data
profile = pipeline.get_active_profile()
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()

camera_info = CameraInfo()
camera_info.width = color_intrinsics.width
camera_info.height = color_intrinsics.height
camera_info.distortion_model = 'plumb_bob'
cx = color_intrinsics.ppx
cy = color_intrinsics.ppy
fx = color_intrinsics.fx
fy = color_intrinsics.fy
camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
camera_info.D = [0, 0, 0, 0, 0]
camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]

bridge = CvBridge()

print("Start node")


while not rospy.is_shutdown():
    
    # Get data from cameras
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Publish color image
    color_image = np.asanyarray(color_frame.get_data())
    color_message = bridge.cv2_to_imgmsg(color_image, encoding="passthrough")
    pub_color.publish(color_message)

    # Publish camera info
    pub_camera_info.publish(camera_info)

    rate.sleep()

# Stop streaming
pipeline.stop()
