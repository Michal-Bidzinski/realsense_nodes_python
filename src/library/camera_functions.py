#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# for image message
from sensor_msgs.msg import Image, CameraInfo

# D435 pipeline
def set_pipeline(array):
    pipeline = rs.pipeline()
    config = rs.config()
    if "color" in array:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    if "depth" in array:
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if "odometry" in array:
        config.enable_stream(rs.stream.pose)

    # Start streaming
    pipeline.start(config)

    return pipeline, config

# record rosbag
def record_rosbag(config):
    config.enable_record_to_file('test1.bag')

# use CvBridge conversion for image
def use_CvBridge():
    return CvBridge()

# get camera info to publish 
def get_camera_info(pipeline):
    profile = pipeline.get_active_profile()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
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

    return camera_info

# convert_image_to_imgmsg
def image_CvBridge_conversion(frame, bridge, time):
    image = np.asanyarray(frame.get_data())
    message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    message.header.stamp = time
    message.header.frame_id = "map"

    return message

# Align depth to rgb image
def align_depth_to_color():
    align_to = rs.stream.color

    return rs.align(align_to)

# variable for create and store pointcloud
def set_pointcloud_variable():
    pc = rs.pointcloud()
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
    colorizer = rs.colorizer()

    return pc, decimate, colorizer

# count timestamp in ros format
def get_timestamp(frames):
    timestamp = frames.get_timestamp()
    t1 = (timestamp / 100000000)
    t2 = (t1 - int(t1)) * 100000
    time = rospy.Time(secs=int(t2), nsecs = int((t2 - int(t2))*100))

    return time

