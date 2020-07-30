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

# T265
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.pose)

# Start streaming
pipeline.start(config)

# Start streaming with requested config
config.enable_record_to_file('test1.bag')

# Node init and publisher definition
rospy.init_node('mapping', anonymous = True)
pub_path = rospy.Publisher("/my_path", Path, queue_size = 100)
rate = rospy.Rate(30) # 30hz

# init trajectory variables
my_path = Path()
my_path.header.frame_id = 'map'


print("Start node")


while not rospy.is_shutdown():
    
    # Get data from camera
    trajectory = pipeline.wait_for_frames()
    pose = trajectory.get_pose_frame()

    # create path, get position and orientation
    my_path, position, orientation, odom = get_path_position_orientation(pose, my_path)

    #publish path
    pub_path.publish(my_path)

    rate.sleep()

# Stop streaming
pipeline.stop()

