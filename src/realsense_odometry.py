#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d

# for trajectory 
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from trajectory_fun import get_path_position_orientation

from library.camera_functions import set_pipeline, record_rosbag, get_timestamp


class camera_T265():
    def __init__(self):
        # D435 pipeline
        self.pipeline, self.config = set_pipeline(["odometry"])

        # Record rosbag
        record_rosbag(self.config)

        # publisher definition
        self.set_publisher()
        
        # create path variable
        self.my_path = Path()
        self.my_path.header.frame_id = 'map'
        
        print("Start node")

    # publisher definition
    def set_publisher(self): 
        self.pub_path = rospy.Publisher("path", Path, queue_size = 100)
        self.pub_odom = rospy.Publisher('odom_t265', Odometry, queue_size=1)

    # camera callback
    def get_frame(self):
       # Get data from camera
       frames = self.pipeline.wait_for_frames()
       pose = frames.get_pose_frame()

       self.time = get_timestamp(frames)

       self.my_path, self.position, self.orientation, self.odom = get_path_position_orientation(pose, self.my_path, self.time)
       self.publish_data()

    # publish messages
    def publish_data(self):
        # publish path
        self.pub_path.publish(self.my_path)

        # publish odom
        self.pub_odom.publish(self.odom)

    # stop streaming camera data
    def stop_streaming(self):
        self.pipeline.stop()

def main():

    # node init
    rospy.init_node('realsense_trajectory', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    # create camera object
    cam_T265 = camera_T265()
    
    # main loop 
    while not rospy.is_shutdown():
        # get and publish data
        cam_T265.get_frame()
        rate.sleep()

    # Stop streaming
    cam_T265.stop_streaming()

if __name__ == '__main__':
    main()

