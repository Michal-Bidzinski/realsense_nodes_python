#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration


class camera_D435():
    def __init__(self):
        self.set_pipeline()
        self.record_rosbag()
        self.set_pointcloud_variable()
        self.set_publisher()
        self.get_camera_info()
        self.use_CvBridge()
        print("Start node")

    # D435 pipeline
    def set_pipeline(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def record_rosbag(self):
        self.config.enable_record_to_file('test1.bag')

    # variable for create and store pointcloud
    def set_pointcloud_variable(self):
        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
        self.colorizer = rs.colorizer()

    # publisher definition
    def set_publisher(self): 
        self.pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
        self.pub_camera_info = rospy.Publisher("camera_info", CameraInfo, queue_size=2)

    # get camera info to publish 
    def get_camera_info(self):
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()

        self.camera_info = CameraInfo()
        self.camera_info.width = color_intrinsics.width
        self.camera_info.height = color_intrinsics.height
        self.camera_info.distortion_model = 'plumb_bob'
        cx = color_intrinsics.ppx
        cy = color_intrinsics.ppy
        fx = color_intrinsics.fx
        fy = color_intrinsics.fy
        self.camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info.D = [0, 0, 0, 0, 0]
        self.camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        self.camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]

    # use CvBridge conversion for image
    def use_CvBridge(self):
        self.bridge = CvBridge()

    # camera callback
    def get_frame(self):
        # Get data from cameras
        frames = self.pipeline.wait_for_frames()
        self.timestamp = frames.get_timestamp()
        self.color_frame = frames.get_color_frame()
        self.depth_frame = frames.get_depth_frame()

        self.create_point_cloud()
        self.publish_data()


    # publish messages
    def publish_data(self):
        # Publish camera info
        self.pub_camera_info.publish(self.camera_info)

        # Publish pointcloud
        self.pub_pointcloud.publish(self.pc2)

    # create point_cloud from depth image
    def create_point_cloud(self):
        _, _, points = get_point_cloud(self.depth_frame, self.color_frame, self.pc, self.decimate, self.colorizer)
        points = point_cloud_filtration(points, 0.01)
        self.pc2 = create_PointCloud2(points, self.timestamp)

    # stop streaming camera data
    def stop_streaming(self):
        self.pipeline.stop()

def main():

    # node init
    rospy.init_node('realsense_point_cloud', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    # create camera object
    cam_D435 = camera_D435()
    
    # main loop 
    while not rospy.is_shutdown():
        # get and publish image and camera_info
        cam_D435.get_frame()
        rate.sleep()

    # Stop streaming
    cam_D435.stop_streaming()

if __name__ == '__main__':
    main()

