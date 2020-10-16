#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration

from library.camera_functions import set_pipeline, record_rosbag, get_camera_info, use_CvBridge, image_CvBridge_conversion, align_depth_to_color, set_pointcloud_variable, get_timestamp


class camera_D435():
    def __init__(self):
        # D435 pipeline
        self.pipeline, self.config = set_pipeline(["color", "depth"])

        # Record rosbag
        record_rosbag(self.config)

        # publisher definition
        self.set_publisher()

        # get camera info to publish 
        self.camera_info = get_camera_info(self.pipeline)

        # use CvBridge conversion for image
        self.bridge = use_CvBridge()

        # variable for create and store pointcloud
        self.pc, self.decimate, self.colorizer = set_pointcloud_variable()

        # Align depth to rgb image
        self.align = align_depth_to_color()

        print("Start node")

    # publisher definition
    def set_publisher(self): 
        self.pub_color = rospy.Publisher("rgb_image", Image, queue_size=2)
        self.pub_align = rospy.Publisher("align_depth", Image, queue_size=2)
        self.pub_camera_info = rospy.Publisher("camera_info", CameraInfo, queue_size=2)
        self.pub_pointcloud = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

    # camera callback
    def get_frame(self):
        # Get data from cameras
        frames = self.pipeline.wait_for_frames()
        self.color_frame = frames.get_color_frame()
        self.depth_frame = frames.get_depth_frame()

        aligned_frames = self.align.process(frames)
        self.aligned_depth_frame = aligned_frames.get_depth_frame()

        self.time = get_timestamp(frames)

        self.color_message = image_CvBridge_conversion(self.color_frame, self.bridge, self.time)
        self.align_message = image_CvBridge_conversion(self.aligned_depth_frame, self.bridge, self.time)

        self.create_point_cloud()
        self.publish_data()


    # create point_cloud from depth image
    def create_point_cloud(self):
        _, _, points = get_point_cloud(self.depth_frame, self.color_frame, self.pc, self.decimate, self.colorizer)
        points = point_cloud_filtration(points, 0.01)
        self.pc2 = create_PointCloud2(points, self.time)

    # publish messages
    def publish_data(self):
        # Publish camera info
        self.camera_info.header.stamp = self.time
        self.pub_camera_info.publish(self.camera_info)

        # Publish color image
        self.pub_color.publish(self.color_message)

        # Publish align depth 
        self.pub_align.publish(self.align_message)

        # Publish pointcloud
        self.pub_pointcloud.publish(self.pc2)


    # stop streaming camera data
    def stop_streaming(self):
        self.pipeline.stop()

def main():

    # node init
    rospy.init_node('realsense_rgb_align_depth_point_cloud', anonymous = True)
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

