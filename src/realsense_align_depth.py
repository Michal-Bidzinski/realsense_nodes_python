#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# for point_cloud
from sensor_msgs.msg import Image, CameraInfo

from library.camera_functions import set_pipeline, record_rosbag, get_camera_info, use_CvBridge, image_CvBridge_conversion, align_depth_to_color, get_timestamp

class camera_L515():
    def __init__(self):
        # D435 pipeline
        self.pipeline, self.config = set_pipeline(["depth"])

        # Record rosbag
        record_rosbag(self.config)

        # publisher definition
        self.set_publisher()

        # get camera info to publish 
        self.camera_info = get_camera_info(self.pipeline, ["depth"])

        # use CvBridge conversion for image
        self.bridge = use_CvBridge()

        # Align depth to rgb image
        self.align = align_depth_to_color()

        print("Start node")

    # publisher definition
    def set_publisher(self): 
        self.pub_align = rospy.Publisher("align_depth", Image, queue_size=2)
        self.pub_camera_info = rospy.Publisher("camera_info", CameraInfo, queue_size=2)

    # camera callback
    def get_frame(self):
        # Get data from cameras
        frames = self.pipeline.wait_for_frames()

        aligned_frames = self.align.process(frames)
        self.aligned_depth_frame = aligned_frames.get_depth_frame()

        self.time = get_timestamp(frames)

        self.align_message = image_CvBridge_conversion(self.aligned_depth_frame, self.bridge, self.time)

        self.publish_data()

    # publish messages
    def publish_data(self):
        # Publish camera info
        self.pub_camera_info.publish(self.camera_info)

        # Publish align depth 
        self.pub_align.publish(self.align_message)

    # stop streaming camera data
    def stop_streaming(self):
        self.pipeline.stop()


def main():

    # node init
    rospy.init_node('realsense_align_depth', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    # create camera object
    cam_L515 = camera_L515()
    
    # main loop 
    while not rospy.is_shutdown():
        # get and publish data
        cam_L515.get_frame()
        rate.sleep()

    # Stop streaming
    cam_D435.stop_streaming()

if __name__ == '__main__':
    main()

