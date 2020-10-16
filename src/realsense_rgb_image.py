#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# for image message
from sensor_msgs.msg import Image, CameraInfo

from library.camera_functions import set_pipeline, record_rosbag, get_camera_info, use_CvBridge, image_CvBridge_conversion, get_timestamp


class camera_D435():
    def __init__(self):
        # D435 pipeline
        self.pipeline, self.config = set_pipeline(["color"])

        # Record rosbag
        record_rosbag(self.config)

        # use CvBridge conversion for image
        self.bridge = use_CvBridge()

        # publisher definition
        self.set_publisher()

        # get camera info to publish 
        self.camera_info = get_camera_info(self.pipeline)

        print("Start node")

    # publisher definition
    def set_publisher(self): 
        self.pub_color = rospy.Publisher("rgb_image", Image, queue_size=2)
        self.pub_camera_info = rospy.Publisher("camera_info", CameraInfo, queue_size=2)

    # publish messages
    def publish_data(self):
        # Publish color image
        self.pub_color.publish(self.color_message)

        # Publish camera info
        self.pub_camera_info.publish(self.camera_info)

    # camera callback
    def get_frame(self):
        # Get data from cameras
        frames = self.pipeline.wait_for_frames()
        self.color_frame = frames.get_color_frame()

        self.time = get_timestamp(frames)

        self.color_message = image_CvBridge_conversion(self.color_frame, self.bridge, self.time)
        self.publish_data()

    # stop streaming camera data
    def stop_streaming(self):
        self.pipeline.stop()


def main():

    # node init
    rospy.init_node('realsense_rgb_image', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    # create camera object
    cam_D435 = camera_D435()
    
    # main loop 
    while not rospy.is_shutdown():
        # get and publish data
        cam_D435.get_frame()
        rate.sleep()

    # Stop streaming
    cam_D435.stop_streaming()

if __name__ == '__main__':
    main()

