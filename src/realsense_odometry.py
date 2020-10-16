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


class camera_T265():
    def __init__(self):
        self.set_pipeline()
        self.record_rosbag()
        self.set_publisher()
        self.set_trajectory_variable()
        print("Start node")

    # T265 pipeline
    def set_pipeline(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)

        # Start streaming
        self.pipeline.start(self.config)

    def record_rosbag(self):
        self.config.enable_record_to_file('test1.bag')

    # publisher definition
    def set_publisher(self): 
        self.pub_path = rospy.Publisher("path", Path, queue_size = 100)
        self.pub_odom = rospy.Publisher('odom_t265', Odometry, queue_size=1)

    # create path variable
    def set_trajectory_variable(self):
        self.my_path = Path()
        self.my_path.header.frame_id = 'map'

    # camera callback
    def get_frame(self):
       # Get data from camera
       trajectory = self.pipeline.wait_for_frames()
       self.timestamp = trajectory.get_timestamp()
       pose = trajectory.get_pose_frame()

       self.get_trajectory_and_odometry(pose)
       self.publish_data()

    # get path and odometry from data
    def get_trajectory_and_odometry(self, pose):
         self.my_path, self.position, self.orientation, self.odom = get_path_position_orientation(pose, self.my_path, self.timestamp)

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

