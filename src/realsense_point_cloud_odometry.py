#!/usr/bin/python3.6
import pyrealsense2 as rs
import rospy
import cv2
import numpy as np
import open3d as o3d
import message_filters
import math as m

# for trajectory 
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path
from trajectory_fun import get_path_position_orientation
from nav_msgs.msg import Odometry

# for point_cloud
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, PointField
from std_msgs.msg import Header
from pointcloud_fun import get_point_cloud, transform_point_cloud, create_PointCloud2, point_cloud_filtration

class cameras_D435_T265():
    def __init__(self):
        self.set_publisher()
        self.set_subscriber()
        print("Start node")

    # publisher definition
    def set_publisher(self): 
        self.pub_odom = rospy.Publisher('odom_t265_sync', Odometry, queue_size=20)
        self.pub_pointcloud = rospy.Publisher("point_cloud2_sync", PointCloud2, queue_size=2)

    # publisher definition
    def set_subscriber(self):
        odometry = message_filters.Subscriber('odom_t265', Odometry)
        point_cloud = message_filters.Subscriber('point_cloud2', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([odometry, point_cloud], 2, 0.00000005,  allow_headerless=True)
        ts.registerCallback(self.cameras_callback)


    def cameras_callback(self, odom_msg, point_cloud2_msg):
        timestamp_pc = point_cloud2_msg.header.stamp
        timestamp_tr = odom_msg.header.stamp
        print("PC: ", timestamp_pc, " TR: ", timestamp_tr, " difference: ", timestamp_tr - timestamp_pc)

        self.point_cloud2_msg = point_cloud2_msg
        self.odom_msg = odom_msg

        self.point_cloud2_msg.header.stamp = rospy.Time()   
        self.odom_msg.header.stamp = rospy.Time() 

        self.publish_data() 
 

    # publish messages
    def publish_data(self):
        # Publish odom
        self.pub_odom.publish(self.odom_msg)

        # Publish camera info
        self.pub_pointcloud.publish(self.point_cloud2_msg)


def main():

    # node init
    rospy.init_node('realsense_server', anonymous = True)
    rate = rospy.Rate(30) # 30hz

    rospy.loginfo("Realsense server is run")

    # create camera object
    cam_D435_T265 = cameras_D435_T265()
    
    rospy.spin()


if __name__ == '__main__':
    main()

