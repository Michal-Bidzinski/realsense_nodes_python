import pyrealsense2 as rs
import rospy
import math as m
import numpy as np

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry


def get_path_position_orientation(pose, my_path, timestamp):
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()

        w_r = data.rotation.w
        x_r = -data.rotation.z
        y_r = data.rotation.x
        z_r = -data.rotation.y

        pitch =  -m.asin(2.0 * (x_r*z_r - w_r*y_r)) * 180.0 / m.pi;
        roll  =  m.atan2(2.0 * (w_r*x_r + y_r*z_r), w_r*w_r - x_r*x_r - y_r*y_r + z_r*z_r) * 180.0 / m.pi
        yaw   =  m.atan2(2.0 * (w_r*z_r + x_r*y_r), w_r*w_r + x_r*x_r - y_r*y_r - z_r*z_r) * 180.0 / m.pi

        #create path
        pose = PoseStamped()
        pose.pose.position.x = data.translation.x 
        pose.pose.position.z = data.translation.y
        pose.pose.position.y = -data.translation.z  

        pose.pose.orientation.x = pitch
        pose.pose.orientation.z = yaw
        pose.pose.orientation.y = roll
        pose.pose.orientation.w = 1 

        pose.header.frame_id = '/odom'        
        pose.header.stamp = rospy.Time.now()

        # count timestamp
        t1 = (timestamp / 100000000)
        t2 = (t1 - int(t1)) * 100000

        time = rospy.Time(secs=int(t2), nsecs = int((t2 - int(t2))*100))


        my_path.poses.append(pose)
        position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        orientation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z]

        #print("oritntation: ", orientation)  

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = time

        odom.pose.pose.position.x = data.translation.x 
        odom.pose.pose.position.z = data.translation.y
        odom.pose.pose.position.y = -data.translation.z

        odom.pose.pose.orientation.x = x_r
        odom.pose.pose.orientation.z = -z_r
        odom.pose.pose.orientation.y = -y_r
        odom.pose.pose.orientation.w = w_r

        odom.pose.covariance = np.diag([1e-1, 1e-1, 1e-1, 1e-2, 1e-2, 1e-2]).ravel()

        #odom.twist.twist.linear.x = data.velocity.x
        #odom.twist.twist.linear.z = data.velocity.y
        #odom.twist.twist.linear.y = -data.velocity.z

        #odom.twist.twist.angular.x = data.acceleration.x
        #odom.twist.twist.angular.y = data.acceleration.y
        #odom.twist.twist.angular.z = data.acceleration.z

        odom.twist.covariance = np.diag([1e-1, 1e-1, 1e-1, 1e-2, 1e-2, 1e-2]).ravel()       

    return my_path, position, orientation, odom


