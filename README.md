# realsense_nodes_python

## Installation
- librealsense https://github.com/IntelRealSense/librealsense
- pip install pyrealsense2 and pip3 install pyrealsense2

## Nodes:
### realsense_rgb_image
Node to stream rgb image from Realsense D435 or D435i. 
Published topics:
- /rgb_image
- /camera_info
### realsense_rgb_align_depth
Node to stream rgb image and depth image alined to color from Realsense D435.
Published topics:
- /rgb_image
- /align_depth
- /camera_info
### realsense_point_cloud
Node to stream point cloud from Realsense D435. 
Published topics:
- /point_cloud2
### realsense_trajectory
Node to stream trajectory from Realsense T265. 
Published topics:
- /path
### realsense_point_cloud_trajectory
Node to stream point cloud from Realsense D435i and trajectory from Realsense T265. 
Published topics:
- /point_cloud2
- /path
### usb_camera
Node to stream rgb image from usb camera (also Realsense D435). 
Published topics:
- /usb_camera_image


