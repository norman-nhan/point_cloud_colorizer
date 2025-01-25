# point_cloud_colorizer
A package to colorize point cloud data by fusing camera and LiDAR data.
# System Overview
1. Subscribe to both image topic and lidar topic
2. Convert ros image msg to opencv image
3. Colorize point cloud data with the given image

    3.1. Transform points in LiDAR frame to camera frame

    3.2 Calculate angle of view in horizontal and vertical

    3.3 Calculate correspondent pixel in horizontal and vertical

    3.4 Apply color to point cloud data

    3.5 Publish colorized cloud
# Prerequisites
## ROS1 Packages
This package will process point cloud data and image data. So it requires `usb_cam`, `image_proc` and `velodyne` package in order to work.
```
sudo apt install ros-<version>-usb-cam
sudo apt install ros-<version>-image-proc
sudo apt install ros-<version>-velodyne
```
# Build
## ROS1
```
cd catkin_ws/src
git clone https://github.com/norman-nhan/point_cloud_colorizer.git
cd ..
rosdep install -iry --from-paths src --ignore-src
catkin build
source devel/setup.bash
```
# Launch
```
roslaunch point_cloud_colorizer point_cloud_colorizer.launch
```
# Result
<Put some gif here>
