# point_cloud_colorizer
A package to colorize point cloud data by fusing camera and LiDAR data.
# Prerequisites
## ROS Packages
This package will process point cloud data and image data. So it requires `usb_cam`, `image_proc` and `velodyne` package in order to work.
You can install these packages via binaries files, by using the following cli:
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
