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
# Parameters setting
This package depends on two main parameters:
1. Pose of camera in refers to lidar frame (no support orientation).
2. Camera field of view (fov)

You can set these parameters in `io/config/point_cloud_colorizer_params.yml`. The file looks like this:
```
input_topic:
  img_topic: usb_cam/image_rect_color
  pc_topic: velodyne_points
camera:
  pose: # position of camera refers to the lidar frame
    t_x: 0.01
    t_y: -0.0001
    t_z: 0.05
  fov:
    vFOV: 0.558
    hFOV: 0.744
  image_bounds:
    W: 640
    H: 480
```
# Launch
```
roslaunch point_cloud_colorizer point_cloud_colorizer.launch
```
# Result
<Put some gif here>
