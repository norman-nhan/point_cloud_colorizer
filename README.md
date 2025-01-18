# point_cloud_colorizer
A pkg to colorize point cloud data with usb camera

# System Overview
1. Subscribe to both image topic and lidar topic
2. Convert ros image msg to opencv image
3. Colorize point cloud data with the given image
   
   3.1 Transform points in LiDAR frame to camera frame
   
   3.2 Calculate angle of view in horizontal and vertical
   
   3.3 Calculate correspondent pixel in horizontal and vertical
   
   3.4 Apply color to point cloud data
   
   3.5 Publish colorized cloud
