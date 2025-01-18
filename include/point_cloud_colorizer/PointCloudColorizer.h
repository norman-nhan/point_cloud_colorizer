#pragma once
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// Eigen
#include <Eigen/Core>
// C++ system
#include <stdio.h>
#include <cmath>
// custom pcl point type
#include "color_cloud.h"

typedef color_cloud::Point PointType;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

////////////////////////////
// Declare new pcl PointT //
////////////////////////////

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point { 
    PCL_ADD_POINT4D;
    union 
    {
        struct 
        {
            float intensity;
            std::uint16_t ring;
            float time;
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
};
}// namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)         
    (float, y, y)         
    (float, z, z)         
    (float, intensity, intensity)  
    (std::uint16_t, ring, ring)             
    (float, time, time)            
)

////////////////////////////////////////
// Declare point cloud colorizer node //
////////////////////////////////////////

namespace point_cloud_colorizer {
class PointCloudColorizer
{
private:
    // ROS stuffs
    ros::NodeHandle& nh_;

    ros::Publisher color_cloud_pub_;

    message_filters::Subscriber<sensor_msgs::Image> img_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;

    boost::shared_ptr<Sync> sync_;

    // YAML file params
    std::string image_topic_;
    std::string pc_topic_;
    std::string color_cloud_topic_;

    float offset_x_;    
    float offset_y_;
    float offset_z_;
    
    float hFOV_; 
    float vFOV_;
    
    int H_;
    int W_;

public:
    PointCloudColorizer(ros::NodeHandle& nh);
    ~PointCloudColorizer();

private:
    void image_lidar_cbk(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
    void colorize(const sensor_msgs::PointCloud2::ConstPtr& msg, const cv::Mat input_img);
};
} // end namespace point_cloud_colorizer