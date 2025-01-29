#pragma once
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// custom pcl point type
#include "color_cloud.h"
// Eigen
#include <Eigen/Core>
// C++ system
#include <stdio.h>
#include <cmath>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "point_cloud_colorizer/CameraParamSliderConfig.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

namespace point_cloud_colorizer {
class PointCloudColorizer
{
private:
    // ROS
    ros::NodeHandle& nh_;
    ros::Publisher color_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::Image> img_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
    boost::shared_ptr<Sync> sync_;
    image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_;

    // Parameters
    std::string img_topic_;
    std::string pc_topic_;
    float t_x_, t_y_, t_z_;
    float hFOV_, vFOV_; 
    int H_, W_;

    // cv bridge
    cv_bridge::CvImage cv_img_;

    // Dynamic recongigure
    dynamic_reconfigure::Server<CameraParamSliderConfig> dr_server_;
    dynamic_reconfigure::Server<CameraParamSliderConfig>::CallbackType dr_callback_;

public:
    PointCloudColorizer(ros::NodeHandle& nh);
    ~PointCloudColorizer();

private:
    void sync_cbk(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
    void colorize(const sensor_msgs::PointCloud2::ConstPtr& msg, const cv::Mat input_img);
    void dr_cbk(CameraParamSliderConfig &config, uint32_t level);
};
} // end namespace point_cloud_colorizer