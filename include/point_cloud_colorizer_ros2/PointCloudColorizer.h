#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "color_cloud.h"

using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;
typedef color_cloud::Point PointType;

class PointCloudColorizer: public rclcpp::Node
{
private:
    // Synchronizers
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    std::shared_ptr<Sync> sync_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_cloud_pub_;

    // Parameters
    std::string img_topic_;
    std::string pc_topic_;
    double t_x_;
    double t_y_;
    double t_z_;
    double hFOV_;
    double vFOV_;
    int H_;
    int W_;

    // OpenCV
    // cv_bridge::CvImagePtr cv_ptr_;

    // Callbacks
    void image_lidar_cbk(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg);
    void colorize(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg, const cv::Mat img);

public:
    PointCloudColorizer();
    ~PointCloudColorizer();
};
