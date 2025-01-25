#include "point_cloud_colorizer_ros2/PointCloudColorizer.h"

PointCloudColorizer::PointCloudColorizer()
: Node("point_cloud_colorizer_node")
{
    // Declare parameters
    this->declare_parameter<std::string>("img_topic", "/usb_cam/image_raw");
    this->declare_parameter<std::string>("pc_topic", "/velodyne_points");
    this->declare_parameter<float>("t_x", 0.01);
    this->declare_parameter<float>("t_y", -0.0001);
    this->declare_parameter<float>("t_z", 0.05);
    this->declare_parameter<float>("vFOV", 0.558);
    this->declare_parameter<float>("hFOV", 0.744);
    this->declare_parameter<int>("W", 640);
    this->declare_parameter<int>("H", 480);

    // Get parameters using get_parameter("param_name", variable)
    this->get_parameter("img_topic", img_topic_);
    this->get_parameter("pc_topic", pc_topic_);
    this->get_parameter("t_x", t_x_);
    this->get_parameter("t_y", t_y_);
    this->get_parameter("t_z", t_z_);
    this->get_parameter("vFOV", vFOV_);
    this->get_parameter("hFOV", hFOV_);
    this->get_parameter("W", W_);
    this->get_parameter("H", H_);

    // Subscribers
    img_sub_.subscribe(this, img_topic_);
    pc_sub_.subscribe(this, pc_topic_);

    // Synchronizers
    sync_ = std::make_shared<Sync>(SyncPolicy(10), img_sub_, pc_sub_);
    sync_->registerCallback(std::bind(&PointCloudColorizer::image_lidar_cbk, this, std::placeholders::_1, std::placeholders::_2));
    // Publisher
    color_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("color_cloud", 10);
}

PointCloudColorizer::~PointCloudColorizer() {}

void PointCloudColorizer::image_lidar_cbk(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        colorize(pc_msg, cv_ptr->image);
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void PointCloudColorizer::colorize(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg, const cv::Mat img)
{
    // Convert ros point cloud to pcl cloud
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*pc_msg, pl_orig);
    pcl::PointCloud<PointType> pl_color;
    pl_color.points.resize(pl_orig.points.size());

    for (size_t i = 0; i < pl_orig.points.size(); ++i) {
        pl_color.points[i].x = pl_orig.points[i].x;
        pl_color.points[i].y = pl_orig.points[i].y;
        pl_color.points[i].z = pl_orig.points[i].z;
        pl_color.points[i].intensity = pl_orig.points[i].intensity;
        pl_color.points[i].ring = pl_orig.points[i].ring;
        pl_color.points[i].time = pl_orig.points[i].time;

        float xC = pl_color.points[i].x - t_x_;
        float yC = pl_color.points[i].y - t_y_;
        float zC = pl_color.points[i].z - t_z_;

        float vA = static_cast<float>(atan2(zC, sqrt(xC * xC + yC * yC)));
        float hA = static_cast<float>(atan2(yC, xC));

        if (vA >= -vFOV_ / 2 && vA <= vFOV_ / 2 && hA >= -hFOV_ / 2 && hA <= hFOV_ / 2 && xC >= 0) {
            int xI = static_cast<int>((W_ / 2) * (1 - hA / (hFOV_ / 2)));
            int yI = static_cast<int>((H_ / 2) * (1 - vA / (vFOV_ / 2)));

            if (yI >= 0 && yI < H_ && xI >= 0 && xI < W_) {
                cv::Vec3b color = img.at<cv::Vec3b>(yI, xI);
                pl_color.points[i].r = color[2];
                pl_color.points[i].g = color[1];
                pl_color.points[i].b = color[0];
                pl_color.points[i].a = 255;
            }
        } else {
            pl_color.points[i].r = 0;
            pl_color.points[i].g = 0;
            pl_color.points[i].b = 0;
            pl_color.points[i].a = 0;
        }
    }

    sensor_msgs::msg::PointCloud2 color_cloud;
    pcl::toROSMsg(pl_color, color_cloud);

    color_cloud.header.frame_id = "velodyne";
    color_cloud.header.stamp = pc_msg->header.stamp;
    color_cloud.is_bigendian = pc_msg->is_bigendian;
    color_cloud.is_dense = pc_msg->is_dense;
    color_cloud_pub_->publish(color_cloud);
}