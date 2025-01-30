#include "point_cloud_colorizer/PointCloudColorizer.h"
#define IMAGE_BOUNDS_OFFSET 50

namespace point_cloud_colorizer {

PointCloudColorizer::PointCloudColorizer(ros::NodeHandle& nh) : nh_(nh)
{
    // Load parameters
    ros::NodeHandle private_nh("~"); 
    private_nh.param<std::string>("input_topic/img_topic", img_topic_, "usb_cam/image_rect_color");
    private_nh.param<std::string>("input_topic/pc_topic", pc_topic_, "velodyne_points");
    private_nh.param<float>("camera/pose/t_x", t_x_, 0.01);
    private_nh.param<float>("camera/pose/t_y", t_y_, 0.01);
    private_nh.param<float>("camera/pose/t_z", t_z_, 0.05); // 0.01 or 0.05 both are good
    private_nh.param<float>("camera/fov/vFOV", vFOV_, 0.558);  // prev_value: 0.516
    private_nh.param<float>("camera/fov/hFOV", hFOV_, 0.744);  // prev_value: 0.734
    private_nh.param<int>("camera/image_bounds/W", W_, 640);
    private_nh.param<int>("camera/image_bounds/H", H_, 480);
    private_nh.param<int>("camera/image_bounds/H", H_, 480);

    // Subscribers
    img_sub_.subscribe(nh_, img_topic_, 1); // input_image
    pc_sub_.subscribe(nh_, pc_topic_, 1);   // input_cloud

    // img_sub_.registerCallback(&PointCloudColorizer::image_cbk, this); // Store images in buffer
    sync_.reset(new Sync(MySyncPolicy(10), img_sub_, pc_sub_));
    sync_->registerCallback(boost::bind(&PointCloudColorizer::sync_cbk, this, _1, _2));

    // Publishers
    color_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("color_cloud", 1);  // output_cloud
}

PointCloudColorizer::~PointCloudColorizer() { }

void PointCloudColorizer::colorize(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, const cv::Mat input_img) 
{
    ros::Time begin_t = ros::Time::now();

    // Convert from ros msg to pcl cloud
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*pc_msg, pl_orig);
    pcl::PointCloud<color_cloud::Point> pl_color;
    pl_color.points.resize(pl_orig.points.size());

    // Iterating
    for (size_t i = 0; i < pl_orig.points.size(); ++i) {
        pl_color.points[i].x = pl_orig.points[i].x;
        pl_color.points[i].y = pl_orig.points[i].y;
        pl_color.points[i].z = pl_orig.points[i].z;
        pl_color.points[i].intensity = pl_orig.points[i].intensity;    // reflection intensity
        pl_color.points[i].ring = pl_orig.points[i].ring;  // ring number (VLP16 has ring number up to 16)
        pl_color.points[i].time = pl_orig.points[i].time;  // time laser beam was shot

        // transform from lidar's frame to camera's frame
        float xC = pl_color.points[i].x - t_x_;
        float yC = pl_color.points[i].y - t_y_;
        float zC = pl_color.points[i].z - t_z_;
        
        // Calculate angles
        float vA = static_cast<float>(atan2(zC, sqrt(xC*xC + yC*yC)));  // vertical angle
        float hA = static_cast<float>(atan2(yC, xC));                   // horizontal angle

        // If point in camera field of view
        if (vA >= -vFOV_/2 && vA <= vFOV_/2 && hA >= -hFOV_/2 && hA <= hFOV_/2 && xC >= 0) {
            int xI = static_cast<int>((W_/2) * (1 - hA/(hFOV_/2)));
            int yI = static_cast<int>((H_/2) * (1 - vA/(vFOV_/2)));
            
            // If pixel in image bounds
            if (yI >=0 + IMAGE_BOUNDS_OFFSET 
                && yI < H_ - IMAGE_BOUNDS_OFFSET 
                && xI >= 0 + IMAGE_BOUNDS_OFFSET
                && xI < W_ - IMAGE_BOUNDS_OFFSET) {
                color_cloud::Point eval_point = pl_color.points[i];
                cv::Vec3b color = input_img.at<cv::Vec3b>(yI, xI);
                pl_color.points[i].r = color[2];
                pl_color.points[i].g = color[1];
                pl_color.points[i].b = color[0];
                pl_color.points[i].a = 255;

            }
        }
        // Out of view processing
        else {
            pl_color.points[i].r = 0;
            pl_color.points[i].g = 0;
            pl_color.points[i].b = 0;
            pl_color.points[i].a = 0;
        }
    }
    // Publish color cloud
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pl_color, ros_cloud);
    ros_cloud.header.frame_id = "velodyne";
    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.is_bigendian = pc_msg->is_bigendian;
    ros_cloud.is_dense = pc_msg->is_dense;
    color_cloud_pub_.publish(ros_cloud);
    
    ros::Time end_t = ros::Time::now();
    ros::Duration process_t = end_t - begin_t;
    ROS_INFO("Coloring time: %u.%09u", process_t.sec, process_t.nsec);
}

void PointCloudColorizer::sync_cbk(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg) 
{
    // Convert ros msg to opencv
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        colorize(pc_msg, cv_ptr->image);
        }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
} // end namespace point_cloud_colorizer