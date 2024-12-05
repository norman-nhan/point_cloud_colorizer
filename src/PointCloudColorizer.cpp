#include "point_cloud_colorizer/PointCloudColorizer.h"

namespace point_cloud_colorizer {

PointCloudColorizer::PointCloudColorizer(ros::NodeHandle& nh) : nh_(nh), it_(nh_)
{
    // Load parameters
    ros::NodeHandle private_nh("~"); 
    private_nh.param<std::string>("img_topic", img_topic_, "/usb_cam/image_raw");
    private_nh.param<std::string>("pc_topic", pc_topic_, "/velodyne_points");
    private_nh.param<std::string>("color_cloud_topic", color_cloud_topic_, "/color_cloud");
    private_nh.param<float>("offset_x", offset_x_, 0.0);
    private_nh.param<float>("offset_y", offset_y_, 0.0);
    private_nh.param<float>("offset_z", offset_z_, 0.0);
    private_nh.param<float>("vFOV", vFOV_, 0.516);
    private_nh.param<float>("hFOV", hFOV_, 0.734);
    private_nh.param<int>("image_width", img_w_, 640);
    private_nh.param<int>("image_height", img_h_, 480);

    // Subscribers
    img_sub_.subscribe(nh_, img_topic_, 1); // input_image
    pc_sub_.subscribe(nh_, pc_topic_, 1);   // input_cloud

    // Synchronize topics using ApproximateTime policy, queue size 10
    sync_.reset(new Sync(MySyncPolicy(10), img_sub_, pc_sub_));
    sync_->registerCallback(boost::bind(&PointCloudColorizer::synchronizer, this, _1, _2));

    // Publishers
    color_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(color_cloud_topic_, 1);  // output_cloud
    // publishes image which contains lidar scans
    img_pub_ = it_.advertise("img_w_lidar_scan", 1);
    
    // cvbridge
    cv_img_.header.frame_id = "camera";
    cv_img_.encoding = "bgr8";
}

PointCloudColorizer::~PointCloudColorizer() // Default Destructor
{

}

// void PointCloudColorizer::img_cbk(const sensor_msgs::Image::ConstPtr& msg) {
//     // Convert ROS image msg to opencv 
//     try {
//         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
//         current_img_ = cv_ptr->image;
//     }
//     catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// void PointCloudColorizer::pc_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//     // Convert Velodyne ROS msg to pcl PointT
//     pcl::PointCloud<velodyne_ros::Point> pl_orig;
//     pcl::fromROSMsg(*msg, pl_orig);
//     pl_color_.points.resize(pl_orig.points.size());
//     // Iterating
//     for (size_t i = 0; i < pl_orig.points.size(); ++i) {
//         // copy data from INPUT CLOUD:"pl_orig" to OUTPUT CLOUD: "pl_color"
//         pl_color_.points[i].x = pl_orig.points[i].x;
//         pl_color_.points[i].y = pl_orig.points[i].y;
//         pl_color_.points[i].z = pl_orig.points[i].z;
//         pl_color_.points[i].intensity = pl_orig.points[i].intensity;    // reflection intensity
//         pl_color_.points[i].ring = pl_orig.points[i].ring;  // ring number (VLP16 has ring number up to 16)
//         pl_color_.points[i].time = pl_orig.points[i].time;  // time laser beam was shot
        
//         /* 
//         converts points from LiDAR coordinates to camera coordinates
//         the offset_x_y_z are the position of camera refers to LiDAR origin
//         */
//         float xC = pl_color_.points[i].x - offset_x_;
//         float yC = pl_color_.points[i].y - offset_y_;
//         float zC = pl_color_.points[i].z - offset_z_;
//         // Calculate angles
//         float vA = static_cast<float>(atan2(zC, sqrt(xC*xC + yC*yC)));  // vertical angle
//         float hA = static_cast<float>(atan2(yC, xC));                   // horizontal angle
//         // If point in camera fov, calculates position of point in image coordinates
//         if (vA >= -vFOV_/2 && vA <= vFOV_/2 && hA >= -hFOV_/2 && hA <= hFOV_/2 && xC >= 0) {
//             int xI = static_cast<int>((img_w_/2) * (1 - hA/(hFOV_/2)));
//             int yI = static_cast<int>((img_h_/2) * (1 - vA/(vFOV_/2)));
//             // if point exists in image bounds
//             if (yI >=0 && yI < img_h_ && xI >= 0 && xI < img_w_) {
//                 if (current_img_.empty()) {
//                     ROS_WARN("Current image is empty; skipping colorization.");
//                     return;
//                 }
//                 // Get color
//                 cv::Vec3b color = current_img_.at<cv::Vec3b>(yI, xI);
//                 pl_color_.points[i].r = color[2];
//                 pl_color_.points[i].g = color[1];
//                 pl_color_.points[i].b = color[0];
//                 pl_color_.points[i].a = 255;

//                 // mark red at these pixel
//                 img_out_ = current_img_.clone();
//                 img_out_.at<cv::Vec3b>(yI, xI) = cv::Vec3b(0, 0, 255); //red, bgr8 encoding;
//             }
//         }
//         // Out of view
//         else {
//             pl_color_.points[i].r = 0;
//             pl_color_.points[i].g = 0;
//             pl_color_.points[i].b = 0;
//             pl_color_.points[i].a = 0;
//         }
//     }
//     // Convert pcl to ros msg
//     pcl::toROSMsg(pl_color_, cloud_out_);
    
//     // Publish color cloud
//     cloud_out_.header.frame_id = "velodyne";
//     cloud_out_.header.stamp = ros::Time::now();
//     cloud_out_.is_bigendian = msg->is_bigendian;
//     cloud_out_.is_dense = msg->is_dense;
//     color_cloud_pub_.publish(cloud_out_);
    
//     // publish img
//     cv_img_.header.stamp = ros::Time::now();
//     // cv_img_.image = current_img_;
//     cv_img_.image = img_out_;
//     img_pub_.publish(cv_img_.toImageMsg());
// }

void PointCloudColorizer::colorize(const sensor_msgs::PointCloud2::ConstPtr& msg, const cv::Mat input_img) {
    // Convert Velodyne ROS msg to pcl PointT
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    pl_color_.points.resize(pl_orig.points.size());

    // copy image to img_out
    img_out_ = input_img.clone();

    // Iterating
    for (size_t i = 0; i < pl_orig.points.size(); ++i) {
        // copy data from INPUT CLOUD:"pl_orig" to OUTPUT CLOUD: "pl_color"
        pl_color_.points[i].x = pl_orig.points[i].x;
        pl_color_.points[i].y = pl_orig.points[i].y;
        pl_color_.points[i].z = pl_orig.points[i].z;
        pl_color_.points[i].intensity = pl_orig.points[i].intensity;    // reflection intensity
        pl_color_.points[i].ring = pl_orig.points[i].ring;  // ring number (VLP16 has ring number up to 16)
        pl_color_.points[i].time = pl_orig.points[i].time;  // time laser beam was shot
        
        /* 
        converts points from LiDAR coordinates to camera coordinates
        the offset_x_y_z are the position of camera refers to LiDAR origin
        */
        float xC = pl_color_.points[i].x - offset_x_;
        float yC = pl_color_.points[i].y - offset_y_;
        float zC = pl_color_.points[i].z - offset_z_;
        // Calculate angles
        float vA = static_cast<float>(atan2(zC, sqrt(xC*xC + yC*yC)));  // vertical angle
        float hA = static_cast<float>(atan2(yC, xC));                   // horizontal angle
        // If point in camera fov, calculates position of point in image coordinates
        if (vA >= -vFOV_/2 && vA <= vFOV_/2 && hA >= -hFOV_/2 && hA <= hFOV_/2 && xC >= 0) {
            int xI = static_cast<int>((img_w_/2) * (1 - hA/(hFOV_/2)));
            int yI = static_cast<int>((img_h_/2) * (1 - vA/(vFOV_/2)));
            // if point exists in image bounds
            if (yI >=0 && yI < img_h_ && xI >= 0 && xI < img_w_) {
                if (input_img.empty()) {
                    ROS_WARN("input image is empty; skipping colorization.");
                    return;
                }
                // Get color
                cv::Vec3b color = input_img.at<cv::Vec3b>(yI, xI);
                pl_color_.points[i].r = color[2];
                pl_color_.points[i].g = color[1];
                pl_color_.points[i].b = color[0];
                pl_color_.points[i].a = 255;

                // mark red at these pixel
                img_out_.at<cv::Vec3b>(yI, xI) = cv::Vec3b(0, 0, 255); //red, bgr8 encoding;
            }
        }
        // Out of view
        else {
            pl_color_.points[i].r = 0;
            pl_color_.points[i].g = 0;
            pl_color_.points[i].b = 0;
            pl_color_.points[i].a = 0;
        }
    }
    // Convert pcl to ros msg
    pcl::toROSMsg(pl_color_, cloud_out_);
    
    // Publish color cloud
    cloud_out_.header.frame_id = "velodyne";
    cloud_out_.header.stamp = ros::Time::now();
    cloud_out_.is_bigendian = msg->is_bigendian;
    cloud_out_.is_dense = msg->is_dense;
    color_cloud_pub_.publish(cloud_out_);
    
    // publish img
    cv_img_.header.stamp = ros::Time::now();
    cv_img_.image = img_out_;
    // cv_img_.image = input_img;
    img_pub_.publish(cv_img_.toImageMsg());
}

void PointCloudColorizer::synchronizer(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& pc_msg) {
    // img_cbk(img_msg);
    // pc_cbk(pc_msg);
    // Convert ROS image msg to opencv 
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        colorize(pc_msg, cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

} // end namespace point_cloud_colorizer