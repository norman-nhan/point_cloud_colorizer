#include <ros/ros.h>
#include "point_cloud_colorizer/PointCloudColorizer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_colorizer_node");
    ros::NodeHandle nh;

    point_cloud_colorizer::PointCloudColorizer rosPointCloudColorizer(nh);

    ros::spin();

    return 0;
}