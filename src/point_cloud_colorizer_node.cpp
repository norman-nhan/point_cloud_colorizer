#include "rclcpp/rclcpp.hpp"
#include "point_cloud_colorizer_ros2/PointCloudColorizer.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudColorizer>());
    rclcpp::shutdown();
    return 0;
}
