#define PCL_NO_PRECOMPILE
#ifndef COLOR_CLOUD_H
#define COLOR_CLOUD_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace color_cloud {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D; // x,y,z
    PCL_ADD_RGB;      // r,g,b,a
    float intensity;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }; // end struct point

  struct EIGEN_ALIGN16 PointNormal {
    PCL_ADD_POINT4D; // x,y,z
    PCL_ADD_NORMAL4D; // normal_x, normal_y, normal_z
    PCL_ADD_RGB;      // r,g,b,a
    float intensity;
    float curvature;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }; // end struct PointNormal
}// namespace color_cloud

POINT_CLOUD_REGISTER_POINT_STRUCT(color_cloud::Point,
  (float, x, x)
  (float, y, y)     
  (float, z, z)   
  (float, rgba, rgba)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(color_cloud::PointNormal,
  (float, x, x)
  (float, y, y)     
  (float, z, z)
  (float, normal_x, normal_x)
  (float, normal_y, normal_y)
  (float, normal_z, normal_z)     
  (float, intensity, intensity)
  (float, curvature, curvature)
  (float, rgba, rgba)
)

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}// end namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

#endif // COLOR_CLOUD_H