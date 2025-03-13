#ifndef ICP_HPP_
#define ICP_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace Eigen;

namespace scan_matching
{
    void laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    Eigen::Matrix4f performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target);
}
#endif