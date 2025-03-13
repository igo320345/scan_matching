#include "scan_matching/icp.hpp"

namespace scan_matching
{
    void laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        float angle = scan->angle_min;
        for (const auto& range : scan->ranges) {
            if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
                pcl::PointXYZ point;
                point.x = range * std::cos(angle);
                point.y = range * std::sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
            angle += scan->angle_increment;
        }
    }

    Eigen::Matrix4f performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        voxel.setInputCloud(source);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);  
        voxel.filter(*source_filtered);
        voxel.setInputCloud(target);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f); 
        voxel.filter(*target_filtered);
        icp.setInputSource(source_filtered);
        icp.setInputTarget(target_filtered);

        icp.setMaxCorrespondenceDistance(20.0); 

        icp.setMaximumIterations(50);  

        icp.setTransformationEpsilon(1e-9); 

        icp.setEuclideanFitnessEpsilon(1e-9); 

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        if (!icp.hasConverged()) {
            return Eigen::Matrix4f::Identity();
        }

        return icp.getFinalTransformation();
    }
}