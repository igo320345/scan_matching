#ifndef SCAN_MATCHING_HPP_
#define SCAN_MATCHING_HPP_

#include "scan_matching/icp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace scan_matching 
{
    class ScanMatching : public rclcpp::Node
    {
        public:
            ScanMatching();
            ~ScanMatching();
        private:
            void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
            void updateOdometry(const Eigen::Matrix4f &T);
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ { new pcl::PointCloud<pcl::PointXYZ> };
    };
}
#endif