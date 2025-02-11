#ifndef SCAN_MATCHING_HPP_
#define SCAN_MATCHING_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "scan_matching/icp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "scan_matching/utils.hpp"

using std::placeholders::_1;

namespace scan_matching 
{
    class ScanMatching : public rclcpp::Node
    {
        public:
            ScanMatching();
            ~ScanMatching();
        private:
            void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            int rate_hz_;
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            sensor_msgs::msg::LaserScan::SharedPtr prev_scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            nav_msgs::msg::Odometry::SharedPtr odom_;
    };
}
#endif