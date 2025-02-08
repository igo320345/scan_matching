#ifndef SCAN_MATCHING_HPP_
#define SCAN_MATCHING_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "scan_matching/icp.hpp"

using std::placeholders::_1;

namespace scan_matching 
{
    class ScanMatching : public rclcpp::Node
    {
        public:
            ScanMatching();
            ~ScanMatching();
            void spin();
        private:
            void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            int rate_hz_;
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            sensor_msgs::msg::LaserScan::SharedPtr prev_scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    };
}
#endif