#include "scan_matching/scan_matching.hpp"

namespace scan_matching
{
    ScanMatching::ScanMatching()
    : Node("scan_matching_node"), rate_hz_(30)
    {
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ScanMatching::laserCallback, this, _1));
    }
    ScanMatching::~ScanMatching()
    {

    }
    void ScanMatching::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        prev_scan_ = scan_;
        scan_ = msg;
    }
    void ScanMatching::spin()
    {
        rclcpp::Rate rate(rate_hz_);
        while (rclcpp::ok()) {
            if (scan_ && prev_scan_) {
                scan_matching::ICP_OUT icp_out = scan_matching::icp(*prev_scan_, *scan_); 
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }
}