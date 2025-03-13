#include "scan_matching/scan_matching.hpp"

namespace scan_matching {
    ScanMatching::ScanMatching() : Node("scan_matching_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "diff_drive/scan", 10, std::bind(&ScanMatching::scan_callback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Scan Matching Node Started");
    }

    ScanMatching::~ScanMatching() {

    }
    void ScanMatching::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        laserScanToPointCloud(scan, curr_cloud);

        if (prev_cloud_->empty()) {
            *prev_cloud_ = *curr_cloud;
            return;
        }

        Eigen::Matrix4f transform = performICP(prev_cloud_, curr_cloud);

        updateOdometry(transform);

        *prev_cloud_ = *curr_cloud;
    }

    void ScanMatching::updateOdometry(const Eigen::Matrix4f &T) {
        static Eigen::Matrix4f P = Eigen::Matrix4f::Identity();

        P = P * T.inverse();

        double x = P(0, 3);
        double y = P(1, 3);

        double theta = std::atan2(P(1, 0), P(0, 0));

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "diff_drive/odom";
        odom_msg.child_frame_id = "diff_drive";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation.z = std::sin(theta / 2);
        odom_msg.pose.pose.orientation.w = std::cos(theta / 2);

        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "diff_drive/odom";
        tf_msg.child_frame_id = "diff_drive";
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.rotation.z = std::sin(theta / 2);
        tf_msg.transform.rotation.w = std::cos(theta / 2);

        tf_broadcaster_->sendTransform(tf_msg);
    }
}