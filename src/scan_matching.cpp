#include "scan_matching/scan_matching.hpp"

namespace scan_matching
{
    ScanMatching::ScanMatching()
    : Node("scan_matching_node"), rate_hz_(30)
    {
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "diff_drive/scan", 10, std::bind(&ScanMatching::laserCallback, this, _1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        string frame_id = "diff_drive/odom";
        string child_frame_id = "diff_drive/base_link";
        odom_ = std::make_shared<nav_msgs::msg::Odometry>();
        odom_->header.frame_id = frame_id;
        odom_->child_frame_id = child_frame_id;
    }
    ScanMatching::~ScanMatching()
    {

    }
    void ScanMatching::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        prev_scan_ = scan_;
        scan_ = msg;

        if (scan_ && prev_scan_) {
            // scan_matching::ICP_OUT icp_out = scan_matching::icp(*prev_scan_, *scan_);
            // TODO: replace with my icp implementation
            auto [A, B] = rangeToPCL(prev_scan_->ranges, scan_->ranges, scan_->angle_min, scan_->angle_max);
            auto source = eigenToPCL(A);
            auto destination = eigenToPCL(B);
            auto icp_trans = icp_pcl(source, destination);
            
            geometry_msgs::msg::TransformStamped transform;
            try {
            transform = tf_buffer_->lookupTransform("diff_drive", "diff_drive/lidar_link", tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform base_link to lidar_link: %s", ex.what());
            }
            matrix_t S = transformToMatrix(transform);
            matrix_t P = poseToMatrix(odom_->pose.pose);
            
            P = (P * S) * icp_trans.inverse() * S.inverse();
            odom_->pose.pose = matrixToPose(P);
            odom_->header.stamp = this->get_clock()->now();

            odom_publisher_->publish(*odom_);

            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = this->get_clock()->now();
            odom_tf.header.frame_id = "diff_drive/odom";
            odom_tf.child_frame_id = "diff_drive";
            odom_tf.transform.translation.x = odom_->pose.pose.position.x;
            odom_tf.transform.translation.y = odom_->pose.pose.position.y;
            odom_tf.transform.translation.z = odom_->pose.pose.position.z;
            odom_tf.transform.rotation = odom_->pose.pose.orientation;

            tf_broadcaster_->sendTransform(odom_tf);
        }
    }
}