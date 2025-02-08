#include "rclcpp/rclcpp.hpp"
#include "scan_matching/scan_matching.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<scan_matching::ScanMatching>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
