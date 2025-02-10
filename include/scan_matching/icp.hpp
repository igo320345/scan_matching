#ifndef ICP_HPP_
#define ICP_HPP_

#include <vector>
#include <numeric>
#include <cmath>
#include "Eigen/Eigen"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nanoflann.hpp>

using namespace std;
using namespace Eigen;

namespace scan_matching
{
    typedef struct{
        Eigen::Matrix4d trans;
        std::vector<float> distances;
        int iter;
    }  ICP_OUT;

    typedef struct{
        std::vector<float> distances;
        std::vector<int> indices;
    } NEIGHBOR;

    tuple<Eigen::MatrixXd, Eigen::MatrixXd> rangeToPCL(const vector<float>& source, const vector<float>& destination, double min_angle, double max_angle);
    Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    ICP_OUT icp(const sensor_msgs::msg::LaserScan &prev_scan, const sensor_msgs::msg::LaserScan &scan, int max_iterations=20, double tolerance=0.0001);
    NEIGHBOR nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
    NEIGHBOR brute_force_nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
    float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb);
}
#endif