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
    using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

    typedef struct{
        matrix_t trans;
        std::vector<float> distances;
        int iter;
    }  ICP_OUT;

    typedef struct{
        std::vector<float> distances;
        std::vector<int> indices;
    } NEIGHBOR;

    tuple<matrix_t, matrix_t> rangeToPCL(const vector<float>& source, const vector<float>& destination, double min_angle, double max_angle);
    matrix_t best_fit_transform(const matrix_t &A, const matrix_t &B);
    ICP_OUT icp(const sensor_msgs::msg::LaserScan &prev_scan, const sensor_msgs::msg::LaserScan &scan, int max_iterations=20, double tolerance=0.0001);
    NEIGHBOR nearest_neighbor(const matrix_t &src, const matrix_t &dst);
}
#endif