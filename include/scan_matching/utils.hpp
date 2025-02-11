#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"

namespace scan_matching
{
    using matrix_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

    matrix_t transformToMatrix(const geometry_msgs::msg::TransformStamped &transform);
    matrix_t poseToMatrix(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::Pose matrixToPose(const matrix_t& P);    
}                   
#endif