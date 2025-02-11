#include "scan_matching/utils.hpp"

namespace scan_matching
{
    matrix_t transformToMatrix(const geometry_msgs::msg::TransformStamped &transform) {
        matrix_t matrix = matrix_t::Identity(4, 4);

        matrix(0, 3) = transform.transform.translation.x;
        matrix(1, 3) = transform.transform.translation.y;
        matrix(2, 3) = transform.transform.translation.z;

        tf2::Quaternion q(transform.transform.rotation.x, 
                        transform.transform.rotation.y, 
                        transform.transform.rotation.z, 
                        transform.transform.rotation.w);
        tf2::Matrix3x3 rotation_matrix(q);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix(i, j) = rotation_matrix[i][j];
            }
        }

        return matrix;
    }

    matrix_t poseToMatrix(const geometry_msgs::msg::Pose& pose) {
        matrix_t P = matrix_t::Identity(4, 4);
        
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 rotationMatrix(q);

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                P(i, j) = rotationMatrix[i][j];

        P(0, 3) = pose.position.x;
        P(1, 3) = pose.position.y;
        P(2, 3) = pose.position.z;

        return P;
    }

    geometry_msgs::msg::Pose matrixToPose(const matrix_t& P) {
        geometry_msgs::msg::Pose pose;

        pose.position.x = P(0, 3);
        pose.position.y = P(1, 3);
        pose.position.z = P(2, 3);

        tf2::Matrix3x3 rotationMatrix(
            P(0, 0), P(0, 1), P(0, 2),
            P(1, 0), P(1, 1), P(1, 2),
            P(2, 0), P(2, 1), P(2, 2)
        );

        tf2::Quaternion q;
        rotationMatrix.getRotation(q);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }
} 
