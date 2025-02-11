#include "scan_matching/icp.hpp"
#include <iostream>
namespace scan_matching
{
    std::tuple<matrix_t, matrix_t> rangeToPCL(const std::vector<float>& source, const std::vector<float>& destination, double min_angle, double max_angle) {
        float beamAngleIncrement = (max_angle - min_angle) / 360;
        float beamAngle = min_angle;
        
        matrix_t pclSource(source.size(), 3);
        matrix_t pclDestination(destination.size(), 3);
        
        for (size_t i = 0; i < source.size(); ++i) {
            float lengthSource = source[i];
            float lengthDestination = destination[i];
            
            if (lengthSource > 0 && lengthSource < INFINITY && lengthDestination > 0 && lengthDestination < INFINITY) {
                pclSource(i, 0) = lengthSource * cos(beamAngle);
                pclSource(i, 1) = lengthSource * sin(beamAngle);
                pclSource(i, 2) = 0;

                pclDestination(i, 0) = lengthDestination * cos(beamAngle);
                pclDestination(i, 1) = lengthDestination * sin(beamAngle);
                pclDestination(i, 2) = 0;
            }        
            beamAngle += beamAngleIncrement;
        }
        
        return std::make_tuple(pclSource, pclDestination);
    }

    matrix_t best_fit_transform(const matrix_t &A, const matrix_t &B){
        matrix_t T = matrix_t::Identity(4,4);
        Eigen::Vector3f centroid_A(0,0,0);
        Eigen::Vector3f centroid_B(0,0,0);
        matrix_t AA = A;
        matrix_t BB = B;
        int row = A.rows();

        for(int i=0; i<row; i++){
            centroid_A += A.block<1,3>(i,0).transpose();
            centroid_B += B.block<1,3>(i,0).transpose();
        }
        centroid_A /= row;
        centroid_B /= row;
        for(int i=0; i<row; i++){
            AA.block<1,3>(i,0) = A.block<1,3>(i,0) - centroid_A.transpose();
            BB.block<1,3>(i,0) = B.block<1,3>(i,0) - centroid_B.transpose();
        }

        matrix_t H = AA.transpose()*BB;
        matrix_t U;
        matrix_t S;
        matrix_t V;
        matrix_t Vt;
        matrix_t R;
        matrix_t t;

        Eigen::JacobiSVD<matrix_t> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        S = svd.singularValues();
        V = svd.matrixV();
        Vt = V.transpose();

        R = Vt.transpose()*U.transpose();

        if (R.determinant() < 0 ){
            Vt.block<1,3>(2,0) *= -1;
            R = Vt.transpose()*U.transpose();
        }

        t = centroid_B - R*centroid_A;

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        return T;
    }

    ICP_OUT icp(const sensor_msgs::msg::LaserScan &prev_scan, const sensor_msgs::msg::LaserScan &scan, int max_iterations, double tolerance){
        auto [A, B] = rangeToPCL(prev_scan.ranges, scan.ranges, scan.angle_min, scan.angle_max);
        
        int row = A.rows();
        matrix_t src = matrix_t::Ones(4, row);
        matrix_t dst = matrix_t::Ones(4, row);
        matrix_t dst_chorder = matrix_t::Ones(4, row);
        matrix_t T;
        NEIGHBOR neighbor;
        ICP_OUT result;
        int iter = 0;

        src.topRows(3) = A.transpose();
        dst.topRows(3) = B.transpose();

        double prev_error = 0;
        double mean_error = 0;

        for (int i = 0; i < max_iterations; i++){
            neighbor = nearest_neighbor(src.topRows(3).transpose(), dst.topRows(3).transpose());

            for(int j=0; j<row; j++){ 
                dst_chorder.block<4,1>(0,j) = dst.block<4, 1>(0, neighbor.indices[j]);
            }
            T = best_fit_transform(src.topRows(3).transpose(),dst_chorder.topRows(3).transpose());
            src = T*src;
            mean_error = std::accumulate(neighbor.distances.begin(),neighbor.distances.end(),0.0)/neighbor.distances.size();
            if (abs(prev_error - mean_error) < tolerance){
                break;
            }
            prev_error = mean_error;
            iter = i+2;
        }

        T = best_fit_transform(A, src.topRows(3).transpose());
        result.trans = T;
        result.distances = neighbor.distances;
        result.iter = iter;

        return result;
    }

    NEIGHBOR nearest_neighbor(const matrix_t &src, const matrix_t &dst){
        using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;
        
        int row_src = src.rows();
        NEIGHBOR neigh;
        neigh.indices.resize(row_src);
        neigh.distances.resize(row_src);
        KDTree index(3, std::cref(dst), 10);
        index.index->buildIndex();

        size_t nearestIndex;
        float outDistSqr;
        nanoflann::KNNResultSet<float> resultSet(1);
        std::vector<float> vec_src(3);

        for (int i = 0; i < row_src; i++) {
            for (int j = 0; j < 3; j++) vec_src[j] = src(i, j);
            resultSet.init(&nearestIndex, &outDistSqr);
            index.index->findNeighbors(resultSet, &vec_src[0], nanoflann::SearchParams());
            neigh.indices[i] = nearestIndex;
            neigh.distances[i] = sqrt(outDistSqr);
        }
        return neigh;
    }
}