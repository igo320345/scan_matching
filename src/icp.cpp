#include "scan_matching/icp.hpp"
#include <iostream>
namespace scan_matching
{
    tuple<Eigen::MatrixXd, Eigen::MatrixXd> rangeToPCL(const vector<float>& source, const vector<float>& destination, double min_angle, double max_angle) {
        double beamAngleIncrement = (max_angle - min_angle) / 360;
        double beamAngle = min_angle;
        
        vector<Vector3d> pointsSource, pointsDestination;
        
        for (size_t i = 0; i < source.size(); ++i) {
            double lengthSource = source[i];
            double lengthDestination = destination[i];
            
            if (lengthSource > 0 && lengthSource < INFINITY && lengthDestination > 0 && lengthDestination < INFINITY) {
                pointsSource.emplace_back(lengthSource * cos(beamAngle), lengthSource * sin(beamAngle), 0);
                pointsDestination.emplace_back(lengthDestination * cos(beamAngle), lengthDestination * sin(beamAngle), 0);
            }
            
            beamAngle += beamAngleIncrement;
        }
        
        Eigen::MatrixXd pclSource(pointsSource.size(), 3);
        Eigen::MatrixXd pclDestination(pointsDestination.size(), 3);
        
        for (size_t i = 0; i < pointsSource.size(); ++i) {
            pclSource.row(i) = pointsSource[i];
            pclDestination.row(i) = pointsDestination[i];
        }
        
        return make_tuple(pclSource.eval(), pclDestination.eval());
    }

    Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B){
        Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4);
        Eigen::Vector3d centroid_A(0,0,0);
        Eigen::Vector3d centroid_B(0,0,0);
        Eigen::MatrixXd AA = A;
        Eigen::MatrixXd BB = B;
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

        Eigen::MatrixXd H = AA.transpose()*BB;
        Eigen::MatrixXd U;
        Eigen::VectorXd S;
        Eigen::MatrixXd V;
        Eigen::MatrixXd Vt;
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        JacobiSVD<Eigen::MatrixXd> svd(H, ComputeFullU | ComputeFullV);
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
        Eigen::MatrixXd src = Eigen::MatrixXd::Ones(4, row);
        Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(4, row);
        Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(4, row);
        Eigen::Matrix4d T;
        NEIGHBOR neighbor;
        ICP_OUT result;
        int iter = 0;

        src.topRows(3) = A.transpose();
        dst.topRows(3) = B.transpose();

        double prev_error = 0;
        double mean_error = 0;

        for (int i = 0; i < max_iterations; i++){
            neighbor = nearest_neighbor(src.topRows(3).transpose(), dst.topRows(3).transpose());
            NEIGHBOR neighbor_bf = brute_force_nearest_neighbor(src.topRows(3).transpose(), dst.topRows(3).transpose());

            // DEBUG 
            // std::cout << "Source point" << std::endl;
            // for (int i = 0; i < 4; i++) std::cout << src(i, 0) << " ";
            // std::cout << std::endl;
            // std::cout << "nanoflann dst point" << std::endl;
            // for (int i = 0; i < 4; i++) std::cout << dst(i, neighbor.indices[0])<< " ";
            // std::cout << std::endl;
            // std::cout << "brute force dst point" << std::endl;
            // for (int i = 0; i < 4; i++) std::cout << dst(i, neighbor_bf.indices[0])<< " ";
            // std::cout << std::endl;
            // std::cout << "----" << std::endl;
            // DEBUG

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

    NEIGHBOR nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst){
        using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>;
        int row_src = src.rows();
        NEIGHBOR neigh;
        neigh.indices.resize(row_src);
        neigh.distances.resize(row_src);
        KDTree index(dst.cols(), std::cref(dst), 5);
        index.index->buildIndex();
        
        for (int i = 0; i < row_src; i++) {
            size_t nearestIndex;
            double outDistSqr;
            nanoflann::KNNResultSet<double> resultSet(1);
            auto vec_src = src.block<1,3>(i,0).transpose();
            resultSet.init(&nearestIndex, &outDistSqr);
            index.index->findNeighbors(resultSet, vec_src.data(), nanoflann::SearchParams());
            neigh.indices[i] = nearestIndex;
            neigh.distances[i] = sqrt(outDistSqr);
        }
        return neigh;
    }

    NEIGHBOR brute_force_nearest_neighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst){
        int row_src = src.rows();
        int row_dst = dst.rows();
        Eigen::Vector3d vec_src;
        Eigen::Vector3d vec_dst;
        NEIGHBOR neigh;
        float min = 100;
        int index = 0;
        float dist_temp = 0;

        for(int ii=0; ii < row_src; ii++){
            vec_src = src.block<1,3>(ii,0).transpose();
            min = 100;
            index = 0;
            dist_temp = 0;
            for(int jj=0; jj < row_dst; jj++){
                vec_dst = dst.block<1,3>(jj,0).transpose();
                dist_temp = dist(vec_src,vec_dst);
                if (dist_temp < min){
                    min = dist_temp;
                    index = jj;
                }
            }
            neigh.distances.push_back(min);
            neigh.indices.push_back(index);
        }
        return neigh;
    }

    float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb){
        return sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]) + (pta[2]-ptb[2])*(pta[2]-ptb[2]));
    }
}