//
// Created by chen-tian on 7/28/17.
//

#ifndef CH8_VO2_POSE_ESTIMATE_H
#define CH8_VO2_POSE_ESTIMATE_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;
using namespace cv;

class pose_estimate {
public:
    struct Measurement
    {
        Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ){}
        Eigen::Vector3d pos_world;
        float grayscale;
    };
    bool poseEstimationDirect( const vector<Measurement>& measurements,
                               Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw );

};


#endif //CH8_VO2_POSE_ESTIMATE_H
