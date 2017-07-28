//
// Created by chen-tian on 7/24/17.
//

#ifndef POSE_ESTIMATION_3D3D_POSE_ESTIMATE_3D3D_H
#define POSE_ESTIMATION_3D3D_POSE_ESTIMATE_3D3D_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>


using namespace std;
using namespace cv;

class pose_estimate_3d3d
{
public:
    void pose_estimation_3d3d(
            const vector<Point3f>& pts1,
            const vector<Point3f>& pts2,
            Mat& R, Mat& t
    );
};

#endif //POSE_ESTIMATION_3D3D_POSE_ESTIMATE_3D3D_H
