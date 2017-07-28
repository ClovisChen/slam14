//
// Created by chen-tian on 7/24/17.
//

#ifndef POSE_ESTIMATION_3D2D_BA_H
#define POSE_ESTIMATION_3D2D_BA_H


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class BA
{
public:
    void bundleadjustment(
            const vector<Point3f> points_3d,
            const vector<Point2f> points_2d,
            const Mat& K,
            const Mat& R,
            const Mat& t);
};


#endif //POSE_ESTIMATION_3D2D_BA_H
