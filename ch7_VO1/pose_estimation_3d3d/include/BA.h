//
// Created by chen-tian on 7/24/17.
//

#ifndef POSE_ESTIMATION_3D3D_BA_H
#define POSE_ESTIMATION_3D3D_BA_H


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class BA
{
public:
    void bundleadjustment(
            const vector<Point3f> pts1,
            const vector<Point3f> pts2,
            Mat& R,
            Mat& t);
};


#endif //POSE_ESTIMATION_3D3D_BA_H
