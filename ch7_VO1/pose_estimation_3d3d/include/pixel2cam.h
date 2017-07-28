//
// Created by chen-tian on 7/24/17.
//

#ifndef POSE_ESTIMATION_3D3D_PIXEL2CAM_H
#define POSE_ESTIMATION_3D3D_PIXEL2CAM_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;


class pixel2cam
{
public:
    Point2d trans(
            const Point2d& p,
            const Mat& K
    );
};

#endif //POSE_ESTIMATION_3D3D_PIXEL2CAM_H
