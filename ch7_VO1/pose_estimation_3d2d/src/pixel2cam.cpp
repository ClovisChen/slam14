//
// Created by chen-tian on 7/24/17.
//

#include "pixel2cam.h"
using namespace cv;

Point2d pixel2cam::trans(const Point2d &p, const Mat &K) {
    return Point2d
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}
