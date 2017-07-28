//
// Created by chen-tian on 7/24/17.
//

#ifndef POSE_ESTIMATION_3D2D_MATCH_H
#define POSE_ESTIMATION_3D2D_MATCH_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

class match
{
public:
    void find_feature_matches (
            const Mat& img_1, const Mat& img_2,
            std::vector<KeyPoint>& keypoints_1,
            std::vector<KeyPoint>& keypoints_2,
            std::vector< DMatch >& matches
    );
};


#endif //POSE_ESTIMATION_3D2D_MATCH_H
