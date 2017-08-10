//
// Created by chen-tian on 8/7/17.
//

#ifndef PROJECT_R2C_VISUAL_ODOMETRY_H
#define PROJECT_R2C_VISUAL_ODOMETRY_H

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState  //! 不限定作用域的枚举类型
    {
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState state_;
    Map::Ptr map_;
    Frame::Ptr ref_;
    Frame::Ptr curr_;

    cv::Ptr<cv::ORB> orb_;
    vector<cv::Point3f> pts_3d_ref_;
    vector<cv::KeyPoint> keypoints_curr_;
    Mat descriptors_curr_;
    Mat descriptors_ref_;
    vector<cv::DMatch> feature_matches_;

    SE3 T_c_r_estimated_;
    int num_inliers_;
    int num_lost_;

    //parameters
    int num_of_features_;
    double scale_factor_; //scale in image pyramid
    int level_pyramid_;  //number of pyramid levels
    float match_ratio_;  //ratio of selecting good matches
    int max_num_lost_;  //max number of continuous lost times
    int min_inliers_;  //minimum inliers

    double key_frame_min_rot;
    double key_frame_min_trans;

    //functions
    VisualOdometry();
    ~VisualOdometry(){};

    bool addFrame( Frame::Ptr frame );  //add a new frame

protected:
    //inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();
};
}

#endif //PROJECT_R2C_VISUAL_ODOMETRY_H
