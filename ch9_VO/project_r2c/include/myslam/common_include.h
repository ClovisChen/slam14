//
// Created by chen-tian on 8/6/17.
//

#ifndef PROJECT_R2C_COMMON_INCLUDE_H
#define PROJECT_R2C_COMMON_INCLUDE_H

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//Sophus
#include <sophus/se3.h>
using Sophus::SE3;
using Sophus::SO3;

//cv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using cv::Mat;

//std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
using namespace std;

#endif //PROJECT1_COMMON_INCLUDE_H
