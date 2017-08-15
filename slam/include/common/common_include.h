//
// Created by chen-tian on 8/6/17.
//

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

//std
#include <algorithm>
#include <boost/format.hpp>  //字符串格式化
#include <boost/timer.hpp>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <math.h>
#include <memory>
#include <set>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>
using Eigen::Vector2d;
using Eigen::Vector3d;

//Sophus
//#include <sophus/se3.h>
//#include "sophus/so3.h"
//using Sophus::SE3;
//using Sophus::SO3;

//cv
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/video/tracking.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

//pcl -->point cloud library
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>

//ceres
//#include <ceres.h>
#include "autodiff.h"
//g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include "g2o/core/batch_stats.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>





#endif //COMMON_INCLUDE_H
