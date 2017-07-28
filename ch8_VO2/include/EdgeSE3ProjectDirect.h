//
// Created by chen-tian on 7/26/17.
//

#ifndef CH8_VO2_EDGESE3PROJECTDIRECT_H
#define CH8_VO2_EDGESE3PROJECTDIRECT_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace cv;

//project a 3d point into an image plane, the error is photometric error
//an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ProjectDirect(){};
    EdgeSE3ProjectDirect( Eigen::Vector3d point, float fx, float fy, float cx, float cy, Mat* image );

    virtual void computeError();

    //plus in manifold
    virtual void linearizeOplus();

    //dummy read and write functions because we don't care...
    virtual bool read (std::istream& in){};
    virtual bool write (std::ostream& out )const {};

protected:
    //get a gray scale value from reference image (bilinear interpolated)
    inline float getPixelValue ( float x, float y );

public:
    Eigen::Vector3d x_world_;
    float cx_=0, cy_=0, fx_=0, fy_=0;//Camera intrinsics
    Mat* image_= nullptr;//reference image
};


#endif //CH8_VO2_EDGESE3PROJECTDIRECT_H
