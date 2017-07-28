//
// Created by chen-tian on 7/25/17.
//

#ifndef POSE_ESTIMATION_3D3D_EDGE_H
#define POSE_ESTIMATION_3D3D_EDGE_H


#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <iostream>

using namespace std;

class EdgeProjectXYZRGBD : public  g2o::BaseBinaryEdge<3,
        Eigen::Vector3d,
        g2o::VertexSBAPointXYZ,
        g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBD(){};

    virtual void computeError();

    virtual void linearizeOplus();

    bool read ( istream& in ) ;
    bool write( ostream& out )const ;
};

#endif //POSE_ESTIMATION_3D3D_EDGE_H
