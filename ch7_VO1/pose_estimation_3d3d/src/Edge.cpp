//
// Created by chen-tian on 7/25/17.
//

#include "Edge.h"

void EdgeProjectXYZRGBD::computeError()
{
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> (_vertices[1]);
    //measurement is p, point is p'
    _error = _measurement - pose -> estimate().map( point->estimate() );
}

void EdgeProjectXYZRGBD::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(pose->estimate());
    g2o::VertexSBAPointXYZ* point = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi = -T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) = 0;
    _jacobianOplusXj(0,1) = -z;
    _jacobianOplusXj(0,2) = y;
    _jacobianOplusXj(0,3) = -1;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = z;


    _jacobianOplusXj(1,0) = z;
    _jacobianOplusXj(1,1) = 0;
    _jacobianOplusXj(1,2) = -x;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1;
    _jacobianOplusXj(1,5) = 0;

    _jacobianOplusXj(2,0) = -y;
    _jacobianOplusXj(2,1) = x;
    _jacobianOplusXj(2,2) = 0;
    _jacobianOplusXj(2,3) = 0;
    _jacobianOplusXj(2,4) = 0;
    _jacobianOplusXj(2,5) = -1;
}

bool EdgeProjectXYZRGBD::read(istream &in) {}
bool EdgeProjectXYZRGBD::write(ostream &out) const {}