//
// Created by chen-tian on 8/9/17.
//

#ifndef PROJECT_MAP_G2O_TYPES_H
#define PROJECT_MAP_G2O_TYPES_H

#include "common_include.h"
#include "camera.h"

namespace myslam
{

    //! VertexSE3Expmap: 李代数位姿
    //! VertexSBAPointXYZ: 空间点位姿
    //! EdgeProjectXYZ2UV: 投影方程边
    class EdgeProjectXYZRGBD: public g2o::BaseBinaryEdge<3, Eigen::Vector3d,
            g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void computeError();
        virtual void linearizeOplus();
        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}
    };

    //only to optimize the pose, no point
    class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Error: measure = R*point+t
        virtual void computeError();
        virtual void linearizeOplus();

        virtual bool read( std::istream& in ){}
        virtual bool write( std::ostream& out) const {}

        Vector3d point_;
    };

    class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError();
        virtual void linearizeOplus();

        virtual bool read( std::istream& in ){}
        virtual bool write(std::ostream& os) const {};

        Vector3d point_;
        Camera* camera_;
    };
}

#endif //PROJECT_R2C_G2O_TYPES_H
