//
// Created by chen-tian on 7/24/17.
//

#include "pose_estimate_3d3d.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

void pose_estimate_3d3d::pose_estimation_3d3d
        (const vector<Point3f> &pts1,
         const vector<Point3f> &pts2,
         Mat &R, Mat &t)
{
    Point3f p1, p2;  //center of mass
    int N = pts1.size();
    for ( int i = 0; i < N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }

    p1 /= N;
    p2 /= N;
    vector<Point3f> q1(N), q2(N);//remove the center
    for ( int i = 0; i < N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    //compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i = 0; i<N; i++ )
    {
        W += Eigen::Vector3d( q1[i].x, q1[i].y, q1[i].z )* Eigen::Vector3d( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    //SVD ON W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U*(V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d( p1.x, p1.y, p1.z ) - R_* Eigen::Vector3d( p2.x, p2.y, p2.z );

    //convert to cv::Mat
    R = (Mat_<double>(3,3)<<
                          R_(0,0), R_(0,1),R_(0,2),
            R_(1,0),R_(1,1),R_(1,2),
            R_(2,0),R_(2,1),R_(2,2) );
    t = ( Mat_<double>(3,1)<< t_(0,0),t_(1,0),t_(2,0));
}