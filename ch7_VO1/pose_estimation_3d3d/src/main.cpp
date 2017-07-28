#include <iostream>

#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "BA.h"
#include "Edge.h"
#include "match.h"
#include "pixel2cam.h"
#include "pose_estimate_3d3d.h"

using namespace std;
using namespace cv;


int main(int argc, char**argv)
{
    if(argc !=5)
    {
        cout<<"usage: pose_estimation_3d3d img1 img2 depth1 depth2! "<<endl;
        return  1;
    }

    Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;

    BA AAA;
    EdgeProjectXYZRGBD BBB;
    match CCC;
    pixel2cam DDD;
    pose_estimate_3d3d FFF;


    CCC.find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    cout<<"totally found: "<<matches.size()<<" pairs. "<<endl;

    //build 3D points
    Mat depth1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat depth2 = imread ( argv[4], CV_LOAD_IMAGE_UNCHANGED); //unsigned single channel

    Mat K = ( Mat_<double> (3,3) << 520.9, 0, 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1, pts2;

    for ( DMatch m:matches ) //DMatch:size matches=m
    {
        ushort d1 = depth1.ptr <unsigned short> ( int (keypoints_1[m.queryIdx].pt.y ) ) [int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth1.ptr <unsigned short> ( int (keypoints_2[m.trainIdx].pt.y ) ) [int(keypoints_2[m.queryIdx].pt.x)];
        if (d1==0||d2==0)
            continue;
        Point2d p1 = DDD.trans ( keypoints_1[m.queryIdx].pt, K );
        Point2d p2 = DDD.trans ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /1000.0;
        float dd2 = float ( d2 ) /1000.0;
        pts1.push_back( Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back( Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }
    cout<<"3d-3d pairs: "<<pts1.size()<<endl;

    Mat R, t;
    FFF.pose_estimation_3d3d(pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t()<<endl;
    cout<<"t_inv = "<<-R.t()<<endl;

    cout<<"calling bundle adjustment"<<endl;

//    AAA.bundleadjustment(pts1, pts2, R, t,);
}