#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include "pixel2cam.h"
#include "match.h"
#include "BA.h"

using namespace std;
using namespace cv;


int main( int argc, char** argv ) {
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d2d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1,keypoints_2;
    vector<DMatch> matches;


    match ooo;

    ooo.find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"totally find "<<matches.size()<<"pairs"<<endl;

    //build 3D point
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat K = (Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f>pts_3d;
    vector<Point2f>pts_2d;
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int(keypoints_1[m.queryIdx].pt.y) )[int(keypoints_1[m.queryIdx].pt.x) ];
        if (d == 0) //bad depth
            continue;
        float dd = d/1000.0;
        pixel2cam mpixel2cam;

        Point2d p1 = mpixel2cam.trans( keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back( Point3f(p1.x*dd, p1.y*dd, dd) );
        pts_2d.push_back( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size()<<endl;

    Mat r, t;
    //PnP: EPnP DLS
    solvePnP( pts_3d, pts_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP );

    Mat R;
    Rodrigues(r, R);//罗德里格斯
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    BA QQQ ;
    QQQ.bundleadjustment(pts_3d, pts_2d, K, R, t);

}