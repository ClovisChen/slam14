//
// Created by chen-tian on 7/24/17.
//

#include "BA.h"
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>


using namespace std;
using namespace cv;

void BA::bundleadjustment
        (const vector<Point3f> points_3d,
         const vector<Point2f> points_2d,
         const Mat& K,
         const Mat& R,
         const Mat& t){
    //initial g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(2,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
            R_mat,
            Eigen::Vector3d( t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))
    ) );
    optimizer.addVertex( pose );

    int index = 1;
    for ( const Point3f p:points_3d )//landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId( index++ );
        point->setEstimate( Eigen::Vector3d(p.x, p.y, p.z ) );
        point->setMarginalized( true );
        optimizer.addVertex( point );
    }

    //K
    g2o::CameraParameters* camera = new g2o::CameraParameters(
            K.at<double>(0,0), Eigen::Vector2d(K.at<double>(0,2), K.at<double>(1,2)),0 );
    camera->setId(0);
    optimizer.addParameter( camera );

    //edges
    index = 1;
    for ( const Point2f p:points_2d)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)) );
        edge->setMeasurement( Eigen::Vector2d( p.x, p.y ) );
        edge->setParameterId(0,0);
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;
};
