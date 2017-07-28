//
// Created by chen-tian on 7/24/17.
//

#include "BA.h"
#include "Edge.h"
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>


using namespace std;
using namespace cv;

void BA::bundleadjustment
        (const vector<Point3f> pts1,
         const vector<Point3f> pts2,
         Mat& R,
         Mat& t)
{
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
    for ( const Point3f p_prime:pts2 )//landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId( index++ );
        point->setEstimate( Eigen::Vector3d(p_prime.x, p_prime.y, p_prime.z ) );
        point->setMarginalized( true );
        optimizer.addVertex( point );
    }

    //edges
    index = 1;
    vector<EdgeProjectXYZRGBD*> edges;
    for ( const Point3f p:pts1)
    {
        EdgeProjectXYZRGBD* edge = new EdgeProjectXYZRGBD();
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)) );
        edge->setVertex( 1, pose );
        edge->setMeasurement( Eigen::Vector3d( p.x, p.y, p.z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);

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

    index = 0;
    for ( EdgeProjectXYZRGBD* edge: edges )
    {
        g2o::VertexSBAPointXYZ* point = dynamic_cast<g2o::VertexSBAPointXYZ*>(edge->vertices()[0] );
        cout<<"p2 = "<<pts2[index++]<<endl;
        cout<<"p2 adjusted = "<<point->estimate().transpose()<<endl;
        cout<<endl;
    }
};
