//
// Created by chen-tian on 7/28/17.
//

#include "pose_estimate.h"
#include "EdgeSE3ProjectDirect.h"

bool pose_estimate::poseEstimationDirect ( const vector<Measurement>& measurements,
                            Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
    //g2o init
    typedef BlockSolver<BlockSolverTraits<6,1>>DirectBlock;//求解的向量是6*1
    DirectBlock::LinearSolverType* linearSolver = new LinearSolverDense<DirectBlock::PoseMatrixType >();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );

    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
    SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate( SE3Quat( Tcw.rotation(), Tcw.translation() ) );
    pose->setId( 0 );
    optimizer.addVertex(pose);

    //add edge
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
                m.pos_world,
                K(0,0), K(1,1), K(0,2), K(1,2), gray );
        edge->setVertex(0,pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation( Eigen::Matrix<double, 1, 1>::Identity() );
        edge->setId(id++);
        optimizer.addEdge( edge );
    }
    cout<<"edges in gragh: "<<optimizer.edges().size()<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize( 30 );
    Tcw = pose->estimate();
}