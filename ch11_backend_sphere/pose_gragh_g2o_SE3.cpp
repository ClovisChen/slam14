//
// Created by chen-tian on 8/26/17.
//
#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
using namespace std;

/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 这里使用g2o/types/slam3d/中的SE3表示位姿，它实质上是四元数而非李代数.
 * *********************************************
 * 在/cmake-build-debug/文件夹中, 打开结果存储结果的文件
 * $ g2o_viewer result.g2o
 * 代码中的优化求解过程和点击图中$ g2o_viewer sphere.g2o的optimize按钮没有区别
 * */

int main(int argc, char** argv){
    if(argc != 2 ){
        cout<<"Usage: pose_gragh_g2o_SE3 sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin( argv[1] );
    if( !fin ){
        cout<<"file"<<argv[1]<<" does not exist. "<<endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;
    ///线性方程求解器
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    ///矩阵块求解器
    Block* solver_ptr = new Block( linearSolver );
    ///梯度下降方法， 从GN, LM, Dogleg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    ///图模型
    g2o::SparseOptimizer optimizer;
    ///设置求解器
    optimizer.setAlgorithm( solver );
    ///顶点和边的数量
    int vertexCnt = 0, edgeCnt = 0;

    while ( !fin.eof() ){
        string name;
        fin>>name;
        if( name == "VERTEX_SE3:QUAT" ){
            ///SE3 顶点
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin>>index;
            v->setId( index );
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if( index==0 )
                v -> setFixed(true);
        }
        else if ( name == "EDGE_SE3:QUAT" ){
            ///SE3-SE3 边
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            ///关联的两个顶点
            int idx1, idx2;
            fin>>idx1>>idx2;
            e -> setId( edgeCnt++ );
            e -> setVertex( 0, optimizer.vertices()[idx1] );
            e -> setVertex( 1, optimizer.vertices()[idx2] );
            e -> read(fin);
            optimizer.addEdge(e);
        }
        if ( !fin.good() ) break;
    }

    cout<<"read total "<<vertexCnt <<"vertices, "<<edgeCnt<<" edges."<<endl;
    cout<<"prepare optimizing ..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizating ..."<<endl;
    optimizer.optimize(30);

    ///结果存储至...
    cout<<"saving optimization results ..."<<endl;
    /// cd cmake-build-debug g2o_viewer result.g2o
    optimizer.save("result.g2o");

    return 0;
}


