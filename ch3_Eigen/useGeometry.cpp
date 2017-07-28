//
// Created by chen-tian on 17-7-4.
//
#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector (M_PI/4, Eigen::Vector3d(0,0,1));//沿z轴旋转45度
    cout .precision(3);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix()<<endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated = rotation_vector*v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    v_rotated=rotation_matrix*v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    //euler Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();//although it is called 3d, this matrix is defeined by 4*4
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1,3,4));
    cout<<"Transform matrix = \n"<<T.matrix()<<endl;

    Eigen::Vector3d v_transformed = T*v;
    cout<<"v transformed = \n"<<v_transformed.transpose()<<endl;

    //quaterniond
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector);
    cout<<"quaternion = \n"<<q.coeffs()<<endl;
    q=Eigen::Quaterniond (rotation_matrix);
    cout<<"quaternion = \n"<<q.coeffs()<<endl;
    v_rotated = q*v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    return 0;
};