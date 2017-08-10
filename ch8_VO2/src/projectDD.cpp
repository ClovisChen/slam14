//
// Created by chen-tian on 7/28/17.
//

#include "projectDD.h"

Eigen::Vector3d projectDD::project2Dto3D ( int x, int y, int d,
                                       float fx, float fy,
                                       float cx, float cy,
                                       float scale )
{
    float zz = float ( d ) /scale;
    float xx = zz* (x-cx)/fx;
    float yy = zz* (y-cy)/fy;
    return Eigen::Vector3d ( xx, yy, zz );
}

Eigen::Vector2d projectDD::project3Dto2D ( float x, float y, float z,
                                       float fx, float fy,
                                       float cx, float cy)
{
    float u = fx*x/z + cx;
    float v = fy*y/z + cy;
    return Eigen::Vector2d ( u,v );
}
