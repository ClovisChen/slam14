//
// Created by chen-tian on 8/6/17.
//

#ifndef PROJECT1_MAPPOINT_H
#define PROJECT1_MAPPOINT_H

#include "common_include.h"

namespace myslam{
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    Vector3d pos_;
    Vector3d norm_; //normal of viewing direction
    Mat descriptor_;
    int observed_times_; //being observed by feature matching algo
    int correct_times_; //being an inliner in pose estimate

    MapPoint();
    MapPoint(long id, Vector3d position, Vector3d norm );

    //factory functions
    static MapPoint::Ptr createMapPoint();
};
}

#endif //PROJECT1_MAPPOINT_H
