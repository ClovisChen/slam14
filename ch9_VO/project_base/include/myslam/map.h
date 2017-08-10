//
// Created by chen-tian on 8/6/17.
//

#ifndef PROJECT1_MAP_H
#define PROJECT1_MAP_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr > map_points_; //all landmaps
    unordered_map<unsigned long, Frame::Ptr > keyframes_; //all key_frames

    Map(){}

    void insertKeyFrame( Frame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif //PROJECT1_MAP_H
