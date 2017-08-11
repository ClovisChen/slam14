//
// Created by chen-tian on 8/6/17.
//

#ifndef PROJECT_MAP_MAPPOINT_H
#define PROJECT_MAP_MAPPOINT_H

#include "common_include.h"
#include "frame.h"

namespace myslam{
    class MapPoint
    {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long           id_;
        static unsigned long    factory_id_;
        bool                    good_; //whether a good point
        Vector3d                pos_;
        Vector3d                norm_; //normal of viewing direction
        Mat                     descriptor_;

        list<Frame*>   observed_frames_;  //key frames that can observe this point

        int            matched_times_; //being an inlier in pose estimation
        int            visible_times_; //being visible in current frame

        MapPoint();
        MapPoint(
                unsigned long id,
                const Vector3d& positon,
                const Vector3d& norm,
                Frame* frame = nullptr,
                const Mat& descriptor = Mat() );

        inline cv::Point3f getPositionCV() const{
            return cv::Point3f(pos_(0,0), pos_(1,0), pos_(2,0) );
        }

        //factory functions
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint(
                const Vector3d& pos_world,
                const Vector3d& norm,
                const Mat& descriptor,
                Frame* frame );
    };
}

#endif //PROJECT_R2C_MAPPOINT_H
