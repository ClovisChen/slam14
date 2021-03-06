//
// Created by chen-tian on 8/6/17.
//

#ifndef PROJECT_MAP_FRAME_H
#define PROJECT_MAP_FRAME_H

#include "common_include.h"
#include "camera.h"

namespace myslam {
    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_;
        double time_stamp_; //when it is recorded
        SE3 T_c_w_;
        Camera::Ptr camera_; //Pinhole RGBD Camera model
        Mat color_, depth_; //color and depth image

    public: //data members
        Frame();

        Frame(long id,
              double time_stamp = 0,
              SE3 T_c_w = SE3(),
              Camera::Ptr camera = nullptr,
              Mat color = Mat(),
              Mat depth = Mat());

        ~Frame();

        //factory function
        static Frame::Ptr createFrame();

        //find the depth
        double findDepth(const cv::KeyPoint &kp);

        //Camera center
        Vector3d getCamCenter() const;

        //check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);
    };
}

#endif //PROJECT1_FRAME_H
