//
// Created by chen-tian on 7/28/17.
//

#ifndef CH8_VO2_PROJECT_H
#define CH8_VO2_PROJECT_H

#include <Eigen/Core>

class projectDD {
public:
    Eigen::Vector3d project2Dto3D (int x, int y, int d,
                                          float fx, float fy,
                                          float cx, float cy,
                                          float scale);
    Eigen::Vector2d project3Dto2D (float x, float y, float z,
                                          float fx, float fy,
                                          float cx, float cy);

};


#endif //CH8_VO2_PROJECT_H
