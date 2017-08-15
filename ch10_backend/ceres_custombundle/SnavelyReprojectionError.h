#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include "common_include.h"
#include "tools/rotation.h"
#include "projection.h"

class SnavelyReprojectionError
{
public:
    SnavelyReprojectionError(double observation_x, double observation_y):observed_x(observation_x),observed_y(observation_y){}

template<typename T>
    //重载符号()  返回值为bool
    //如果类重载了函数调用运算符()，可以像使用函数一样使用该类的对象
    bool operator()(const T* const camera,
                const T* const point,
                T* residuals)const{                  
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);

        return true;
    }

    static ceres::CostFunction* Create(const double observed_x, const double observed_y){
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError,2,9,3>(
            new SnavelyReprojectionError(observed_x,observed_y)));
    }


private:
    double observed_x;
    double observed_y;
};

#endif // SnavelyReprojection.h

