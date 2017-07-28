cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_2d2d )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )


find_package( OpenCV 3 REQUIRED PATHS ${/home/chen-tian/Desktop/2_Opencv/opencv-master/install/share/OpenCV} NO_DEFAULT_PATH )


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries( pose_estimation_2d2d ${OpenCV_LIBS} )

