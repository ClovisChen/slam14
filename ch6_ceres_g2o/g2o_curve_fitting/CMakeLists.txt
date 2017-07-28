cmake_minimum_required(VERSION 2.8)
project(g2o_curvefitting)

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 11)

#openCV
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_DIRS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} "/usr/include/eigen3" )

add_executable(g2o_curve_fitting g2o_curve_fitting.cpp)
target_link_libraries( g2o_curve_fitting ${OpenCV_LIBS} g2o_core g2o_stuff )