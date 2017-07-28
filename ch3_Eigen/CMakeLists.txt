cmake_minimum_required(VERSION 2.7)
project(ch3)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES eigenMatrix.cpp)


#find_package( eigen3 )
#include_directories( ${eigen3_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3")
#Eigen just have .h files, don't need target_link_libraries command
add_executable(useEigen eigenMatrix.cpp )
add_executable(useGeometry useGeometry.cpp)