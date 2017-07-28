cmake_minimum_required(VERSION 3.7)
project(ch6_ceres_g2o)

set(CMAKE_CXX_STANDARD 11)

#openCV
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Ceres
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

#set(SOURCE_FILES ceres_curve_fitting.cpp g2o_curve_fitting.cpp)
add_executable(ceres_curve_fitting  ceres_curve_fitting.cpp )
target_link_libraries( ceres_curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )