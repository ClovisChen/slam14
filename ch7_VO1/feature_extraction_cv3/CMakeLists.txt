cmake_minimum_required(VERSION 3.8)
project(feature_extraction_cv3)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV 3 REQUIRED PATHS /home/chen-tian/Desktop/2_Opencv/opencv-master/install/share/OpenCV NO_DEFAULT_PATH)

set(SOURCE_FILES feature_extraction_cv3.cpp)
add_executable(feature_extraction_cv3 ${SOURCE_FILES})
target_link_libraries(feature_extraction_cv3 ${OpenCV_LIBS} )