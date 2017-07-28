cmake_minimum_required(VERSION 3.7)
project(ch5_openCV_jointMap)

#c++11
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)

#openCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories("/usr/include/eigen3/")

#PCL
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES jointMap.cpp)
add_executable(jointMap ${SOURCE_FILES})

#target openCV library
target_link_libraries( jointMap ${OpenCV_LIBS} ${PCL_LIBRARIES})