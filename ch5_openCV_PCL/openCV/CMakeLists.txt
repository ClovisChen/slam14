cmake_minimum_required(VERSION 3.7)
project(ch5_openCV_jointMap)

#c++11
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)

#openCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#set(SOURCE_FILES imageBasics.cpp)
add_executable(imageBasics imageBasics.cpp)

#target openCV library
target_link_libraries( imageBasics ${OpenCV_LIBS})