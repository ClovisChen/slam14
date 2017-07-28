cmake_minimum_required(VERSION 3.7)
project(src)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES useHello.cpp libHelloSLAM.cpp libHelloSLAM.h)
#add_library(hello libHelloSLAM.cpp) #static
add_library(hello_shared SHARED libHelloSLAM.cpp)

add_executable(useHello useHello.cpp)

add_executable(a helloSLAM.cpp)
target_link_libraries(useHello hello_shared)