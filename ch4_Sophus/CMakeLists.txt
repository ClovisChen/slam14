cmake_minimum_required(VERSION 3.7)
project(ch4_Sophus)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(SOURCE_FILES useSopgus.cpp)
add_executable(useSophus ${SOURCE_FILES})
target_link_libraries(useSophus ${Sophus_LIBRARIES})
