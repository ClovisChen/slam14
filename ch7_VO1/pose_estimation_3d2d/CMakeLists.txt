cmake_minimum_required(VERSION 3.8)
project(pose_estimation_3d2d)

set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_STANDARD 11)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package(OpenCV 3 REQUIRED PATHS /home/chen-tian/Desktop/2_Opencv/opencv-master/install/share/OpenCV NO_DEFAULT_PATH)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
#find_package( Eigen)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/")

add_library(${PROJECT_NAME} SHARED
        src/BA.cpp
        src/match.cpp
        src/pixel2cam.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )

add_executable(3d2d src/pose_estimation_3d2d.cpp)

target_link_libraries(3d2d
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )
