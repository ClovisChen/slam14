cmake_minimum_required(VERSION 2.8)
project(g2o_custombundle)

set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS "-03")

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)

#! include all g2o libs --> G2O_LIBS
SET(G2O_LIBS
        g2o_cli   g2o_ext_freeglut_minimal  g2o_simulator
        g2o_solver_slam2d_linear  g2o_types_icp  g2o_types_slam2d
        g2o_types_slam3d  g2o_types_sba  g2o_core  g2o_solver_csparse
        g2o_interface  g2o_solver_structure_only  g2o_csparse_extension
        g2o_opengl_helper  g2o_solver_dense  g2o_stuff  g2o_parser
        g2o_solver_pcg  g2o_types_data  g2o_types_sim3 cxsparse)

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${CHOLMOD_INCLUDE_DIR}
                    ${PROJECT_SOURCE_DIR}/ceres)

include_directories(include/common
        include/common/tools
        include/common/flags
        include/common/BALProblem)

#cout
message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})

add_library(BALProblem SHARED include/common/BALProblem/BAlproblem.cpp)
add_library(ParseCmd SHARED include/common/flags/command_args.cpp)

add_executable(${PROJECT_NAME} g2o_BA.cpp)
target_link_libraries(${PROJECT_NAME}
        ${G2O_LIBS} ${CHOLMOD_LIBRARIES}
        BALProblem ParseCmd)