cmake_minimum_required(VERSION 2.8)
project()
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

#Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# target_link_libraries('xx' ${Sophus_LIBRARIES})

#cv2
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries( ‘XX’ ${OpenCV_LIBS})

#cv3
find_package(OpenCV 3 REQUIRED PATHS /home/chen-tian/Desktop/2_Opencv/opencv-master/install/share/OpenCV NO_DEFAULT_PATH)
INCLUDE_DIRECTORIES(${OpenCV_DIRS})
#target_link_libraries( ‘XX’ ${OpenCV_LIBS})

#PCL
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# target_link_libraries('xx' ${PCL_LIBRARIES})

# Ceres
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
# target_link_libraries( 'xx' ${CERES_LIBRARIES} )

#Ceres_include
include_directories(${PROJECT_SOURCE_DIR}/include/ceres)

#g2o
FIND_PACKAGE(G2O REQUIRED)
SET(G2O_LIBS
        g2o_cli   g2o_ext_freeglut_minimal  g2o_simulator
        g2o_solver_slam2d_linear  g2o_types_icp  g2o_types_slam2d
        g2o_types_slam3d  g2o_types_sba  g2o_core  g2o_solver_csparse
        g2o_interface  g2o_solver_structure_only  g2o_csparse_extension
        g2o_opengl_helper  g2o_solver_dense  g2o_stuff  g2o_parser
        g2o_solver_pcg  g2o_types_data  g2o_types_sim3 cxsparse)
include_directories( ${G2O_INCLUDE_DIRS})
#target_link_libraries( ‘XX’ ${G2O_LIBS})

#CSparse
find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})
#target_link_libraries('xx'${CSPARSE_LIBRARY})

#Cholmod
FIND_PACKAGE(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
# target_link_libraries('xx' ${CHOLMOD_LIBRARIES})

################################## libraries ###################################

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/myslam)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/include/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/include/common/flags/command_args.cpp)

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES} )

################################## source and test #############################
add_subdirectory( src )
add_subdirectory( test )
################################ cout ##########################################
message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})
