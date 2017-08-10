cmake_minimum_required(VERSION 2.8)
project(ch8_VO2)

set(CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_STANDARD 11)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_NAME}/bin)
set( LIBRARY_OUTPUT_PATH ${PROJECT_NAME}/lib)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
find_package(OpenCV 3 REQUIRED PATHS /home/chen-tian/Desktop/2_Opencv/opencv-master/install/share/OpenCV NO_DEFAULT_PATH)
find_package(CSparse REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/")

add_library(${PROJECT_NAME} SHARED
        src/EdgeSE3ProjectDirect.cpp
        src/projectDD.cpp
        src/pose_estimate.cpp
        )

target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba
        g2o_csparse_extension
        ${CSPARSE_LIBRARY}
)

add_executable(LK useLK.cpp)
add_executable(sparse direct_sparse.cpp )
add_executable(semidense direct_semi_dense.cpp)

target_link_libraries(LK ${OpenCV_LIBS})

target_link_libraries(
        sparse
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba
        g2o_csparse_extension
        ${CSPARSE_LIBRARY}
)

target_link_libraries(
        semidense
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba
        g2o_csparse_extension
        ${CSPARSE_LIBRARY}
)
