add_executable(cere ceresBundle.cpp)
target_link_libraries(cere
        BALProblem
        ParseCmd
        ${CERES_LIBRARIES} )