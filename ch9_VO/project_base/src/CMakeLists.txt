add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        camera.cpp
        map.cpp
        config.cpp
        )

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS}
        )