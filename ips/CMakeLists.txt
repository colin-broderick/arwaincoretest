project(ips)

add_library(${PROJECT_NAME} indoor_positioning_wrapper.cpp corner_detector.cpp floor_tracker.cpp)
