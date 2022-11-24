project(ips)

add_library(${PROJECT_NAME} src/indoor_positioning_wrapper.cpp src/corner_detector.cpp src/floor_tracker.cpp)
