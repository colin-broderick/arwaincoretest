cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

add_definitions(-DUSE_NCS2)
set(CMAKE_CXX_STANDARD 17)

set(SRC_FILES
    calibration.cpp
    stance.cpp
    velocity_prediction.cpp
    altimeter.cpp
    imu_reader.cpp
    std_output.cpp
    transmit_lora.cpp
    multi_imu.cpp
    arwain.cpp
    arwain_tests.cpp
    command_line.cpp
    configuration.cpp
    arwain_utils.cpp
    activity_metric.cpp
)
if(USE_UUBLA)
set(SRC_FILES ${SRC_FILES} uwb_reader.cpp)
endif()
if(USE_REALSENSE)
set(SRC_FILES ${SRC_FILES} realsense.cpp)
endif()

add_library(arwain_lib ${SRC_FILES})

target_link_libraries(arwain_lib stdc++fs)
include_directories(
    ../vector3
    ../quaternion
    ../logger
    ../lora
    ../uubla/include
    ../input_parser
    ../arwain_thread
    ../bmp384
    ../iim42652
    ../orientation
    ../timers
    ../lis3mdl
    ../ips
    ../realsense_t265
)
