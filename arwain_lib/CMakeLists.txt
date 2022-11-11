cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

add_definitions(-DUSE_NCS2)
set(CMAKE_CXX_STANDARD 17)

add_library(arwain_lib 
    calibration.cpp
    stance.cpp
    velocity_prediction.cpp
    altimeter.cpp
    imu_reader.cpp
    uwb_reader.cpp
    std_output.cpp
    transmit_lora.cpp
    multi_imu.cpp
    arwain.cpp
    arwain_tests.cpp
    command_line.cpp
    configuration.cpp
)
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
)
