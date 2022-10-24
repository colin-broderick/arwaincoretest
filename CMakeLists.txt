cmake_minimum_required(VERSION 3.10.2)

project(arwain)

## Enable ctest unit testing framework
include(CTest)
enable_testing()

## Set include and library source directories
include_directories(
    /usr/local/include
    lora
    orientation
    src
    iim42652
    orientation_fusion
    quaternion
    vector3
    logger
    kalman
    input_parser
    timers
    lis3mdl
    minitrace
    ips
    bmp384
    uubla/include
    # realsense_t265
)

# TODO : Additional includes for tensorflow. These need to be tidied up.
include_directories(
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/session
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/providers/vsi_npu
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite-vx-delegate/usr/include/tensorflow-lite-vx-delegate
    /opt/yocto/imx-yocto-bsp/build/tmp/work/armv8a-poky-linux/flatbuffers/1.12.0-r0/image/usr/include/
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite/usr/lib
)

option(NUMCPP_NO_USE_BOOST "Build NumCpp without Boost as a dependency" OFF)
if(NUMCPP_NO_USE_BOOST)
    add_compile_definitions(NUMCPP_NO_USE_BOOST)
    # TODO This should do find_package(boost) plus includes and links in the else() block.
endif()

option(USEROS "Use ROS topics to publish diagnostic data" OFF)
if(USEROS)
    message("Using ROS for diagnostic data topics")
    add_definitions(-DUSEROS)
    find_package(roscpp REQUIRED)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
endif(USEROS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)

## Locate source files
set(PROJECT_CPP_FILES
    src/main.cpp
    src/calibration.cpp
    src/stance.cpp
    src/velocity_prediction.cpp
    src/altimeter.cpp
    src/imu_reader.cpp
    src/uwb_reader.cpp
    src/std_output.cpp
    src/transmit_lora.cpp
    src/multi_imu.cpp
    src/arwain.cpp
    src/arwain_tests.cpp
    src/command_line.cpp
    src/configuration.cpp
)

add_subdirectory(iim42652)
add_subdirectory(orientation)
add_subdirectory(lora)
add_subdirectory(quaternion)
add_subdirectory(logger)
add_subdirectory(input_parser)
add_subdirectory(kalman)
add_subdirectory(timers)
add_subdirectory(lis3mdl)
add_subdirectory(minitrace)
add_subdirectory(ips)
add_subdirectory(bmp384)
add_subdirectory(uubla)
# add_subdirectory(realsense_t265)

## Executables to build
add_executable(${PROJECT_NAME} ${PROJECT_CPP_FILES})
add_executable(${PROJECT_NAME}_cmdr src/lora_commander.cpp)
add_executable(${PROJECT_NAME}_test src/tests.cpp)

## Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -g -Wno-psabi -Wall -Wshadow)
target_compile_options(${PROJECT_NAME}_cmdr PRIVATE -g -Wno-psabi -Wall -Wshadow)
target_link_libraries(${PROJECT_NAME}_test -lgcov --coverage)
target_compile_options(${PROJECT_NAME}_test PRIVATE -O0 -g -ftest-coverage -fprofile-arcs)

## Link libraries
target_link_libraries(${PROJECT_NAME} boost_system madgwick_filter efaroe_filter -pthread dl stdc++fs i2c zmq iim42652_driver quaternion logger input_parser kalman timers lis3mdl minitrace ips bmp384 lora fusion_filter uublal)
# FOr tensorflow
target_link_libraries(
    ${PROJECT_NAME}
    tensorflow-lite
    vx_delegate
    flatbuffers
)
target_link_libraries(${PROJECT_NAME}_cmdr input_parser lora)

add_test(Test_Test ${PROJECT_NAME}_test Test_Test)

## Install configuration and related files to correct locations.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/build/arwain DESTINATION /usr/bin)
