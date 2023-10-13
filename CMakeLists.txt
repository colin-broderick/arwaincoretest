cmake_minimum_required(VERSION 3.10.2)

project(arwain)
set(CMAKE_BUILD_TYPE Debug)
find_package(GTest CONFIG REQUIRED)

option(NUMCPP_NO_USE_BOOST  "Build NumCpp without Boost as a dependency"          ON)
option(USE_ROS              "Use ROS topics to publish diagnostic data"           OFF)
option(USE_UUBLA            "Include the UUBLA ultra-wideband library"            OFF)
option(USE_REALSENSE        "Use the Realsense T265 camera for positioning"       OFF)
option(UNIT_TESTS           "Make private class methods public to enable testing" OFF)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
link_directories(/usr/local64/lib)

add_definitions(-DNUMCPP_NO_USE_BOOST)

include_directories(/usr/local64/include)

if(UNIT_TESTS)
    message("Building with all public class methods for test")
    add_definitions(-DUNIT_TESTS)
    add_definitions(-DTESTABLE=public)
    add_definitions(-DI2CDEVICEDRIVER=MockI2CDevice)
    add_definitions(-DSPIDEVICEDRIVER=MockSpiDevice)
    else()
    add_definitions(-DTESTABLE=private)
    add_definitions(-DI2CDEVICEDRIVER=LinuxSmbusI2CDevice)
    add_definitions(-DSPIDEVICEDRIVER=LinuxSpiDevice)
endif()

if(USE_REALSENSE)
    message("Using librealsense for T265 camera")
    find_package(realsense2 REQUIRED)
    add_definitions(-DUSE_REALSENSE)
endif()

# TODO : Additional includes for tensorflow. These need to be tidied up.
include_directories(
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/session
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/providers/vsi_npu
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite-vx-delegate/usr/include/tensorflow-lite-vx-delegate
    /opt/yocto/imx-yocto-bsp/build/tmp/work/armv8a-poky-linux/flatbuffers/1.12.0-r0/image/usr/include/
    /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite/usr/lib
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(ALL_ARWAIN_LIBRARIES arwain_lib madgwick_filter efaroe_filter arwain_logger input_parser kalman timers lora fusion_filter arwain_thread arwain_math)
file(GLOB UNIT_TEST_SOURCES CONFIGURE_DEPENDS "unit_tests/*.cpp")

include_directories(
    lora/include
    arwain_thread/include
    orientation/include
    orientation_fusion/include
    kalman/include
    timers/include
    arwain_lib/include
    /opt/arwain/workspace/tensorflow_src
    realsense_t265/include
)

if(USE_UUBLA)
    message("Compiling with UUBLA support")
    set(ALL_ARWAIN_LIBRARIES ${ALL_ARWAIN_LIBRARIES} uubla)
    add_definitions(-DUSE_UUBLA)
endif()

add_subdirectory(orientation)
add_subdirectory(lora)
add_subdirectory(kalman)
add_subdirectory(timers)
add_subdirectory(arwain_thread)
add_subdirectory(arwain_lib)
if(USE_REALSENSE)
add_subdirectory(realsense_t265)
endif()

## Executables to build
add_executable(${PROJECT_NAME} src/main.cpp)
if(UNIT_TESTS)
    add_executable(${PROJECT_NAME}_test ${UNIT_TEST_SOURCES})
endif()

if(USE_ROS)
    message("Using ROS for diagnostic data topics")
    add_definitions(-DUSE_ROS)
    find_package(roscpp REQUIRED)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
    set(ALL_ARWAIN_LIBRARIES ${ALL_ARWAIN_LIBRARIES} ${roscpp_LIBRARIES})
endif(USE_ROS)

if(USE_REALSENSE)
    set(ALL_ARWAIN_LIBRARIES ${ALL_ARWAIN_LIBRARIES} ${realsense2_LIBRARY} t265)
endif()

if(NUMCPP_NO_USE_BOOST)
    message("Compiling NumCpp without Boost")
    add_definitions(-DNUMCPP_NO_USE_BOOST)
else()
    # TODO This should do find_package(boost) plus includes and links.
endif()

## Set compiler flags
target_compile_options(${PROJECT_NAME}      PRIVATE     -g -Wno-psabi -Wall -Wshadow -Wextra)
if(UNIT_TESTS)
target_compile_options(${PROJECT_NAME}_test PRIVATE        -Wno-psabi -Wall -Wshadow -Wextra -ftest-coverage -fprofile-arcs)
endif()
target_link_libraries(${PROJECT_NAME}      ${ALL_ARWAIN_LIBRARIES} -lgcov                         i2c stdc++fs pthread boost_system tensorflow-lite flatbuffers)
if(UNIT_TESTS)
target_link_libraries(${PROJECT_NAME}_test ${ALL_ARWAIN_LIBRARIES} -lgcov --coverage GTest::gtest i2c stdc++fs pthread boost_system tensorflow-lite flatbuffers)
endif()

## Install configuration and related files to correct locations.
install(TARGETS ${PROJECT_NAME} DESTINATION /usr/bin)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/models/tflite/model_float32.tflite DESTINATION /etc)
