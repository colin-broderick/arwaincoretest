cmake_minimum_required(VERSION 3.10.2)

project(arwain)

## Set include and library source directories
include_directories(lora orientation bmp280 src iim42652 bmi270 orientation_fusion quaternion vector3 logger kalman input_parser)
include_directories(/home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/package/usr/include/onnxruntime/core/session )
include_directories(/home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/package/usr/include/onnxruntime/core/providers/vsi_npu)

link_directories(lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)

## Locate source files
set(ARWAIN_CPP_FILES src/main.cpp src/indoor_positioning_wrapper.cpp src/stance.cpp src/utils.cpp src/velocity_prediction.cpp src/altimeter.cpp src/imu_reader.cpp src/std_output.cpp src/transmit_lora.cpp src/multi_imu.cpp)
set(CALIBRATE_CPP_FILES src/calibrate_imu.cpp)

add_subdirectory(iim42652)
add_subdirectory(bmi270)
add_subdirectory(bmp280)
add_subdirectory(orientation)
add_subdirectory(lora)
add_subdirectory(quaternion)
add_subdirectory(logger)
add_subdirectory(input_parser)

## Executables to build
add_executable(${PROJECT_NAME} ${ARWAIN_CPP_FILES})
add_executable(${PROJECT_NAME}_calibration ${CALIBRATE_CPP_FILES})
add_executable(madgwick_standalone orientation/madgwick.cpp orientation/madgwick_standalone.cpp)
add_executable(onnx_test src/onnx_test.cpp)
add_executable(orientation_fusion orientation_fusion/fusion.cpp orientation/madgwick.cpp)

## Force certain C files to be compiled as C++

## Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
target_compile_options(${PROJECT_NAME}_calibration PRIVATE -Wno-psabi -Wall -Wshadow)
target_compile_options(madgwick_standalone PRIVATE -Wno-psabi -Wall)
target_compile_options(orientation_fusion PRIVATE -Wno-psabi -Wall)

## Link libraries
target_link_libraries(${PROJECT_NAME} madgwick_filter efaroe_filter -pthread dl stdc++fs i2c zmq iim42652_driver bmi270_driver bmp280_driver quaternion logger input_parser)
target_link_libraries(${PROJECT_NAME}_calibration -li2c)
target_link_libraries(orientation_fusion -li2c)
target_link_libraries(onnx_test -lonnxruntime)

install(TARGETS onnx_test DESTINATION bin)

## Install configuration and related files to correct locations.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/arwain.conf DESTINATION /etc)

## TEMPORARY FOR TESTING : REMOVE THIS IN FINAL BUILD
install(FILES /home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/git/nodejs/test/testdata/squeezenet.onnx DESTINATION bin)
