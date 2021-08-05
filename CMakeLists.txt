cmake_minimum_required(VERSION 3.10.2)

project(arwain)

## Set include and library source directories
include_directories(lora_driver orientation bmp280 src)
include_directories(/home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/package/usr/include/onnxruntime/core/session)
include_directories(/home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/package/usr/include/onnxruntime/core/providers/vsi_npu)

link_directories(lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)

## Locate source files
set(BMI_C_FILES src/bmi2_ois.c src/bmi2.c src/bmi270.c src/bmm150.c)
set(BMP280_C_FILES bmp280/bmp280.c)
set(ARWAIN_CPP_FILES src/bmp280.cpp src/main.cpp src/imu_utils.cpp src/indoor_positioning_wrapper.cpp src/input_parser.cpp src/math_util.cpp src/quaternions.cpp src/stance.cpp src/utils.cpp src/velocity_prediction.cpp src/logger.cpp)
set(CALIBRATE_CPP_FILES src/calibrate_bmi270.cpp src/utils.cpp src/input_parser.cpp src/imu_utils.cpp)

## Static libraries to build
# add_library(${PROJECT_NAME}_lorapi STATIC lora_driver/lora.cpp lora_driver/packet.cpp)
add_library(${PROJECT_NAME}_madgwick STATIC orientation/madgwick.cpp)
add_library(${PROJECT_NAME}_efaroe STATIC orientation/efaroe.cpp)

## Executables to build
add_executable(${PROJECT_NAME} ${ARWAIN_CPP_FILES} ${BMI_C_FILES} ${BMP280_C_FILES})
add_executable(${PROJECT_NAME}_calibration ${CALIBRATE_CPP_FILES} ${BMI_C_FILES})
# add_executable(lora_rx_test lora_driver/lora_rx_test.cpp)
# add_executable(lora_tx_test lora_driver/lora_tx_test.cpp)
add_executable(madgwick_standalone orientation/madgwick.cpp src/input_parser.cpp orientation/madgwick_standalone.cpp)
add_executable(efaroe_standalone orientation/efaroe.cpp src/input_parser.cpp orientation/efaroe_standalone.cpp src/quaternions.cpp src/math_util.cpp)
add_executable(onnx_test src/onnx_test.cpp)

## Force certain C files to be compiled as C++
set_source_files_properties(${BMI_C_FILES} PROPERTIES LANGUAGE CXX)
set_source_files_properties(${BMP280_C_FILES} PROPERTIES LANGUAGE CXX)

## Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
target_compile_options(${PROJECT_NAME}_calibration PRIVATE -Wno-psabi -Wall -Wshadow)
target_compile_options(madgwick_standalone PRIVATE -Wno-psabi -Wall)
target_compile_options(efaroe_standalone PRIVATE -Wno-psabi -Wall)

## Link libraries
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_madgwick ${PROJECT_NAME}_efaroe -pthread -ldl -lstdc++fs -li2c)
target_link_libraries(${PROJECT_NAME}_calibration -li2c)
target_link_libraries(onnx_test -lonnxruntime)
# target_link_libraries(lora_rx_test arwain_lorapi -lwiringPi)
# target_link_libraries(lora_tx_test arwain_lorapi -lwiringPi)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_calibration madgwick_standalone efaroe_standalone DESTINATION bin)
install(TARGETS onnx_test DESTINATION bin)

## Install configuration and related files to correct locations.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/arwain.conf DESTINATION /etc)

## TEMPORARY FOR TESTING : REMOVE THIS IN FINAL BUILD
install(FILES /home/colin/imx-yocto-bsp/build-full2/tmp/work/cortexa53-crypto-poky-linux/onnxruntime/1.5.3-r0/git/nodejs/test/testdata/squeezenet.onnx DESTINATION bin)
