cmake_minimum_required(VERSION 3.13.4)

project(arwain)

include_directories(lora_driver)
link_directories(lib)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)

set(BMI_C_FILES src/bmi2_ois.c src/bmi2.c src/bmi270.c src/bmm150.c)
set(ARWAIN_CPP_FILES src/main.cpp src/bin_log.cpp src/efaroe.cpp src/imu_utils.cpp src/indoor_positioning_wrapper.cpp src/input_parser.cpp src/madgwick.cpp src/math_util.cpp src/quaternions.cpp src/stance.cpp src/utils.cpp src/half.cpp)
set(CALIBRATE_CPP_FILES src/calibrate_bmi270.cpp src/utils.cpp src/input_parser.cpp src/imu_utils.cpp)

add_library(lorapi STATIC lora_driver/lora.cpp lora_driver/packet.cpp)
add_executable(${PROJECT_NAME} ${ARWAIN_CPP_FILES} ${BMI_C_FILES})
add_executable(${PROJECT_NAME}_calibration ${CALIBRATE_CPP_FILES} ${BMI_C_FILES})

# This line forces the use of C++ compiler for C files, necessary for the BMI files.
set_source_files_properties(${BMI_C_FILES} PROPERTIES LANGUAGE CXX)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
target_compile_options(${PROJECT_NAME}_calibration PRIVATE -Wno-psabi -Wall -Wshadow)
target_link_libraries(${PROJECT_NAME} lorapi -pthread -ldl -lstdc++fs -li2c -larwain_torch -lzmq -lwiringPi)
target_link_libraries(${PROJECT_NAME}_calibration -li2c)
