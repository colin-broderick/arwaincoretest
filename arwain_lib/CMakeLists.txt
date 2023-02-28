cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

add_definitions(-DUSE_NCS2)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM SRC_FILES ${CMAKE_SOURCE_DIR}/arwain_lib/src/realsense.cpp)

if(USE_UUBLA)
set(SRC_FILES ${SRC_FILES} src/uwb_reader.cpp)
else()
list(REMOVE_ITEM SRC_FILES ${CMAKE_SOURCE_DIR}/arwain_lib/src/uwb_reader.cpp)
endif()
if(USE_REALSENSE)
set(SRC_FILES ${SRC_FILES} src/realsense.cpp)
endif()

add_library(arwain_lib ${SRC_FILES})

if(UNIT_TESTS)
    message("Building ${PROJECT_NAME} with test coverage")
    target_compile_options(${PROJECT_NAME} PRIVATE -ftest-coverage -fprofile-arcs)
endif()

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
