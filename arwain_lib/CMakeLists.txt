cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

set(CMAKE_BUILD_TYPE Debug)

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
    target_link_libraries(${PROJECT_NAME} GTest::gtest)
    message("Building ${PROJECT_NAME} with test coverage")
    target_compile_options(${PROJECT_NAME} PRIVATE -ftest-coverage -fprofile-arcs -Wfatal-errors)
endif()

target_link_libraries(${PROJECT_NAME} stdc++fs)

include_directories(
    ../vector3
    ../lora
    ../arwain_thread
    ../orientation
    ../timers
    ../ips
    ../realsense_t265
)
