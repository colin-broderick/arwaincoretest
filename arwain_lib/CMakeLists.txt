cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenSSL REQUIRED)

include_directories(include)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM SRC_FILES ${CMAKE_SOURCE_DIR}/arwain_lib/src/realsense.cpp)

if(USE_REALSENSE)
set(SRC_FILES ${SRC_FILES} src/realsense.cpp)
endif()

add_library(arwain_lib ${SRC_FILES})

include(tests.cmake)

target_link_libraries(${PROJECT_NAME} stdc++fs arwain_websocket OpenSSL::SSL OpenSSL::Crypto )

include_directories(
    ../ips
    ../realsense_t265
)
