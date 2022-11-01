cmake_minimum_required(VERSION 3.10.2)

project(logger)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_executable(${PROJECT_NAME} logger.cpp logger_test.cpp)
    target_compile_options(${PROJECT_NAME} PRIVATE -std=c++17 -g -O0 -Wall -Werror -Wextra -Wpedantic)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC /home/cbroderick/googletest/googletest/include)
    target_link_directories(${PROJECT_NAME} PRIVATE /home/cbroderick/googletest/build/lib)
    target_link_libraries(${PROJECT_NAME} gtest pthread)
else()
    add_library(${PROJECT_NAME} logger.cpp)
endif()

