cmake_minimum_required(VERSION 3.16)
project(logger)
include_directories(include)
add_library(${PROJECT_NAME} src/logger.cpp)
