cmake_minimum_required(VERSION 3.16)
project(arwain_thread)
include_directories(include)
add_library(${PROJECT_NAME} src/arwain_thread.cpp)
