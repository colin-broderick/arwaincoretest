cmake_minimum_required(VERSION 3.16)
project(timers)
include_directories(include)
add_Library(${PROJECT_NAME} STATIC src/timers.cpp)
