project(lis3mdl)

add_library(${PROJECT_NAME} STATIC lis3mdl.cpp calibration.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
