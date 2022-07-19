project(t265)

add_library(${PROJECT_NAME} STATIC t265.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow -lrealsense2)
