project(t265)

add_library(${PROJECT_NAME} STATIC t265.cpp)

find_package(realsense2 REQUIRED)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
