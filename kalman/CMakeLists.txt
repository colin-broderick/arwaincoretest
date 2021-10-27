project(kalman)

add_library(${PROJECT_NAME} STATIC kalman.cpp kalman_example.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
