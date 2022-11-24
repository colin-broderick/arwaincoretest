project(kalman)

add_library(${PROJECT_NAME} STATIC src/kalman.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
