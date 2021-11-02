project(mlx90395)

add_library(${PROJECT_NAME} STATIC mlx90395.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
