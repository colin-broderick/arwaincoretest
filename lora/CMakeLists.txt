project(lora)

add_library(${PROJECT_NAME} STATIC lora.cpp spi.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)
