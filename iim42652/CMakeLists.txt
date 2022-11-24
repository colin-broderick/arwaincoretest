project(iim42652_driver)

add_library(${PROJECT_NAME} STATIC src/IMU_IIM42652_driver.cpp)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi -Wall -Wshadow)

target_link_libraries(${PROJECT_NAME} i2c)
