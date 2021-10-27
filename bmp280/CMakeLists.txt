project(bmp280_driver)

set_source_files_properties(bmp280/bmp280.c PROPERTIES LANGUAGE CXX)

add_library(${PROJECT_NAME} STATIC bmp280.cpp bmp280.c)
