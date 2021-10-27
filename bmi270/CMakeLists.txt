project(bmi270_driver)

set(BMI270_C_FILES bmi2_ois.c bmi2.c bmi270.c bmm150.c)

set_source_files_properties(${BMI270_C_FILES} PROPERTIES LANGUAGE CXX)

add_library(${PROJECT_NAME} STATIC bmi270.cpp)

target_link_libraries(${PROJECT_NAME} i2c)
