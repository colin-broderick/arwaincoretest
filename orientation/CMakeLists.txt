cmake_minimum_required(VERSION 3.16)
project(orientation)

option(UNIT_TESTS "Make private class methods public to enable testing" OFF)

if(UNIT_TESTS)
    message("Building with all public class methods for test")
    add_definitions(-DTESTABLE=public)
else()
    add_definitions(-DTESTABLE=private)
endif()

include_directories(include)
add_library(madgwick_filter STATIC src/madgwick.cpp)
add_library(efaroe_filter STATIC src/efaroe.cpp)
# add_library(android_fusion STATIC android_fusion.cpp)
set(FUSION_C_FILES src/new_madgwick_FusionAhrs.cpp src/new_madgwick_FusionBias.cpp src/new_madgwick_FusionCompass.cpp)
set_source_files_properties(${FUSION_C_FILES} PROPERTIES LANGUAGE CXX)
add_library(fusion_filter STATIC ${FUSION_C_FILES})
