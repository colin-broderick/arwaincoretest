cmake_minimum_required(VERSION 3.16)
project(arwain_lib)

add_definitions(-DUSENCS2)
set(CMAKE_CXX_STANDARD 17)

add_library(arwain_lib 
    calibration.cpp
    stance.cpp
    velocity_prediction.cpp
    altimeter.cpp
    imu_reader.cpp
    uwb_reader.cpp
    std_output.cpp
    transmit_lora.cpp
    multi_imu.cpp
    arwain.cpp
    arwain_tests.cpp
    command_line.cpp
    configuration.cpp
)
target_link_libraries(arwain_lib stdc++fs)
# target_compile_options(arwain_lib PRIVATE std=c++17)
include_directories(
    ../vector3
    ../quaternion
    ../logger
    ../lora
    ../uubla/include
    ../input_parser
    ../arwain_thread
    ../bmp384
    ../iim42652
    ../orientation
    ../timers
    ../lis3mdl
    ../ips
)

# project(arwain_lib)

# ## Set include and library source directories
# include_directories(
#     /usr/local/include
#     lora
#     arwain_thread
#     orientation
#     src
#     iim42652
#     orientation_fusion
#     quaternion
#     vector3
#     logger
#     kalman
#     input_parser
#     timers
#     lis3mdl
#     minitrace
#     ips
#     bmp384
#     uubla/include
#     /opt/arwain/workspace/tensorflow_src
#     # realsense_t265
# )

# # TODO : Additional includes for tensorflow. These need to be tidied up.
# include_directories(
#     /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/session
#     /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/onnxruntime/usr/include/onnxruntime/core/providers/vsi_npu
#     /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite-vx-delegate/usr/include/tensorflow-lite-vx-delegate
#     /opt/yocto/imx-yocto-bsp/build/tmp/work/armv8a-poky-linux/flatbuffers/1.12.0-r0/image/usr/include/
#     /opt/yocto/imx-yocto-bsp/build/tmp/sysroots-components/armv8a/tensorflow-lite/usr/lib
# )

# option(NUMCPP_NO_USE_BOOST "Build NumCpp without Boost as a dependency" OFF)
# if(NUMCPP_NO_USE_BOOST)
#     add_compile_definitions(NUMCPP_NO_USE_BOOST)
#     # TODO This should do find_package(boost) plus includes and links in the else() block.
# endif()

# option(USEROS "Use ROS topics to publish diagnostic data" OFF)
# if(USEROS)
#     message("Using ROS for diagnostic data topics")
#     add_definitions(-DUSEROS)
#     find_package(roscpp REQUIRED)
#     target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${roscpp_INCLUDE_DIRS})
#     target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
# endif(USEROS)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# ## Locate source files
# set(PROJECT_CPP_FILES
#     calibration.cpp
#     stance.cpp
#     velocity_prediction.cpp
#     altimeter.cpp
#     imu_reader.cpp
#     uwb_reader.cpp
#     std_output.cpp
#     transmit_lora.cpp
#     multi_imu.cpp
#     arwain.cpp
#     arwain_tests.cpp
#     command_line.cpp
#     configuration.cpp
# )

# add_subdirectory(iim42652)
# add_subdirectory(orientation)
# add_subdirectory(vector3)
# add_subdirectory(lora)
# add_subdirectory(quaternion)
# add_subdirectory(logger)
# add_subdirectory(input_parser)
# add_subdirectory(kalman)
# add_subdirectory(timers)
# add_subdirectory(lis3mdl)
# add_subdirectory(minitrace)
# add_subdirectory(ips)
# add_subdirectory(bmp384)
# add_subdirectory(uubla)
# add_subdirectory(arwain_thread)

# add_library(${PROJECT_NAME} ${PROJECT_CPP_FILES})


# option(USENCS2 "Use the Neural Compute Stick for inference" OFF)
# if(USENCS2)
#     message("Using NCS2 for inference")
#     add_definitions(-DUSENCS2)
#     target_link_libraries(${PROJECT_NAME} zmq)
# else()
#     set(TENSORFLOW_SOURCE_DIR "/opt/arwain/workspace/tensorflow_src" CACHE PATH
#     "Directory that contains the TensorFlow project" )
#     if(NOT TENSORFLOW_SOURCE_DIR)
#     get_filename_component(TENSORFLOW_SOURCE_DIR
#         "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)
#     endif()

#     if(NOT EXISTS "tfdone")  
#         add_subdirectory(
#             "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
#             "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite" EXCLUDE_FROM_ALL
#         )
#         execute_process(COMMAND touch tfdone)
#     endif()

#     # FOr tensorflow
#     target_link_libraries(
#         ${PROJECT_NAME}
#         tensorflow-lite
#         flatbuffers
#     )
# endif()

# target_compile_options(${PROJECT_NAME} PRIVATE -g -Wno-psabi -Wall -Wshadow)
# target_link_libraries(${PROJECT_NAME} boost_system madgwick_filter efaroe_filter -pthread dl stdc++fs i2c iim42652_driver quaternion logger input_parser kalman timers lis3mdl minitrace ips bmp384 lora fusion_filter uubla vector3)
# target_link_libraries(
#     ${PROJECT_NAME}
#     boost_system
#     madgwick_filter
#     efaroe_filter
#     -pthread
#     dl
#     stdc++fs
#     i2c
#     iim42652_driver
#     quaternion
#     logger
#     input_parser
#     kalman
#     timers
#     lis3mdl
#     minitrace
#     ips
#     bmp384
#     lora
#     fusion_filter
#     uubla
#     arwain_thread
# )
