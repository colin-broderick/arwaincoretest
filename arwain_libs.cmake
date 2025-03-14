include(FetchContent)

function(provide_library url as_name tag)
    FetchContent_Declare(
        ${as_name}
        GIT_REPOSITORY ${url}
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        GIT_TAG ${tag})
    FetchContent_MakeAvailable(${as_name})
    include_directories(${${as_name}_SOURCE_DIR}/include)
endfunction()

FetchContent_Declare(
    simple-websocket-server
    SOURCE_DIR ${CMAKE_BINARY_DIR}/_deps/sws/simple-websocket-server
    GIT_REPOSITORY https://gitlab.com/eidheim/Simple-WebSocket-Server
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    GIT_TAG v2.0.2)
FetchContent_MakeAvailable(simple-websocket-server)
include_directories(${CMAKE_BINARY_DIR}/_deps/sws)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_SHALLOW TRUE
    GIT_TAG 0.8.0)
FetchContent_MakeAvailable(yaml-cpp)
include_directories(${yaml-cpp_SOURCE_DIR}/include)

if(BUILT_WITH_YOCTO)
else()
    FetchContent_Declare(
        tensorflow
        GIT_REPOSITORY https://github.com/tensorflow/tensorflow.git
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE
        GIT_TAG v2.14.0
        SOURCE_SUBDIR tensorflow/lite)
    FetchContent_MakeAvailable(tensorflow)
    include_directories(./build/_deps/tensorflow-src)
    include_directories(./build/flatbuffers/include)
endif()

provide_library("https://github.com/dpilger26/NumCpp.git"                 NumCpp      Version_2.12.1)
provide_library("https://github.com/colin-broderick/arwain_math"          arwain_math         master)
provide_library("https://github.com/colin-broderick/arwain_config"        arwain_config       master)
provide_library("https://github.com/colin-broderick/bmp384"               bmp384              master)
provide_library("https://github.com/colin-broderick/logger"               logger              master)
provide_library("https://github.com/colin-broderick/timers"               timers              master)
provide_library("https://github.com/colin-broderick/input_parser"         input_parser        master)
provide_library("https://github.com/colin-broderick/arwain_rfm95w"        arwain_rfm95w       master)
provide_library("https://github.com/colin-broderick/iim42652"             iim42652            master)
provide_library("https://github.com/colin-broderick/arwain_event"         arwain_event        master)
provide_library("https://github.com/colin-broderick/uublacpp-"            uublacpp            master)
provide_library("https://github.com/colin-broderick/lis3mdl"              lis3mdl             master)
provide_library("https://github.com/colin-broderick/arwain_orientation"   arwain_orientation  master)
provide_library("https://github.com/colin-broderick/arwain_icp"           arwain_icp          master)
provide_library("https://github.com/colin-broderick/arwain_websocket"     arwain_websocket    master)
