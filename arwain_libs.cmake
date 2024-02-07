FetchContent_Declare(
    arwain_config
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_config
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(arwain_config)
include_directories(${arwain_config_SOURCE_DIR}/include)

FetchContent_Declare(
    bmp384
    GIT_REPOSITORY https://bitbucket.org/arwain/bmp384
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(bmp384)
include_directories(${bmp384_SOURCE_DIR}/include)

FetchContent_Declare(
    logger
    GIT_REPOSITORY https://bitbucket.org/arwain/logger
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(logger)
include_directories(${logger_SOURCE_DIR}/include)

FetchContent_Declare(
    timers
    GIT_REPOSITORY https://bitbucket.org/arwain/timers
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(timers)
include_directories(${timers_SOURCE_DIR}/include)

FetchContent_Declare(
    input_parser
    GIT_REPOSITORY https://bitbucket.org/arwain/input_parser
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(input_parser)
include_directories(${input_parser_SOURCE_DIR}/include)

FetchContent_Declare(
    arwain_rfm95w
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_rfm95w
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(arwain_rfm95w)
include_directories(${arwain_rfm95w_SOURCE_DIR}/include)

FetchContent_Declare(
    iim42652
    GIT_REPOSITORY https://bitbucket.org/arwain/iim42652
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(iim42652)
include_directories(${iim42652_SOURCE_DIR}/include)

FetchContent_Declare(
    arwain_event
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_event
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(arwain_event)
include_directories(${arwain_event_SOURCE_DIR}/include)

FetchContent_Declare(
    uublacpp
    GIT_REPOSITORY https://bitbucket.org/arwain/uublacpp
    GIT_SHALLOW TRUE
    GIT_TAG temp17
)
FetchContent_MakeAvailable(uublacpp)
include_directories(${uublacpp_SOURCE_DIR}/include)

FetchContent_Declare(
    lis3mdl
    GIT_REPOSITORY https://bitbucket.org/arwain/lis3mdl
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(lis3mdl)
include_directories(${lis3mdl_SOURCE_DIR}/include)

FetchContent_Declare(
    arwain_orientation
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_orientation
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(arwain_orientation)
include_directories(${arwain_orientation_SOURCE_DIR}/include)

FetchContent_Declare(
    arwain_icp
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_icp
    GIT_SHALLOW TRUE
    GIT_TAG modern
)
FetchContent_MakeAvailable(arwain_icp)
include_directories(${arwain_icp_SOURCE_DIR}/include)

FetchContent_Declare(
    arwain_websocket
    GIT_REPOSITORY https://bitbucket.org/arwain/arwain_websocket
    GIT_SHALLOW TRUE
    GIT_TAG master
)
FetchContent_MakeAvailable(arwain_websocket)
include_directories(${arwain_websocket_SOURCE_DIR}/include)
