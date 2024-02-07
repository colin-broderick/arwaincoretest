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

provide_library("https://bitbucket.org/arwain/arwain_math"          arwain_math         master)
provide_library("https://bitbucket.org/arwain/arwain_config"        arwain_config       master)
provide_library("https://bitbucket.org/arwain/bmp384"               bmp384              master)
provide_library("https://bitbucket.org/arwain/logger"               logger              master)
provide_library("https://bitbucket.org/arwain/timers"               timers              master)
provide_library("https://bitbucket.org/arwain/input_parser"         input_parser        master)
provide_library("https://bitbucket.org/arwain/arwain_rfm95w"        arwain_rfm95w       master)
provide_library("https://bitbucket.org/arwain/iim42652"             iim42652            master)
provide_library("https://bitbucket.org/arwain/arwain_event"         arwain_event        master)
provide_library("https://bitbucket.org/arwain/uublacpp"             uublacpp            temp17)
provide_library("https://bitbucket.org/arwain/lis3mdl"              lis3mdl             master)
provide_library("https://bitbucket.org/arwain/arwain_orientation"   arwain_orientation  master)
provide_library("https://bitbucket.org/arwain/arwain_icp"           arwain_icp          modern)
provide_library("https://bitbucket.org/arwain/arwain_websocket"     arwain_websocket    master)
