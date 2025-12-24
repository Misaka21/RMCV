# RMCVbag - 录制和回放模块
add_library(rmcv_bag STATIC
    recorder_node.cpp
    playback_node.cpp
)

target_include_directories(rmcv_bag PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(rmcv_bag PUBLIC
    ${OpenCV_LIBS}
    fmt::fmt
    plugin
    hardware
)
