# 添加子目录
add_subdirectory(hik_cam)
add_subdirectory(serial)

# 创建硬件节点库
add_library(hardware STATIC
    hardware_node.cpp
)

target_include_directories(hardware PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接所有硬件子库
target_link_libraries(hardware PUBLIC
    hardware_camera
    hardware_serial
    ${OpenCV_LIBS}
    fmt::fmt
    plugin
)
