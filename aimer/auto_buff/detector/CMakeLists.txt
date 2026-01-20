# =============================================================================
# auto_buff/detector 模块 CMakeLists.txt
# =============================================================================

# 定义 common 目录路径供子模块使用
set(BUFF_DETECTOR_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common)

# 传统检测器库
add_library(buff_detector_traditional STATIC
    traditional/traditional_detector.cpp
)

target_include_directories(buff_detector_traditional PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/traditional
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_detector_traditional PUBLIC
    auto_buff_common
    plugin
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
)

# 检测器节点库
add_library(buff_detector_node STATIC
    detector_factory.cpp
    detector_node.cpp
)

target_include_directories(buff_detector_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_detector_node PUBLIC
    buff_detector_traditional
    auto_buff_common
    aimer_common
    plugin
    hardware
    ${OpenCV_LIBS}
    fmt::fmt
)

# 创建 buff_detector 统一接口库
add_library(buff_detector INTERFACE)

target_include_directories(buff_detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

target_link_libraries(buff_detector INTERFACE
    buff_detector_traditional
    buff_detector_node
)
