# =============================================================================
# Detector 模块 CMakeLists.txt
# ==============================================================================

# 检测器选项
option(ENABLE_YOLO_DETECTOR "Enable YOLO detector (requires OpenVINO)" OFF)

# 添加传统检测器 (始终编译)
add_subdirectory(detector_rv)

# 可选: YOLO检测器
if(ENABLE_YOLO_DETECTOR)
    # add_subdirectory(detector_yolo)
    message(STATUS "YOLO detector: ENABLED")
else()
    message(STATUS "YOLO detector: DISABLED")
endif()

# 创建 detector_node 库
add_library(detector_node STATIC
    detector_node.cpp
)

target_include_directories(detector_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(detector_node PUBLIC
    detector_traditional
    plugin
    hardware
    aimer_common
    ${OpenCV_LIBS}
    fmt::fmt
)

# 传递编译宏
if(ENABLE_YOLO_DETECTOR)
    target_compile_definitions(detector_node PUBLIC ENABLE_YOLO_DETECTOR)
    # target_link_libraries(detector_node PUBLIC detector_yolo)
endif()

# 创建 detector 统一接口库
add_library(detector INTERFACE)

target_include_directories(detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 链接检测器
target_link_libraries(detector INTERFACE detector_traditional detector_node)
