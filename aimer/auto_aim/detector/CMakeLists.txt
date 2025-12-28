# =============================================================================
# Detector 模块 CMakeLists.txt
# ==============================================================================

# 检测器选项
option(ENABLE_OPENVINO_DETECTOR "Enable OpenVINO YOLO detector" OFF)
option(ENABLE_TENSORRT_DETECTOR "Enable TensorRT YOLO detector" OFF)

# 定义 common 目录路径供子模块使用
set(DETECTOR_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common)

# 添加传统检测器 (始终编译)
add_subdirectory(detector_rv)

# 可选: OpenVINO 检测器
if(ENABLE_OPENVINO_DETECTOR)
    add_subdirectory(detector_ov)
    message(STATUS "YOLO detector (OpenVINO): ENABLED")
else()
    message(STATUS "YOLO detector (OpenVINO): DISABLED")
endif()

# 可选: TensorRT 检测器
if(ENABLE_TENSORRT_DETECTOR)
    add_subdirectory(detector_trt)
    message(STATUS "YOLO detector (TensorRT): ENABLED")
else()
    message(STATUS "YOLO detector (TensorRT): DISABLED")
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

# 传递编译宏并链接 YOLO 检测器
if(ENABLE_OPENVINO_DETECTOR)
    target_compile_definitions(detector_node PUBLIC ENABLE_OPENVINO_DETECTOR)
    target_link_libraries(detector_node PUBLIC detector_openvino)
endif()

if(ENABLE_TENSORRT_DETECTOR)
    target_compile_definitions(detector_node PUBLIC ENABLE_TENSORRT_DETECTOR)
    target_link_libraries(detector_node PUBLIC detector_tensorrt)
endif()

# 创建 detector 统一接口库
add_library(detector INTERFACE)

target_include_directories(detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 链接检测器
target_link_libraries(detector INTERFACE detector_traditional detector_node)
