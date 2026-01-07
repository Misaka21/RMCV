# =============================================================================
# Detector 模块 CMakeLists.txt
# ==============================================================================

# 添加 TensorRT 查找模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/detector_trt)

# 自动检测 TensorRT
find_package(TensorRT QUIET)
if(TensorRT_FOUND)
    set(_TENSORRT_DEFAULT ON)
    message(STATUS "TensorRT ${TensorRT_VERSION} detected, auto-enabling")
else()
    set(_TENSORRT_DEFAULT OFF)
endif()

# 自动检测 OpenVINO
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
    set(_OPENVINO_DEFAULT ON)
    message(STATUS "OpenVINO detected, auto-enabling")
else()
    set(_OPENVINO_DEFAULT OFF)
endif()

# 检测器选项 (自动检测后可手动覆盖)
option(ENABLE_OPENVINO_DETECTOR "Enable OpenVINO YOLO detector" ${_OPENVINO_DEFAULT})
option(ENABLE_TENSORRT_DETECTOR "Enable TensorRT YOLO detector" ${_TENSORRT_DEFAULT})

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
    detector_factory.cpp
    detector_node.cpp
    detector_helpers.cpp
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

# 条件编译 YOLO 检测器 (宏定义是 PRIVATE，不传播到上层)
if(ENABLE_OPENVINO_DETECTOR)
    target_compile_definitions(detector_node PRIVATE ENABLE_OPENVINO_DETECTOR)
    target_link_libraries(detector_node PRIVATE detector_openvino)
endif()

if(ENABLE_TENSORRT_DETECTOR)
    target_compile_definitions(detector_node PRIVATE ENABLE_TENSORRT_DETECTOR)
    target_link_libraries(detector_node PRIVATE detector_tensorrt)
endif()

# 创建 detector 统一接口库
add_library(detector INTERFACE)

target_include_directories(detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 链接检测器
target_link_libraries(detector INTERFACE detector_traditional detector_node)
