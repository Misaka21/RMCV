# =============================================================================
# auto_buff/detector 模块 CMakeLists.txt
# =============================================================================

# 定义 common 目录路径供子模块使用
set(BUFF_DETECTOR_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common)

# 自动检测 TensorRT (复用 auto_aim 的 FindTensorRT.cmake)
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/aimer/auto_aim/detector/detector_trt)
find_package(TensorRT QUIET)
if(TensorRT_FOUND)
    set(_BUFF_TRT_DEFAULT ON)
    message(STATUS "TensorRT detected for buff detector, auto-enabling")
else()
    set(_BUFF_TRT_DEFAULT OFF)
endif()

# 自动检测 OpenVINO
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
    set(_BUFF_OV_DEFAULT ON)
    message(STATUS "OpenVINO detected for buff detector, auto-enabling")
else()
    set(_BUFF_OV_DEFAULT OFF)
endif()

# 检测器选项 (可手动覆盖)
option(ENABLE_BUFF_OPENVINO_DETECTOR "Enable OpenVINO YOLO buff detector" ${_BUFF_OV_DEFAULT})
option(ENABLE_BUFF_TENSORRT_DETECTOR "Enable TensorRT YOLO buff detector" ${_BUFF_TRT_DEFAULT})

# YOLOX 通用库 (preprocess + postprocess + rune_decoder)
add_library(buff_detector_core STATIC
    common/preprocess.cpp
    common/postprocess.cpp
    decoder/rune_decoder.cpp
)

target_include_directories(buff_detector_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/decoder
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_detector_core PUBLIC
    auto_buff_common
    aimer_common
    ${OpenCV_LIBS}
    plugin
)

# 传统检测器 (始终构建)
add_subdirectory(detector_traditional)
message(STATUS "Buff traditional detector: ENABLED")

# 可选: OpenVINO 检测器
if(ENABLE_BUFF_OPENVINO_DETECTOR)
    add_subdirectory(detector_ov)
    message(STATUS "Buff YOLO detector (OpenVINO): ENABLED")
else()
    message(STATUS "Buff YOLO detector (OpenVINO): DISABLED")
endif()

# 可选: TensorRT 检测器
if(ENABLE_BUFF_TENSORRT_DETECTOR)
    add_subdirectory(detector_trt)
    message(STATUS "Buff YOLO detector (TensorRT): ENABLED")
else()
    message(STATUS "Buff YOLO detector (TensorRT): DISABLED")
endif()

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
    buff_detector_core
    auto_buff_common
    aimer_common
    plugin
    hardware
    ${OpenCV_LIBS}
    fmt::fmt
    buff_detector_traditional
)

# 传统检测器 (始终可用)
target_compile_definitions(buff_detector_node PUBLIC ENABLE_BUFF_TRADITIONAL_DETECTOR)

# 条件编译 YOLO 检测器 (宏定义 PRIVATE，不传播)
if(ENABLE_BUFF_OPENVINO_DETECTOR)
    target_compile_definitions(buff_detector_node PRIVATE ENABLE_BUFF_OPENVINO_DETECTOR)
    target_link_libraries(buff_detector_node PRIVATE buff_detector_openvino)
endif()

if(ENABLE_BUFF_TENSORRT_DETECTOR)
    target_compile_definitions(buff_detector_node PRIVATE ENABLE_BUFF_TENSORRT_DETECTOR)
    target_link_libraries(buff_detector_node PRIVATE buff_detector_tensorrt)
endif()

# 创建 buff_detector 统一接口库
add_library(buff_detector INTERFACE)

target_include_directories(buff_detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

target_link_libraries(buff_detector INTERFACE
    buff_detector_node
)
