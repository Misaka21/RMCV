# detector_trt - TensorRT 检测器模块

# 添加本地 CMake 模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 查找 TensorRT
find_package(TensorRT REQUIRED)

# 创建库
add_library(detector_tensorrt STATIC
    tensorrt_detector.cpp
)

target_include_directories(detector_tensorrt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DETECTOR_COMMON_DIR}
    ${PROJECT_SOURCE_DIR}
    ${TensorRT_INCLUDE_DIRS}
)

target_link_libraries(detector_tensorrt PUBLIC
    ${TensorRT_LIBS}
    ${OpenCV_LIBS}
    plugin
)
