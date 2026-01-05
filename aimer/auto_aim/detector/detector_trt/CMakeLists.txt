# detector_trt - TensorRT 检测器模块

# 添加本地 CMake 模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 查找 TensorRT
find_package(TensorRT REQUIRED)

# 启用 CUDA
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# 创建库 (包含 CUDA 源文件)
add_library(detector_tensorrt STATIC
    tensorrt_detector.cpp
    cuda_preprocess.cu
)

# CUDA 编译选项
set_target_properties(detector_tensorrt PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "72;75;86;87"  # Xavier(72), Turing(75), Ampere(86,87)
)

target_include_directories(detector_tensorrt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DETECTOR_COMMON_DIR}
    ${PROJECT_SOURCE_DIR}
    ${TensorRT_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(detector_tensorrt PUBLIC
    ${TensorRT_LIBS}
    ${OpenCV_LIBS}
    CUDA::cudart
    plugin
)
