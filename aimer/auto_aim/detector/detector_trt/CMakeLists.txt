# detector_trt - TensorRT 检测器模块

# 添加本地 CMake 模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 查找 TensorRT
find_package(TensorRT REQUIRED)

# 启用 CUDA
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# CUDA 编译标志 (避免继承不兼容的 C++ 标志)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --use_fast_math")

# CUDA 架构设置
# 可通过 cmake -DRMCV_CUDA_ARCH="72" 指定单一架构加速编译
# 默认编译所有常用架构: Xavier(72), Turing(75), Ampere(86,87)
if(NOT DEFINED RMCV_CUDA_ARCH)
    set(RMCV_CUDA_ARCH "72;75;86;87")
endif()
message(STATUS "CUDA architectures: ${RMCV_CUDA_ARCH}")

# 创建库 (包含 CUDA 源文件)
add_library(detector_tensorrt STATIC
    tensorrt_detector.cpp
    cuda_preprocess.cu
)

# CUDA 编译选项
set_target_properties(detector_tensorrt PROPERTIES
    CUDA_STANDARD 14
    CUDA_STANDARD_REQUIRED ON
    CUDA_ARCHITECTURES "${RMCV_CUDA_ARCH}"
    INTERPROCEDURAL_OPTIMIZATION OFF  # 禁用 LTO 避免与 CUDA 冲突
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
