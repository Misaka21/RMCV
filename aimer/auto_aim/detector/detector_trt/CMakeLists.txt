# detector_trt - TensorRT 检测器模块

# 添加本地 CMake 模块路径
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 查找 TensorRT
find_package(TensorRT REQUIRED)

# 自动检测 CUDA 编译器 (解决 CLion/snap CMake 找不到 nvcc 的问题)
if(NOT CMAKE_CUDA_COMPILER)
    # 常见 CUDA 安装路径 (桌面 + Jetson)
    set(_CUDA_SEARCH_PATHS
        /usr/local/cuda/bin          # 通用符号链接
        /usr/local/cuda-12.0/bin     # 桌面 CUDA 12.x
        /usr/local/cuda-11.8/bin
        /usr/local/cuda-11.7/bin
        /usr/local/cuda-11.6/bin
        /usr/local/cuda-11.4/bin     # Jetson JetPack 5.x
        /usr/local/cuda-10.2/bin     # Jetson JetPack 4.x
        /opt/cuda/bin
    )
    find_program(_NVCC_EXECUTABLE nvcc HINTS ${_CUDA_SEARCH_PATHS})
    if(_NVCC_EXECUTABLE)
        set(CMAKE_CUDA_COMPILER ${_NVCC_EXECUTABLE} CACHE FILEPATH "CUDA compiler")
        message(STATUS "Auto-detected CUDA compiler: ${CMAKE_CUDA_COMPILER}")
    endif()
    unset(_NVCC_EXECUTABLE CACHE)
endif()

# 启用 CUDA
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# CUDA 编译标志 (避免继承不兼容的 C++ 标志)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use_fast_math")

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
