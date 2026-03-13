# detector_trt - TensorRT 能量机关 YOLOX 检测器模块

# 添加本地 CMake 模块路径 (包含 FindTensorRT.cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 查找 TensorRT (复用 auto_aim 的 FindTensorRT.cmake)
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/aimer/auto_aim/detector/detector_trt)
find_package(TensorRT REQUIRED)

# 自动检测 CUDA 编译器
if(NOT CMAKE_CUDA_COMPILER)
    set(_CUDA_SEARCH_PATHS
        /usr/local/cuda/bin
        /usr/local/cuda-12.0/bin
        /usr/local/cuda-11.8/bin
        /usr/local/cuda-11.7/bin
        /usr/local/cuda-11.4/bin
        /usr/local/cuda-10.2/bin
        /opt/cuda/bin
    )
    find_program(_NVCC_EXECUTABLE nvcc HINTS ${_CUDA_SEARCH_PATHS})
    if(_NVCC_EXECUTABLE)
        set(CMAKE_CUDA_COMPILER ${_NVCC_EXECUTABLE} CACHE FILEPATH "CUDA compiler")
    endif()
    unset(_NVCC_EXECUTABLE CACHE)
endif()

enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

if(NOT DEFINED RMCV_CUDA_ARCH)
    set(RMCV_CUDA_ARCH "72;75;86;87")
endif()

# 注意: 不需要 CUDA 预处理 kernel,
# YOLOX 模型不做 /255 归一化, 预处理在 CPU 完成
add_library(buff_detector_tensorrt STATIC
    tensorrt_buff_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/preprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/postprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../decoder/rune_decoder.cpp
)

set_target_properties(buff_detector_tensorrt PROPERTIES
    CUDA_STANDARD 14
    CUDA_STANDARD_REQUIRED ON
    CUDA_ARCHITECTURES "${RMCV_CUDA_ARCH}"
    INTERPROCEDURAL_OPTIMIZATION OFF
)

target_include_directories(buff_detector_tensorrt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${BUFF_DETECTOR_COMMON_DIR}
    ${PROJECT_SOURCE_DIR}
    ${TensorRT_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(buff_detector_tensorrt PUBLIC
    auto_buff_common
    aimer_common
    ${TensorRT_LIBS}
    ${OpenCV_LIBS}
    CUDA::cudart
    plugin
)
