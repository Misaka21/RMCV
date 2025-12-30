# FindTensorRT.cmake
# 查找 TensorRT 库，支持 Jetson 和桌面平台
#
# 设置以下变量:
#   TensorRT_FOUND        - 是否找到 TensorRT
#   TensorRT_INCLUDE_DIRS - 头文件目录
#   TensorRT_LIBS         - 链接库列表
#   TensorRT_VERSION      - TensorRT 版本
#
# 用户可设置以下变量来指定搜索路径:
#   TensorRT_ROOT         - TensorRT 安装根目录
#   TENSORRT_DIR          - 同上 (兼容)

include(FindPackageHandleStandardArgs)

# 查找 CUDA (不 REQUIRED，找不到则 TensorRT 也找不到)
find_package(CUDA QUIET)
if(NOT CUDA_FOUND)
    set(TensorRT_FOUND FALSE)
    return()
endif()

# 确定搜索路径
set(_TensorRT_SEARCH_PATHS)

# 用户指定的路径优先
if(DEFINED TensorRT_ROOT)
    list(APPEND _TensorRT_SEARCH_PATHS ${TensorRT_ROOT})
endif()
if(DEFINED TENSORRT_DIR)
    list(APPEND _TensorRT_SEARCH_PATHS ${TENSORRT_DIR})
endif()
if(DEFINED ENV{TensorRT_ROOT})
    list(APPEND _TensorRT_SEARCH_PATHS $ENV{TensorRT_ROOT})
endif()
if(DEFINED ENV{TENSORRT_DIR})
    list(APPEND _TensorRT_SEARCH_PATHS $ENV{TENSORRT_DIR})
endif()

# 平台相关的默认路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # Jetson 平台
    list(APPEND _TensorRT_SEARCH_PATHS
        /usr/src/tensorrt
        /usr/local/tensorrt
        /usr
    )
    set(_TensorRT_LIB_SUFFIX "aarch64-linux-gnu")
else()
    # x86_64 桌面平台
    # 搜索 /usr/local/TensorRT-* 目录
    file(GLOB _TensorRT_LOCAL_DIRS "/usr/local/TensorRT-*")
    list(SORT _TensorRT_LOCAL_DIRS ORDER DESCENDING)  # 优先选择最新版本
    list(APPEND _TensorRT_SEARCH_PATHS
        ${_TensorRT_LOCAL_DIRS}
        /usr/local/tensorrt
        /usr
    )
    set(_TensorRT_LIB_SUFFIX "x86_64-linux-gnu")
endif()

# 添加 CUDA 路径
list(APPEND _TensorRT_SEARCH_PATHS ${CUDA_TOOLKIT_ROOT_DIR})

# 查找头文件
find_path(TensorRT_INCLUDE_DIR NvInfer.h
    HINTS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES include
)

# 查找 nvinfer 库
find_library(TensorRT_LIBRARY_INFER nvinfer
    HINTS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES
        lib
        lib64
        lib/${_TensorRT_LIB_SUFFIX}
        targets/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/lib
)

# 查找 nvinfer_plugin 库
find_library(TensorRT_LIBRARY_PLUGIN nvinfer_plugin
    HINTS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES
        lib
        lib64
        lib/${_TensorRT_LIB_SUFFIX}
        targets/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/lib
)

# 查找 nvonnxparser 库 (用于解析 ONNX)
find_library(TensorRT_LIBRARY_ONNXPARSER nvonnxparser
    HINTS ${_TensorRT_SEARCH_PATHS}
    PATH_SUFFIXES
        lib
        lib64
        lib/${_TensorRT_LIB_SUFFIX}
        targets/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/lib
)

# 获取版本号
if(TensorRT_INCLUDE_DIR)
    file(READ "${TensorRT_INCLUDE_DIR}/NvInferVersion.h" _NvInferVersion_H)
    string(REGEX MATCH "#define NV_TENSORRT_MAJOR ([0-9]+)" _ ${_NvInferVersion_H})
    set(_TRT_MAJOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "#define NV_TENSORRT_MINOR ([0-9]+)" _ ${_NvInferVersion_H})
    set(_TRT_MINOR ${CMAKE_MATCH_1})
    string(REGEX MATCH "#define NV_TENSORRT_PATCH ([0-9]+)" _ ${_NvInferVersion_H})
    set(_TRT_PATCH ${CMAKE_MATCH_1})
    set(TensorRT_VERSION "${_TRT_MAJOR}.${_TRT_MINOR}.${_TRT_PATCH}")
endif()

# 汇总 include 目录
set(TensorRT_INCLUDE_DIRS
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)

# 汇总库
set(TensorRT_LIBS
    ${TensorRT_LIBRARY_INFER}
    ${TensorRT_LIBRARY_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUDA_cudart_LIBRARY}
)

# 如果找到 nvonnxparser，添加到库列表
if(TensorRT_LIBRARY_ONNXPARSER)
    list(APPEND TensorRT_LIBS ${TensorRT_LIBRARY_ONNXPARSER})
endif()

# 处理查找结果
find_package_handle_standard_args(TensorRT
    REQUIRED_VARS
        TensorRT_INCLUDE_DIR
        TensorRT_LIBRARY_INFER
        TensorRT_LIBRARY_PLUGIN
    VERSION_VAR TensorRT_VERSION
)

# 打印找到的信息
if(TensorRT_FOUND)
    message(STATUS "Found TensorRT ${TensorRT_VERSION}")
    message(STATUS "  Include: ${TensorRT_INCLUDE_DIR}")
    message(STATUS "  Libraries: ${TensorRT_LIBS}")
endif()

mark_as_advanced(
    TensorRT_INCLUDE_DIR
    TensorRT_LIBRARY_INFER
    TensorRT_LIBRARY_PLUGIN
    TensorRT_LIBRARY_ONNXPARSER
)
