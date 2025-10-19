# =============================================================================
# 检测可用的推理后端
# ==============================================================================

# 检测 OpenVINO
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
    set(HAS_OPENVINO TRUE)
    message(STATUS "Found OpenVINO: ${OpenVINO_VERSION}")
else()
    set(HAS_OPENVINO FALSE)
    message(STATUS "OpenVINO not found")
endif()

# 检测 TensorRT
find_package(TensorRT QUIET)
if(TensorRT_FOUND)
    set(HAS_TENSORRT TRUE)
    message(STATUS "Found TensorRT: ${TensorRT_VERSION}")
else()
    set(HAS_TENSORRT FALSE)
    message(STATUS "TensorRT not found")
endif()

# 设置用户可选择的推理后端
set(DETECTOR_BACKEND "AUTO" CACHE STRING "Detector backend to use")
set_property(CACHE DETECTOR_BACKEND PROPERTY STRINGS "AUTO" "OPENCV" "OPENVINO" "TENSORRT")

# 自动选择逻辑
if(DETECTOR_BACKEND STREQUAL "AUTO")
    if(HAS_OPENVINO)
        set(DETECTOR_BACKEND "OPENVINO")
        message(STATUS "Auto-selected OpenVINO backend")
    elseif(HAS_TENSORRT)
        set(DETECTOR_BACKEND "TENSORRT")
        message(STATUS "Auto-selected TensorRT backend")
    else()
        set(DETECTOR_BACKEND "OPENCV")
        message(STATUS "Auto-selected OpenCV backend (fallback)")
    endif()
endif()

# 验证用户选择的backend是否可用
if(DETECTOR_BACKEND STREQUAL "OPENVINO" AND NOT HAS_OPENVINO)
    message(FATAL_ERROR "OpenVINO backend selected but OpenVINO not found!")
elseif(DETECTOR_BACKEND STREQUAL "TENSORRT" AND NOT HAS_TENSORRT)
    message(FATAL_ERROR "TensorRT backend selected but TensorRT not found!")
endif()

message(STATUS "Using detector backend: ${DETECTOR_BACKEND}")

# =============================================================================
# 编译配置
# ==============================================================================

# 添加子目录并设置条件编译
if(DETECTOR_BACKEND STREQUAL "OPENVINO")
    add_subdirectory(detector_ov)
    target_link_libraries(detector INTERFACE detector_ov)
elseif(DETECTOR_BACKEND STREQUAL "TENSORRT")
    add_subdirectory(detector_trt)
    target_link_libraries(detector INTERFACE detector_trt)
else()  # OPENCV (默认/后备)
    add_subdirectory(detector_rv)
    target_link_libraries(detector INTERFACE detector_rv)
endif()

# 创建detector统一接口库
add_library(detector INTERFACE)
target_include_directories(detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 传递宏定义到编译器
if(DETECTOR_BACKEND STREQUAL "OPENVINO")
    target_compile_definitions(detector INTERFACE USE_OPENVINO)
elseif(DETECTOR_BACKEND STREQUAL "TENSORRT")
    target_compile_definitions(detector INTERFACE USE_TENSORRT)
else()
    target_compile_definitions(detector INTERFACE USE_OPENCV)
endif()

# 根据选择的backend链接对应的库
if(DETECTOR_BACKEND STREQUAL "OPENVINO")
    target_link_libraries(detector INTERFACE detector_ov)
elseif(DETECTOR_BACKEND STREQUAL "TENSORRT")
    target_link_libraries(detector INTERFACE detector_trt)
else()
    target_link_libraries(detector INTERFACE detector_rv)
endif()