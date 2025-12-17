# =============================================================================
# Detector 模块 CMakeLists.txt
# ==============================================================================

# 添加 traditional 检测器子目录
add_subdirectory(traditional)

# 创建 detector 统一接口库
add_library(detector INTERFACE)

target_include_directories(detector INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# 链接 traditional 检测器
target_link_libraries(detector INTERFACE detector_traditional)
