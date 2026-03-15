cmake_minimum_required(VERSION 3.16)
set(lib_name "plugin")

# 依赖已在根目录 find_package，这里直接使用

# Collect source
aux_source_directory(./debug debug_src)
aux_source_directory(./param param_src)
aux_source_directory(./visualizer visualizer_src)

# Create plugin library
add_library(${lib_name} STATIC ${debug_src} ${param_src} ${visualizer_src})

target_include_directories(${lib_name} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# 基础链接库
target_link_libraries(${lib_name} PUBLIC
        ${OpenCV_LIBS}
        Eigen3::Eigen
        fmt::fmt
)

# 添加 rmcv_bag 子模块 (录制和回放)
add_subdirectory(rmcv_bag)

# 添加 Rerun 可视化子模块
if(ENABLE_RERUN)
    add_subdirectory(rerun)
    target_link_libraries(${lib_name} PUBLIC rerun_plugin)
endif()
