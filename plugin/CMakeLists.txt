cmake_minimum_required(VERSION 3.16)
set(lib_name "plugin")

# 依赖已在根目录 find_package，这里直接使用

# Collect debug source
aux_source_directory(./debug debug_src)
aux_source_directory(./param param_src)
aux_source_directory(./plotter plotter_src)
aux_source_directory(./visualizer visualizer_src)

# Webview sources (仅在启用时编译)
if(ENABLE_WEBVIEW)
    aux_source_directory(./webview webview_src)
endif()

# Create plugin library
add_library(${lib_name} STATIC ${debug_src} ${param_src} ${plotter_src} ${visualizer_src} ${webview_src})

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

# 如果启用 ENABLE_WEBVIEW，添加 pybind11 依赖
if(ENABLE_WEBVIEW)
    target_include_directories(${lib_name} PUBLIC
            ${Python_INCLUDE_DIRS}
            ${pybind11_INCLUDE_DIRS}
    )
    target_link_libraries(${lib_name} PUBLIC
            pybind11::embed
            ${Python_LIBRARIES}
    )
endif()

# 添加 rmcv_bag 子模块 (录制和回放)
add_subdirectory(rmcv_bag)
