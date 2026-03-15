cmake_minimum_required(VERSION 3.16)

add_library(rerun_plugin STATIC rmcv_rerun.cpp)

target_include_directories(rerun_plugin PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}          # plugin/param/static_config.hpp 等 header-only 头文件
    ${PROJECT_SOURCE_DIR}/umt      # umt/BasicObjManager.hpp
)

# 不链接 plugin (避免循环依赖)，plugin 中用到的头文件都是 header-only
target_link_libraries(rerun_plugin PUBLIC
    rerun_sdk
    fmt::fmt
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
