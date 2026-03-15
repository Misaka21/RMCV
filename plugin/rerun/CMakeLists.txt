cmake_minimum_required(VERSION 3.16)

add_library(rerun_plugin STATIC rmcv_rerun.cpp)

target_include_directories(rerun_plugin PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(rerun_plugin PUBLIC
    rerun_sdk
    fmt::fmt
    Eigen3::Eigen
    ${OpenCV_LIBS}
    plugin
)
