cmake_minimum_required(VERSION 3.16)

set(lib_name "recorder")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

add_library(${lib_name} STATIC
    recorder_node.cpp
)

target_include_directories(${lib_name} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${lib_name} PUBLIC
    ${OpenCV_LIBS}
    fmt::fmt
    plugin
    hardware
)
