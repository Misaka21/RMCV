# detector_ov - OpenVINO 检测器模块

# 查找 OpenVINO
find_package(OpenVINO REQUIRED)

# 创建库
add_library(detector_openvino STATIC
    openvino_detector.cpp
)

target_include_directories(detector_openvino PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DETECTOR_COMMON_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(detector_openvino PUBLIC
    openvino::runtime
    ${OpenCV_LIBS}
    plugin
)
