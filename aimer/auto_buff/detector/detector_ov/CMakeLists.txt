# detector_ov - OpenVINO 能量机关检测器模块

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(buff_detector_openvino STATIC
    openvino_buff_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/preprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/postprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../decoder/buff_decoder.cpp
)

target_include_directories(buff_detector_openvino PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${BUFF_DETECTOR_COMMON_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_detector_openvino PUBLIC
    auto_buff_common
    aimer_common
    openvino::runtime
    ${OpenCV_LIBS}
    plugin
)
