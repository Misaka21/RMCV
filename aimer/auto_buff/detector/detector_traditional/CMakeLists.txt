# 传统能量机关检测器 (纯 CV 管线)
add_library(buff_detector_traditional STATIC
    traditional_buff_detector.cpp
)

target_include_directories(buff_detector_traditional PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_detector_traditional PUBLIC
    buff_detector_core
    auto_buff_common
    ${OpenCV_LIBS}
    plugin
)
