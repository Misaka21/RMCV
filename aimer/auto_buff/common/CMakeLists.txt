# auto_buff/common 模块

add_library(auto_buff_common INTERFACE)

target_include_directories(auto_buff_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(auto_buff_common INTERFACE
    aimer_common
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
