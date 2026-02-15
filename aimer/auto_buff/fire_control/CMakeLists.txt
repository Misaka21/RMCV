# auto_buff/fire_control 模块

add_library(buff_fire_control STATIC
    fire_controller.cpp
    fire_control_node.cpp
)

target_include_directories(buff_fire_control PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(buff_fire_control PUBLIC
    auto_buff_common
    aimer_common
    Eigen3::Eigen
    plugin
    fmt::fmt
)

