# 火控模块 CMakeLists.txt
# AutoAim 专用火控

# AutoAim 火控库
add_library(autoaim_fire_control STATIC
    # 目标选择层
    selection/target_selector.cpp
    selection/armor_aim.cpp

    # 决策层
    decision/fire_decision.cpp

    # 主控制器
    fire_controller.cpp
    fire_control_node.cpp
)

target_include_directories(autoaim_fire_control PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(autoaim_fire_control PUBLIC
    aimer_common           # 弹道解算等通用功能
    Eigen3::Eigen
    plugin
)

# 保留旧名称以兼容
add_library(fire_control ALIAS autoaim_fire_control)
