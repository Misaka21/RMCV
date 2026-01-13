# 火控模块 CMakeLists.txt
# AutoAim 专用火控，依赖 fire_control 通用模块

# Ceres 依赖 (通过 fire_control_core)
find_package(Ceres REQUIRED)

# TinyMPC 库
add_library(tinympc STATIC
    mpc/tinympc/admm.cpp
    mpc/tinympc/tiny_api.cpp
)
target_include_directories(tinympc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mpc
)
target_link_libraries(tinympc PUBLIC
    Eigen3::Eigen
)

# AutoAim 火控库
add_library(autoaim_fire_control STATIC
    # AutoAim 专用模块
    target_selector/target_selector.cpp
    fire_decision.cpp

    # MPC 模式
    mpc/gimbal_planner.cpp

    # PID 模式
    pid/spin_aim.cpp

    # 主类
    fire_controller.cpp

    # 节点入口
    fire_control_node.cpp
)

target_include_directories(autoaim_fire_control PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(autoaim_fire_control PUBLIC
    fire_control_core      # 通用火控 (弹道解算)
    fire_control_interface # 接口定义
    tinympc
    Eigen3::Eigen
    plugin
)

# 保留旧名称以兼容
add_library(fire_control ALIAS autoaim_fire_control)
