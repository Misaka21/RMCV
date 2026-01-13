# 目标决策模块 CMakeLists.txt
# AutoAim 专用目标决策，依赖 fire_control 通用模块

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

# AutoAim 目标决策库
add_library(autoaim_target_decision STATIC
    # 目标选择
    target_selector/target_selector.cpp

    # 开火判断
    fire_decision.cpp

    # MPC 模式
    mpc/gimbal_planner.cpp

    # PID 模式
    pid/spin_aim.cpp

    # 火控逻辑 (内部使用)
    fire_controller.cpp

    # 决策器入口 (实现 TargetDeciderInterface)
    autoaim_decider.cpp
)

target_include_directories(autoaim_target_decision PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(autoaim_target_decision PUBLIC
    fire_control_core      # 通用火控 (弹道解算)
    fire_control_interface # 接口定义
    tinympc
    Eigen3::Eigen
    plugin
)

# 保留旧名称以兼容
add_library(autoaim_fire_control ALIAS autoaim_target_decision)
