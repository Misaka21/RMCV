# 火控模块 CMakeLists.txt

# Ceres 依赖 (弹道解算)
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

# 火控库
add_library(fire_control STATIC
    # 通用模块
    trajectory/trajectory_solver.cpp
    target_selector/target_selector.cpp

    # MPC 模式
    mpc/gimbal_planner.cpp
    mpc/mpc_controller.cpp

    # PID 模式
    pid/spin_aim.cpp
    pid/pid_controller.cpp

    # 调度器
    fire_controller.cpp
    fire_controller_node.cpp
)

target_include_directories(fire_control PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(fire_control PUBLIC
    tinympc
    Eigen3::Eigen
    umt
    debug_log
    ${CERES_LIBRARIES}
)
