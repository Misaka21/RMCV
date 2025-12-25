# 火控模块 CMakeLists.txt

# Ceres 依赖 (弹道解算)
find_package(Ceres REQUIRED)

# TinyMPC 库
add_library(tinympc STATIC
    planner/tinympc/admm.cpp
    planner/tinympc/tiny_api.cpp
)
target_include_directories(tinympc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/planner
)
target_link_libraries(tinympc PUBLIC
    Eigen3::Eigen
)

# 火控库
add_library(fire_control STATIC
    trajectory/trajectory_solver.cpp
    target_selector/target_selector.cpp
    planner/gimbal_planner.cpp
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
