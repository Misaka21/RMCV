# fire_control 独立模块
# 提供与具体目标类型无关的火控功能

# 接口库 (header-only)
add_library(fire_control_interface INTERFACE)
target_include_directories(fire_control_interface INTERFACE
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(fire_control_interface INTERFACE
    Eigen3::Eigen
)

# 核心库 (弹道解算) - 需要 Ceres
if(Ceres_FOUND)
    add_library(fire_control_core
        core/trajectory/trajectory_solver.cpp
        core/trajectory/rk4_trajectory_solver.cpp
    )
    target_link_libraries(fire_control_core PUBLIC
        fire_control_interface
        Ceres::ceres
        plugin  # runtime_param
    )
    message(STATUS "fire_control_core will be built (Ceres found)")
else()
    message(STATUS "fire_control_core skipped (Ceres not found)")
endif()

