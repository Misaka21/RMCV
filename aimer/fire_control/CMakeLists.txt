# fire_control 独立模块
# 只提供弹道解算等通用功能

find_package(Ceres QUIET)

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
    target_include_directories(fire_control_core PUBLIC
        ${CERES_INCLUDE_DIRS}
    )
    target_link_libraries(fire_control_core PUBLIC
        fire_control_interface
        ${CERES_LIBRARIES}
        plugin  # runtime_param
    )
    message(STATUS "fire_control_core will be built (Ceres found)")
else()
    message(STATUS "fire_control_core skipped (Ceres not found)")
endif()
