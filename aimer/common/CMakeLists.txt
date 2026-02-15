# aimer/common 公共模块
#
# Note: Ceres is required (AdaptiveEkf uses ceres::Jet). We expect the top-level
# CMake to provide the Ceres target (either via system install or FetchContent).

add_library(aimer_common STATIC
    transformer/transformer.cpp
    trajectory/trajectory_solver.cpp
    trajectory/rk4_trajectory_solver.cpp
    # 头文件 (便于IDE识别)
    math/math.hpp
    filter/adaptive_ekf.hpp
    types.hpp
)

target_include_directories(aimer_common PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(aimer_common PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Ceres::ceres
    fmt::fmt
    plugin
)
