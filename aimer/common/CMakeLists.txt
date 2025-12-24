# aimer/common 公共模块

# 查找Ceres (自动微分EKF需要)
find_package(Ceres REQUIRED)

add_library(aimer_common STATIC
    transformer/transformer.cpp
    # 头文件 (便于IDE识别)
    math/math.hpp
    filter/adaptive_ekf.hpp
    types.hpp
)

target_include_directories(aimer_common PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(aimer_common PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${CERES_LIBRARIES}
    fmt::fmt
    plugin
)
