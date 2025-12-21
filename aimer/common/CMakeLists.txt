# aimer/common 公共模块

add_library(aimer_common STATIC
    transformer/transformer.cpp
    # 头文件 (便于IDE识别)
    math/math.hpp
    types.hpp
)

target_include_directories(aimer_common PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(aimer_common PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    plugin
)
