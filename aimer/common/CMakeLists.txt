# aimer/common 公共模块

add_library(aimer_common STATIC
    transformer/transformer.cpp
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
