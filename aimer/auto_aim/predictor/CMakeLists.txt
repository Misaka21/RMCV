# =============================================================================
# Predictor 模块 CMakeLists.txt
# =============================================================================

# 添加子目录
add_subdirectory(enemy_state)
add_subdirectory(enemy_model)

# 创建 predictor 库
add_library(predictor STATIC
    enemy_predictor.cpp
    predictor_node.cpp
)

target_include_directories(predictor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(predictor PUBLIC
    enemy_state
    enemy_model
    aimer_common
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
)
