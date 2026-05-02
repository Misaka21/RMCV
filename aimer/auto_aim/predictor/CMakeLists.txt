# =============================================================================
# Predictor 模块 CMakeLists.txt
# =============================================================================

# 创建 predictor 库
add_library(predictor STATIC
    enemy_predictor.cpp
    predictor_node.cpp

    # observer/ (PnP 观测 + 跟踪)
    observer/armor_observer.cpp
    observer/armor_tracker.cpp

    # model/
    model/enemy_model_factory.cpp
    model/vehicle/vehicle_model.cpp
    model/vehicle/ekf/armor_ekf.cpp
    model/vehicle/ekf/sp_ekf.cpp
    model/outpost/outpost_model.cpp
    model/outpost/outpost_ekf.cpp
    model/base/base_model.cpp
)

target_include_directories(predictor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(predictor PUBLIC
    aimer_common
    plugin
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
)
