# auto_buff/predictor 模块

find_package(Ceres REQUIRED)

add_library(buff_predictor STATIC
    buff_predictor.cpp
    predictor_node.cpp
    slot_debouncer.cpp
    direction_estimator.cpp
    mode_manager.cpp
    models/const_model.cpp
    models/small_ekf_model.cpp
    models/large_lsm_model.cpp
)

target_include_directories(buff_predictor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(buff_predictor PUBLIC
    auto_buff_common
    aimer_common
    plugin
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    ${CERES_LIBRARIES}
)
