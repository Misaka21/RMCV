# enemy_model 模块

add_subdirectory(motion)

add_library(enemy_model STATIC
    vehicle_model.cpp
    outpost_model.cpp
    base_model.cpp
    enemy_model_factory.cpp
)

target_include_directories(enemy_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(enemy_model PUBLIC
    motion
    enemy_state
    aimer_common
    Eigen3::Eigen
)
