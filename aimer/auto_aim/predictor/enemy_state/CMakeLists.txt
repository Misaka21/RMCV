# enemy_state 模块

add_library(enemy_state STATIC
    enemy_state.cpp
    armor_observer.cpp
    armor_identifier.cpp
)

target_include_directories(enemy_state PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(enemy_state PUBLIC
    aimer_common
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
