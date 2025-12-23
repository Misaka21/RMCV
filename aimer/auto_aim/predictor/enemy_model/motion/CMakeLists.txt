# motion 模块 (运动模型)

add_library(motion STATIC
    armor_model.cpp
)

target_include_directories(motion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(motion PUBLIC
    Eigen3::Eigen
)
