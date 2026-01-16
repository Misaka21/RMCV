# motion 模块 (运动模型)

add_library(motion STATIC
    armor_motion.cpp
    spin_motion.cpp
    lmtd_motion.cpp
    sp_motion.cpp
    outpost_motion.cpp
)

target_include_directories(motion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(motion PUBLIC
    aimer_common   # 包含YPD滤波器和Ceres
    Eigen3::Eigen
)
