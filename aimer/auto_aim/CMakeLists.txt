# auto_aim 模块

add_subdirectory(detector)
add_subdirectory(predictor)
# 重命名: fire_control -> target_decision
# add_subdirectory(fire_control)  # 已弃用，保留目录待删除
add_subdirectory(target_decision)
