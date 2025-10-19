# 聚合所有硬件子库的接口库
add_library(hardware INTERFACE)

# 添加子目录
add_subdirectory(hik_cam)
# add_subdirectory(serial)  # 待添加串口库时取消注释

# 聚合所有硬件组件
target_link_libraries(hardware INTERFACE
    hardware_camera
    # hardware_serial  # 待添加串口库时取消注释
)
